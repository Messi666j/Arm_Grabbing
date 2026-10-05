import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import Header

from so_arm101_model_based.kinematics import SOArm101Kinematics, rpy_to_rotation_matrix
from so_arm101_model_based.trajectory import TrajectoryPlanner, quintic_polynomial_step, interpolate_joint_space

def rpy_to_quaternion(roll, pitch, yaw):
    """Convert Roll-Pitch-Yaw to Quaternion [x, y, z, w]."""
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class SOArm101SimulationNode(Node):

    def __init__(self):
        super().__init__('so_arm101_sim_node')

        self.get_logger().info("==================================================")
        self.get_logger().info("🤖 SO-ARM101 Traditional Control Simulation Node...")
        self.get_logger().info("==================================================")

        # 1. 实例化正逆运动学求解器
        self.solver = SOArm101Kinematics()
        self.planner = TrajectoryPlanner(self.solver)

        # 2. 创建 ROS 2 发布者
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.marker_pub = self.create_publisher(Marker, '/target_marker', 10)
        self.path_pub = self.create_publisher(Marker, '/ee_path_marker', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)

        # 3. 真实 OnShape CAD URDF 关节名称
        self.joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']

        # 当前关节角状态
        self.current_q = np.zeros(6)

        # 轨迹插值队列
        self.trajectory_queue = []
        self.path_points_history = []

        # 规划模式: FIGURE8 -> PICK_PLACE -> CIRCLE -> FIGURE8 ...
        self.mode = "FIGURE8"
        self.step_counter = 0

        # 生成初始 8 字形轨迹
        self.prepare_figure8_trajectory()

        # 4. 定时器 (50Hz 控制循环, dt = 0.02s)
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        fk_init, _, _ = self.solver.forward_kinematics(self.current_q)
        self.get_logger().info("✅ Initial FK / IK Solver Ready!")
        self.get_logger().info(f"📍 Base Link to End-Effector ({self.solver.model.frames[self.solver.tool_frame_id].name}) Pos: {fk_init}")
        self.get_logger().info("🚀 Simulation Controller Started (Publishing to /joint_states @ 50Hz)")

    def prepare_figure8_trajectory(self):
        """生成 3D 8字形 (Bernoulli Lemniscate 双扭线) 轨迹"""
        self.get_logger().info("♾️ Mode [FIGURE8]: Generating 3D Cartesian Task Space Figure-8 Trajectory...")
        center_pos = np.array([0.26, 0.0, 0.22])
        duration = 10.0

        joint_traj, cartesian_points, target_rpys, ik_success_list = self.planner.generate_figure8_trajectory(
            center_pos=center_pos, rx=0.06, ry=0.08, rz=0.02, duration=duration, dt=0.02, q_init=self.current_q
        )

        self.trajectory_queue = []
        for i in range(len(joint_traj)):
            self.trajectory_queue.append({
                'q': joint_traj[i],
                'target_pos': cartesian_points[i],
                'target_rpy': target_rpys[i],
                'ik_success': ik_success_list[i],
                'mode_name': 'Task-Space Figure-8 (8字轨迹)'
            })

        self.path_points_history = [Point(x=p[0], y=p[1], z=p[2]) for p in cartesian_points]

    def prepare_circle_trajectory(self):
        """生成空间画圆轨迹模式"""
        self.get_logger().info("🎯 Mode [CIRCLE]: Generating 3D Cartesian Task Space Circle Trajectory...")
        center_pos = np.array([0.28, 0.0])
        radius = 0.06
        height = 0.20
        duration = 8.0

        joint_traj, cartesian_points, target_rpys, ik_success_list = self.planner.generate_circle_trajectory(
            center_pos=center_pos, radius=radius, height=height, duration=duration, dt=0.02, q_init=self.current_q
        )

        self.trajectory_queue = []
        for i in range(len(joint_traj)):
            self.trajectory_queue.append({
                'q': joint_traj[i],
                'target_pos': cartesian_points[i],
                'target_rpy': target_rpys[i],
                'ik_success': ik_success_list[i],
                'mode_name': 'Task-Space Circle (画圆轨迹)'
            })

        self.path_points_history = [Point(x=p[0], y=p[1], z=p[2]) for p in cartesian_points]

    def prepare_pick_place_trajectory(self):
        """生成 Pick and Place 多阶段连续规划"""
        self.get_logger().info("📦 Mode [PICK_PLACE]: Generating Pick & Place Waypoints Trajectory...")
        waypoints = self.planner.generate_pick_and_place_waypoints()

        self.trajectory_queue = []
        q_curr = np.copy(self.current_q)
        all_cart_points = []

        for wp in waypoints:
            target_pos, target_rpy, gripper_val, phase_name, duration = wp

            q_target_sol, success, _, _ = self.solver.inverse_kinematics_dls(
                target_pos, q_init=q_curr, damping=1e-3, position_only=True
            )
            q_target_sol[5] = gripper_val

            q_sub_traj, _ = interpolate_joint_space(q_curr, q_target_sol, duration, dt=0.02)

            for q_step in q_sub_traj:
                pos_fk, _, _ = self.solver.forward_kinematics(q_step)
                all_cart_points.append(Point(x=pos_fk[0], y=pos_fk[1], z=pos_fk[2]))
                self.trajectory_queue.append({
                    'q': q_step,
                    'target_pos': target_pos,
                    'target_rpy': target_rpy,
                    'ik_success': success,
                    'mode_name': f'Pick&Place: {phase_name}'
                })

            q_curr = np.copy(q_target_sol)

        self.path_points_history = all_cart_points

    def control_loop(self):
        """50Hz 核心控制循环"""
        if not self.trajectory_queue:
            # 模式状态机切换: FIGURE8 -> PICK_PLACE -> CIRCLE -> FIGURE8
            if self.mode == "FIGURE8":
                self.mode = "PICK_PLACE"
                self.prepare_pick_place_trajectory()
            elif self.mode == "PICK_PLACE":
                self.mode = "CIRCLE"
                self.prepare_circle_trajectory()
            else:
                self.mode = "FIGURE8"
                self.prepare_figure8_trajectory()
            return

        item = self.trajectory_queue.pop(0)
        q_cmd = item['q']
        target_pos = item['target_pos']
        target_rpy = item['target_rpy']
        ik_success = item['ik_success']
        mode_name = item['mode_name']

        self.current_q = np.copy(q_cmd)

        # 1. 验算正运动学 (FK Validation)
        fk_pos, fk_R, fk_rpy = self.solver.forward_kinematics(self.current_q)
        pos_error = np.linalg.norm(target_pos - fk_pos)
        manipulability = self.solver.compute_manipulability(self.current_q)

        # 2. 发布 ROS 2 JointState
        joint_msg = JointState()
        joint_msg.header = Header()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = self.current_q.tolist()
        self.joint_pub.publish(joint_msg)

        # 3. 发布目标点 Marker (红色闪烁球体)
        target_marker = Marker()
        target_marker.header.frame_id = "world"
        target_marker.header.stamp = joint_msg.header.stamp
        target_marker.ns = "target_point"
        target_marker.id = 0
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        target_marker.pose.position.x = float(target_pos[0])
        target_marker.pose.position.y = float(target_pos[1])
        target_marker.pose.position.z = float(target_pos[2])
        target_marker.pose.orientation = rpy_to_quaternion(*target_rpy)
        target_marker.scale.x = 0.025
        target_marker.scale.y = 0.025
        target_marker.scale.z = 0.025
        target_marker.color.r = 1.0
        target_marker.color.g = 0.2
        target_marker.color.b = 0.2
        target_marker.color.a = 0.85
        self.marker_pub.publish(target_marker)

        # 4. 发布末端历史轨迹 Line Strip (蓝色迹线)
        path_marker = Marker()
        path_marker.header.frame_id = "world"
        path_marker.header.stamp = joint_msg.header.stamp
        path_marker.ns = "ee_trajectory"
        path_marker.id = 1
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.004
        path_marker.color.r = 0.0
        path_marker.color.g = 0.7
        path_marker.color.b = 1.0
        path_marker.color.a = 0.95
        path_marker.points = self.path_points_history
        self.path_pub.publish(path_marker)

        # 5. 发布 Target Pose
        pose_msg = PoseStamped()
        pose_msg.header = joint_msg.header
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = float(target_pos[0])
        pose_msg.pose.position.y = float(target_pos[1])
        pose_msg.pose.position.z = float(target_pos[2])
        pose_msg.pose.orientation = target_marker.pose.orientation
        self.pose_pub.publish(pose_msg)

        # 6. 定期打印分析日志 (每 10 帧/0.2秒)
        self.step_counter += 1
        if self.step_counter % 10 == 0:
            q_deg = np.rad2deg(self.current_q[:5])
            self.get_logger().info(
                f"[{mode_name}] Step {self.step_counter:04d} | "
                f"Tgt: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}] | "
                f"FK_EE: [{fk_pos[0]:.3f}, {fk_pos[1]:.3f}, {fk_pos[2]:.3f}] | "
                f"IK Residual Err: {pos_error*1000.0:.3f} mm | "
                f"Manipulability: {manipulability:.4f} | "
                f"Q(deg): [{q_deg[0]:.1f}°, {q_deg[1]:.1f}°, {q_deg[2]:.1f}°, {q_deg[3]:.1f}°, {q_deg[4]:.1f}°]"
            )

def main(args=None):
    rclpy.init(args=args)
    node = SOArm101SimulationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
