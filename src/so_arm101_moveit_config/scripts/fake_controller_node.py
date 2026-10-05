#!/usr/bin/env python3
import time
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory

class FakeTrajectoryController(Node):
    def __init__(self):
        super().__init__('fake_trajectory_controller')

        self.arm_joints = [
            'shoulder_pan',
            'shoulder_lift',
            'elbow_flex',
            'wrist_flex',
            'wrist_roll'
        ]
        self.gripper_joints = ['gripper']
        self.all_joints = self.arm_joints + self.gripper_joints

        self.lock = threading.Lock()
        self.joint_positions = {j: 0.0 for j in self.all_joints}

        self.cancel_requested = False

        # Publisher for /joint_states at 50Hz
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.02, self.publish_joint_states)

        # Action server for arm_controller
        self._arm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_arm_trajectory,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Action server for gripper_controller
        self._gripper_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'gripper_controller/follow_joint_trajectory',
            execute_callback=self.execute_gripper_trajectory,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('FakeTrajectoryController initialized and ready for arm and gripper execution.')

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            msg.name = list(self.joint_positions.keys())
            msg.position = [float(self.joint_positions[j]) for j in msg.name]
            msg.velocity = [0.0] * len(msg.name)
            msg.effort = [0.0] * len(msg.name)
        self.joint_pub.publish(msg)

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received trajectory with {len(goal_request.trajectory.points)} points.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel request received.')
        self.cancel_requested = True
        return CancelResponse.ACCEPT

    def execute_arm_trajectory(self, goal_handle):
        return self._execute_trajectory(goal_handle)

    def execute_gripper_trajectory(self, goal_handle):
        return self._execute_trajectory(goal_handle)

    def _execute_trajectory(self, goal_handle):
        traj = goal_handle.request.trajectory
        joint_names = traj.joint_names
        points = traj.points

        if not points:
            goal_handle.succeed()
            res = FollowJointTrajectory.Result()
            res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return res

        self.cancel_requested = False
        start_time = time.time()
        total_duration = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1e-9

        times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in points]
        positions = np.array([p.positions for p in points])

        if len(points) == 1 or total_duration <= 0.0:
            with self.lock:
                for idx, j_name in enumerate(joint_names):
                    if j_name in self.joint_positions:
                        self.joint_positions[j_name] = positions[-1, idx]
            goal_handle.succeed()
            res = FollowJointTrajectory.Result()
            res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return res

        dt = 0.02
        while True:
            elapsed = time.time() - start_time
            if self.cancel_requested:
                goal_handle.canceled()
                res = FollowJointTrajectory.Result()
                res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return res

            if elapsed >= total_duration:
                with self.lock:
                    for idx, j_name in enumerate(joint_names):
                        if j_name in self.joint_positions:
                            self.joint_positions[j_name] = positions[-1, idx]
                break

            with self.lock:
                for idx, j_name in enumerate(joint_names):
                    if j_name in self.joint_positions:
                        val = np.interp(elapsed, times, positions[:, idx])
                        self.joint_positions[j_name] = float(val)

            time.sleep(dt)

        goal_handle.succeed()
        res = FollowJointTrajectory.Result()
        res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info('Trajectory execution completed successfully.')
        return res

def main(args=None):
    rclpy.init(args=args)
    node = FakeTrajectoryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
