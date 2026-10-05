import os
import subprocess
import numpy as np
import pinocchio as pin

def rpy_to_rotation_matrix(roll, pitch, yaw):
    """Convert Roll-Pitch-Yaw angles (in radians) to 3x3 Rotation Matrix (XYZ convention)."""
    cr = np.cos(roll)
    sr = np.sin(roll)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cy = np.cos(yaw)
    sy = np.sin(yaw)

    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

    return Rz @ Ry @ Rx

def rotation_matrix_to_rpy(R):
    """Convert 3x3 Rotation Matrix to Roll-Pitch-Yaw angles (XYZ convention)."""
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
    if np.abs(np.cos(pitch)) > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = 0.0
        yaw = np.arctan2(-R[0, 1], R[1, 1])
    return np.array([roll, pitch, yaw])

class SOArm101Kinematics:
    """
    SO-ARM101 机械臂正/逆运动学求解器与动力学基础类 (基于 Pinocchio & DLS 数值法)
    """

    def __init__(self, xacro_path=None, urdf_str=None):
        if urdf_str is None:
            if xacro_path is None:
                try:
                    from ament_index_python.packages import get_package_share_directory
                    pkg_share = get_package_share_directory('so_arm101_description')
                    xacro_path = os.path.join(pkg_share, 'urdf', 'so_arm101.urdf.xacro')
                except Exception:
                    pkg_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
                    xacro_path = os.path.join(pkg_dir, 'so_arm101_description', 'urdf', 'so_arm101.urdf.xacro')
            
            if not os.path.exists(xacro_path):
                raise FileNotFoundError(f"URDF Xacro file not found at: {xacro_path}")
            
            cmd = ['xacro', xacro_path]
            urdf_str = subprocess.check_output(cmd).decode('utf-8')

        self.model = pin.buildModelFromXML(urdf_str)
        self.data = self.model.createData()

        self.tool_frame_id = self.model.getFrameId('gripper_frame_link')
        if self.tool_frame_id >= len(self.model.frames):
            self.tool_frame_id = self.model.getFrameId('gripper_link')

        self.num_joints = self.model.nq
        self.q_min = self.model.lowerPositionLimit
        self.q_max = self.model.upperPositionLimit

    def forward_kinematics(self, q):
        """
        计算正运动学 (FK)
        :param q: 关节角度向量 numpy array [q1, q2, q3, q4, q5, q_gripper]
        :return: pos (x,y,z), R (3x3 rotation matrix), rpy (roll, pitch, yaw)
        """
        q_arr = np.asarray(q, dtype=np.float64)
        pin.forwardKinematics(self.model, self.data, q_arr)
        pin.updateFramePlacements(self.model, self.data)

        placement = self.data.oMf[self.tool_frame_id]
        pos = np.copy(placement.translation)
        R = np.copy(placement.rotation)
        rpy = rotation_matrix_to_rpy(R)

        return pos, R, rpy

    def compute_jacobian(self, q):
        """
        计算末端执行器 6D 雅可比矩阵
        """
        q_arr = np.asarray(q, dtype=np.float64)
        pin.computeJointJacobians(self.model, self.data, q_arr)
        pin.updateFramePlacements(self.model, self.data)

        J = pin.getFrameJacobian(self.model, self.data, self.tool_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        return J

    def compute_manipulability(self, q):
        """
        计算 Yoshikawa 操纵度 (Manipulability Index)，用于衡量机械臂是否接近奇异点
        """
        J = self.compute_jacobian(q)[:3, :5] # 前5个旋转关节位置雅可比
        JJT = J @ J.T
        det = np.linalg.det(JJT)
        return np.sqrt(max(0.0, det))

    def inverse_kinematics_dls(
        self,
        target_pos,
        target_rpy=None,
        target_R=None,
        q_init=None,
        max_iter=150,
        eps_pos=1e-4,
        eps_rot=1e-3,
        damping=1e-3,
        position_only=True
    ):
        """
        阻尼最小二乘法 (Damped Least Squares, DLS) 逆运动学求解
        :param target_pos: 目标位置 [x, y, z]
        :param target_rpy: 目标姿态 [roll, pitch, yaw] (或传入 target_R)
        :param target_R: 目标 3x3 旋转矩阵
        :param q_init: 初始关节角度向量 (默认为 neutral)
        :param max_iter: 最大迭代次数
        :param eps_pos: 位置容差 (米)
        :param eps_rot: 姿态容差 (弧度)
        :param damping: 阻尼因子 $\lambda$ (防止在奇异构型下求逆爆炸)
        :param position_only: 是否优先约束 3D 笛卡尔位置 (适用于 5-DOF 机械臂空间轨迹精准追踪)
        :return: (q_sol, success, pos_err_val, rot_err_val)
        """
        if q_init is None:
            q = pin.neutral(self.model)
        else:
            q = np.copy(q_init).astype(np.float64)

        if target_R is None and target_rpy is not None:
            target_R = rpy_to_rotation_matrix(*target_rpy)

        target_p = np.asarray(target_pos, dtype=np.float64)
        success = False

        pos_err_val = 0.0
        rot_err_val = 0.0

        for i in range(max_iter):
            curr_pos, curr_R, _ = self.forward_kinematics(q)

            # 位置误差
            pos_err = target_p - curr_pos
            pos_err_val = np.linalg.norm(pos_err)

            # 姿态误差
            if target_R is not None:
                R_err = curr_R.T @ target_R
                rot_err = pin.log3(R_err)
                rot_err_val = np.linalg.norm(rot_err)
            else:
                rot_err = np.zeros(3)
                rot_err_val = 0.0

            if position_only:
                if pos_err_val < eps_pos:
                    success = True
                    break
                
                # 3D 位置雅可比 (3 x 5)
                J_pos = self.compute_jacobian(q)[:3, :5]
                JJT = J_pos @ J_pos.T
                dls_mat = JJT + (damping ** 2) * np.eye(3)
                dq5 = J_pos.T @ np.linalg.solve(dls_mat, pos_err)

            else:
                if pos_err_val < eps_pos and rot_err_val < eps_rot:
                    success = True
                    break

                err_6d = np.concatenate([pos_err, rot_err])
                J_6d = self.compute_jacobian(q)[:6, :5]
                JJT = J_6d @ J_6d.T
                dls_mat = JJT + (damping ** 2) * np.eye(6)
                dq5 = J_6d.T @ np.linalg.solve(dls_mat, err_6d)

            dq = np.zeros(self.num_joints)
            dq[:5] = dq5

            # 动态步长调节
            step_size = 0.5 if i < 30 else 0.8
            q = q + step_size * dq

            # 关节限位保护
            q = np.clip(q, self.q_min, self.q_max)

        q[5] = np.clip(q[5], self.q_min[5], self.q_max[5])
        return q, success, pos_err_val, rot_err_val
