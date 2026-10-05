"""
Kinematics Solver for SO-ARM101 Robotic Manipulator.

Integrates Pinocchio rigid-body dynamics with numerical Damped Least Squares (DLS)
Inverse Kinematics, Jacobian calculations, and manipulability analysis.
"""

import os
import subprocess
from typing import Optional, Tuple
import numpy as np
import pinocchio as pin

from so_arm101_model_based.spatial_math import (
    rpy_to_rotation_matrix,
    rotation_matrix_to_rpy,
    normalize_angles,
)

# Canonical SO-ARM101 Joint Names
CANONICAL_JOINT_NAMES = [
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
    'gripper'
]

ARM_DOF = 5
TOTAL_DOF = 6


def find_urdf_xacro_path() -> str:
    """Find the URDF/Xacro path across ROS install space or workspace src tree."""
    # 1. Try ament index (installed environment)
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('so_arm101_description')
        candidate = os.path.join(pkg_share, 'urdf', 'so_arm101.urdf.xacro')
        if os.path.exists(candidate):
            return candidate
    except Exception:
        pass

    # 2. Search workspace directories by walking upwards
    curr = os.path.abspath(__file__)
    for _ in range(6):
        curr = os.path.dirname(curr)
        candidate = os.path.join(curr, 'src', 'so_arm101_description', 'urdf', 'so_arm101.urdf.xacro')
        if os.path.exists(candidate):
            return candidate
        candidate_direct = os.path.join(curr, 'so_arm101_description', 'urdf', 'so_arm101.urdf.xacro')
        if os.path.exists(candidate_direct):
            return candidate_direct

    raise FileNotFoundError("Could not locate so_arm101.urdf.xacro in install or source space.")


class SOArm101Kinematics:
    """
    Kinematics solver for SO-ARM101 using Pinocchio.
    Provides forward kinematics, Jacobian computation, manipulability analysis,
    and Damped Least Squares (DLS) numerical inverse kinematics.
    """

    def __init__(self, xacro_path: Optional[str] = None, urdf_str: Optional[str] = None):
        if urdf_str is None:
            if xacro_path is None:
                xacro_path = find_urdf_xacro_path()

            if not os.path.exists(xacro_path):
                raise FileNotFoundError(f"URDF Xacro file not found at: {xacro_path}")

            cmd = ['xacro', xacro_path]
            urdf_str = subprocess.check_output(cmd).decode('utf-8')

        self.model = pin.buildModelFromXML(urdf_str)
        self.data = self.model.createData()

        # Resolve End-Effector Tool Frame
        self.tool_frame_id = self.model.getFrameId('gripper_frame_link')
        if self.tool_frame_id >= len(self.model.frames):
            self.tool_frame_id = self.model.getFrameId('gripper_link')
            if self.tool_frame_id >= len(self.model.frames):
                self.tool_frame_id = len(self.model.frames) - 1

        self.num_joints = self.model.nq
        self.q_min = self.model.lowerPositionLimit
        self.q_max = self.model.upperPositionLimit

    def forward_kinematics(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute Forward Kinematics (FK) for the end-effector.

        :param q: Joint configuration vector [q1..q6]
        :return: (pos [x,y,z], rotation_matrix [3x3], rpy [roll, pitch, yaw])
        """
        q_arr = np.asarray(q, dtype=np.float64)
        pin.forwardKinematics(self.model, self.data, q_arr)
        pin.updateFramePlacements(self.model, self.data)

        placement = self.data.oMf[self.tool_frame_id]
        pos = np.copy(placement.translation)
        R = np.copy(placement.rotation)
        rpy = rotation_matrix_to_rpy(R)

        return pos, R, rpy

    def compute_jacobian(self, q: np.ndarray) -> np.ndarray:
        """Compute the 6D geometric Jacobian matrix for the end-effector in world frame."""
        q_arr = np.asarray(q, dtype=np.float64)
        pin.computeJointJacobians(self.model, self.data, q_arr)
        pin.updateFramePlacements(self.model, self.data)

        return pin.getFrameJacobian(
            self.model, self.data, self.tool_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )

    def compute_manipulability(self, q: np.ndarray) -> float:
        """Compute Yoshikawa Manipulability Index for the 5-DOF positioning arm."""
        J = self.compute_jacobian(q)[:3, :ARM_DOF]
        JJT = J @ J.T
        det = np.linalg.det(JJT)
        return float(np.sqrt(max(0.0, det)))

    def inverse_kinematics_dls(
        self,
        target_pos: np.ndarray,
        target_rpy: Optional[np.ndarray] = None,
        target_R: Optional[np.ndarray] = None,
        q_init: Optional[np.ndarray] = None,
        max_iter: int = 150,
        eps_pos: float = 1e-4,
        eps_rot: float = 1e-3,
        damping: float = 1e-3,
        position_only: bool = True
    ) -> Tuple[np.ndarray, bool, float, float]:
        """
        Damped Least Squares (DLS) numerical Inverse Kinematics solver.

        :param target_pos: Target 3D Cartesian position [x, y, z] (m)
        :param target_rpy: Optional target orientation in RPY angles (rad)
        :param target_R: Optional target 3x3 rotation matrix
        :param q_init: Initial joint angles guess (default: neutral)
        :param max_iter: Maximum iterations
        :param eps_pos: Position tolerance (m)
        :param eps_rot: Orientation tolerance (rad)
        :param damping: Levenberg-Marquardt damping factor
        :param position_only: If True, prioritize 3D position convergence
        :return: (q_sol, success, pos_error, rot_error)
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

        damping_sq = damping ** 2

        for i in range(max_iter):
            curr_pos, curr_R, _ = self.forward_kinematics(q)

            # Position error
            pos_err = target_p - curr_pos
            pos_err_val = float(np.linalg.norm(pos_err))

            # Orientation error
            if target_R is not None:
                R_err = curr_R.T @ target_R
                rot_err = pin.log3(R_err)
                rot_err_val = float(np.linalg.norm(rot_err))
            else:
                rot_err = np.zeros(3)
                rot_err_val = 0.0

            if position_only:
                if pos_err_val < eps_pos:
                    success = True
                    break

                J_pos = self.compute_jacobian(q)[:3, :ARM_DOF]
                dls_mat = (J_pos @ J_pos.T) + damping_sq * np.eye(3)
                dq_arm = J_pos.T @ np.linalg.solve(dls_mat, pos_err)

            else:
                if pos_err_val < eps_pos and rot_err_val < eps_rot:
                    success = True
                    break

                err_6d = np.concatenate([pos_err, rot_err])
                J_6d = self.compute_jacobian(q)[:6, :ARM_DOF]
                dls_mat = (J_6d @ J_6d.T) + damping_sq * np.eye(6)
                dq_arm = J_6d.T @ np.linalg.solve(dls_mat, err_6d)

            dq = np.zeros(self.num_joints)
            dq[:ARM_DOF] = dq_arm

            # Adaptive step size
            step_size = 0.5 if i < 30 else 0.8
            q = q + step_size * dq
            q = np.clip(q, self.q_min, self.q_max)

        # Clip gripper joint if present
        if self.num_joints > ARM_DOF:
            q[ARM_DOF] = np.clip(q[ARM_DOF], self.q_min[ARM_DOF], self.q_max[ARM_DOF])

        return q, success, pos_err_val, rot_err_val
