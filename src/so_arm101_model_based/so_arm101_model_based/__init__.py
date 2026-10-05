"""
SO-ARM101 Traditional Model-Based Control Package.
Provides FK, DLS IK, Trajectory Generation and ROS 2 Simulation interface.
"""

from so_arm101_model_based.spatial_math import (
    normalize_angle,
    normalize_angles,
    rpy_to_rotation_matrix,
    rotation_matrix_to_rpy,
    rpy_to_quaternion,
    quaternion_to_rpy,
    rotation_matrix_to_quaternion,
)
from so_arm101_model_based.kinematics import (
    SOArm101Kinematics,
    CANONICAL_JOINT_NAMES,
    ARM_DOF,
    TOTAL_DOF,
)
from so_arm101_model_based.trajectory import (
    TrajectoryPlanner,
    quintic_polynomial_step,
    interpolate_joint_space,
)

__version__ = '0.1.0'

__all__ = [
    'SOArm101Kinematics',
    'CANONICAL_JOINT_NAMES',
    'ARM_DOF',
    'TOTAL_DOF',
    'TrajectoryPlanner',
    'quintic_polynomial_step',
    'interpolate_joint_space',
    'normalize_angle',
    'normalize_angles',
    'rpy_to_rotation_matrix',
    'rotation_matrix_to_rpy',
    'rpy_to_quaternion',
    'quaternion_to_rpy',
    'rotation_matrix_to_quaternion',
]
