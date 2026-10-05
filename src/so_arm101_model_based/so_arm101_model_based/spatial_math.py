"""
Spatial Math Module for SO-ARM101 Kinematics and Trajectory Planning.

Consolidates SO(3) rotations, Euler angles (RPY, XYZ convention),
unit quaternions, and angle wrapping utilities behind a high-locality interface.
"""

from typing import Tuple, Union
import numpy as np


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi] radians."""
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def normalize_angles(angles: np.ndarray) -> np.ndarray:
    """Normalize an array of angles to [-pi, pi] radians."""
    arr = np.asarray(angles, dtype=np.float64)
    return (arr + np.pi) % (2.0 * np.pi) - np.pi


def rpy_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Convert Roll-Pitch-Yaw angles (radians) to a 3x3 SO(3) rotation matrix.
    Uses the standard robotics XYZ Tait-Bryan convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll).
    """
    cr = np.cos(roll)
    sr = np.sin(roll)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cy = np.cos(yaw)
    sy = np.sin(yaw)

    Rx = np.array([[1.0, 0.0, 0.0],
                   [0.0, cr, -sr],
                   [0.0, sr, cr]], dtype=np.float64)
    Ry = np.array([[cp, 0.0, sp],
                   [0.0, 1.0, 0.0],
                   [-sp, 0.0, cp]], dtype=np.float64)
    Rz = np.array([[cy, -sy, 0.0],
                   [sy, cy, 0.0],
                   [0.0, 0.0, 1.0]], dtype=np.float64)

    return Rz @ Ry @ Rx


def rotation_matrix_to_rpy(R: np.ndarray) -> np.ndarray:
    """
    Convert a 3x3 SO(3) rotation matrix to Roll-Pitch-Yaw angles (XYZ convention).
    Handles gimbal lock when cos(pitch) ~= 0.
    """
    R_mat = np.asarray(R, dtype=np.float64)
    pitch = np.arctan2(-R_mat[2, 0], np.sqrt(R_mat[0, 0] ** 2 + R_mat[1, 0] ** 2))

    if np.abs(np.cos(pitch)) > 1e-6:
        roll = np.arctan2(R_mat[2, 1], R_mat[2, 2])
        yaw = np.arctan2(R_mat[1, 0], R_mat[0, 0])
    else:
        # Gimbal lock singularity
        roll = 0.0
        yaw = np.arctan2(-R_mat[0, 1], R_mat[1, 1])

    return np.array([roll, pitch, yaw], dtype=np.float64)


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """
    Convert Roll-Pitch-Yaw to unit quaternion (qx, qy, qz, qw).
    """
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

    norm = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm > 1e-12:
        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

    return float(qx), float(qy), float(qz), float(qw)


def quaternion_to_rpy(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """
    Convert unit quaternion (qx, qy, qz, qw) to Roll-Pitch-Yaw angles (radians).
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if np.abs(sinp) >= 1.0:
        pitch = np.sign(sinp) * (np.pi / 2.0)
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw], dtype=np.float64)


def rotation_matrix_to_quaternion(R: np.ndarray) -> Tuple[float, float, float, float]:
    """
    Convert 3x3 rotation matrix to quaternion (qx, qy, qz, qw) using Shepperd's algorithm.
    """
    R_mat = np.asarray(R, dtype=np.float64)
    tr = np.trace(R_mat)

    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R_mat[2, 1] - R_mat[1, 2]) / S
        qy = (R_mat[0, 2] - R_mat[2, 0]) / S
        qz = (R_mat[1, 0] - R_mat[0, 1]) / S
    elif (R_mat[0, 0] > R_mat[1, 1]) and (R_mat[0, 0] > R_mat[2, 2]):
        S = np.sqrt(1.0 + R_mat[0, 0] - R_mat[1, 1] - R_mat[2, 2]) * 2.0
        qw = (R_mat[2, 1] - R_mat[1, 2]) / S
        qx = 0.25 * S
        qy = (R_mat[0, 1] + R_mat[1, 0]) / S
        qz = (R_mat[0, 2] + R_mat[2, 0]) / S
    elif R_mat[1, 1] > R_mat[2, 2]:
        S = np.sqrt(1.0 + R_mat[1, 1] - R_mat[0, 0] - R_mat[2, 2]) * 2.0
        qw = (R_mat[0, 2] - R_mat[2, 0]) / S
        qx = (R_mat[0, 1] + R_mat[1, 0]) / S
        qy = 0.25 * S
        qz = (R_mat[1, 2] + R_mat[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R_mat[2, 2] - R_mat[0, 0] - R_mat[1, 1]) * 2.0
        qw = (R_mat[1, 0] - R_mat[0, 1]) / S
        qx = (R_mat[0, 2] + R_mat[2, 0]) / S
        qy = (R_mat[1, 2] + R_mat[2, 1]) / S
        qz = 0.25 * S

    return float(qx), float(qy), float(qz), float(qw)
