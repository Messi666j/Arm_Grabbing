"""Unit tests for spatial_math module."""

import numpy as np
import pytest

from so_arm101_model_based.spatial_math import (
    normalize_angle,
    normalize_angles,
    rpy_to_rotation_matrix,
    rotation_matrix_to_rpy,
    rpy_to_quaternion,
    quaternion_to_rpy,
    rotation_matrix_to_quaternion,
)


def test_angle_normalization():
    assert np.isclose(normalize_angle(0.0), 0.0)
    assert np.isclose(normalize_angle(np.pi), -np.pi)  # wrap edge
    assert np.isclose(normalize_angle(3.0 * np.pi), -np.pi)
    assert np.isclose(normalize_angle(0.5 * np.pi), 0.5 * np.pi)

    arr = np.array([0.0, np.pi, 2.5 * np.pi, -3.5 * np.pi])
    norm_arr = normalize_angles(arr)
    assert len(norm_arr) == 4
    assert np.all(norm_arr >= -np.pi)
    assert np.all(norm_arr <= np.pi)


def test_rpy_and_rotation_matrix_identity():
    R = rpy_to_rotation_matrix(0.0, 0.0, 0.0)
    assert np.allclose(R, np.eye(3))

    rpy = rotation_matrix_to_rpy(np.eye(3))
    assert np.allclose(rpy, np.zeros(3))


def test_rpy_to_matrix_and_back():
    test_cases = [
        (0.2, -0.3, 0.5),
        (0.0, 0.4, -0.8),
        (-0.5, 0.0, 1.2),
        (1.0, -0.7, 0.0),
    ]
    for roll, pitch, yaw in test_cases:
        R = rpy_to_rotation_matrix(roll, pitch, yaw)
        # Check orthogonality: R @ R.T = I, det(R) = 1
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-7)
        assert np.isclose(np.linalg.det(R), 1.0, atol=1e-7)

        rpy_rec = rotation_matrix_to_rpy(R)
        assert np.allclose([roll, pitch, yaw], rpy_rec, atol=1e-6)


def test_quaternion_conversions():
    # Identity
    qx, qy, qz, qw = rpy_to_quaternion(0.0, 0.0, 0.0)
    assert np.isclose(qw, 1.0)
    assert np.isclose(qx, 0.0) and np.isclose(qy, 0.0) and np.isclose(qz, 0.0)

    # 90 deg around Z
    qx, qy, qz, qw = rpy_to_quaternion(0.0, 0.0, np.pi / 2.0)
    assert np.isclose(qz, np.sin(np.pi / 4.0))
    assert np.isclose(qw, np.cos(np.pi / 4.0))

    # Roundtrip RPY <-> Quaternion
    rpy_orig = np.array([0.3, -0.4, 0.6])
    q = rpy_to_quaternion(*rpy_orig)
    rpy_rec = quaternion_to_rpy(*q)
    assert np.allclose(rpy_orig, rpy_rec, atol=1e-6)


def test_rotation_matrix_to_quaternion():
    R = rpy_to_rotation_matrix(0.4, -0.5, 0.8)
    q = rotation_matrix_to_quaternion(R)
    # Check unit norm
    norm = np.linalg.norm(q)
    assert np.isclose(norm, 1.0)

    # Reconstruct RPY from quaternion
    rpy_from_q = quaternion_to_rpy(*q)
    assert np.allclose([0.4, -0.5, 0.8], rpy_from_q, atol=1e-6)
