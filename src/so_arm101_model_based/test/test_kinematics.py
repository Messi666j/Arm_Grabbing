"""Unit tests for SO-ARM101 kinematics solver."""

import numpy as np
import pytest

from so_arm101_model_based.kinematics import SOArm101Kinematics, ARM_DOF, TOTAL_DOF


@pytest.fixture(scope="module")
def kinematics_solver():
    return SOArm101Kinematics()


def test_kinematics_init(kinematics_solver):
    assert kinematics_solver.num_joints == TOTAL_DOF
    assert kinematics_solver.tool_frame_id > 0
    assert len(kinematics_solver.q_min) == TOTAL_DOF
    assert len(kinematics_solver.q_max) == TOTAL_DOF


def test_forward_kinematics_neutral(kinematics_solver):
    q_zero = np.zeros(TOTAL_DOF)
    pos, R, rpy = kinematics_solver.forward_kinematics(q_zero)

    assert len(pos) == 3
    assert R.shape == (3, 3)
    assert len(rpy) == 3
    # Robot should have positive reach forward and above base
    assert pos[0] > 0.05
    assert pos[2] > 0.05


def test_jacobian_and_manipulability(kinematics_solver):
    q_neutral = np.array([0.0, 0.2, -0.4, 0.2, 0.0, 0.0])
    J = kinematics_solver.compute_jacobian(q_neutral)
    assert J.shape[0] == 6
    assert J.shape[1] == TOTAL_DOF

    w = kinematics_solver.compute_manipulability(q_neutral)
    assert w > 0.0, "Manipulability index should be strictly positive for non-singular configuration"


def test_inverse_kinematics_dls_reachable(kinematics_solver):
    # Select a known reachable forward pose
    q_target = np.array([0.1, 0.3, -0.5, 0.2, 0.0, 0.0])
    pos_target, _, _ = kinematics_solver.forward_kinematics(q_target)

    # Solve IK from zero guess
    q_sol, success, pos_err, _ = kinematics_solver.inverse_kinematics_dls(
        pos_target,
        q_init=np.zeros(TOTAL_DOF),
        eps_pos=1e-3,
        position_only=True
    )

    assert success, f"DLS IK failed to converge to reachable pose, residual err: {pos_err*1000:.2f} mm"
    assert pos_err < 1e-3

    # Check joint limits
    assert np.all(q_sol >= kinematics_solver.q_min - 1e-5)
    assert np.all(q_sol <= kinematics_solver.q_max + 1e-5)
