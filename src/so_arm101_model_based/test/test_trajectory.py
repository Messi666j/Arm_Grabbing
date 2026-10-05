"""Unit tests for trajectory generation."""

import numpy as np
import pytest

from so_arm101_model_based.spatial_math import normalize_angles
from so_arm101_model_based.kinematics import SOArm101Kinematics, TOTAL_DOF
from so_arm101_model_based.trajectory import (
    quintic_polynomial_step,
    interpolate_joint_space,
    TrajectoryPlanner,
)


@pytest.fixture(scope="module")
def kinematics_solver():
    return SOArm101Kinematics()


def test_quintic_polynomial_boundaries():
    assert np.isclose(quintic_polynomial_step(0.0), 0.0)
    assert np.isclose(quintic_polynomial_step(1.0), 1.0)
    assert np.isclose(quintic_polynomial_step(-0.5), 0.0)
    assert np.isclose(quintic_polynomial_step(1.5), 1.0)

    # Check midpoint
    mid = quintic_polynomial_step(0.5)
    assert np.isclose(mid, 0.5)


def test_interpolate_joint_space():
    q_start = np.zeros(TOTAL_DOF)
    q_end = np.array([0.5, -0.3, 0.4, -0.2, 0.1, 0.8])
    duration = 2.0
    dt = 0.02

    traj, t_samples = interpolate_joint_space(q_start, q_end, duration, dt=dt)

    assert len(traj) == 100
    assert len(t_samples) == 100
    assert np.allclose(traj[0], q_start)
    assert np.allclose(traj[-1], q_end)


def test_figure8_trajectory_generation(kinematics_solver):
    planner = TrajectoryPlanner(kinematics_solver)
    traj, cart_pts, rpys, success_list = planner.generate_figure8_trajectory(
        duration=2.0, dt=0.1
    )

    assert len(traj) == len(cart_pts)
    assert len(cart_pts) > 0
    # Success rate should be high
    success_rate = sum(success_list) / len(success_list)
    assert success_rate > 0.8, f"Figure-8 IK success rate too low: {success_rate:.2f}"


def test_circle_trajectory_generation(kinematics_solver):
    planner = TrajectoryPlanner(kinematics_solver)
    traj, cart_pts, rpys, success_list = planner.generate_circle_trajectory(
        duration=2.0, dt=0.1
    )

    assert len(traj) == len(cart_pts)
    assert len(cart_pts) > 0
    success_rate = sum(success_list) / len(success_list)
    assert success_rate > 0.8, f"Circle IK success rate too low: {success_rate:.2f}"


def test_pick_and_place_waypoints(kinematics_solver):
    planner = TrajectoryPlanner(kinematics_solver)
    waypoints = planner.generate_pick_and_place_waypoints()

    assert len(waypoints) == 10
    for wp in waypoints:
        pos, rpy, gripper_val, phase_name, duration = wp
        assert len(pos) == 3
        assert len(rpy) == 3
        assert 0.0 <= gripper_val <= 1.5
        assert duration > 0.0
