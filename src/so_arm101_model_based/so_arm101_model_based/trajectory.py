"""
Trajectory Generation and Planning Module for SO-ARM101.

Provides quintic polynomial interpolation in joint space and Cartesian task space
generators for 3D Lemniscate (figure-8), circular paths, and pick-and-place waypoints.
"""

from typing import Callable, List, Optional, Tuple
import numpy as np


def quintic_polynomial_step(s: float) -> float:
    """
    Normalized quintic polynomial step factor for s in [0, 1].
    s(t) = 10*s^3 - 15*s^4 + 6*s^5
    Guarantees zero velocity and zero acceleration at boundary conditions (s=0, s=1).
    """
    s_clamped = np.clip(s, 0.0, 1.0)
    return float(10.0 * (s_clamped ** 3) - 15.0 * (s_clamped ** 4) + 6.0 * (s_clamped ** 5))


def interpolate_joint_space(
    q_start: np.ndarray,
    q_end: np.ndarray,
    duration: float,
    dt: float = 0.02
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Generate smooth quintic polynomial joint trajectory between two configurations.

    :param q_start: Starting joint configuration
    :param q_end: Ending joint configuration
    :param duration: Segment duration in seconds
    :param dt: Time sampling interval in seconds
    :return: (trajectory array [N x num_joints], time samples array [N])
    """
    q_start_arr = np.asarray(q_start, dtype=np.float64)
    q_end_arr = np.asarray(q_end, dtype=np.float64)

    num_steps = int(max(1, round(duration / dt)))
    t_samples = np.linspace(0.0, duration, num_steps)

    trajectory = []
    for t in t_samples:
        s = t / duration if duration > 0 else 1.0
        factor = quintic_polynomial_step(s)
        q_curr = q_start_arr + factor * (q_end_arr - q_start_arr)
        trajectory.append(q_curr)

    return np.array(trajectory), t_samples


class TrajectoryPlanner:
    """
    Trajectory planner for SO-ARM101 task-space and joint-space motions.
    """

    def __init__(self, kinematics_solver):
        self.solver = kinematics_solver

    def sample_cartesian_trajectory(
        self,
        path_fn: Callable[[float], Tuple[np.ndarray, np.ndarray]],
        duration: float,
        dt: float = 0.02,
        q_init: Optional[np.ndarray] = None,
        damping: float = 1e-3,
        position_only: bool = True
    ) -> Tuple[np.ndarray, np.ndarray, List[np.ndarray], List[bool]]:
        """
        Deep template generator that samples a Cartesian path function and tracks it via DLS IK.

        :param path_fn: Function mapping t in [0, duration] -> (target_pos [3], target_rpy [3])
        :param duration: Total trajectory duration in seconds
        :param dt: Sample interval
        :param q_init: Initial joint configuration guess
        :param damping: DLS damping factor
        :param position_only: Whether to prioritize Cartesian position over orientation
        :return: (joint_trajectory [N x num_joints], cartesian_points [N x 3], target_rpys, ik_success_list)
        """
        num_steps = int(max(1, round(duration / dt)))
        t_samples = np.linspace(0.0, duration, num_steps)

        joint_traj = []
        cartesian_points = []
        target_rpys = []
        ik_success_list = []

        if q_init is None:
            q_curr = np.zeros(self.solver.num_joints)
        else:
            q_curr = np.copy(q_init)

        for t in t_samples:
            target_pos, target_rpy = path_fn(t)
            cartesian_points.append(target_pos)
            target_rpys.append(target_rpy)

            q_sol, success, _, _ = self.solver.inverse_kinematics_dls(
                target_pos,
                target_rpy=target_rpy if not position_only else None,
                q_init=q_curr,
                damping=damping,
                position_only=position_only
            )

            joint_traj.append(q_sol)
            ik_success_list.append(success)
            q_curr = np.copy(q_sol)

        return np.array(joint_traj), np.array(cartesian_points), target_rpys, ik_success_list

    def generate_figure8_trajectory(
        self,
        center_pos: Optional[np.ndarray] = None,
        rx: float = 0.06,
        ry: float = 0.08,
        rz: float = 0.02,
        duration: float = 10.0,
        dt: float = 0.02,
        q_init: Optional[np.ndarray] = None
    ) -> Tuple[np.ndarray, np.ndarray, List[np.ndarray], List[bool]]:
        """
        Generate 3D Cartesian Lemniscate of Bernoulli (figure-8) trajectory.
        x(t) = x_c + rx * sin(theta)
        y(t) = y_c + ry * sin(theta) * cos(theta)
        z(t) = z_c + rz * cos(2*theta)
        """
        if center_pos is None:
            center_pos = np.array([0.26, 0.0, 0.22])
        else:
            center_pos = np.asarray(center_pos, dtype=np.float64)

        def path_fn(t: float) -> Tuple[np.ndarray, np.ndarray]:
            theta = 2.0 * np.pi * (t / duration)
            x = center_pos[0] + rx * np.sin(theta)
            y = center_pos[1] + ry * np.sin(theta) * np.cos(theta)
            z = center_pos[2] + rz * np.cos(2.0 * theta)
            return np.array([x, y, z]), np.zeros(3)

        return self.sample_cartesian_trajectory(path_fn, duration, dt, q_init)

    def generate_circle_trajectory(
        self,
        center_pos: Optional[np.ndarray] = None,
        radius: float = 0.06,
        height: float = 0.20,
        duration: float = 8.0,
        dt: float = 0.02,
        q_init: Optional[np.ndarray] = None
    ) -> Tuple[np.ndarray, np.ndarray, List[np.ndarray], List[bool]]:
        """
        Generate 3D Cartesian circular trajectory in task space.
        Supports both 3D center [x, y, z] and legacy 2D center [x, y] with separate height parameter.
        """
        if center_pos is None:
            c = np.array([0.28, 0.0, height])
        else:
            c_arr = np.asarray(center_pos, dtype=np.float64)
            if len(c_arr) == 2:
                c = np.array([c_arr[0], c_arr[1], height])
            else:
                c = c_arr

        def path_fn(t: float) -> Tuple[np.ndarray, np.ndarray]:
            angle = 2.0 * np.pi * (t / duration)
            x = c[0] + radius * np.cos(angle)
            y = c[1] + radius * np.sin(angle)
            z = c[2] + 0.01 * np.sin(2.0 * angle)
            return np.array([x, y, z]), np.zeros(3)

        return self.sample_cartesian_trajectory(path_fn, duration, dt, q_init)

    def generate_pick_and_place_waypoints(self) -> List[Tuple[np.ndarray, np.ndarray, float, str, float]]:
        """
        Return sequenced (pos, rpy, gripper_angle, phase_name, duration) waypoints
        for full pick and place cycle.
        """
        return [
            (np.array([0.28, 0.00, 0.25]), np.zeros(3), 0.0, "Home Pose", 2.5),
            (np.array([0.26, 0.10, 0.18]), np.zeros(3), 0.8, "Approach Object A", 2.0),
            (np.array([0.26, 0.10, 0.10]), np.zeros(3), 0.8, "Descend & Touch A", 1.5),
            (np.array([0.26, 0.10, 0.10]), np.zeros(3), 0.0, "Grasp Object A", 1.0),
            (np.array([0.26, 0.10, 0.25]), np.zeros(3), 0.0, "Lift Object A", 1.5),
            (np.array([0.26, -0.10, 0.25]), np.zeros(3), 0.0, "Transfer to Target B", 2.5),
            (np.array([0.26, -0.10, 0.10]), np.zeros(3), 0.0, "Lower to Target B", 1.5),
            (np.array([0.26, -0.10, 0.10]), np.zeros(3), 0.8, "Release Object A", 1.0),
            (np.array([0.26, -0.10, 0.25]), np.zeros(3), 0.8, "Retract Arm", 1.5),
            (np.array([0.28, 0.00, 0.25]), np.zeros(3), 0.0, "Return Home", 2.0),
        ]
