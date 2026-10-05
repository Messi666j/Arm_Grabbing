import numpy as np

def quintic_polynomial_step(s):
    """
    五次多项式无因次时间插值系数 s in [0, 1]
    s(t) = 10*s^3 - 15*s^4 + 6*s^5
    保证始末端速度和加速度为零 (Zero velocity & acceleration boundary conditions)
    """
    s = np.clip(s, 0.0, 1.0)
    return 10.0 * (s ** 3) - 15.0 * (s ** 4) + 6.0 * (s ** 5)

def interpolate_joint_space(q_start, q_end, duration, dt=0.02):
    """
    关节空间五次多项式平滑轨迹生成
    """
    q_start = np.asarray(q_start, dtype=np.float64)
    q_end = np.asarray(q_end, dtype=np.float64)

    num_steps = int(max(1, round(duration / dt)))
    t_samples = np.linspace(0.0, duration, num_steps)

    trajectory = []
    for t in t_samples:
        s = t / duration if duration > 0 else 1.0
        factor = quintic_polynomial_step(s)
        q_curr = q_start + factor * (q_end - q_start)
        trajectory.append(q_curr)

    return np.array(trajectory), t_samples

class TrajectoryPlanner:
    """
    SO-ARM101 真实物理 CAD 模型的笛卡尔空间与关节空间轨迹规划器
    """

    def __init__(self, kinematics_solver):
        self.solver = kinematics_solver

    def generate_figure8_trajectory(self, center_pos=None, rx=0.06, ry=0.08, rz=0.02, duration=10.0, dt=0.02, q_init=None):
        """
        生成笛卡尔空间 3D 8字形 (Bernoulli Lemniscate 双扭线) 轨迹
        x(t) = x_c + R_x * sin(w*t)
        y(t) = y_c + R_y * sin(w*t) * cos(w*t)
        z(t) = z_c + R_z * cos(2*w*t)
        """
        if center_pos is None:
            center_pos = np.array([0.26, 0.0, 0.22])

        num_steps = int(max(1, round(duration / dt)))
        t_samples = np.linspace(0.0, duration, num_steps)

        joint_traj = []
        cartesian_points = []
        target_rpys = []
        ik_success_list = []

        if q_init is None:
            q_curr = np.zeros(6)
        else:
            q_curr = np.copy(q_init)

        for t in t_samples:
            theta = 2.0 * np.pi * (t / duration)
            x = center_pos[0] + rx * np.sin(theta)
            y = center_pos[1] + ry * np.sin(theta) * np.cos(theta)
            z = center_pos[2] + rz * np.cos(2.0 * theta)

            target_pos = np.array([x, y, z])
            cartesian_points.append(target_pos)
            target_rpys.append(np.zeros(3))

            # DLS 逆运动学求解
            q_sol, success, p_err, _ = self.solver.inverse_kinematics_dls(
                target_pos, q_init=q_curr, damping=1e-3, position_only=True
            )

            joint_traj.append(q_sol)
            ik_success_list.append(success)
            q_curr = np.copy(q_sol)

        return np.array(joint_traj), np.array(cartesian_points), target_rpys, ik_success_list

    def generate_circle_trajectory(self, center_pos=None, radius=0.06, height=0.20, duration=8.0, dt=0.02, q_init=None):
        """
        生成笛卡尔空间 3D 画圆轨迹及对应的关节角序列
        """
        if center_pos is None:
            center_pos = np.array([0.28, 0.0])

        num_steps = int(max(1, round(duration / dt)))
        t_samples = np.linspace(0.0, duration, num_steps)

        joint_traj = []
        cartesian_points = []
        target_rpys = []
        ik_success_list = []

        if q_init is None:
            q_curr = np.zeros(6)
        else:
            q_curr = np.copy(q_init)

        for t in t_samples:
            angle = 2.0 * np.pi * (t / duration)
            x = center_pos[0] + radius * np.cos(angle)
            y = center_pos[1] + radius * np.sin(angle)
            z = height + 0.01 * np.sin(2.0 * angle)

            target_pos = np.array([x, y, z])
            cartesian_points.append(target_pos)
            target_rpys.append(np.zeros(3))

            q_sol, success, p_err, _ = self.solver.inverse_kinematics_dls(
                target_pos, q_init=q_curr, damping=1e-3, position_only=True
            )

            joint_traj.append(q_sol)
            ik_success_list.append(success)
            q_curr = np.copy(q_sol)

        return np.array(joint_traj), np.array(cartesian_points), target_rpys, ik_success_list

    def generate_pick_and_place_waypoints(self):
        """
        生成 SO-ARM101 真实 CAD 模型的 3D 抓取与放置关键位姿序列
        """
        waypoints = [
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
        return waypoints
