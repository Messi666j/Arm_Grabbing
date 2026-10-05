#!/usr/bin/env python3
"""
Fake Trajectory Controller for SO-ARM101 MoveIt 2 Execution.

Implements FollowJointTrajectory action servers for arm and gripper controllers,
interpolating joint positions and publishing 50Hz /joint_states concurrently using
MultiThreadedExecutor to prevent callback starvation during trajectory execution.
"""

import time
import threading
from typing import Dict, Set
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory

from so_arm101_model_based.kinematics import CANONICAL_JOINT_NAMES


class FakeTrajectoryController(Node):
    """
    Action server simulating controller execution for MoveIt 2.
    """

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

        self.cb_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.joint_positions: Dict[str, float] = {j: 0.0 for j in self.all_joints}

        self.canceled_goals: Set[str] = set()

        # 50Hz JointState Publisher
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.02, self.publish_joint_states, callback_group=self.cb_group)

        # Action server for arm_controller
        self._arm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_arm_trajectory,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        # Action server for gripper_controller
        self._gripper_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'gripper_controller/follow_joint_trajectory',
            execute_callback=self.execute_gripper_trajectory,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        self.get_logger().info('FakeTrajectoryController initialized (MultiThreaded, Reentrant) for arm & gripper.')

    def publish_joint_states(self):
        """Broadcast current joint states at 50Hz."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            msg.name = list(self.joint_positions.keys())
            msg.position = [float(self.joint_positions[j]) for j in msg.name]
            msg.velocity = [0.0] * len(msg.name)
            msg.effort = [0.0] * len(msg.name)
        self.joint_pub.publish(msg)

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received trajectory goal with {len(goal_request.trajectory.points)} points.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        goal_id_str = str(goal_handle.goal_id.uuid)
        self.get_logger().info(f'Cancel request received for goal: {goal_id_str}')
        with self.lock:
            self.canceled_goals.add(goal_id_str)
        return CancelResponse.ACCEPT

    def execute_arm_trajectory(self, goal_handle):
        return self._execute_trajectory(goal_handle)

    def execute_gripper_trajectory(self, goal_handle):
        return self._execute_trajectory(goal_handle)

    def _execute_trajectory(self, goal_handle):
        traj = goal_handle.request.trajectory
        joint_names = traj.joint_names
        points = traj.points
        goal_id_str = str(goal_handle.goal_id.uuid)

        if not points:
            goal_handle.succeed()
            res = FollowJointTrajectory.Result()
            res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return res

        start_time = time.time()
        total_duration = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1e-9

        times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in points]
        positions = np.array([p.positions for p in points])

        if len(points) == 1 or total_duration <= 0.0:
            with self.lock:
                for idx, j_name in enumerate(joint_names):
                    if j_name in self.joint_positions:
                        self.joint_positions[j_name] = float(positions[-1, idx])
            goal_handle.succeed()
            res = FollowJointTrajectory.Result()
            res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return res

        dt = 0.02
        while True:
            elapsed = time.time() - start_time

            # Check if this goal was canceled
            with self.lock:
                if goal_id_str in self.canceled_goals:
                    self.canceled_goals.remove(goal_id_str)
                    goal_handle.canceled()
                    res = FollowJointTrajectory.Result()
                    res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    return res

            if elapsed >= total_duration:
                with self.lock:
                    for idx, j_name in enumerate(joint_names):
                        if j_name in self.joint_positions:
                            self.joint_positions[j_name] = float(positions[-1, idx])
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
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException, Exception):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
