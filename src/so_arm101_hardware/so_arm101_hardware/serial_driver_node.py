"""
SO-ARM101 Hardware Serial Driver Node.

Publishes 50Hz JointState from STS3215 bus servos (or simulation shadow)
and subscribes to /joint_commands for real-time actuator control.
"""

from typing import List
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from so_arm101_hardware.sts3215_driver import (
    STS3215Driver,
    JointCalibration,
    DEFAULT_CANONICAL_JOINTS
)


class SerialDriverNode(Node):
    """
    ROS 2 Hardware Interface Driver for SO-ARM101 Manipulator.
    Aligns hardware state publication with canonical URDF kinematics.
    """

    def __init__(self):
        super().__init__('so_arm101_serial_driver_node')

        # Declare configurable parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('joint_names', DEFAULT_CANONICAL_JOINTS)
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('zero_offsets', [2048, 2048, 2048, 2048, 2048, 2048])
        self.declare_parameter('direction_signs', [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.joint_names = list(self.get_parameter('joint_names').value)
        servo_ids = list(self.get_parameter('servo_ids').value)
        zero_offsets = np.array(self.get_parameter('zero_offsets').value, dtype=np.int32)
        direction_signs = np.array(self.get_parameter('direction_signs').value, dtype=np.float32)

        self.get_logger().info(f"Initializing SO-ARM101 Hardware Driver on {port} @ {baudrate} baud...")
        self.get_logger().info(f"Target Joint Seam: {self.joint_names}")

        calibration = JointCalibration(
            joint_names=self.joint_names,
            servo_ids=servo_ids,
            zero_offsets=zero_offsets,
            direction_signs=direction_signs
        )

        self.driver = STS3215Driver(
            port=port,
            baudrate=baudrate,
            calibration=calibration
        )

        # Publishers & Subscribers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/joint_commands',
            self.joint_command_callback,
            10
        )

        # Timer for publish loop
        timer_period = 1.0 / max(1.0, publish_rate)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        mode_str = "HARDWARE" if self.driver.is_open else "SIMULATION SHADOW"
        self.get_logger().info(f"SO-ARM101 Serial Driver active in [{mode_str}] mode @ {publish_rate} Hz.")

    def timer_callback(self):
        """Query servo positions and broadcast JointState."""
        radians, _ = self.driver.read_all_positions()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = radians.tolist()
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)

        self.joint_pub.publish(msg)

    def joint_command_callback(self, msg: Float64MultiArray):
        """Receive joint commands and dispatch to actuators."""
        n_joints = len(self.joint_names)
        if len(msg.data) >= n_joints:
            self.driver.write_joint_positions(np.array(msg.data[:n_joints], dtype=np.float32))

    def destroy_node(self):
        self.driver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialDriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException, Exception):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
