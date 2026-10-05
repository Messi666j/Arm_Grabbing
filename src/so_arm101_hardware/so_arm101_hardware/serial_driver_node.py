import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

from so_arm101_hardware.sts3215_driver import STS3215Driver


class SerialDriverNode(Node):
    """
    SO-ARM101 串口硬件驱动节点
    按 50Hz 订阅/发布关节角度，与真实/模拟 STS3215 舵机进行通信。
    """

    JOINT_NAMES = [
        'Rotation',
        'Pitch',
        'Elbow',
        'Wrist_Pitch',
        'Wrist_Roll',
        'Jaw'
    ]

    def __init__(self):
        super().__init__('so_arm101_serial_driver_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('publish_rate', 50.0)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        publish_rate = self.get_parameter('publish_rate').value

        self.get_logger().info(f"Initializing SO-ARM101 Hardware Driver on port {port} @ {baudrate} bps...")

        self.driver = STS3215Driver(port=port, baudrate=baudrate, num_servos=len(self.JOINT_NAMES))

        # JointState 发布器
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # 指令订阅器
        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/joint_commands',
            self.joint_command_callback,
            10
        )

        # 50Hz 定时器
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("SO-ARM101 Hardware Driver Node fully initialized.")

    def timer_callback(self):
        """定时获取舵机角度并发布 JointState"""
        radians, _ = self.driver.read_all_positions()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.JOINT_NAMES
        msg.position = radians.tolist()

        self.joint_pub.publish(msg)

    def joint_command_callback(self, msg: Float64MultiArray):
        """接收上位机控制指令并下发"""
        if len(msg.data) >= len(self.JOINT_NAMES):
            self.driver.write_joint_positions(msg.data[:len(self.JOINT_NAMES)])

    def destroy_node(self):
        self.driver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
