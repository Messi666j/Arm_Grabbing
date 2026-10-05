import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_hardware = get_package_share_directory('so_arm101_hardware')
    pkg_description = get_package_share_directory('so_arm101_description')

    xacro_file = os.path.join(pkg_description, 'urdf', 'so_arm101.urdf.xacro')

    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyUSB0', description='Serial port path')
    baudrate_arg = DeclareLaunchArgument('baudrate', default_value='1000000', description='Baud rate')

    # Robot State Publisher Node
    robot_description = Command(['xacro ', xacro_file])
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Serial Driver Node
    driver_node = Node(
        package='so_arm101_hardware',
        executable='serial_driver_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate')
        }]
    )

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        rsp_node,
        driver_node,
        rviz_node
    ])
