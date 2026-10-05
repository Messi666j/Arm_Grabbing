import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_so_arm101_desc = FindPackageShare('so_arm101_description')
    pkg_so_arm101_model = FindPackageShare('so_arm101_model_based')

    xacro_file = PathJoinSubstitution([pkg_so_arm101_desc, 'urdf', 'so_arm101.urdf.xacro'])
    rviz_config_file = PathJoinSubstitution([pkg_so_arm101_model, 'config', 'kinematics_sim.rviz'])

    # 解析 xacro 生成 URDF robot_description 参数 (ROS 2 Humble 必须指定 ParameterValue value_type=str)
    robot_description_content = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    robot_description = {'robot_description': robot_description_content}

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 2. SO-ARM101 传统控制仿真节点
    sim_controller_node = Node(
        package='so_arm101_model_based',
        executable='simulation_node',
        output='screen'
    )

    # 3. RViz 2 界面
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        sim_controller_node,
        rviz_node
    ])
