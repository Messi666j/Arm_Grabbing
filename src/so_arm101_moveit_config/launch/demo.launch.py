import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    xacro_path = os.path.join(
        get_package_share_directory("so_arm101_description"),
        "urdf",
        "so_arm101.urdf.xacro"
    )

    moveit_config_share = get_package_share_directory("so_arm101_moveit_config")
    rviz_config_path = os.path.join(moveit_config_share, "config", "moveit.rviz")

    moveit_config = (
        MoveItConfigsBuilder("so_arm101", package_name="so_arm101_moveit_config")
        .robot_description(file_path=xacro_path)
        .robot_description_semantic(file_path="config/so_arm101.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )

    gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="false",
        description="Whether to start joint_state_publisher_gui instead of fake trajectory controller",
    )

    use_gui = LaunchConfiguration("use_gui")

    return LaunchDescription(
        [
            gui_arg,
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[moveit_config.to_dict()],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_path],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.planning_pipelines,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[moveit_config.robot_description],
            ),
            # Fake trajectory controller action server for Plan & Execute
            Node(
                package="so_arm101_moveit_config",
                executable="fake_controller_node.py",
                name="fake_trajectory_controller",
                output="screen",
                condition=UnlessCondition(use_gui),
            ),
            # Optional GUI slider mode
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                condition=IfCondition(use_gui),
            ),
        ]
    )
