from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = Path(get_package_share_directory("robot_bringup")) / "config" / "demo.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("motion_component", default_value="mock_arm"),
        DeclareLaunchArgument("mock_action_ticks", default_value="3"),
        DeclareLaunchArgument("mock_fail_pick", default_value="false"),
        DeclareLaunchArgument("mock_motion_permitted", default_value="true"),
        Node(
            package="robot_bt_runtime",
            executable="robot_runtime",
            name="robot_runtime",
            output="screen",
            parameters=[str(config), {
                "motion_component": LaunchConfiguration("motion_component"),
                "mock_action_ticks": ParameterValue(LaunchConfiguration("mock_action_ticks"), value_type=int),
                "mock_fail_pick": ParameterValue(LaunchConfiguration("mock_fail_pick"), value_type=bool),
                "mock_motion_permitted": ParameterValue(LaunchConfiguration("mock_motion_permitted"), value_type=bool),
            }],
        ),
    ])
