from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    share = Path(get_package_share_directory("robot_panda_demo"))
    config = (
        MoveItConfigsBuilder("moveit_resources_panda", package_name="moveit_resources_panda_moveit_config")
        .robot_description(file_path="config/panda.urdf.xacro",
                           mappings={"ros2_control_hardware_type": "mock_components"})
        .robot_description_semantic(file_path="config/panda.srdf")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    # Hardware selection is fixed to GenericSystem; no real driver launch argument.
    return LaunchDescription([
        DeclareLaunchArgument("mock_motion_permitted", default_value="true"),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["--frame-id", "world", "--child-frame-id", "panda_link0"], output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[config.robot_description], output="screen"),
        Node(package="moveit_ros_move_group", executable="move_group",
             parameters=[config.to_dict()], output="screen"),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[config.robot_description, str(share / "config" / "controllers.yaml")],
             remappings=[("/controller_manager/robot_description", "/robot_description")], output="screen"),
        *[Node(package="controller_manager", executable="spawner",
               arguments=[name, "-c", "/controller_manager", "--controller-manager-timeout", "60"],
               output="screen")
          for name in ["joint_state_broadcaster", "panda_arm_controller", "panda_hand_controller"]],
        Node(package="robot_bt_runtime", executable="robot_runtime", name="robot_runtime",
             parameters=[str(share / "config" / "runtime.yaml"), {
                 "mock_motion_permitted": ParameterValue(LaunchConfiguration("mock_motion_permitted"), value_type=bool),
             }], output="screen"),
    ])
