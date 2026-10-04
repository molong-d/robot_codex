from pathlib import Path
import json
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def _gazebo_robot_description(description: str, controllers_file: Path, fixed_base: bool) -> str:
    root = ET.fromstring(description)
    for hardware in root.findall(".//ros2_control/hardware"):
        plugin = hardware.find("plugin")
        if plugin is None or plugin.text != "mock_components/GenericSystem":
            raise RuntimeError("Panda description did not contain the expected GenericSystem hardware")
        plugin.text = "gz_ros2_control/GazeboSimSystem"

    for link in ("panda_leftfinger", "panda_rightfinger"):
        contact = ET.SubElement(root, "gazebo", {"reference": link})
        ET.SubElement(contact, "mu1").text = "5.0"
        ET.SubElement(contact, "mu2").text = "5.0"

    gazebo = ET.SubElement(root, "gazebo")
    plugin = ET.SubElement(gazebo, "plugin", {
        "filename": "libgz_ros2_control-system.so",
        "name": "gz_ros2_control::GazeboSimROS2ControlPlugin",
    })
    ET.SubElement(plugin, "robot_param_node").text = "robot_state_publisher"
    ET.SubElement(plugin, "parameters").text = str(controllers_file)
    if fixed_base:
        ET.SubElement(root, "link", {"name": "world"})
        anchor = ET.SubElement(root, "joint", {"name": "world_anchor", "type": "fixed"})
        ET.SubElement(anchor, "parent", {"link": "world"})
        ET.SubElement(anchor, "child", {"link": "panda_link0"})
    return ET.tostring(root, encoding="unicode")


def generate_launch_description():
    share = Path(get_package_share_directory("robot_panda_gz_sim"))
    acceptance_file = share / "config" / "acceptance.json"
    acceptance = json.loads(acceptance_file.read_text(encoding="utf-8"))
    acceptance_parameters = {
        "perception_frame": acceptance["world_frame"],
        "gazebo_world_name": acceptance["gazebo_world_name"],
        "support_surface_z": acceptance["support_surface_z_m"],
        "object_height_m": acceptance["object_height_m"],
        "grasp_lift_m": acceptance["grasp_lift_m"],
        "placement_xy_tolerance_m": acceptance["placement_xy_tolerance_m"],
        "placement_z_tolerance_m": acceptance["placement_z_tolerance_m"],
        "stable_speed_mps": acceptance["stable_speed_mps"],
        "verification_window_ms": acceptance["verification_window_ms"],
        "verification_minimum_samples": acceptance["minimum_physical_samples"],
        "evidence_max_age_ms": acceptance["evidence_max_age_ms"],
        "contact_pose_pairing_tolerance_ms": acceptance["contact_pose_pairing_tolerance_ms"],
        "stop_timeout_ms": acceptance["stop_timeout_ms"],
    }
    moveit = (
        MoveItConfigsBuilder("moveit_resources_panda", package_name="moveit_resources_panda_moveit_config")
        .robot_description(file_path="config/panda.urdf.xacro",
                           mappings={"ros2_control_hardware_type": "mock_components"})
        .robot_description_semantic(file_path="config/panda.srdf")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    original_description = moveit.robot_description["robot_description"]
    hardware_description = _gazebo_robot_description(
        original_description, share / "config" / "controllers.yaml", False)
    moveit.robot_description["robot_description"] = hardware_description
    moveit_dict = moveit.to_dict()
    moveit_dict.setdefault("trajectory_execution", {}).update({
        "allowed_execution_duration_scaling": 1.2,
        "allowed_goal_duration_margin": 5.0,
    })
    moveit_dict["robot_description"] = hardware_description
    spawn_description = _gazebo_robot_description(
        original_description, share / "config" / "controllers.yaml", True)
    world = share / "worlds" / "panda_pick_place.sdf"
    controller_library = Path(get_package_prefix("moveit_simple_controller_manager")) / \
        "lib/libmoveit_simple_controller_manager.so"
    if not controller_library.is_file():
        raise RuntimeError(f"MoveIt controller plugin library missing: {controller_library}")
    preload = " ".join(filter(None, [str(controller_library), os.environ.get("LD_PRELOAD", "")]))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory("ros_gz_sim")) / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-s -r -v 2 --seed ", LaunchConfiguration("seed"),
                                       " ", LaunchConfiguration("world_file")],
                          "on_exit_shutdown": "true"}.items(),
    )
    spawn_robot = Node(
        package="ros_gz_sim", executable="create", name="spawn_panda",
        arguments=["-param", "robot_description", "-name", "panda", "-world", "panda_pick_place"],
        parameters=[{"robot_description": spawn_description, "use_sim_time": True}],
        output="screen",
    )
    joint_states = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": spawn_description, "use_sim_time": True}], output="screen")
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group",
        parameters=[moveit_dict, {"use_sim_time": True}],
        additional_env={"LD_PRELOAD": preload}, output="screen")
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="gazebo_ros_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/panda_gz/workpiece/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/panda_gz/tray/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/world/panda_pick_place/model/workpiece/link/link/sensor/workpiece_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
            "/world/panda_pick_place/model/table/link/link/sensor/table_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
        ], output="screen")
    fault_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="gazebo_fault_bridge",
        arguments=["/world/panda_pick_place/set_pose@ros_gz_interfaces/srv/SetEntityPose"],
        condition=IfCondition(LaunchConfiguration("enable_test_fault_services")), output="screen")
    scene_sync = Node(package="robot_panda_gz_sim", executable="gz_scene_sync",
                      parameters=[{"use_sim_time": True,
                                   "world_frame": acceptance["world_frame"],
                                   "gazebo_world_name": acceptance["gazebo_world_name"],
                                   "object_id": acceptance["object_id"],
                                   "evidence_max_age_ms": acceptance["evidence_max_age_ms"],
                                   "publish_grasp_feedback": ParameterValue(
                                       LaunchConfiguration("publish_grasp_feedback"), value_type=bool)}],
                      output="screen")

    broadcaster = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager", "--controller-manager-timeout", "90"],
        output="screen")
    arm = Node(
        package="controller_manager", executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager", "--controller-manager-timeout", "90"],
        output="screen")
    hand = Node(
        package="controller_manager", executable="spawner",
        arguments=["panda_hand_controller", "-c", "/controller_manager", "--controller-manager-timeout", "90"],
        output="screen")
    runtime = Node(
        package="robot_bt_runtime", executable="robot_runtime", name="robot_runtime",
        parameters=[LaunchConfiguration("config_file"), acceptance_parameters,
                    {"use_sim_time": True}], output="screen")
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=str(share / "config" / "runtime.yaml")),
        DeclareLaunchArgument("seed", default_value="42"),
        DeclareLaunchArgument("world_file", default_value=str(world)),
        DeclareLaunchArgument("publish_grasp_feedback", default_value="true"),
        DeclareLaunchArgument("enable_test_fault_services", default_value="false"),
        gazebo,
        bridge,
        fault_bridge,
        joint_states,
        move_group,
        scene_sync,
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[broadcaster])),
        RegisterEventHandler(OnProcessExit(target_action=broadcaster, on_exit=[arm])),
        RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=[hand])),
        RegisterEventHandler(OnProcessExit(target_action=hand, on_exit=[runtime])),
        spawn_robot,
    ])
