#!/usr/bin/env python3
"""Launch the isolated Panda/Gazebo stack and run one measured pick-place goal."""

import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import tempfile
import time
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

from control_msgs.action import ParallelGripperCommand
from geometry_msgs.msg import Pose
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rclpy.time import Time
from robot_interfaces.action import ExecuteTask
from robot_interfaces.msg import GraspContact
from robot_interfaces.srv import GetExecution, GetRuntimeState
from ros_gz_interfaces.msg import Contacts, Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

from gazebo_evidence import audit_physics
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/robot_panda_gz_sim/scripts"))
from source_time import SourceTimeGuard


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-ms", type=int, default=30000)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--object-dx", type=float, default=0.0)
    parser.add_argument("--object-dy", type=float, default=0.0)
    parser.add_argument("--placement-offset-y", type=float,
                        help="fault injection: add a Y error to the tool-in-tray calibration")
    parser.add_argument("--grasp-miss-x", type=float,
                        help="fault injection: shift the calibrated grasp pose in X to miss the object")
    parser.add_argument("--cancel-after-ms", type=int, default=0)
    parser.add_argument("--pause-after-ms", type=int, default=0)
    parser.add_argument("--pause-duration-ms", type=int, default=1000)
    parser.add_argument("--reset-after-ms", type=int, default=0,
                        help="reset the Gazebo world while the task is active")
    parser.add_argument("--drop-grasp-feedback", action="store_true")
    parser.add_argument("--drop-contact-after-release", action="store_true",
                        help="stop stamped contact feedback after release is independently confirmed")
    parser.add_argument("--disturb-object-before-verification", action="store_true",
                        help="fault-inject an off-tray Gazebo pose after placement motion, before verification")
    parser.add_argument("--drop-object-after-grasp", action="store_true",
                        help="open the simulated gripper during lift to produce a physical drop")
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--code-commit", default=None)
    parser.add_argument("--result-json", type=Path, default=Path("docs/reports/phase2-gazebo-result.json"))
    parser.add_argument("--probe-gripper", action="store_true",
                        help="command the actuated Panda finger and report the measured joint state")
    parser.add_argument("--launch-log", type=Path, default=Path("docs/reports/phase2-gazebo-launch.log"))
    args = parser.parse_args()
    if not 1000 <= args.timeout_ms <= 600000:
        parser.error("--timeout-ms must be between 1000 and 600000")

    root = Path(__file__).resolve().parents[1]
    run_id = args.run_id or f"standalone-{uuid.uuid4().hex}"
    code_commit = args.code_commit or subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=root, text=True, capture_output=True, check=True).stdout.strip()
    acceptance = json.loads((root / "src/robot_panda_gz_sim/config/acceptance.json").read_text(encoding="utf-8"))
    args.launch_log.parent.mkdir(parents=True, exist_ok=True)
    args.result_json.parent.mkdir(parents=True, exist_ok=True)
    if args.result_json.exists():
        parser.error(f"refusing to reuse an existing result file: {args.result_json}")
    numeric_options = (args.object_dx, args.object_dy) + tuple(
        value for value in (args.placement_offset_y, args.grasp_miss_x) if value is not None)
    if not all(map(math.isfinite, numeric_options)):
        parser.error("pose offsets must be finite")
    if abs(args.object_dx) > 0.05 or abs(args.object_dy) > 0.05:
        parser.error("object offsets must stay within 0.05 m of the configured pose")
    if args.grasp_miss_x is not None and abs(args.grasp_miss_x) > 0.15:
        parser.error("--grasp-miss-x must stay within 0.15 m of the calibrated grasp pose")
    if args.cancel_after_ms < 0 or args.pause_after_ms < 0 or args.reset_after_ms < 0 or args.pause_duration_ms <= 0:
        parser.error("cancel/pause delays must be nonnegative and pause duration must be positive")
    if sum(bool(value) for value in (args.cancel_after_ms, args.pause_after_ms, args.reset_after_ms)) > 1:
        parser.error("run cancellation, pause, and reset scenarios separately")

    def emit_result(value):
        value["run_id"] = run_id
        value["code_commit"] = code_commit
        value["seed"] = args.seed
        serialized = json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2)
        args.result_json.write_text(serialized + "\n", encoding="utf-8")
        print(serialized)
    launch_output = args.launch_log.open("w")
    launch_env = dict(os.environ)
    temp_world_dir = None
    world_path = root / "src/robot_panda_gz_sim/worlds/panda_pick_place.sdf"
    config_path = root / "src/robot_panda_gz_sim/config/runtime.yaml"
    if (args.object_dx != 0.0 or args.object_dy != 0.0 or args.placement_offset_y is not None or
            args.grasp_miss_x is not None):
        temp_world_dir = tempfile.TemporaryDirectory(prefix="robot-codex-gz-case-")
    if args.object_dx != 0.0 or args.object_dy != 0.0:
        tree = ET.parse(world_path)
        workpiece = tree.find(".//model[@name='workpiece']")
        if workpiece is None or workpiece.find("pose") is None:
            parser.error("world file has no workpiece model pose")
        pose = workpiece.find("pose")
        xyzrpy = [float(value) for value in pose.text.split()]
        xyzrpy[0] += args.object_dx
        xyzrpy[1] += args.object_dy
        pose.text = " ".join(f"{value:.9g}" for value in xyzrpy)
        world_path = Path(temp_world_dir.name) / "panda_pick_place.sdf"
        tree.write(world_path, encoding="utf-8", xml_declaration=True)
    if args.placement_offset_y is not None or args.grasp_miss_x is not None:
        config_text = config_path.read_text(encoding="utf-8")
        def yaml_float(value):
            formatted = f"{value:.9g}"
            return formatted if any(char in formatted for char in ".eE") else formatted + ".0"

        injections = (("place_tool_offset", 1, args.placement_offset_y),
                      ("grasp_tool_offset", 0, args.grasp_miss_x))
        for key, index, offset in injections:
            if offset is None:
                continue
            pattern = rf"(?m)^(\s*target_resolution\.{key}:\s*\[)([^\]]+)(\])"
            match = re.search(pattern, config_text)
            if match is None:
                parser.error(f"runtime config has no target_resolution.{key}")
            values = [float(value.strip()) for value in match.group(2).split(",")]
            if len(values) != 7:
                parser.error(f"target_resolution.{key} must contain seven values")
            values[index] += offset
            replacement = match.group(1) + ", ".join(yaml_float(value) for value in values) + match.group(3)
            config_text = config_text[:match.start()] + replacement + config_text[match.end():]
        config_path = Path(temp_world_dir.name) / "runtime.yaml"
        config_path.write_text(config_text, encoding="utf-8")
    launch_command = ["ros2", "launch", "robot_panda_gz_sim", "panda_pick_place.launch.py",
                      f"seed:={args.seed}", f"world_file:={world_path}",
                      f"config_file:={config_path}",
                      f"publish_grasp_feedback:={'false' if args.drop_grasp_feedback else 'true'}",
                      f"enable_test_fault_services:={'true' if args.disturb_object_before_verification else 'false'}"]
    process = subprocess.Popen(
        launch_command,
        cwd=root, stdout=launch_output, stderr=subprocess.STDOUT,
        start_new_session=True, env=launch_env)
    context = Context()
    executor = None
    node = None
    tf_listener = None
    tf_buffer = None
    outcome = {"success": False, "status": "not_started", "error_code": "", "message": ""}
    pose_samples = {}
    pose_history = []
    contact_samples = []
    task_events = []
    sample_guard = SourceTimeGuard(max_age_ns=int(acceptance["evidence_max_age_ms"] * 1_000_000))
    stream_sequences = {}
    scene_ready_state = {"ready": False, "messages": 0}
    cancel_timing = {"requested_monotonic_ns": None, "requested_sim_time_ns": None,
                     "result_received_monotonic_ns": None, "stop_confirmed_monotonic_ns": None,
                     "stop_confirmed_source_ns": None, "timeout_deadline_monotonic_ns": None,
                     "stop_trigger_monotonic_ns": None, "stop_trigger_sim_time_ns": None}
    contact_state = {"messages": 0, "stamp_ns": 0, "contacts": [], "seen": [],
                     "table_messages": 0, "table_collisions": [], "table_seen": [],
                     "grasp_feedback_messages": 0, "grasp_feedback": None,
                     "stamped_grasp_messages": 0, "stamped_grasp_source_ns": 0,
                     "stamped_grasp_state": None}
    arm_state = {"hand_tf": None, "joint_state": None}
    finger_extrema = {"panda_finger_joint1": [None, None], "panda_finger_joint2": [None, None]}
    gazebo_topics = []
    action_endpoints = []
    task_feedback = {"active_skill": "", "status": "", "message": "", "history": []}
    feedback_parameter_client = None
    set_pose_client = None
    verification_fault_future = None
    verification_fault_started = False
    reset_result = None
    try:
        rclpy.init(context=context)
        # Sensor source stamps are Gazebo /clock time. Keep ROS time for source
        # ordering/age checks and use time.monotonic_ns() only for receipt age,
        # cancellation and process-stop deadlines.
        node = rclpy.create_node("gazebo_pick_place_test", context=context,
                                 parameter_overrides=[Parameter("use_sim_time", value=True)])
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        if args.drop_contact_after_release:
            feedback_parameter_client = node.create_client(SetParameters, "/gazebo_scene_sync/set_parameters")
            if not feedback_parameter_client.wait_for_service(timeout_sec=8.0):
                raise TimeoutError("Gazebo contact-feedback parameter service did not become ready")
        if args.disturb_object_before_verification:
            set_pose_client = node.create_client(SetEntityPose, "/world/panda_pick_place/set_pose")
            if not set_pose_client.wait_for_service(timeout_sec=8.0):
                raise TimeoutError("Gazebo test-only set-pose service did not become ready")
        tf_buffer = Buffer(node=node)
        tf_listener = TransformListener(tf_buffer, node, spin_thread=False)
        def on_poses(message):
            for transform in message.transforms:
                receipt_ns = time.monotonic_ns()
                sim_ns = node.get_clock().now().nanoseconds
                source_ns = transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec
                child = transform.child_frame_id
                if child == "panda_hand":
                    arm_state["hand_tf"] = {
                        "parent": transform.header.frame_id,
                        "stamp_ns": transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec,
                        "xyz": [transform.transform.translation.x, transform.transform.translation.y,
                                transform.transform.translation.z],
                        "quaternion_xyzw": [transform.transform.rotation.x, transform.transform.rotation.y,
                                             transform.transform.rotation.z, transform.transform.rotation.w],
                    }
                object_id = ("workpiece" if child == "workpiece" or child.endswith("::workpiece") else
                             "tray" if child == "tray" or child.endswith("::tray") else None)
                if object_id is None:
                    continue
                accepted, epoch, reason = sample_guard.observe(f"pose:{object_id}", source_ns, sim_ns)
                stream_sequences[object_id] = stream_sequences.get(object_id, 0) + 1
                source_frame = transform.header.frame_id
                canonical_frame = (acceptance["world_frame"] if source_frame in
                                   (acceptance["world_frame"], acceptance["gazebo_world_name"])
                                   else source_frame)
                sample = {
                    "stream": f"pose:{object_id}", "run_id": run_id, "object_id": object_id, "child_frame_id": child,
                    "frame_id": canonical_frame, "source_frame_id": source_frame,
                    "source_stamp_ns": source_ns,
                    "sim_time_ns": sim_ns, "receipt_monotonic_ns": receipt_ns,
                    "epoch": epoch, "sequence": stream_sequences[object_id],
                    "accepted": accepted, "reject_reason": reason,
                    "xyz": [transform.transform.translation.x, transform.transform.translation.y,
                            transform.transform.translation.z],
                }
                pose_history.append(sample)
                if accepted:
                    pose_samples[object_id] = sample
        pose_subscriptions = [
            node.create_subscription(TFMessage, "/panda_gz/workpiece/pose", on_poses, 10),
            node.create_subscription(TFMessage, "/panda_gz/tray/pose", on_poses, 10),
        ]
        def on_joint_state(message):
            receipt_ns = time.monotonic_ns()
            source_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            sim_ns = node.get_clock().now().nanoseconds
            accepted, epoch, reason = sample_guard.observe("joint_state", source_ns, sim_ns)
            arm_state["joint_state"] = {
                "stamp_ns": source_ns, "sim_time_ns": sim_ns, "epoch": epoch,
                "accepted": accepted, "reject_reason": reason,
                "receipt_monotonic_ns": receipt_ns,
                "names": list(message.name), "positions": list(message.position),
                "velocities": list(message.velocity),
            }
            stop_trigger = cancel_timing["stop_trigger_monotonic_ns"]
            if stop_trigger is not None and cancel_timing["stop_confirmed_monotonic_ns"] is None:
                required = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                            "panda_joint5", "panda_joint6", "panda_joint7",
                            "panda_finger_joint1", "panda_finger_joint2"]
                stationary = all(name in message.name and
                    message.name.index(name) < len(message.velocity) and
                    abs(message.velocity[message.name.index(name)]) <= 0.001 for name in required)
                source_ns = arm_state["joint_state"]["stamp_ns"]
                trigger_sim = cancel_timing["stop_trigger_sim_time_ns"]
                after_trigger = trigger_sim is None or source_ns > trigger_sim
                if accepted and epoch == sample_guard.epoch and stationary and after_trigger and \
                        receipt_ns >= stop_trigger:
                    cancel_timing["stop_confirmed_monotonic_ns"] = receipt_ns
                    cancel_timing["stop_confirmed_source_ns"] = source_ns
            for name, extrema in finger_extrema.items():
                if name in message.name:
                    value = message.position[message.name.index(name)]
                    extrema[0] = value if extrema[0] is None else min(extrema[0], value)
                    extrema[1] = value if extrema[1] is None else max(extrema[1], value)
        pose_subscriptions.append(node.create_subscription(JointState, "/joint_states", on_joint_state, 20))
        def on_contacts(message):
            receipt_ns = time.monotonic_ns()
            sim_ns = node.get_clock().now().nanoseconds
            source_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            contact_state["messages"] += 1
            contact_state["stamp_ns"] = source_ns
            contact_state["contacts"] = [
                [contact.collision1.name, contact.collision2.name] for contact in message.contacts
            ]
            contact_state["seen"] = sorted({tuple(pair) for pair in contact_state["seen"] + contact_state["contacts"]})
            accepted, epoch, reason = sample_guard.observe("contact", source_ns, sim_ns)
            stream_sequences["contact"] = stream_sequences.get("contact", 0) + 1
            contact_samples.append({
                "stream": "contact", "run_id": run_id, "object_id": "workpiece", "frame_id": acceptance["world_frame"],
                "source_frame_id": message.header.frame_id,
                "source_stamp_ns": source_ns, "sim_time_ns": sim_ns,
                "receipt_monotonic_ns": receipt_ns, "epoch": epoch,
                "sequence": stream_sequences["contact"], "accepted": accepted,
                "reject_reason": reason, "contacts": contact_state["contacts"],
            })
        pose_subscriptions.append(node.create_subscription(Contacts,
            "/world/panda_pick_place/model/workpiece/link/link/sensor/workpiece_contact/contact", on_contacts, 10))
        def on_table_contacts(message):
            contact_state["table_messages"] += 1
            contact_state["table_collisions"] = [
                [contact.collision1.name, contact.collision2.name] for contact in message.contacts
            ]
            contact_state["table_seen"] = sorted(
                {tuple(pair) for pair in contact_state["table_seen"] + contact_state["table_collisions"]})
        pose_subscriptions.append(node.create_subscription(Contacts,
            "/world/panda_pick_place/model/table/link/link/sensor/table_contact/contact",
            on_table_contacts, 10))
        def on_grasp_feedback(message):
            contact_state["grasp_feedback_messages"] += 1
            contact_state["grasp_feedback"] = message.data
        pose_subscriptions.append(node.create_subscription(Bool, "/panda/grasp_contact", on_grasp_feedback, 10))
        def on_stamped_grasp_feedback(message):
            contact_state["stamped_grasp_messages"] += 1
            contact_state["stamped_grasp_source_ns"] = (
                message.source_stamp.sec * 1_000_000_000 + message.source_stamp.nanosec)
            contact_state["stamped_grasp_state"] = message.state
            contact_state["stamped_grasp_schema_version"] = message.schema_version
            contact_state["stamped_grasp_epoch"] = message.epoch
            contact_state["stamped_grasp_sequence"] = message.sequence
            contact_state["stamped_grasp_frame"] = message.frame_id
            contact_state["stamped_grasp_object_id"] = message.object_id
        pose_subscriptions.append(node.create_subscription(
            GraspContact, "/panda/grasp_contact_stamped", on_stamped_grasp_feedback, 10))
        pose_subscriptions.append(node.create_subscription(
            Bool, "/panda_gz/scene_ready",
            lambda message: scene_ready_state.update(ready=message.data,
                                                     messages=scene_ready_state["messages"] + 1), 10))
        client = ActionClient(node, ExecuteTask, "/execute_task")
        deadline = time.monotonic() + 90.0
        last_launch_check = 0.0
        while time.monotonic() < deadline and process.poll() is None:
            executor.spin_once(timeout_sec=0.1)
            if client.server_is_ready():
                break
            now = time.monotonic()
            if now - last_launch_check >= 1.0:
                launch_output.flush()
                launch_text = args.launch_log.read_text(encoding="utf-8", errors="replace")
                if "[robot_runtime-" in launch_text and "process has died" in launch_text:
                    raise RuntimeError("robot_runtime exited before its action server became ready")
                last_launch_check = now
        else:
            raise RuntimeError("robot_runtime action server did not become ready")
        sample_deadline = time.monotonic() + 1.0
        while time.monotonic() < sample_deadline:
            executor.spin_once(timeout_sec=0.05)
        topic_result = subprocess.run(["gz", "topic", "-l"], cwd=root, env=launch_env,
                                      text=True, capture_output=True, timeout=8.0)
        if topic_result.returncode == 0:
            gazebo_topics = [line.strip() for line in topic_result.stdout.splitlines()
                             if any(token in line.lower() for token in ("contact", "panda_gz", "sensor"))]
        action_result = subprocess.run(["ros2", "action", "list", "-t"], cwd=root, env=launch_env,
                                       text=True, capture_output=True, timeout=8.0)
        if action_result.returncode == 0:
            action_endpoints = [line.strip() for line in action_result.stdout.splitlines() if line.strip()]

        scene_deadline = time.monotonic() + 20.0
        while time.monotonic() < scene_deadline and not scene_ready_state["ready"]:
            executor.spin_once(timeout_sec=0.1)
        if not scene_ready_state["ready"]:
            raise TimeoutError("MoveIt planning scene did not receive a successful current-epoch confirmation")

        if args.probe_gripper:
            probe = {"goals": [], "joint_state": arm_state["joint_state"]}
            gripper_client = ActionClient(node, ParallelGripperCommand,
                                          "/panda_hand_controller/gripper_cmd")
            if not gripper_client.wait_for_server(timeout_sec=5.0):
                raise RuntimeError("Panda hand trajectory action server unavailable for probe")
            for joint_position in (0.04, 0.0):
                goal = ParallelGripperCommand.Goal()
                goal.command.name = ["panda_finger_joint1"]
                goal.command.position = [joint_position]
                goal.command.effort = [20.0]
                sent_probe = gripper_client.send_goal_async(goal)
                executor.spin_until_future_complete(sent_probe, timeout_sec=8.0)
                if not sent_probe.done() or not sent_probe.result().accepted:
                    probe["goals"].append({"joint_position": joint_position, "accepted": False})
                    continue
                result_probe = sent_probe.result().get_result_async()
                executor.spin_until_future_complete(result_probe, timeout_sec=8.0)
                settle = time.monotonic() + 0.5
                while time.monotonic() < settle:
                    executor.spin_once(timeout_sec=0.05)
                probe["goals"].append({
                    "joint_position": joint_position,
                    "accepted": True,
                    "reached_goal": result_probe.result().result.reached_goal if result_probe.done() else None,
                    "stalled": result_probe.result().result.stalled if result_probe.done() else None,
                    "joint_state": arm_state["joint_state"],
                })
            outcome = {"status": "gripper_probe", "gripper_probe": probe,
                       "launch_log": str(args.launch_log), "action_endpoints": action_endpoints,
                       "seed": args.seed, "object_offset_xy_m": [args.object_dx, args.object_dy]}
            emit_result(outcome)
            gripper_client.destroy()
            return 0

        goal = ExecuteTask.Goal(task_name="pick_place", object_id="workpiece", target_id="tray",
                                timeout_ms=args.timeout_ms)
        def on_feedback(message):
            entry = {"active_skill": message.feedback.active_skill, "status": message.feedback.status,
                     "message": message.feedback.message,
                     "receipt_monotonic_ns": time.monotonic_ns(),
                     "sim_time_ns": node.get_clock().now().nanoseconds,
                     "run_id": run_id, "object_id": "workpiece", "target_id": "tray",
                     "epoch": sample_guard.epoch}
            if "release confirmed" in entry["message"].lower():
                entry["phase"] = "release_confirmed"
            elif "grasp confirmed" in entry["message"].lower():
                entry["phase"] = "grasp_confirmed"
            task_feedback.update(entry)
            if not task_feedback["history"] or task_feedback["history"][-1] != entry:
                task_feedback["history"].append(entry)
                task_events.append(entry)
        sent = client.send_goal_async(goal, feedback_callback=on_feedback)
        executor.spin_until_future_complete(sent, timeout_sec=15.0)
        if not sent.done():
            raise TimeoutError("task goal response timed out")
        handle = sent.result()
        if not handle.accepted:
            raise RuntimeError("robot_runtime rejected the pick_place goal")
        task_started = time.monotonic()
        task_started_monotonic_ns = time.monotonic_ns()
        cancel_timing["timeout_deadline_monotonic_ns"] = (
            task_started_monotonic_ns + args.timeout_ms * 1_000_000)
        cancel_timing["stop_trigger_monotonic_ns"] = cancel_timing["timeout_deadline_monotonic_ns"]
        cancel_timing["stop_trigger_sim_time_ns"] = None
        cancel_requested_at = None
        cancel_accepted = None
        feedback_dropped_after_release = False
        feedback_drop_attempted = False
        object_drop_command = None
        pause_result = []
        if args.pause_after_ms:
            pause_deadline = task_started + args.pause_after_ms / 1000.0
            while time.monotonic() < pause_deadline:
                executor.spin_once(timeout_sec=min(0.05, pause_deadline-time.monotonic()))
            for paused in (True, False):
                command = ["gz", "service", "-s", "/world/panda_pick_place/control",
                           "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
                           "--timeout", "3000", "--req", f"pause: {'true' if paused else 'false'}"]
                service_result = subprocess.run(command, cwd=root, env=launch_env,
                                                text=True, capture_output=True, timeout=5.0)
                pause_result.append({"paused": paused, "returncode": service_result.returncode,
                                     "stdout": service_result.stdout.strip(),
                                     "stderr": service_result.stderr.strip()})
                if service_result.returncode != 0:
                    raise RuntimeError(f"Gazebo world {'pause' if paused else 'resume'} service failed")
                if paused:
                    wait_until = time.monotonic() + args.pause_duration_ms / 1000.0
                    while time.monotonic() < wait_until:
                        executor.spin_once(timeout_sec=min(0.05, wait_until-time.monotonic()))
        if args.reset_after_ms:
            reset_deadline = task_started + args.reset_after_ms / 1000.0
            while time.monotonic() < reset_deadline:
                executor.spin_once(timeout_sec=min(0.05, reset_deadline-time.monotonic()))
            reset_command = ["gz", "service", "-s", "/world/panda_pick_place/control",
                             "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
                             "--timeout", "3000", "--req", "reset: {all: true}"]
            reset_service = subprocess.run(reset_command, cwd=root, env=launch_env,
                                           text=True, capture_output=True, timeout=5.0)
            reset_result = {"returncode": reset_service.returncode,
                            "stdout": reset_service.stdout.strip(),
                            "stderr": reset_service.stderr.strip()}
            if reset_service.returncode != 0:
                raise RuntimeError("Gazebo world reset service failed")
        if args.cancel_after_ms:
            cancel_deadline = task_started + args.cancel_after_ms / 1000.0
            while time.monotonic() < cancel_deadline:
                executor.spin_once(timeout_sec=min(0.05, cancel_deadline-time.monotonic()))
            cancel_requested_at = time.monotonic()
            cancel_timing["requested_monotonic_ns"] = time.monotonic_ns()
            cancel_timing["requested_sim_time_ns"] = node.get_clock().now().nanoseconds
            cancel_timing["stop_trigger_monotonic_ns"] = cancel_timing["requested_monotonic_ns"]
            cancel_timing["stop_trigger_sim_time_ns"] = cancel_timing["requested_sim_time_ns"]
            cancel_future = handle.cancel_goal_async()
            executor.spin_until_future_complete(cancel_future, timeout_sec=8.0)
            cancel_accepted = bool(cancel_future.done() and cancel_future.result().goals_canceling)
        result_future = handle.get_result_async()
        result_deadline = time.monotonic() + args.timeout_ms / 1000.0 + 20.0
        while not result_future.done() and time.monotonic() < result_deadline:
            executor.spin_once(timeout_sec=min(0.1, max(0.0, result_deadline-time.monotonic())))
            release_seen = any(event.get("phase") == "release_confirmed" for event in task_events)
            grasp_seen = any(event.get("phase") == "grasp_confirmed" for event in task_events)
            if args.drop_contact_after_release and not feedback_drop_attempted and release_seen:
                feedback_drop_attempted = True
                feedback_started = time.monotonic()
                request = SetParameters.Request()
                request.parameters = [Parameter("publish_grasp_feedback", value=False).to_parameter_msg()]
                future = feedback_parameter_client.call_async(request)
                executor.spin_until_future_complete(future, timeout_sec=2.0)
                response = future.result() if future.done() else None
                successful = bool(response and response.results and response.results[0].successful)
                feedback_dropped_after_release = successful
                task_events.append({"phase": "contact_feedback_interrupted_after_release",
                    "sim_time_ns": node.get_clock().now().nanoseconds,
                    "receipt_monotonic_ns": time.monotonic_ns(),
                    "returncode": 0 if successful else 1,
                    "elapsed_wall_s": time.monotonic()-feedback_started,
                    "successful": successful,
                    "reason": "parameter response timed out or was rejected" if not successful else ""})
            if (args.disturb_object_before_verification and not verification_fault_started and
                    task_feedback.get("active_skill") == "verify_placement"):
                verification_fault_started = True
                request = SetEntityPose.Request()
                request.entity.name = "workpiece"
                request.entity.type = Entity.MODEL
                request.pose = Pose()
                request.pose.position.x = 0.80
                request.pose.position.y = -0.18
                request.pose.position.z = 0.40
                request.pose.orientation.w = 1.0
                fault_started = time.monotonic()
                verification_fault_future = set_pose_client.call_async(request)
                executor.spin_until_future_complete(verification_fault_future, timeout_sec=2.0)
                response = verification_fault_future.result() if verification_fault_future.done() else None
                successful = bool(response and response.success)
                task_events.append({"phase": "off_target_pose_injected_before_verification",
                    "sim_time_ns": node.get_clock().now().nanoseconds,
                    "receipt_monotonic_ns": time.monotonic_ns(),
                    "returncode": 0 if successful else 1,
                    "elapsed_wall_s": time.monotonic()-fault_started,
                    "requested_pose_m": [0.80, -0.18, 0.40],
                    "successful": successful})
            if args.drop_object_after_grasp and object_drop_command is None and grasp_seen:
                command_text = "{command: {name: [panda_finger_joint1], position: [0.04], effort: [20.0]}}"
                drop_result = subprocess.run(
                    ["ros2", "action", "send_goal", "/panda_hand_controller/gripper_cmd",
                     "control_msgs/action/ParallelGripperCommand", command_text],
                    cwd=root, env=launch_env, text=True, capture_output=True, timeout=12.0)
                object_drop_command = {"returncode": drop_result.returncode,
                                       "stdout": drop_result.stdout.strip(),
                                       "stderr": drop_result.stderr.strip()}
                task_events.append({"phase": "external_gripper_open_during_lift",
                    "sim_time_ns": node.get_clock().now().nanoseconds,
                    "receipt_monotonic_ns": time.monotonic_ns(), **object_drop_command})
        if not result_future.done():
            raise TimeoutError("pick_place result did not arrive before the observation deadline")
        wrapped = result_future.result()
        result_received_monotonic_ns = time.monotonic_ns()
        cancel_timing["result_received_monotonic_ns"] = result_received_monotonic_ns
        if cancel_requested_at is None and wrapped.result.status != "timed_out":
            cancel_timing["stop_trigger_monotonic_ns"] = result_received_monotonic_ns
            cancel_timing["stop_trigger_sim_time_ns"] = node.get_clock().now().nanoseconds
        settle_deadline = time.monotonic() + 0.5
        while time.monotonic() < settle_deadline:
            executor.spin_once(timeout_sec=0.05)
        hand_transform = None
        try:
            transform = tf_buffer.lookup_transform("panda_link0", "panda_hand", Time())
            hand_transform = {
                "stamp_ns": transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec,
                "xyz": [transform.transform.translation.x, transform.transform.translation.y,
                        transform.transform.translation.z],
                "quaternion_xyzw": [transform.transform.rotation.x, transform.transform.rotation.y,
                                     transform.transform.rotation.z, transform.transform.rotation.w],
            }
        except Exception as error:
            hand_transform = {"error": f"{type(error).__name__}: {error}"}
        execution_record = None
        execution_client = node.create_client(GetExecution, "/get_execution")
        if execution_client.wait_for_service(timeout_sec=3.0):
            execution_request = GetExecution.Request()
            execution_request.execution_id = ""
            execution_future = execution_client.call_async(execution_request)
            executor.spin_until_future_complete(execution_future, timeout_sec=3.0)
            if execution_future.done():
                response = execution_future.result()
                snapshot = response.record.snapshot if response.found else None
                execution_record = {
                    "found": response.found,
                    "lookup_status": response.lookup_status,
                    "status": response.record.status if response.found else None,
                    "error_code": response.record.error_code if response.found else None,
                    "stop_confirmed": snapshot.stop_confirmed if snapshot is not None else None,
                    "resources_empty": snapshot.resources_empty if snapshot is not None else None,
                    "fault_latched": snapshot.fault_latched if snapshot is not None else None,
                    "resource_leases": ([{"resource_id": lease.resource_id,
                                           "owner_request_id": lease.owner_request_id}
                                          for lease in snapshot.resource_leases]
                                         if snapshot is not None else None),
                    "steps": ([{"skill_id": step.step.skill_id, "status": step.status,
                                "error_code": step.error_code,
                                "has_verification": step.has_verification}
                               for step in response.record.steps] if response.found else None),
                }
        runtime_state = None
        state_client = node.create_client(GetRuntimeState, "/get_runtime_state")
        if state_client.wait_for_service(timeout_sec=3.0):
            state_future = state_client.call_async(GetRuntimeState.Request())
            executor.spin_until_future_complete(state_future, timeout_sec=3.0)
            if state_future.done():
                response = state_future.result()
                runtime_state = {
                    "busy": response.busy,
                    "active_execution_id": response.active_execution_id,
                    "resources_empty": response.snapshot.resources_empty,
                    "fault_latched": response.snapshot.fault_latched,
                    "resource_leases": [{"resource_id": lease.resource_id,
                                         "owner_request_id": lease.owner_request_id}
                                        for lease in response.snapshot.resource_leases],
                }
        outcome = {
            "success": bool(wrapped.result.success),
            "status": wrapped.result.status,
            "error_code": wrapped.result.error_code,
            "message": wrapped.result.message,
            "object_id": "workpiece", "target_id": "tray",
            "goal_status": int(wrapped.status),
            "launch_log": str(args.launch_log),
            "pose_samples": pose_samples,
            "pose_history": pose_history,
            "contact_samples": contact_samples,
            "task_events": task_events,
            "contact_state": contact_state,
            "arm_state": arm_state,
            "finger_joint_extrema_rad": finger_extrema,
            "post_result_hand_tf": hand_transform,
            "task_feedback": task_feedback,
            "execution_record": execution_record,
            "runtime_state": runtime_state,
            "gazebo_topics": gazebo_topics,
            "action_endpoints": action_endpoints,
            "seed": args.seed,
            "run_id": run_id,
            "code_commit": code_commit,
            "final_sim_time_ns": node.get_clock().now().nanoseconds,
            "final_receipt_monotonic_ns": time.monotonic_ns(),
            "final_epoch": sample_guard.epoch,
            "scene_ready": scene_ready_state["ready"],
            "object_offset_xy_m": [args.object_dx, args.object_dy],
            "injected_placement_offset_y_m": args.placement_offset_y,
            "injected_grasp_miss_x_m": args.grasp_miss_x,
            "task_elapsed_wall_s": time.monotonic()-task_started,
            "cancel_requested": cancel_requested_at is not None,
            "cancel_accepted": cancel_accepted,
            "cancel_requested_monotonic_ns": cancel_timing["requested_monotonic_ns"],
            "timeout_deadline_monotonic_ns": cancel_timing["timeout_deadline_monotonic_ns"],
            "cancel_result_received_monotonic_ns": cancel_timing["result_received_monotonic_ns"],
            "cancel_stop_confirmed_monotonic_ns": cancel_timing["stop_confirmed_monotonic_ns"],
            "cancel_stop_confirmed_source_ns": cancel_timing["stop_confirmed_source_ns"],
            "cancel_to_result_s": None if cancel_requested_at is None else
                (result_received_monotonic_ns-cancel_timing["requested_monotonic_ns"])*1e-9,
            "cancel_to_stop_confirmed_s": None if cancel_timing["stop_confirmed_monotonic_ns"] is None or
                cancel_requested_at is None else
                (cancel_timing["stop_confirmed_monotonic_ns"]-cancel_timing["requested_monotonic_ns"])*1e-9,
            "timeout_to_stop_confirmed_s": None if cancel_timing["stop_confirmed_monotonic_ns"] is None or
                args.cancel_after_ms else
                (cancel_timing["stop_confirmed_monotonic_ns"]-
                 cancel_timing["timeout_deadline_monotonic_ns"])*1e-9,
            "pause_control": pause_result,
            "reset_control": reset_result,
            "grasp_feedback_enabled": not args.drop_grasp_feedback,
            "contact_feedback_dropped_after_release": feedback_dropped_after_release,
            "object_drop_command": object_drop_command,
        }
        emit_result(outcome)
        client.destroy()
        if outcome["success"]:
            return 0
        return 1
    except Exception as error:
        outcome.update({"error_code": type(error).__name__, "message": str(error),
                        "status": "runner_error",
                        "success": False,
                        "launch_log": str(args.launch_log), "pose_samples": pose_samples,
                        "pose_history": pose_history, "contact_samples": contact_samples,
                        "task_events": task_events,
                        "contact_state": contact_state, "arm_state": arm_state,
                        "finger_joint_extrema_rad": finger_extrema,
                        "task_feedback": task_feedback,
                        "gazebo_topics": gazebo_topics, "action_endpoints": action_endpoints,
                        "seed": args.seed, "object_offset_xy_m": [args.object_dx, args.object_dy],
                        "run_id": run_id, "code_commit": code_commit,
                        "final_sim_time_ns": 0 if node is None else node.get_clock().now().nanoseconds,
                        "final_receipt_monotonic_ns": time.monotonic_ns(),
                        "final_epoch": sample_guard.epoch,
                        "scene_ready": scene_ready_state["ready"],
                        "cancel_requested_monotonic_ns": cancel_timing["requested_monotonic_ns"],
                        "cancel_result_received_monotonic_ns": cancel_timing["result_received_monotonic_ns"],
                        "cancel_stop_confirmed_monotonic_ns": cancel_timing["stop_confirmed_monotonic_ns"],
                        "cancel_stop_confirmed_source_ns": cancel_timing["stop_confirmed_source_ns"],
                        "injected_placement_offset_y_m": args.placement_offset_y,
                        "injected_grasp_miss_x_m": args.grasp_miss_x,
                        "reset_control": reset_result,
                        "grasp_feedback_enabled": not args.drop_grasp_feedback})
        emit_result(outcome)
        return 2
    finally:
        if executor is not None and node is not None:
            executor.remove_node(node)
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok(context=context):
            rclpy.shutdown(context=context)
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5.0)
        launch_output.close()
        if temp_world_dir is not None:
            temp_world_dir.cleanup()


if __name__ == "__main__":
    sys.exit(main())
