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
import xml.etree.ElementTree as ET
from pathlib import Path

from control_msgs.action import ParallelGripperCommand
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time
from robot_interfaces.action import ExecuteTask
from robot_interfaces.msg import GraspContact
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


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
    parser.add_argument("--result-json", type=Path, default=Path("docs/reports/phase2-gazebo-result.json"))
    parser.add_argument("--probe-gripper", action="store_true",
                        help="command the actuated Panda finger and report the measured joint state")
    parser.add_argument("--launch-log", type=Path, default=Path("docs/reports/phase2-gazebo-launch.log"))
    args = parser.parse_args()
    if not 1000 <= args.timeout_ms <= 600000:
        parser.error("--timeout-ms must be between 1000 and 600000")

    root = Path(__file__).resolve().parents[1]
    args.launch_log.parent.mkdir(parents=True, exist_ok=True)
    args.result_json.parent.mkdir(parents=True, exist_ok=True)
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
    process = subprocess.Popen(
        ["ros2", "launch", "robot_panda_gz_sim", "panda_pick_place.launch.py",
         f"seed:={args.seed}", f"world_file:={world_path}",
         f"config_file:={config_path}",
         f"publish_grasp_feedback:={'false' if args.drop_grasp_feedback else 'true'}"],
        cwd=root, stdout=launch_output, stderr=subprocess.STDOUT,
        start_new_session=True, env=launch_env)
    context = Context()
    executor = None
    node = None
    tf_listener = None
    tf_buffer = None
    outcome = {"success": False, "status": "not_started", "error_code": "", "message": ""}
    pose_samples = {}
    contact_state = {"messages": 0, "stamp_ns": 0, "contacts": [], "seen": [],
                     "table_messages": 0, "table_collisions": [], "table_seen": [],
                     "grasp_feedback_messages": 0, "grasp_feedback": None,
                     "stamped_grasp_messages": 0, "stamped_grasp_source_ns": 0,
                     "stamped_grasp_detected": None}
    arm_state = {"hand_tf": None, "joint_state": None}
    finger_extrema = {"panda_finger_joint1": [None, None], "panda_finger_joint2": [None, None]}
    gazebo_topics = []
    action_endpoints = []
    task_feedback = {"active_skill": "", "status": "", "message": "", "history": []}
    reset_result = None
    try:
        rclpy.init(context=context)
        node = rclpy.create_node("gazebo_pick_place_test", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        tf_buffer = Buffer(node=node)
        tf_listener = TransformListener(tf_buffer, node, spin_thread=False)
        def on_poses(message):
            for transform in message.transforms:
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
                if child in ("workpiece", "tray") or child.endswith("::workpiece") or child.endswith("::tray"):
                    pose_samples[child] = {
                        "parent": transform.header.frame_id,
                        "stamp_ns": transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec,
                        "xyz": [transform.transform.translation.x, transform.transform.translation.y,
                                transform.transform.translation.z],
                    }
        pose_subscriptions = [
            node.create_subscription(TFMessage, "/panda_gz/workpiece/pose", on_poses, 10),
            node.create_subscription(TFMessage, "/panda_gz/tray/pose", on_poses, 10),
        ]
        def on_joint_state(message):
            arm_state["joint_state"] = {
                "stamp_ns": message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec,
                "names": list(message.name), "positions": list(message.position),
                "velocities": list(message.velocity),
            }
            for name, extrema in finger_extrema.items():
                if name in message.name:
                    value = message.position[message.name.index(name)]
                    extrema[0] = value if extrema[0] is None else min(extrema[0], value)
                    extrema[1] = value if extrema[1] is None else max(extrema[1], value)
        pose_subscriptions.append(node.create_subscription(JointState, "/joint_states", on_joint_state, 20))
        def on_contacts(message):
            contact_state["messages"] += 1
            contact_state["stamp_ns"] = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            contact_state["contacts"] = [
                [contact.collision1.name, contact.collision2.name] for contact in message.contacts
            ]
            contact_state["seen"] = sorted({tuple(pair) for pair in contact_state["seen"] + contact_state["contacts"]})
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
            contact_state["stamped_grasp_detected"] = message.detected
        pose_subscriptions.append(node.create_subscription(
            GraspContact, "/panda/grasp_contact_stamped", on_stamped_grasp_feedback, 10))
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
                     "message": message.feedback.message}
            task_feedback.update(entry)
            if not task_feedback["history"] or task_feedback["history"][-1] != entry:
                task_feedback["history"].append(entry)
        sent = client.send_goal_async(goal, feedback_callback=on_feedback)
        executor.spin_until_future_complete(sent, timeout_sec=15.0)
        if not sent.done():
            raise TimeoutError("task goal response timed out")
        handle = sent.result()
        if not handle.accepted:
            raise RuntimeError("robot_runtime rejected the pick_place goal")
        task_started = time.monotonic()
        cancel_requested_at = None
        cancel_accepted = None
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
            cancel_future = handle.cancel_goal_async()
            executor.spin_until_future_complete(cancel_future, timeout_sec=8.0)
            cancel_accepted = bool(cancel_future.done() and cancel_future.result().goals_canceling)
        result_future = handle.get_result_async()
        executor.spin_until_future_complete(result_future, timeout_sec=args.timeout_ms / 1000.0 + 20.0)
        if not result_future.done():
            raise TimeoutError("pick_place result did not arrive before the observation deadline")
        wrapped = result_future.result()
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
        outcome = {
            "success": bool(wrapped.result.success),
            "status": wrapped.result.status,
            "error_code": wrapped.result.error_code,
            "message": wrapped.result.message,
            "goal_status": int(wrapped.status),
            "launch_log": str(args.launch_log),
            "pose_samples": pose_samples,
            "contact_state": contact_state,
            "arm_state": arm_state,
            "finger_joint_extrema_rad": finger_extrema,
            "post_result_hand_tf": hand_transform,
            "task_feedback": task_feedback,
            "gazebo_topics": gazebo_topics,
            "action_endpoints": action_endpoints,
            "seed": args.seed,
            "object_offset_xy_m": [args.object_dx, args.object_dy],
            "injected_placement_offset_y_m": args.placement_offset_y,
            "injected_grasp_miss_x_m": args.grasp_miss_x,
            "task_elapsed_wall_s": time.monotonic()-task_started,
            "cancel_requested": cancel_requested_at is not None,
            "cancel_accepted": cancel_accepted,
            "cancel_to_result_s": None if cancel_requested_at is None else time.monotonic()-cancel_requested_at,
            "pause_control": pause_result,
            "reset_control": reset_result,
            "grasp_feedback_enabled": not args.drop_grasp_feedback,
        }
        emit_result(outcome)
        client.destroy()
        if outcome["success"]:
            return 0
        return 1
    except Exception as error:
        outcome.update({"error_code": type(error).__name__, "message": str(error),
                        "launch_log": str(args.launch_log), "pose_samples": pose_samples,
                        "contact_state": contact_state, "arm_state": arm_state,
                        "finger_joint_extrema_rad": finger_extrema,
                        "task_feedback": task_feedback,
                        "gazebo_topics": gazebo_topics, "action_endpoints": action_endpoints,
                        "seed": args.seed, "object_offset_xy_m": [args.object_dx, args.object_dy],
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
