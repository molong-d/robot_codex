"""Exercise actual MoveIt, controllers, TF and our adapters with GenericSystem."""
import os
import signal
import subprocess
import tempfile
import time
import math
import unittest
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, ParallelGripperCommand
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from robot_interfaces.action import ExecuteTask, ExecutePlan
from robot_interfaces.msg import SkillStep
from robot_interfaces.srv import GetExecution, GetRuntimeState


class PandaTests(unittest.TestCase):
    native_poses = False
    implementation = "standard"

    @classmethod
    def setUpClass(cls):
        domain = (int(os.environ.get("ROS_DOMAIN_ID", "42")) + (110 if cls.native_poses else 0)) % 201
        rclpy.init(domain_id=domain)
        cls.log = tempfile.TemporaryFile(mode="w+")
        command = ["ros2", "launch", "robot_panda_demo", "panda.launch.py"]
        if cls.native_poses:
            config = Path(__file__).resolve().parents[1]/"src/robot_panda_demo/config/runtime_native_poses.yaml"
            command.append(f"config_file:={config}")
        cls.process = subprocess.Popen(command, stdout=cls.log, stderr=subprocess.STDOUT, start_new_session=True,
                                       env=dict(os.environ, ROS_DOMAIN_ID=str(domain)))
        cls.node = rclpy.create_node("panda_adapter_tests")
        cls.client = ActionClient(cls.node, ExecuteTask, "execute_task")
        cls.plan_client = ActionClient(cls.node, ExecutePlan, "execute_plan")
        cls.arm = ActionClient(cls.node, MoveGroup, "move_action")
        cls.trajectory = ActionClient(cls.node, FollowJointTrajectory, "panda_arm_controller/follow_joint_trajectory")
        cls.hand = ActionClient(cls.node, ParallelGripperCommand, "panda_hand_controller/gripper_cmd")
        cls.states = []
        cls.subscription = cls.node.create_subscription(JointState, "joint_states", lambda s: cls.states.append(s), 10)

    @classmethod
    def tearDownClass(cls):
        cls.client.destroy(); cls.plan_client.destroy(); cls.arm.destroy(); cls.trajectory.destroy(); cls.hand.destroy(); cls.node.destroy_node()
        forced_kill = False
        if cls.process.poll() is None:
            cls.process.send_signal(signal.SIGINT)  # launch propagates once to its children
            try:
                cls.process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                forced_kill = True
                os.killpg(cls.process.pid, signal.SIGKILL); cls.process.wait(timeout=5)
        cls.log.seek(0); output = cls.log.read(); print(output); cls.log.close()
        rclpy.shutdown()
        if forced_kill or "Segmentation fault" in output or "process has died" in output:
            raise AssertionError("Panda stack did not exit cleanly; inspect subprocess logs")

    def wait(self, future, timeout=100):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertTrue(future.done(), "future timed out")
        return future.result()

    def ready(self):
        self.assertTrue(self.arm.wait_for_server(timeout_sec=60), "MoveIt server unavailable")
        self.assertTrue(self.trajectory.wait_for_server(timeout_sec=60), "arm trajectory controller unavailable")
        self.assertTrue(self.hand.wait_for_server(timeout_sec=60), "gripper controller unavailable")
        self.assertTrue(self.client.wait_for_server(timeout_sec=30), "runtime unavailable")
        controllers = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        try:
            self.assertTrue(controllers.wait_for_service(timeout_sec=30), "controller manager unavailable")
            end = time.monotonic()+30
            while time.monotonic() < end:
                response = self.wait(controllers.call_async(ListControllers.Request()), timeout=5)
                active = {controller.name for controller in response.controller if controller.state == "active"}
                if {"joint_state_broadcaster", "panda_arm_controller", "panda_hand_controller"} <= active:
                    break
                rclpy.spin_once(self.node, timeout_sec=0.05)
            else:
                self.fail("Panda controllers did not become active")
        finally:
            self.node.destroy_client(controllers)
        # Action servers may exist before the state broadcaster is activated.
        # Wait for actual fresh stationary samples, never dispatch/retry a probe goal.
        required = [f"panda_joint{i}" for i in range(1, 8)]+["panda_finger_joint1"]
        end = time.monotonic()+30
        first_stamp = last_stamp = None
        samples = 0
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if not self.states:
                continue
            state = self.states[-1]
            stamp = state.header.stamp.sec*1_000_000_000+state.header.stamp.nanosec
            age = (self.node.get_clock().now().nanoseconds-stamp)/1_000_000_000
            valid = stamp > 0 and 0 <= age <= 0.5
            for name in required:
                if name not in state.name:
                    valid = False; break
                i = state.name.index(name)
                if i >= len(state.position) or i >= len(state.velocity) or not math.isfinite(state.position[i]) or \
                        not math.isfinite(state.velocity[i]) or abs(state.velocity[i]) > 0.001:
                    valid = False; break
            if not valid:
                first_stamp = last_stamp = None; samples = 0
                continue
            if last_stamp is None or stamp > last_stamp:
                if first_stamp is None:
                    first_stamp = stamp
                last_stamp = stamp; samples += 1
            if samples >= 3 and stamp-first_stamp >= 200_000_000:
                return
        self.fail("fresh stationary arm/gripper state broadcaster samples unavailable")

    def test_01_repeatable_pick_place_with_actual_ros_stack(self):
        self.ready()
        for _ in range(2):
            goal = ExecuteTask.Goal(task_name="pick_place", object_id="workpiece", target_id="tray", timeout_ms=90000)
            handle = self.wait(self.client.send_goal_async(goal))
            self.assertTrue(handle.accepted)
            result = self.wait(handle.get_result_async())
            self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED, result.result.message)
            self.assertTrue(result.result.success, result.result.error_code)
        self.assertTrue(any("panda_joint1" in s.name for s in self.states), "no controller feedback")

    def test_02_task_timeout_stops_and_releases_for_next_task(self):
        self.ready()
        goal = ExecuteTask.Goal(task_name="pick_place", object_id="workpiece", target_id="tray", timeout_ms=100)
        handle = self.wait(self.client.send_goal_async(goal))
        self.assertTrue(handle.accepted)
        result = self.wait(handle.get_result_async())
        self.assertEqual(result.result.status, "timed_out", result.result.message)
        next_goal = ExecuteTask.Goal(task_name="pick_place", object_id="workpiece", target_id="tray", timeout_ms=90000)
        next_handle = self.wait(self.client.send_goal_async(next_goal))
        self.assertTrue(next_handle.accepted, "resource ownership not released after measured stop")
        self.assertTrue(self.wait(next_handle.get_result_async()).result.success)

    def test_03_second_configured_object_target_and_alias(self):
        self.ready()
        goal = ExecuteTask.Goal(task_name="transfer_two", object_id="workpiece_two", target_id="tray_two", timeout_ms=90000)
        handle = self.wait(self.client.send_goal_async(goal))
        self.assertTrue(handle.accepted)
        result = self.wait(handle.get_result_async())
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED, result.result.message)
        self.assertTrue(result.result.success, result.result.error_code)

    def test_04_structured_plan_uses_actual_moveit_and_controllers(self):
        self.ready()
        self.assertTrue(self.plan_client.wait_for_server(timeout_sec=10))
        steps = [SkillStep(skill_id=skill, implementation_id=self.implementation, argument_names=list(args), argument_values=list(args.values()))
                 for skill, args in [("locate_object", {"object": "workpiece"}), ("pick_object", {"object": "workpiece"}),
                                     ("locate_object", {"object": "tray"}),
                                     ("place_object", {"object": "workpiece", "target": "tray"})]]
        handle = self.wait(self.plan_client.send_goal_async(ExecutePlan.Goal(schema_version=1, steps=steps, timeout_ms=90000)))
        self.assertTrue(handle.accepted)
        result = self.wait(handle.get_result_async())
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED, result.result.message)
        self.assertTrue(result.result.success, result.result.error_code)
        self.assertEqual(result.result.completed_steps, 4)
        client = self.node.create_client(GetExecution, "get_execution")
        state_client = self.node.create_client(GetRuntimeState, "get_runtime_state")
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=5))
            self.assertTrue(state_client.wait_for_service(timeout_sec=5))
            record = self.wait(client.call_async(GetExecution.Request(execution_id=bytes(handle.goal_id.uuid).hex())))
            state = self.wait(state_client.call_async(GetRuntimeState.Request()))
            self.assertTrue(record.found)
            self.assertEqual(record.record.completed_steps, 4)
            self.assertEqual(record.record.status, "succeeded")
            self.assertTrue(record.record.snapshot.stop_confirmed)
            self.assertFalse(record.record.snapshot.resource_leases)
            self.assertEqual(state.backend, "panda_ros")
            self.assertTrue(state.simulation_only)
            self.assertEqual(state.runtime_id, record.runtime_id)
        finally:
            self.node.destroy_client(client); self.node.destroy_client(state_client)

    def test_05_verified_template_marks_synthetic_outcome_proofs(self):
        self.ready()
        handle = self.wait(self.client.send_goal_async(ExecuteTask.Goal(
            task_name="verified_pick_place", object_id="workpiece", target_id="tray", timeout_ms=90000)))
        self.assertTrue(handle.accepted)
        result = self.wait(handle.get_result_async())
        self.assertTrue(result.result.success, result.result.error_code+": "+result.result.message)
        client = self.node.create_client(GetExecution, "get_execution")
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=5))
            record = self.wait(client.call_async(GetExecution.Request(execution_id=bytes(handle.goal_id.uuid).hex())))
            self.assertEqual(record.schema_version, 2)
            self.assertEqual(record.record.completed_steps, 6)
            if self.native_poses:
                for index in (1, 4):
                    self.assertEqual(record.record.steps[index].step.implementation_id, "pose_resolved")
                    self.assertIn("calibration=demo_camera_v1", record.record.steps[index].transitions[0].message)
            for index in (2, 5):
                self.assertTrue(record.record.steps[index].has_verification)
                proof = record.record.steps[index].verification
                self.assertTrue(proof.synthetic)
                self.assertGreaterEqual(proof.samples, 3)
                self.assertGreaterEqual(proof.stable_ms, 100)
                self.assertEqual(proof.source, "demo_outcome")
                self.assertEqual(proof.entity_id, "workpiece")
                self.assertEqual(proof.target_id, "" if index == 2 else "tray")
            candidates = {entry.entity_id: entry.location_id for entry in record.record.snapshot.placement_candidates}
            locations = {entry.entity_id: entry.location_id for entry in record.record.snapshot.known_locations}
            self.assertNotIn("workpiece", candidates)
            self.assertEqual(locations.get("workpiece"), "tray")
            self.assertTrue(record.record.snapshot.stop_confirmed)
        finally:
            self.node.destroy_client(client)


class NativePandaTests(PandaTests):
    native_poses = True
    implementation = "pose_resolved"

    def test_06_native_observation_is_not_a_precomputed_motion_target(self):
        self.ready()
        handle = self.wait(self.client.send_goal_async(ExecuteTask.Goal(
            task_name="inspect_object", object_id="workpiece", target_id="", timeout_ms=5000)))
        self.assertTrue(handle.accepted)
        self.assertTrue(self.wait(handle.get_result_async()).result.success)
        client = self.node.create_client(GetRuntimeState, "get_runtime_state")
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=5))
            state = self.wait(client.call_async(GetRuntimeState.Request()))
            observation = next(o for o in state.snapshot.observations if o.entity_id == "workpiece")
            self.assertEqual((observation.frame_id, observation.pose_meaning), ("demo_camera", "object_pose"))
            self.assertEqual(observation.source, "configured_native_demo")
            self.assertTrue(observation.synthetic)
            self.assertAlmostEqual(observation.pose[0], 0.15)
            self.assertAlmostEqual(observation.pose[2], 0.32)
        finally:
            self.node.destroy_client(client)


if __name__ == "__main__":
    unittest.main(verbosity=2)
