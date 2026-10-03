"""ROS 2 integration tests against the actual BehaviorTree.CPP runtime, no hardware."""
import os
import signal
import subprocess
import tempfile
import time
import unittest
import copy
import json
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from robot_interfaces.action import ExecuteTask, ExecutePlan
from robot_interfaces.msg import SkillStep
from robot_interfaces.srv import GetCatalog, GetExecution, GetRuntimeState


class RuntimeTests(unittest.TestCase):
    domain_sequence = 0

    def setUp(self):
        # Repeated action servers must not share stale DDS discovery/response
        # endpoints. Give each test a fresh context, executor and ROS domain.
        RuntimeTests.domain_sequence += 1
        self.domain_id = (int(os.environ.get("ROS_DOMAIN_ID", "42")) + RuntimeTests.domain_sequence) % 101
        self.runtime_env = dict(os.environ, ROS_DOMAIN_ID=str(self.domain_id))
        self.context = Context()
        rclpy.init(context=self.context, domain_id=self.domain_id)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.process = None
        self.log = tempfile.TemporaryFile(mode="w+")
        self.node = rclpy.create_node(f"robot_runtime_test_{self.domain_id}", context=self.context)
        self.executor.add_node(self.node)
        self.client = ActionClient(self.node, ExecuteTask, "execute_task")
        self.plan_client = ActionClient(self.node, ExecutePlan, "execute_plan")

    def tearDown(self):
        self.client.destroy()
        self.plan_client.destroy()
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown(context=self.context)
        if self.process is not None:
            if self.process.poll() is None:
                os.killpg(self.process.pid, signal.SIGINT)
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGKILL)
                    self.process.wait(timeout=5)
            self.log.seek(0)
            print(self.log.read())
        self.log.close()

    def start(self, *parameters, config=None):
        command = ["ros2", "run", "robot_bt_runtime", "robot_runtime", "--ros-args"]
        if config is not None:
            command += ["--params-file", str(config)]
        for parameter in parameters:
            command += ["-p", parameter]
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT,
                                        start_new_session=True, env=self.runtime_env)
        self.assertTrue(self.client.wait_for_server(timeout_sec=15), "runtime did not start")

    def wait(self, future, timeout=10):
        self.executor.spin_until_future_complete(future, timeout_sec=timeout)
        self.assertTrue(future.done(), "future timed out")
        return future.result()

    def send(self, timeout=5000, task="pick_place", feedback=None, object_id="workpiece", target_id="tray"):
        goal = ExecuteTask.Goal(task_name=task, object_id=object_id, target_id=target_id, timeout_ms=timeout)
        return self.wait(self.client.send_goal_async(goal, feedback_callback=feedback))

    def catalog(self):
        client = self.node.create_client(GetCatalog, "get_catalog")
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=5))
            return self.wait(client.call_async(GetCatalog.Request()))
        finally:
            self.node.destroy_client(client)

    def query(self, service, endpoint, **values):
        client = self.node.create_client(service, endpoint)
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=5))
            return self.wait(client.call_async(service.Request(**values)))
        finally:
            self.node.destroy_client(client)

    def state(self):
        return self.query(GetRuntimeState, "get_runtime_state")

    def record(self, handle=None, execution_id=None):
        if execution_id is None:
            execution_id = bytes(handle.goal_id.uuid).hex() if handle is not None else ""
        return self.query(GetExecution, "get_execution", execution_id=execution_id)

    def wait_for_pick(self, feedback):
        end = time.monotonic()+5
        while not any(f.active_skill == "pick_object" and f.status == "running" for f in feedback):
            self.assertLess(time.monotonic(), end, "no running pick feedback")
            self.executor.spin_once(timeout_sec=0.02)

    def plan_steps(self, object_id="workpiece", target_id="tray"):
        return [SkillStep(skill_id=skill, implementation_id="standard", argument_names=list(args), argument_values=list(args.values()))
                for skill, args in [("locate_object", {"object": object_id}), ("pick_object", {"object": object_id}),
                                    ("locate_object", {"object": target_id}),
                                    ("place_object", {"object": object_id, "target": target_id})]]

    def send_plan(self, steps=None, version=1, timeout=5000, feedback=None):
        self.assertTrue(self.plan_client.wait_for_server(timeout_sec=5))
        goal = ExecutePlan.Goal(schema_version=version, steps=self.plan_steps() if steps is None else steps, timeout_ms=timeout)
        return self.wait(self.plan_client.send_goal_async(goal, feedback_callback=feedback))

    def test_diagnostics_idle_and_rejected_goals_have_no_record(self):
        self.start()
        state = self.state()
        self.assertEqual(state.schema_version, 2)
        self.assertEqual(len(state.runtime_id), 32)
        self.assertFalse(state.busy)
        self.assertTrue(state.simulation_only)
        self.assertTrue(state.snapshot.stop_confirmed)
        self.assertTrue(state.snapshot.resources_empty)
        self.assertFalse(self.record().found)
        self.assertEqual(self.record().lookup_status, "no_records")
        self.assertFalse(self.send(task="unknown").accepted)
        self.assertFalse(self.record().found)
        self.assertFalse(self.record(execution_id="0"*32).found)
        self.assertEqual(self.record(execution_id="0"*32).lookup_status, "not_found")

    def test_task_record_and_world_snapshot_distinguish_inferred_placement(self):
        self.start()
        handle = self.send()
        self.assertTrue(self.wait(handle.get_result_async()).result.success)
        response = self.record(handle)
        self.assertTrue(response.found)
        self.assertEqual(response.lookup_status, "completed")
        record = response.record
        self.assertEqual(record.execution_id, bytes(handle.goal_id.uuid).hex())
        self.assertEqual((record.entrypoint, record.task_name), ("execute_task", "pick_place"))
        self.assertEqual((record.status, record.total_steps, record.completed_steps), ("succeeded", 4, 4))
        self.assertEqual([s.step.skill_id for s in record.steps], ["locate_object", "pick_object", "locate_object", "place_object"])
        self.assertTrue(record.snapshot.stop_confirmed)
        self.assertFalse(record.snapshot.resource_leases)
        self.assertFalse(record.snapshot.attached_object)
        self.assertFalse(record.snapshot.known_locations)
        self.assertEqual([(p.entity_id, p.location_id) for p in record.snapshot.placement_candidates], [("workpiece", "tray")])
        self.assertTrue(record.started_unix_ms > 0)
        # A later task changes current world state without changing the old record.
        inspect = self.send(task="inspect_object", target_id="")
        self.assertTrue(self.wait(inspect.get_result_async()).result.success)
        self.assertEqual(self.record(handle).record, record)
        observation = self.state().snapshot.observations[0]
        self.assertEqual(observation.source, "configured_demo")
        self.assertTrue(observation.synthetic)
        self.assertEqual(observation.quality, 1.0)
        self.assertEqual(observation.pose_meaning, "motion_target")
        self.assertTrue(observation.valid and observation.stamp_valid)
        self.assertEqual(observation.frame_id, "base_link")
        self.assertEqual(len(observation.pose), 7)

    def test_plan_diagnostics_track_ownership_and_confirmed_cancel(self):
        self.start("mock_action_ticks:=50", "mock_stop_ticks:=50")
        feedback = []
        handle = self.send_plan(feedback=lambda msg: feedback.append(msg.feedback))
        self.wait_for_pick(feedback)
        state = self.state()
        self.assertTrue(state.busy)
        self.assertEqual(state.active_execution_id, bytes(handle.goal_id.uuid).hex())
        self.assertFalse(state.snapshot.stop_confirmed)
        record = self.record(handle).record
        self.assertEqual(record.entrypoint, "execute_plan")
        self.assertEqual(record.steps[1].status, "running")
        self.assertEqual({lease.resource_id for lease in state.snapshot.resource_leases}, {"demo_arm", "demo_gripper"})
        self.assertTrue(all(lease.owner_request_id == record.steps[1].request_id for lease in state.snapshot.resource_leases))
        self.assertTrue(self.wait(handle.cancel_goal_async()).goals_canceling)
        stopping = self.record(handle).record
        self.assertEqual(stopping.status, "canceling")
        self.assertFalse(stopping.snapshot.stop_confirmed)
        self.assertTrue(stopping.snapshot.resource_leases)
        result = self.wait(handle.get_result_async())
        self.assertEqual(result.status, GoalStatus.STATUS_CANCELED)
        completed = self.record(handle).record
        self.assertEqual((completed.status, completed.completed_steps), ("canceled", 1))
        self.assertTrue(completed.snapshot.stop_confirmed)
        self.assertFalse(completed.snapshot.resource_leases)
        self.assertEqual([t.status for t in completed.steps[1].transitions], ["running", "canceling", "canceled"])
        self.assertTrue(self.wait(self.send_plan().get_result_async()).result.success)

    def test_stop_unconfirmed_record_keeps_fault_and_leases(self):
        self.start("mock_action_ticks:=50", "mock_stop_ticks:=100000", "stop_timeout_ms:=40")
        feedback = []
        handle = self.send(feedback=lambda msg: feedback.append(msg.feedback))
        self.wait_for_pick(feedback)
        self.assertTrue(self.wait(handle.cancel_goal_async()).goals_canceling)
        result = self.wait(handle.get_result_async())
        self.assertEqual(result.result.error_code, "STOP_UNCONFIRMED")
        record = self.record(handle).record
        self.assertEqual(record.status, "faulted")
        self.assertFalse(record.snapshot.stop_confirmed)
        self.assertTrue(record.snapshot.fault_latched)
        self.assertFalse(record.snapshot.resources_empty)
        self.assertEqual(len(record.snapshot.resource_leases), 2)
        self.assertEqual(record.steps[-1].status, "canceling")  # no fabricated skill completion
        state = self.state()
        self.assertFalse(state.busy)
        self.assertFalse(state.active_execution_id)
        self.assertTrue(state.snapshot.fault_latched)
        self.assertFalse(state.snapshot.stop_confirmed)
        self.assertEqual(len(state.snapshot.resource_leases), 2)
        self.assertFalse(self.send().accepted)
        self.assertFalse(self.send_plan().accepted)
        self.assertEqual(self.record(handle).record, record)  # queries do not unlock the fault

    def test_execution_history_capacity_and_export_example(self):
        self.start("execution_history_capacity:=2")
        handles = []
        for _ in range(3):
            handle = self.send(task="inspect_object", target_id="")
            self.assertTrue(self.wait(handle.get_result_async()).result.success)
            handles.append(handle)
        state = self.state()
        self.assertEqual(state.history_capacity, 2)
        self.assertEqual(state.evicted_records, 1)
        self.assertEqual(state.recent_execution_ids, [bytes(h.goal_id.uuid).hex() for h in reversed(handles[1:])])
        self.assertEqual(self.record(handles[0]).lookup_status, "not_found")
        self.assertEqual(self.record().record.execution_id, bytes(handles[-1].goal_id.uuid).hex())
        client = AsyncParameterClient(self.node, "robot_runtime")
        self.assertTrue(client.wait_for_services(timeout_sec=5))
        update = self.wait(client.set_parameters([Parameter("execution_history_capacity", value=8)]))
        self.assertFalse(update.results[0].successful)
        exporter = Path(__file__).resolve().parent / "inspect_runtime.py"
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)/"record.json"
            command = ["python3", str(exporter), "--output", str(output)]
            result = subprocess.run(command, capture_output=True, text=True, timeout=20, env=self.runtime_env)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            payload = json.loads(output.read_text())
            self.assertEqual(payload["execution"]["record"]["execution_id"], bytes(handles[-1].goal_id.uuid).hex())
            self.assertEqual(payload["runtime_state"]["runtime_id"], payload["execution"]["runtime_id"])
            original = output.read_text()
            self.assertNotEqual(subprocess.run(command, capture_output=True, timeout=20, env=self.runtime_env).returncode, 0)
            self.assertEqual(output.read_text(), original)  # no accidental overwrite

    def test_failed_records_match_action_outcomes(self):
        self.start("mock_motion_permitted:=false")
        handle = self.send()
        result = self.wait(handle.get_result_async()).result
        record = self.record(handle).record
        self.assertEqual((record.status, record.error_code), (result.status, result.error_code))
        self.assertEqual(record.completed_steps, 1)
        self.assertEqual(len(record.steps), 2)
        self.assertFalse(record.snapshot.resource_leases)
        self.assertTrue(record.snapshot.stop_confirmed)

    def verified_steps(self):
        steps = self.plan_steps()
        steps.insert(2, SkillStep(skill_id="verify_grasp", implementation_id="standard", argument_names=["object"], argument_values=["workpiece"]))
        steps.append(SkillStep(skill_id="verify_placement", implementation_id="standard",
                               argument_names=["object", "target"], argument_values=["workpiece", "tray"]))
        return steps

    def native_config(self):
        return Path(__file__).resolve().parents[1]/"src/robot_bringup/config/demo_native_poses.yaml"

    def test_native_pose_template_keeps_perception_and_calibration_semantics(self):
        self.start(config=self.native_config())
        inspection = self.send(task="inspect_object", target_id="")
        self.assertTrue(self.wait(inspection.get_result_async()).result.success)
        observation = self.state().snapshot.observations[0]
        self.assertEqual((observation.frame_id, observation.pose_meaning, observation.source),
                         ("demo_camera", "object_pose", "configured_native_demo"))
        self.assertAlmostEqual(observation.pose[0], 0.15)
        handle = self.send(task="verified_pick_place")
        result = self.wait(handle.get_result_async()).result
        self.assertTrue(result.success, result.message)
        record = self.record(handle).record
        self.assertEqual(record.completed_steps, 6)
        self.assertTrue(all(s.step.implementation_id == "pose_resolved" for s in record.steps))
        for index in (1, 4):
            self.assertIn("calibration=demo_camera_v1", record.steps[index].transitions[0].message)
            self.assertIn("source_frame=demo_camera", record.steps[index].transitions[0].message)
        self.assertTrue(record.steps[2].has_verification and record.steps[5].has_verification)
        self.assertEqual([(p.entity_id, p.location_id) for p in record.snapshot.known_locations], [("workpiece", "tray")])

    def test_native_pose_plan_and_client_select_second_implementation(self):
        self.start(config=self.native_config())
        steps = self.verified_steps()
        for step in steps:
            step.implementation_id = "pose_resolved"
        handle = self.send_plan(steps)
        result = self.wait(handle.get_result_async()).result
        self.assertEqual((result.success, result.completed_steps), (True, 6), result.message)
        planner = Path(__file__).resolve().parent/"plan_pick_place.py"
        result = subprocess.run(["python3", str(planner), "--implementation", "pose_resolved", "--verify-outcomes",
                                 "--object", "workpiece_two", "--target", "tray_two", "--timeout-ms", "5000"],
                                capture_output=True, text=True, timeout=20, env=self.runtime_env)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertIn('"completed_steps": 6', result.stdout)

    def test_missing_resolver_rejects_entire_plan_before_execution(self):
        self.start()
        pick = next(s for s in self.catalog().skills if s.skill_id == "pick_object")
        resolved = next(i for i in pick.implementations if i.implementation_id == "pose_resolved")
        self.assertFalse(resolved.dependencies_satisfied)
        self.assertIn("target_resolver", {c.role for c in resolved.components})
        steps = self.plan_steps()
        for step in steps:
            step.implementation_id = "pose_resolved"
        self.assertFalse(self.send_plan(steps).accepted)
        self.assertFalse(self.record().found)
        self.assertFalse(self.state().snapshot.resource_leases)

    def test_native_pose_wrong_source_frame_fails_before_moving(self):
        self.start("target_resolution.source_frame:=other_camera", config=self.native_config())
        handle = self.send()
        result = self.wait(handle.get_result_async()).result
        self.assertEqual((result.success, result.error_code), (False, "TARGET_RESOLUTION_FAILED"))
        record = self.record(handle).record
        self.assertEqual(record.completed_steps, 1)
        self.assertFalse(record.snapshot.attached_object)
        self.assertFalse(record.snapshot.placement_candidates)
        self.assertTrue(record.snapshot.stop_confirmed)
        self.assertFalse(record.snapshot.resource_leases)

    def test_native_pose_cancellation_uses_measured_stop_path(self):
        self.start("mock_action_ticks:=50", "mock_stop_ticks:=50", config=self.native_config())
        feedback = []
        handle = self.send(feedback=lambda msg: feedback.append(msg.feedback))
        self.wait_for_pick(feedback)
        self.assertTrue(self.wait(handle.cancel_goal_async()).goals_canceling)
        self.assertTrue(self.state().snapshot.resource_leases)
        result = self.wait(handle.get_result_async())
        self.assertEqual(result.status, GoalStatus.STATUS_CANCELED)
        record = self.record(handle).record
        self.assertEqual(record.steps[1].step.implementation_id, "pose_resolved")
        self.assertTrue(record.snapshot.stop_confirmed)
        self.assertFalse(record.snapshot.resource_leases)
        self.assertFalse(record.snapshot.attached_object)

    def test_native_pose_invalid_tool_calibration_rejected_at_startup(self):
        self.process = subprocess.Popen(["ros2", "run", "robot_bt_runtime", "robot_runtime", "--ros-args", "--params-file",
                                         str(self.native_config()), "-p", "target_resolution.tool_frame:=other_tool"],
                                        stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True, env=self.runtime_env)
        self.assertNotEqual(self.process.wait(timeout=10), 0)
        self.log.seek(0)
        self.assertIn("calibration tool frame must match", self.log.read())

    def test_verified_template_retains_both_step_proofs_and_provenance(self):
        self.start(config=Path(__file__).resolve().parents[1]/"src/robot_bringup/config/demo.yaml")
        handle = self.send(task="verified_pick_place", object_id="workpiece_two", target_id="tray_two")
        self.assertTrue(handle.accepted)
        outcome = self.wait(handle.get_result_async()).result
        self.assertTrue(outcome.success, outcome.message)
        record = self.record(handle).record
        self.assertEqual((record.total_steps, record.completed_steps), (6, 6))
        self.assertEqual([s.step.skill_id for s in record.steps], ["locate_object", "pick_object", "verify_grasp", "locate_object", "place_object", "verify_placement"])
        for index in (2, 5):
            self.assertTrue(record.steps[index].has_verification)
            proof = record.steps[index].verification
            self.assertEqual((proof.entity_id, proof.source), ("workpiece_two", "demo_outcome"))
            self.assertTrue(proof.synthetic and proof.stamp_valid)
            self.assertGreaterEqual(proof.samples, 3)
            self.assertGreaterEqual(proof.stable_ms, 100)
            self.assertGreater(proof.sample_id, 0)
        self.assertFalse(record.snapshot.placement_candidates)
        self.assertFalse(record.snapshot.grasp_verifications)
        self.assertEqual([(p.entity_id, p.location_id) for p in record.snapshot.known_locations], [("workpiece_two", "tray_two")])
        self.assertEqual(record.snapshot.placement_verifications[0].source, "demo_outcome")
        self.assertTrue(record.snapshot.placement_verifications[0].synthetic)

    def test_verified_plan_and_cli_succeed_with_six_steps(self):
        self.start()
        handle = self.send_plan(self.verified_steps())
        result = self.wait(handle.get_result_async()).result
        self.assertTrue(result.success, result.message)
        self.assertEqual(result.completed_steps, 6)
        self.assertTrue(self.record(handle).record.steps[5].has_verification)
        planner = Path(__file__).resolve().parent/"plan_pick_place.py"
        result = subprocess.run(["python3", str(planner), "--verify-outcomes", "--timeout-ms", "5000"],
                                capture_output=True, text=True, timeout=20, env=self.runtime_env)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertIn('"completed_steps": 6', result.stdout)

    def test_missing_outcome_component_rejects_before_execution(self):
        self.start("simulation_outcome_evidence:=false")
        catalog = self.catalog()
        verify = next(s for s in catalog.skills if s.skill_id == "verify_placement")
        self.assertFalse(verify.implementations[0].dependencies_satisfied)
        self.assertFalse(self.send_plan(self.verified_steps()).accepted)
        self.assertFalse(self.record().found)
        self.assertFalse(self.state().snapshot.resource_leases)
        self.assertTrue(self.wait(self.send().get_result_async()).result.success)

    def test_verification_failure_aborts_without_promoting_placement(self):
        self.start("mock_verification_failure:=placement")
        handle = self.send_plan(self.verified_steps())
        outcome = self.wait(handle.get_result_async()).result
        self.assertEqual((outcome.success, outcome.error_code, outcome.completed_steps), (False, "PLACEMENT_NOT_VERIFIED", 5))
        record = self.record(handle).record
        self.assertFalse(record.steps[5].has_verification)
        self.assertFalse(record.snapshot.placement_verifications)
        self.assertFalse(record.snapshot.known_locations)
        self.assertEqual([(p.entity_id, p.location_id) for p in record.snapshot.placement_candidates], [("workpiece", "tray")])
        self.assertTrue(record.snapshot.stop_confirmed)
        self.assertFalse(record.snapshot.resource_leases)

    def test_cancellation_during_verification_preserves_held_state(self):
        self.start("verification_window_ms:=1000")
        feedback = []
        handle = self.send_plan(self.verified_steps(), feedback=lambda msg: feedback.append(msg.feedback))
        end = time.monotonic()+5
        while not any(f.active_skill == "verify_grasp" and f.status == "running" for f in feedback):
            self.assertLess(time.monotonic(), end)
            self.executor.spin_once(timeout_sec=0.01)
        self.assertTrue(self.wait(handle.cancel_goal_async()).goals_canceling)
        outcome = self.wait(handle.get_result_async())
        self.assertEqual(outcome.status, GoalStatus.STATUS_CANCELED)
        record = self.record(handle).record
        self.assertEqual(record.completed_steps, 2)
        self.assertEqual(record.snapshot.attached_object, "workpiece")
        self.assertFalse(record.snapshot.grasp_verifications)
        self.assertFalse(record.steps[2].has_verification)
        self.assertTrue(record.snapshot.stop_confirmed)
        self.assertFalse(record.snapshot.resource_leases)

    def test_plan_second_pair_succeeds_with_four_verified_steps(self):
        self.start(config=Path(__file__).resolve().parents[1] / "src/robot_bringup/config/demo.yaml")
        handle = self.send_plan(self.plan_steps("workpiece_two", "tray_two"))
        self.assertTrue(handle.accepted)
        result = self.wait(handle.get_result_async())
        self.assertTrue(result.result.success)
        self.assertEqual(result.result.completed_steps, 4)

    def test_plan_invalid_late_steps_and_transport_shapes_rejected(self):
        self.start()
        invalid = []
        for field, value in [("skill_id", "unknown"), ("implementation_id", "missing")]:
            steps = self.plan_steps(); setattr(steps[-1], field, value); invalid.append(steps)
        steps = self.plan_steps(); steps[-1].argument_values[-1] = "unknown_target"; invalid.append(steps)
        steps = self.plan_steps(); steps[-1].argument_names = ["object", "object"]; invalid.append(steps)
        steps = self.plan_steps(); steps[-1].argument_values = ["workpiece"]; invalid.append(steps)
        steps = self.plan_steps(); steps[0].argument_values = ["{injected}"]; invalid.append(steps)
        invalid.extend([[], [copy.deepcopy(self.plan_steps()[0]) for _ in range(33)]])
        for steps in invalid:
            self.assertFalse(self.send_plan(steps).accepted)
        self.assertFalse(self.send_plan(version=2).accepted)
        self.assertFalse(self.send_plan(timeout=0).accepted)
        self.assertTrue(self.wait(self.send().get_result_async()).result.success)

    def test_plan_invalid_semantic_order_rejected(self):
        self.start()
        steps = self.plan_steps(); steps[0], steps[1] = steps[1], steps[0]
        self.assertFalse(self.send_plan(steps).accepted)
        steps = self.plan_steps(); del steps[2]
        self.assertFalse(self.send_plan(steps).accepted)
        steps = self.plan_steps(); steps[-1].argument_values[0] = "tray"
        self.assertFalse(self.send_plan(steps).accepted)

    def test_plan_and_task_share_busy_guard_and_cancel_confirmation(self):
        self.start("mock_action_ticks:=50")
        feedback = []
        handle = self.send_plan(feedback=lambda msg: feedback.append(msg.feedback))
        self.assertTrue(handle.accepted)
        end = time.monotonic()+5
        while not any(f.step_index == 1 and f.active_skill == "pick_object" and f.status == "running" for f in feedback):
            self.assertLess(time.monotonic(), end)
            self.executor.spin_once(timeout_sec=0.02)
        self.assertFalse(self.send().accepted)
        self.assertFalse(self.send_plan().accepted)
        self.assertTrue(self.wait(handle.cancel_goal_async()).goals_canceling)
        result = self.wait(handle.get_result_async())
        self.assertEqual(result.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(result.result.completed_steps, 1)
        # The opposite direction also shares the same guard and stop path.
        task = self.send()
        self.assertTrue(task.accepted)
        self.assertFalse(self.send_plan().accepted)
        self.assertTrue(self.wait(task.cancel_goal_async()).goals_canceling)
        self.assertEqual(self.wait(task.get_result_async()).status, GoalStatus.STATUS_CANCELED)
        self.assertTrue(self.wait(self.send_plan().get_result_async()).result.success)

    def test_plan_deadline_stops_then_allows_next_plan(self):
        self.start("mock_action_ticks:=50")
        handle = self.send_plan(timeout=100)
        self.assertTrue(handle.accepted)
        result = self.wait(handle.get_result_async())
        self.assertEqual(result.result.status, "timed_out")
        self.assertIn(result.result.completed_steps, (0, 1))  # deadline may precede the first tick
        self.assertTrue(self.wait(self.send_plan().get_result_async()).result.success)

    def test_plan_skill_failure_aborts_remaining_steps(self):
        self.start("mock_fail_pick:=true")
        handle = self.send_plan()
        result = self.wait(handle.get_result_async())
        self.assertEqual(result.result.error_code, "GRIPPER_FAILED")
        self.assertEqual(result.result.completed_steps, 1)
        self.assertFalse(result.result.success)
        inspect = self.send(task="inspect_object", target_id="")
        self.assertTrue(self.wait(inspect.get_result_async()).result.success)

    def test_planner_example_queries_catalog_and_submits_plan(self):
        self.start(config=Path(__file__).resolve().parents[1] / "src/robot_bringup/config/demo.yaml")
        planner = Path(__file__).resolve().parent / "plan_pick_place.py"
        result = subprocess.run(["python3", str(planner), "--object", "workpiece_two", "--target", "tray_two", "--timeout-ms", "5000"],
                                capture_output=True, text=True, timeout=20, env=self.runtime_env)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertIn('"completed_steps": 4', result.stdout)

    def test_catalog_describes_schemas_and_implementation_dependencies(self):
        self.start("mock_motion_permitted:=false")
        catalog = self.catalog()
        self.assertEqual(catalog.schema_version, 1)
        skills = {s.skill_id: s for s in catalog.skills}
        self.assertEqual(set(skills), {"locate_object", "pick_object", "place_object", "verify_grasp", "verify_placement"})
        self.assertEqual([(p.name, p.type, p.required) for p in skills["place_object"].inputs],
                         [("object", "entity_id", True), ("target", "entity_id", True)])
        pick = next(i for i in skills["pick_object"].implementations if i.implementation_id == "standard")
        self.assertTrue(pick.dependencies_satisfied)  # does not claim the closed gate is armed
        self.assertEqual(pick.execution_gate_role, "safety")
        self.assertEqual({r.role: r.interface_version for r in pick.components},
                         {"motion": 2, "gripper": 2, "safety": 1})
        self.assertEqual(skills["locate_object"].implementations[0].components[0].interface_version, 2)
        verify = skills["verify_placement"].implementations[0]
        self.assertTrue(verify.dependencies_satisfied)
        self.assertFalse(verify.execution_gate_role)
        self.assertEqual({r.role: r.interface_id for r in verify.components}, {"outcome": "manipulation_observer"})
        self.assertEqual(set(catalog.object_ids), {"workpiece"})
        self.assertEqual(set(catalog.target_ids), {"tray"})

    def test_second_configured_pair_and_task_alias(self):
        config = Path(__file__).resolve().parents[1] / "src/robot_bringup/config/demo.yaml"
        self.start(config=config)
        catalog = self.catalog()
        self.assertIn("workpiece_two", catalog.object_ids)
        self.assertIn("tray_two", catalog.target_ids)
        self.assertEqual({t.task_name: t.template_id for t in catalog.tasks}["transfer_two"], "pick_place")
        for task, obj, target in [("transfer_two", "workpiece_two", "tray_two"),
                                  ("pick_place", "workpiece", "tray")]:
            handle = self.send(task=task, object_id=obj, target_id=target)
            self.assertTrue(handle.accepted)
            self.assertTrue(self.wait(handle.get_result_async()).result.success)

    def test_invalid_parameters_rejected_and_runtime_remains_usable(self):
        self.start()
        for obj, target, timeout in [("unknown", "tray", 5000), ("workpiece", "unknown", 5000),
                                     ("tray", "workpiece", 5000), ("", "tray", 5000),
                                     ("workpiece", "", 5000), ("../workpiece", "tray", 5000),
                                     ("workpiece", "tray", 0), ("workpiece", "tray", 600001)]:
            self.assertFalse(self.send(object_id=obj, target_id=target, timeout=timeout).accepted)
        valid = self.send()
        self.assertTrue(valid.accepted)
        self.assertTrue(self.wait(valid.get_result_async()).result.success)

    def test_locate_only_template_uses_no_actuator_permission(self):
        self.start("mock_motion_permitted:=false")
        self.assertFalse(self.send(task="inspect_object").accepted)  # extra target
        handle = self.send(task="inspect_object", object_id="tray", target_id="")
        self.assertTrue(handle.accepted)
        result = self.wait(handle.get_result_async())
        self.assertTrue(result.result.success)
        self.assertIn("demo motion target", result.result.message)

    def test_scene_parameters_are_read_only_after_startup(self):
        self.start()
        client = AsyncParameterClient(self.node, "robot_runtime")
        self.assertTrue(client.wait_for_services(timeout_sec=5))
        result = self.wait(client.set_parameters([Parameter("entities.workpiece.pose", value=[0.2, 0.1, 0.2, 0.0, 0.0, 0.0, 1.0])]))
        self.assertFalse(result.results[0].successful)
        self.assertTrue(self.wait(self.send().get_result_async()).result.success)

    def test_invalid_configuration_fails_before_runtime_starts(self):
        for parameter in ["object_ids:=[workpiece, workpiece]",
                          "entities.workpiece.pose:=[0.4, 0.1, 0.2, 0.0, 0.0, 0.0]",
                          "entities.workpiece.pose:=[0.4, 0.1, 0.2, 0.0, 0.0, 0.0, 0.0]",
                          "tasks.pick_place.template_id:=../external.xml",
                          "tasks.pick_place.implementation_id:=missing"]:
            self.process = subprocess.Popen(["ros2", "run", "robot_bt_runtime", "robot_runtime", "--ros-args", "-p", parameter],
                                            stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True, env=self.runtime_env)
            self.assertNotEqual(self.process.wait(timeout=15), 0, parameter)

    def test_success(self):
        self.start()
        goal = self.send()
        self.assertTrue(goal.accepted)
        outcome = self.wait(goal.get_result_async())
        self.assertEqual(outcome.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(outcome.result.success)
        self.assertEqual(outcome.result.status, "succeeded")

    def test_component_rebinding(self):
        self.start("motion_component:=slow_mock_arm")
        goal = self.send()
        self.assertTrue(self.wait(goal.get_result_async()).result.success)

    def test_backend_failure_propagates(self):
        self.start("mock_fail_pick:=true")
        goal = self.send()
        outcome = self.wait(goal.get_result_async())
        self.assertEqual(outcome.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(outcome.result.error_code, "GRIPPER_FAILED")
        self.assertFalse(outcome.result.success)

    def test_execution_gate_denies_before_actuation(self):
        self.start("mock_motion_permitted:=false")
        goal = self.send()
        outcome = self.wait(goal.get_result_async())
        self.assertEqual(outcome.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(outcome.result.error_code, "SAFETY_INTERLOCK")
        self.assertFalse(outcome.result.success)

    def test_unknown_task_rejected(self):
        self.start()
        self.assertFalse(self.send(task="arbitrary_generated_xml").accepted)

    def test_deadline_stops_task(self):
        self.start("mock_action_ticks:=50")
        goal = self.send(timeout=100)
        outcome = self.wait(goal.get_result_async())
        self.assertEqual(outcome.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(outcome.result.status, "timed_out")
        record = self.record(goal).record
        self.assertEqual((record.status, record.error_code), (outcome.result.status, outcome.result.error_code))
        self.assertTrue(record.snapshot.stop_confirmed)
        self.assertTrue(record.snapshot.resources_empty)

    def test_busy_rejection_and_cancel_confirmation(self):
        self.start("mock_action_ticks:=50")
        feedback = []
        goal = self.send(feedback=lambda msg: feedback.append(msg.feedback))
        self.assertTrue(goal.accepted)
        end = time.monotonic() + 5
        while not any(f.active_skill == "pick_object" and f.status == "running" for f in feedback):
            self.assertLess(time.monotonic(), end, "no running pick feedback")
            self.executor.spin_once(timeout_sec=0.02)
        self.assertFalse(self.send().accepted)
        response = self.wait(goal.cancel_goal_async())
        self.assertTrue(response.goals_canceling)
        outcome = self.wait(goal.get_result_async())
        self.assertEqual(outcome.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(outcome.result.status, "canceled")
        # Successful cancel must release ownership; a subsequent task is accepted.
        next_goal = self.send()
        self.assertTrue(next_goal.accepted)
        self.assertTrue(self.wait(next_goal.get_result_async()).result.success)


if __name__ == "__main__":
    unittest.main(verbosity=2)
