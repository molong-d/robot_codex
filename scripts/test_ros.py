"""ROS 2 integration tests against the actual BehaviorTree.CPP runtime, no hardware."""
import os
import signal
import subprocess
import tempfile
import time
import unittest
import copy
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
from robot_interfaces.srv import GetCatalog


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

    def plan_steps(self, object_id="workpiece", target_id="tray"):
        return [SkillStep(skill_id=skill, implementation_id="standard", argument_names=list(args), argument_values=list(args.values()))
                for skill, args in [("locate_object", {"object": object_id}), ("pick_object", {"object": object_id}),
                                    ("locate_object", {"object": target_id}),
                                    ("place_object", {"object": object_id, "target": target_id})]]

    def send_plan(self, steps=None, version=1, timeout=5000, feedback=None):
        self.assertTrue(self.plan_client.wait_for_server(timeout_sec=5))
        goal = ExecutePlan.Goal(schema_version=version, steps=self.plan_steps() if steps is None else steps, timeout_ms=timeout)
        return self.wait(self.plan_client.send_goal_async(goal, feedback_callback=feedback))

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
        self.assertEqual(set(skills), {"locate_object", "pick_object", "place_object"})
        self.assertEqual([(p.name, p.type, p.required) for p in skills["place_object"].inputs],
                         [("object", "entity_id", True), ("target", "entity_id", True)])
        pick = skills["pick_object"].implementations[0]
        self.assertTrue(pick.dependencies_satisfied)  # does not claim the closed gate is armed
        self.assertEqual(pick.execution_gate_role, "safety")
        self.assertEqual({r.role: r.interface_version for r in pick.components},
                         {"motion": 2, "gripper": 2, "safety": 1})
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
        self.assertIn("demo pose", result.result.message)

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
