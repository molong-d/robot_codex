"""ROS 2 integration tests against the actual BehaviorTree.CPP runtime, no hardware."""
import os
import signal
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from robot_interfaces.action import ExecuteTask
from robot_interfaces.srv import GetCatalog


class RuntimeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.process = None
        self.log = tempfile.TemporaryFile(mode="w+")
        self.node = rclpy.create_node("robot_runtime_test")
        self.client = ActionClient(self.node, ExecuteTask, "execute_task")

    def tearDown(self):
        self.client.destroy()
        self.node.destroy_node()
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
                                        start_new_session=True)
        self.assertTrue(self.client.wait_for_server(timeout_sec=15), "runtime did not start")

    def wait(self, future, timeout=10):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
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
                                            stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
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
            rclpy.spin_once(self.node, timeout_sec=0.02)
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
