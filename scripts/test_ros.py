"""ROS 2 integration tests against the actual BehaviorTree.CPP runtime, no hardware."""
import os
import signal
import subprocess
import tempfile
import time
import unittest

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from robot_interfaces.action import ExecuteTask


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

    def start(self, *parameters):
        command = ["ros2", "run", "robot_bt_runtime", "robot_runtime", "--ros-args"]
        for parameter in parameters:
            command += ["-p", parameter]
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT,
                                        start_new_session=True)
        self.assertTrue(self.client.wait_for_server(timeout_sec=15), "runtime did not start")

    def wait(self, future, timeout=10):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertTrue(future.done(), "future timed out")
        return future.result()

    def send(self, timeout=5000, task="pick_place", feedback=None):
        goal = ExecuteTask.Goal(task_name=task, object_id="workpiece", target_id="tray", timeout_ms=timeout)
        return self.wait(self.client.send_goal_async(goal, feedback_callback=feedback))

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
        self.assertEqual(outcome.result.error_code, "EXECUTION_FAILED")
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
