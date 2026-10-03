"""Exercise actual MoveIt, controllers, TF and our adapters with GenericSystem."""
import os
import signal
import subprocess
import tempfile
import time
import unittest

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import ParallelGripperCommand
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from robot_interfaces.action import ExecuteTask


class PandaTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.log = tempfile.TemporaryFile(mode="w+")
        cls.process = subprocess.Popen(["ros2", "launch", "robot_panda_demo", "panda.launch.py"],
                                       stdout=cls.log, stderr=subprocess.STDOUT, start_new_session=True)
        cls.node = rclpy.create_node("panda_adapter_tests")
        cls.client = ActionClient(cls.node, ExecuteTask, "execute_task")
        cls.arm = ActionClient(cls.node, MoveGroup, "move_action")
        cls.hand = ActionClient(cls.node, ParallelGripperCommand, "panda_hand_controller/gripper_cmd")
        cls.states = []
        cls.subscription = cls.node.create_subscription(JointState, "joint_states", lambda s: cls.states.append(s), 10)

    @classmethod
    def tearDownClass(cls):
        cls.client.destroy(); cls.arm.destroy(); cls.hand.destroy(); cls.node.destroy_node()
        if cls.process.poll() is None:
            cls.process.send_signal(signal.SIGINT)  # launch propagates once to its children
            try:
                cls.process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(cls.process.pid, signal.SIGKILL); cls.process.wait(timeout=5)
        cls.log.seek(0); print(cls.log.read()); cls.log.close()
        rclpy.shutdown()

    def wait(self, future, timeout=100):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertTrue(future.done(), "future timed out")
        return future.result()

    def ready(self):
        self.assertTrue(self.arm.wait_for_server(timeout_sec=60), "MoveIt server unavailable")
        self.assertTrue(self.hand.wait_for_server(timeout_sec=60), "gripper controller unavailable")
        self.assertTrue(self.client.wait_for_server(timeout_sec=30), "runtime unavailable")
        # Give TF, state broadcaster and initial feedback time to reach runtime.
        end = time.monotonic()+3
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)

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


if __name__ == "__main__":
    unittest.main(verbosity=2)
