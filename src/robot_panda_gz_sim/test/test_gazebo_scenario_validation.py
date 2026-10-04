#!/usr/bin/env python3
"""Regression for the pre-verification physical wrong-placement fault order."""

import json
import sys
import unittest
from pathlib import Path

root = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(root / "scripts"))
from test_gazebo_scenarios import validate_case


EXPECTATION = json.loads((root / "src/robot_panda_gz_sim/config/scenario_expectations.json")
                    .read_text(encoding="utf-8"))["placement_verification_reject"]


def fixture(trigger_skill="place_object", released=True):
    result = {
        "status": "failed", "error_code": "PLACEMENT_NOT_VERIFIED", "message": "placement mismatch",
        "final_receipt_monotonic_ns": 300,
        "execution_record": {"stop_confirmed": True, "resources_empty": True, "fault_latched": False,
            "steps": [{"skill_id": "place_object", "status": "succeeded", "error_code": ""},
                      {"skill_id": "verify_placement", "status": "failed",
                       "error_code": "PLACEMENT_NOT_VERIFIED"}]},
        "runtime_state": {"busy": False, "resources_empty": True, "fault_latched": False},
        "task_events": [
            {"phase": "release_confirmed", "receipt_monotonic_ns": 100, "sim_time_ns": 100},
            {"phase": "off_target_pose_injected_after_release", "receipt_monotonic_ns": 200,
             "sim_time_ns": 200, "returncode": 0, "successful": True,
             "trigger_active_skill": trigger_skill, "release_already_observed": released}],
    }
    trial = {"result_present": True, "run_id": "scenario-test", "seed": 701,
        "result_json": "scenario-test/result.json", "gazebo_log": "scenario-test/gazebo.log",
        "identity_checks": {"run_id": True, "seed": True, "code": True},
        "process_exit_code": 1, "independently_valid_physical_outcome": False}
    return trial, result


class ScenarioValidationTests(unittest.TestCase):
    def test_wrong_placement_requires_release_trigger_and_completed_skill_order(self):
        trial, result = fixture()
        checked = validate_case("placement_verification_reject", EXPECTATION, trial, result)
        self.assertTrue(checked["passed"], checked["checks"])
        self.assertTrue(checked["checks"]["fault_injected_after_release_before_verification"])

    def test_trigger_without_release_or_from_verifier_is_rejected(self):
        for trigger_skill, released in (("verify_placement", True), ("place_object", False)):
            with self.subTest(trigger_skill=trigger_skill, released=released):
                trial, result = fixture(trigger_skill, released)
                checked = validate_case("placement_verification_reject", EXPECTATION, trial, result)
                self.assertFalse(checked["passed"], checked["checks"])
                self.assertFalse(checked["checks"]["fault_injected_after_release_before_verification"])


if __name__ == "__main__":
    unittest.main()
