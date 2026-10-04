#!/usr/bin/env python3
"""Release-window audit regressions, including old pre-release stability."""

import importlib.util
import sys
import unittest
from pathlib import Path

test_dir = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("test_gazebo_trials_fixture", test_dir / "test_gazebo_trials.py")
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts"))
from gazebo_evidence import audit_physics


class ReleaseWindowAuditTests(unittest.TestCase):
    def test_pre_release_500ms_plus_post_release_10ms_is_rejected(self):
        result = fixture.successful_physics()
        result["task_events"] = [{"phase": "release_confirmed", "run_id": "test-run",
                                  "object_id": "workpiece", "target_id": "tray", "epoch": 0,
                                  "sim_time_ns": 2_890_000_000, "receipt_monotonic_ns": 9_900_000_000}]
        result["pose_history"] = [
            fixture.pose("workpiece", 2_380_000_000, [0.600, -0.180, 0.370], sequence=2),
            fixture.pose("workpiece", 2_580_000_000, [0.600, -0.180, 0.370], sequence=3),
            fixture.pose("workpiece", 2_780_000_000, [0.600, -0.180, 0.370], sequence=4),
            fixture.pose("workpiece", 2_890_000_000, [0.600, -0.180, 0.370], sequence=5),
            fixture.pose("workpiece", 2_900_000_000, [0.600, -0.180, 0.370], sequence=6),
        ]
        result["pose_history"].extend([
            fixture.pose("tray", 2_890_000_000, [0.600, -0.180, 0.370], sequence=5),
            fixture.pose("tray", 2_900_000_000, [0.600, -0.180, 0.370], sequence=6),
        ])
        result["contact_samples"] = [fixture.contact(
            stamp, [["panda::workpiece", "tray::tray_floor::collision"]], sequence=index)
            for index, stamp in enumerate((2_380_000_000, 2_580_000_000, 2_780_000_000,
                                           2_890_000_000, 2_900_000_000), 1)]
        audit = audit_physics(result, fixture.ACCEPTANCE)
        self.assertFalse(audit["checks"]["stable_window_observed"], audit["checks"])
        self.assertFalse(audit["independently_valid_physical_outcome"], audit)

    def test_complete_post_release_window_is_accepted(self):
        result = fixture.successful_physics()
        result["task_events"] = [{"phase": "release_confirmed", "run_id": "test-run",
                                  "object_id": "workpiece", "target_id": "tray", "epoch": 0,
                                  "sim_time_ns": 2_300_000_000, "receipt_monotonic_ns": 9_900_000_000}]
        audit = audit_physics(result, fixture.ACCEPTANCE)
        self.assertTrue(audit["checks"]["stable_window_observed"], audit["checks"])

    def test_post_release_sample_count_or_duration_is_insufficient(self):
        for keep in (2, 5):
            result = fixture.successful_physics()
            result["pose_history"] = [s for s in result["pose_history"]
                if s["object_id"] != "workpiece" or s["source_stamp_ns"] < 2_400_000_000 + keep * 50_000_000]
            result["pose_history"] = [s for s in result["pose_history"]
                if s["object_id"] != "tray" or s["source_stamp_ns"] < 2_400_000_000 + keep * 50_000_000]
            result["contact_samples"] = [s for s in result["contact_samples"]
                if s["source_stamp_ns"] < 2_400_000_000 + keep * 50_000_000]
            audit = audit_physics(result, fixture.ACCEPTANCE)
            self.assertFalse(audit["checks"]["stable_window_observed"], audit["checks"])

    def test_single_finger_contact_mid_window_breaks_stability(self):
        result = fixture.successful_physics()
        result["contact_samples"][6]["contacts"] = [
            ["panda::workpiece", "panda_leftfinger::collision"],
            ["panda::workpiece", "tray::tray_floor::collision"],
        ]
        audit = audit_physics(result, fixture.ACCEPTANCE)
        self.assertFalse(audit["checks"]["stable_window_observed"], audit["checks"])
        self.assertFalse(audit["independently_valid_physical_outcome"], audit)

    def test_leaving_target_during_window_resets_the_candidate(self):
        result = fixture.successful_physics()
        workpiece = [s for s in result["pose_history"] if s["object_id"] == "workpiece"]
        workpiece[6]["xyz"][0] += 0.10
        audit = audit_physics(result, fixture.ACCEPTANCE)
        self.assertFalse(audit["checks"]["stable_window_observed"], audit["checks"])

    def test_contact_evidence_gap_breaks_the_stability_window(self):
        result = fixture.successful_physics()
        result["contact_samples"] = [s for i, s in enumerate(result["contact_samples"])
                                     if not 5 <= i <= 8]
        audit = audit_physics(result, fixture.ACCEPTANCE)
        self.assertFalse(audit["checks"]["stable_window_observed"], audit["checks"])

    def test_epoch_rewind_cannot_join_pre_and_post_reset_samples(self):
        result = fixture.successful_physics()
        result["final_epoch"] = 1
        result["task_events"][-1]["epoch"] = 0
        audit = audit_physics(result, fixture.ACCEPTANCE)
        self.assertFalse(audit["checks"]["release_identity_matches"], audit["checks"])
        self.assertFalse(audit["checks"]["stable_window_observed"], audit["checks"])

    def test_historical_tray_contact_does_not_prove_continuous_support(self):
        result = fixture.successful_physics()
        for sample in result["contact_samples"]:
            if sample["source_stamp_ns"] > 2_400_000_000:
                sample["contacts"] = []
        result["contact_samples"][1]["contacts"] = [
            ["panda::workpiece", "tray::tray_floor::collision"]]
        audit = audit_physics(result, fixture.ACCEPTANCE)
        self.assertFalse(audit["checks"]["tray_contact_during_stable_window"], audit["checks"])
        self.assertFalse(audit["independently_valid_physical_outcome"], audit)

    def test_wrong_run_or_release_identity_is_rejected(self):
        result = fixture.successful_physics()
        result["task_events"][0]["run_id"] = "older-run"
        audit = audit_physics(result, fixture.ACCEPTANCE)
        self.assertFalse(audit["checks"]["release_identity_matches"], audit["checks"])
        self.assertFalse(audit["checks"]["stable_window_observed"], audit["checks"])


if __name__ == "__main__":
    unittest.main()
