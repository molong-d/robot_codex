#!/usr/bin/env python3
"""Regression tests for stale trial result reuse and physical audit evidence."""

import json
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts"))
from gazebo_evidence import audit_physics
from test_gazebo_trials import run_one_trial


ACCEPTANCE = {
    "world_frame": "world", "gazebo_world_name": "panda_pick_place",
    "object_id": "workpiece", "target_id": "tray",
    "support_surface_z_m": 0.35, "object_height_m": 0.04, "grasp_lift_m": 0.04,
    "placement_xy_tolerance_m": 0.05, "placement_z_tolerance_m": 0.015,
    "stable_speed_mps": 0.04, "verification_window_ms": 500,
    "minimum_physical_samples": 3, "evidence_max_age_ms": 500,
    "contact_pose_pairing_tolerance_ms": 100,
}


def pose(object_id, stamp, xyz, receipt=9_900_000_000, frame="world", epoch=0, sequence=None):
    return {
        "stream": f"pose:{object_id}", "object_id": object_id, "frame_id": frame,
        "source_frame_id": frame,
        "source_stamp_ns": stamp, "sim_time_ns": stamp + 10_000_000,
        "receipt_monotonic_ns": receipt, "epoch": epoch, "sequence": sequence or stamp,
        "accepted": True, "xyz": xyz,
    }


def contact(stamp, pairs, receipt=9_900_000_000, epoch=0, sequence=None, frame="world"):
    return {
        "stream": "contact", "object_id": "workpiece", "frame_id": frame,
        "source_frame_id": frame,
        "source_stamp_ns": stamp, "sim_time_ns": stamp + 10_000_000,
        "receipt_monotonic_ns": receipt, "epoch": epoch, "sequence": sequence or stamp,
        "accepted": True, "contacts": pairs,
    }


def successful_physics():
    poses = [
        pose("workpiece", 1_200_000_000, [0.44, 0.05, 0.43], sequence=1),
        pose("workpiece", 2_400_000_000, [0.600, -0.180, 0.370], sequence=2),
        pose("workpiece", 2_600_000_000, [0.601, -0.180, 0.370], sequence=3),
        pose("workpiece", 2_800_000_000, [0.600, -0.181, 0.370], sequence=4),
        pose("workpiece", 2_900_000_000, [0.600, -0.180, 0.370], sequence=5),
    ]
    poses.extend(pose("tray", stamp, [0.600, -0.180, 0.370], sequence=i)
                 for i, stamp in enumerate((2_400_000_000, 2_600_000_000,
                                             2_800_000_000, 2_900_000_000), 1))
    both = [["panda::workpiece", "panda_leftfinger::collision"],
            ["panda::workpiece", "panda_rightfinger::collision"]]
    floor = [["panda::workpiece", "tray::tray_floor::collision"]]
    contacts = [contact(1_000_000_000, both, sequence=1),
                contact(2_400_000_000, [], sequence=2),
                contact(2_600_000_000, floor, sequence=3),
                contact(2_800_000_000, floor, sequence=4),
                contact(2_900_000_000, floor, sequence=5)]
    return {
        "success": True, "status": "succeeded", "seed": 42, "code_commit": "abc",
        "run_id": "test-run", "pose_history": poses, "contact_samples": contacts,
        "task_events": [{"phase": "release_confirmed", "sim_time_ns": 2_300_000_000}],
        "final_sim_time_ns": 2_910_000_000, "final_receipt_monotonic_ns": 10_000_000_000,
        "final_epoch": 0,
    }


def require(value, message):
    if not value:
        raise AssertionError(message)


def main():
    # A legacy successful result is deliberately present. The new batch gets a
    # never-before-used child path; return 127 and no output must remain a fail.
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        batch = root / "batch-unique"
        batch.mkdir()
        (batch / "trial-100.json").write_text(json.dumps(successful_physics()), encoding="utf-8")

        def failed_launch(command, cwd, log_file, timeout_s):
            return 127, False, 0.01

        record = run_one_trial(root, batch, 1, 100, "commit", 60, ACCEPTANCE, failed_launch)
        require(record["process_exit_code"] == 127 and not record["result_present"],
                "failed launch reused a successful stale result")
        require(not record["trial_accepted"] and not record["false_success"],
                "missing output after process failure counted as success")
        require(record["run_id"] in record["command"], "trial identity was not passed to the child")

    result = successful_physics()
    audit = audit_physics(result, ACCEPTANCE)
    require(audit["independently_valid_physical_outcome"],
            f"valid evidence fixture rejected: {audit['checks']}")

    stale = successful_physics()
    next(s for s in reversed(stale["pose_history"]) if s["object_id"] == "workpiece")[
        "receipt_monotonic_ns"] = 1
    require(not audit_physics(stale, ACCEPTANCE)["checks"]["fresh_final_object_pose"],
            "old cached pose was accepted as final evidence")

    old_stamp = successful_physics()
    old_pose = next(s for s in reversed(old_stamp["pose_history"]) if s["object_id"] == "workpiece")
    old_pose["source_stamp_ns"] = 1_000_000_000
    require(not audit_physics(old_stamp, ACCEPTANCE)["independently_valid_physical_outcome"],
            "old source timestamp was accepted in the terminal state")

    wrong_frame = successful_physics()
    next(s for s in reversed(wrong_frame["pose_history"]) if s["object_id"] == "workpiece")[
        "frame_id"] = "base_link"
    require(not audit_physics(wrong_frame, ACCEPTANCE)["independently_valid_physical_outcome"],
            "wrong-coordinate-frame sample was accepted")

    wrong_source_frame = successful_physics()
    next(s for s in reversed(wrong_source_frame["pose_history"]) if s["object_id"] == "workpiece")[
        "source_frame_id"] = "base_link"
    require(not audit_physics(wrong_source_frame, ACCEPTANCE)["independently_valid_physical_outcome"],
            "unrecognized source frame was accepted after normalization")

    gaz_world_alias = successful_physics()
    for sample in gaz_world_alias["pose_history"]:
        sample["source_frame_id"] = ACCEPTANCE["gazebo_world_name"]
    require(audit_physics(gaz_world_alias, ACCEPTANCE)["independently_valid_physical_outcome"],
            "configured Gazebo world alias was not normalized to the world frame")

    historical_contact = successful_physics()
    historical_contact["contact_samples"][-1]["contacts"] = []
    require(not audit_physics(historical_contact, ACCEPTANCE)["checks"]["final_tray_floor_contact"],
            "historical tray contact substituted for final support evidence")

    split_fingers = successful_physics()
    dual_contact = split_fingers["contact_samples"][0]
    dual_contact["contacts"] = [dual_contact["contacts"][0]]
    split_fingers["contact_samples"].insert(1, contact(
        1_010_000_000, [["panda::workpiece", "panda_rightfinger::collision"]], sequence=2))
    for sequence, sample in enumerate(split_fingers["contact_samples"], 1):
        sample["sequence"] = sequence
    require(not audit_physics(split_fingers, ACCEPTANCE)["checks"]["dual_finger_contact_same_sample"],
            "separate finger samples were combined into simultaneous grasp evidence")

    dropped = successful_physics()
    dropped["pose_history"].append(pose("workpiece", 2_950_000_000,
                                         [0.600, -0.180, 0.310], sequence=6))
    dropped["contact_samples"].append(contact(
        2_950_000_000, [["panda::workpiece", "table::collision"]], sequence=6))
    require(not audit_physics(dropped, ACCEPTANCE)["independently_valid_physical_outcome"],
            "post-release drop was accepted as a stable placement")

    old_epoch = successful_physics()
    for sample in old_epoch["pose_history"]:
        if sample["object_id"] == "workpiece":
            sample["epoch"] = 1
    require(not audit_physics(old_epoch, ACCEPTANCE)["checks"]["same_epoch_world_poses"],
            "cross-epoch pose was combined with current evidence")

    wrong_identity = successful_physics()
    for sample in wrong_identity["pose_history"]:
        if sample["object_id"] == "workpiece":
            sample["object_id"] = "other_object"
    require(not audit_physics(wrong_identity, ACCEPTANCE)["checks"]["fresh_final_object_pose"],
            "a different object's pose was accepted")

    # The audit intentionally ignores the controller/runtime success bit.
    physical_only = successful_physics()
    physical_only["success"] = False
    physical_only["status"] = "failed"
    require(audit_physics(physical_only, ACCEPTANCE)["independently_valid_physical_outcome"],
            "physical audit improperly depended on runtime success")
    print("stale-result isolation and independent physical evidence audit passed")


if __name__ == "__main__":
    main()
