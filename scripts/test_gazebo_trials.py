#!/usr/bin/env python3
"""Run fixed-seed Panda pick-place trials and independently audit final state."""

import argparse
import json
import math
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


def git_output(root, *args):
    result = subprocess.run(["git", *args], cwd=root, text=True, capture_output=True)
    return result.stdout.strip() if result.returncode == 0 else None


def audit(result):
    pose_samples = result.get("pose_samples", {})
    item = pose_samples.get("workpiece")
    tray = pose_samples.get("tray")
    contacts = result.get("contact_state", {})
    seen = contacts.get("seen", [])
    flat = [" ".join(pair).lower() for pair in seen]
    left_finger = any("workpiece" in pair and "leftfinger" in pair for pair in flat)
    right_finger = any("workpiece" in pair and "rightfinger" in pair for pair in flat)
    tray_floor = any("workpiece" in pair and "tray_floor" in pair for pair in flat)
    final_grasp = contacts.get("grasp_feedback")

    xy_error = None
    z_error = None
    if item and tray:
        xy_error = math.hypot(item["xyz"][0] - tray["xyz"][0],
                              item["xyz"][1] - tray["xyz"][1])
        z_error = abs(item["xyz"][2] - tray["xyz"][2])

    checks = {
        "task_succeeded": result.get("success") is True and result.get("status") == "succeeded",
        "both_fingers_contacted_workpiece": left_finger and right_finger,
        "released_at_final_sample": final_grasp is False,
        "workpiece_contacted_tray_floor": tray_floor,
        "final_xy_error_within_0.05m": xy_error is not None and xy_error <= 0.05,
        "final_z_error_within_0.015m": z_error is not None and z_error <= 0.015,
    }
    return {
        "independently_valid_success": all(checks.values()),
        "checks": checks,
        "placement_xy_error_m": xy_error,
        "placement_z_error_m": z_error,
        "final_workpiece_xyz_m": None if item is None else item["xyz"],
        "tray_xyz_m": None if tray is None else tray["xyz"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--seed-start", type=int, default=100)
    parser.add_argument("--timeout-ms", type=int, default=30000)
    parser.add_argument("--output-dir", type=Path, default=Path("docs/reports/phase2-trials"))
    parser.add_argument("--summary", type=Path, default=Path("docs/reports/phase2-trials-summary.json"))
    args = parser.parse_args()
    if args.count <= 0 or args.count > 1000:
        parser.error("--count must be between 1 and 1000")
    if args.seed_start < 0 or args.seed_start + args.count - 1 > 2**31 - 1:
        parser.error("seed range must fit a nonnegative signed 32-bit integer")
    if not 1000 <= args.timeout_ms <= 600000:
        parser.error("--timeout-ms must be between 1000 and 600000")

    root = Path(__file__).resolve().parents[1]
    output_dir = (args.output_dir if args.output_dir.is_absolute() else root / args.output_dir).resolve()
    summary_path = (args.summary if args.summary.is_absolute() else root / args.summary).resolve()
    if not output_dir.is_relative_to(root) or not summary_path.is_relative_to(root):
        parser.error("trial outputs must be inside the repository so the container can save them")
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    workspace_status_before_trials = git_output(root, "status", "--porcelain")
    trials = []

    for index in range(args.count):
        seed = args.seed_start + index
        result_path = output_dir / f"trial-{seed}.json"
        launch_log = output_dir / f"trial-{seed}.log"
        command = [
            "scripts/with_jazzy.sh", "scripts/test_gazebo.sh",
            "--timeout-ms", str(args.timeout_ms), "--seed", str(seed),
            "--result-json", str(result_path.relative_to(root)),
            "--launch-log", str(launch_log.relative_to(root)),
        ]
        started = time.monotonic()
        completed = subprocess.run(command, cwd=root, text=True, capture_output=True)
        elapsed = time.monotonic() - started
        result = {}
        parse_error = None
        try:
            result = json.loads(result_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            parse_error = f"{type(error).__name__}: {error}"
        checks = audit(result) if result else {
            "independently_valid_success": False,
            "checks": {},
            "placement_xy_error_m": None,
            "placement_z_error_m": None,
            "final_workpiece_xyz_m": None,
            "tray_xyz_m": None,
        }
        trial = {
            "seed": seed,
            "command": command,
            "exit_code": completed.returncode,
            "elapsed_wall_s": elapsed,
            "result_json": str(result_path.relative_to(root)),
            "launch_log": str(launch_log.relative_to(root)),
            "status": result.get("status"),
            "error_code": result.get("error_code"),
            "message": result.get("message"),
            **checks,
        }
        if parse_error:
            trial["result_parse_error"] = parse_error
        if completed.returncode != 0 and completed.stdout:
            trial["stdout_tail"] = completed.stdout[-2000:]
        if completed.stderr:
            trial["stderr_tail"] = completed.stderr[-2000:]
        trials.append(trial)
        label = "PASS" if trial["independently_valid_success"] and completed.returncode == 0 else "FAIL"
        print(f"[{index + 1}/{args.count}] seed={seed} {label} "
              f"status={trial['status']} xy_error_m={trial['placement_xy_error_m']} "
              f"elapsed_wall_s={elapsed:.2f}", flush=True)

    successes = sum(trial["independently_valid_success"] for trial in trials)
    false_successes = sum(
        trial.get("status") == "succeeded" and not trial["independently_valid_success"]
        for trial in trials
    )
    valid_errors = [trial["placement_xy_error_m"] for trial in trials
                    if trial["independently_valid_success"]]
    measured_errors = [trial["placement_xy_error_m"] for trial in trials
                       if trial["placement_xy_error_m"] is not None]
    summary = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "code_commit": git_output(root, "rev-parse", "HEAD"),
        "workspace_status_before_trials": workspace_status_before_trials,
        "command_template": ["scripts/with_jazzy.sh", "scripts/test_gazebo.sh",
                             "--timeout-ms", str(args.timeout_ms), "--seed", "<seed>"],
        "requested_count": args.count,
        "completed_count": len(trials),
        "seed_start": args.seed_start,
        "seed_end": args.seed_start + args.count - 1,
        "independently_valid_success_count": successes,
        "success_rate": successes / len(trials),
        "false_success_count": false_successes,
        "placement_xy_error_m": {
            "measured_count": len(measured_errors),
            "measured_values": measured_errors,
            "valid_success_count": len(valid_errors),
            "valid_success_mean": None if not valid_errors else sum(valid_errors) / len(valid_errors),
            "valid_success_max": None if not valid_errors else max(valid_errors),
        },
        "false_success_audit": {
            "required": ["task status succeeded", "two-finger contact observed",
                         "released in final sample", "Gazebo tray-floor contact observed",
                         "final XY error <= 0.05 m", "final Z error <= 0.015 m"],
            "stability_window_ms": 500,
            "stability_window_verified_by": "runtime GazeboWorldAdapter using distinct advancing simulation-time samples",
        },
        "trials": trials,
    }
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                            encoding="utf-8")
    print(f"summary={summary_path.relative_to(root) if summary_path.is_relative_to(root) else summary_path}")
    print(f"successes={successes}/{len(trials)} false_successes={false_successes}")
    return 0 if successes == len(trials) and false_successes == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
