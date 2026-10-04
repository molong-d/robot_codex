#!/usr/bin/env python3
"""Run uniquely identified fixed-seed Panda trials and audit their physics."""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

from gazebo_evidence import audit_physics


def git_output(root, *args):
    result = subprocess.run(["git", *args], cwd=root, text=True, capture_output=True)
    return result.stdout.strip() if result.returncode == 0 else None


def load_result(path):
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        return None, f"{type(error).__name__}: {error}"
    if not isinstance(value, dict):
        return None, "ValueError: result JSON root must be an object"
    return value, None


def identity_checks(result, run_id, seed, code_commit):
    return {
        "run_id_matches": result.get("run_id") == run_id,
        "seed_matches": result.get("seed") == seed,
        "code_commit_matches": result.get("code_commit") == code_commit,
    }


def _terminate_group(process):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=5.0)


def _launch(command, root, log_file, timeout_s):
    started = time.monotonic()
    process = subprocess.Popen(command, cwd=root, stdout=log_file, stderr=subprocess.STDOUT,
                               start_new_session=True, text=True)
    timed_out = False
    try:
        process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        timed_out = True
        _terminate_group(process)
    return process.returncode, timed_out, time.monotonic() - started


def run_one_trial(root, batch_dir, index, seed, code_commit, process_timeout_s,
                  acceptance, launch=_launch, task_timeout_ms=30000, extra_args=()):
    run_id = f"trial-{index:03d}-seed-{seed}-{uuid.uuid4().hex[:12]}"
    trial_dir = batch_dir / f"trial-{index:03d}-seed-{seed}"
    trial_dir.mkdir(parents=True, exist_ok=False)
    result_path = trial_dir / "result.json"
    launch_log = trial_dir / "gazebo.log"
    process_log = trial_dir / "runner.log"
    if result_path.exists():
        raise RuntimeError(f"fresh trial result unexpectedly exists: {result_path}")
    command = [
        "scripts/with_jazzy.sh", "scripts/test_gazebo.sh",
        "--timeout-ms", str(task_timeout_ms), "--seed", str(seed),
        "--run-id", run_id, "--code-commit", code_commit,
        "--result-json", str(result_path.relative_to(root)),
        "--launch-log", str(launch_log.relative_to(root)),
    ] + list(extra_args)
    start_ns = time.monotonic_ns()
    with process_log.open("w", encoding="utf-8") as log_file:
        try:
            exit_code, timed_out, elapsed = launch(command, root, log_file, process_timeout_s)
        except Exception as error:
            exit_code, timed_out, elapsed = None, False, (time.monotonic_ns() - start_ns) * 1e-9
            launch_error = f"{type(error).__name__}: {error}"
        else:
            launch_error = None
    result, parse_error = load_result(result_path)
    if result is None:
        checks = {
            "result_complete": False,
            "task_completed_successfully": False,
            "independently_valid_physical_outcome": False,
            "checks": {},
        }
        identities = {"run_id_matches": False, "seed_matches": False, "code_commit_matches": False}
    else:
        required = ("run_id", "seed", "code_commit", "status", "success", "pose_history",
                    "contact_samples", "task_events", "final_sim_time_ns",
                    "final_receipt_monotonic_ns", "final_epoch")
        identities = identity_checks(result, run_id, seed, code_commit)
        complete = (all(key in result for key in required) and
                    isinstance(result.get("pose_history"), list) and
                    isinstance(result.get("contact_samples"), list) and
                    isinstance(result.get("task_events"), list) and
                    isinstance(result.get("execution_record"), dict) and
                    isinstance(result.get("runtime_state"), dict) and
                    isinstance(result.get("status"), str) and
                    isinstance(result.get("success"), bool) and
                    all(isinstance(result.get(key), int) for key in
                        ("final_sim_time_ns", "final_receipt_monotonic_ns", "final_epoch")))
        checks = {
            "result_complete": complete,
            "task_completed_successfully": result.get("status") == "succeeded" and result.get("success") is True,
        }
        try:
            checks.update(audit_physics(result, acceptance) if complete else
                          {"independently_valid_physical_outcome": False, "checks": {}})
        except Exception as error:
            checks.update({"independently_valid_physical_outcome": False, "checks": {},
                           "audit_error": f"{type(error).__name__}: {error}"})
    process_ok = exit_code == 0 and not timed_out
    runtime_stopped = bool(result and isinstance(result.get("execution_record"), dict) and
        result["execution_record"].get("stop_confirmed") is True and
        result["execution_record"].get("resources_empty") is True and
        result["execution_record"].get("fault_latched") is False and
        isinstance(result.get("runtime_state"), dict) and
        result["runtime_state"].get("busy") is False and
        result["runtime_state"].get("resources_empty") is True)
    accepted = bool(process_ok and result is not None and checks.get("result_complete") and
                    all(identities.values()) and checks.get("task_completed_successfully") and
                    checks.get("independently_valid_physical_outcome") and runtime_stopped)
    task_reported_success = bool(result and result.get("status") == "succeeded" and result.get("success") is True)
    record = {
        "run_id": run_id,
        "seed": seed,
        "code_commit": code_commit,
        "command": command,
        "process_exit_code": exit_code,
        "process_timed_out": timed_out,
        "process_timeout_s": process_timeout_s,
        "elapsed_wall_s": elapsed,
        "process_log": str(process_log.relative_to(root)),
        "gazebo_log": str(launch_log.relative_to(root)),
        "result_json": str(result_path.relative_to(root)),
        "result_present": result is not None,
        "result_parse_error": parse_error,
        "launch_error": launch_error,
        "identity_checks": identities,
        "trial_accepted": accepted,
        "false_success": bool(task_reported_success and not checks.get("independently_valid_physical_outcome")),
        "runtime_stop_and_resource_release_confirmed": runtime_stopped,
        **checks,
    }
    if result is not None:
        record["status"] = result.get("status")
        record["error_code"] = result.get("error_code")
        record["message"] = result.get("message")
    (trial_dir / "trial-record.json").write_text(
        json.dumps(record, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return record


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--seed-start", type=int, default=200)
    parser.add_argument("--process-timeout-s", type=int, default=240)
    parser.add_argument("--output-dir", type=Path, default=Path(".review-runs"))
    parser.add_argument("--batch-id")
    args = parser.parse_args(argv)
    if args.count <= 0 or args.count > 1000:
        parser.error("--count must be between 1 and 1000")
    if args.seed_start < 0 or args.seed_start + args.count - 1 > 2**31 - 1:
        parser.error("seed range must fit a nonnegative signed 32-bit integer")
    if not 60 <= args.process_timeout_s <= 1800:
        parser.error("--process-timeout-s must be between 60 and 1800")

    root = Path(__file__).resolve().parents[1]
    output_base = (args.output_dir if args.output_dir.is_absolute() else root / args.output_dir).resolve()
    if not output_base.is_relative_to(root):
        parser.error("trial outputs must be inside the repository mount")
    batch_id = args.batch_id or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:10]
    if not batch_id.replace("-", "").replace("_", "").isalnum():
        parser.error("--batch-id must contain only letters, digits, '-' or '_'")
    batch_dir = output_base / batch_id
    try:
        batch_dir.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        parser.error(f"batch output already exists; refusing to reuse it: {batch_dir}")
    commit = git_output(root, "rev-parse", "HEAD")
    status = git_output(root, "status", "--porcelain")
    acceptance_path = root / "src/robot_panda_gz_sim/config/acceptance.json"
    acceptance = json.loads(acceptance_path.read_text(encoding="utf-8"))

    trials = []
    for index in range(args.count):
        trial = run_one_trial(root, batch_dir, index + 1, args.seed_start + index, commit,
                              args.process_timeout_s, acceptance)
        trials.append(trial)
        label = "PASS" if trial["trial_accepted"] else "FAIL"
        print(f"[{index + 1}/{args.count}] {label} run_id={trial['run_id']} seed={trial['seed']} "
              f"exit={trial['process_exit_code']} status={trial.get('status')} "
              f"xy_error_m={trial.get('placement_xy_error_m')}", flush=True)
    successes = sum(trial["trial_accepted"] for trial in trials)
    false_successes = sum(trial["false_success"] for trial in trials)
    summary = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "batch_id": batch_id,
        "code_commit": commit,
        "workspace_status_before_trials": status,
        "workspace_clean_before_trials": not bool(status),
        "acceptance_config": str(acceptance_path.relative_to(root)),
        "command_template": ["scripts/with_jazzy.sh", "scripts/test_gazebo.sh",
                             "--timeout-ms", "30000", "--seed", "<seed>",
                             "--run-id", "<unique>", "--code-commit", commit],
        "requested_count": args.count,
        "completed_count": len(trials),
        "seed_start": args.seed_start,
        "seed_end": args.seed_start + args.count - 1,
        "accepted_success_count": successes,
        "success_rate": successes / len(trials),
        "false_success_count": false_successes,
        "process_failure_count": sum(t["process_exit_code"] != 0 or t["process_timed_out"] for t in trials),
        "result_invalid_count": sum(not t["result_complete"] for t in trials),
        "audit_failure_count": sum(not t.get("independently_valid_physical_outcome", False) for t in trials),
        "placement_xy_error_m": {
            "measured_count": sum(t.get("placement_xy_error_m") is not None for t in trials),
            "values": [t["placement_xy_error_m"] for t in trials if t.get("placement_xy_error_m") is not None],
        },
        "trials": trials,
    }
    summary_path = batch_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                            encoding="utf-8")
    print(f"summary={summary_path.relative_to(root)}")
    print(f"successes={successes}/{len(trials)} false_successes={false_successes} "
          f"process_failures={summary['process_failure_count']} audit_failures={summary['audit_failure_count']}")
    return 0 if successes == args.count and false_successes == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
