#!/usr/bin/env python3
"""Run fault-injected Panda simulations and require their expected stage/evidence."""

import argparse
import json
import subprocess
import sys
import uuid
from datetime import datetime, timezone
from pathlib import Path

from test_gazebo_trials import run_one_trial


def _steps(result):
    execution = result.get("execution_record") or {}
    return execution.get("steps") or []


def _step(result, name):
    return next((step for step in _steps(result) if step.get("skill_id") == name), None)


def validate_case(name, expectation, trial, result):
    checks = {
        "result_present": trial["result_present"],
        "run_identity_matches": all(trial["identity_checks"].values()),
    }
    if "expected_object_offset_xy_m" in expectation:
        checks["initial_pose_perturbation_applied"] = (
            result.get("object_offset_xy_m") == expectation["expected_object_offset_xy_m"])
    if "expected_status" in expectation:
        checks["expected_status"] = result.get("status") == expectation["expected_status"]
    if "forbidden_status" in expectation:
        checks["forbidden_status_not_seen"] = result.get("status") != expectation["forbidden_status"]
    if "expected_error_code" in expectation:
        actual_code = result.get("error_code")
        if name == "placement_verification_reject":
            failed_step = _step(result, "verify_placement")
            actual_code = None if failed_step is None else failed_step.get("error_code")
        checks["expected_error_code"] = actual_code == expectation["expected_error_code"]

    required_step = expectation.get("requires_step")
    if required_step:
        step = _step(result, required_step)
        checks["required_step_reached"] = step is not None
        if step is not None:
            checks["required_step_status"] = step.get("status") == expectation.get("requires_step_status", "failed")
            checks["verification_step_was_terminal_step"] = _steps(result)[-1].get("skill_id") == required_step

    if expectation.get("requires_completed_place_motion"):
        place = _step(result, "place_object")
        checks["place_motion_completed"] = place is not None and place.get("status") == "succeeded"
        checks["verification_rejected_after_motion"] = bool(
            place and _step(result, "verify_placement") and
            _steps(result).index(place) < _steps(result).index(_step(result, "verify_placement")))

    if expectation.get("requires_physical_audit_success"):
        checks["independent_physics_audit_passed"] = trial.get("independently_valid_physical_outcome") is True
    if expectation.get("requires_physical_audit_rejection"):
        checks["independent_physics_audit_rejected"] = trial.get("independently_valid_physical_outcome") is False

    execution = result.get("execution_record") or {}
    runtime = result.get("runtime_state") or {}
    if expectation.get("requires_safe_terminal_resources"):
        checks["stop_confirmed"] = execution.get("stop_confirmed") is True
        checks["execution_resources_empty"] = execution.get("resources_empty") is True
        checks["no_fault_latched"] = execution.get("fault_latched") is False
        checks["runtime_not_busy"] = runtime.get("busy") is False
        checks["runtime_resources_empty"] = runtime.get("resources_empty") is True
    if expectation.get("requires_retained_resources"):
        checks["stop_not_confirmed"] = execution.get("stop_confirmed") is False
        checks["resources_retained"] = execution.get("resources_empty") is False
        checks["fault_latched"] = execution.get("fault_latched") is True
        checks["resource_lease_present"] = bool(execution.get("resource_leases"))

    if expectation.get("requires_drop_command_success"):
        command = result.get("object_drop_command") or {}
        checks["physical_gripper_open_command_succeeded"] = command.get("returncode") == 0
        checks["drop_injection_event_recorded"] = any(
            e.get("phase") == "external_gripper_open_during_lift" for e in result.get("task_events", []))
    if expectation.get("requires_object_disturbance"):
        disturbance = next((e for e in result.get("task_events", [])
                            if e.get("phase") == "off_target_pose_injected_after_release"), None)
        checks["off_target_fault_injection_succeeded"] = bool(
            disturbance and disturbance.get("returncode") == 0 and disturbance.get("successful") is True)
        release = next((e for e in result.get("task_events", []) if e.get("phase") == "release_confirmed"), None)
        place = _step(result, "place_object")
        verification = _step(result, "verify_placement")
        checks["fault_injected_after_release_before_verification"] = bool(
            disturbance and release and place and place.get("status") == "succeeded" and verification and
            disturbance.get("trigger_active_skill") == "place_object" and
            disturbance.get("release_already_observed") is True and
            release.get("receipt_monotonic_ns", 0) < disturbance.get("receipt_monotonic_ns", 0) <
            result.get("final_receipt_monotonic_ns", 0))
        checks["fault_injected_after_place_motion"] = bool(
            _step(result, "place_object") and _step(result, "place_object").get("status") == "succeeded" and
            _step(result, "verify_placement"))
    if expectation.get("requires_feedback_interruption"):
        checks["feedback_was_interrupted_after_release"] = \
            result.get("contact_feedback_dropped_after_release") is True
        checks["feedback_interruption_event_recorded"] = any(
            e.get("phase") == "contact_feedback_interrupted_after_release" and e.get("returncode") == 0
            for e in result.get("task_events", []))
    if expectation.get("requires_cancel_accepted"):
        checks["cancel_accepted"] = result.get("cancel_accepted") is True
        checks["cancel_result_time_measured"] = result.get("cancel_to_result_s") is not None
        duration = result.get("cancel_to_stop_confirmed_s")
        checks["stop_confirmed_within_timeout"] = duration is not None and \
            duration * 1000 <= expectation["maximum_stop_confirmation_ms"]
    if expectation.get("maximum_stop_confirmation_ms") and name == "timeout":
        duration = result.get("timeout_to_stop_confirmed_s")
        checks["timeout_stop_confirmed_within_timeout"] = duration is not None and \
            0 <= duration * 1000 <= expectation["maximum_stop_confirmation_ms"]
    if expectation.get("requires_pause_and_resume_success"):
        controls = result.get("pause_control") or []
        checks["pause_and_resume_services_succeeded"] = len(controls) == 2 and all(
            item.get("returncode") == 0 for item in controls)
    if expectation.get("requires_reset_success"):
        checks["world_reset_service_succeeded"] = \
            (result.get("reset_control") or {}).get("returncode") == 0

    # A nonzero process return is expected for task failures, but only when the
    # result proves the requested stage and safety postconditions above.
    if expectation.get("expected_status") in ("failed", "faulted", "canceled", "timed_out") or \
            "forbidden_status" in expectation:
        checks["task_failure_returned_nonzero"] = trial.get("process_exit_code") not in (None, 0)
    else:
        checks["process_exit_zero"] = trial.get("process_exit_code") == 0
    return {"scenario": name, "expected": expectation, "passed": all(checks.values()),
            "checks": checks, "run_id": trial["run_id"], "seed": trial["seed"],
            "process_exit_code": trial["process_exit_code"],
            "result_json": trial["result_json"], "launch_log": trial["gazebo_log"],
            "status": result.get("status"), "error_code": result.get("error_code"),
            "message": result.get("message"),
            "execution_record": result.get("execution_record"),
            "runtime_state": result.get("runtime_state"),
            "task_feedback": result.get("task_feedback"),
            "independent_audit": {key: value for key, value in trial.items()
                                  if key.startswith("independently_valid") or
                                  key in ("checks", "placement_xy_error_m", "placement_z_error_m")},
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", action="append", help="run selected scenario (repeatable)")
    parser.add_argument("--output-dir", type=Path, default=Path(".review-runs"))
    parser.add_argument("--process-timeout-s", type=int, default=240)
    args = parser.parse_args(argv)
    root = Path(__file__).resolve().parents[1]
    expectations_path = root / "src/robot_panda_gz_sim/config/scenario_expectations.json"
    expectations = json.loads(expectations_path.read_text(encoding="utf-8"))
    names = args.scenario or list(expectations)
    unknown = sorted(set(names) - set(expectations))
    if unknown:
        parser.error(f"unknown scenarios: {', '.join(unknown)}")
    output_base = (args.output_dir if args.output_dir.is_absolute() else root / args.output_dir).resolve()
    if not output_base.is_relative_to(root):
        parser.error("scenario outputs must be inside the repository mount")
    batch_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:10]
    batch_dir = output_base / f"scenarios-{batch_id}"
    batch_dir.mkdir(parents=True, exist_ok=False)
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root, text=True,
                            capture_output=True, check=True).stdout.strip()
    acceptance = json.loads((root / "src/robot_panda_gz_sim/config/acceptance.json").read_text())
    reports = []
    for index, name in enumerate(names, 1):
        expected = expectations[name]
        trial = run_one_trial(root, batch_dir, index, expected["seed"], commit,
                              args.process_timeout_s, acceptance,
                              task_timeout_ms=expected.get("task_timeout_ms", 30000),
                              extra_args=expected.get("args", ()))
        result_path = root / trial["result_json"]
        try:
            result = json.loads(result_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            result = {"status": "missing_or_invalid_result",
                      "error_code": f"{type(error).__name__}: {error}"}
        report = validate_case(name, expected, trial, result)
        case_dir = root / trial["result_json"]
        case_report_path = case_dir.parent / "scenario-check.json"
        case_report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                                    encoding="utf-8")
        reports.append(report)
        print(f"[{index}/{len(names)}] {'PASS' if report['passed'] else 'FAIL'} "
              f"scenario={name} status={report.get('status')} error={report.get('error_code')}",
              flush=True)
    summary = {"batch_id": batch_id, "code_commit": commit, "expectations_file":
               str(expectations_path.relative_to(root)), "scenario_count": len(reports),
               "passed_count": sum(report["passed"] for report in reports),
               "failed_count": sum(not report["passed"] for report in reports),
               "scenarios": reports}
    summary_path = batch_dir / "scenario-summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                            encoding="utf-8")
    print(f"summary={summary_path.relative_to(root)}")
    return 0 if summary["failed_count"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
