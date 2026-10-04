#!/usr/bin/env python3
"""Lossless, ROS-free JSONL evidence package for the independent Gazebo audit."""

import gzip
import hashlib
import io
import json
import tarfile
from pathlib import Path

from gazebo_evidence import AUDIT_VERSION, audit_physics

FORMAT_VERSION = "robot-codex-gazebo-audit-jsonl-gzip-v1"


def _sha256_bytes(value):
    return hashlib.sha256(value).hexdigest()


def _sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _line(value):
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")


def _trial_map(batch_dir):
    summary_path = Path(batch_dir) / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    return summary, {item["run_id"]: item for item in summary.get("trials", [])}


def _audit_input(result):
    return {key: result.get(key) for key in (
        "run_id", "seed", "code_commit", "status", "success", "error_code", "message",
        "object_id", "target_id", "final_sim_time_ns", "final_receipt_monotonic_ns", "final_epoch",
        "pose_history", "contact_samples", "task_events")}


def export_batch(batch_dir, output_path, acceptance_path):
    """Export every raw audit sample from each result named in summary.json."""
    batch_dir = Path(batch_dir)
    output_path = Path(output_path)
    acceptance_path = Path(acceptance_path)
    summary, trials = _trial_map(batch_dir)
    acceptance_bytes = acceptance_path.read_bytes()
    acceptance = json.loads(acceptance_bytes)
    code_commit = summary.get("code_commit")
    if not isinstance(code_commit, str) or not code_commit:
        raise ValueError("batch summary has no tested code_commit")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    records = io.BytesIO()
    runs = []
    line_count = 0
    for trial in summary.get("trials", []):
        run_id = trial.get("run_id")
        result_path = batch_dir.parent.parent / trial.get("result_json", "")
        if not result_path.is_file():
            # The failed attempt is still represented in the manifest; absence
            # is evidence of process/output failure and is never filled in.
            runs.append({"run_id": run_id, "seed": trial.get("seed"), "code_commit": code_commit,
                         "process_exit_code": trial.get("process_exit_code"),
                         "process_timed_out": trial.get("process_timed_out"),
                         "result_present": False, "result_sha256": None,
                         "task_status": None, "task_success": None, "execution": None,
                         "raw_audit": None, "export_recompute_matches_raw": None})
            continue
        raw_bytes = result_path.read_bytes()
        result = json.loads(raw_bytes)
        if result.get("run_id") != run_id or result.get("seed") != trial.get("seed") or \
                result.get("code_commit") != code_commit or trial.get("code_commit") != code_commit:
            raise ValueError(f"run identity mismatch for {run_id}: result, trial and batch must agree")
        audit_input = _audit_input(result)
        has_raw = all(isinstance(audit_input.get(key), list) for key in
                      ("pose_history", "contact_samples", "task_events"))
        raw_audit = audit_physics(audit_input, acceptance) if has_raw else None
        record = {
            "record_type": "run",
            "run_id": run_id,
            "seed": trial.get("seed"),
            "code_commit": code_commit,
            "process_exit_code": trial.get("process_exit_code"),
            "process_timed_out": trial.get("process_timed_out"),
            "process_timeout_s": trial.get("process_timeout_s"),
            "result_present": True,
            "result_sha256": _sha256_bytes(raw_bytes),
            "task_status": result.get("status"),
            "task_success": result.get("success"),
            "error_code": result.get("error_code"),
            "object_id": result.get("object_id", acceptance["object_id"]),
            "target_id": result.get("target_id", acceptance["target_id"]),
            "final_sim_time_ns": result.get("final_sim_time_ns"),
            "final_receipt_monotonic_ns": result.get("final_receipt_monotonic_ns"),
            "final_epoch": result.get("final_epoch"),
            "execution": {
                "record_status": (result.get("execution_record") or {}).get("status"),
                "error_code": (result.get("execution_record") or {}).get("error_code"),
                "stop_confirmed": (result.get("execution_record") or {}).get("stop_confirmed"),
                "resources_empty": (result.get("execution_record") or {}).get("resources_empty"),
                "runtime_busy": (result.get("runtime_state") or {}).get("busy"),
                "runtime_resources_empty": (result.get("runtime_state") or {}).get("resources_empty"),
                "steps": (result.get("execution_record") or {}).get("steps"),
            },
            "raw_audit": raw_audit,
        }
        records.write(_line(record))
        line_count += 1
        for key, record_type in (("pose_history", "pose"), ("contact_samples", "contact"),
                                 ("task_events", "event")):
            for sample in audit_input[key]:
                # Preserve the entire source sample, including rejected,
                # stale, out-of-order and invalid entries.
                records.write(_line({"record_type": record_type, "run_id": run_id, "sample": sample}))
                line_count += 1
        if raw_audit is not None:
            records.write(_line({"record_type": "audit", "run_id": run_id, "audit": raw_audit}))
            line_count += 1
        runs.append({
            "run_id": run_id, "seed": trial.get("seed"), "code_commit": code_commit,
            "process_exit_code": trial.get("process_exit_code"),
            "process_timed_out": trial.get("process_timed_out"),
            "result_present": True, "result_sha256": record["result_sha256"],
            "task_status": result.get("status"), "task_success": result.get("success"),
            "error_code": result.get("error_code"), "execution": record["execution"],
            "raw_audit": raw_audit, "export_recompute_matches_raw": None,
        })

    uncompressed = records.getvalue()
    compressed_buffer = io.BytesIO()
    with gzip.GzipFile(fileobj=compressed_buffer, mode="wb", mtime=0, filename="samples.jsonl") as zipped:
        zipped.write(uncompressed)
    compressed = compressed_buffer.getvalue()
    manifest = {
        "format_version": FORMAT_VERSION,
        "audit_program_version": AUDIT_VERSION,
        "code_commit": code_commit,
        "batch_id": summary.get("batch_id"),
        "acceptance_config": acceptance,
        "acceptance_file_sha256": _sha256_bytes(acceptance_bytes),
        "source_workspace_clean_before_trials": summary.get("workspace_clean_before_trials"),
        "data_file": "samples.jsonl.gz",
        "data_compression": "gzip",
        "data_row_count": line_count,
        "data_uncompressed_bytes": len(uncompressed),
        "data_uncompressed_sha256": _sha256_bytes(uncompressed),
        "data_compressed_bytes": len(compressed),
        "data_compressed_sha256": _sha256_bytes(compressed),
        "run_count": len(runs),
        "runs": runs,
        "export_recompute_matches_raw": False,
    }
    def write_archive():
        manifest_bytes = _line(manifest)
        with tarfile.open(output_path, mode="w", format=tarfile.PAX_FORMAT) as archive:
            for name, content in (("manifest.json", manifest_bytes), ("acceptance.json", acceptance_bytes),
                                  ("samples.jsonl.gz", compressed)):
                info = tarfile.TarInfo(name)
                info.size = len(content)
                info.mtime = 0
                info.uid = 0
                info.gid = 0
                info.uname = ""
                info.gname = ""
                info.mode = 0o644
                archive.addfile(info, io.BytesIO(content))

    write_archive()
    first_pass = recompute_archive(output_path)
    if not first_pass["all_recomputations_match"]:
        raise ValueError("exported audit results differ from the raw result JSON audit")
    manifest["export_recompute_matches_raw"] = True
    write_archive()
    verified = recompute_archive(output_path)
    if not verified["all_recomputations_match"]:
        raise ValueError("final package failed offline audit round-trip verification")
    checksum_path = output_path.with_name(output_path.name + ".sha256")
    checksum_path.write_text(f"{_sha256_file(output_path)}  {output_path.name}\n", encoding="ascii")
    return {"manifest": manifest, "archive": str(output_path), "archive_bytes": output_path.stat().st_size,
            "archive_sha256": _sha256_file(output_path), "checksum_file": str(checksum_path)}


def recompute_archive(archive_path):
    """Recompute all audits from package samples using only Python stdlib + this auditor."""
    with tarfile.open(archive_path, mode="r:") as archive:
        manifest = json.load(archive.extractfile("manifest.json"))
        acceptance_bytes = archive.extractfile("acceptance.json").read()
        data_bytes = archive.extractfile("samples.jsonl.gz").read()
    if manifest.get("format_version") != FORMAT_VERSION:
        raise ValueError("unsupported evidence package format")
    if manifest.get("audit_program_version") != AUDIT_VERSION:
        raise ValueError(f"package audit version {manifest.get('audit_program_version')} != local {AUDIT_VERSION}")
    if _sha256_bytes(acceptance_bytes) != manifest.get("acceptance_file_sha256"):
        raise ValueError("acceptance configuration checksum mismatch")
    if _sha256_bytes(data_bytes) != manifest.get("data_compressed_sha256"):
        raise ValueError("compressed sample checksum mismatch")
    with gzip.GzipFile(fileobj=io.BytesIO(data_bytes), mode="rb") as zipped:
        uncompressed = zipped.read()
    if _sha256_bytes(uncompressed) != manifest.get("data_uncompressed_sha256"):
        raise ValueError("uncompressed JSONL checksum mismatch")
    acceptance = json.loads(acceptance_bytes)
    grouped = {}
    for line in uncompressed.splitlines():
        if not line:
            continue
        value = json.loads(line)
        run_id = value.get("run_id")
        if not isinstance(run_id, str):
            raise ValueError("record is missing run_id")
        bucket = grouped.setdefault(run_id, {"pose_history": [], "contact_samples": [], "task_events": [],
                                              "audit": None, "meta": None})
        if value["record_type"] == "run":
            bucket["meta"] = value
        elif value["record_type"] == "pose":
            bucket["pose_history"].append(value["sample"])
        elif value["record_type"] == "contact":
            bucket["contact_samples"].append(value["sample"])
        elif value["record_type"] == "event":
            bucket["task_events"].append(value["sample"])
        elif value["record_type"] == "audit":
            bucket["audit"] = value["audit"]
        else:
            raise ValueError(f"unknown record_type {value['record_type']}")
    outcomes = []
    for run in manifest.get("runs", []):
        run_id = run["run_id"]
        bucket = grouped.get(run_id)
        if not run.get("result_present") or bucket is None or bucket["meta"] is None:
            outcomes.append({"run_id": run_id, "recomputed_audit": None, "matches_raw": None})
            continue
        meta = bucket["meta"]
        result = {key: meta.get(key) for key in (
            "run_id", "seed", "code_commit", "task_status", "task_success", "error_code",
            "object_id", "target_id", "final_sim_time_ns", "final_receipt_monotonic_ns", "final_epoch")}
        result["status"] = result.pop("task_status")
        result["success"] = result.pop("task_success")
        result.update({"pose_history": bucket["pose_history"], "contact_samples": bucket["contact_samples"],
                       "task_events": bucket["task_events"]})
        recomputed = audit_physics(result, acceptance)
        matches = recomputed == run.get("raw_audit") == bucket["audit"]
        outcomes.append({"run_id": run_id, "recomputed_audit": recomputed, "matches_raw": matches})
    return {"format_version": FORMAT_VERSION, "audit_program_version": AUDIT_VERSION,
            "code_commit": manifest.get("code_commit"), "run_count": len(outcomes),
            "all_recomputations_match": all(item["matches_raw"] is True or item["matches_raw"] is None
                                               for item in outcomes),
            "outcomes": outcomes}
