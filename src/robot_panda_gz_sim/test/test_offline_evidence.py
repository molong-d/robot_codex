#!/usr/bin/env python3
"""Offline evidence archive round-trip and lossless rejection-sample tests."""

import gzip
import importlib.util
import json
import sys
import tarfile
import tempfile
import unittest
from pathlib import Path

root = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(root / "scripts"))
from gazebo_evidence_package import export_batch, recompute_archive
fixture_path = Path(__file__).resolve().parent / "test_gazebo_trials.py"
fixture_spec = importlib.util.spec_from_file_location("gazebo_trial_fixtures", fixture_path)
fixture_module = importlib.util.module_from_spec(fixture_spec)
fixture_spec.loader.exec_module(fixture_module)


class OfflineEvidenceTests(unittest.TestCase):
    def test_package_recomputes_identically_and_preserves_rejected_samples(self):
        with tempfile.TemporaryDirectory() as temporary:
            root_tmp = Path(temporary)
            batch = root_tmp / ".review-runs" / "batch"
            trial_dir = batch / "trial-001-seed-42"
            trial_dir.mkdir(parents=True)
            code_commit = "a" * 40
            result = fixture_module.successful_physics()
            result["code_commit"] = code_commit
            result["pose_history"].append({"run_id": "test-run", "stream": "pose:workpiece",
                "object_id": "workpiece", "frame_id": "world", "source_frame_id": "world",
                "source_stamp_ns": 2_950_000_000, "sim_time_ns": 2_960_000_000,
                "receipt_monotonic_ns": 9_960_000_000, "epoch": 0, "sequence": 99,
                "accepted": False, "reject_reason": "duplicate_or_out_of_order", "xyz": [0, 0, 0]})
            result_path = trial_dir / "result.json"
            result_path.write_text(json.dumps(result), encoding="utf-8")
            summary = {"batch_id": "batch", "code_commit": code_commit,
                "workspace_clean_before_trials": True, "trials": [{
                    "run_id": "test-run", "seed": 42, "code_commit": code_commit,
                    "process_exit_code": 0, "process_timed_out": False, "process_timeout_s": 240,
                    "result_json": ".review-runs/batch/trial-001-seed-42/result.json"}]}
            (batch / "summary.json").write_text(json.dumps(summary), encoding="utf-8")
            acceptance_path = root / "src/robot_panda_gz_sim/config/acceptance.json"
            output = root_tmp / "physical.tar"
            package = export_batch(batch, output, acceptance_path)
            recomputed = recompute_archive(output)
            self.assertTrue(package["manifest"]["export_recompute_matches_raw"])
            self.assertTrue(recomputed["all_recomputations_match"])
            with tarfile.open(output, "r:") as archive:
                compressed = archive.extractfile("samples.jsonl.gz").read()
            rows = [json.loads(line) for line in gzip.decompress(compressed).splitlines()]
            rejected = [row for row in rows if row["record_type"] == "pose" and
                        row["sample"].get("accepted") is False]
            self.assertEqual(len(rejected), 1, "rejected samples must be exported without filtering")


if __name__ == "__main__":
    unittest.main()
