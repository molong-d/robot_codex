#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import unittest


module_path = Path(__file__).resolve().parents[1] / "scripts" / "gz_scene_sync.py"
spec = importlib.util.spec_from_file_location("gz_scene_sync", module_path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class SourceTimestampTests(unittest.TestCase):
    def test_current_sample_is_fresh(self):
        self.assertTrue(module._fresh_source_stamp(1_000_000_000, 1_000_000_000))

    def test_cached_paused_sample_expires_against_steady_sim_clock(self):
        self.assertFalse(module._fresh_source_stamp(1_000_000_000, 1_600_000_001))

    def test_future_or_zero_stamp_is_rejected(self):
        self.assertFalse(module._fresh_source_stamp(1_100_000_000, 1_000_000_000))
        self.assertFalse(module._fresh_source_stamp(0, 1_000_000_000))

    def test_new_epoch_after_reset_is_accepted(self):
        self.assertTrue(module._fresh_source_stamp(100_000_000, 100_000_000))


if __name__ == "__main__":
    unittest.main()
