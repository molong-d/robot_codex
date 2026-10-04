#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import sys
import unittest


module_path = Path(__file__).resolve().parents[1] / "scripts" / "gz_scene_sync.py"
spec = importlib.util.spec_from_file_location("gz_scene_sync", module_path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
sys.path.insert(0, str(module_path.parent))
from source_time import SourceTimeGuard


class SourceTimestampTests(unittest.TestCase):
    def test_support_contact_collision_matrix_is_explicit(self):
        from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix

        source = AllowedCollisionMatrix()
        source.entry_names = ["panda_link0", "panda_link1"]
        source.entry_values = [AllowedCollisionEntry(enabled=[True, False]),
                              AllowedCollisionEntry(enabled=[False, True])]
        source.default_entry_names = ["panda_link0"]
        source.default_entry_values = [False]
        matrix = module._collision_matrix_with_allowed_pairs(
            source, (("sim_table", "workpiece"), ("sim_tray", "workpiece"),
                    ("workpiece", "panda_leftfinger"),
                    ("workpiece", "panda_rightfinger")))
        self.assertEqual(matrix.entry_names, ["panda_link0", "panda_link1", "sim_table", "workpiece",
                                              "sim_tray", "panda_leftfinger", "panda_rightfinger"])
        self.assertFalse(matrix.entry_values[0].enabled[1], "existing self-collision policy was overwritten")
        self.assertFalse(matrix.entry_values[1].enabled[0], "existing matrix symmetry was overwritten")
        self.assertTrue(matrix.entry_values[2].enabled[3])
        self.assertTrue(matrix.entry_values[3].enabled[2])
        self.assertTrue(matrix.entry_values[3].enabled[4])
        self.assertTrue(matrix.entry_values[4].enabled[3])
        self.assertTrue(matrix.entry_values[3].enabled[5])
        self.assertTrue(matrix.entry_values[5].enabled[3])
        self.assertTrue(matrix.entry_values[3].enabled[6])
        self.assertTrue(matrix.entry_values[6].enabled[3])
        self.assertEqual(matrix.default_entry_names, ["panda_link0"])
        self.assertEqual(matrix.default_entry_values, [False])

    def test_current_sample_is_fresh(self):
        self.assertTrue(module._fresh_source_stamp(1_000_000_000, 1_000_000_000))

    def test_cached_sample_expires_against_sim_clock(self):
        self.assertFalse(module._fresh_source_stamp(1_000_000_000, 1_600_000_001))

    def test_future_or_zero_stamp_is_rejected(self):
        self.assertFalse(module._fresh_source_stamp(1_100_000_000, 1_000_000_000))
        self.assertFalse(module._fresh_source_stamp(0, 1_000_000_000))

    def test_new_epoch_waits_for_old_stream_watermark(self):
        guard = SourceTimeGuard()
        self.assertTrue(guard.observe("contact", 2_000_000_000, 2_000_000_000)[0])
        accepted, epoch, reason = guard.observe("contact", 100_000_000, 100_000_000)
        self.assertFalse(accepted)
        self.assertEqual(epoch, 1)
        self.assertEqual(reason, "pre_reset_source_time")
        self.assertTrue(guard.observe("contact", 2_100_000_000, 2_100_000_000)[0])

    def test_old_source_stamp_with_forward_clock_is_rejected(self):
        guard = SourceTimeGuard()
        self.assertTrue(guard.observe("pose", 2_000_000_000, 2_000_000_000)[0])
        self.assertEqual(guard.observe("pose", 1_900_000_000, 2_100_000_000),
                         (False, 0, "duplicate_or_out_of_order"))
        self.assertEqual(guard.last_source_by_stream["pose"], 2_000_000_000)

    def test_pause_duplicate_does_not_refresh_sample(self):
        guard = SourceTimeGuard()
        first = guard.observe("joint", 1_000_000_000, 1_000_000_000)
        duplicate = guard.observe("joint", 1_000_000_000, 1_000_000_000)
        self.assertTrue(first[0])
        self.assertEqual(duplicate[2], "duplicate_or_out_of_order")

    def test_small_clock_jitter_is_not_an_epoch(self):
        guard = SourceTimeGuard()
        guard.observe("joint", 2_000_000_000, 2_000_000_000)
        accepted, epoch, reason = guard.observe("joint", 1_950_000_000, 1_950_000_000)
        self.assertFalse(accepted)
        self.assertEqual((epoch, reason), (0, "clock_jitter"))

    def test_future_stale_and_old_post_reset_samples_are_rejected(self):
        guard = SourceTimeGuard()
        self.assertTrue(guard.observe("joint", 2_000_000_000, 2_000_000_000)[0])
        self.assertEqual(guard.observe("joint", 2_100_000_000, 2_000_000_000)[2], "future_source_time")
        self.assertEqual(guard.observe("joint", 1_000_000_000, 2_000_000_001)[2], "stale_source_time")
        self.assertEqual(guard.observe("joint", 100_000_000, 100_000_000),
                         (False, 1, "pre_reset_source_time"))
        self.assertEqual(guard.observe("joint", 1_950_000_000, 1_950_000_000)[2], "pre_reset_source_time")
        self.assertTrue(guard.observe("joint", 2_100_000_000, 2_100_000_000)[0])


if __name__ == "__main__":
    unittest.main()
