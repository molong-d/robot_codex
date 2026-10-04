#!/usr/bin/env python3
"""Contract regressions for dual-finger contact evidence."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from contact_semantics import ContactState, attachment_after_contact, classify_contacts, release_confirmed


LEFT = ["panda::workpiece", "panda_leftfinger::collision"]
RIGHT = ["panda::workpiece", "panda_rightfinger::collision"]


class ContactSemanticTests(unittest.TestCase):
    def test_distinguishes_each_observed_contact_state(self):
        self.assertEqual(classify_contacts([LEFT]), ContactState.LEFT_ONLY)
        self.assertEqual(classify_contacts([RIGHT]), ContactState.RIGHT_ONLY)
        self.assertEqual(classify_contacts([LEFT, RIGHT]), ContactState.BOTH)
        self.assertEqual(classify_contacts([]), ContactState.NONE)

    def test_missing_stale_wrong_epoch_and_out_of_order_are_unknown(self):
        for sample in (None, {"fresh": False, "epoch": 2, "sequence": 3, "state": "none"},
                       {"fresh": True, "epoch": 1, "sequence": 3, "state": "none"},
                       {"fresh": True, "epoch": 2, "sequence": 1, "state": "none"}):
            self.assertEqual(release_confirmed(sample, expected_epoch=2,
                                               after_source_ns=100, last_sequence=3),
                             ContactState.UNKNOWN)

    def test_single_finger_contact_is_not_release_even_inside_target(self):
        self.assertFalse(release_confirmed(
            {"fresh": True, "epoch": 2, "sequence": 4, "source_ns": 101,
             "state": "left_only"}, expected_epoch=2, after_source_ns=100,
            last_sequence=3) == ContactState.NONE)

    def test_single_finger_does_not_detach_and_both_fingers_do_not_release(self):
        self.assertTrue(attachment_after_contact(True, ContactState.LEFT_ONLY))
        self.assertTrue(attachment_after_contact(True, ContactState.RIGHT_ONLY))
        self.assertTrue(attachment_after_contact(True, ContactState.BOTH))
        self.assertFalse(attachment_after_contact(True, ContactState.NONE))
        self.assertFalse(attachment_after_contact(False, ContactState.UNKNOWN))

    def test_only_post_action_fresh_dual_no_contact_confirms_release(self):
        self.assertEqual(release_confirmed(
            {"fresh": True, "epoch": 2, "sequence": 4, "source_ns": 101,
             "state": "none"}, expected_epoch=2, after_source_ns=100,
            last_sequence=3), ContactState.NONE)


if __name__ == "__main__":
    unittest.main()
