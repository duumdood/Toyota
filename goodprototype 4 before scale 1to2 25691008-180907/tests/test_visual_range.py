"""Visual range is independent of the firmware's capped control gap."""
import sys
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from road_board_io import BoardTelemetry, BoardView, visual_gap


class Receiver:
    def snapshot(self):
        return self.packet, self.received, 'BOARD CONNECTED', 0


class VisualRangeTests(unittest.TestCase):
    def setUp(self):
        self.receiver = Receiver()
        self.view = BoardView()
        self.packet = BoardTelemetry(
            0, 0, 0, 0, 0, 0, 70, None, False, False, False, False,
            1600, 1200, 0, 0, 0)
        self.now = 1.0

    def send(self, **changes):
        self.now += .05
        self.packet = replace(self.packet, sequence=self.packet.sequence + 1,
                              board_ms=self.packet.board_ms + 50, **changes)
        self.receiver.packet = self.packet
        self.receiver.received = self.now
        self.view.refresh(self.receiver, now=self.now)

    def test_boundary_and_unclamped_distance(self):
        for mm, expected, visible in [(1200, 120, False), (801, 80.1, False),
                                      (800, 80, True), (400, 40, True)]:
            with self.subTest(mm=mm):
                self.send(distance_mm=mm)
                self.view.refresh(self.receiver, now=self.now + .05)
                self.assertAlmostEqual(visual_gap(self.packet), expected)
                self.assertAlmostEqual(self.view.scene_gap_m, expected)
                self.assertEqual(self.view.scene_visible, visible)
                self.assertEqual(self.packet.gap_m, 70)  # No control mutation.

    def test_reentry_snaps_to_new_distance(self):
        self.send(distance_mm=100)
        self.send(distance_mm=1200)
        self.assertFalse(self.view.scene_visible)
        self.send(distance_mm=800)
        self.assertTrue(self.view.scene_visible)
        self.assertEqual(self.view.scene_gap_m, 80)

    def test_invalid_fault_and_disconnect_hide_car(self):
        self.send(distance_mm=400)
        self.assertTrue(self.view.scene_visible)
        self.send(range_status=1, distance_mm=None)
        self.assertIsNone(visual_gap(self.packet))
        self.assertFalse(self.view.scene_visible)
        self.send(range_status=2, distance_mm=400)
        self.assertFalse(self.view.scene_visible)
        self.send(range_status=0, fault=True)
        self.assertFalse(self.view.scene_visible)
        self.send(fault=False)
        self.assertTrue(self.view.scene_visible)
        self.view.refresh(self.receiver, now=self.now + .6)
        self.assertFalse(self.view.scene_visible)

    def test_sequence_result_freezes_visual_gap_until_reset(self):
        self.send(distance_mm=400, done=True, done_reason=1)
        self.send(distance_mm=1200)
        self.assertEqual(self.view.scene_gap_m, 40)
        self.assertTrue(self.view.scene_visible)
        self.send(reset_id=1, done=False, done_reason=0)
        self.assertEqual(self.view.scene_gap_m, 120)
        self.assertFalse(self.view.scene_visible)


if __name__ == '__main__':
    unittest.main()
