"""F10 — the 0x6C stale-tile report: proto roundtrip and the base-side scan.

The reassembly-timeout keyframe self-heal was measured net harmful (n=2/side:
+1.88 pts loss, −16% frames, +31% timeouts) but could not simply be deleted:
the canvas is persistent, so a lost tile update displays stale imagery until
the encoder happens to resend. The 0x6C report closes that loop from the only
side that knows what arrived, 84× cheaper than the keyframe it replaces, and
rides the tractor's existing age-escalation machinery.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lora_proto import (  # noqa: E402
    CMD_OP_TILE_STALE,
    pack_command_frame,
    parse_command_frame,
    pack_tile_stale,
    parse_tile_stale,
)
from image_pipeline.canvas import Canvas  # noqa: E402
from web_ui import compute_stale_tiles    # noqa: E402


class ProtoRoundtripTests(unittest.TestCase):

    def test_roundtrip(self) -> None:
        body = pack_tile_stale(0x1234, [0, 7, 8, 42, 95], 96)
        self.assertEqual(len(body), 2 + 12, "12x8 grid = 14 B body")
        parsed = parse_tile_stale(body)
        self.assertIsNotNone(parsed)
        base_seq, tiles = parsed
        self.assertEqual(base_seq, 0x1234)
        self.assertEqual(tiles, [0, 7, 8, 42, 95])

    def test_empty_bitmap(self) -> None:
        base_seq, tiles = parse_tile_stale(pack_tile_stale(1, [], 96))
        self.assertEqual(tiles, [])

    def test_out_of_range_index_raises(self) -> None:
        with self.assertRaises(ValueError):
            pack_tile_stale(0, [96], 96)

    def test_truncated_body_returns_none(self) -> None:
        self.assertIsNone(parse_tile_stale(b"\x01\x00"))

    def test_rides_the_command_frame(self) -> None:
        body = pack_tile_stale(7, [3], 96)
        frame = pack_command_frame(CMD_OP_TILE_STALE, body)
        op, args = parse_command_frame(frame)
        self.assertEqual(op, CMD_OP_TILE_STALE)
        self.assertEqual(parse_tile_stale(args), (7, [3]))

    def test_wire_cost_is_one_minimum_command_frame_class(self) -> None:
        """14 B body + 2 B cmd hdr + 8 B hop hdr = 24 B on air ≈ 15.4 ms at
        DTS — the 84x-cheaper-than-a-keyframe claim rests on this size."""
        body = pack_tile_stale(0, list(range(96)), 96)
        self.assertEqual(2 + len(body), 16)


class StaleScanTests(unittest.TestCase):

    def _canvas_with_keyframe(self, now_ms: int) -> Canvas:
        c = Canvas(clock_ms=lambda: now_ms)
        c._has_keyframe = True
        return c

    def test_no_keyframe_reports_nothing(self) -> None:
        c = Canvas(clock_ms=lambda: 50_000)
        self.assertEqual(compute_stale_tiles(c, 50_000, 20_000), [],
                         "an unpopulated canvas has nothing to be stale")

    def test_fresh_tiles_not_reported(self) -> None:
        c = self._canvas_with_keyframe(100_000)
        for t in c._tiles:
            t.arrived_ms = 95_000
        self.assertEqual(compute_stale_tiles(c, 100_000, 20_000), [])

    def test_stale_and_never_arrived_are_reported(self) -> None:
        c = self._canvas_with_keyframe(100_000)
        for t in c._tiles:
            t.arrived_ms = 95_000
        c._tiles[5].arrived_ms = 70_000     # 30 s old > 20 s horizon
        c._tiles[9].arrived_ms = 0          # never arrived post-keyframe
        self.assertEqual(compute_stale_tiles(c, 100_000, 20_000), [5, 9])

    def test_threshold_boundary_is_exclusive(self) -> None:
        c = self._canvas_with_keyframe(100_000)
        for t in c._tiles:
            t.arrived_ms = 80_000           # exactly 20 s: NOT stale (>)
        self.assertEqual(compute_stale_tiles(c, 100_000, 20_000), [])


if __name__ == "__main__":
    unittest.main()
