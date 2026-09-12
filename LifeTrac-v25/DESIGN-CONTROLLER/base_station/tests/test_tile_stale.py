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
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lora_proto import (  # noqa: E402
    CMD_OP_TILE_STALE,
    pack_command_frame,
    parse_command_frame,
    pack_tile_stale,
    parse_tile_stale,
)
from image_pipeline.canvas import Canvas  # noqa: E402

# web_ui connects to the MQTT broker at import time (and RAISES after 30 s
# if none is listening — CI has no broker), so it must be imported under the
# same paho stub the other web_ui tests use. Skip cleanly on pure-firmware
# checkouts without fastapi/paho.
try:
    import paho.mqtt.client  # noqa: F401
    import fastapi           # noqa: F401
except ImportError:
    raise unittest.SkipTest("paho-mqtt + fastapi required for web_ui import")

with mock.patch("paho.mqtt.client.Client") as _mqtt_class:
    _instance = _mqtt_class.return_value
    _instance.connect = mock.MagicMock()
    _instance.loop_start = mock.MagicMock()
    _instance.subscribe = mock.MagicMock()
    _instance.publish = mock.MagicMock()
    import importlib
    import web_ui
    importlib.reload(web_ui)   # rebind module-level mqtt stub
    compute_stale_tiles = web_ui.compute_stale_tiles
    summarize_tile_ages = web_ui.summarize_tile_ages


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


class TileAgeSummaryTests(unittest.TestCase):
    """RS-6.1 aggregate age telemetry (published on status/tile_age)."""

    def _canvas(self, now_ms: int) -> Canvas:
        c = Canvas(clock_ms=lambda: now_ms)
        c._has_keyframe = True
        return c

    def test_none_before_keyframe(self) -> None:
        c = Canvas(clock_ms=lambda: 1000)
        self.assertIsNone(summarize_tile_ages(c, 1000, 20_000))

    def test_percentiles_and_stale_count(self) -> None:
        c = self._canvas(100_000)
        for i, t in enumerate(c._tiles):
            t.arrived_ms = 100_000 - (i + 1) * 100   # ages 100..9600 ms
        s = summarize_tile_ages(c, 100_000, 5_000)
        self.assertEqual(s["n_tiles"], 96)
        self.assertEqual(s["missing"], 0)
        self.assertEqual(s["max_ms"], 9_600)
        self.assertEqual(s["p50_ms"], 4_900)          # sorted ages, idx 48
        self.assertEqual(s["p95_ms"], 9_200)          # idx 91
        # ages 5100..9600 exceed the 5 s threshold -> 46 stale
        self.assertEqual(s["stale"], 46)

    def test_never_arrived_counts_in_missing_and_stale(self) -> None:
        c = self._canvas(50_000)
        for t in c._tiles:
            t.arrived_ms = 49_000
        c._tiles[7].arrived_ms = 0
        s = summarize_tile_ages(c, 50_000, 20_000)
        self.assertEqual(s["missing"], 1)
        self.assertEqual(s["stale"], 1, "missing tiles count as stale")
        self.assertEqual(s["max_ms"], 1_000,
                         "missing tiles stay out of the percentiles")


if __name__ == "__main__":
    unittest.main()


class MotionAwareHorizonTests(unittest.TestCase):
    """RS-4.15: the horizon follows the measured sweep rotation."""

    def setUp(self) -> None:
        self.refresh_intervals = web_ui.refresh_intervals
        self.rotation_estimate_ms = web_ui.rotation_estimate_ms
        self.effective = web_ui.effective_stale_horizon_ms
        self.due = web_ui.stale_report_due

    def test_refresh_intervals_only_for_advanced_arrivals(self) -> None:
        prev = [1000, 2000, 0, 5000, 7000]
        cur = [1000, 2300, 900, 0, 6000]          # same, advanced, new, gone, backwards
        self.assertEqual(self.refresh_intervals(prev, cur), [(1, 300)])

    def test_refresh_intervals_tolerates_grid_growth(self) -> None:
        self.assertEqual(self.refresh_intervals([100], [100, 200]), [])

    def test_rotation_is_the_slowest_recent_refresh(self) -> None:
        samples = [(1000, 30000), (50000, 500), (60000, 12000)]
        self.assertEqual(self.rotation_estimate_ms(samples, 60000, 60000), 30000)
        # the 30 s interval ages out of the window: motion tiles dominate
        self.assertEqual(self.rotation_estimate_ms(samples, 62000, 60000), 12000)
        self.assertIsNone(self.rotation_estimate_ms([], 0, 60000))

    def test_lost_tiles_cannot_inflate_the_estimate(self) -> None:
        # A tile that never refreshes produces no interval at all.
        prev = [1000, 1000]
        cur = [1000, 31000]                        # tile 0 lost, tile 1 swept
        ivs = self.refresh_intervals(prev, cur)
        self.assertEqual([iv for _, iv in ivs], [30000])

    def test_effective_horizon(self) -> None:
        self.assertEqual(self.effective(20000, None, 1.5, 120000), 20000)
        self.assertEqual(self.effective(20000, 30000, 0.0, 120000), 20000)
        self.assertEqual(self.effective(20000, 30000, 1.5, 120000), 45000)
        self.assertEqual(self.effective(20000, 5000, 1.5, 120000), 20000)
        self.assertEqual(self.effective(20000, 200000, 1.5, 120000), 120000)

    def test_static_scene_keeps_repair_within_two_rotations(self) -> None:
        # Bench static scene: ~30 s rotation -> 45 s horizon, well under the cap.
        self.assertLessEqual(self.effective(20000, 30000, 1.5, 120000), 60000)

    def test_report_due(self) -> None:
        self.assertTrue(self.due(None, 0.0, b"a", 100.0, 10.0))
        self.assertTrue(self.due(b"a", 95.0, b"b", 100.0, 10.0))
        self.assertFalse(self.due(b"a", 95.0, b"a", 100.0, 10.0))
        self.assertTrue(self.due(b"a", 89.0, b"a", 100.0, 10.0))
