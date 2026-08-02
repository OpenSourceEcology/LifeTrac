"""F11 — gating the per-gap keyframe request behind LIFETRAC_KF_ON_SEQ_GAP.

The gap-tolerant canvas (IP-PlanRev) requests a keyframe on EVERY base_seq
gap, including a single lost delta frame. F10's 0x6C stale-tile path now
detects and repairs exactly that damage tile-by-tile, so the per-gap
keyframe is largely redundant — but per the F10 protocol the default stays
ON until the A/B is measured on air. These tests pin the gate mechanics:

- Canvas reports structured causes (seq_gap / tile_error) so web_ui's
  policy never string-matches reasons.
- web_ui suppresses the publish ONLY for pure-gap requests with the gate
  off; cold start, grid mismatch, and tile errors always pass through.
"""

import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from image_pipeline import codec_decode                      # noqa: E402
from image_pipeline.canvas import Canvas                     # noqa: E402
from image_pipeline.frame_format import (                    # noqa: E402
    CODEC_MONO_G4,
    CODEC_WEBP,
    TileBlob,
    TileDeltaFrame,
)

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


def _frame(seq, indices, *, keyframe=False, codec=CODEC_WEBP,
           grid=(12, 8, 32)):
    gw, gh, px = grid
    return TileDeltaFrame(
        frame_kind=1 if keyframe else 0, base_seq=seq,
        grid_w=gw, grid_h=gh, tile_px=px,
        changed_indices=list(indices),
        tiles=[TileBlob(index=i, tx=i % gw, ty=i // gw, blob=b"RIFFwebp")
               for i in indices],
        codec=codec,
    )


class CanvasStructuredCauseTests(unittest.TestCase):
    def setUp(self) -> None:
        self.canvas = Canvas(clock_ms=lambda: 1000)
        self.canvas.apply(_frame(0, range(96), keyframe=True))

    def test_clean_delta_sets_no_flags(self) -> None:
        upd = self.canvas.apply(_frame(1, [3]))
        self.assertFalse(upd.request_keyframe)
        self.assertFalse(upd.seq_gap)
        self.assertFalse(upd.tile_error)

    def test_gap_sets_seq_gap_only(self) -> None:
        upd = self.canvas.apply(_frame(5, [3]))     # expected 1
        self.assertTrue(upd.request_keyframe)
        self.assertTrue(upd.seq_gap)
        self.assertFalse(upd.tile_error)
        self.assertEqual(upd.updated_indices, [3], "gap still applies tiles")

    def test_tile_error_sets_tile_error_only(self) -> None:
        boom = mock.Mock(side_effect=ValueError("bad blob"))
        with mock.patch.dict(codec_decode._TRANSCODERS,
                             {CODEC_MONO_G4: boom}):
            upd = self.canvas.apply(_frame(1, [3], codec=CODEC_MONO_G4))
        self.assertTrue(upd.request_keyframe)
        self.assertFalse(upd.seq_gap)
        self.assertTrue(upd.tile_error)

    def test_gap_plus_tile_error_sets_both(self) -> None:
        boom = mock.Mock(side_effect=ValueError("bad blob"))
        with mock.patch.dict(codec_decode._TRANSCODERS,
                             {CODEC_MONO_G4: boom}):
            upd = self.canvas.apply(_frame(7, [3], codec=CODEC_MONO_G4))
        self.assertTrue(upd.seq_gap)
        self.assertTrue(upd.tile_error)

    def test_cold_start_sets_neither_flag(self) -> None:
        cold = Canvas(clock_ms=lambda: 0)
        upd = cold.apply(_frame(4, [0]))
        self.assertTrue(upd.request_keyframe)
        self.assertFalse(upd.seq_gap)
        self.assertFalse(upd.tile_error)


class WebUiGateTests(unittest.TestCase):
    """Drive web_ui._ingest_tile_delta with the reassembler stubbed to
    hand back a fully-parsed frame, and count req_keyframe publishes."""

    KF_TOPIC = "lifetrac/v25/cmd/req_keyframe"

    def setUp(self) -> None:
        self._saved_gate = web_ui._KF_ON_SEQ_GAP
        self._saved_canvas = web_ui._image_canvas
        web_ui._image_canvas = Canvas(clock_ms=lambda: 1000)
        web_ui._image_publisher.canvas = web_ui._image_canvas
        web_ui.mqtt_client.publish = mock.MagicMock()

    def tearDown(self) -> None:
        web_ui._KF_ON_SEQ_GAP = self._saved_gate
        web_ui._image_canvas = self._saved_canvas
        web_ui._image_publisher.canvas = self._saved_canvas

    def _ingest(self, frame) -> None:
        with mock.patch.object(web_ui._image_reassembler, "feed",
                               return_value=frame):
            web_ui._ingest_tile_delta(b"\x00")

    def _kf_publishes(self):
        return [c for c in web_ui.mqtt_client.publish.call_args_list
                if c.args and c.args[0] == self.KF_TOPIC]

    def test_gate_off_suppresses_pure_gap(self) -> None:
        web_ui._KF_ON_SEQ_GAP = False
        self._ingest(_frame(0, range(96), keyframe=True))
        self._ingest(_frame(5, [3]))                       # gap: expected 1
        self.assertEqual(self._kf_publishes(), [])
        self.assertFalse(web_ui._image_publisher.needs_keyframe)
        # The gap's own tiles were still applied.
        self.assertEqual(web_ui._image_canvas._tiles[3].arrived_ms, 1000)

    def test_gate_on_default_still_requests_on_gap(self) -> None:
        web_ui._KF_ON_SEQ_GAP = True
        self._ingest(_frame(0, range(96), keyframe=True))
        self._ingest(_frame(5, [3]))
        pubs = self._kf_publishes()
        self.assertEqual(len(pubs), 1)
        self.assertIn(b"base_seq gap", pubs[0].args[1])

    def test_gate_off_still_requests_on_cold_start(self) -> None:
        web_ui._KF_ON_SEQ_GAP = False
        self._ingest(_frame(4, [0]))                       # no keyframe yet
        pubs = self._kf_publishes()
        self.assertEqual(len(pubs), 1)
        self.assertIn(b"before any keyframe", pubs[0].args[1])

    def test_gate_off_still_requests_on_gap_with_tile_error(self) -> None:
        web_ui._KF_ON_SEQ_GAP = False
        self._ingest(_frame(0, range(96), keyframe=True))
        boom = mock.Mock(side_effect=ValueError("bad blob"))
        with mock.patch.dict(codec_decode._TRANSCODERS,
                             {CODEC_MONO_G4: boom}):
            self._ingest(_frame(7, [3], codec=CODEC_MONO_G4))
        self.assertEqual(len(self._kf_publishes()), 1)

    def test_default_env_is_gate_on(self) -> None:
        self.assertTrue(self._saved_gate,
                        "F11 protocol: default must stay ON until the "
                        "on-air A/B is measured")


if __name__ == "__main__":
    unittest.main()
