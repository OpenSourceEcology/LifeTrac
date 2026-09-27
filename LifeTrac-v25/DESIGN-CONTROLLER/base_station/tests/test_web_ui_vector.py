"""web_ui's VS1 routing (VECTOR_SCENE.md §7.1, §7.2, §6).

A codec-6 frame goes to the vector store and never to ``Canvas.apply``, never
requests a keyframe, flips ``encode_mode``/``safety_detector`` and shows up
as ``vector_scene`` in the snapshot; a tile frame afterwards restores pixel
mode; garbage bodies are counted, not raised; the settings endpoint accepts
``vector`` and uses its own 60–100 detail dial instead of the carried tile
quality. Same import recipe as test_web_ui_radio_profile.py.
"""
from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

try:
    import paho.mqtt.client  # noqa: F401
    import fastapi           # noqa: F401
    from fastapi.testclient import TestClient  # noqa: F401
except ImportError:  # pragma: no cover
    raise unittest.SkipTest("paho-mqtt + fastapi required for web_ui vector tests")

from image_pipeline.frame_format import (  # noqa: E402
    CODEC_VECTOR, TileDeltaFrame, encode_tile_delta_frame, parse_tile_delta_frame,
)
from image_pipeline.vector_scene import codec as vs  # noqa: E402
from tests.test_kf_on_seq_gap import _frame as tile_frame  # noqa: E402
from tests.test_vector_codec import worked_scene  # noqa: E402

KF_TOPIC = "lifetrac/v25/cmd/req_keyframe"


def vector_frame(body: bytes, key: bool = True, seq: int = 1) -> TileDeltaFrame:
    wire = encode_tile_delta_frame(TileDeltaFrame(
        frame_kind=1 if key else 0, base_seq=seq, grid_w=12, grid_h=8, tile_px=32,
        codec=CODEC_VECTOR, vector_body=body))
    return parse_tile_delta_frame(wire)


class WebUiVectorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls._tmp = tempfile.TemporaryDirectory()
        os.environ["LIFETRAC_PIN"] = "424242"
        os.environ["LIFETRAC_ENCODE_MODE_STORE"] = str(Path(cls._tmp.name) / ".encode_mode")
        os.environ["LIFETRAC_RADIO_PROFILE_STORE"] = str(Path(cls._tmp.name) / ".radio_profile")
        with mock.patch("paho.mqtt.client.Client") as mqtt_class:
            instance = mqtt_class.return_value
            instance.connect = mock.MagicMock()
            instance.loop_start = mock.MagicMock()
            instance.subscribe = mock.MagicMock()
            pub_info = mock.MagicMock()
            pub_info.rc = 0
            instance.publish = mock.MagicMock(return_value=pub_info)
            import importlib
            import web_ui
            importlib.reload(web_ui)
            cls.web_ui = web_ui
            cls.client = TestClient(web_ui.app)
            cls.mqtt = instance

    @classmethod
    def tearDownClass(cls) -> None:
        cls._tmp.cleanup()

    def setUp(self) -> None:
        self.client.post("/api/login", json={"pin": "424242"})
        self.mqtt.publish.reset_mock()
        self.web_ui._vector_store.reset()
        self.web_ui._vector_active = False
        self.web_ui._image_publisher.encode_mode = "full"
        self.web_ui._image_publisher.safety_detector = "pixels"

    def _ingest(self, frame) -> None:
        with mock.patch.object(self.web_ui._image_reassembler, "feed", return_value=frame):
            self.web_ui._ingest_tile_delta(b"\x00")

    def _kf_publishes(self):
        return [c for c in self.mqtt.publish.call_args_list if c.args and c.args[0] == KF_TOPIC]

    def test_codec6_goes_to_the_store_and_never_asks_for_a_keyframe(self) -> None:
        self.assertIsNotNone(self.web_ui._vector_store, "store missing")
        tiles_before = [t.arrived_ms for t in self.web_ui._image_canvas._tiles]
        body = vs.encode_frame(vs.Header(True, 1, 3), worked_scene(), 197)
        self._ingest(vector_frame(body, key=True))
        st = self.web_ui._vector_store.stats
        self.assertEqual((st["frames_rx"], st["frames_applied"], st["frames_bad"]), (1, 1, 0))
        self.assertTrue(self.web_ui._vector_active)
        pub = self.web_ui._image_publisher
        self.assertEqual((pub.encode_mode, pub.safety_detector), ("vector", "no_pixels"))
        self.assertFalse(pub.needs_keyframe)
        self.assertEqual(self._kf_publishes(), [])
        # the photo canvas is untouched: VS frames never reach Canvas.apply
        self.assertEqual([t.arrived_ms for t in self.web_ui._image_canvas._tiles], tiles_before)
        snap = pub.snapshot()
        self.assertIn("vector_scene", snap)
        self.assertIsNotNone(snap["vector_scene"])
        self.assertEqual(snap["vector_scene"]["epoch"], 3)
        self.assertEqual(snap["vector_scene"]["badge"], 7)
        self.assertGreater(len(snap["vector_scene"]["layers"]), 0)
        self.assertIsNone(snap["self_model"])
        self.assertEqual(snap["safety_detector"], "no_pixels")
        json.dumps(snap)                                   # the WS payload must serialise

    def test_tile_frame_after_vector_restores_pixel_mode(self) -> None:
        body = vs.encode_frame(vs.Header(True, 0, 0), worked_scene(), 197)
        self._ingest(vector_frame(body))
        self.assertTrue(self.web_ui._vector_active)
        self._ingest(tile_frame(0, range(96), keyframe=True))
        self.assertFalse(self.web_ui._vector_active)
        pub = self.web_ui._image_publisher
        self.assertEqual((pub.encode_mode, pub.safety_detector), ("full", "pixels"))

    def test_garbage_vector_body_is_counted_not_raised(self) -> None:
        self._ingest(vector_frame(bytes([0xFE, 0x00, 0xFF, 0x12]), key=False))
        st = self.web_ui._vector_store.stats
        self.assertEqual((st["frames_rx"], st["frames_bad"], st["frames_applied"]), (1, 1, 0))
        self.assertTrue(self.web_ui._vector_active)          # still a vector frame on the wire
        self.assertIsNone(self.web_ui._image_publisher.snapshot()["vector_scene"])

    def test_settings_accepts_vector_with_its_own_detail_dial(self) -> None:
        self.assertIn("vector", self.web_ui._ENCODE_MODE_UI_CHOICES)
        self.assertIn("vector", self.web_ui._ENCODE_MODE_CYCLE_ORDER)

        def last_published():
            return [json.loads(c.args[1]) for c in self.mqtt.publish.call_args_list
                    if c.args and c.args[0] == self.web_ui._ENCODE_MODE_TOPIC][-1]

        r = self.client.post("/api/settings/encode_mode", json={"mode": "full", "quality": 40})
        self.assertEqual(r.status_code, 200, r.text)
        self.assertEqual(last_published()["quality"], 40)
        r = self.client.post("/api/settings/encode_mode", json={"mode": "vector"})
        self.assertEqual(r.status_code, 200, r.text)
        self.assertEqual((r.json()["mode"], r.json()["quality"]), ("vector", 80))   # default detail = band V0, not the tile 40
        r = self.client.post("/api/settings/encode_mode", json={"mode": "vector", "quality": 30})
        self.assertEqual(r.status_code, 200, r.text)
        self.assertEqual(r.json()["quality"], 60)          # operator detail is clamped into 60..100
        self.assertEqual((last_published()["mode"], last_published()["quality"]), ("vector", 60))
        # leaving VECTOR sends the tile dial again, never the vector detail
        r = self.client.post("/api/settings/encode_mode", json={"mode": "full"})
        self.assertEqual(r.status_code, 200, r.text)
        self.assertEqual((last_published()["mode"], last_published()["quality"]), ("full", 40))
        self.assertEqual(self.web_ui._vector_detail, 60)
        # both dials survive a restart: the loader repopulates them from the store
        self.web_ui._vector_detail, self.web_ui._tile_quality = 80, None
        self.assertEqual(self.web_ui._load_encode_mode_state(), ("full", 40))
        self.assertEqual((self.web_ui._vector_detail, self.web_ui._tile_quality), (60, 40))

    def test_failed_persist_leaves_both_dials_untouched(self) -> None:
        self.client.post("/api/settings/encode_mode", json={"mode": "full", "quality": 45})
        before = (self.web_ui._vector_detail, self.web_ui._tile_quality)
        with mock.patch.object(self.web_ui, "_persist_encode_mode_override", side_effect=OSError("disk")):
            r = self.client.post("/api/settings/encode_mode", json={"mode": "vector", "quality": 95})
            self.assertEqual(r.status_code, 500, r.text)
            r = self.client.post("/api/settings/encode_mode", json={"mode": "full", "quality": 70})
            self.assertEqual(r.status_code, 500, r.text)
        self.assertEqual((self.web_ui._vector_detail, self.web_ui._tile_quality), before)
        self.assertEqual(self.web_ui._get_runtime_encode_state(), ("full", 45))


if __name__ == "__main__":
    unittest.main()
