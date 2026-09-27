"""Encoder ↔ store interoperability: the tractor's VectorEncoder feeds the
base's VectorSceneStore through the real TileDeltaFrame container.

This is the test that says the two ends agree on the wire format *and* on
the conventions behind DIGEST and CONFIRM (state-hash serialisation, group
membership, render state): a static second frame must leave the store with
``digest_ok`` True and no orphans, or the two implementations have drifted.
Needs numpy and cv2 (the encoder's extraction runs on OpenCV), so it skips
on a machine without them; CI installs both from Phase 1 on.
"""
from __future__ import annotations

import os
import sys
import unittest

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_BS_DIR = os.path.dirname(_THIS_DIR)
_X8_DIR = os.path.normpath(os.path.join(_THIS_DIR, "..", "..", "firmware", "tractor_x8"))
for _p in (_BS_DIR, _X8_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    import numpy as np
    import cv2  # noqa: F401
    _HAVE_CV = True
except ImportError:  # pragma: no cover
    _HAVE_CV = False

from image_pipeline.frame_format import CODEC_VECTOR, HEADER_FIXED_LEN, parse_tile_delta_frame  # noqa: E402
from image_pipeline.vector_scene import codec as vs  # noqa: E402
from image_pipeline.vector_scene_store import VectorSceneStore  # noqa: E402

if _HAVE_CV:
    from x8_image_pipeline.encode_vector import VectorEncoder  # noqa: E402

W, H = 384, 256
HORIZON_Y = 100.0          # true horizon row at x = 192


def synthetic_scene(shift_x: int = 0) -> "np.ndarray":
    """Sky gradient over a tilted horizon, green ground, a soil patch and a
    dark tree; deterministic, no files."""
    img = np.zeros((H, W, 3), np.uint8)
    xs = np.arange(W)
    horizon = HORIZON_Y + (xs - 192) * 0.03          # about 1.7° of roll
    for y in range(H):
        sky = y < horizon
        t = y / H
        img[y, sky] = (int(140 - 60 * t), int(180 - 40 * t), int(235 - 20 * t))
        img[y, ~sky] = (int(90 + 30 * t), int(140 - 40 * t), int(50 + 10 * t))
    # soil patch (a mass) and a tree (dark green blob) on the ground
    img[150:210, 60:180] = (120, 85, 50)
    cy, cx = 130, 290
    yy, xx = np.ogrid[:H, :W]
    tree = ((yy - cy) / 22.0) ** 2 + ((xx - cx) / 14.0) ** 2 <= 1.0
    img[tree] = (30, 70, 30)
    if shift_x:
        img = np.roll(img, shift_x, axis=1)
    return img


@unittest.skipUnless(_HAVE_CV, "numpy + cv2 required for the encoder")
class EncoderStoreInteropTests(unittest.TestCase):
    def setUp(self) -> None:
        self.enc = VectorEncoder(canvas=(W, H))
        self.store = VectorSceneStore(W, H)

    def _send(self, payload: bytes, rx_ms: int):
        frame = parse_tile_delta_frame(payload)
        self.assertEqual(frame.codec, CODEC_VECTOR)
        self.assertEqual((frame.grid_w, frame.grid_h, frame.tile_px), (12, 8, 32))
        res = self.store.ingest(frame.vector_body, frame.frame_kind, rx_ms, 0.0)
        self.assertTrue(res.applied, res)
        return frame

    def test_epoch_start_fits_the_budget_and_anchors_the_store(self) -> None:
        payload = self.enc.frame(synthetic_scene(), 203, epoch_start=True, quality=80, seq=1)
        self.assertEqual(payload[0], 1)                                   # frame_kind = key
        self.assertLessEqual(len(payload) - HEADER_FIXED_LEN, 203 - HEADER_FIXED_LEN - 1)  # F − 1 on an epoch start
        decoded = vs.decode_frame(payload[HEADER_FIXED_LEN:], payload[0])
        self.assertTrue(decoded.header.key)
        kinds = [type(r) for r in decoded.records]
        self.assertTrue(any(k in (vs.HznAbs, vs.HznNoHorizon) for k in kinds), kinds)
        self.assertIn(vs.LayerClear, kinds)
        self.assertIn(vs.Digest, kinds)
        self._send(payload, rx_ms=1000)
        snap = self.store.snapshot(1200)
        self.assertIsNotNone(snap)
        self.assertEqual(snap["horizon"]["mode"], "abs")
        centre_y = snap["horizon"]["pts"][1][1]
        self.assertAlmostEqual(centre_y, HORIZON_Y, delta=8.0)             # within one 8 px cell
        shapes = [s for layer in snap["layers"] for s in layer["shapes"]]
        self.assertGreaterEqual(len(shapes), 1)
        self.assertEqual(self.store.stats["orphans"], 0)

    def test_static_second_frame_agrees_on_digest_and_confirm(self) -> None:
        img = synthetic_scene()
        f1 = self.enc.frame(img, 203, epoch_start=True, quality=80, seq=1)
        self._send(f1, rx_ms=1000)
        f2 = self.enc.frame(img, 203, quality=80, seq=2)
        self.assertEqual(f2[0], 0)                                        # an update, not a key
        self.assertLessEqual(len(f2), 203)
        self._send(f2, rx_ms=1500)
        st = self.store.stats
        self.assertEqual((st["frames_applied"], st["frames_bad"], st["orphans"]), (2, 0, 0))
        snap = self.store.snapshot(1600)
        self.assertTrue(snap["digest_ok"], "encoder DIGEST does not match the store's mirror")
        self.assertFalse(snap["resync"])
        # CONFIRMed shapes were verified at the second capture: ages measure from it
        ages = [s["age_ms"] for layer in snap["layers"] for s in layer["shapes"]]
        self.assertTrue(ages and max(ages) <= 600, ages)

    def test_pan_keeps_the_two_ends_in_sync(self) -> None:
        f1 = self.enc.frame(synthetic_scene(), 203, epoch_start=True, quality=80, seq=1)
        self._send(f1, rx_ms=1000)
        for i, dx in enumerate((6, 12, 18), start=2):
            f = self.enc.frame(synthetic_scene(shift_x=dx), 203, quality=80, moving=True, seq=i)
            self.assertLessEqual(len(f), 203)
            self._send(f, rx_ms=1000 + 500 * (i - 1))
        st = self.store.stats
        self.assertEqual((st["frames_bad"], st["orphans"]), (0, 0))
        self.assertTrue(self.store.snapshot(3000)["digest_ok"])

    def test_quality_band_sets_the_level_bit(self) -> None:
        img = synthetic_scene()
        for quality, level in ((80, 0), (50, 1), (30, 2), (10, 3)):
            enc = VectorEncoder(canvas=(W, H))
            payload = enc.frame(img, 203, epoch_start=True, quality=quality, seq=1)
            self.assertEqual(vs.decode_frame(payload[HEADER_FIXED_LEN:], payload[0]).header.level, level, quality)


if __name__ == "__main__":
    unittest.main()
