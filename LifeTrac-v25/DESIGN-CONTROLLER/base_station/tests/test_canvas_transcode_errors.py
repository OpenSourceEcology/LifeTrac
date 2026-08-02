"""Canvas must survive a transcoder failing with a non-codec exception.

Reproduces the 2026-08-01 on-air incident (F10 acceptance RESULTS §6): the
stream switched to mono_g4 on a base station without Pillow, the lazy
``from PIL import Image`` raised ModuleNotFoundError, and the exception
unwound out of Canvas.apply AFTER ``_last_base_seq`` was adopted but BEFORE
any tile applied — the canvas silently froze while appearing to track the
stream, and the request_keyframe publish path never ran.

Two layers now guarantee the invariant "one bad tile never kills the frame":
transcode_to_webp folds any transcoder exception into CodecDecodeError, and
Canvas.apply's per-tile catch is a broad backstop regardless.
"""

import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from image_pipeline import codec_decode                      # noqa: E402
from image_pipeline.canvas import Canvas                     # noqa: E402
from image_pipeline.codec_decode import (                    # noqa: E402
    CodecDecodeError,
    transcode_to_webp,
)
from image_pipeline.frame_format import (                    # noqa: E402
    CODEC_MONO_G4,
    CODEC_WEBP,
    TileBlob,
    TileDeltaFrame,
)


def _keyframe_webp(indices):
    return TileDeltaFrame(
        frame_kind=1, base_seq=0, grid_w=12, grid_h=8, tile_px=32,
        changed_indices=list(indices),
        tiles=[TileBlob(index=i, tx=i % 12, ty=i // 12, blob=b"RIFFwebp")
               for i in indices],
        codec=CODEC_WEBP,
    )


def _delta_mono(seq, indices):
    return TileDeltaFrame(
        frame_kind=0, base_seq=seq, grid_w=12, grid_h=8, tile_px=32,
        changed_indices=list(indices),
        tiles=[TileBlob(index=i, tx=i % 12, ty=i // 12, blob=b"\x00\xaa\xbb")
               for i in indices],
        codec=CODEC_MONO_G4,
    )


class TranscodeErrorTaxonomyTests(unittest.TestCase):
    """transcode_to_webp folds foreign exceptions into CodecDecodeError."""

    def test_import_error_becomes_codec_decode_error(self) -> None:
        boom = mock.Mock(
            side_effect=ModuleNotFoundError("No module named 'PIL'"))
        with mock.patch.dict(codec_decode._TRANSCODERS,
                             {CODEC_MONO_G4: boom}):
            with self.assertRaises(CodecDecodeError) as ctx:
                transcode_to_webp(CODEC_MONO_G4, b"\x00\xaa", 32)
        self.assertIn("PIL", str(ctx.exception))
        self.assertIsInstance(ctx.exception.__cause__, ModuleNotFoundError)

    def test_codec_decode_error_passes_through_unwrapped(self) -> None:
        boom = mock.Mock(side_effect=CodecDecodeError("mono_g4 underrun"))
        with mock.patch.dict(codec_decode._TRANSCODERS,
                             {CODEC_MONO_G4: boom}):
            with self.assertRaises(CodecDecodeError) as ctx:
                transcode_to_webp(CODEC_MONO_G4, b"\x00", 32)
        self.assertEqual(str(ctx.exception), "mono_g4 underrun")


class CanvasSurvivesTranscoderDeathTests(unittest.TestCase):
    def setUp(self) -> None:
        self.now = [50]
        self.canvas = Canvas(clock_ms=lambda: self.now[0])
        self.canvas.apply(_keyframe_webp(range(96)))   # all tiles at t=50
        self.now[0] = 1000

    def test_incident_repro_frame_survives_missing_pil(self) -> None:
        """The exact on-air failure: mono_g4 delta, PIL missing."""
        boom = mock.Mock(
            side_effect=ModuleNotFoundError("No module named 'PIL'"))
        with mock.patch.dict(codec_decode._TRANSCODERS,
                             {CODEC_MONO_G4: boom}):
            upd = self.canvas.apply(_delta_mono(1, [5, 6]))
        # Frame applied: seq adopted AND the failure surfaced as a
        # keyframe request instead of an unwinding exception.
        self.assertEqual(self.canvas._last_base_seq, 1)
        self.assertTrue(upd.request_keyframe)
        self.assertIn("PIL", upd.reason)
        self.assertEqual(upd.updated_indices, [])
        # The keyframe-era tiles are untouched, not refreshed and not zeroed.
        self.assertEqual(self.canvas._tiles[5].arrived_ms, 50)
        self.assertEqual(self.canvas._tiles[7].arrived_ms, 50)

    def test_backstop_survives_arbitrary_exception_mid_frame(self) -> None:
        """Canvas's own catch holds even for a non-CodecDecodeError."""
        calls = {"n": 0}

        def flaky(codec, blob):
            calls["n"] += 1
            if calls["n"] == 2:
                raise RuntimeError("cache exploded")
            return b"RIFFwebp"

        with mock.patch.object(self.canvas, "_transcoded", side_effect=flaky):
            upd = self.canvas.apply(_delta_mono(1, [10, 11, 12]))
        # Tile 11 (2nd) died; 10 and 12 still applied with fresh arrived_ms.
        self.assertEqual(upd.updated_indices, [10, 12])
        self.assertEqual(self.canvas._tiles[10].arrived_ms, 1000)
        self.assertEqual(self.canvas._tiles[11].arrived_ms, 50)
        self.assertEqual(self.canvas._tiles[12].arrived_ms, 1000)
        self.assertTrue(upd.request_keyframe)
        self.assertIn("cache exploded", upd.reason)


if __name__ == "__main__":
    unittest.main()
