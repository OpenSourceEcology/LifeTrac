"""IP-W2-08 unit tests for the offline IPC capture replay tool.

Fabricates a multi-frame capture (keyframe + 2 deltas) using the
production X8 IPC encoder and the base-station TileDeltaFrame encoder,
runs ``replay_ipc_capture.replay`` against it, and asserts:

  * Every IPC frame decodes (no decode errors).
  * ``ReplayStats`` separates keyframes from deltas correctly.
  * One PNG per decoded frame lands in the output dir, named per the
    documented ``frame_NNNN_seqXXX_{key,delta}.png`` template.
  * Each PNG opens at the canvas dimensions implied by the frame
    (grid_w × tile_px, grid_h × tile_px).
  * The ``--no-pngs`` / ``write_pngs=False`` path runs without PIL
    being involved in the canvas pipeline (skipped if PIL absent).

Catches before bench:
  * IPC framer + TileDeltaFrame parser drift between X8 and base.
  * Replay losing the persisted canvas across deltas (would emit a
    nearly-blank PNG for the second/third frame in a sequence).
"""

from __future__ import annotations

import importlib.util
import io
import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path


_BS_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
_REPO_ROOT = os.path.normpath(os.path.join(_BS_DIR, "..", ".."))
_X8_DIR = os.path.normpath(os.path.join(
    _BS_DIR, "..", "firmware", "tractor_x8"))
for d in (_BS_DIR, _X8_DIR, _REPO_ROOT):
    if d not in sys.path:
        sys.path.insert(0, d)


def _have_pil() -> bool:
    try:
        import PIL  # noqa: F401
        return True
    except Exception:
        return False


def _load_replay_module():
    spec = importlib.util.spec_from_file_location(
        "_replay_ipc_capture_under_test",
        os.path.join(_BS_DIR, "replay_ipc_capture.py"))
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _load_x8_ipc_module():
    spec = importlib.util.spec_from_file_location(
        "_x8_ipc_for_replay_test",
        os.path.join(_X8_DIR, "image_pipeline", "ipc_to_h747.py"))
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _load_bs_frame_format():
    spec = importlib.util.spec_from_file_location(
        "_bs_ff_for_replay_test",
        os.path.join(_BS_DIR, "image_pipeline", "frame_format.py"))
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _make_webp_tile(tile_px: int, fill: tuple[int, int, int]) -> bytes:
    from PIL import Image  # type: ignore
    img = Image.new("RGB", (tile_px, tile_px), fill)
    buf = io.BytesIO()
    img.save(buf, format="WEBP", quality=60, method=4)
    return buf.getvalue()


def _build_capture(grid_w: int = 12, grid_h: int = 8, tile_px: int = 32) -> bytes:
    """Synthesize a capture: keyframe (full canvas) + 2 deltas (a few tiles)."""
    ipc_mod = _load_x8_ipc_module()
    bs_ff = _load_bs_frame_format()
    n_tiles = grid_w * grid_h

    # Keyframe: paint every tile a deterministic colour derived from index.
    tiles_kf = []
    for idx in range(n_tiles):
        ty, tx = divmod(idx, grid_w)
        colour = ((tx * 21) & 0xFF, (ty * 31) & 0xFF, ((tx + ty) * 17) & 0xFF)
        tiles_kf.append(bs_ff.TileBlob(
            index=idx, tx=tx, ty=ty,
            blob=_make_webp_tile(tile_px, colour)))
    keyframe = bs_ff.TileDeltaFrame(
        frame_kind=bs_ff.FRAME_KIND_KEY,
        base_seq=0, grid_w=grid_w, grid_h=grid_h, tile_px=tile_px,
        changed_indices=list(range(n_tiles)),
        tiles=tiles_kf)
    wire_kf = bs_ff.encode_tile_delta_frame(keyframe)

    # Delta #1: 4 tiles, all bright red.
    delta_indices_1 = [0, 5, 13, 95]
    tiles_d1 = []
    for idx in delta_indices_1:
        ty, tx = divmod(idx, grid_w)
        tiles_d1.append(bs_ff.TileBlob(
            index=idx, tx=tx, ty=ty,
            blob=_make_webp_tile(tile_px, (240, 16, 16))))
    delta1 = bs_ff.TileDeltaFrame(
        frame_kind=bs_ff.FRAME_KIND_DELTA,
        base_seq=1, grid_w=grid_w, grid_h=grid_h, tile_px=tile_px,
        changed_indices=delta_indices_1, tiles=tiles_d1)
    wire_d1 = bs_ff.encode_tile_delta_frame(delta1)

    # Delta #2: 2 tiles, green.
    delta_indices_2 = [42, 50]
    tiles_d2 = []
    for idx in delta_indices_2:
        ty, tx = divmod(idx, grid_w)
        tiles_d2.append(bs_ff.TileBlob(
            index=idx, tx=tx, ty=ty,
            blob=_make_webp_tile(tile_px, (16, 220, 16))))
    delta2 = bs_ff.TileDeltaFrame(
        frame_kind=bs_ff.FRAME_KIND_DELTA,
        base_seq=2, grid_w=grid_w, grid_h=grid_h, tile_px=tile_px,
        changed_indices=delta_indices_2, tiles=tiles_d2)
    wire_d2 = bs_ff.encode_tile_delta_frame(delta2)

    # Wrap each TileDeltaFrame in an IPC frame; sprinkle a junk byte in
    # between to exercise the framer's resync.
    capture = bytearray()
    capture += ipc_mod.encode_ipc_frame(wire_kf, is_keyframe=True)
    capture += b"\x00"
    capture += ipc_mod.encode_ipc_frame(wire_d1)
    capture += ipc_mod.encode_ipc_frame(wire_d2)
    return bytes(capture)


@unittest.skipUnless(_have_pil(), "PIL not installed")
class ReplayIpcCaptureTests(unittest.TestCase):

    def setUp(self) -> None:
        self.replay_mod = _load_replay_module()
        self.tmpdir = Path(tempfile.mkdtemp(prefix="lifetrac_replay_"))
        self.capture_path = self.tmpdir / "capture.bin"
        self.out_dir = self.tmpdir / "pngs"
        self.capture_path.write_bytes(_build_capture())

    def tearDown(self) -> None:
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def test_decodes_all_three_frames(self) -> None:
        stats = self.replay_mod.replay(
            self.capture_path, self.out_dir, write_pngs=True)
        self.assertEqual(stats.ipc_frames, 3,
                         f"expected 3 IPC frames, got {stats.ipc_frames}")
        self.assertEqual(stats.decoded_frames, 3)
        self.assertEqual(stats.keyframes, 1)
        self.assertEqual(stats.deltas, 2)
        self.assertEqual(stats.decode_errors, 0)
        self.assertEqual(stats.pngs_written, 3)

    def test_pngs_use_documented_naming(self) -> None:
        self.replay_mod.replay(
            self.capture_path, self.out_dir, write_pngs=True)
        pngs = sorted(p.name for p in self.out_dir.glob("*.png"))
        self.assertEqual(pngs, [
            "frame_0001_seq000_key.png",
            "frame_0002_seq001_delta.png",
            "frame_0003_seq002_delta.png",
        ])

    def test_pngs_have_expected_canvas_size(self) -> None:
        from PIL import Image  # type: ignore
        self.replay_mod.replay(
            self.capture_path, self.out_dir, write_pngs=True)
        for png in self.out_dir.glob("*.png"):
            with Image.open(png) as img:
                self.assertEqual(img.size, (12 * 32, 8 * 32),
                                 f"{png.name} has wrong canvas size {img.size}")

    def test_limit_caps_decoded_count(self) -> None:
        stats = self.replay_mod.replay(
            self.capture_path, self.out_dir, limit=2, write_pngs=True)
        self.assertEqual(stats.decoded_frames, 2)
        self.assertEqual(stats.pngs_written, 2)

    def test_no_pngs_path_skips_writes(self) -> None:
        stats = self.replay_mod.replay(
            self.capture_path, self.out_dir, write_pngs=False)
        self.assertEqual(stats.decoded_frames, 3)
        self.assertEqual(stats.pngs_written, 0)
        self.assertFalse(self.out_dir.exists(),
                         "out_dir should not be created in --no-pngs mode")

    def test_corrupt_payload_records_decode_error(self) -> None:
        # Take the keyframe IPC frame, but truncate its payload mid-tile so
        # parse_tile_delta_frame raises FrameDecodeError. The IPC framer
        # itself still validates CRC, so corrupt at the TileDeltaFrame layer:
        ipc_mod = _load_x8_ipc_module()
        bs_ff = _load_bs_frame_format()
        broken_payload = bs_ff.encode_tile_delta_frame(bs_ff.TileDeltaFrame(
            frame_kind=bs_ff.FRAME_KIND_KEY, base_seq=7,
            grid_w=12, grid_h=8, tile_px=32,
            changed_indices=[0],
            tiles=[bs_ff.TileBlob(0, 0, 0,
                                  _make_webp_tile(32, (10, 10, 10)))]))
        # Drop the last 2 bytes of the WebP body inside the IPC payload.
        ipc_frame = ipc_mod.encode_ipc_frame(broken_payload[:-2])
        bad_capture = self.tmpdir / "bad.bin"
        bad_capture.write_bytes(ipc_frame)
        stats = self.replay_mod.replay(
            bad_capture, self.out_dir, write_pngs=False)
        self.assertEqual(stats.ipc_frames, 1)
        self.assertEqual(stats.decoded_frames, 0)
        self.assertEqual(stats.decode_errors, 1)


@unittest.skipUnless(_have_pil(), "PIL not installed")
class ReplayCliTests(unittest.TestCase):
    """Smoke-test the argparse/CLI path."""

    def setUp(self) -> None:
        self.replay_mod = _load_replay_module()
        self.tmpdir = Path(tempfile.mkdtemp(prefix="lifetrac_replay_cli_"))
        self.capture_path = self.tmpdir / "capture.bin"
        self.out_dir = self.tmpdir / "out"
        self.capture_path.write_bytes(_build_capture())

    def tearDown(self) -> None:
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def test_main_returns_zero_on_success(self) -> None:
        rc = self.replay_mod.main([
            str(self.capture_path),
            "--out-dir", str(self.out_dir),
        ])
        self.assertEqual(rc, 0)
        self.assertTrue(any(self.out_dir.glob("*.png")))

    def test_missing_capture_returns_two(self) -> None:
        rc = self.replay_mod.main([
            str(self.tmpdir / "does_not_exist.bin"),
        ])
        self.assertEqual(rc, 2)


if __name__ == "__main__":
    unittest.main()
