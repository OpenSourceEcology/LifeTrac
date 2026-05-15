"""IP-W2-08 offline bench: decode a captured IPC byte stream into PNGs.

The X8 publishes one IPC frame per encoded TileDeltaFrame onto
``/dev/ttymxc1`` toward the H747 M7 (see
``firmware/tractor_x8/image_pipeline/ipc_to_h747.py``). When debugging
adaptive-budget / encode-mode / ROI behaviour against a real run it's
much easier to capture that stream once::

    # On the X8 (or via adb shell)
    cat /dev/ttymxc1 > /tmp/ipc_capture.bin

…and then replay it offline against a known parser, rather than try to
catch a transient in-the-loop.

Usage::

    py -3 -m base_station.replay_ipc_capture \
        capture.bin --out-dir replay_out

For each IPC frame in the capture this tool:

  1. Uses :func:`iter_ipc_frames` to extract framed payloads (sync byte
     ``0xA5``, flags, length, CRC-8/SMBus).
  2. Hands each payload to :func:`parse_tile_delta_frame` to lift out
     the per-tile WebP blobs.
  3. Maintains a running 384×256 RGB canvas (`Canvas` semantics — only
     dirty tiles are repainted; missing tiles stay at their last value).
  4. Writes ``frame_NNNN_seq{base_seq}_{kind}.png`` for every frame.

Exit code 0 if at least one frame decoded; non-zero otherwise. Designed
for the host (PIL needed for PNG output); the device need not have PIL.
"""
from __future__ import annotations

import argparse
import importlib.util
import os
import sys
from dataclasses import dataclass
from pathlib import Path


_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.normpath(os.path.join(_THIS_DIR, "..", ".."))
_X8_DIR = os.path.normpath(os.path.join(
    _THIS_DIR, "..", "firmware", "tractor_x8"))


def _load_x8_ipc_module():
    """Load the X8-side ``ipc_to_h747`` module without polluting
    ``sys.modules['image_pipeline']`` (the base-station has its own
    same-named package)."""
    if _X8_DIR not in sys.path:
        sys.path.insert(0, _X8_DIR)
    spec = importlib.util.spec_from_file_location(
        "_x8_ipc_to_h747",
        os.path.join(_X8_DIR, "image_pipeline", "ipc_to_h747.py"))
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _load_bs_frame_format():
    """Load the base-station ``frame_format`` directly (avoids the
    ``image_pipeline`` package shadowing the X8 one when both are on
    sys.path)."""
    spec = importlib.util.spec_from_file_location(
        "_bs_frame_format_replay",
        os.path.join(_THIS_DIR, "image_pipeline", "frame_format.py"))
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


@dataclass
class ReplayStats:
    ipc_frames: int = 0
    decoded_frames: int = 0
    decode_errors: int = 0
    keyframes: int = 0
    deltas: int = 0
    pngs_written: int = 0


def _blit_tile(canvas, tile_img, tx: int, ty: int, tile_px: int) -> None:
    canvas.paste(tile_img, (tx * tile_px, ty * tile_px))


def replay(capture_path: Path, out_dir: Path,
           limit: int | None = None,
           write_pngs: bool = True) -> ReplayStats:
    """Decode ``capture_path`` and emit one PNG per TileDeltaFrame.

    Returns a :class:`ReplayStats` summary. Pure function aside from the
    PNG writes — useful for the unit test which calls with
    ``write_pngs=False``.
    """
    ipc_mod = _load_x8_ipc_module()
    bs_ff = _load_bs_frame_format()

    raw = capture_path.read_bytes()
    stats = ReplayStats()

    if write_pngs:
        out_dir.mkdir(parents=True, exist_ok=True)
        from PIL import Image  # type: ignore
    else:
        Image = None  # type: ignore[assignment]

    canvas = None  # type: ignore[assignment]
    tile_px = 0

    for frame in ipc_mod.iter_ipc_frames(raw):
        stats.ipc_frames += 1
        try:
            tdf = bs_ff.parse_tile_delta_frame(frame.payload)
        except bs_ff.FrameDecodeError as exc:
            stats.decode_errors += 1
            print(f"  ipc#{stats.ipc_frames}: TileDeltaFrame decode error: {exc}",
                  file=sys.stderr)
            continue
        stats.decoded_frames += 1
        if tdf.is_keyframe:
            stats.keyframes += 1
        else:
            stats.deltas += 1

        if write_pngs:
            from PIL import Image  # type: ignore
            if canvas is None or tile_px != tdf.tile_px:
                canvas = Image.new("RGB",
                                   (tdf.canvas_w, tdf.canvas_h),
                                   (16, 16, 16))
                tile_px = tdf.tile_px
            for tile in tdf.tiles:
                try:
                    tile_img = Image.open(__import__("io").BytesIO(tile.blob))
                    tile_img.load()
                    if tile_img.size != (tile_px, tile_px):
                        tile_img = tile_img.resize((tile_px, tile_px))
                    if tile_img.mode != "RGB":
                        tile_img = tile_img.convert("RGB")
                    _blit_tile(canvas, tile_img, tile.tx, tile.ty, tile_px)
                except Exception as exc:                      # pragma: no cover
                    print(f"  ipc#{stats.ipc_frames} tile {tile.index}: "
                          f"WebP decode failed: {exc}", file=sys.stderr)
            kind = "key" if tdf.is_keyframe else "delta"
            png_path = out_dir / (
                f"frame_{stats.decoded_frames:04d}_seq{tdf.base_seq:03d}_"
                f"{kind}.png")
            canvas.save(png_path, format="PNG")
            stats.pngs_written += 1

        if limit is not None and stats.decoded_frames >= limit:
            break

    return stats


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        description="Decode a captured /dev/ttymxc1 IPC byte log into PNGs.")
    p.add_argument("capture", type=Path,
                   help="Path to captured IPC byte stream.")
    p.add_argument("--out-dir", type=Path, default=Path("replay_out"),
                   help="Output directory for PNGs (default: replay_out/).")
    p.add_argument("--limit", type=int, default=None,
                   help="Stop after N decoded frames (default: all).")
    p.add_argument("--no-pngs", action="store_true",
                   help="Skip PNG writes; only print decode statistics.")
    args = p.parse_args(argv)

    if not args.capture.exists():
        print(f"capture not found: {args.capture}", file=sys.stderr)
        return 2

    print(f"Replaying {args.capture} ({args.capture.stat().st_size} B) -> "
          f"{args.out_dir if not args.no_pngs else '<no PNGs>'}")
    stats = replay(args.capture, args.out_dir,
                   limit=args.limit, write_pngs=not args.no_pngs)
    print(f"  ipc_frames     = {stats.ipc_frames}")
    print(f"  decoded_frames = {stats.decoded_frames}")
    print(f"  keyframes      = {stats.keyframes}")
    print(f"  deltas         = {stats.deltas}")
    print(f"  decode_errors  = {stats.decode_errors}")
    print(f"  pngs_written   = {stats.pngs_written}")
    return 0 if stats.decoded_frames > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
