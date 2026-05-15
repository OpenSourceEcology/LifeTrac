#!/usr/bin/env python3
"""
LifeTrac v25 — tractor X8 camera service.

Captures from the active camera, tiles into a 12×8 grid of 32×32 WebP blobs,
diffs against the last sent canvas, and publishes either an I-frame (every
tile) or a P-frame (changed tiles only) as a single binary MQTT payload on
``lifetrac/v25/cmd/image_frame``.

The M7 (`tractor_m7.ino`) consumes that topic over the X8↔H747 UART, wraps
each chunk in TelemetryFrame topic ``0x25`` (TileDeltaFrame) and fragments
it across the air per LORA_PROTOCOL.md § TileDeltaFrame.

This file is the *encoder side* only. Reassembly + canvas live in
``base_station/image_pipeline/`` and are out of scope here.

Per MASTER_PLAN.md §8.19 / `../AI NOTES/2026-04-27_Image_Transmission_*.md`.

Capture backend:
  * Default: libcamera via the ``picamera2`` Python binding (the supported
    path on Portenta X8's MIPI-CSI sensor).
  * Fallback: a synthetic frame source for bench testing without a sensor
    (set ``LIFETRAC_CAMERA_SOURCE=synthetic``).

Sizing constraints (from LORA_PROTOCOL.md § TileDeltaFrame):
  * grid_w=12, grid_h=8, tile_px=32 → 384×256 logical canvas.
  * Per-tile WebP blob ≤ 256 B (``tile_size_minus1`` is one byte).
  * I-frame cadence: every ``KEYFRAME_PERIOD_S`` seconds OR on
    ``CMD_REQ_KEYFRAME`` (forwarded from the M7 on topic
    ``lifetrac/v25/cmd/req_keyframe``).
"""

from __future__ import annotations

import io
import logging
import os
import struct
import time
from dataclasses import dataclass

try:                                  # IP-309: optional numpy fast path
    import numpy as _np               # noqa: F401  (presence check)
    _HAS_NUMPY = True
except ImportError:                   # X8 image without numpy still works
    _np = None                        # type: ignore[assignment]
    _HAS_NUMPY = False

LOG = logging.getLogger("camera_service")

GRID_W = 12
GRID_H = 8
TILE_PX = 32
CANVAS_W = GRID_W * TILE_PX        # 384
CANVAS_H = GRID_H * TILE_PX        # 256
TILE_BYTES_MAX = 256               # tile_size_minus1 is u8, so ≤256 B

KEYFRAME_PERIOD_S = float(os.environ.get("LIFETRAC_KEYFRAME_PERIOD_S", "10"))
TARGET_FPS        = float(os.environ.get("LIFETRAC_CAMERA_FPS", "2"))
# IP-208: clamp WEBP quality to a sensible range so a typo can't disable
# the encoder entirely (1 would skip the in-loop guard) or push past lossless.
_RAW_WEBP_Q = int(os.environ.get("LIFETRAC_WEBP_QUALITY", "55"))
if _RAW_WEBP_Q < 20 or _RAW_WEBP_Q > 100:
    LOG.warning("WEBP_QUALITY=%d out of range; clamping to [20, 100]", _RAW_WEBP_Q)
WEBP_QUALITY      = max(20, min(100, _RAW_WEBP_Q))
SOURCE            = os.environ.get("LIFETRAC_CAMERA_SOURCE", "libcamera")
MQTT_HOST         = os.environ.get("LIFETRAC_MQTT_HOST", "localhost")

# IP-W2-02: USB camera path validated 2026-05-15 on the bench (Kurokesu C2
# on `/dev/video1`, MJPEG 1920x1080@30fps, captured via a static aarch64
# `ffmpeg` binary at `/tmp/ffmpeg`). The systemd unit must put the service
# user in the `video` group; sudo is *not* baked in here.
V4L2_DEVICE       = os.environ.get("LIFETRAC_CAMERA_DEVICE", "/dev/video1")
V4L2_INPUT_FORMAT = os.environ.get("LIFETRAC_V4L2_INPUT_FORMAT", "mjpeg")
V4L2_INPUT_SIZE   = os.environ.get("LIFETRAC_V4L2_INPUT_SIZE", "1920x1080")
V4L2_INPUT_FPS    = int(os.environ.get("LIFETRAC_V4L2_INPUT_FPS", "30"))
FFMPEG_PATH       = os.environ.get("LIFETRAC_FFMPEG_PATH", "ffmpeg")

# IP-104: primary path for encoded image fragments is the X8 → H747 UART
# (length-framed via image_pipeline/ipc_to_h747.py). The MQTT publish is
# kept only when ``LIFETRAC_CAMERA_DEBUG_MQTT=1`` so a forensic listener
# can subscribe without the M7 having to bridge it.
M7_UART_DEVICE    = os.environ.get("LIFETRAC_M7_UART", "/dev/ttymxc1")
DEBUG_MQTT        = os.environ.get("LIFETRAC_CAMERA_DEBUG_MQTT", "").strip() == "1"

PUBLISH_TOPIC     = "lifetrac/v25/cmd/image_frame"
KEYFRAME_REQ_TOPIC = "lifetrac/v25/cmd/req_keyframe"


# ---- capture backends -------------------------------------------------

class SyntheticCamera:
    """Test-only source: emits a slowly-shifting gradient. Useful when the
    X8 has no MIPI sensor wired up yet — exercises the diff/encode path
    without needing libcamera."""

    def __init__(self) -> None:
        self._t = 0

    def grab_rgb(self) -> bytes:
        # 3 bytes/pixel, scrolling diagonal stripes so P-frames have
        # something to encode.
        self._t = (self._t + 1) & 0xFF
        out = bytearray(CANVAS_W * CANVAS_H * 3)
        t = self._t
        for y in range(CANVAS_H):
            for x in range(CANVAS_W):
                v = (x + y + t) & 0xFF
                i = (y * CANVAS_W + x) * 3
                out[i]     = v
                out[i + 1] = (v << 1) & 0xFF
                out[i + 2] = (255 - v) & 0xFF
        return bytes(out)


class LibcameraCamera:
    """Real backend via picamera2. Imported lazily so the synthetic mode
    works on a bench without libcamera installed."""

    def __init__(self) -> None:
        from picamera2 import Picamera2  # type: ignore
        self._cam = Picamera2()
        cfg = self._cam.create_video_configuration(
            main={"size": (CANVAS_W, CANVAS_H), "format": "RGB888"})
        self._cam.configure(cfg)
        self._cam.start()

    def grab_rgb(self) -> bytes:
        arr = self._cam.capture_array("main")  # numpy HxWx3 uint8
        return arr.tobytes()


def build_ffmpeg_argv(device: str = V4L2_DEVICE,
                      input_format: str = V4L2_INPUT_FORMAT,
                      input_size: str = V4L2_INPUT_SIZE,
                      input_fps: int = V4L2_INPUT_FPS,
                      ffmpeg_path: str = FFMPEG_PATH,
                      canvas_w: int = CANVAS_W,
                      canvas_h: int = CANVAS_H) -> list[str]:
    """Argv for the bench-validated USB-camera capture pipeline.

    Mirrors the 2026-05-15 invocation that produced the first real frame
    off the Kurokesu C2:

        ffmpeg -hide_banner -loglevel error \\
               -f v4l2 -input_format mjpeg \\
               -video_size 1920x1080 -framerate 30 \\
               -i /dev/video1 \\
               -vf scale=384:256 -pix_fmt rgb24 \\
               -f rawvideo -

    Pure helper so the unit tests can pin the argv shape without spawning
    a subprocess.
    """
    return [
        ffmpeg_path,
        "-hide_banner", "-loglevel", "error",
        "-f", "v4l2",
        "-input_format", input_format,
        "-video_size", input_size,
        "-framerate", str(int(input_fps)),
        "-i", device,
        "-vf", f"scale={canvas_w}:{canvas_h}",
        "-pix_fmt", "rgb24",
        "-f", "rawvideo",
        "-",
    ]


class V4l2FfmpegCamera:
    """USB-camera backend: spawn `ffmpeg` and read raw RGB frames from stdout.

    Matches the bench-validated W2-02 path. The subprocess is restarted on
    EOF / read error so a transient USB hiccup (handled by the W2-01 host-
    controller mitigations) doesn't kill the encoder loop. Each `grab_rgb()`
    call returns exactly ``CANVAS_W * CANVAS_H * 3`` bytes; on a permanent
    failure to start `ffmpeg` it raises ``RuntimeError`` so `_make_camera()`
    can fall through to the next backend.
    """

    _FRAME_BYTES = CANVAS_W * CANVAS_H * 3

    def __init__(self,
                 device: str = V4L2_DEVICE,
                 input_format: str = V4L2_INPUT_FORMAT,
                 input_size: str = V4L2_INPUT_SIZE,
                 input_fps: int = V4L2_INPUT_FPS,
                 ffmpeg_path: str = FFMPEG_PATH) -> None:
        self.device = device
        self.input_format = input_format
        self.input_size = input_size
        self.input_fps = input_fps
        self.ffmpeg_path = ffmpeg_path
        self._proc = None  # type: ignore[assignment]
        self._spawn()

    def _spawn(self) -> None:
        import subprocess
        argv = build_ffmpeg_argv(
            device=self.device,
            input_format=self.input_format,
            input_size=self.input_size,
            input_fps=self.input_fps,
            ffmpeg_path=self.ffmpeg_path,
        )
        try:
            self._proc = subprocess.Popen(
                argv,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
        except (OSError, FileNotFoundError) as exc:
            raise RuntimeError(
                f"V4l2FfmpegCamera: failed to spawn {self.ffmpeg_path!r}: {exc}"
            ) from exc
        LOG.info("camera_service: v4l2/ffmpeg pid=%s device=%s %s@%dfps -> %dx%d rgb24",
                 self._proc.pid, self.device, self.input_size, self.input_fps,
                 CANVAS_W, CANVAS_H)

    def _read_exact(self, n: int) -> bytes | None:
        assert self._proc is not None and self._proc.stdout is not None
        out = bytearray()
        while len(out) < n:
            chunk = self._proc.stdout.read(n - len(out))
            if not chunk:
                return None
            out.extend(chunk)
        return bytes(out)

    def grab_rgb(self) -> bytes:
        for _attempt in range(2):
            if self._proc is None or self._proc.poll() is not None:
                self._cleanup()
                self._spawn()
            buf = self._read_exact(self._FRAME_BYTES)
            if buf is not None and len(buf) == self._FRAME_BYTES:
                return buf
            stderr_tail = b""
            if self._proc is not None and self._proc.stderr is not None:
                try:
                    stderr_tail = self._proc.stderr.read() or b""
                except Exception:
                    pass
            LOG.warning("camera_service: v4l2/ffmpeg short read (%s); restarting. stderr=%r",
                        len(buf) if buf is not None else None,
                        stderr_tail[-200:].decode("utf-8", errors="replace"))
            self._cleanup()
        raise RuntimeError("V4l2FfmpegCamera: ffmpeg refused to produce a frame after restart")

    def _cleanup(self) -> None:
        proc = self._proc
        self._proc = None
        if proc is None:
            return
        try:
            proc.terminate()
        except Exception:
            pass
        try:
            proc.wait(timeout=1.0)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass

    def close(self) -> None:
        self._cleanup()

    def __del__(self) -> None:
        try:
            self._cleanup()
        except Exception:
            pass


def _make_camera():
    """Pick the active capture backend.

    SOURCE selector (env ``LIFETRAC_CAMERA_SOURCE``):
      * ``synthetic``          — :class:`SyntheticCamera` (CI / no hardware)
      * ``v4l2`` / ``ffmpeg``  — :class:`V4l2FfmpegCamera` (USB cameras
                                 e.g. Kurokesu C2; bench-validated 2026-05-15)
      * ``libcamera`` (default) — :class:`LibcameraCamera` (MIPI-CSI sensor),
                                  with a v4l2/ffmpeg fallback when the
                                  libcamera stack is unavailable.
    """
    src = SOURCE.lower()
    if src == "synthetic":
        return SyntheticCamera()
    if src in ("v4l2", "ffmpeg", "usb"):
        try:
            return V4l2FfmpegCamera()
        except RuntimeError as exc:                           # pragma: no cover
            LOG.warning("camera_service: v4l2/ffmpeg unavailable (%s) — "
                        "falling back to synthetic", exc)
            return SyntheticCamera()
    try:
        return LibcameraCamera()
    except Exception as exc:                                  # pragma: no cover
        LOG.warning("camera_service: libcamera unavailable (%s) — "
                    "trying v4l2/ffmpeg", exc)
    try:
        return V4l2FfmpegCamera()
    except RuntimeError as exc:                               # pragma: no cover
        LOG.warning("camera_service: v4l2/ffmpeg unavailable (%s) — "
                    "falling back to synthetic", exc)
        return SyntheticCamera()


# ---- tile encode ------------------------------------------------------

def _slice_tile_rgb(rgb_canvas: bytes, tx: int, ty: int) -> bytes:
    """Return the raw 32×32 RGB bytes for tile (tx, ty) from the canvas.

    Shared between :func:`_encode_tile` and the optional W2-05 encode
    cache so the cache hash and the WebP encoder operate on byte-identical
    inputs.
    """
    row_stride = CANVAS_W * 3
    out = bytearray(TILE_PX * TILE_PX * 3)
    for ry in range(TILE_PX):
        src_off = ((ty * TILE_PX) + ry) * row_stride + (tx * TILE_PX) * 3
        dst_off = ry * TILE_PX * 3
        out[dst_off:dst_off + TILE_PX * 3] = \
            rgb_canvas[src_off:src_off + TILE_PX * 3]
    return bytes(out)


# IP-W2-07: encode-mode auto-fallback ladder (per IMAGE_PIPELINE.md §3.4 +
# LORA_PROTOCOL.md opcode 0x63). The base-station's ``EncodeModeController``
# emits ``CMD_ENCODE_MODE`` in response to airtime pressure; the X8 honours
# it by switching the per-tile encode strategy in :func:`_encode_tile`.
#
#   0  full          colour at WEBP_QUALITY (legacy default)
#   1  y_only        luma-only (greyscale) at WEBP_QUALITY
#   2  motion_only   luma-only, hard quality cap of MOTION_ONLY_QUALITY
#   3  wireframe     luma-only, hard quality cap of WIREFRAME_QUALITY
#
# A true wireframe (Sobel-edge) would need numpy/scipy; we approximate by
# leaning on WebP at very low quality, which already strips fine detail to
# blocky luma — the operator-visible "is anything moving?" signal survives
# while bandwidth collapses by ~5×.
ENCODE_MODE_FULL        = 0
ENCODE_MODE_Y_ONLY      = 1
ENCODE_MODE_MOTION_ONLY = 2
ENCODE_MODE_WIREFRAME   = 3
ENCODE_MODE_NAMES = ("full", "y_only", "motion_only", "wireframe")
# Quality ceilings applied as ``min(requested_quality, ceiling)`` so the
# ROI-inside boost still wins when the ceiling is high enough.
MOTION_ONLY_QUALITY = max(5, min(100, int(os.environ.get(
    "LIFETRAC_MOTION_ONLY_QUALITY", "30"))))
WIREFRAME_QUALITY   = max(5, min(100, int(os.environ.get(
    "LIFETRAC_WIREFRAME_QUALITY",   "20"))))
ENCODE_MODE = max(0, min(3, int(os.environ.get("LIFETRAC_ENCODE_MODE", "0"))))


def _apply_encode_mode_quality(quality: int, mode: int) -> int:
    """Return the effective quality for ``mode`` given a requested ``quality``.

    Mode 0 / 1 pass quality through unchanged; mode 2 / 3 cap it.
    """
    if mode == ENCODE_MODE_MOTION_ONLY:
        return min(quality, MOTION_ONLY_QUALITY)
    if mode == ENCODE_MODE_WIREFRAME:
        return min(quality, WIREFRAME_QUALITY)
    return quality


def _encode_tile(rgb_canvas: bytes, tx: int, ty: int,
                 quality: int | None = None,
                 encode_mode: int | None = None) -> bytes:
    """Encode tile (tx,ty) as a WebP blob. Caller is responsible for
    keeping the blob ≤ TILE_BYTES_MAX; we degrade quality if needed.

    ``quality`` overrides the module-global :data:`WEBP_QUALITY` so the
    ROI planner can spend more bytes on operator-relevant tiles and
    aggressively compress the rest. ``None`` keeps the legacy global
    behaviour. ``encode_mode`` overrides the module-global
    :data:`ENCODE_MODE`; ``None`` reads the global so the back-channel
    dispatcher can change behaviour by mutating one int.
    """
    from PIL import Image  # type: ignore
    out = _slice_tile_rgb(rgb_canvas, tx, ty)
    img = Image.frombytes("RGB", (TILE_PX, TILE_PX), out)
    mode = ENCODE_MODE if encode_mode is None else max(0, min(3, int(encode_mode)))
    if mode != ENCODE_MODE_FULL:
        # Drop chroma. PIL's L→RGB round-trip keeps the WebP container
        # in RGB so the wire decoder doesn't need to know about modes.
        img = img.convert("L").convert("RGB")
    quality = WEBP_QUALITY if quality is None else max(5, min(100, int(quality)))
    quality = _apply_encode_mode_quality(quality, mode)
    while quality >= 5:
        buf = io.BytesIO()
        img.save(buf, format="WEBP", quality=quality, method=4)
        blob = buf.getvalue()
        if len(blob) <= TILE_BYTES_MAX:
            return blob
        quality -= 10
    # Last resort: ship a minimal WebP — accept >256 B truncation by
    # caller's policy. Returning the smallest blob we managed.
    return blob


@dataclass
class FrameAccum:
    last_canvas: bytes | None = None
    last_keyframe_t: float = 0.0
    seq: int = 0


# IP-W2-03: per-tile quality split for the ROI planner. Inside-ROI tiles
# get ``ROI_QUALITY_INSIDE`` (default 65), outside-ROI tiles get
# ``ROI_QUALITY_OUTSIDE`` (default 30) when a planner is wired in.
ROI_QUALITY_INSIDE  = max(20, min(100, int(os.environ.get(
    "LIFETRAC_ROI_QUALITY_INSIDE",  str(WEBP_QUALITY + 10)))))
ROI_QUALITY_OUTSIDE = max( 5, min(100, int(os.environ.get(
    "LIFETRAC_ROI_QUALITY_OUTSIDE", "30"))))

# IP-W2-03: optional airtime byte-budget. When set (positive int), the
# encoder drops outside-ROI changed tiles first, then oldest inside-ROI
# tiles, until the encoded body fits. ``None`` / 0 disables the cap so
# the legacy behaviour is preserved.
#
# IP-W2-04: the budget is now also adaptable at runtime from the base
# station via ``CMD_LINK_PROFILE`` so a degraded link can shrink it
# without restarting the service. The opcode picks one of
# :data:`LINK_PHY_NAMES` and an explicit fragment count, then we
# recompute bytes via :func:`max_payload_for_n_fragments`.
LINK_PHY_NAMES: tuple[str, ...] = (
    "image", "telemetry", "control_sf9", "control_sf8", "control_sf7",
)


def _compute_link_bytes(n_fragments: int, profile_name: str) -> int | None:
    """Resolve ``(n_fragments, profile_name)`` -> byte cap, or ``None``."""
    if n_fragments <= 0:
        return None
    try:
        from image_pipeline.fragment import max_payload_for_n_fragments
        from lora_proto import PHY_BY_NAME
    except Exception:                                         # pragma: no cover
        return None
    profile = PHY_BY_NAME.get(profile_name)
    if profile is None:
        return None
    try:
        return max_payload_for_n_fragments(n_fragments, profile=profile)
    except Exception:                                         # pragma: no cover
        return None


def _resolve_byte_budget() -> int | None:
    raw = os.environ.get("LIFETRAC_FRAGMENT_BUDGET", "").strip()
    if not raw:
        return None
    try:
        n = int(raw)
    except ValueError:
        return None
    if n <= 0:
        return None
    profile_name = os.environ.get("LIFETRAC_FRAGMENT_PROFILE", "image").strip() or "image"
    return _compute_link_bytes(n, profile_name)


class LinkBudget:
    """Mutable holder shared between the back-channel reader and encode loop.

    The encode loop reads ``self.bytes`` once per frame; the back-channel
    reader updates it from ``CMD_LINK_PROFILE`` without locking (single
    writer, single reader, single attribute).
    """

    __slots__ = ("bytes", "n_fragments", "profile_name")

    def __init__(self, bytes_: int | None = None,
                 n_fragments: int | None = None,
                 profile_name: str | None = None) -> None:
        self.bytes = bytes_
        self.n_fragments = n_fragments
        self.profile_name = profile_name

    def update(self, n_fragments: int, profile_index: int) -> bool:
        """Apply a CMD_LINK_PROFILE pair. Returns True iff state changed."""
        if not 0 <= profile_index < len(LINK_PHY_NAMES):
            return False
        profile_name = LINK_PHY_NAMES[profile_index]
        new_bytes = _compute_link_bytes(n_fragments, profile_name)
        if new_bytes is None:
            return False
        self.bytes = new_bytes
        self.n_fragments = n_fragments
        self.profile_name = profile_name
        return True


def _build_frame(cam, accum: FrameAccum, force_keyframe: bool,
                 *,
                 roi_planner=None,
                 byte_budget: int | None = None,
                 encode_cache=None) -> bytes:
    """Capture, diff, encode, return the wire payload (header + tiles).
    Wire format mirrors LORA_PROTOCOL.md § TileDeltaFrame so the M7 can
    forward it byte-for-byte after fragmentation.

    Optional data-saving knobs:
      * ``roi_planner`` — :class:`image_pipeline.roi.RoiPlanner`. When
        provided, inside-ROI changed tiles are encoded at
        :data:`ROI_QUALITY_INSIDE` and outside-ROI tiles at
        :data:`ROI_QUALITY_OUTSIDE`. Inside-ROI tiles are always sent
        first when a budget cap forces drops.
      * ``byte_budget`` — soft cap on the combined size of the encoded
        tile bodies (each tile contributes ``len(blob) + 1`` bytes for
        its ``tile_size_minus1`` prefix). Outside-ROI tiles are dropped
        first; if still over, the lowest-priority inside tiles are
        dropped. The header + bitmap are always emitted; dropped tiles
        are cleared from the bitmap so the parser stays in sync.
      * ``encode_cache`` — :class:`image_pipeline.tile_cache.TileEncodeCache`.
        When provided, a tile whose raw 32×32 RGB slice byte-hashes to a
        recently encoded blob is reused instead of round-tripping through
        WebP. Pure CPU saver; wire payload is unchanged. Quality changes
        invalidate the cache for the affected tile because ``quality``
        is folded into the hash key (different quality → different blob).
    """
    canvas = cam.grab_rgb()
    now = time.monotonic()
    is_key = (force_keyframe or accum.last_canvas is None or
              (now - accum.last_keyframe_t) >= KEYFRAME_PERIOD_S)

    # Build the changed-bitmap.
    n_tiles = GRID_W * GRID_H
    bitmap_bytes = (n_tiles + 7) // 8
    bitmap = bytearray(bitmap_bytes)
    if is_key:
        for i in range(n_tiles):
            bitmap[i // 8] |= (1 << (i % 8))
    else:
        prev = accum.last_canvas
        if _HAS_NUMPY:
            # IP-309: vectorize per-tile any-diff. Reshape RGB byte buffer
            # to (GRID_H, TILE_PX, GRID_W, TILE_PX, 3) and reduce over the
            # in-tile axes. ~50× faster than the row-major double loop on
            # the X8 at 384×256 / 12×8 / 32 px.
            cur_a = _np.frombuffer(canvas, dtype=_np.uint8).reshape(
                GRID_H, TILE_PX, GRID_W, TILE_PX, 3)
            prv_a = _np.frombuffer(prev, dtype=_np.uint8).reshape(
                GRID_H, TILE_PX, GRID_W, TILE_PX, 3)
            changed_grid = (cur_a != prv_a).any(axis=(1, 3, 4))  # (GH, GW)
            for ty in range(GRID_H):
                row_changed = changed_grid[ty]
                for tx in range(GRID_W):
                    if row_changed[tx]:
                        i = ty * GRID_W + tx
                        bitmap[i // 8] |= (1 << (i % 8))
        else:
            # Per-tile coarse hash: any byte difference within the tile region.
            row_stride = CANVAS_W * 3
            for ty in range(GRID_H):
                for tx in range(GRID_W):
                    changed = False
                    for ry in range(TILE_PX):
                        src_off = ((ty * TILE_PX) + ry) * row_stride + (tx * TILE_PX) * 3
                        if canvas[src_off:src_off + TILE_PX * 3] != \
                           prev[src_off:src_off + TILE_PX * 3]:
                            changed = True
                            break
                    if changed:
                        i = ty * GRID_W + tx
                        bitmap[i // 8] |= (1 << (i % 8))

    # Collect the row-major changed indices.
    changed_indices: list[int] = []
    for ty in range(GRID_H):
        for tx in range(GRID_W):
            i = ty * GRID_W + tx
            if bitmap[i // 8] & (1 << (i % 8)):
                changed_indices.append(i)

    # Decide per-tile quality + priority order using the (optional) ROI mask.
    if roi_planner is not None:
        roi_mask = roi_planner.mask()
        inside_q  = ROI_QUALITY_INSIDE
        outside_q = ROI_QUALITY_OUTSIDE
        priority: list[tuple[int, int, int]] = []   # (rank, idx, quality)
        for i in changed_indices:
            if roi_mask[i]:
                priority.append((0, i, inside_q))
            else:
                priority.append((1, i, outside_q))
    else:
        priority = [(0, i, WEBP_QUALITY) for i in changed_indices]

    # Encode in priority order so a budget cap drops the right tiles.
    # We sort by (rank, idx) so within each rank the row-major order is kept;
    # this also makes the kept set deterministic for tests.
    priority.sort(key=lambda t: (t[0], t[1]))
    kept: list[tuple[int, bytes]] = []   # (idx, blob)
    used = 0
    cap = byte_budget if (byte_budget is not None and byte_budget > 0) else None
    for _rank, i, q in priority:
        ty, tx = divmod(i, GRID_W)
        if encode_cache is not None:
            # Hash the raw 32×32 RGB slice + quality byte + encode-mode byte
            # so a re-encode at the same settings returns byte-identical blobs
            # from cache, and a CMD_ENCODE_MODE flip invalidates automatically.
            raw = _slice_tile_rgb(canvas, tx, ty)
            key_input = raw + bytes([q & 0xFF, ENCODE_MODE & 0xFF])
            blob = encode_cache.lookup(i, key_input)
            if blob is None:
                blob = _encode_tile(canvas, tx, ty, quality=q)
                encode_cache.store(i, key_input, blob)
        else:
            blob = _encode_tile(canvas, tx, ty, quality=q)
        cost = min(len(blob), TILE_BYTES_MAX) + 1   # +1 for size prefix
        if cap is not None and used + cost > cap and kept:
            # No room for this one and we've already shipped at least one tile;
            # drop this tile and every lower-priority one.
            LOG.debug("camera_service: byte_budget=%d hit at tile %d (used=%d, cost=%d); dropping remainder",
                      cap, i, used, cost)
            break
        kept.append((i, blob))
        used += cost

    # Rebuild the bitmap from the kept set so dropped tiles' bits are cleared.
    kept.sort(key=lambda kv: kv[0])
    bitmap = bytearray(bitmap_bytes)
    for i, _blob in kept:
        bitmap[i // 8] |= (1 << (i % 8))

    # Header per LORA_PROTOCOL.md § TileDeltaFrame:
    #   frame_kind (1) | seq (1) | grid_w (1) | grid_h (1) | tile_px (1)
    #   | changed_bitmap (bitmap_bytes)
    accum.seq = (accum.seq + 1) & 0xFF
    header = struct.pack("BBBBB",
                         1 if is_key else 0,
                         accum.seq, GRID_W, GRID_H, TILE_PX) + bytes(bitmap)
    body = bytearray()
    for _i, blob in kept:
        # tile_size_minus1 is u8 → max 256 B
        size = min(len(blob), TILE_BYTES_MAX)
        body.append(size - 1)
        body.extend(blob[:size])

    accum.last_canvas = canvas
    if is_key:
        accum.last_keyframe_t = now
    return bytes(header + body)


# ---- M7 → X8 back-channel (IP-104 follow-on) -------------------------
#
# The M7 forwards CMD_REQ_KEYFRAME / CMD_CAMERA_SELECT / CMD_ENCODE_MODE /
# CMD_CAMERA_QUALITY over the same UART that carries image fragments,
# KISS-framed with the reserved ``X8_CMD_TOPIC = 0xC0`` topic byte.
# These constants and the pure dispatcher live at module scope so the
# unit tests can drive them without spinning up ``main()``.

X8_CMD_TOPIC       = 0xC0
CMD_REQ_KEYFRAME   = 0x62
CMD_CAMERA_SELECT  = 0x03
CMD_CAMERA_QUALITY = 0x04
CMD_ENCODE_MODE    = 0x63
CMD_ROI_HINT       = 0x61
CMD_LINK_PROFILE   = 0x64
X8_KISS_FEND, X8_KISS_FESC = 0xC0, 0xDB
X8_KISS_TFEND, X8_KISS_TFESC = 0xDC, 0xDD


def dispatch_back_channel(frame: bytes, force_key_evt, *,
                          roi_planner=None,
                          link_budget: "LinkBudget | None" = None) -> None:
    """Pure dispatcher for one decoded back-channel frame.

    ``force_key_evt`` is a ``threading.Event`` (the encode loop reads
    ``.is_set()`` / clears it). ``roi_planner`` is an optional
    :class:`image_pipeline.roi.RoiPlanner`; when present, ``CMD_ROI_HINT``
    forwards the operator-painted rectangle into it. ``link_budget`` is
    an optional :class:`LinkBudget` mutated by ``CMD_LINK_PROFILE``.
    Returning early on malformed input keeps the reader thread tolerant
    of a noisy UART.
    """
    # Frame layout: <topic_id:1> <opcode:1> <args:N>
    if len(frame) < 2 or frame[0] != X8_CMD_TOPIC:
        return
    opcode = frame[1]
    if opcode == CMD_REQ_KEYFRAME:
        force_key_evt.set()
    elif opcode == CMD_CAMERA_SELECT:
        cam_idx = frame[2] if len(frame) >= 3 else 0
        LOG.info("camera_service: CMD_CAMERA_SELECT cam=%d (no-op)", cam_idx)
    elif opcode == CMD_CAMERA_QUALITY:
        if len(frame) >= 3:
            q = max(20, min(100, frame[2]))
            global WEBP_QUALITY  # noqa: PLW0603
            WEBP_QUALITY = q
            LOG.info("camera_service: CMD_CAMERA_QUALITY -> %d", q)
    elif opcode == CMD_ENCODE_MODE:
        if len(frame) >= 3:
            raw_mode = frame[2]
            if 0 <= raw_mode <= 3:
                global ENCODE_MODE  # noqa: PLW0603
                ENCODE_MODE = raw_mode
                LOG.info("camera_service: CMD_ENCODE_MODE -> %d (%s)",
                         raw_mode, ENCODE_MODE_NAMES[raw_mode])
            else:
                LOG.warning("camera_service: CMD_ENCODE_MODE rejected (mode=%d)",
                            raw_mode)
    elif opcode == CMD_ROI_HINT:
        # Args: col_lo, col_hi, row_lo, row_hi (each u8, tile coords).
        if roi_planner is not None and len(frame) >= 6:
            roi_planner.apply_hint(frame[2], frame[3], frame[4], frame[5])
            LOG.info("camera_service: CMD_ROI_HINT col=%d..%d row=%d..%d",
                     frame[2], frame[3], frame[4], frame[5])
    elif opcode == CMD_LINK_PROFILE:
        # Args: n_fragments u8, profile_index u8 (index into LINK_PHY_NAMES).
        if link_budget is not None and len(frame) >= 4:
            n_frag = frame[2]
            phy_idx = frame[3]
            if link_budget.update(n_frag, phy_idx):
                LOG.info("camera_service: CMD_LINK_PROFILE n=%d phy=%s -> budget=%d B",
                         n_frag, link_budget.profile_name, link_budget.bytes)
            else:
                LOG.warning("camera_service: CMD_LINK_PROFILE rejected (n=%d phy_idx=%d)",
                            n_frag, phy_idx)
    # Force a keyframe on any reconfiguration so the new settings are
    # immediately observable downstream.
    if opcode in (CMD_CAMERA_SELECT, CMD_CAMERA_QUALITY, CMD_ENCODE_MODE,
                  CMD_ROI_HINT, CMD_LINK_PROFILE):
        force_key_evt.set()


# ---- entry point ------------------------------------------------------

def main() -> None:
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")

    cam = _make_camera()
    accum = FrameAccum()
    # IP-208: replace the `force_key_flag` dict (which was mutated from the
    # MQTT thread and read from the encode loop) with a thread-safe Event.
    import threading
    force_key_evt = threading.Event()

    # IP-104: open the X8 → H747 UART writer. Lazy import keeps the
    # synthetic-camera test path free of pyserial requirements.
    from image_pipeline.ipc_to_h747 import IpcWriter
    ipc = IpcWriter(device=M7_UART_DEVICE)
    ipc.open()

    # IP-W2-03: optional ROI planner + airtime byte-budget. Both are
    # opt-in via env vars so the default service behaves identically to
    # before this change.
    roi_planner = None
    if os.environ.get("LIFETRAC_ROI_ENABLE", "").strip() == "1":
        try:
            from image_pipeline.roi import RoiPlanner
            roi_planner = RoiPlanner()
            roi_planner.update_mode(
                os.environ.get("LIFETRAC_ROI_DEFAULT_MODE", "idle"))
            LOG.info("camera_service: ROI planner enabled (mode=%s, q_in=%d, q_out=%d)",
                     roi_planner.mode, ROI_QUALITY_INSIDE, ROI_QUALITY_OUTSIDE)
        except Exception as exc:                              # pragma: no cover
            LOG.warning("camera_service: ROI planner unavailable (%s)", exc)
            roi_planner = None
    byte_budget = _resolve_byte_budget()
    link_budget = LinkBudget(
        bytes_=byte_budget,
        n_fragments=None,
        profile_name=os.environ.get("LIFETRAC_FRAGMENT_PROFILE", "image").strip() or "image",
    )
    if byte_budget is not None:
        LOG.info("camera_service: byte_budget=%d B/frame (LIFETRAC_FRAGMENT_BUDGET, phy=%s)",
                 byte_budget, link_budget.profile_name)

    # IP-W2-05: optional per-tile WebP encode cache. Opt-in so the W2-03 /
    # W2-04 hot-path tests stay byte-equivalent to the no-cache path.
    encode_cache = None
    if os.environ.get("LIFETRAC_TILE_CACHE_ENABLE", "").strip() == "1":
        try:
            from image_pipeline.tile_cache import TileEncodeCache
            history = max(1, int(os.environ.get("LIFETRAC_TILE_CACHE_HISTORY", "4")))
            encode_cache = TileEncodeCache(n_tiles=GRID_W * GRID_H, history=history)
            LOG.info("camera_service: tile encode cache enabled (history=%d)", history)
        except Exception as exc:                              # pragma: no cover
            LOG.warning("camera_service: tile encode cache unavailable (%s)", exc)
            encode_cache = None

    def _back_channel_reader() -> None:
        try:
            rfh = open(M7_UART_DEVICE, "rb", buffering=0)
        except OSError as exc:
            LOG.warning("camera_service: back-channel disabled (%s)", exc)
            return
        # Minimal KISS state machine. ``buf`` accumulates one decoded frame.
        buf = bytearray()
        in_frame = False
        escaped  = False
        while True:
            try:
                chunk = rfh.read(64)
            except OSError as exc:
                LOG.warning("camera_service: back-channel read failed (%s)", exc)
                return
            if not chunk:
                continue
            for b in chunk:
                if b == X8_KISS_FEND:
                    if in_frame and buf:
                        dispatch_back_channel(bytes(buf), force_key_evt,
                                              roi_planner=roi_planner,
                                              link_budget=link_budget)
                    buf.clear()
                    in_frame = True
                    escaped = False
                    continue
                if not in_frame:
                    continue
                if escaped:
                    if   b == X8_KISS_TFEND: buf.append(X8_KISS_FEND)
                    elif b == X8_KISS_TFESC: buf.append(X8_KISS_FESC)
                    escaped = False
                elif b == X8_KISS_FESC:
                    escaped = True
                else:
                    buf.append(b)

    threading.Thread(target=_back_channel_reader, name="x8-back-channel",
                     daemon=True).start()

    client = None
    if DEBUG_MQTT:
        try:
            import paho.mqtt.client as mqtt
        except ImportError:
            LOG.error("camera_service: paho-mqtt not installed; debug MQTT disabled")
        else:
            client = mqtt.Client(client_id="camera_service")

            def _on_msg(_c, _u, _msg):
                # CMD_REQ_KEYFRAME forwarded by the M7. Payload contents are
                # ignored; receipt alone is the trigger.
                force_key_evt.set()

            client.on_message = _on_msg
            client.connect(MQTT_HOST, 1883)
            client.subscribe(KEYFRAME_REQ_TOPIC, qos=1)
            client.loop_start()

    period = 1.0 / max(TARGET_FPS, 0.1)
    next_t = time.monotonic()
    while True:
        force = force_key_evt.is_set()
        force_key_evt.clear()
        try:
            payload = _build_frame(cam, accum, force_keyframe=force,
                                   roi_planner=roi_planner,
                                   byte_budget=link_budget.bytes,
                                   encode_cache=encode_cache)
            # Primary: UART to the M7 (length-framed).
            try:
                ipc.write(payload, is_keyframe=force)
            except OSError as exc:
                LOG.warning("camera_service: UART write failed (%s); reopening", exc)
                ipc.close()
                ipc.open()
            # Optional: forensic MQTT mirror.
            if client is not None:
                client.publish(PUBLISH_TOPIC, payload, qos=0)
        except Exception as exc:                              # pragma: no cover
            LOG.exception("camera_service: frame build failed (%s)", exc)
        next_t += period
        sleep_s = next_t - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            # We slipped — resync rather than burn CPU catching up.
            next_t = time.monotonic()


if __name__ == "__main__":
    main()
