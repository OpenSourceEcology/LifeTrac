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

GRID_W = int(os.environ.get("LIFETRAC_GRID_W", "12"))
GRID_H = int(os.environ.get("LIFETRAC_GRID_H", "8"))
TILE_PX = int(os.environ.get("LIFETRAC_TILE_PX", "32"))
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
# 2026-05-27 W2-02 strict-path bridge: when set, the daemon hands
# /dev/ttymxc3 off to image_tx_daemon (which speaks the Method-G
# HostLink protocol the L072 actually understands) and routes every
# TileDeltaFrame over MQTT instead of writing to the M7 directly.
# Implies DEBUG_MQTT=1.
USE_LORA_BRIDGE   = os.environ.get("LIFETRAC_USE_LORA_BRIDGE", "").strip() == "1"
if USE_LORA_BRIDGE:
    DEBUG_MQTT = True

PUBLISH_TOPIC     = "lifetrac/v25/cmd/image_frame"
KEYFRAME_REQ_TOPIC = "lifetrac/v25/cmd/req_keyframe"


# ---- capture backends -------------------------------------------------

class SyntheticCamera:
    """Test-only source: emits a slowly-shifting gradient. Useful when the
    X8 has no MIPI sensor wired up yet — exercises the diff/encode path
    without needing libcamera.

    2026-05-27 W2-02 LoRa bridge: when ``LIFETRAC_SYNTHETIC_MODE=delta``
    the gradient is FROZEN and only one tile per second flips between
    two values. This keeps P-frames tiny (~1 tile of WebP) so the SF7
    fragment budget actually has headroom — the original scrolling mode
    re-encodes every pixel every frame and overflows the 256-fragment
    ceiling.
    """

    _MODE = os.environ.get("LIFETRAC_SYNTHETIC_MODE", "scroll").strip().lower()

    def __init__(self) -> None:
        self._t = 0

    def grab_rgb(self) -> bytes:
        # 3 bytes/pixel, scrolling diagonal stripes so P-frames have
        # something to encode.
        self._t = (self._t + 1) & 0xFF
        out = bytearray(CANVAS_W * CANVAS_H * 3)
        if self._MODE == "delta":
            # Frozen background gradient; one tile (top-left 32×32)
            # alternates colour every grab so exactly one tile shows up
            # in the diff. Background uses t=0 so the gradient never
            # moves underneath.
            t = 0
            for y in range(CANVAS_H):
                for x in range(CANVAS_W):
                    v = (x + y) & 0xFF
                    i = (y * CANVAS_W + x) * 3
                    out[i]     = v
                    out[i + 1] = (v << 1) & 0xFF
                    out[i + 2] = (255 - v) & 0xFF
            # Paint the top-left tile a flat colour that toggles per
            # grab. Keeps the magnitude bin well above the diff floor.
            flat = 0x40 if (self._t & 1) else 0xC0
            for y in range(min(TILE_PX, CANVAS_H)):
                for x in range(min(TILE_PX, CANVAS_W)):
                    i = (y * CANVAS_W + x) * 3
                    out[i]     = flat
                    out[i + 1] = flat
                    out[i + 2] = flat
            return bytes(out)
        # Legacy scrolling mode (default — preserves the original
        # behaviour for any non-LoRa-bridge consumers).
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


_default_deshake = "0" if USE_LORA_BRIDGE else "1"
LIFETRAC_CAMERA_DESHAKE = os.environ.get("LIFETRAC_CAMERA_DESHAKE", _default_deshake) == "1"


def build_ffmpeg_argv(device: str = V4L2_DEVICE,
                      input_format: str = V4L2_INPUT_FORMAT,
                      input_size: str = V4L2_INPUT_SIZE,
                      input_fps: int = V4L2_INPUT_FPS,
                      ffmpeg_path: str = FFMPEG_PATH,
                      output_fps: float = TARGET_FPS,
                      canvas_w: int = CANVAS_W,
                      canvas_h: int = CANVAS_H) -> list[str]:
    """Argv for the bench-validated USB-camera capture pipeline.

    Mirrors the 2026-05-15 invocation that produced the first real frame
    off the Kurokesu C2:

        ffmpeg -hide_banner -loglevel error \
               -f v4l2 -input_format mjpeg \
               -video_size 1920x1080 -framerate 30 \
               -i /dev/video1 \
               -vf scale=384:256 -pix_fmt rgb24 \
               -f rawvideo -

    Pure helper so the unit tests can pin the argv shape without spawning
    a subprocess.
    """
    # Keep capture latency bounded: emit at the same rate the encoder loop
    # consumes so ffmpeg stdout does not accumulate stale rawvideo frames.
    target_fps = max(0.1, float(output_fps))
    scale_filter = f"fps={target_fps:g},scale={canvas_w}:{canvas_h}"
    if LIFETRAC_CAMERA_DESHAKE:
        scale_filter += ",deshake=open2=1:search=16"

    return [
        ffmpeg_path,
        "-hide_banner", "-loglevel", "error",
        "-f", "v4l2",
        "-input_format", input_format,
        "-video_size", input_size,
        "-framerate", str(int(input_fps)),
        "-thread_queue_size", "2",
        "-fflags", "nobuffer",
        "-flags", "low_delay",
        "-analyzeduration", "0",
        "-probesize", "32",
        "-i", device,
        "-vf", scale_filter,
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
ENCODE_MODE_FULL           = 0
ENCODE_MODE_Y_ONLY         = 1
ENCODE_MODE_MOTION_ONLY    = 2
ENCODE_MODE_WIREFRAME      = 3
# Bench-rotation modes from AI NOTES/2026-05-25_Grayscale_Quantization_*
# Encoder branches land in a follow-up patch; until then the receiver
# logs the requested mode and clamps to Y_ONLY so the link does not
# crash on a CMD_ENCODE_MODE for an unimplemented value.
ENCODE_MODE_BTC4_PER_TILE  = 4
ENCODE_MODE_BTC4_PER_FRAME = 5
ENCODE_MODE_MONO_G4        = 6
ENCODE_MODE_ADAPTIVE       = 7
ENCODE_MODE_RAWSTREAM      = 8
ENCODE_MODE_NAMES = (
    "full", "y_only", "motion_only", "wireframe",
    "btc4_per_tile", "btc4_per_frame", "mono_g4", "adaptive", "rawstream",
)
# Modes whose encoder is actually implemented today. Anything outside
# this set is silently clamped to ENCODE_MODE_Y_ONLY at the receive
# boundary so a stale base-station can't brick the tractor's video.
_ENCODE_MODE_IMPLEMENTED = frozenset({
    ENCODE_MODE_FULL,
    ENCODE_MODE_Y_ONLY,
    ENCODE_MODE_MOTION_ONLY,
    ENCODE_MODE_WIREFRAME,
    ENCODE_MODE_MONO_G4,
    ENCODE_MODE_RAWSTREAM,
})


def _clamp_encode_mode(requested: int) -> int:
    """Clamp ``requested`` to an implemented encoder mode.

    Unknown or unimplemented values fall back to ``ENCODE_MODE_Y_ONLY`` so
    the tractor never refuses to encode just because the base sent a
    forward-compatible mode this build doesn't ship yet.
    """
    try:
        m = int(requested)
    except (TypeError, ValueError):
        return ENCODE_MODE_Y_ONLY
    if m in _ENCODE_MODE_IMPLEMENTED:
        return m
    return ENCODE_MODE_Y_ONLY
# Quality ceilings applied as ``min(requested_quality, ceiling)`` so the
# ROI-inside boost still wins when the ceiling is high enough.
MOTION_ONLY_QUALITY = max(5, min(100, int(os.environ.get(
    "LIFETRAC_MOTION_ONLY_QUALITY", "30"))))
WIREFRAME_QUALITY   = max(5, min(100, int(os.environ.get(
    "LIFETRAC_WIREFRAME_QUALITY",   "20"))))
ENCODE_MODE = _clamp_encode_mode(int(os.environ.get("LIFETRAC_ENCODE_MODE", "0")))

# Per-frame codec ids (mirrors base_station/image_pipeline/frame_format.py).
# Kept duplicated to avoid importing the base-station tree from the tractor.
CODEC_WEBP            = 0
CODEC_MONO_G4         = 1
CODEC_BTC4_PER_TILE   = 2
CODEC_BTC4_PER_FRAME  = 3
CODEC_WEBP_LUMA       = 4   # grayscale WebP; base routes through Recolouriser
CODEC_WEBP_RAWSTREAM  = 5   # raw WebP VP8/VP8L bitstream (container-stripped)

# Map operator-selected EncodeMode -> wire codec for the frame header.
# Modes whose encoders are not implemented yet still use CODEC_WEBP
# because ``_clamp_encode_mode`` demotes them to Y_ONLY. Each later step
# (MONO_G4, BTC4_*, ADAPTIVE) flips one row of this table.
_ENCODE_MODE_CODEC: dict[int, int] = {
    ENCODE_MODE_FULL:            CODEC_WEBP,
    ENCODE_MODE_Y_ONLY:          CODEC_WEBP_LUMA,
    ENCODE_MODE_MOTION_ONLY:     CODEC_WEBP_LUMA,
    ENCODE_MODE_WIREFRAME:       CODEC_WEBP_LUMA,
    ENCODE_MODE_BTC4_PER_TILE:   CODEC_WEBP_LUMA,   # placeholder; clamped to Y_ONLY
    ENCODE_MODE_BTC4_PER_FRAME:  CODEC_WEBP_LUMA,   # placeholder; clamped to Y_ONLY
    ENCODE_MODE_MONO_G4:         CODEC_MONO_G4,
    ENCODE_MODE_ADAPTIVE:        CODEC_WEBP_LUMA,   # placeholder; clamped to Y_ONLY
    ENCODE_MODE_RAWSTREAM:       CODEC_WEBP_RAWSTREAM,
}

# Deployment knob: strip WebP containers for FULL-mode tiles without a
# CMD_ENCODE_MODE round trip. Honored by BOTH _encode_tile (strips the
# bytes) and _codec_for_mode (flips the wire codec byte) so the two can
# never desynchronise. Read once at import; the per-mode RAWSTREAM path
# does not depend on it.
CONTAINER_STRIP = os.environ.get("LIFETRAC_CONTAINER_STRIP", "0") == "1"


def _codec_for_mode(mode: int) -> int:
    """Return the wire codec for ``mode`` after the safe-clamp.

    Always returns the codec of the *actually-shipped* mode, so the
    base-station decoder dispatches to the matching path. This is the
    single point where the encoder ladder and the wire-format byte are
    kept consistent.
    """
    codec = _ENCODE_MODE_CODEC.get(_clamp_encode_mode(mode), CODEC_WEBP)
    if CONTAINER_STRIP and codec == CODEC_WEBP:
        return CODEC_WEBP_RAWSTREAM   # matches _encode_tile's strip path
    return codec


def _apply_encode_mode_quality(quality: int, mode: int) -> int:
    """Return the effective quality for ``mode`` given a requested ``quality``.

    Mode 0 / 1 pass quality through unchanged; mode 2 / 3 cap it.
    """
    if mode == ENCODE_MODE_MOTION_ONLY:
        return min(quality, MOTION_ONLY_QUALITY)
    if mode == ENCODE_MODE_WIREFRAME:
        return min(quality, WIREFRAME_QUALITY)
    return quality


def _encode_tile_mono_g4(img) -> bytes:
    """Encode a 32x32 RGB tile as MONO_G4 (1-bit Floyd-Steinberg + zlib).

    Wire layout matches ``base_station/image_pipeline/codec_decode.py
    :_decode_mono_g4``:

        u8  flag         ; bit0 set iff payload is zlib-compressed
        u8  payload[]    ; raw 1bpp packed (MSB-first row-major) or zlib

    We compress with zlib level 9 and fall back to raw packed bits when
    zlib actually grows the payload (rare on noisy tiles). Result is
    bounded by ``1 + (TILE_PX*TILE_PX)//8 = 129 B`` worst case, so it
    always fits the ``tile_size_minus1`` u8 envelope.
    """
    import zlib
    bits_packed = img.convert("1").tobytes()  # PIL emits MSB-first packed.
    expected = (TILE_PX * TILE_PX + 7) // 8
    if len(bits_packed) != expected:                # pragma: no cover
        # PIL's "1" mode is documented to pack tightly; this guard is
        # defensive for the (unsupported) non-multiple-of-8 tile case.
        bits_packed = bits_packed[:expected].ljust(expected, b"\x00")
    zbuf = zlib.compress(bits_packed, level=9)
    if len(zbuf) < len(bits_packed):
        return b"\x01" + zbuf
    return b"\x00" + bits_packed


def _encode_tile(rgb_canvas: bytes, tx: int, ty: int,
                 quality: int | None = None,
                 encode_mode: int | None = None,
                 is_key: bool = False) -> bytes | None:
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
    mode = ENCODE_MODE if encode_mode is None else _clamp_encode_mode(encode_mode)
    if mode == ENCODE_MODE_MONO_G4:
        return _encode_tile_mono_g4(img)
    if mode not in (ENCODE_MODE_FULL, ENCODE_MODE_RAWSTREAM):
        # Drop chroma. PIL's L→RGB round-trip keeps the WebP container
        # in RGB so the wire decoder doesn't need to know about modes.
        # RAWSTREAM is exempt: it is FULL-colour with the container
        # stripped, NOT a degraded mode (2026-07-24 fix — mode 8 was
        # silently grayscale because this branch caught it).
        img = img.convert("L").convert("RGB")
    quality = WEBP_QUALITY if quality is None else max(5, min(100, int(quality)))
    quality = _apply_encode_mode_quality(quality, mode)
    # Strip only when the wire codec byte will say RAWSTREAM (see
    # _codec_for_mode) — stripping under any other codec id ships tiles
    # the base cannot decode (2026-07-24 fix: the env knob used to strip
    # while the codec byte stayed CODEC_WEBP).
    strip_container = (mode == ENCODE_MODE_RAWSTREAM
                       or (CONTAINER_STRIP and mode == ENCODE_MODE_FULL))
    method = 6 if is_key else 4
    while quality >= 5:
        buf = io.BytesIO()
        img.save(buf, format="WEBP", quality=quality, method=method)
        blob = buf.getvalue()
        if len(blob) <= TILE_BYTES_MAX:
            if strip_container and len(blob) >= 20 and blob.startswith(b"RIFF"):
                blob = blob[20:]
            return blob
        quality -= 10
    # F14: NEVER return >256 B — _build_frame would ship blob[:256], a
    # truncated RIFF container the base cannot decode. Grayscale retry, then drop.
    gray = img.convert("L").convert("RGB")
    buf = io.BytesIO()
    gray.save(buf, format="WEBP", quality=5, method=6)
    blob = buf.getvalue()
    if len(blob) <= TILE_BYTES_MAX:
        if strip_container and len(blob) >= 20 and blob.startswith(b"RIFF"):
            blob = blob[20:]
        return blob
    LOG.warning("tile unencodable <=%d B even gray-q5 (%d B); dropping",
                TILE_BYTES_MAX, len(blob))
    return None


@dataclass
class FrameAccum:
    last_canvas: bytes | None = None
    last_keyframe_t: float = 0.0
    seq: int = 0
    # IP-PlanRev (Method C): per-tile monotonic counter of when each tile
    # was last *shipped*. Used by the rotating fair-share sweep so the
    # canvas eventually paints in full even when motion is concentrated
    # in one ROI. Lazily allocated on first use to keep Method A's
    # FrameAccum() construction signature unchanged.
    tile_last_seq: list[int] | None = None
    sweep_seq: int = 0


# IP-PlanRev: image-pipeline plan revision selector. Letter scheme parallel
# to the LoRa "Method G" tracker. See
# ``LifeTrac-v25/DESIGN-CONTROLLER/IMAGE_PIPELINE_METHODS.md``.
#
#   A  shipping behaviour: byte-diff bitmap + (rank, idx) row-major priority.
#   B  magnitude-ranked priority: tile-level L1 diff above a noise floor;
#      changed tiles encoded in descending magnitude so a tight byte budget
#      spends bytes on actual motion, not on the top-left strip.
#   C  Method B + rotating fair-share sweep: each P-frame promotes
#      ``LIFETRAC_SWEEP_STEP`` of the longest-unsent tiles into the
#      changed bitmap so a still scene still converges to full coverage.
# Default selection:
#   * LoRa-bridge mode: Method C so we keep emitting useful tile deltas even
#     when scene motion is low/noisy and avoid long runs of empty 9-byte P-frames.
#   * Non-bridge mode: preserve legacy Method A behaviour unless explicitly set.
_default_image_method = "C" if USE_LORA_BRIDGE else "A"
IMAGE_METHOD = os.environ.get("LIFETRAC_IMAGE_METHOD", _default_image_method).strip().upper()
if IMAGE_METHOD not in ("A", "B", "C"):
    LOG.warning("LIFETRAC_IMAGE_METHOD=%r unknown; falling back to %s",
                IMAGE_METHOD, _default_image_method)
    IMAGE_METHOD = _default_image_method

# Method B/C: L1-magnitude floor (sum of |cur-prev| over the 32×32×3 RGB
# tile). In LoRa-bridge mode keep a low-but-nonzero default: high values can
# misclassify slow real motion as static and make the UI appear frozen.
_default_tile_mag_min = "4000" if USE_LORA_BRIDGE else "8000"
TILE_MAGNITUDE_MIN = max(0, int(os.environ.get(
    "LIFETRAC_TILE_MAGNITUDE_MIN", _default_tile_mag_min)))

# Method C: how many "stale" tiles to force into each P-frame's changed
# bitmap. With SWEEP_STEP=2 and a 1 fps loop, a static 96-tile canvas
# converges to full coverage in ~48 s even with zero motion. 0 disables.
SWEEP_STEP = max(0, int(os.environ.get("LIFETRAC_SWEEP_STEP", "2")))

LOG.info("IP-PlanRev: IMAGE_METHOD=%s TILE_MAGNITUDE_MIN=%d SWEEP_STEP=%d numpy=%s",
         IMAGE_METHOD, TILE_MAGNITUDE_MIN, SWEEP_STEP, _HAS_NUMPY)


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
    "image_bw250", "image_bw500",   # append-only: wire indices 0-4 stay stable
)


def _compute_link_bytes(n_fragments: int, profile_name: str) -> int | None:
    """Resolve ``(n_fragments, profile_name)`` -> byte cap, or ``None``."""
    if n_fragments <= 0:
        return None
    # Conservative fallback used when protocol helpers are unavailable in a
    # stripped runtime image: keep payload small enough to preserve recency.
    fallback_bytes = n_fragments * 40
    try:
        from image_pipeline.fragment import max_payload_for_n_fragments
        from lora_proto import PHY_BY_NAME
    except Exception:                                         # pragma: no cover
        return fallback_bytes
    profile = PHY_BY_NAME.get(profile_name)
    if profile is None:
        return fallback_bytes
    try:
        return max_payload_for_n_fragments(n_fragments, profile=profile)
    except Exception:                                         # pragma: no cover
        return fallback_bytes


def _resolve_byte_budget() -> int | None:
    raw = os.environ.get("LIFETRAC_FRAGMENT_BUDGET", "").strip()
    if not raw and USE_LORA_BRIDGE:
        # Keep bridge mode responsive by default: cap each frame to a
        # LoRa-friendly fragment budget unless the unit file overrides it.
        raw = os.environ.get("LIFETRAC_FRAGMENT_BUDGET_BRIDGE_DEFAULT", "12").strip()
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

    # IP-PlanRev Method B/C: per-tile L1 magnitudes (sum of |cur-prev| over
    # the 32×32×3 RGB tile). ``magnitudes[i] == 0`` for I-frames or when
    # numpy is unavailable. Computed once and reused for both the bitmap
    # decision and the priority sort.
    magnitudes: list[int] = [0] * n_tiles
    if (not is_key and accum.last_canvas is not None
            and IMAGE_METHOD in ("B", "C") and _HAS_NUMPY):
        prev = accum.last_canvas
        cur_a = _np.frombuffer(canvas, dtype=_np.uint8).reshape(
            GRID_H, TILE_PX, GRID_W, TILE_PX, 3).astype(_np.int16)
        prv_a = _np.frombuffer(prev, dtype=_np.uint8).reshape(
            GRID_H, TILE_PX, GRID_W, TILE_PX, 3).astype(_np.int16)
        mag_grid = _np.abs(cur_a - prv_a).sum(axis=(1, 3, 4))
        for ty in range(GRID_H):
            for tx in range(GRID_W):
                magnitudes[ty * GRID_W + tx] = int(mag_grid[ty, tx])

    if is_key:
        for i in range(n_tiles):
            bitmap[i // 8] |= (1 << (i % 8))
    elif IMAGE_METHOD in ("B", "C") and _HAS_NUMPY and accum.last_canvas is not None:
        # Method B/C: motion-aware bitmap. A tile is "changed" iff its L1
        # magnitude exceeds TILE_MAGNITUDE_MIN — kills sensor-noise
        # false-positives that otherwise mark every tile changed under
        # the legacy byte-diff path.
        for i, m in enumerate(magnitudes):
            if m > TILE_MAGNITUDE_MIN:
                bitmap[i // 8] |= (1 << (i % 8))
    else:
        # Method A (and Method B/C fallback when numpy is missing): legacy
        # byte-diff bitmap.
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

    # Method C: promote the SWEEP_STEP longest-unsent tiles so a still
    # scene (or a numpy-less Method-A fallback) still converges to full
    # coverage instead of stalling on whatever was in the last keyframe.
    # Applies on P-frames only — keyframes already set every bit, but
    # the priority sort below uses tile_last_seq to rotate them.
    sweep_indices: set[int] = set()
    if IMAGE_METHOD == "C" and not is_key and SWEEP_STEP > 0:
        if accum.tile_last_seq is None:
            accum.tile_last_seq = [0] * n_tiles
        unsent = [i for i in range(n_tiles)
                  if not (bitmap[i // 8] & (1 << (i % 8)))]
        unsent.sort(key=lambda j: accum.tile_last_seq[j])  # oldest first
        for j in unsent[:SWEEP_STEP]:
            bitmap[j // 8] |= (1 << (j % 8))
            sweep_indices.add(j)

    # Collect the row-major changed indices.
    changed_indices: list[int] = []
    for ty in range(GRID_H):
        for tx in range(GRID_W):
            i = ty * GRID_W + tx
            if bitmap[i // 8] & (1 << (i % 8)):
                changed_indices.append(i)

    # Decide per-tile quality + priority order using the (optional) ROI mask.
    # IP-PlanRev:
    #   Method A — (rank, idx) so the lowest-index tiles always win. This
    #              is what shipped first; under a tight byte budget it
    #              produces the well-known top-left-strip starvation
    #              symptom documented in
    #              ``AI NOTES/2026-05-25_Live_Camera_Pipeline.md``.
    #   Method B/C — (rank, -magnitude, idx) so the most-changed tiles win
    #              within each ROI rank. idx is the deterministic tie
    #              breaker for tests.
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
    if IMAGE_METHOD == "C" and is_key and accum.tile_last_seq is not None:
        # Method C keyframe rotation: when a tight byte budget clips the
        # keyframe (typical at LIFETRAC_FRAGMENT_BUDGET≈250), ship the
        # longest-unsent tiles first so successive keyframes rotate
        # through the canvas instead of re-shipping the top-left strip
        # every 10 s. tile_last_seq=0 → never shipped → wins first.
        priority.sort(key=lambda t: (t[0], accum.tile_last_seq[t[1]], t[1]))
    elif IMAGE_METHOD in ("B", "C") and not is_key:
        # Under tight byte budgets, sweep-first ordering can starve true
        # motion (the sweep tiles are often static and consume all slots).
        # Keep sweep as a fallback, but send measured motion first.
        priority.sort(key=lambda t: (
            t[0],
            1 if t[1] in sweep_indices else 0,
            -magnitudes[t[1]],
            t[1],
        ))
    else:
        # Method A (and Method B/C first-frame): (rank, idx) row-major.
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
                blob = _encode_tile(canvas, tx, ty, quality=q, is_key=is_key)
                if blob is not None:
                    encode_cache.store(i, key_input, blob)
        else:
            blob = _encode_tile(canvas, tx, ty, quality=q, is_key=is_key)
        if blob is None:
            continue
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
    #   | codec (1) | changed_bitmap (bitmap_bytes)
    accum.seq = (accum.seq + 1) & 0xFF
    codec = _codec_for_mode(ENCODE_MODE)
    header = struct.pack("BBBBBB",
                         1 if is_key else 0,
                         accum.seq, GRID_W, GRID_H, TILE_PX,
                         codec) + bytes(bitmap)
    body = bytearray()
    for _i, blob in kept:
        # tile_size_minus1 is u8 → max 256 B
        size = min(len(blob), TILE_BYTES_MAX)
        body.append(size - 1)
        body.extend(blob[:size])

    accum.last_canvas = canvas
    if is_key:
        accum.last_keyframe_t = now
    # IP-PlanRev Method C: track ship age per tile so the next P-frame's
    # sweep promotes the longest-unsent tiles. Updated for every method
    # so a runtime switch from A → C immediately has useful data.
    if accum.tile_last_seq is None:
        accum.tile_last_seq = [0] * n_tiles
    accum.sweep_seq += 1
    for i, _blob in kept:
        accum.tile_last_seq[i] = accum.sweep_seq
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
            effective = _clamp_encode_mode(raw_mode)
            global ENCODE_MODE  # noqa: PLW0603
            ENCODE_MODE = effective
            if effective == raw_mode:
                LOG.info("camera_service: CMD_ENCODE_MODE -> %d (%s)",
                         effective, ENCODE_MODE_NAMES[effective])
            else:
                # Forward-compatible mode this build can't encode yet.
                # We accept it on the wire so the base can stay current,
                # but log clearly that we fell back so the operator can
                # tell the bench dashboard from the running encoder.
                req_name = (ENCODE_MODE_NAMES[raw_mode]
                            if 0 <= raw_mode < len(ENCODE_MODE_NAMES)
                            else f"unknown({raw_mode})")
                LOG.warning(
                    "camera_service: CMD_ENCODE_MODE %d (%s) not implemented; "
                    "clamping to %d (%s)",
                    raw_mode, req_name, effective, ENCODE_MODE_NAMES[effective],
                )
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
    # 2026-05-27 W2-02: when LIFETRAC_USE_LORA_BRIDGE=1, image_tx_daemon
    # owns /dev/ttymxc3 (HostLink protocol) and we mirror frames over
    # MQTT only, so skip the M7 IpcWriter entirely.
    if USE_LORA_BRIDGE:
        LOG.info("camera_service: LIFETRAC_USE_LORA_BRIDGE=1 — skipping "
                 "M7 IpcWriter; frames routed to MQTT %s only", PUBLISH_TOPIC)
        ipc = None
    else:
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

    if not USE_LORA_BRIDGE:
        # The back-channel reader opens M7_UART_DEVICE for read — skip it
        # under the LoRa-bridge path so we never contend with image_tx_daemon.
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
    frame_health_log = os.environ.get("LIFETRAC_CAMERA_HEALTH_LOG", "").strip() == "1"
    frame_health_every_s = max(1.0, float(os.environ.get("LIFETRAC_CAMERA_HEALTH_EVERY_S", "2")))
    _last_health_t = 0.0
    _last_canvas_sig: int | None = None
    _same_canvas_run = 0
    while True:
        force = force_key_evt.is_set()
        force_key_evt.clear()
        try:
            payload = _build_frame(cam, accum, force_keyframe=force,
                                   roi_planner=roi_planner,
                                   byte_budget=link_budget.bytes,
                                   encode_cache=encode_cache)
            if frame_health_log and accum.last_canvas is not None:
                sig = hash(accum.last_canvas[:4096])
                if _last_canvas_sig is not None and sig == _last_canvas_sig:
                    _same_canvas_run += 1
                else:
                    _same_canvas_run = 0
                now_h = time.monotonic()
                if (now_h - _last_health_t) >= frame_health_every_s:
                    LOG.info("camera_service: frame_health same_run=%d sig=%s fps=%.2f",
                             _same_canvas_run, sig, TARGET_FPS)
                    _last_health_t = now_h
                _last_canvas_sig = sig
            # Primary: UART to the M7 (length-framed). Skipped under the
            # LoRa-bridge path; image_tx_daemon picks up the same payload
            # over MQTT and feeds the L072 HostLink directly.
            if ipc is not None:
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
