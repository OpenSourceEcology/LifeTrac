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


def _make_camera():
    if SOURCE == "synthetic":
        return SyntheticCamera()
    try:
        return LibcameraCamera()
    except Exception as exc:                                  # pragma: no cover
        LOG.warning("camera_service: libcamera unavailable (%s) — "
                    "falling back to synthetic", exc)
        return SyntheticCamera()


# ---- tile encode ------------------------------------------------------

def _encode_tile(rgb_canvas: bytes, tx: int, ty: int) -> bytes:
    """Encode tile (tx,ty) as a WebP blob. Caller is responsible for
    keeping the blob ≤ TILE_BYTES_MAX; we degrade quality if needed."""
    from PIL import Image  # type: ignore
    # Slice the row-major RGB buffer into a 32×32 tile.
    row_stride = CANVAS_W * 3
    out = bytearray(TILE_PX * TILE_PX * 3)
    for ry in range(TILE_PX):
        src_off = ((ty * TILE_PX) + ry) * row_stride + (tx * TILE_PX) * 3
        dst_off = ry * TILE_PX * 3
        out[dst_off:dst_off + TILE_PX * 3] = \
            rgb_canvas[src_off:src_off + TILE_PX * 3]
    img = Image.frombytes("RGB", (TILE_PX, TILE_PX), bytes(out))
    quality = WEBP_QUALITY
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


def _build_frame(cam, accum: FrameAccum, force_keyframe: bool) -> bytes:
    """Capture, diff, encode, return the wire payload (header + tiles).
    Wire format mirrors LORA_PROTOCOL.md § TileDeltaFrame so the M7 can
    forward it byte-for-byte after fragmentation."""
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

    # Encode tiles whose bit is set, row-major.
    blobs: list[bytes] = []
    for ty in range(GRID_H):
        for tx in range(GRID_W):
            i = ty * GRID_W + tx
            if bitmap[i // 8] & (1 << (i % 8)):
                blobs.append(_encode_tile(canvas, tx, ty))

    # Header per LORA_PROTOCOL.md § TileDeltaFrame:
    #   frame_kind (1) | seq (1) | grid_w (1) | grid_h (1) | tile_px (1)
    #   | changed_bitmap (bitmap_bytes)
    accum.seq = (accum.seq + 1) & 0xFF
    header = struct.pack("BBBBB",
                         1 if is_key else 0,
                         accum.seq, GRID_W, GRID_H, TILE_PX) + bytes(bitmap)
    body = bytearray()
    for blob in blobs:
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
X8_KISS_FEND, X8_KISS_FESC = 0xC0, 0xDB
X8_KISS_TFEND, X8_KISS_TFESC = 0xDC, 0xDD


def dispatch_back_channel(frame: bytes, force_key_evt) -> None:
    """Pure dispatcher for one decoded back-channel frame.

    ``force_key_evt`` is a ``threading.Event`` (the encode loop reads
    ``.is_set()`` / clears it). Returning early on malformed input keeps
    the reader thread tolerant of a noisy UART.
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
        mode = frame[2] if len(frame) >= 3 else 0
        LOG.info("camera_service: CMD_ENCODE_MODE %d (no-op)", mode)
    # Force a keyframe on any reconfiguration so the new settings are
    # immediately observable downstream.
    if opcode in (CMD_CAMERA_SELECT, CMD_CAMERA_QUALITY, CMD_ENCODE_MODE):
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
                        dispatch_back_channel(bytes(buf), force_key_evt)
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
            payload = _build_frame(cam, accum, force_keyframe=force)
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
