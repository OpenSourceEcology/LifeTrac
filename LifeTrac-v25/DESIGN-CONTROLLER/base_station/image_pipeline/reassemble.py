"""TileDeltaFrame fragment reassembler.

Today ``camera_service.py`` publishes one complete TileDeltaFrame per MQTT
message because the bridge in front of it (`lora_bridge.py` over USB-CDC) is
local. Once the M7↔SX1276 SPI driver lands the frame will arrive in
≤25 ms-airtime fragments, with a small fragment header prepended:

    fragment header (4 bytes, big-endian):
        u8  magic           ; 0xFE
        u8  frag_seq        ; rolls 0..255 across the *whole canvas*
        u8  frag_idx        ; 0..(total-1)
        u8  total_minus1    ; total fragments == this+1, so 1..256

If the magic byte is anything other than 0xFE the payload is treated as a
single complete frame (the current local-bridge case).

Fragments time out after ``timeout_ms`` of inactivity; the partial set is
dropped and the caller is told to request a keyframe.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable

from .frame_format import (
    FrameDecodeError,
    TileDeltaFrame,
    parse_tile_delta_frame,
)

FRAGMENT_MAGIC = 0xFE
FRAGMENT_HEADER_LEN = 4
DEFAULT_TIMEOUT_MS = 1500


@dataclass
class _PartialFrame:
    total: int
    parts: dict[int, bytes] = field(default_factory=dict)
    first_seen_ms: int = 0
    last_seen_ms: int = 0


@dataclass
class ReassemblyStats:
    completed_frames: int = 0
    timeouts: int = 0
    duplicate_fragments: int = 0
    bad_magic_passthroughs: int = 0
    decode_errors: int = 0


class FragmentReassembler:
    """Collect TileDeltaFrame fragments keyed by ``frag_seq``.

    Pure-Python, no MQTT/threading. Drive it from the bridge / a unit test.
    """

    def __init__(
        self,
        timeout_ms: int = DEFAULT_TIMEOUT_MS,
        clock_ms: Callable[[], int] | None = None,
    ) -> None:
        self.timeout_ms = timeout_ms
        self._clock_ms = clock_ms or (lambda: int(time.monotonic() * 1000))
        self._partials: dict[int, _PartialFrame] = {}
        self.stats = ReassemblyStats()

    def feed(self, raw: bytes) -> TileDeltaFrame | None:
        """Process one MQTT/UART payload. Returns a full frame iff this
        payload completed (or *was*) one. Returns ``None`` while a frame is
        still in flight or on decode error."""
        if not raw:
            return None
        now = self._clock_ms()
        self._gc(now)
        if raw[0] != FRAGMENT_MAGIC:
            self.stats.bad_magic_passthroughs += 1
            return self._decode_or_record_error(raw)
        if len(raw) < FRAGMENT_HEADER_LEN:
            self.stats.decode_errors += 1
            return None
        _magic, frag_seq, frag_idx, total_minus1 = raw[0], raw[1], raw[2], raw[3]
        total = total_minus1 + 1
        if frag_idx >= total:
            self.stats.decode_errors += 1
            return None
        body = bytes(raw[FRAGMENT_HEADER_LEN:])
        partial = self._partials.get(frag_seq)
        if partial is None:
            partial = _PartialFrame(total=total, first_seen_ms=now, last_seen_ms=now)
            self._partials[frag_seq] = partial
        if partial.total != total:
            # Producer changed its mind mid-frame. Restart this slot.
            partial = _PartialFrame(total=total, first_seen_ms=now, last_seen_ms=now)
            self._partials[frag_seq] = partial
        if frag_idx in partial.parts:
            self.stats.duplicate_fragments += 1
            return None
        partial.parts[frag_idx] = body
        partial.last_seen_ms = now
        if len(partial.parts) == total:
            del self._partials[frag_seq]
            full = b"".join(partial.parts[i] for i in range(total))
            return self._decode_or_record_error(full)
        return None

    def pending_frag_seqs(self) -> list[int]:
        """Frag seqs that still have missing fragments. Used by `canvas.py`
        to decide whether to send `CMD_REQ_KEYFRAME`."""
        return list(self._partials.keys())

    def _gc(self, now_ms: int) -> None:
        if not self._partials:
            return
        cutoff = now_ms - self.timeout_ms
        stale = [seq for seq, p in self._partials.items() if p.last_seen_ms < cutoff]
        for seq in stale:
            del self._partials[seq]
            self.stats.timeouts += 1

    def _decode_or_record_error(self, full: bytes) -> TileDeltaFrame | None:
        try:
            frame = parse_tile_delta_frame(full)
        except FrameDecodeError:
            self.stats.decode_errors += 1
            return None
        self.stats.completed_frames += 1
        return frame
