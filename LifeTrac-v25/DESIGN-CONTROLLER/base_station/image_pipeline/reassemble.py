"""TileDeltaFrame fragment reassembler.

Today ``camera_service.py`` publishes one complete TileDeltaFrame per MQTT
message because the bridge in front of it (`lora_bridge.py` over USB-CDC) is
local. Once the M7↔SX1276 SPI driver lands the frame will arrive in
≤25 ms-airtime fragments, with a small fragment header prepended.

Two wire-format versions are supported:

    v1 fragment header (4 bytes):
        u8  magic = 0xFE
        u8  frag_seq        ; rolls 0..255 across the *whole canvas*
        u8  frag_idx        ; 0..(total-1)
        u8  total_minus1    ; total fragments == this+1, so 1..256

    v2 fragment header (5 bytes; W2-02 P0c redundancy 2026-05-20):
        u8  magic = 0xFD
        u8  frag_seq
        u8  frag_idx
        u8  total_minus1
        u8  redundancy = (total_copies << 4) | copy_idx
                         ; total_copies in 1..15, copy_idx in 0..15
                         ; copy_idx must be < total_copies

Both versions share dedup semantics: the *first* copy of any
(frag_seq, frag_idx) wins; subsequent copies (whether duplicate v1 lines or
extra v2 redundancy copies) are dropped at reassembly. v2 additionally
records which copies arrived, so the W2-02 redundancy probe can ask
"did all copies make it" vs "were we saved by the redundant copy".

If the magic byte is neither 0xFE nor 0xFD the payload is treated as a
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

# v1 (legacy) and v2 (W2-02 P0c redundancy header).
FRAGMENT_MAGIC = 0xFE
FRAGMENT_HEADER_LEN = 4
FRAGMENT_MAGIC_V2 = 0xFD
FRAGMENT_HEADER_LEN_V2 = 5
DEFAULT_TIMEOUT_MS = 1500


@dataclass
class _PartialFrame:
    total: int
    parts: dict[int, bytes] = field(default_factory=dict)
    first_seen_ms: int = 0
    last_seen_ms: int = 0
    # v2 only: per-idx bitmask of which copy_idx values arrived.
    # Bit `k` set => copy_idx=k has been seen for that frag_idx.
    copy_bitmasks: dict[int, int] = field(default_factory=dict)


@dataclass
class ReassemblyStats:
    completed_frames: int = 0
    timeouts: int = 0
    duplicate_fragments: int = 0
    bad_magic_passthroughs: int = 0
    decode_errors: int = 0
    # v2-only counters; harmless for pure-v1 traffic.
    v2_fragments_seen: int = 0
    v2_redundant_copies_seen: int = 0   # copies beyond the first that filled the slot
    v2_bad_header: int = 0              # copy_idx >= total_copies, or total_copies == 0


class FragmentReassembler:
    """Collect TileDeltaFrame fragments keyed by ``frag_seq``.

    Pure-Python, no MQTT/threading. Drive it from the bridge / a unit test.
    """

    # Bounded LRU cap for recently-completed frag_seqs. v2 redundancy
    # copies that arrive after the frame already completed must be
    # counted as v2_redundant_copies_seen (not as a new frame). 64 is
    # comfortably > the worst-case fragments-in-flight on the bench.
    _COMPLETED_LRU_CAP = 64

    def __init__(
        self,
        timeout_ms: int = DEFAULT_TIMEOUT_MS,
        clock_ms: Callable[[], int] | None = None,
    ) -> None:
        self.timeout_ms = timeout_ms
        self._clock_ms = clock_ms or (lambda: int(time.monotonic() * 1000))
        self._partials: dict[int, _PartialFrame] = {}
        # Insertion-ordered set of frag_seqs that have completed
        # recently; capped at _COMPLETED_LRU_CAP. Used to swallow late v2
        # redundancy copies so they don't accidentally re-open a slot
        # and (worse) double-complete the same frame.
        self._completed_recent: dict[int, None] = {}
        self.stats = ReassemblyStats()

    def _mark_completed(self, frag_seq: int) -> None:
        self._completed_recent.pop(frag_seq, None)
        self._completed_recent[frag_seq] = None
        while len(self._completed_recent) > self._COMPLETED_LRU_CAP:
            self._completed_recent.pop(next(iter(self._completed_recent)))

    def feed(self, raw: bytes) -> TileDeltaFrame | None:
        """Process one MQTT/UART payload. Returns a full frame iff this
        payload completed (or *was*) one. Returns ``None`` while a frame is
        still in flight or on decode error."""
        if not raw:
            return None
        now = self._clock_ms()
        self._gc(now)
        magic = raw[0]
        if magic == FRAGMENT_MAGIC:
            header_len = FRAGMENT_HEADER_LEN
            is_v2 = False
        elif magic == FRAGMENT_MAGIC_V2:
            header_len = FRAGMENT_HEADER_LEN_V2
            is_v2 = True
        else:
            self.stats.bad_magic_passthroughs += 1
            return self._decode_or_record_error(raw)
        if len(raw) < header_len:
            self.stats.decode_errors += 1
            return None
        frag_seq = raw[1]
        frag_idx = raw[2]
        total_minus1 = raw[3]
        total = total_minus1 + 1
        if frag_idx >= total:
            self.stats.decode_errors += 1
            return None
        if is_v2:
            self.stats.v2_fragments_seen += 1
            red_byte = raw[4]
            total_copies = (red_byte >> 4) & 0x0F
            copy_idx = red_byte & 0x0F
            if total_copies == 0 or copy_idx >= total_copies:
                self.stats.v2_bad_header += 1
                self.stats.decode_errors += 1
                return None
        else:
            total_copies = 1
            copy_idx = 0
        # Late-arriving copy of a v2 frame we already completed: count as
        # redundant and drop. (For v1 we fall through to the partials
        # path which will recreate a slot -- harmless because v1 has no
        # mid-stream redundancy.)
        if is_v2 and frag_seq in self._completed_recent:
            self.stats.v2_redundant_copies_seen += 1
            return None
        body = bytes(raw[header_len:])
        partial = self._partials.get(frag_seq)
        if partial is None:
            partial = _PartialFrame(total=total, first_seen_ms=now, last_seen_ms=now)
            self._partials[frag_seq] = partial
        if partial.total != total:
            # Producer changed its mind mid-frame. Restart this slot.
            partial = _PartialFrame(total=total, first_seen_ms=now, last_seen_ms=now)
            self._partials[frag_seq] = partial
        # v2 per-idx copy bookkeeping (no-op for v1 traffic).
        if is_v2:
            mask = partial.copy_bitmasks.get(frag_idx, 0)
            bit = 1 << copy_idx
            if mask & bit:
                # Same (frag_idx, copy_idx) seen twice -- straight duplicate
                # on the air (e.g. RX re-delivered the same payload).
                self.stats.duplicate_fragments += 1
                return None
            partial.copy_bitmasks[frag_idx] = mask | bit
            if frag_idx in partial.parts:
                # A different copy of this fragment already filled the slot;
                # this redundancy copy is what saves us from a lost-copy
                # scenario but is not needed because we already had data.
                self.stats.v2_redundant_copies_seen += 1
                partial.last_seen_ms = now
                return None
        else:
            if frag_idx in partial.parts:
                self.stats.duplicate_fragments += 1
                return None
        partial.parts[frag_idx] = body
        partial.last_seen_ms = now
        if len(partial.parts) == total:
            del self._partials[frag_seq]
            self._mark_completed(frag_seq)
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
