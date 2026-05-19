"""S1.3 helper: "newest frame wins" counter for the W2-02 stability harness.

Motivation (see AI NOTES/2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md
§ "Patch the W2-02 stability gate to add a newest-frame-wins gate"):

The current W2-02 ``cmd_decode`` decodes the FIRST reassembled
``TileDeltaFrame`` and reports stats. In a real bench run the harness must
prefer the *newest* fully-reassembled frame and **discard** stale ones whose
``base_seq`` is older than one already accepted — otherwise a slow-completing
old frame painted over a fresh one would look fine in stats but show the
operator a stale image.

This counter exists as a pure-Python observability primitive: feed it every
completed-frame ``base_seq`` in arrival order, and read the counters back at
the end of the run. The W2-02 stability harness asserts on the
``stale_dropped`` and ``accepted`` totals — a healthy run with N keyframes
expected should produce ``accepted == N``, ``stale_dropped == 0`` once the
firmware-side coalescing fix from S3 lands. Today the counter primarily
catches harness bugs where the same ``base_seq`` is re-emitted (duplicate),
and surfaces stale-arrival cases that future S3 firmware changes must
eliminate.

The counter does **not** modify the underlying ``FragmentReassembler`` or
the frame stream; it observes ``base_seq`` after the fact. Run-to-run state
is not persisted — the harness creates a fresh counter per orchestrator run.

base_seq is the 16-bit camera-timestamp-derived monotonic field from
``frame_format.TileDeltaFrame``; wraparound is handled by tracking the last
accepted value and treating a new value as "newer" iff the unsigned 16-bit
forward distance is smaller than the backward distance (RFC-1982-style).
"""

from __future__ import annotations

from dataclasses import dataclass, field


_SEQ_MOD = 1 << 16  # base_seq is 16-bit per frame_format spec.
_SEQ_HALF = _SEQ_MOD >> 1


def _is_newer_seq(candidate: int, last_accepted: int) -> bool:
    """RFC-1982 serial-number-arithmetic newer-than for a 16-bit seq.

    Returns True iff ``candidate`` is strictly newer than ``last_accepted``
    under wraparound. A candidate equal to ``last_accepted`` returns False
    (treated as duplicate by ``NewestFrameWinsCounter``).
    """
    diff = (candidate - last_accepted) % _SEQ_MOD
    if diff == 0:
        return False
    return diff < _SEQ_HALF


@dataclass
class NewestFrameWinsStats:
    """Per-run counters. All start at zero.

    * ``accepted`` — frames the counter would forward to the operator canvas.
    * ``stale_dropped`` — frames that arrived after a strictly-newer
      ``base_seq`` was already accepted. The S3 gate (TODO) requires this to
      be 0 on a 10-minute mixed-load run.
    * ``duplicate_dropped`` — frames with the same ``base_seq`` as the last
      accepted. Indicates either firmware re-tx or harness double-feed; never
      forwarded.
    * ``first_frame_accepted`` — convenience flag; True after the first
      ``observe`` call regardless of outcome (so the harness can distinguish
      "no frames at all" from "all frames were stale").
    """

    accepted: int = 0
    stale_dropped: int = 0
    duplicate_dropped: int = 0
    first_frame_accepted: bool = False


@dataclass
class NewestFrameWinsCounter:
    """Observability counter — see module docstring.

    Usage:
        counter = NewestFrameWinsCounter()
        for frame in completed_frames_in_arrival_order:
            if counter.observe(frame.base_seq):
                paint(frame)        # newest so far
            else:
                pass                # stale or duplicate — drop on the floor
        run_summary["newest_frame_wins"] = counter.stats_dict()
    """

    _last_accepted_seq: int | None = None
    stats: NewestFrameWinsStats = field(default_factory=NewestFrameWinsStats)

    def observe(self, base_seq: int) -> bool:
        """Returns True iff this ``base_seq`` is the newest seen so far and
        should be painted. Updates stats in-place.
        """
        seq = int(base_seq) & (_SEQ_MOD - 1)
        if self._last_accepted_seq is None:
            self._last_accepted_seq = seq
            self.stats.accepted += 1
            self.stats.first_frame_accepted = True
            return True
        if seq == self._last_accepted_seq:
            self.stats.duplicate_dropped += 1
            return False
        if _is_newer_seq(seq, self._last_accepted_seq):
            self._last_accepted_seq = seq
            self.stats.accepted += 1
            return True
        self.stats.stale_dropped += 1
        return False

    def stats_dict(self) -> dict[str, int | bool | None]:
        """Render the counter as a JSON-serialisable dict for the run summary."""
        return {
            "accepted": self.stats.accepted,
            "stale_dropped": self.stats.stale_dropped,
            "duplicate_dropped": self.stats.duplicate_dropped,
            "first_frame_accepted": self.stats.first_frame_accepted,
            "last_accepted_seq": self._last_accepted_seq,
        }
