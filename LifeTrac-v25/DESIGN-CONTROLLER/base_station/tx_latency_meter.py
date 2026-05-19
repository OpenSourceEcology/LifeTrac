"""S1.2 host-side TX latency instrumentation (pure Python, HW-independent).

Records three per-priority-class signals as the host orchestrator pushes a
frame through the priority queue → serial writer:

* **queue_age_ms** — wall-clock time the frame waited between ``mark_enqueue``
  and ``mark_dequeue``. This is the P0 TX-start-delay surrogate that the
  TODO S1.2 entry calls out: the longer P0 sits behind any other class, the
  worse the gate score.
* **tx_done_ms** — full enqueue→serial-write-complete latency. Stands in for
  the firmware ``TX_DONE`` event until the L072 reattaches and we can swap in
  a real on-air timestamp; the host serial write is the only thing we control
  before the radio.
* **toa_delta_ms** — ``actual_tx_done_ms - predicted_toa_ms``. Per LORA_IMPLEMENTATION.md
  §4 + TODO §S1 gate, the delta between measured encrypted ToA and the PHY
  airtime prediction must stay under 10 ms on at least one P0 and one P3 class.

All measurements are *opt-in*: a frame that never calls ``mark_enqueue`` costs
zero. The bridge wraps every meter call in a try/except so instrumentation
failures can never break TX. Data collection past the host boundary (real
on-air ``TX_DONE``) is HW-blocked until the L072 firmware reattaches and the
``TX_DONE_URC`` (host type 0x90) is plumbed back to ``Bridge``.

Tokens are bare ints (monotonic counter); no per-frame allocation beyond the
rolling deque entry.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from threading import Lock
from typing import Deque


# Default rolling window per class. Sized for ~10 minutes of P3 traffic at
# 1 Hz (which is the slowest interesting class on the W2-02 bench).
_DEFAULT_WINDOW = 600


def _percentile(samples: list[float], pct: float) -> float | None:
    """Nearest-rank percentile over an unsorted list. Returns None on empty.

    Tiny and self-contained so this module has no numpy dependency. Matches
    the convention used elsewhere in the suite (see test_e2e_image_pipeline
    p50/p99 computation).
    """
    if not samples:
        return None
    ordered = sorted(samples)
    if pct <= 0:
        return ordered[0]
    if pct >= 100:
        return ordered[-1]
    # Nearest-rank: ceil(pct/100 * N) − 1, clamped.
    idx = max(0, min(len(ordered) - 1, int((pct / 100.0) * len(ordered)) - 1))
    return ordered[idx]


@dataclass
class _InflightFrame:
    """Bookkeeping for a frame between ``mark_enqueue`` and ``mark_done``."""

    prio: int
    enqueue_ms: int
    dequeue_ms: int | None = None
    predicted_toa_ms: float | None = None


@dataclass
class _ClassSamples:
    """Rolling per-class sample windows. Bounded so memory cannot grow."""

    queue_age: Deque[float] = field(default_factory=lambda: deque(maxlen=_DEFAULT_WINDOW))
    tx_done: Deque[float] = field(default_factory=lambda: deque(maxlen=_DEFAULT_WINDOW))
    toa_delta: Deque[float] = field(default_factory=lambda: deque(maxlen=_DEFAULT_WINDOW))


class TxLatencyMeter:
    """Per-priority-class p50/p99 meter for the host TX path.

    Thread-safe: the bridge has one TX writer plus a separate caller thread
    that does the enqueue, so all mutators take ``_lock``. Reads (``snapshot``)
    take the same lock so a snapshot is a coherent view rather than torn.

    Usage:
        meter = TxLatencyMeter()
        token = meter.mark_enqueue(prio=0, now_ms=t0)
        # ... heapq.heappush + cv notify ...
        meter.mark_dequeue(token, now_ms=t1)
        # ... encrypt + ser.write ...
        meter.mark_done(token, now_ms=t2, predicted_toa_ms=phy_predicted)

    A token that never reaches ``mark_done`` is forgotten on the next
    ``mark_done`` for the same priority once the in-flight cap is reached
    (so a crash mid-write does not leak unbounded memory).
    """

    # Hard cap on outstanding tokens, in case a caller forgets mark_done.
    # The bridge has at most a handful in flight (single writer thread) so
    # this is generous and only exists as a leak guard.
    _MAX_INFLIGHT = 64

    def __init__(self, window: int = _DEFAULT_WINDOW) -> None:
        self._lock = Lock()
        self._next_token = 0
        self._inflight: dict[int, _InflightFrame] = {}
        self._classes: dict[int, _ClassSamples] = {}
        # Per-class deque size override (mainly for unit tests).
        self._window = window

    # ---- ingestion ------------------------------------------------------

    def mark_enqueue(self, prio: int, now_ms: int) -> int:
        """Record that a frame entered the priority queue. Returns an opaque
        token the caller must pass to ``mark_dequeue`` / ``mark_done``.
        """
        with self._lock:
            token = self._next_token
            self._next_token += 1
            self._inflight[token] = _InflightFrame(prio=prio, enqueue_ms=now_ms)
            if len(self._inflight) > self._MAX_INFLIGHT:
                # Drop the oldest token wholesale. This is a defensive guard
                # only — the bridge has a single writer so realistic in-flight
                # counts stay under 10.
                oldest = min(self._inflight)
                self._inflight.pop(oldest, None)
            return token

    def mark_dequeue(self, token: int, now_ms: int) -> None:
        """Record that the TX worker popped this frame off the heap."""
        with self._lock:
            frame = self._inflight.get(token)
            if frame is None:
                return
            frame.dequeue_ms = now_ms
            bucket = self._bucket_locked(frame.prio)
            bucket.queue_age.append(float(now_ms - frame.enqueue_ms))

    def mark_done(
        self,
        token: int,
        now_ms: int,
        predicted_toa_ms: float | None = None,
    ) -> None:
        """Record TX-write-complete. ``predicted_toa_ms`` is the PHY airtime
        prediction (LORA_IMPLEMENTATION.md §4) — when provided, we also log
        the actual−predicted delta per S1 gate.
        """
        with self._lock:
            frame = self._inflight.pop(token, None)
            if frame is None:
                return
            bucket = self._bucket_locked(frame.prio)
            tx_done = float(now_ms - frame.enqueue_ms)
            bucket.tx_done.append(tx_done)
            if predicted_toa_ms is not None and frame.dequeue_ms is not None:
                actual_air = float(now_ms - frame.dequeue_ms)
                bucket.toa_delta.append(actual_air - float(predicted_toa_ms))

    # ---- export ---------------------------------------------------------

    def snapshot(self) -> dict[int, dict[str, float | int | None]]:
        """Return per-class p50/p99 over the rolling window.

        Keys are priority ints (0..3). Values are dicts with:
          ``tx_done_p50_ms``, ``tx_done_p99_ms``,
          ``queue_age_p50_ms``, ``queue_age_p99_ms``,
          ``toa_delta_p50_ms``, ``toa_delta_p99_ms``,
          ``sample_count`` (tx_done count — the canonical "completed frames"
          metric per class).

        Classes with zero samples are omitted so the snapshot stays small.
        """
        with self._lock:
            out: dict[int, dict[str, float | int | None]] = {}
            for prio, bucket in self._classes.items():
                if not bucket.tx_done and not bucket.queue_age:
                    continue
                out[prio] = {
                    "tx_done_p50_ms": _percentile(list(bucket.tx_done), 50),
                    "tx_done_p99_ms": _percentile(list(bucket.tx_done), 99),
                    "queue_age_p50_ms": _percentile(list(bucket.queue_age), 50),
                    "queue_age_p99_ms": _percentile(list(bucket.queue_age), 99),
                    "toa_delta_p50_ms": _percentile(list(bucket.toa_delta), 50),
                    "toa_delta_p99_ms": _percentile(list(bucket.toa_delta), 99),
                    "sample_count": len(bucket.tx_done),
                }
            return out

    def reset(self) -> None:
        """Clear all rolling windows and in-flight tokens. Intended for tests
        and for harness-driven re-baselining between bench runs.
        """
        with self._lock:
            self._next_token = 0
            self._inflight.clear()
            self._classes.clear()

    # ---- internals ------------------------------------------------------

    def _bucket_locked(self, prio: int) -> _ClassSamples:
        bucket = self._classes.get(prio)
        if bucket is None:
            bucket = _ClassSamples(
                queue_age=deque(maxlen=self._window),
                tx_done=deque(maxlen=self._window),
                toa_delta=deque(maxlen=self._window),
            )
            self._classes[prio] = bucket
        return bucket
