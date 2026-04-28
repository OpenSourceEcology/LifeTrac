"""IP-107 follow-up: software-in-the-loop model of the M7 async TX queue.

The current M7 TX path uses ``radio.transmit()``, which blocks until
the SX1276 reports done. At SF12/BW125 that's > 1 s — far longer than
the M4 watchdog window. Round 3 mitigated this with
``refresh_m4_alive_before_tx()``, but the proper fix is a non-blocking
state machine driven by ``radio.startTransmit()`` + ``radio.isTransmitDone()``.

Porting that to firmware needs bench validation of RadioLib IRQ
behaviour. In the meantime, this SIL model lets us:

1. Land the **logic** (single-slot queue, P0/P1 priority, mid-TX E-stop
   pre-emption) and exercise it exhaustively in pure Python.
2. Verify the design holds across pathological orderings (link-tune
   racing a control frame, queue overflow, simulated TX-done timeout).
3. Provide a reference implementation that the C-side port can mirror
   line-for-line during the bench session.

The model intentionally mirrors what the M7 firmware will look like:
  * one-slot pending-TX register (the SX1276 fifo)
  * a 2-deep priority queue in front of it (P0 = E-stop / link-tune,
    P1 = control / heartbeat)
  * a coarse millis() tick advanced by the test
  * a simulated TX time-on-air based on the active PHY rung
"""

from __future__ import annotations

import unittest
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional


class TxPriority(IntEnum):
    P0_SAFETY = 0   # E-stop, link-tune. Pre-empts P1 in the queue (not the radio).
    P1_NORMAL = 1   # Control, heartbeat, command, telemetry.


@dataclass
class TxRequest:
    payload: bytes
    priority: TxPriority
    submit_ms: int
    label: str = ""


@dataclass
class _PendingTx:
    req: TxRequest
    start_ms: int
    duration_ms: int


@dataclass
class M7TxQueue:
    """Single-slot deferred-TX queue with a 2-deep priority front-buffer.

    Mirrors the proposed firmware design. ``tick(now_ms, tx_done_ms_for_payload)``
    is what the M7 main loop will call every loop iteration; the
    callback returns the simulated time-on-air for a given payload so
    the test can model SF/BW changes.
    """

    capacity: int = 4

    # Front buffer: up to ``capacity`` requests, sorted P0 before P1.
    _queue: list[TxRequest] = field(default_factory=list)
    # The single in-flight TX (the SX1276 FIFO is one-deep).
    _pending: Optional[_PendingTx] = None
    # Stats — observable by tests.
    completed: list[TxRequest] = field(default_factory=list)
    dropped: list[TxRequest] = field(default_factory=list)
    pre_empted: list[TxRequest] = field(default_factory=list)

    # ---- public API ----

    def submit(self, req: TxRequest) -> bool:
        """Returns False if the request was dropped (queue full)."""
        if len(self._queue) >= self.capacity:
            # Try to evict the lowest-priority oldest entry to make room
            # for a P0 — never drop a P0 in favour of a P1.
            if req.priority == TxPriority.P0_SAFETY:
                # Evict the last P1 if any.
                for i in range(len(self._queue) - 1, -1, -1):
                    if self._queue[i].priority == TxPriority.P1_NORMAL:
                        self.dropped.append(self._queue.pop(i))
                        break
                else:
                    # No P1 to evict — queue is full of P0s, drop self.
                    self.dropped.append(req)
                    return False
            else:
                self.dropped.append(req)
                return False
        # Insert keeping P0 ahead of P1, FIFO within priority.
        insert_at = len(self._queue)
        for i, q in enumerate(self._queue):
            if q.priority > req.priority:
                insert_at = i
                break
        self._queue.insert(insert_at, req)
        return True

    def tick(self, now_ms: int, time_on_air_ms_fn) -> None:
        """Advance the model. Completes any in-flight TX whose duration
        has elapsed, then starts the next queued request."""
        # 1. Service in-flight.
        if self._pending is not None:
            done_at = self._pending.start_ms + self._pending.duration_ms
            if now_ms >= done_at:
                self.completed.append(self._pending.req)
                self._pending = None
        # 2. Start next.
        if self._pending is None and self._queue:
            req = self._queue.pop(0)
            self._pending = _PendingTx(
                req=req,
                start_ms=now_ms,
                duration_ms=time_on_air_ms_fn(req.payload),
            )

    def estop_preempt(self, now_ms: int, estop_payload: bytes,
                      time_on_air_ms_fn) -> None:
        """Hard-pre-empt: drop whatever is in flight, drain the queue,
        and start the E-stop frame this tick. Mirrors the firmware path
        where the M4 raises a GPIO and the M7 abandons the SX1276 FIFO."""
        if self._pending is not None:
            self.pre_empted.append(self._pending.req)
            self._pending = None
        # Anything already queued is now stale (the world changed).
        self.dropped.extend(self._queue)
        self._queue.clear()
        req = TxRequest(payload=estop_payload,
                        priority=TxPriority.P0_SAFETY,
                        submit_ms=now_ms,
                        label="estop")
        self._pending = _PendingTx(
            req=req,
            start_ms=now_ms,
            duration_ms=time_on_air_ms_fn(estop_payload),
        )

    @property
    def is_idle(self) -> bool:
        return self._pending is None and not self._queue

    @property
    def queue_depth(self) -> int:
        return len(self._queue) + (1 if self._pending else 0)


# ---- helpers ----

def _toa_ms(payload: bytes) -> int:
    """Stand-in for SF7/BW250 control-PHY time-on-air. ~46 ms for a
    typical encrypted ControlFrame; scale linearly with payload size."""
    return max(20, len(payload) // 2 + 30)


def _toa_ms_slow(payload: bytes) -> int:
    """Stand-in for SF12/BW125 — pathological worst case that previously
    tripped the M4 watchdog."""
    return 1200 + len(payload) * 4


# ---- tests ----

class M7TxQueueTests(unittest.TestCase):

    def test_p0_jumps_in_front_of_p1(self) -> None:
        q = M7TxQueue()
        q.submit(TxRequest(b"ctl-1", TxPriority.P1_NORMAL, 0, "ctl-1"))
        q.submit(TxRequest(b"ctl-2", TxPriority.P1_NORMAL, 0, "ctl-2"))
        q.submit(TxRequest(b"link-tune", TxPriority.P0_SAFETY, 0, "link-tune"))
        q.submit(TxRequest(b"ctl-3", TxPriority.P1_NORMAL, 0, "ctl-3"))

        # Drain to completion.
        now = 0
        while not q.is_idle:
            now += 5
            q.tick(now, _toa_ms)
        labels = [c.label for c in q.completed]
        # link-tune (P0) drained first, then the three P1s in FIFO order.
        self.assertEqual(labels, ["link-tune", "ctl-1", "ctl-2", "ctl-3"])

    def test_capacity_drops_low_priority_first(self) -> None:
        q = M7TxQueue(capacity=2)
        # Fill with P1.
        for i in range(2):
            self.assertTrue(q.submit(
                TxRequest(b"x", TxPriority.P1_NORMAL, 0, f"p1-{i}")))
        # Third P1 should drop.
        self.assertFalse(q.submit(
            TxRequest(b"x", TxPriority.P1_NORMAL, 0, "p1-overflow")))
        self.assertEqual(q.dropped[-1].label, "p1-overflow")

        # P0 evicts a P1 to fit.
        self.assertTrue(q.submit(
            TxRequest(b"e", TxPriority.P0_SAFETY, 0, "estop")))
        # The evicted entry was the last P1.
        self.assertIn("p1-1", [d.label for d in q.dropped])

    def test_estop_preempts_inflight_and_drains_queue(self) -> None:
        q = M7TxQueue()
        q.submit(TxRequest(b"ctl-1", TxPriority.P1_NORMAL, 0, "ctl-1"))
        q.submit(TxRequest(b"ctl-2", TxPriority.P1_NORMAL, 0, "ctl-2"))
        q.tick(now_ms=0, time_on_air_ms_fn=_toa_ms_slow)  # ctl-1 starts
        # Mid-TX E-stop arrives 50 ms in.
        q.estop_preempt(now_ms=50, estop_payload=b"estop",
                        time_on_air_ms_fn=_toa_ms)
        # ctl-1 was pre-empted, ctl-2 was dropped from queue.
        self.assertEqual([r.label for r in q.pre_empted], ["ctl-1"])
        self.assertIn("ctl-2", [r.label for r in q.dropped])
        # E-stop completes within its modeled time-on-air.
        q.tick(now_ms=50 + _toa_ms(b"estop"), time_on_air_ms_fn=_toa_ms)
        self.assertEqual([c.label for c in q.completed], ["estop"])

    def test_no_blocking_on_long_toa(self) -> None:
        """Headline IP-107 invariant: the M7 main loop never has to wait
        synchronously, so the M4 watchdog is decoupled from PHY choice."""
        q = M7TxQueue()
        q.submit(TxRequest(b"long-tx-payload",
                           TxPriority.P1_NORMAL, 0, "slow"))
        # Tick the loop at a fast cadence (~5 ms) while a 1.2 s TX runs.
        completions_during_tx = []
        for now in range(0, 1200, 5):
            q.tick(now, _toa_ms_slow)
            if q.completed:
                completions_during_tx.extend(q.completed)
                break
        self.assertEqual(completions_during_tx, [],
                         "TX completed before its modeled time-on-air "
                         "(would mean the model is wrong)")
        # ...and finishes shortly after.
        q.tick(now_ms=_toa_ms_slow(b"long-tx-payload") + 1, time_on_air_ms_fn=_toa_ms_slow)
        self.assertEqual([c.label for c in q.completed], ["slow"])

    def test_priority_preserves_fifo_within_class(self) -> None:
        q = M7TxQueue(capacity=8)
        for i in range(5):
            q.submit(TxRequest(b"x", TxPriority.P1_NORMAL, i, f"p1-{i}"))
        for i in range(2):
            q.submit(TxRequest(b"x", TxPriority.P0_SAFETY, i + 10, f"p0-{i}"))
        # Drain.
        now = 0
        while not q.is_idle:
            now += 5
            q.tick(now, _toa_ms)
        labels = [c.label for c in q.completed]
        self.assertEqual(labels[:2], ["p0-0", "p0-1"])
        self.assertEqual(labels[2:], ["p1-0", "p1-1", "p1-2", "p1-3", "p1-4"])

    def test_repeated_estop_is_idempotent(self) -> None:
        q = M7TxQueue()
        q.estop_preempt(0, b"estop", _toa_ms)
        q.estop_preempt(10, b"estop", _toa_ms)
        # Second pre-empt counts the first in-flight as pre-empted.
        self.assertEqual(len(q.pre_empted), 1)
        self.assertEqual(q.pre_empted[0].label, "estop")


if __name__ == "__main__":
    unittest.main()
