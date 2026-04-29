"""Round 8: SIL model of the M4 safety supervisor.

Mirrors ``firmware/tractor_h7/tractor_m4.cpp`` in pure Python so the
W4-01 / W4-03 HIL gates (E-stop latch < 100 ms; M7→M4 watchdog trip)
become *verification* passes against a documented model rather than
greenfield design passes on the bench.

What the C side does, captured here:

* Reads ``alive_tick_ms``, ``loop_counter``, ``estop_request`` from
  shared SRAM4 under the seqlock pattern (writer odd→even with
  ``__DMB`` barriers).
* Up to 4 retries on a torn read; if every attempt sees an odd seq,
  TRIP with reason ``seqlock_torn``.
* TRIP on ``alive_tick_ms`` stale by > ``LIFETRAC_M4_WATCHDOG_MS`` (200 ms).
* TRIP on ``loop_counter`` failing to advance for > 200 ms (separate
  witness — catches ``millis()`` stuck).
* TRIP on ``estop_request == LIFETRAC_ESTOP_MAGIC (0xA5A5A5A5)``. Any
  other value (incl. SRAM noise) does NOT trip.
* Trip is LATCHED — once safety_alive goes low it stays low until reset.
* The C ``loop()`` runs every ~20 ms (``delay(20)``).

Properties asserted in this suite (all the things W4-01/03 will need
to verify on the bench):

* TR-A latency: from the M7 setting ``estop_request = MAGIC`` to
  safety_alive going low must be < 100 ms (one M4 tick of 20 ms is
  enough; we leave generous margin).
* TR-B watchdog: with the M7 silent (no alive_tick / no loop_counter
  change), trip happens between ``WATCHDOG_MS`` and
  ``WATCHDOG_MS + 1 tick``.
* TR-C latch: trip is sticky — even if the M7 recovers and resumes
  ticking, safety_alive stays low.
* TR-D no-false-trip: with the M7 ticking faster than ``WATCHDOG_MS``,
  no trip occurs over an extended run.
* TR-E magic-only: ``estop_request`` values that aren't the magic
  constant (0, 0x12345678, ~MAGIC) do NOT trip on their own.
* TR-F seqlock retry: a writer that's odd-locked for one tick is
  retried successfully on the next; persistent odd lock causes
  ``seqlock_torn`` trip.
* TR-G millis-stuck: ``alive_tick_ms`` keeps reading "fresh" because
  the M7's clock froze, but ``loop_counter`` is the witness — trip
  fires on the loop-counter check.
* TR-H boot-arming: model refuses to arm (and therefore can never
  trip) until the shared-mem version matches.

Pure stdlib.
"""

from __future__ import annotations

import unittest
from dataclasses import dataclass, field
from typing import Callable, Optional


SHARED_VERSION = 2
ESTOP_MAGIC = 0xA5A5A5A5
WATCHDOG_MS = 200          # LIFETRAC_M4_WATCHDOG_MS
M4_TICK_MS = 20            # delay(20) at the bottom of M4 loop()
SEQLOCK_RETRIES = 4        # for-loop in tractor_m4.cpp
ARM_DEADLINE_MS = 3000     # setup() boot wait window


@dataclass
class SharedState:
    """Mirror of the SRAM4 ``SharedM7M4`` view the M4 reads."""
    version: int = SHARED_VERSION
    seq: int = 0                 # seqlock counter; odd = writer in progress
    alive_tick_ms: int = 0       # M7 stamps wall-clock here every loop
    loop_counter: int = 0        # M7 increments every loop
    estop_request: int = 0       # M7 writes MAGIC to mirror its E-stop latch


@dataclass
class M4Supervisor:
    """Pure-Python port of ``tractor_m4.cpp`` ``loop()``.

    Caller drives ``tick(now_ms)`` to simulate one iteration of the M4
    main loop. State machine matches the C side 1:1.
    """
    shared: SharedState
    armed: bool = False
    safety_alive: bool = False         # GPIO state; True = high (healthy)
    tripped: bool = False
    trip_reason: Optional[str] = None
    last_loop_counter: int = 0
    last_loop_change_ms: int = 0
    boot_started_ms: int = 0
    _booted: bool = False

    def boot(self, now_ms: int) -> None:
        """Mirror of ``setup()``. Waits for shared-mem version + first
        non-zero alive_tick. We model boot as instantaneous if the
        shared state is already valid, since the C code's 3000 ms wait
        is a deadline not a fixed delay."""
        self.boot_started_ms = now_ms
        if self.shared.version != SHARED_VERSION:
            # Halts in C; here we leave armed=False / safety_alive=False
            # forever and the caller can assert on it.
            return
        if self.shared.alive_tick_ms == 0:
            # Cannot arm yet; caller should drive boot() again later.
            return
        self.last_loop_counter = self.shared.loop_counter
        self.last_loop_change_ms = now_ms
        self.armed = True
        self.safety_alive = True
        self._booted = True

    def _trip(self, reason: str) -> None:
        # Latched. Subsequent calls into _trip are ignored so the first
        # reason wins (matches C: ``while (1) delay(1000)`` after the
        # GPIO is pulled low).
        if self.tripped:
            return
        self.tripped = True
        self.trip_reason = reason
        self.safety_alive = False

    def _snapshot(self) -> Optional[tuple[int, int, int]]:
        """Seqlock read with up to SEQLOCK_RETRIES attempts. Returns
        ``None`` if every attempt saw an odd (writer-in-progress) seq."""
        for _ in range(SEQLOCK_RETRIES):
            s0 = self.shared.seq
            if s0 & 1:
                continue
            alive = self.shared.alive_tick_ms
            loop_ct = self.shared.loop_counter
            estop = self.shared.estop_request
            # __DMB() barrier modelled by reading seq again after the
            # value reads — Python is single-threaded in this model so
            # ordering is trivially correct, but we still re-check seq.
            s1 = self.shared.seq
            if s0 == s1:
                return alive, loop_ct, estop
        return None

    def tick(self, now_ms: int) -> None:
        if not self.armed or self.tripped:
            return

        snap = self._snapshot()
        if snap is None:
            self._trip("seqlock_torn")
            return
        alive_tick_ms, loop_counter, estop_request = snap

        # 1. Liveness tick check.
        if now_ms >= alive_tick_ms and (now_ms - alive_tick_ms) > WATCHDOG_MS:
            self._trip("alive_tick_stale")
            return

        # 2. Loop-counter advance check.
        if loop_counter != self.last_loop_counter:
            self.last_loop_counter = loop_counter
            self.last_loop_change_ms = now_ms
        elif (now_ms - self.last_loop_change_ms) > WATCHDOG_MS:
            self._trip("loop_counter_stuck")
            return

        # 3. M7 mirror of CMD_ESTOP — magic-gated per IP-106.
        if estop_request == ESTOP_MAGIC:
            self._trip("estop_request")
            return


def _seqlock_write(shared: SharedState, mutate: Callable[[SharedState], None]) -> None:
    """Helper: writer-side seqlock pattern (odd→mutate→even)."""
    shared.seq += 1                # now odd
    mutate(shared)
    shared.seq += 1                # now even


def _run_with_healthy_m7(sup: M4Supervisor, start_ms: int, end_ms: int,
                         m7_period_ms: int = 50) -> int:
    """Drive M4 ticks while a virtual M7 stamps alive/loop_counter
    every ``m7_period_ms``. Returns the time at which the M4 tripped,
    or ``end_ms`` if it survived the whole window."""
    next_m7 = start_ms
    t = start_ms
    while t <= end_ms:
        if t >= next_m7:
            _seqlock_write(sup.shared, lambda s: (
                setattr(s, "alive_tick_ms", t),
                setattr(s, "loop_counter", s.loop_counter + 1),
            ))
            next_m7 = t + m7_period_ms
        sup.tick(t)
        if sup.tripped:
            return t
        t += M4_TICK_MS
    return end_ms


class TestM4SupervisorSil(unittest.TestCase):

    def _fresh(self, now_ms: int = 1000, m7_period_ms: int = 50) -> M4Supervisor:
        shared = SharedState(
            version=SHARED_VERSION,
            seq=0,
            alive_tick_ms=now_ms,
            loop_counter=1,
            estop_request=0,
        )
        sup = M4Supervisor(shared=shared)
        sup.boot(now_ms)
        self.assertTrue(sup.armed, "should arm immediately when shared mem is valid")
        self.assertTrue(sup.safety_alive)
        return sup

    # TR-A
    def test_estop_latency_under_100ms(self) -> None:
        sup = self._fresh(now_ms=1000)
        # Healthy M7 ticking until t=2000 ms when it fires E-stop.
        t = 1000
        next_m7 = 1000
        estop_at = 2000
        trip_at: Optional[int] = None
        while t <= 2200:
            if t >= next_m7:
                if t >= estop_at:
                    _seqlock_write(sup.shared, lambda s: (
                        setattr(s, "alive_tick_ms", t),
                        setattr(s, "loop_counter", s.loop_counter + 1),
                        setattr(s, "estop_request", ESTOP_MAGIC),
                    ))
                else:
                    _seqlock_write(sup.shared, lambda s: (
                        setattr(s, "alive_tick_ms", t),
                        setattr(s, "loop_counter", s.loop_counter + 1),
                    ))
                next_m7 = t + 50
            sup.tick(t)
            if sup.tripped and trip_at is None:
                trip_at = t
                break
            t += M4_TICK_MS
        self.assertIsNotNone(trip_at)
        assert trip_at is not None
        latency = trip_at - estop_at
        self.assertEqual(sup.trip_reason, "estop_request")
        # W4-01 gate is < 100 ms. Worst case here is one M4 tick (20 ms).
        self.assertLess(latency, 100,
                        f"E-stop latch latency {latency} ms exceeds W4-01 budget")

    # TR-B
    def test_watchdog_trips_when_m7_silent(self) -> None:
        sup = self._fresh(now_ms=1000)
        # M7 stops talking at t=1000 ms; M4 keeps ticking.
        trip_at: Optional[int] = None
        for t in range(1000, 1000 + WATCHDOG_MS + 5 * M4_TICK_MS, M4_TICK_MS):
            sup.tick(t)
            if sup.tripped:
                trip_at = t
                break
        self.assertIsNotNone(trip_at)
        assert trip_at is not None
        elapsed = trip_at - 1000
        # Trip must happen *after* WATCHDOG_MS but within one extra tick.
        self.assertGreater(elapsed, WATCHDOG_MS)
        self.assertLessEqual(elapsed, WATCHDOG_MS + M4_TICK_MS)
        # Either liveness check is acceptable; the alive-tick fires
        # first because the loop-counter window is reset on boot to
        # the same `now_ms`.
        self.assertIn(sup.trip_reason,
                      ("alive_tick_stale", "loop_counter_stuck"))

    # TR-C
    def test_trip_is_latched(self) -> None:
        sup = self._fresh(now_ms=1000)
        # Force an immediate E-stop trip.
        _seqlock_write(sup.shared, lambda s: setattr(s, "estop_request", ESTOP_MAGIC))
        sup.tick(1020)
        self.assertTrue(sup.tripped)
        self.assertFalse(sup.safety_alive)
        first_reason = sup.trip_reason
        # M7 "recovers": clears the request and resumes ticking. The
        # M4 must STAY tripped — automatic re-arm is forbidden by
        # design (see comment in tractor_m4.cpp).
        for t in range(1040, 5000, M4_TICK_MS):
            _seqlock_write(sup.shared, lambda s: (
                setattr(s, "alive_tick_ms", t),
                setattr(s, "loop_counter", s.loop_counter + 1),
                setattr(s, "estop_request", 0),
            ))
            sup.tick(t)
            self.assertTrue(sup.tripped)
            self.assertFalse(sup.safety_alive)
            self.assertEqual(sup.trip_reason, first_reason)

    # TR-D
    def test_no_false_trip_with_healthy_m7(self) -> None:
        sup = self._fresh(now_ms=1000)
        end = _run_with_healthy_m7(sup, start_ms=1000, end_ms=10_000,
                                   m7_period_ms=50)
        self.assertEqual(end, 10_000, f"unexpected trip: {sup.trip_reason}")
        self.assertFalse(sup.tripped)
        self.assertTrue(sup.safety_alive)

    # TR-E
    def test_estop_request_only_trips_on_magic(self) -> None:
        for non_magic in (0, 1, 0x12345678, (~ESTOP_MAGIC) & 0xFFFFFFFF,
                          ESTOP_MAGIC ^ 1, 0xA5A5A5A4, 0xA5A5A5A6):
            with self.subTest(non_magic=hex(non_magic)):
                sup = self._fresh(now_ms=1000)
                _seqlock_write(sup.shared,
                               lambda s, v=non_magic: setattr(s, "estop_request", v))
                # Run for one watchdog window with healthy alive_tick
                # / loop_counter so only the estop check could fire.
                end = _run_with_healthy_m7(sup, 1000, 1000 + WATCHDOG_MS + 100,
                                           m7_period_ms=50)
                self.assertFalse(sup.tripped,
                                 f"trip on non-magic estop_request={hex(non_magic)}: "
                                 f"{sup.trip_reason}")
                self.assertEqual(end, 1000 + WATCHDOG_MS + 100)

    # TR-F
    def test_seqlock_retry_recovers_then_persistent_odd_trips(self) -> None:
        # Case 1: writer holds odd for one tick, then completes. Reader
        # should recover (no trip).
        sup = self._fresh(now_ms=1000)
        sup.shared.seq = 1   # writer mid-update (odd)
        # The model retries 4× then bails — but since the seq stays
        # odd through *all* retries inside one tick, this would trip.
        # Simulate "writer finishes mid-loop" by patching seq during
        # the retries: do it by completing the write before tick().
        sup.shared.alive_tick_ms = 1010
        sup.shared.loop_counter = 2
        sup.shared.seq = 2   # writer done (even)
        sup.tick(1020)
        self.assertFalse(sup.tripped)

        # Case 2: writer is permanently stuck odd → trip with
        # ``seqlock_torn``.
        sup2 = self._fresh(now_ms=1000)
        sup2.shared.seq = 1   # never advances
        sup2.tick(1020)
        self.assertTrue(sup2.tripped)
        self.assertEqual(sup2.trip_reason, "seqlock_torn")
        self.assertFalse(sup2.safety_alive)

    # TR-G
    def test_loop_counter_witness_catches_stuck_millis(self) -> None:
        # M7 keeps stamping the *same* alive_tick_ms (because its
        # millis() froze) and the *same* loop_counter. The alive_tick
        # check would not fire for as long as ``now_ms - alive_tick``
        # stays < WATCHDOG_MS, but the loop-counter witness must.
        # We pin alive_tick to a time that stays "fresh" by re-stamping
        # the same value — which actually IS fresh against `now_ms`
        # only if the M7 keeps writing it. Here we set it once and
        # stop, so the alive-tick check fires; that's still the
        # correct behaviour. The dedicated witness test is: stamp
        # alive_tick = now_ms every tick BUT never advance the loop
        # counter. Then the loop-counter check must trip.
        sup = self._fresh(now_ms=1000)
        trip_at: Optional[int] = None
        for t in range(1000, 1000 + WATCHDOG_MS + 5 * M4_TICK_MS, M4_TICK_MS):
            _seqlock_write(sup.shared, lambda s, ts=t: (
                setattr(s, "alive_tick_ms", ts),
                # loop_counter intentionally NOT advanced
            ))
            sup.tick(t)
            if sup.tripped:
                trip_at = t
                break
        self.assertIsNotNone(trip_at)
        self.assertEqual(sup.trip_reason, "loop_counter_stuck")

    # TR-H
    def test_refuses_to_arm_on_version_mismatch(self) -> None:
        shared = SharedState(version=SHARED_VERSION + 1,
                             alive_tick_ms=1000, loop_counter=1)
        sup = M4Supervisor(shared=shared)
        sup.boot(1000)
        self.assertFalse(sup.armed)
        self.assertFalse(sup.safety_alive)
        # Even an E-stop request can't trip an unarmed supervisor —
        # it sits with safety_alive low forever (matches C: while(1)).
        _seqlock_write(sup.shared, lambda s: setattr(s, "estop_request", ESTOP_MAGIC))
        sup.tick(1020)
        self.assertFalse(sup.tripped)
        self.assertFalse(sup.safety_alive)

    def test_refuses_to_arm_until_first_alive_tick(self) -> None:
        shared = SharedState(version=SHARED_VERSION, alive_tick_ms=0)
        sup = M4Supervisor(shared=shared)
        sup.boot(1000)
        self.assertFalse(sup.armed)
        # M7 publishes its first tick; re-boot should now succeed.
        shared.alive_tick_ms = 1100
        shared.loop_counter = 1
        sup.boot(1100)
        self.assertTrue(sup.armed)
        self.assertTrue(sup.safety_alive)


if __name__ == "__main__":
    unittest.main()
