"""Round 10: SIL model of the Opta Modbus slave + M7 Modbus client.

Mirrors ``firmware/tractor_opta/opta_modbus_slave.ino`` and the
``apply_control()`` writer in ``firmware/tractor_h7/tractor_h7.ino`` so
the W4-04 HIL gate ("Modbus failure → E-stop within 1 s") becomes a
*verification* pass against a documented model, in line with the
Round-8 ``test_m4_safety_sil.py`` pattern.

What the C side does, captured here:

M7 side (``apply_control`` in ``tractor_h7.ino``):

* Builds a 7-register holding block on every call:
  - REG_VALVE_COILS, REG_FLOW_SP_1, REG_FLOW_SP_2, REG_AUX_OUTPUTS,
    REG_WATCHDOG_CTR (incremented every call), REG_CMD_SOURCE,
    REG_ARM_ESTOP.
* When ``g_estop_latched`` is true, writes the all-zero block with
  REG_ARM_ESTOP=1 and REG_CMD_SOURCE=0 (fail-closed).
* Calls ``ModbusRTUClient.endTransmission()``; on failure increments a
  static ``s_modbus_fail_count``. When the count hits 10, latches
  ``g_estop_latched`` to true. **The counter is *consecutive*** — a
  successful TX resets it to 0, matching the W4-04 runbook prose. This
  is the post-Round-14 behaviour; prior to Round 14 the firmware
  counted cumulatively (see TR-I, which now pins the corrected
  semantics).

Opta side (``opta_modbus_slave.ino``):

* ``on_holding_change`` is gated by ``g_safety_state``: while the
  state is anything other than SAFETY_NORMAL, only writes to
  REG_WATCHDOG_CTR are honoured (so the master can keep proving
  liveness). All other writes are silently dropped.
* REG_VALVE_COILS → ``apply_valve_coils`` (8 relay channels).
* REG_FLOW_SP_{1,2} → A0602 0..10000 mV, clamped at 10000.
* REG_AUX_OUTPUTS → masked to ``AUX_OUTPUTS_VALID_MASK = 0x0007`` so
  bits outside R10/R11/R12 cannot energise valve channels.
* REG_WATCHDOG_CTR write timestamps ``g_last_alive_change_ms``.
* REG_ARM_ESTOP != 0 → engine-kill HIGH and enter SAFETY_ESTOP_LATCHED.
* ``check_safety()`` runs every loop iteration:
  - REG_WATCHDOG_CTR not changing for > 200 ms → enter
    SAFETY_WATCHDOG_TRIPPED, drop PSR, all coils off, AOs to 0.
  - PIN_ESTOP_LOOP LOW → enter SAFETY_ESTOP_LATCHED.

Properties asserted in this suite (the W4-04 verification surface):

* TR-A latency: with the M7 calling ``apply_control`` at 100 Hz, the
  Opta watchdog must NOT trip across a 5 s run.
* TR-B silence trip: if the M7 stops writing entirely, the Opta
  watchdog trips between 200 ms and 200 ms + one tick.
* TR-C bus-fail latch: after 10 consecutive ``endTransmission()``
  failures the M7 latches ``g_estop_latched`` and starts writing the
  fail-closed block.
* TR-D fail-closed shape: once latched, the block written has
  REG_VALVE_COILS=0, REG_FLOW_SP_*=0, REG_ARM_ESTOP=1.
* TR-E Opta latches on REG_ARM_ESTOP: a single write of REG_ARM_ESTOP=1
  enters SAFETY_ESTOP_LATCHED and refuses subsequent motion writes.
* TR-F refuse-motion-while-latched: while safety_state != NORMAL, a
  REG_VALVE_COILS write is silently dropped (does NOT energise relays).
* TR-G external E-stop loop: PIN_ESTOP_LOOP LOW enters
  SAFETY_ESTOP_LATCHED and drops all coils.
* TR-H aux-mask: REG_AUX_OUTPUTS=0xFFFF energises only R10/R11/R12
  (the valid-mask bits) and never the boom/bucket SSR channels.
* TR-I (post-Round-14 fix): the M7 fail-count is now *consecutive*.
  A successful poll between two failures resets the counter, so a
  single failure followed
  by 9 successes followed by 9 more failures still latches at the 10th
  total failure does not eventually latch. The W4-04 runbook prose
  says "10 *consecutive*
  failures", so this test is marked ``expectedFailure`` to flag the
  discrepancy until the firmware is updated to reset on success.
* TR-J end-to-end timing: from the first ``endTransmission()`` failure
  to ``g_estop_latched`` must be < 1 s when ``apply_control`` is called
  at the documented ≥ 10 Hz rate.

Pure stdlib.
"""

from __future__ import annotations

import unittest
from dataclasses import dataclass, field
from typing import Callable, Dict, Optional


# ---- constants mirroring the C side ----
WATCHDOG_MS = 200            # Opta-side check_safety() threshold
M7_PERIOD_MS = 10            # apply_control() target rate ≥ 10 Hz → 100 Hz here
OPTA_TICK_MS = 5             # Opta loop() iteration cadence (fast enough to
                             # see a 200 ms watchdog edge cleanly)
MODBUS_FAIL_LATCH = 10       # apply_control() s_modbus_fail_count threshold
AUX_OUTPUTS_VALID_MASK = 0x0007

# Holding-register addresses (must match the firmware headers).
REG_VALVE_COILS = 0x0000
REG_FLOW_SP_1 = 0x0001
REG_FLOW_SP_2 = 0x0002
REG_AUX_OUTPUTS = 0x0003
REG_WATCHDOG_CTR = 0x0004
REG_CMD_SOURCE = 0x0005
REG_ARM_ESTOP = 0x0006
HOLDING_BLOCK_LEN = 7

# Safety-state enum values (must match SafetyState in opta_modbus_slave.ino).
SAFETY_NORMAL = 0
SAFETY_WATCHDOG_TRIPPED = 1
SAFETY_ESTOP_LATCHED = 2
SAFETY_IGNITION_OFF = 3

# Aux relay bit positions inside REG_AUX_OUTPUTS.
AUX_R10_BIT = 0
AUX_R11_BIT = 1
AUX_R12_BIT = 2

# D1608S channels for the boom/bucket bank (must NOT be touched by AUX writes).
D1608S_BOOM_BUCKET_CHANNELS = (0, 1, 2, 3)
# D1608S channels for the spare aux bank (R10..R12).
D1608S_AUX_CHANNELS = (4, 5, 6)


# ---------------------------------------------------------------------------
# Opta slave model
# ---------------------------------------------------------------------------


@dataclass
class OptaSlave:
    """Pure-Python model of ``opta_modbus_slave.ino`` safety surface.

    Only the bits W4-04 cares about are modelled — analog telemetry,
    boot self-test, and the OptaController expansion-bus quirks are out
    of scope (they are unrelated to the Modbus-failure → E-stop path).
    """

    # Wall-clock millis() the model thinks it's running under. Driven by
    # the test harness via ``advance(ms)`` so we don't wait on real time.
    now_ms: int = 0

    # Safety state machine.
    safety_state: int = SAFETY_NORMAL

    # Watchdog tracking.
    last_alive_change_ms: int = 0
    last_alive_value: int = 0
    alive_seen: bool = False

    # External E-stop loop input. HIGH (True) = healthy, LOW (False) = trip.
    estop_loop_healthy: bool = True

    # Output mirrors so tests can introspect.
    psr_alive: bool = True              # PIN_PSR_ALIVE
    engine_kill: bool = False           # PIN_ENGINE_KILL
    relays: Dict[str, bool] = field(default_factory=lambda: {
        "R1_drive_lf": False,
        "R2_drive_lr": False,
        "R3_drive_rf": False,
        "R4_drive_rr": False,
        "R5_boom_up": False,
        "R6_boom_dn": False,
        "R7_bucket_curl": False,
        "R8_bucket_dump": False,
        "R10_aux": False,
        "R11_aux": False,
        "R12_aux": False,
    })
    flow_sp_mv: Dict[int, int] = field(default_factory=lambda: {0: 0, 1: 0})

    # ------------------------------------------------------------------
    # Helpers mirroring the C side.
    # ------------------------------------------------------------------

    def _all_coils_off(self) -> None:
        for k in self.relays:
            self.relays[k] = False
        self.flow_sp_mv[0] = 0
        self.flow_sp_mv[1] = 0

    def _enter_safe_state(self, why: int) -> None:
        # Mirror enter_safe_state(): set state, drop coils, drop PSR.
        # The C code only enters once (callers gate on state == NORMAL),
        # so we preserve that — a second call is a no-op for ``why``.
        self.safety_state = why
        self._all_coils_off()
        self.psr_alive = False

    def _apply_valve_coils(self, coils: int) -> None:
        self.relays["R1_drive_lf"] = bool(coils & (1 << 0))
        self.relays["R2_drive_lr"] = bool(coils & (1 << 1))
        self.relays["R3_drive_rf"] = bool(coils & (1 << 2))
        self.relays["R4_drive_rr"] = bool(coils & (1 << 3))
        self.relays["R5_boom_up"] = bool(coils & (1 << 4))
        self.relays["R6_boom_dn"] = bool(coils & (1 << 5))
        self.relays["R7_bucket_curl"] = bool(coils & (1 << 6))
        self.relays["R8_bucket_dump"] = bool(coils & (1 << 7))

    # ------------------------------------------------------------------
    # Public API used by the tests.
    # ------------------------------------------------------------------

    def on_holding_change(self, address: int, value: int) -> None:
        """Mirrors ``on_holding_change`` in ``opta_modbus_slave.ino``."""
        # Refuse motion writes while latched; only watchdog writes pass.
        if self.safety_state != SAFETY_NORMAL and address != REG_WATCHDOG_CTR:
            return

        if address == REG_VALVE_COILS:
            self._apply_valve_coils(value)
        elif address == REG_FLOW_SP_1:
            self.flow_sp_mv[0] = min(value, 10000)
        elif address == REG_FLOW_SP_2:
            self.flow_sp_mv[1] = min(value, 10000)
        elif address == REG_AUX_OUTPUTS:
            aux = value & AUX_OUTPUTS_VALID_MASK
            self.relays["R10_aux"] = bool(aux & (1 << AUX_R10_BIT))
            self.relays["R11_aux"] = bool(aux & (1 << AUX_R11_BIT))
            self.relays["R12_aux"] = bool(aux & (1 << AUX_R12_BIT))
        elif address == REG_WATCHDOG_CTR:
            self.last_alive_value = value
            self.last_alive_change_ms = self.now_ms
            self.alive_seen = True
        elif address == REG_CMD_SOURCE:
            pass   # logging only
        elif address == REG_ARM_ESTOP:
            if value != 0:
                self.engine_kill = True
                if self.safety_state == SAFETY_NORMAL:
                    self._enter_safe_state(SAFETY_ESTOP_LATCHED)
            else:
                self.engine_kill = False

    def write_block(self, regs: Dict[int, int]) -> None:
        """Mirrors a Modbus write-multiple-registers call across the
        7-register holding block. Order matters because the C
        ``on_holding_change`` callback fires in address order."""
        for addr in sorted(regs):
            self.on_holding_change(addr, regs[addr])

    def check_safety(self) -> None:
        """Mirrors ``check_safety()`` — called from Opta loop()."""
        if (self.alive_seen
                and (self.now_ms - self.last_alive_change_ms) > WATCHDOG_MS
                and self.safety_state == SAFETY_NORMAL):
            self._enter_safe_state(SAFETY_WATCHDOG_TRIPPED)

        if not self.estop_loop_healthy and self.safety_state == SAFETY_NORMAL:
            self._enter_safe_state(SAFETY_ESTOP_LATCHED)

    def tick(self, ms: int = OPTA_TICK_MS) -> None:
        """Advance the Opta clock and run one safety check."""
        self.now_ms += ms
        self.check_safety()


# ---------------------------------------------------------------------------
# M7 client model (apply_control side)
# ---------------------------------------------------------------------------


@dataclass
class M7ModbusClient:
    """Pure-Python model of ``apply_control()`` Modbus framing + the
    fail-count latch added in IP-205."""

    # Wall-clock the M7 thinks it's running under (independent of Opta's
    # clock, but tests usually advance them in lockstep).
    now_ms: int = 0

    # Mirrors of the C statics.
    watchdog_ctr: int = 0           # g_watchdog_ctr
    estop_latched: bool = False     # g_estop_latched
    fail_count: int = 0             # s_modbus_fail_count

    # Bus health, controlled by the test harness. Each call to
    # ``apply_control`` looks up this attribute to decide whether
    # endTransmission() succeeds.
    bus_healthy: bool = True

    # Last block written (kept for assertions on fail-closed shape).
    last_block: Dict[int, int] = field(default_factory=dict)

    # ------------------------------------------------------------------

    def _build_block(self, axis_active: bool) -> Dict[int, int]:
        """Build the 7-register holding block. ``axis_active`` is a
        coarse stand-in for "operator commanded motion" — when False
        the block is all zeros (matches the no-fresh-source branch).
        """
        self.watchdog_ctr += 1
        regs = {
            REG_VALVE_COILS: 0,
            REG_FLOW_SP_1: 0,
            REG_FLOW_SP_2: 0,
            REG_AUX_OUTPUTS: 0,
            REG_WATCHDOG_CTR: self.watchdog_ctr & 0xFFFF,
            REG_CMD_SOURCE: 0,
            REG_ARM_ESTOP: 0,
        }
        if self.estop_latched:
            regs[REG_ARM_ESTOP] = 1
            regs[REG_CMD_SOURCE] = 0
        elif axis_active:
            # Drive both tracks forward at full deflection.
            regs[REG_VALVE_COILS] = (1 << 0) | (1 << 2)   # LF+RF
            regs[REG_FLOW_SP_1] = 10000
            regs[REG_FLOW_SP_2] = 10000
            regs[REG_CMD_SOURCE] = 1                      # HANDHELD
        return regs

    def apply_control(self, opta: OptaSlave, axis_active: bool = False) -> bool:
        """One iteration of the M7's apply_control(). Returns True if the
        Modbus TX succeeded, False otherwise."""
        regs = self._build_block(axis_active)
        self.last_block = dict(regs)

        if self.bus_healthy:
            opta.write_block(regs)
            # Round 14: firmware now resets s_modbus_fail_count on every
            # successful poll, so the 10-strike latch counts *consecutive*
            # failures (matching the W4-04 runbook prose). TR-I pins this.
            self.fail_count = 0
            return True

        # endTransmission() failure path.
        self.fail_count += 1
        if self.fail_count >= MODBUS_FAIL_LATCH:
            self.estop_latched = True
        return False

    def tick(self, ms: int = M7_PERIOD_MS) -> None:
        self.now_ms += ms


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class OptaWatchdogTests(unittest.TestCase):
    """TR-A / TR-B — Opta-side 200 ms watchdog on REG_WATCHDOG_CTR."""

    def test_no_trip_when_master_healthy(self):
        """TR-A: 100 Hz writes from a healthy M7 must not trip the
        watchdog over a 5 s run."""
        opta = OptaSlave()
        m7 = M7ModbusClient()
        for _ in range(500):       # 500 * 10 ms = 5 s
            m7.apply_control(opta)
            m7.tick()
            opta.now_ms = m7.now_ms
            opta.check_safety()
        self.assertEqual(opta.safety_state, SAFETY_NORMAL)
        self.assertTrue(opta.psr_alive)

    def test_watchdog_trips_when_master_silent(self):
        """TR-B: with the M7 silent, watchdog must trip between 200 ms
        and 200 ms + one Opta tick."""
        opta = OptaSlave()
        m7 = M7ModbusClient()
        # First, prime the watchdog with one healthy write.
        m7.apply_control(opta)
        # Now stop calling apply_control; just tick the Opta clock.
        opta.tick(50)              # t=50 ms — still fresh
        self.assertEqual(opta.safety_state, SAFETY_NORMAL)
        opta.tick(150)             # t=200 ms — boundary, NOT yet > 200
        self.assertEqual(opta.safety_state, SAFETY_NORMAL)
        opta.tick(OPTA_TICK_MS)    # t=205 ms — strictly past 200
        self.assertEqual(opta.safety_state, SAFETY_WATCHDOG_TRIPPED)
        self.assertFalse(opta.psr_alive)

    def test_watchdog_trip_drops_all_coils(self):
        """TR-B follow-on: watchdog trip must drop relays and zero AOs."""
        opta = OptaSlave()
        m7 = M7ModbusClient()
        # Drive motion so coils are non-zero before the silence.
        m7.apply_control(opta, axis_active=True)
        self.assertTrue(opta.relays["R1_drive_lf"])
        self.assertEqual(opta.flow_sp_mv[0], 10000)
        # Go silent past the watchdog boundary.
        for _ in range(60):
            opta.tick()
        self.assertEqual(opta.safety_state, SAFETY_WATCHDOG_TRIPPED)
        self.assertFalse(any(opta.relays.values()))
        self.assertEqual(opta.flow_sp_mv, {0: 0, 1: 0})


class M7FailLatchTests(unittest.TestCase):
    """TR-C / TR-D / TR-J — M7 ``apply_control`` fail-count latch."""

    def test_fail_count_latches_at_threshold(self):
        """TR-C: 10 consecutive failures latch ``g_estop_latched``."""
        opta = OptaSlave()
        m7 = M7ModbusClient(bus_healthy=False)
        for _ in range(MODBUS_FAIL_LATCH - 1):
            m7.apply_control(opta)
            self.assertFalse(m7.estop_latched)
        m7.apply_control(opta)        # 10th failure
        self.assertTrue(m7.estop_latched)

    def test_fail_closed_block_shape(self):
        """TR-D: once latched, the block written has zero coils, zero
        flow set-points, and REG_ARM_ESTOP=1."""
        opta = OptaSlave()
        m7 = M7ModbusClient(bus_healthy=False)
        for _ in range(MODBUS_FAIL_LATCH):
            m7.apply_control(opta, axis_active=True)
        # Now restore the bus and call once more so the latched block
        # actually gets shipped to the (mocked) Opta.
        m7.bus_healthy = True
        m7.apply_control(opta, axis_active=True)
        self.assertEqual(m7.last_block[REG_VALVE_COILS], 0)
        self.assertEqual(m7.last_block[REG_FLOW_SP_1], 0)
        self.assertEqual(m7.last_block[REG_FLOW_SP_2], 0)
        self.assertEqual(m7.last_block[REG_ARM_ESTOP], 1)
        self.assertEqual(m7.last_block[REG_CMD_SOURCE], 0)

    def test_w4_04_end_to_end_within_one_second(self):
        """TR-J: from the first endTransmission() failure to
        ``g_estop_latched`` must be < 1 s at the 100 Hz call rate."""
        opta = OptaSlave()
        m7 = M7ModbusClient()
        # Healthy for 1 s so the baseline is established.
        for _ in range(100):
            m7.apply_control(opta)
            m7.tick()
            opta.now_ms = m7.now_ms
            opta.check_safety()
        # Yank the bus.
        m7.bus_healthy = False
        t_first_fail_ms = m7.now_ms
        while not m7.estop_latched and m7.now_ms - t_first_fail_ms < 2000:
            m7.apply_control(opta)
            m7.tick()
            opta.now_ms = m7.now_ms
            opta.check_safety()
        self.assertTrue(m7.estop_latched)
        elapsed = m7.now_ms - t_first_fail_ms
        self.assertLess(elapsed, 1000,
                        f"W4-04 budget violated: latch took {elapsed} ms")


class OptaArmEstopTests(unittest.TestCase):
    """TR-E / TR-F — Opta latches on REG_ARM_ESTOP and refuses motion
    afterwards."""

    def test_arm_estop_latches_safe_state(self):
        opta = OptaSlave()
        opta.on_holding_change(REG_ARM_ESTOP, 1)
        self.assertEqual(opta.safety_state, SAFETY_ESTOP_LATCHED)
        self.assertTrue(opta.engine_kill)
        self.assertFalse(opta.psr_alive)

    def test_motion_writes_dropped_after_latch(self):
        opta = OptaSlave()
        opta.on_holding_change(REG_ARM_ESTOP, 1)
        # Now try to energise drive coils + open flow valve.
        opta.on_holding_change(REG_VALVE_COILS, 0xFF)
        opta.on_holding_change(REG_FLOW_SP_1, 8000)
        opta.on_holding_change(REG_FLOW_SP_2, 8000)
        self.assertFalse(any(opta.relays.values()))
        self.assertEqual(opta.flow_sp_mv, {0: 0, 1: 0})

    def test_watchdog_writes_pass_through_after_latch(self):
        """Master must keep proving liveness even while latched, so
        REG_WATCHDOG_CTR writes are explicitly NOT dropped."""
        opta = OptaSlave()
        opta.on_holding_change(REG_ARM_ESTOP, 1)
        opta.now_ms = 1000
        opta.on_holding_change(REG_WATCHDOG_CTR, 42)
        self.assertEqual(opta.last_alive_value, 42)
        self.assertEqual(opta.last_alive_change_ms, 1000)


class OptaExternalEstopTests(unittest.TestCase):
    """TR-G — PIN_ESTOP_LOOP LOW enters SAFETY_ESTOP_LATCHED."""

    def test_external_loop_open_latches(self):
        opta = OptaSlave()
        m7 = M7ModbusClient()
        m7.apply_control(opta, axis_active=True)
        self.assertTrue(opta.relays["R1_drive_lf"])
        opta.estop_loop_healthy = False
        opta.tick()
        self.assertEqual(opta.safety_state, SAFETY_ESTOP_LATCHED)
        self.assertFalse(any(opta.relays.values()))
        self.assertFalse(opta.psr_alive)


class OptaAuxMaskTests(unittest.TestCase):
    """TR-H — REG_AUX_OUTPUTS only honours bits inside the valid mask."""

    def test_full_aux_word_only_lights_documented_bits(self):
        opta = OptaSlave()
        # Energise a boom valve via the proper register first so we can
        # detect any spurious flip from the aux write.
        opta.on_holding_change(REG_VALVE_COILS, 1 << 4)   # boom up
        self.assertTrue(opta.relays["R5_boom_up"])
        # All-ones aux word: only R10/R11/R12 must respond.
        opta.on_holding_change(REG_AUX_OUTPUTS, 0xFFFF)
        self.assertTrue(opta.relays["R10_aux"])
        self.assertTrue(opta.relays["R11_aux"])
        self.assertTrue(opta.relays["R12_aux"])
        # Boom/bucket bank untouched by the aux write.
        self.assertTrue(opta.relays["R5_boom_up"])
        self.assertFalse(opta.relays["R6_boom_dn"])
        self.assertFalse(opta.relays["R7_bucket_curl"])
        self.assertFalse(opta.relays["R8_bucket_dump"])

    def test_aux_clears_on_zero(self):
        opta = OptaSlave()
        opta.on_holding_change(REG_AUX_OUTPUTS, 0x0007)
        self.assertTrue(opta.relays["R10_aux"])
        opta.on_holding_change(REG_AUX_OUTPUTS, 0x0000)
        self.assertFalse(opta.relays["R10_aux"])
        self.assertFalse(opta.relays["R11_aux"])
        self.assertFalse(opta.relays["R12_aux"])


class M7FailCountSemanticsTests(unittest.TestCase):
    """TR-I — Pin the *consecutive* fail-count semantics that match
    the W4-04 runbook prose. The firmware was changed in Round 14 to
    reset ``s_modbus_fail_count`` on every successful poll; this test
    catches any regression that re-introduces the cumulative variant."""

    def test_fail_count_is_consecutive(self):
        """A successful poll between two failures must reset the
        counter, so 9 failures + 1 success + 1 failure does NOT latch."""
        opta = OptaSlave()
        m7 = M7ModbusClient()
        # 9 failures: under threshold, no latch.
        m7.bus_healthy = False
        for _ in range(MODBUS_FAIL_LATCH - 1):
            m7.apply_control(opta)
        self.assertEqual(m7.fail_count, MODBUS_FAIL_LATCH - 1)
        self.assertFalse(m7.estop_latched)
        # One success must clear the strike count.
        m7.bus_healthy = True
        m7.apply_control(opta)
        self.assertEqual(m7.fail_count, 0,
                         "successful poll must reset s_modbus_fail_count")
        # One more failure: count back to 1, nowhere near the latch.
        m7.bus_healthy = False
        m7.apply_control(opta)
        self.assertEqual(m7.fail_count, 1)
        self.assertFalse(m7.estop_latched,
                        "single intermittent failure must not latch "
                        "E-stop — if it does, firmware regressed to "
                        "the pre-Round-14 cumulative count.")

    def test_intermittent_failures_never_latch(self):
        """Pathological alternating fail/success pattern over a long
        run must never latch. This is the property that motivates the
        consecutive count: a noisy bus shouldn't be a slow-burn nuisance
        trip."""
        opta = OptaSlave()
        m7 = M7ModbusClient()
        # 1000 cycles alternating fail / success — cumulative would
        # have tripped at cycle 19 (10th failure). Consecutive must
        # never trip.
        for i in range(1000):
            m7.bus_healthy = (i % 2 == 1)
            m7.apply_control(opta)
        self.assertFalse(m7.estop_latched,
                         "alternating fail/success must never latch "
                         "under consecutive semantics")
        self.assertLessEqual(m7.fail_count, 1)

    def test_nine_then_one_then_nine_does_not_latch(self):
        """9 failures, 1 success, 9 failures: cumulative would latch
        on the 19th call (10th total failure); consecutive must not."""
        opta = OptaSlave()
        m7 = M7ModbusClient()
        m7.bus_healthy = False
        for _ in range(9):
            m7.apply_control(opta)
        m7.bus_healthy = True
        m7.apply_control(opta)         # reset
        m7.bus_healthy = False
        for _ in range(9):
            m7.apply_control(opta)
        self.assertEqual(m7.fail_count, 9)
        self.assertFalse(m7.estop_latched)


if __name__ == "__main__":           # pragma: no cover
    unittest.main()
