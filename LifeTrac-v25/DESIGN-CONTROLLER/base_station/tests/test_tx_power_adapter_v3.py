"""SIL tests for :mod:`tx_power_adapter_v3` — S6.1 + S6.3 + S6.4.

These tests synthesise (snr, ok, airtime) traces and walk the adapter
through every transition. No LoRa hardware, no I/O.

Invariants pinned (failures here = design regression, not test brittleness):

* I1  NORMAL → MARGIN_LIMITED on sustained low-SNR streak.
* I2  NORMAL → AIRTIME_LIMITED on sustained high airtime util.
* I3  S6.4 mutual exclusivity: MARGIN_LIMITED never transitions
      directly to AIRTIME_LIMITED (or vice versa). RECOVERY or NORMAL
      always sits between them in the event log.
* I4  Cascade ordering (margin-limited): RAISE_POWER fires before
      SF_STEP_UP — and SF_STEP_UP only after power saturates at cap.
* I5  Cascade ordering (airtime-limited): CANCEL_P3 fires before
      CANCEL_P2 before ENCODE_DEGRADE.
* I6  10 Hz inner cap: two RAISE_POWER actions can't fire within the
      same 100 ms window even if both ticks request one.
* I7  Hysteresis: a single bad SNR sample does NOT trigger
      MARGIN_LIMITED. INNER_HYSTERESIS_PACKETS consecutive bad samples
      are required.
* I8  RECOVERY dwell: after conditions clear, the adapter holds in
      RECOVERY for DWELL_RECOVERY_TO_NORMAL_MS before returning to
      NORMAL.
* I9  RECOVERY re-entry: if conditions re-degrade during RECOVERY, the
      adapter jumps directly back to the matching LIMITED state.
"""
from __future__ import annotations

import sys
from pathlib import Path

# Allow `import tx_power_adapter_v3` when pytest is invoked from repo root.
_BASE_DIR = Path(__file__).resolve().parent.parent
if str(_BASE_DIR) not in sys.path:
    sys.path.insert(0, str(_BASE_DIR))

import pytest

from tx_power_adapter_v3 import (  # noqa: E402
    AdapterAction,
    AdapterState,
    AIRTIME_FLOOR,
    DWELL_RECOVERY_TO_NORMAL_MS,
    INNER_HYSTERESIS_PACKETS,
    INNER_MIN_INTERVAL_MS,
    MARGIN_CLEAR_SNR_DB,
    MARGIN_FLOOR_SNR_DB,
    P_MAX_DBM,
    P_MIN_DBM,
    TxPowerAdapterV3,
)


# ---------- helpers --------------------------------------------------

def _flood_snr(adapter: TxPowerAdapterV3, now_ms: int, snr_db: float,
               n: int = INNER_HYSTERESIS_PACKETS, ok: bool = True) -> int:
    """Feed `n` SNR samples 1 ms apart so the EWMA + streak counter advance."""
    for i in range(n):
        adapter.observe_packet(now_ms + i, snr_db, ok)
    return now_ms + n


# ---------- I1 / I7 --------------------------------------------------

def test_normal_to_margin_limited_requires_hysteresis():
    """I1 + I7: NORMAL transitions to MARGIN_LIMITED only after the
    hysteresis streak is reached."""
    adapter = TxPowerAdapterV3()
    adapter.observe_airtime(total=0.1)
    now = 0

    # One bad sample alone must NOT trip the state.
    adapter.observe_packet(now, MARGIN_FLOOR_SNR_DB - 5.0, ok=True)
    dec = adapter.tick(now)
    assert dec.state == AdapterState.NORMAL, (
        "single bad SNR sample should not flip state")

    # Feed the remaining samples to clear the hysteresis gate.
    now = _flood_snr(adapter, now + 1, MARGIN_FLOOR_SNR_DB - 5.0,
                     n=INNER_HYSTERESIS_PACKETS - 1)
    dec = adapter.tick(now)
    assert dec.state == AdapterState.MARGIN_LIMITED


def test_normal_to_airtime_limited_on_high_util():
    """I2."""
    adapter = TxPowerAdapterV3()
    # Healthy SNR so margin branch can't win.
    _flood_snr(adapter, 0, +10.0, n=INNER_HYSTERESIS_PACKETS)
    adapter.observe_airtime(total=AIRTIME_FLOOR + 0.05, image=0.4)
    dec = adapter.tick(100)
    assert dec.state == AdapterState.AIRTIME_LIMITED


# ---------- I3 -------------------------------------------------------

def test_no_direct_margin_to_airtime_transition():
    """I3 (S6.4 mutual exclusivity): swing from MARGIN_LIMITED through
    a clean window into AIRTIME_LIMITED — adapter must pass through
    RECOVERY first (NEVER a direct margin→airtime jump)."""
    adapter = TxPowerAdapterV3()
    adapter.observe_airtime(total=0.1)
    now = _flood_snr(adapter, 0, MARGIN_FLOOR_SNR_DB - 5.0)
    adapter.tick(now)
    assert adapter.state == AdapterState.MARGIN_LIMITED

    # Now: clean SNR + high airtime, all in one tick. Use a strong
    # positive value so the EWMA (which is currently sitting near the
    # floor) crosses the clear threshold within the hysteresis window.
    now = _flood_snr(adapter, now + 1, MARGIN_CLEAR_SNR_DB + 30.0,
                     n=INNER_HYSTERESIS_PACKETS * 3)
    adapter.observe_airtime(total=AIRTIME_FLOOR + 0.05, image=0.4)
    dec = adapter.tick(now)
    # MUST be RECOVERY, NOT AIRTIME_LIMITED. The adapter is required to
    # log a state change so observers can attribute root cause.
    assert dec.state == AdapterState.RECOVERY, (
        f"forbidden direct MARGIN→AIRTIME transition: got {dec.state}")


def test_no_direct_airtime_to_margin_transition():
    """I3 reverse direction."""
    adapter = TxPowerAdapterV3()
    _flood_snr(adapter, 0, +10.0)
    adapter.observe_airtime(total=AIRTIME_FLOOR + 0.05, image=0.4)
    adapter.tick(100)
    assert adapter.state == AdapterState.AIRTIME_LIMITED

    now = _flood_snr(adapter, 200, MARGIN_FLOOR_SNR_DB - 5.0)
    adapter.observe_airtime(total=0.1)
    dec = adapter.tick(now)
    assert dec.state == AdapterState.RECOVERY, (
        f"forbidden direct AIRTIME→MARGIN transition: got {dec.state}")


# ---------- I4 -------------------------------------------------------

def test_margin_cascade_raises_power_before_sf_step():
    """I4: while in MARGIN_LIMITED, the adapter emits RAISE_POWER on
    each tick (rate-limited) and only SF_STEP_UP once power is at cap."""
    adapter = TxPowerAdapterV3(initial_power_dbm=P_MAX_DBM - 1)
    adapter.observe_airtime(total=0.1)
    now = _flood_snr(adapter, 0, MARGIN_FLOOR_SNR_DB - 5.0)
    # Enter MARGIN_LIMITED.
    dec = adapter.tick(now)
    assert dec.state == AdapterState.MARGIN_LIMITED

    # First tick after entry — must RAISE_POWER (cheapest), not SF.
    now += INNER_MIN_INTERVAL_MS
    dec = adapter.tick(now)
    assert dec.action == AdapterAction.RAISE_POWER, dec
    assert dec.value == P_MAX_DBM

    # Power now pinned at cap → next eligible tick must SF_STEP_UP.
    now += INNER_MIN_INTERVAL_MS
    dec = adapter.tick(now)
    assert dec.action == AdapterAction.SF_STEP_UP, dec
    assert dec.value == 1


# ---------- I5 -------------------------------------------------------

def test_airtime_cascade_cancels_p3_before_p2_before_encode_degrade():
    """I5: cheapest-first sacrifice order."""
    adapter = TxPowerAdapterV3()
    _flood_snr(adapter, 0, +10.0)

    # P3 (image) is the biggest contributor → must be cancelled first.
    adapter.observe_airtime(total=AIRTIME_FLOOR + 0.05,
                            image=0.4, telemetry=0.2)
    adapter.tick(100)
    assert adapter.state == AdapterState.AIRTIME_LIMITED
    dec = adapter.tick(200)
    assert dec.action == AdapterAction.CANCEL_P3

    # Pretend the caller dropped image → now telemetry is the only
    # remaining knob short of encode-degrade.
    adapter.observe_airtime(total=AIRTIME_FLOOR + 0.05,
                            image=0.0, telemetry=0.4)
    dec = adapter.tick(300)
    assert dec.action == AdapterAction.CANCEL_P2

    # Pretend telemetry has also been throttled.
    adapter.observe_airtime(total=AIRTIME_FLOOR + 0.05,
                            image=0.0, telemetry=0.0)
    dec = adapter.tick(400)
    assert dec.action == AdapterAction.ENCODE_DEGRADE


# ---------- I6 -------------------------------------------------------

def test_power_change_rate_capped_at_10_hz():
    """I6: two ticks 50 ms apart must NOT both emit a power change."""
    adapter = TxPowerAdapterV3(initial_power_dbm=P_MIN_DBM)
    adapter.observe_airtime(total=0.1)
    now = _flood_snr(adapter, 0, MARGIN_FLOOR_SNR_DB - 5.0)
    adapter.tick(now)              # enter MARGIN_LIMITED
    dec1 = adapter.tick(now + INNER_MIN_INTERVAL_MS)
    assert dec1.action == AdapterAction.RAISE_POWER
    p1 = dec1.value

    # 50 ms later — too soon.
    dec2 = adapter.tick(now + INNER_MIN_INTERVAL_MS + 50)
    assert dec2.action == AdapterAction.NONE, (
        f"rate cap violated: emitted {dec2.action} only 50 ms after RAISE_POWER")
    assert adapter.power_dbm == p1

    # 100 ms later — eligible.
    dec3 = adapter.tick(now + 2 * INNER_MIN_INTERVAL_MS + 50)
    assert dec3.action == AdapterAction.RAISE_POWER
    assert dec3.value == p1 + 1


# ---------- I8 / I9 --------------------------------------------------

def test_recovery_dwell_holds_before_normal():
    """I8: after MARGIN clears, hold in RECOVERY for the full dwell
    before returning to NORMAL."""
    adapter = TxPowerAdapterV3()
    adapter.observe_airtime(total=0.1)
    now = _flood_snr(adapter, 0, MARGIN_FLOOR_SNR_DB - 5.0)
    adapter.tick(now)
    assert adapter.state == AdapterState.MARGIN_LIMITED

    # Clear margin (strong positive to overcome EWMA lag).
    now = _flood_snr(adapter, now + 1, MARGIN_CLEAR_SNR_DB + 30.0,
                     n=INNER_HYSTERESIS_PACKETS * 3)
    dec = adapter.tick(now)
    assert dec.state == AdapterState.RECOVERY
    recovery_entered = now

    # Just before dwell expires.
    dec = adapter.tick(recovery_entered + DWELL_RECOVERY_TO_NORMAL_MS - 1)
    assert dec.state == AdapterState.RECOVERY

    # Exactly at expiry.
    dec = adapter.tick(recovery_entered + DWELL_RECOVERY_TO_NORMAL_MS)
    assert dec.state == AdapterState.NORMAL


def test_recovery_re_enters_limited_on_redegrade():
    """I9: if SNR craters again mid-RECOVERY, jump back to MARGIN_LIMITED."""
    adapter = TxPowerAdapterV3()
    adapter.observe_airtime(total=0.1)
    now = _flood_snr(adapter, 0, MARGIN_FLOOR_SNR_DB - 5.0)
    adapter.tick(now)
    now = _flood_snr(adapter, now + 1, MARGIN_CLEAR_SNR_DB + 30.0,
                     n=INNER_HYSTERESIS_PACKETS * 3)
    adapter.tick(now)
    assert adapter.state == AdapterState.RECOVERY

    # Re-degrade (strong negative to drag EWMA back below floor).
    now = _flood_snr(adapter, now + 1, MARGIN_FLOOR_SNR_DB - 30.0,
                     n=INNER_HYSTERESIS_PACKETS * 3)
    dec = adapter.tick(now)
    assert dec.state == AdapterState.MARGIN_LIMITED, (
        "RECOVERY must re-enter MARGIN_LIMITED on re-degradation, "
        f"not stall (got {dec.state})")


# ---------- additional sanity ---------------------------------------

def test_normal_action_is_none_when_quiet():
    adapter = TxPowerAdapterV3()
    _flood_snr(adapter, 0, +10.0)
    adapter.observe_airtime(total=0.1)
    dec = adapter.tick(100)
    assert dec.state == AdapterState.NORMAL
    assert dec.action == AdapterAction.NONE


def test_power_saturates_at_cap_and_floor():
    """Power steps respect the [P_MIN_DBM, P_MAX_DBM] envelope."""
    adapter = TxPowerAdapterV3(initial_power_dbm=P_MAX_DBM)
    adapter.observe_airtime(total=0.1)
    now = _flood_snr(adapter, 0, MARGIN_FLOOR_SNR_DB - 5.0)
    adapter.tick(now)
    # Already at cap → MUST escalate to SF (not over-cap power).
    dec = adapter.tick(now + INNER_MIN_INTERVAL_MS)
    assert dec.action == AdapterAction.SF_STEP_UP
    assert adapter.power_dbm == P_MAX_DBM


def test_root_cause_tie_break_prefers_margin():
    """When both margin and airtime fire on the same tick from NORMAL,
    the adapter takes the MARGIN_LIMITED branch (raising power can drain
    PER even with high airtime; encode-degrade can't fix a margin hole)."""
    adapter = TxPowerAdapterV3()
    _flood_snr(adapter, 0, MARGIN_FLOOR_SNR_DB - 5.0)
    adapter.observe_airtime(total=AIRTIME_FLOOR + 0.05, image=0.4)
    dec = adapter.tick(100)
    assert dec.state == AdapterState.MARGIN_LIMITED
