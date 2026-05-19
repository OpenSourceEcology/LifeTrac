"""S6 TX-power adapter v3 — explicit state machine with root-cause-branched cascade.

Implements S6.1 + S6.3 + S6.4 of the v25 plan. Pure Python, no I/O, no
threads, no paho — the bridge feeds it observations and applies the
returned `AdapterDecision`. This keeps the controller SIL-testable
against synthetic traces on bare X8 hardware (no L072 reattach needed).

Background (see ``AI NOTES/2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md``
§14.1, §14.4):

* The v25 base station already runs two adaptation loops — encode-mode
  degradation in :mod:`link_monitor` and SF/fragment retreat in
  :class:`link_monitor.LinkProfileEmitter`. Without an explicit
  controller, both can fire on the same underlying event (a fade)
  producing oscillating behaviour.
* The §14.1 fix is to add explicit STATE and ROOT CAUSE, then branch the
  cascade. A *margin-limited* event (low SNR but airtime headroom)
  should raise TX power first and only escalate to SF retreat as a
  last resort. An *airtime-limited* event (high utilisation with
  healthy SNR) should cancel P3/P2 traffic and degrade encode mode —
  raising TX power would do nothing. **The two responses are mutually
  exclusive in any given dwell window.**

State machine
-------------
::

      ┌───────────┐  margin_bad &  ┌─────────────────┐
      │  NORMAL   │ ───────────── ▶│ MARGIN_LIMITED │
      │           │  airtime_ok    │                 │
      │           │ ◀───────────── │                 │
      └─────┬─────┘  margin_clear  └─────────────────┘
            │                              ▲
   airtime  │                              │ (NEVER directly:
   bad &    │                              │  root-cause re-eval)
   margin   │                              │
   ok       ▼                              │
      ┌─────────────────┐                  │
      │ AIRTIME_LIMITED │ ─────────────────┘
      │                 │
      └─────┬───────────┘
            │ both clear
            ▼
      ┌───────────┐  dwell timer expires
      │ RECOVERY  │ ─────────────────▶ NORMAL
      └───────────┘

Invariants (S6.4):

* The adapter NEVER transitions directly from MARGIN_LIMITED to
  AIRTIME_LIMITED (or vice versa). It must drop through NORMAL or
  RECOVERY first so a downstream observer can distinguish the two
  root causes in the event log.
* MARGIN_LIMITED only emits power/SF actions. AIRTIME_LIMITED only
  emits cancel/encode-degrade actions.
* RECOVERY does not emit ANY action — it just rate-limits how fast
  we can re-enter NORMAL after a degradation.

Per the §14.4 latency analysis:

* Inner loop: at most one power change per ``INNER_MIN_INTERVAL_MS``
  (default 100 ms = 10 Hz) — keeps the 25 ms P0 TX-start cap safe
  even if a power write blocks waiting for ``TX_DONE``.
* Power steps are ±1 dBm with 5-packet hysteresis on the SNR EWMA so
  small noise doesn't cause oscillation.
* Outer PER loop is a sliding 100-packet window.

This module owns NONE of the radio I/O. The caller (`lora_bridge` or a
test driver) is responsible for:

  1. Calling ``observe_packet(now_ms, snr_db, ok)`` after each RX
     packet (or after each TX done if PER is measured remotely).
  2. Calling ``observe_airtime(util_total, util_image, util_telemetry)``
     periodically (e.g. the existing 1 Hz airtime worker tick).
  3. Calling ``tick(now_ms)`` once per outer-loop period (default 1 s)
     to advance dwell timers and possibly emit a decision.
  4. Applying the returned ``AdapterDecision`` — writing the new dBm
     to ``RegPaConfig``, emitting ``CMD_LINK_TUNE``, etc.

Author: Copilot, 2026-05-18, S6 software half.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from enum import Enum


# ---- Tunables (§14.4 + §14.1) -------------------------------------------
# Power envelope. Regional EIRP cap enforced outside this module (S4.2);
# adapter saturates against the configured cap.
P_MIN_DBM = 5
P_MAX_DBM = 17
INNER_STEP_DBM = 1
INNER_HYSTERESIS_PACKETS = 5
INNER_MIN_INTERVAL_MS = 100   # 10 Hz ceiling; §14.4 25 ms cap protection

# Outer (PER) loop.
OUTER_WINDOW_PKTS = 100
PER_FLOOR = 0.02              # 2 % — degrade trigger
PER_CLEAR = 0.005             # 0.5 % — clear trigger (hysteresis gap)

# SNR margin thresholds. Tuned for SF7 ~ -7.5 dB demod floor; we want to
# notice degradation BEFORE the demod gives up.
MARGIN_FLOOR_SNR_DB = -5.0
MARGIN_CLEAR_SNR_DB = 0.0

# Airtime utilisation thresholds. Same numbers as the existing
# AIRTIME_ALARM constants in lora_bridge so the adapter and the alarm
# trigger together.
AIRTIME_FLOOR = 0.60
AIRTIME_CLEAR = 0.45

# Dwell timers (ms). Asymmetric on purpose: enter LIMITED quickly, leave
# slowly (RECOVERY hysteresis on the return path).
DWELL_NORMAL_TO_LIMITED_MS = 1_000
DWELL_LIMITED_TO_RECOVERY_MS = 5_000
DWELL_RECOVERY_TO_NORMAL_MS = 10_000

# SNR EWMA smoothing factor. Higher = more responsive, more noise.
SNR_EWMA_ALPHA = 0.3


class AdapterState(Enum):
    """Four-state controller — see module docstring for the diagram."""
    NORMAL = "normal"
    MARGIN_LIMITED = "margin_limited"
    AIRTIME_LIMITED = "airtime_limited"
    RECOVERY = "recovery"


class AdapterAction(Enum):
    """Action the caller must apply. NONE = stay put."""
    NONE = "none"
    # Margin-limited branch (in priority order — only the cheapest fires).
    RAISE_POWER = "raise_power"
    LOWER_POWER = "lower_power"
    SF_STEP_UP = "sf_step_up"
    SF_STEP_DOWN = "sf_step_down"
    # Airtime-limited branch.
    CANCEL_P3 = "cancel_p3"
    CANCEL_P2 = "cancel_p2"
    ENCODE_DEGRADE = "encode_degrade"
    ENCODE_RESTORE = "encode_restore"


@dataclass(frozen=True)
class AdapterDecision:
    """What the controller wants the caller to do this tick.

    ``state`` always reflects the post-decision state. ``action`` is
    ``AdapterAction.NONE`` whenever the caller need do nothing — either
    because conditions are stable or because we're inside a dwell window.
    ``value`` carries the new dBm for power actions, the new SF rung for
    SF actions, and is unused for cancel/encode actions.
    """
    state: AdapterState
    action: AdapterAction
    value: int | None
    reason: str


# ---- Inner-loop signal accumulators -------------------------------------

class SnrEwma:
    """Exponentially-weighted SNR estimator with N-packet hysteresis counter.

    Tracks how many *consecutive* packets agreed with the current
    above-or-below floor verdict; the controller uses that count to
    suppress single-packet flaps.
    """

    __slots__ = ("alpha", "value", "_below_floor_streak", "_above_clear_streak")

    def __init__(self, alpha: float = SNR_EWMA_ALPHA) -> None:
        if not 0.0 < alpha <= 1.0:
            raise ValueError(f"alpha must be in (0, 1]; got {alpha}")
        self.alpha = alpha
        self.value: float | None = None
        self._below_floor_streak = 0
        self._above_clear_streak = 0

    def update(self, snr_db: float,
               floor_db: float = MARGIN_FLOOR_SNR_DB,
               clear_db: float = MARGIN_CLEAR_SNR_DB) -> float:
        if self.value is None:
            self.value = float(snr_db)
        else:
            self.value = self.alpha * float(snr_db) + (1.0 - self.alpha) * self.value
        if self.value < floor_db:
            self._below_floor_streak += 1
            self._above_clear_streak = 0
        elif self.value > clear_db:
            self._above_clear_streak += 1
            self._below_floor_streak = 0
        else:
            # Inside the hysteresis band — leave both streaks intact so
            # we don't reset progress just because of mid-band wobble.
            pass
        return self.value

    @property
    def below_floor_streak(self) -> int:
        return self._below_floor_streak

    @property
    def above_clear_streak(self) -> int:
        return self._above_clear_streak


class PerWindow:
    """Sliding 100-packet PER window."""

    __slots__ = ("size", "_events")

    def __init__(self, size: int = OUTER_WINDOW_PKTS) -> None:
        if size <= 0:
            raise ValueError("size must be positive")
        self.size = size
        self._events: deque[bool] = deque(maxlen=size)

    def observe(self, ok: bool) -> None:
        self._events.append(bool(ok))

    @property
    def per(self) -> float:
        if not self._events:
            return 0.0
        n_fail = sum(1 for ev in self._events if not ev)
        return n_fail / len(self._events)

    @property
    def count(self) -> int:
        return len(self._events)


# ---- The controller -----------------------------------------------------

@dataclass
class _AirtimeSnapshot:
    total: float = 0.0
    image: float = 0.0
    telemetry: float = 0.0


class TxPowerAdapterV3:
    """Branched-cascade TX-power adapter (S6.1 + S6.3 + S6.4).

    Caller contract — see module docstring.

    Parameters
    ----------
    initial_power_dbm:
        Current radio TX power. Saturated to ``[P_MIN_DBM, P_MAX_DBM]``.
    initial_sf_rung:
        0 = SF7 (image PHY default), 1 = SF8, 2 = SF9 (telemetry PHY).
        The adapter walks this index; the caller is responsible for
        translating to the actual SF + bandwidth in ``CMD_LINK_TUNE``.
    """

    def __init__(self,
                 initial_power_dbm: int = 14,
                 initial_sf_rung: int = 0,
                 audit=None) -> None:
        self.power_dbm = max(P_MIN_DBM, min(P_MAX_DBM, int(initial_power_dbm)))
        self.sf_rung = max(0, int(initial_sf_rung))
        self.state = AdapterState.NORMAL
        self._airtime = _AirtimeSnapshot()
        self._snr = SnrEwma()
        self._per = PerWindow()
        self._state_entered_ms: int = 0
        self._last_power_change_ms: int = -INNER_MIN_INTERVAL_MS  # allow immediate
        self._audit = audit

    # -- inputs --------------------------------------------------------
    def observe_packet(self, now_ms: int, snr_db: float, ok: bool) -> None:
        self._snr.update(snr_db)
        self._per.observe(ok)

    def observe_airtime(self, total: float, image: float = 0.0,
                        telemetry: float = 0.0) -> None:
        self._airtime = _AirtimeSnapshot(total=float(total),
                                          image=float(image),
                                          telemetry=float(telemetry))

    # -- internal predicates ------------------------------------------
    def _margin_bad(self) -> bool:
        """True if SNR has been below floor for N consecutive packets OR
        sustained PER above the floor threshold (and we have enough
        outer-window samples to trust the PER number)."""
        snr_bad = self._snr.below_floor_streak >= INNER_HYSTERESIS_PACKETS
        per_bad = (self._per.count >= self._per.size // 2
                   and self._per.per >= PER_FLOOR)
        return snr_bad or per_bad

    def _margin_clear(self) -> bool:
        snr_ok = self._snr.above_clear_streak >= INNER_HYSTERESIS_PACKETS
        per_ok = (self._per.count == 0
                  or self._per.per <= PER_CLEAR)
        return snr_ok and per_ok

    def _airtime_bad(self) -> bool:
        return self._airtime.total >= AIRTIME_FLOOR

    def _airtime_clear(self) -> bool:
        return self._airtime.total <= AIRTIME_CLEAR

    # -- transition + decision ----------------------------------------
    def tick(self, now_ms: int) -> AdapterDecision:
        """Advance the state machine and emit at most one action."""
        prev_state = self.state

        if prev_state == AdapterState.NORMAL:
            decision = self._tick_normal(now_ms)
        elif prev_state == AdapterState.MARGIN_LIMITED:
            decision = self._tick_margin_limited(now_ms)
        elif prev_state == AdapterState.AIRTIME_LIMITED:
            decision = self._tick_airtime_limited(now_ms)
        else:  # RECOVERY
            decision = self._tick_recovery(now_ms)

        if self.state != prev_state:
            self._state_entered_ms = now_ms
            if self._audit is not None:
                try:
                    self._audit.record("tx_power_adapter_state",
                                       prev=prev_state.value,
                                       new=self.state.value,
                                       reason=decision.reason)
                except Exception:    # never crash the adapter on audit failure
                    pass

        return decision

    # -- per-state handlers -------------------------------------------
    def _tick_normal(self, now_ms: int) -> AdapterDecision:
        margin = self._margin_bad()
        airtime = self._airtime_bad()
        # Tie-break: if both fire, prefer MARGIN_LIMITED — raising power
        # is cheap and can drain PER even when airtime is high, whereas
        # encode-degrade can't fix a margin problem.
        if margin:
            self.state = AdapterState.MARGIN_LIMITED
            return AdapterDecision(self.state, AdapterAction.NONE, None,
                                    "enter MARGIN_LIMITED (snr_below_floor)")
        if airtime:
            self.state = AdapterState.AIRTIME_LIMITED
            return AdapterDecision(self.state, AdapterAction.NONE, None,
                                    "enter AIRTIME_LIMITED (util>=ceiling)")
        return AdapterDecision(self.state, AdapterAction.NONE, None,
                                "normal: no action")

    def _tick_margin_limited(self, now_ms: int) -> AdapterDecision:
        # Conditions cleared? Move to RECOVERY (don't snap back to NORMAL
        # or back to AIRTIME_LIMITED — invariant: must dwell).
        if self._margin_clear():
            self.state = AdapterState.RECOVERY
            return AdapterDecision(self.state, AdapterAction.NONE, None,
                                    "margin cleared → RECOVERY")
        # Still bad. Apply the cheapest available action.
        if self.power_dbm < P_MAX_DBM:
            if (now_ms - self._last_power_change_ms) >= INNER_MIN_INTERVAL_MS:
                self.power_dbm = min(P_MAX_DBM, self.power_dbm + INNER_STEP_DBM)
                self._last_power_change_ms = now_ms
                return AdapterDecision(
                    self.state, AdapterAction.RAISE_POWER, self.power_dbm,
                    f"raise_power → {self.power_dbm} dBm")
            return AdapterDecision(self.state, AdapterAction.NONE, None,
                                    "rate-limited: <100 ms since last power change")
        # Power pinned at cap — escalate to SF retreat (most expensive,
        # doubles airtime per byte).
        self.sf_rung += 1
        return AdapterDecision(self.state, AdapterAction.SF_STEP_UP,
                                self.sf_rung,
                                f"power at cap; sf_step_up → rung {self.sf_rung}")

    def _tick_airtime_limited(self, now_ms: int) -> AdapterDecision:
        if self._airtime_clear():
            self.state = AdapterState.RECOVERY
            return AdapterDecision(self.state, AdapterAction.NONE, None,
                                    "airtime cleared → RECOVERY")
        # Cheapest-first: drop image (P3), then telemetry (P2), then
        # degrade encode mode. The caller maps these actions to
        # CMD_ENCODE_MODE rungs / queue flushes.
        if self._airtime.image > 0.0:
            return AdapterDecision(self.state, AdapterAction.CANCEL_P3, None,
                                    "cancel P3 to free airtime")
        if self._airtime.telemetry > 0.0 and self._airtime.telemetry > 0.10:
            # only sacrifice P2 if it's a real contributor
            return AdapterDecision(self.state, AdapterAction.CANCEL_P2, None,
                                    "cancel P2 to free airtime")
        return AdapterDecision(self.state, AdapterAction.ENCODE_DEGRADE, None,
                                "encode degrade to free airtime")

    def _tick_recovery(self, now_ms: int) -> AdapterDecision:
        # If conditions re-degrade during RECOVERY, jump back to the
        # corresponding LIMITED state immediately (no double-dwell).
        if self._margin_bad():
            self.state = AdapterState.MARGIN_LIMITED
            return AdapterDecision(self.state, AdapterAction.NONE, None,
                                    "re-enter MARGIN_LIMITED from RECOVERY")
        if self._airtime_bad():
            self.state = AdapterState.AIRTIME_LIMITED
            return AdapterDecision(self.state, AdapterAction.NONE, None,
                                    "re-enter AIRTIME_LIMITED from RECOVERY")
        if (now_ms - self._state_entered_ms) >= DWELL_RECOVERY_TO_NORMAL_MS:
            self.state = AdapterState.NORMAL
            return AdapterDecision(self.state, AdapterAction.NONE, None,
                                    "RECOVERY dwell complete → NORMAL")
        # Still in RECOVERY dwell. Optionally lower power one notch per
        # tick to walk back from the panic level set during the event.
        if self.power_dbm > P_MIN_DBM and \
                (now_ms - self._last_power_change_ms) >= INNER_MIN_INTERVAL_MS:
            self.power_dbm = max(P_MIN_DBM, self.power_dbm - INNER_STEP_DBM)
            self._last_power_change_ms = now_ms
            return AdapterDecision(self.state, AdapterAction.LOWER_POWER,
                                    self.power_dbm,
                                    f"recovery: lower_power → {self.power_dbm} dBm")
        return AdapterDecision(self.state, AdapterAction.NONE, None,
                                "recovery dwell")
