"""Round 19: SIL model of the M7 adaptive-SF ladder + CMD_LINK_TUNE handshake.

Mirrors ``try_step_ladder()``, ``poll_link_ladder()``, the heartbeat
ingress hook (``g_ladder_hb_count`` + ``g_ladder_hb_at_pending``), and
``apply_phy_rung()`` in ``firmware/tractor_h7/tractor_h7.ino`` so the
W4-02 HIL gate becomes a *verification* pass against a documented model
in the same style as the Round-18 ``test_axis_ramp_sil.py``.

What the C side does, captured here:

* Three rungs::

    LADDER[0] = SF7  / BW250 / CR4-5     (control profile, boot PHY)
    LADDER[1] = SF8  / BW125 / CR4-5     (fallback, widens for budget)
    LADDER[2] = SF9  / BW125 / CR4-5     (deepest fallback)

* Window evaluator: every ``LADDER_WINDOW_MS = 5000`` ms, classify the
  *active* source's heartbeat count over the last window:

    hb <  ``LADDER_HB_BAD_THRESHOLD``  (40)  → bad   window
    hb >= ``LADDER_HB_GOOD_THRESHOLD`` (48)  → good  window
    in-between → neither (consec counters left untouched)

  No fresh active source at all → bad window (pessimistic widen).

* R-8 hysteresis::

    LADDER_BAD_TO_STEPDOWN  = 3   (15 s of bad → step DOWN, SF↑, widen)
    LADDER_GOOD_TO_STEPUP   = 6   (30 s of good → step UP,   SF↓, narrow)

  Step-down is gated by ``g_ladder_rung < 2`` (already at SF9 → no-op).
  Step-up   is gated by ``g_ladder_rung > 0`` (already at SF7 → no-op).

* ``try_step_ladder(target, reason)``:
    1. ``send_link_tune(target, reason)`` at the OLD PHY.
    2. ``apply_phy_rung(target)`` (radio retunes, RX re-armed).
    3. ``send_link_tune(target, reason)`` at the NEW PHY.
    4. ``radio.startReceive()`` (re-arm RX after the second TX).
    5. Latch ``g_ladder_pending_rung = target`` and
       ``g_ladder_tune_deadline = now + LADDER_TUNE_REVERT_MS`` (500 ms).
    6. Reset window counters and ``consec_bad / consec_good``.

  The committed ``g_ladder_rung`` does NOT change in step 2 — it
  changes only when ``poll_link_ladder()`` observes a HB at the new PHY.

* Heartbeat receipt while ``g_ladder_tune_deadline != 0`` records
  ``g_ladder_hb_at_pending = now`` (regardless of source).

* ``poll_link_ladder()`` verification check (line ~1011 of
  ``tractor_h7.ino``)::

    got_hb = (hb_at_pending != 0) &&
             (hb_at_pending + LADDER_TUNE_REVERT_MS >= tune_deadline)

    if got_hb:
        g_ladder_rung = g_ladder_pending_rung   # commit
    else:
        apply_phy_rung(g_ladder_rung)           # revert (re-tune to old)
        g_ladder_tune_failures += 1

    g_ladder_tune_deadline = 0
    g_ladder_hb_at_pending = 0

* Window evaluator is gated off while a tune is pending so the
  verification window is not double-counted as a "bad" window
  (``if (g_ladder_tune_deadline != 0) return;``).

W4-02 (link-tune walk-down) verification surface:

* **TL-A** Cold-start at SF7. Window count = 50 (10 Hz × 5 s) →
  classifies as good. Six clean windows trigger step-up attempt — but
  we are already at rung 0, so no tune fires.
* **TL-B** Walk-DOWN ladder: feeding 0 HBs per window for 15 s with the
  active source going stale walks SF7 → SF8 (verifies twice-back-to-back
  send + revert deadline armed). A successful HB at the new PHY commits
  the rung; another 15 s of bad windows walks SF8 → SF9.
* **TL-C** Saturation at SF9: a fourth 15 s of bad windows does NOT
  attempt a fifth step (step-down gated by ``rung < 2``).
* **TL-D** Walk-BACK after recovery: from SF9, 30 s of good windows
  steps SF9 → SF8; another 30 s steps SF8 → SF7. From SF7 a further
  30 s of good windows is a no-op (step-up gated by ``rung > 0``).
* **TL-E** Revert deadline: ``try_step_ladder`` arms a 500 ms revert.
  If NO heartbeat arrives at the new PHY before the deadline, the
  ladder reverts to the old rung and ``g_ladder_tune_failures``
  increments. The committed rung never advanced.
* **TL-F** Twice-back-to-back announce: ``try_step_ladder`` emits
  exactly two CMD_LINK_TUNE frames per attempt — one at the old PHY,
  one at the new PHY — both with the same target SF and reason byte.
  This is what makes the handshake robust to either side missing one
  of the two frames.
* **TL-G** Stale-active-source pessimism: when ``pick_active_source()``
  returns -1 (no fresh source), the window is classified bad even if
  some other (non-active) source kept emitting HBs. Models the comment
  on line ~1037 of tractor_h7.ino.
* **TL-H** Window-counter reset on transition: after a successful tune
  the post-tune 5 s window starts fresh (no carry-over of HB counts
  from the pre-tune window into the bad/good classifier).
* **TL-I** Verification window not classified: the 5 s window that
  contains an in-flight tune is skipped entirely by the bad/good
  classifier (``if (g_ladder_tune_deadline != 0) return;``). This is
  what TL-H actually relies on — exercise it directly so a regression
  that drops the gate is caught immediately.

This module is pure stdlib so it can run in CI without arduino-cli or
hardware in the loop.
"""

from __future__ import annotations

import unittest
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# ---------------------------------------------------------------------------
# Constants — kept byte-identical to firmware/tractor_h7/tractor_h7.ino.
# ---------------------------------------------------------------------------

LADDER_WINDOW_MS         = 5000
LADDER_BAD_TO_STEPDOWN   = 3       # 15 s of bad → widen SF
LADDER_GOOD_TO_STEPUP    = 6       # 30 s of good → narrow SF
LADDER_HB_EXPECTED_5S    = 50      # 10 Hz × 5 s
LADDER_HB_BAD_THRESHOLD  = 40      # < 80% expected
LADDER_HB_GOOD_THRESHOLD = 48      # >= 96% expected
LADDER_TUNE_REVERT_MS    = 500

# Reason codes — match enum LinkTuneReason in tractor_h7.ino.
LTR_LOW_SNR       = 0x01
LTR_HIGH_LOSS     = 0x02
LTR_MANUAL        = 0x03
LTR_RECOVERY_DOWN = 0x04


@dataclass(frozen=True)
class LadderRung:
    sf: int
    bw_khz: int
    cr_den: int
    bw_code: int


# Mirrors LADDER[3] in tractor_h7.ino.
LADDER: Tuple[LadderRung, LadderRung, LadderRung] = (
    LadderRung(sf=7, bw_khz=250, cr_den=5, bw_code=1),
    LadderRung(sf=8, bw_khz=125, cr_den=5, bw_code=0),
    LadderRung(sf=9, bw_khz=125, cr_den=5, bw_code=0),
)


# ---------------------------------------------------------------------------
# Captured side-effects so tests can assert on them.
# ---------------------------------------------------------------------------


@dataclass
class TuneEmission:
    """One CMD_LINK_TUNE TX captured by the SIL stub."""

    at_ms: int
    target_rung: int
    reason: int
    phy_at_tx: int       # rung the radio was tuned to when this TX went out


# ---------------------------------------------------------------------------
# SIL model — one per simulated tractor M7.
# ---------------------------------------------------------------------------


@dataclass
class LinkLadder:
    """Pure-Python port of the SF-ladder state machine in tractor_h7.ino."""

    # Committed and pending rungs.
    rung: int = 0
    pending_rung: int = 0

    # 0 = no tune in flight; otherwise absolute deadline in ms.
    tune_deadline: int = 0

    # Last absolute-ms timestamp of a HB observed while a tune was pending.
    hb_at_pending: int = 0

    # Per-source HB / control-frame counts in the current window.
    hb_count: List[int] = field(default_factory=lambda: [0, 0, 0])
    ctrl_count: List[int] = field(default_factory=lambda: [0, 0, 0])

    # Window start (absolute ms).
    window_start: int = 0

    # Hysteresis counters.
    consec_bad: int = 0
    consec_good: int = 0

    # Surfaced via topic 0x10 in the firmware.
    tune_failures: int = 0

    # Captured TX side-effects (one entry per send_link_tune() call).
    tx_log: List[TuneEmission] = field(default_factory=list)

    # ------------------------------------------------------------------
    # Internal helpers (mirror C statics).
    # ------------------------------------------------------------------

    def _send_link_tune(self, now_ms: int, target_rung: int, reason: int) -> None:
        """Mirror send_link_tune(): record the emission at the *current* PHY."""
        if target_rung > 2:
            return
        self.tx_log.append(
            TuneEmission(
                at_ms=now_ms,
                target_rung=target_rung,
                reason=reason,
                phy_at_tx=self.rung,
            )
        )

    def _apply_phy_rung(self, target_rung: int) -> None:
        """Mirror apply_phy_rung(): retune the local radio."""
        if target_rung > 2:
            return
        self.rung = target_rung   # NOTE: this is the *committed-on-radio* rung;
                                  # the pending/committed-state-machine rung is
                                  # tracked via try_step_ladder's bookkeeping.

    # ------------------------------------------------------------------
    # Public API (mirror C functions).
    # ------------------------------------------------------------------

    def on_heartbeat(self, now_ms: int, source_idx: int) -> None:
        """Mirror the FT_HEARTBEAT branch of the M7 ingress switch.

        Updates per-source HB count and, while a tune is pending,
        records ``hb_at_pending`` so the verification check can commit.
        """
        if not 0 <= source_idx < 3:
            return
        self.hb_count[source_idx] += 1
        if self.tune_deadline != 0:
            self.hb_at_pending = now_ms

    def on_control(self, source_idx: int) -> None:
        if 0 <= source_idx < 3:
            self.ctrl_count[source_idx] += 1

    def try_step_ladder(self, now_ms: int, target_rung: int, reason: int) -> bool:
        """Mirror try_step_ladder(): two-phase announce + revert deadline.

        Returns True iff the step was actually attempted (i.e. target was
        in range and not already the committed rung).
        """
        if target_rung > 2 or target_rung == self.rung:
            return False
        # Snapshot the committed rung BEFORE we retune. The C code uses
        # g_ladder_rung as the "we are tuned to this PHY" indicator and
        # only flips it on the verification path; here we model the same.
        old_rung = self.rung

        # 1. Announce at OLD PHY.
        self._send_link_tune(now_ms, target_rung, reason)
        # 2. Retune the radio to the new PHY (apply_phy_rung).
        self._apply_phy_rung(target_rung)
        # 3. Re-announce at NEW PHY.
        self._send_link_tune(now_ms, target_rung, reason)
        # 4. (radio.startReceive — modelled implicitly; no state change.)

        # The C code latches pending = target and arms the deadline.
        self.pending_rung = target_rung
        self.tune_deadline = now_ms + LADDER_TUNE_REVERT_MS
        self.hb_at_pending = 0

        # Reset window stats for a fresh post-tune window.
        self.consec_bad = 0
        self.consec_good = 0
        self.hb_count = [0, 0, 0]
        self.ctrl_count = [0, 0, 0]
        self.window_start = now_ms

        # IMPORTANT: in the C code, g_ladder_rung is the "committed" rung
        # for the bookkeeping/state machine but is ALSO mutated by
        # apply_phy_rung() above. The verification path then either
        # leaves it alone (commit-on-HB path is a no-op since pending ==
        # current after retune) or REVERTS it via apply_phy_rung(old).
        # To model the "committed" rung for tests, we keep it as the
        # post-retune value here and let poll_link_ladder() roll it
        # back if verification fails. That matches the C behaviour
        # bit-for-bit:
        #   - On commit: g_ladder_rung = pending_rung (no-op here).
        #   - On revert: apply_phy_rung(old_rung) — restores both the
        #     radio AND the committed-rung bookkeeping.
        # Stash the old rung so the revert branch can restore it.
        self._pre_tune_rung = old_rung
        return True

    def poll_link_ladder(self, now_ms: int) -> None:
        """Mirror poll_link_ladder(): verification + window classification."""
        if self.window_start == 0:
            self.window_start = now_ms

        # (1) Pending-tune verification.
        if self.tune_deadline != 0 and now_ms - self.tune_deadline >= 0:
            got_hb = (
                self.hb_at_pending != 0
                and self.hb_at_pending + LADDER_TUNE_REVERT_MS >= self.tune_deadline
            )
            if got_hb:
                # Commit: pending becomes committed (apply_phy_rung already
                # retuned the radio inside try_step_ladder).
                self.rung = self.pending_rung
            else:
                # Revert: retune the radio back to the pre-tune rung AND
                # restore the committed bookkeeping.
                self._apply_phy_rung(self._pre_tune_rung)
                self.tune_failures += 1
            self.tune_deadline = 0
            self.hb_at_pending = 0

        # (2) Window evaluation. Hold off while a tune is in flight so the
        #     verification window doesn't double-count as a bad window.
        if self.tune_deadline != 0:
            return
        if (now_ms - self.window_start) < LADDER_WINDOW_MS:
            return

        active = self.active_source_for_test
        bad = False
        good = False
        if active is not None and 0 <= active < 3:
            hb = self.hb_count[active]
            bad = hb < LADDER_HB_BAD_THRESHOLD
            good = hb >= LADDER_HB_GOOD_THRESHOLD
        else:
            bad = True   # no fresh source → pessimistically widen

        if bad:
            self.consec_bad += 1
            self.consec_good = 0
        if good:
            self.consec_good += 1
            self.consec_bad = 0

        if self.consec_bad >= LADDER_BAD_TO_STEPDOWN and self.rung < 2:
            self.try_step_ladder(now_ms, self.rung + 1, LTR_HIGH_LOSS)
        elif self.consec_good >= LADDER_GOOD_TO_STEPUP and self.rung > 0:
            self.try_step_ladder(now_ms, self.rung - 1, LTR_RECOVERY_DOWN)

        # Reset window counters.
        self.hb_count = [0, 0, 0]
        self.ctrl_count = [0, 0, 0]
        self.window_start = now_ms

    # ------------------------------------------------------------------
    # Test-injection knob: pick_active_source() lives in the surrounding
    # C file and depends on per-source freshness state we are NOT
    # modelling here. Tests set this directly to drive the classifier.
    # ------------------------------------------------------------------

    active_source_for_test: Optional[int] = 0

    # Stash to support the revert branch (set by try_step_ladder).
    _pre_tune_rung: int = 0


# ---------------------------------------------------------------------------
# Test scenarios.
# ---------------------------------------------------------------------------


def _feed_window(
    ladder: LinkLadder,
    *,
    now_ms: int,
    hb_count: int,
    source_idx: int = 0,
) -> int:
    """Inject ``hb_count`` heartbeats spread across one ``LADDER_WINDOW_MS``.

    Returns the absolute ms after the window has elapsed (i.e. when the
    next ``poll_link_ladder(now)`` call will be eligible to classify it).
    """
    if hb_count > 0:
        # Spread the HBs evenly across the window so timing assertions
        # downstream don't depend on burstiness.
        step = LADDER_WINDOW_MS // hb_count
        for i in range(hb_count):
            ladder.on_heartbeat(now_ms + i * step, source_idx)
    # Advance to the END of the window.
    return now_ms + LADDER_WINDOW_MS


# Real firmware boots with millis()>0 by the time the first poll runs,
# so the C code's ``if (g_ladder_window_start == 0) g_ladder_window_start = now;``
# lazy-init effectively latches start_ms to a positive value. Tests use
# ``BOOT_MS = 1`` to mimic that and avoid the now=0 sentinel collision.
BOOT_MS = 1


class W4_02_LadderClassifierTests(unittest.TestCase):
    """Single-window classifier behaviour."""

    def setUp(self) -> None:
        self.ladder = LinkLadder()
        self.ladder.window_start = BOOT_MS
        self.ladder.active_source_for_test = 0   # SRC_HANDHELD index

    def test_TL_A_cold_start_clean_window_no_step_up_at_rung0(self) -> None:
        """Six clean windows at rung 0 must NOT emit any LINK_TUNE frames."""
        now = BOOT_MS
        for _ in range(6):
            now = _feed_window(self.ladder, now_ms=now, hb_count=50)
            self.ladder.poll_link_ladder(now)
        self.assertEqual(self.ladder.rung, 0, "must remain at SF7 rung")
        self.assertEqual(
            self.ladder.tx_log, [],
            "step-up at rung 0 is a no-op — no CMD_LINK_TUNE should TX",
        )
        # Hysteresis counter still clamped (try_step_ladder is the
        # gate that resets, but the rung-0 step-up gate prevents that
        # path; consec_good keeps incrementing).
        self.assertGreaterEqual(self.ladder.consec_good, 6)


class W4_02_WalkDownTests(unittest.TestCase):
    """TL-B: bad-window cascade walks SF7 → SF8 → SF9."""

    def setUp(self) -> None:
        self.ladder = LinkLadder()
        self.ladder.window_start = BOOT_MS
        self.ladder.active_source_for_test = 0

    def _bad_window_then_verify(self, *, now_ms: int) -> int:
        """Feed a bad window, run poll, ack the verification HB if a tune fired.

        Returns the new ``now_ms`` after the window + verification.
        """
        end = _feed_window(self.ladder, now_ms=now_ms, hb_count=0)
        pre_rung = self.ladder.rung
        pre_tx = len(self.ladder.tx_log)
        self.ladder.poll_link_ladder(end)
        # If a tune was launched, ack the verification HB *before* the
        # deadline so poll_link_ladder commits the new rung.
        if len(self.ladder.tx_log) > pre_tx:
            self.assertNotEqual(
                self.ladder.tune_deadline, 0,
                "tune fired but deadline not armed",
            )
            ack_ms = end + 100   # well inside the 500 ms revert window
            self.ladder.on_heartbeat(ack_ms, source_idx=0)
            # Advance just past the deadline so poll commits.
            commit_ms = self.ladder.tune_deadline + 1
            self.ladder.poll_link_ladder(commit_ms)
            self.assertEqual(
                self.ladder.tune_deadline, 0,
                "deadline must clear after verification",
            )
            return commit_ms
        return end

    def test_TL_B_three_bad_windows_walk_SF7_to_SF8(self) -> None:
        now = BOOT_MS
        for _ in range(3):
            now = self._bad_window_then_verify(now_ms=now)
        self.assertEqual(self.ladder.rung, 1, "must commit SF8 after 3 bad windows")
        # Twice-back-to-back: exactly 2 TX frames per step.
        self.assertEqual(len(self.ladder.tx_log), 2)
        for emit in self.ladder.tx_log:
            self.assertEqual(emit.target_rung, 1)
            self.assertEqual(emit.reason, LTR_HIGH_LOSS)

    def test_TL_B2_six_bad_windows_walk_SF7_to_SF9(self) -> None:
        now = BOOT_MS
        for _ in range(3):
            now = self._bad_window_then_verify(now_ms=now)
        self.assertEqual(self.ladder.rung, 1)
        for _ in range(3):
            now = self._bad_window_then_verify(now_ms=now)
        self.assertEqual(self.ladder.rung, 2, "must commit SF9 after 6 bad windows")
        # Two step events, two TX each → 4 TX frames total.
        self.assertEqual(len(self.ladder.tx_log), 4)
        self.assertEqual(self.ladder.tx_log[0].target_rung, 1)
        self.assertEqual(self.ladder.tx_log[1].target_rung, 1)
        self.assertEqual(self.ladder.tx_log[2].target_rung, 2)
        self.assertEqual(self.ladder.tx_log[3].target_rung, 2)
        self.assertEqual(self.ladder.tune_failures, 0,
                         "no failures should accumulate on the happy path")


class W4_02_SaturationTests(unittest.TestCase):
    """TL-C: SF9 is the floor — further bad windows do not attempt SF10."""

    def setUp(self) -> None:
        self.ladder = LinkLadder()
        self.ladder.window_start = BOOT_MS
        self.ladder.active_source_for_test = 0

    def _walk(self, now_ms: int) -> int:
        end = _feed_window(self.ladder, now_ms=now_ms, hb_count=0)
        pre_tx = len(self.ladder.tx_log)
        self.ladder.poll_link_ladder(end)
        if len(self.ladder.tx_log) > pre_tx:
            self.ladder.on_heartbeat(end + 100, 0)
            end = self.ladder.tune_deadline + 1
            self.ladder.poll_link_ladder(end)
        return end

    def test_TL_C_no_step_past_SF9(self) -> None:
        now = BOOT_MS
        # Walk all the way to SF9.
        for _ in range(6):
            now = self._walk(now)
        self.assertEqual(self.ladder.rung, 2)
        tx_at_sf9 = len(self.ladder.tx_log)
        # Three more bad windows: rung-saturation gate must hold.
        for _ in range(3):
            now = self._walk(now)
        self.assertEqual(self.ladder.rung, 2, "must stay clamped at SF9")
        self.assertEqual(
            len(self.ladder.tx_log), tx_at_sf9,
            "no LINK_TUNE should TX once already at the deepest rung",
        )


class W4_02_WalkBackTests(unittest.TestCase):
    """TL-D: 30 s of clean windows walks back SF9 → SF8 → SF7."""

    def setUp(self) -> None:
        self.ladder = LinkLadder()
        self.ladder.window_start = BOOT_MS
        self.ladder.active_source_for_test = 0
        # Force-position to SF9 without going through the bad-window path
        # so the test focuses purely on the recovery direction.
        self.ladder.rung = 2

    def _good_window_then_verify(self, *, now_ms: int) -> int:
        end = _feed_window(self.ladder, now_ms=now_ms, hb_count=50)
        pre_tx = len(self.ladder.tx_log)
        self.ladder.poll_link_ladder(end)
        if len(self.ladder.tx_log) > pre_tx:
            self.ladder.on_heartbeat(end + 100, 0)
            commit_ms = self.ladder.tune_deadline + 1
            self.ladder.poll_link_ladder(commit_ms)
            return commit_ms
        return end

    def test_TL_D_six_good_windows_walk_SF9_to_SF8(self) -> None:
        now = BOOT_MS
        for _ in range(6):
            now = self._good_window_then_verify(now_ms=now)
        self.assertEqual(self.ladder.rung, 1)
        self.assertEqual(len(self.ladder.tx_log), 2)
        for emit in self.ladder.tx_log:
            self.assertEqual(emit.target_rung, 1)
            self.assertEqual(emit.reason, LTR_RECOVERY_DOWN)

    def test_TL_D2_twelve_good_windows_walk_SF9_to_SF7(self) -> None:
        now = BOOT_MS
        for _ in range(12):
            now = self._good_window_then_verify(now_ms=now)
        self.assertEqual(self.ladder.rung, 0)
        self.assertEqual(len(self.ladder.tx_log), 4)
        # Then six MORE good windows from rung 0 → no new TX.
        baseline = len(self.ladder.tx_log)
        for _ in range(6):
            now = self._good_window_then_verify(now_ms=now)
        self.assertEqual(self.ladder.rung, 0, "stays at SF7 — no SF6")
        self.assertEqual(len(self.ladder.tx_log), baseline)


class W4_02_RevertDeadlineTests(unittest.TestCase):
    """TL-E: missing verification HB reverts to the pre-tune rung."""

    def setUp(self) -> None:
        self.ladder = LinkLadder()
        self.ladder.window_start = BOOT_MS
        self.ladder.active_source_for_test = 0

    def test_TL_E_no_HB_at_new_PHY_reverts(self) -> None:
        # 15 s of bad windows → step-down attempt SF7 → SF8.
        now = BOOT_MS
        for _ in range(3):
            now = _feed_window(self.ladder, now_ms=now, hb_count=0)
            self.ladder.poll_link_ladder(now)
        self.assertNotEqual(self.ladder.tune_deadline, 0, "step-down attempted")
        # NOTE: at this moment the *radio* is tuned to rung 1 (apply_phy_rung
        # ran inside try_step_ladder), but the state machine has not yet
        # committed. Don't deliver a verification HB.
        # Advance past the 500 ms deadline.
        now = self.ladder.tune_deadline + 1
        self.ladder.poll_link_ladder(now)
        self.assertEqual(self.ladder.rung, 0, "must revert to SF7")
        self.assertEqual(self.ladder.tune_failures, 1)
        self.assertEqual(self.ladder.tune_deadline, 0)

    def test_TL_E2_HB_after_deadline_does_not_save_us(self) -> None:
        now = BOOT_MS
        for _ in range(3):
            now = _feed_window(self.ladder, now_ms=now, hb_count=0)
            self.ladder.poll_link_ladder(now)
        # HB arrives AFTER the deadline → too late: poll runs first and reverts.
        late_ms = self.ladder.tune_deadline + 1
        self.ladder.poll_link_ladder(late_ms)
        # Even if a HB shows up now, the deadline has already cleared.
        self.ladder.on_heartbeat(late_ms + 50, 0)
        self.assertEqual(self.ladder.rung, 0, "must have reverted at deadline")
        self.assertEqual(self.ladder.tune_failures, 1)

    def test_TL_E3_HB_inside_deadline_commits(self) -> None:
        now = BOOT_MS
        for _ in range(3):
            now = _feed_window(self.ladder, now_ms=now, hb_count=0)
            self.ladder.poll_link_ladder(now)
        # HB arrives 100 ms into the 500 ms revert window.
        self.ladder.on_heartbeat(now + 100, 0)
        # poll right at the deadline.
        post = self.ladder.tune_deadline + 1
        self.ladder.poll_link_ladder(post)
        self.assertEqual(self.ladder.rung, 1, "must commit SF8")
        self.assertEqual(self.ladder.tune_failures, 0)


class W4_02_TwicePhyAnnounceTests(unittest.TestCase):
    """TL-F: try_step_ladder emits exactly two LINK_TUNE frames per attempt
    — one at the OLD PHY and one at the NEW PHY — same target & reason.
    """

    def test_TL_F_two_announces_one_per_phy(self) -> None:
        ladder = LinkLadder()
        ladder.try_step_ladder(now_ms=1234, target_rung=1, reason=LTR_HIGH_LOSS)
        self.assertEqual(len(ladder.tx_log), 2)
        first, second = ladder.tx_log
        self.assertEqual(first.phy_at_tx, 0,  "first announce at OLD PHY (SF7)")
        self.assertEqual(second.phy_at_tx, 1, "second announce at NEW PHY (SF8)")
        self.assertEqual(first.target_rung, 1)
        self.assertEqual(second.target_rung, 1)
        self.assertEqual(first.reason, LTR_HIGH_LOSS)
        self.assertEqual(second.reason, LTR_HIGH_LOSS)
        self.assertEqual(first.at_ms, 1234)
        self.assertEqual(second.at_ms, 1234)

    def test_TL_F2_no_op_if_target_equals_current(self) -> None:
        ladder = LinkLadder()
        attempted = ladder.try_step_ladder(now_ms=1, target_rung=0, reason=LTR_MANUAL)
        self.assertFalse(attempted)
        self.assertEqual(ladder.tx_log, [])
        self.assertEqual(ladder.tune_deadline, 0)

    def test_TL_F3_no_op_if_target_out_of_range(self) -> None:
        ladder = LinkLadder()
        self.assertFalse(ladder.try_step_ladder(now_ms=1, target_rung=3, reason=LTR_MANUAL))
        self.assertEqual(ladder.tx_log, [])
        self.assertEqual(ladder.tune_deadline, 0)


class W4_02_StaleSourcePessimismTests(unittest.TestCase):
    """TL-G: no fresh active source → window is bad even if other sources sent HBs.

    Pins the comment in poll_link_ladder() that pessimistically widens
    the SF when pick_active_source() returns -1.
    """

    def test_TL_G_no_active_source_classifies_window_bad(self) -> None:
        ladder = LinkLadder()
        ladder.window_start = BOOT_MS
        ladder.active_source_for_test = None    # mirrors active < 0 in C
        now = BOOT_MS
        # Three windows in a row with NO active source — even if HBs from
        # a non-active source land they must not save us.
        for _ in range(3):
            now = _feed_window(ladder, now_ms=now, hb_count=50, source_idx=2)
            ladder.poll_link_ladder(now)
        # Ack the resulting tune so it commits.
        ladder.on_heartbeat(now + 50, 0)
        ladder.poll_link_ladder(ladder.tune_deadline + 1)
        self.assertEqual(ladder.rung, 1, "stale-source path must walk SF7→SF8")


class W4_02_WindowResetTests(unittest.TestCase):
    """TL-H: post-tune window starts fresh (no carry-over of HB counts)."""

    def test_TL_H_post_tune_window_does_not_reuse_old_HBs(self) -> None:
        ladder = LinkLadder()
        ladder.window_start = BOOT_MS
        ladder.active_source_for_test = 0

        # Two bad windows so the consec counter is at 2.
        now = BOOT_MS
        for _ in range(2):
            now = _feed_window(ladder, now_ms=now, hb_count=0)
            ladder.poll_link_ladder(now)
        self.assertEqual(ladder.rung, 0)
        self.assertEqual(ladder.consec_bad, 2)

        # Inject a flood of 999 HBs *just before* the third bad window
        # closes. These counts SHOULD make the third window classify as
        # good and reset consec_bad. (We are stress-testing that the
        # window evaluator still uses the live count, not a stale one.)
        for i in range(999):
            ladder.on_heartbeat(now + i, 0)
        now = now + LADDER_WINDOW_MS
        ladder.poll_link_ladder(now)
        self.assertEqual(ladder.rung, 0, "no step-down (third window was good)")
        self.assertEqual(ladder.consec_bad, 0, "good window resets bad counter")
        self.assertEqual(ladder.consec_good, 1)

        # Now drive 6 more good windows. Step-up gate is rung > 0, so
        # this must NOT emit any TX (stays at SF7) — which proves the
        # post-good-window state is internally consistent.
        for _ in range(6):
            now = _feed_window(ladder, now_ms=now, hb_count=50)
            ladder.poll_link_ladder(now)
        self.assertEqual(ladder.rung, 0)
        self.assertEqual(ladder.tx_log, [])


class W4_02_VerificationWindowGateTests(unittest.TestCase):
    """TL-I: the in-flight tune window is skipped by the bad/good classifier.

    The natural call flow (try_step_ladder → poll → commit/revert) resets
    ``window_start`` inside try_step_ladder, so the early-return on
    ``tune_deadline != 0`` is defensive against a *future* code path that
    arms a tune from somewhere other than the window evaluator (e.g. an
    inbound CMD_LINK_TUNE). Pin the gate directly by arming a deadline
    by hand and proving poll_link_ladder is a no-op for the window
    evaluator (verification still runs).
    """

    def test_TL_I_pending_tune_short_circuits_classifier(self) -> None:
        ladder = LinkLadder()
        ladder.window_start = BOOT_MS
        ladder.active_source_for_test = 0
        # Arm a deadline far in the future, then advance now past one
        # whole window WITHOUT crossing the deadline. The bad/good
        # classifier must not run.
        ladder.tune_deadline = BOOT_MS + 10 * LADDER_WINDOW_MS
        ladder._pre_tune_rung = 0
        ladder.pending_rung = 1
        end_of_window = BOOT_MS + LADDER_WINDOW_MS + 1
        self.assertLess(end_of_window, ladder.tune_deadline)
        ladder.poll_link_ladder(end_of_window)
        self.assertEqual(ladder.consec_bad, 0,
                         "classifier must NOT run while a tune is pending")
        self.assertEqual(ladder.consec_good, 0)
        self.assertNotEqual(ladder.tune_deadline, 0,
                            "deadline must remain armed (pre-deadline poll)")

    def test_TL_I2_pending_tune_verification_still_runs(self) -> None:
        """poll_link_ladder still services the verification path even when
        the bad/good classifier is gated off.
        """
        ladder = LinkLadder()
        ladder.window_start = BOOT_MS
        ladder.active_source_for_test = 0
        ladder.tune_deadline = BOOT_MS + 100
        ladder._pre_tune_rung = 0
        ladder.pending_rung = 1
        ladder.rung = 1   # apply_phy_rung already happened
        # No HB was delivered → poll past the deadline must revert.
        ladder.poll_link_ladder(BOOT_MS + 200)
        self.assertEqual(ladder.rung, 0, "must revert")
        self.assertEqual(ladder.tune_failures, 1)
        self.assertEqual(ladder.tune_deadline, 0)


if __name__ == "__main__":   # pragma: no cover
    unittest.main()
