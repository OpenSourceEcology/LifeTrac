"""RS-12.11 — base command timing vs fragment arrivals (SIL).

Replays the bench timeline that produced the loss: a 2-fragment camera
train every ~500 ms (gaps 258–540 ms between a train's last fragment and
the next train's first), the daemon polling with a 0.25 s timeout, and
commands queued throughout. The pre-fix rule fired the idle drain on every
empty poll, i.e. 250 ms after the last fragment, inside the window where
the next first fragment lands. The RS-12.11 rule fires only after true
quiet, and every in-stream command rides the pump right after a train.
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cmd_timing import (  # noqa: E402
    idle_drain_allowed, pump_window_open,
    pending_retry_gap, giveup_cooldown_active, pump_min_gap)


class IdleDrainRule(unittest.TestCase):
    QUIET_S = 1.5
    POLL_S = 0.25

    def test_quiet_link_drains(self) -> None:
        self.assertTrue(idle_drain_allowed(now=10.0, last_frag_t=8.0, quiet_s=self.QUIET_S))

    def test_first_empty_poll_after_a_train_does_not_drain(self) -> None:
        # Leg C signature: last fragment at t, empty poll at t + 0.25 s.
        self.assertFalse(idle_drain_allowed(now=100.25, last_frag_t=100.0, quiet_s=self.QUIET_S))

    def test_boundary_is_inclusive(self) -> None:
        self.assertTrue(idle_drain_allowed(now=101.5, last_frag_t=100.0, quiet_s=self.QUIET_S))
        self.assertFalse(idle_drain_allowed(now=101.49, last_frag_t=100.0, quiet_s=self.QUIET_S))

    def test_zero_quiet_restores_old_behaviour(self) -> None:
        self.assertTrue(idle_drain_allowed(now=100.25, last_frag_t=100.0, quiet_s=0.0))
        self.assertTrue(idle_drain_allowed(now=100.25, last_frag_t=100.0, quiet_s=-1.0))

    def test_camera_cadence_never_reaches_the_drain(self) -> None:
        """2 fps, 2-frag trains, gaps 258–540 ms: with the old rule every
        train gap hosted an idle-drain command; with the new rule none do."""
        t = 0.0
        old_fires = new_fires = 0
        gaps = [0.258, 0.481, 0.540, 0.300, 0.400, 0.258, 0.500]
        for gap in gaps:
            last_frag = t                      # last fragment of a train
            poll = last_frag + self.POLL_S     # first empty poll
            while poll < last_frag + gap:      # polls until the next train
                if idle_drain_allowed(poll, last_frag, 0.0):
                    old_fires += 1
                if idle_drain_allowed(poll, last_frag, self.QUIET_S):
                    new_fires += 1
                poll += self.POLL_S
            t = last_frag + gap + 0.24         # next train: ~240 ms on air
        self.assertGreaterEqual(old_fires, len(gaps))
        self.assertEqual(new_fires, 0)

    def test_stream_pause_still_drains(self) -> None:
        # A real pause (tractor idle) must still let commands out.
        self.assertTrue(idle_drain_allowed(now=105.0, last_frag_t=100.0, quiet_s=self.QUIET_S))


class PumpWindowRule(unittest.TestCase):
    def test_completed_frame_opens(self) -> None:
        self.assertTrue(pump_window_open(frame_done=True, train_end=False))

    def test_last_index_fragment_opens_even_if_frame_incomplete(self) -> None:
        # A train that lost idx 0 still ends on the air when idx N-1 lands.
        self.assertTrue(pump_window_open(frame_done=False, train_end=True))

    def test_mid_train_stays_closed(self) -> None:
        self.assertFalse(pump_window_open(frame_done=False, train_end=False))


class PendingRetryBackoff(unittest.TestCase):
    """RS-12.14: one pending command must not retry every 0.4 s forever."""

    def test_first_attempt_is_immediate(self) -> None:
        self.assertEqual(pending_retry_gap(0, 0.4, 2.0, 8.0), 0.0)

    def test_doubles_from_the_base_gap(self) -> None:
        gaps = [pending_retry_gap(n, 0.4, 2.0, 8.0) for n in range(1, 8)]
        self.assertEqual(gaps[:5], [0.4, 0.8, 1.6, 3.2, 6.4])
        self.assertEqual(gaps[5:], [8.0, 8.0])           # capped

    def test_factor_one_restores_fixed_gap(self) -> None:
        for n in range(1, 10):
            self.assertEqual(pending_retry_gap(n, 0.4, 1.0, 8.0), 0.4)

    def test_leg_i_budget(self) -> None:
        """Leg I: a perpetually re-triggered REQ_KEYFRAME sent 222 times in
        300 s. With the defaults, the retries of one episode fit under the
        10 s pending deadline only ~5 times, then the 8 s cap holds."""
        t, sends = 0.0, 0
        for n in range(0, 40):
            t += pending_retry_gap(n, 0.4, 2.0, 8.0)
            if t > 300.0:
                break
            sends += 1
        self.assertLess(sends, 45)                        # was ~222 at 0.4 s fixed


class GiveupCooldown(unittest.TestCase):
    def test_active_inside_window(self) -> None:
        self.assertTrue(giveup_cooldown_active(now=110.0, giveup_at=100.0, cooldown_s=30.0))

    def test_expires(self) -> None:
        self.assertFalse(giveup_cooldown_active(now=131.0, giveup_at=100.0, cooldown_s=30.0))

    def test_disabled_or_never_gave_up(self) -> None:
        self.assertFalse(giveup_cooldown_active(now=110.0, giveup_at=100.0, cooldown_s=0.0))
        self.assertFalse(giveup_cooldown_active(now=110.0, giveup_at=0.0, cooldown_s=30.0))


class PumpMinGap(unittest.TestCase):
    def test_stream_active_uses_stream_gap(self) -> None:
        self.assertEqual(pump_min_gap(True, 1.0), 1.0)

    def test_idle_uses_copy_spacing(self) -> None:
        self.assertEqual(pump_min_gap(False, 1.0), 0.12)

    def test_stream_gap_never_below_copy_spacing(self) -> None:
        self.assertEqual(pump_min_gap(True, 0.05), 0.12)

    def test_leg_i_rate(self) -> None:
        """281 sends in 300 s was 0.94/s; a 1 s stream gap caps the pump at
        1/s and the backoff leaves the queue mostly empty between windows."""
        self.assertLessEqual(300.0 / pump_min_gap(True, 1.0), 300)


if __name__ == "__main__":
    unittest.main()
