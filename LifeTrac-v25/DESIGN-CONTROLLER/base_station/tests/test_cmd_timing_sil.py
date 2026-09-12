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

from cmd_timing import idle_drain_allowed, pump_window_open  # noqa: E402


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


if __name__ == "__main__":
    unittest.main()
