"""RS-11.4 — smooth airtime pacing in the tractor's AirtimeBudget.

The pure token bucket bursts to the budget and then stalls hard. On air that
meant 9 fragments back-to-back then a ~161 ms block, opening a ~60 ms dead-air
hole mid-train and costing fragment index 10 in ~59% of trains whenever the
preceding idle exceeded the 1 s window. Confirmed causally: moving the budget
930_000 -> 600_000 us took index 10 from 59.1% to 7.5%.

Smooth pacing holds each fragment to the spacing the duty target already
implies (ToA x window / budget), which removes the hole WITHOUT changing the
duty. The rolling-window check is retained as the hard regulatory backstop —
these tests pin that it still binds, because pacing is an optimisation and the
window is the guarantee.
"""

import os
import sys
import threading
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "firmware", "tractor_x8"))

DTS_BUDGET_US = 930_000
FRAG_TOA_US = 99_904          # 255 B at SF7/BW500/CR4-5, matches on-air exactly


def _fresh(monkey_env=None, **kw):
    """Import a private copy of the module so env changes take effect."""
    for k, v in (monkey_env or {}).items():
        if v is None:
            os.environ.pop(k, None)
        else:
            os.environ[k] = v
    for mod in list(sys.modules):
        if mod == "image_tx_daemon":
            del sys.modules[mod]
    import image_tx_daemon as m
    return m.AirtimeBudget(**kw)


class DutyIsNeverExceededTests(unittest.TestCase):
    """The regulatory property. Pacing may change WHEN we send; it must never
    let us send MORE."""

    def test_window_still_refuses_an_over_budget_fragment(self) -> None:
        b = _fresh({"LIFETRAC_AIRTIME_PACING": "bucket"},
                   budget_us=DTS_BUDGET_US, window_s=1.0)
        # Nine fragments fit 930 ms; a tenth does not.
        for _ in range(9):
            b.record(FRAG_TOA_US)
        self.assertGreater(9 * FRAG_TOA_US + FRAG_TOA_US, DTS_BUDGET_US,
                           "premise: 10 fragments must not fit the budget")
        stop = threading.Event()
        stop.set()                       # do not actually block the test
        self.assertFalse(b.admit(FRAG_TOA_US, stop),
                         "an over-budget fragment must be refused, not admitted")

    def test_smooth_mode_keeps_the_same_hard_window(self) -> None:
        b = _fresh({"LIFETRAC_AIRTIME_PACING": "smooth"},
                   budget_us=DTS_BUDGET_US, window_s=1.0)
        for _ in range(9):
            b.record(FRAG_TOA_US)
        stop = threading.Event()
        stop.set()
        self.assertFalse(b.admit(FRAG_TOA_US, stop),
                         "smooth pacing must not weaken the regulatory backstop")


class SpacingArithmeticTests(unittest.TestCase):

    def test_paced_duty_sits_BELOW_the_budget_not_on_it(self) -> None:
        """The defect that regressed the first implementation: pacing to
        exactly the budget leaves used == budget, so the hard window check
        fires on jitter and its stall compounds with the pacing delay. Headroom
        must keep steady-state duty strictly under the budget so the backstop
        stays a backstop."""
        import image_tx_daemon as m
        spacing_s = FRAG_TOA_US * 1.0 / (DTS_BUDGET_US * m._PACING_HEADROOM)
        duty_us_per_s = (1.0 / spacing_s) * FRAG_TOA_US
        self.assertLess(duty_us_per_s, DTS_BUDGET_US,
                        "paced duty must be strictly below the budget")
        self.assertLess(m._PACING_HEADROOM, 1.0)
        self.assertGreater(m._PACING_HEADROOM, 0.5, "headroom must not throttle us to nothing")

    def test_smooth_does_not_cost_train_time_versus_the_bucket(self) -> None:
        """With headroom the train is no longer FASTER than the bucket, only
        level with it — the win is removing the mid-train hole, not throughput.
        Recorded so nobody later 'optimises' the headroom away expecting a
        speedup that was never there."""
        import image_tx_daemon as m
        spacing_ms = FRAG_TOA_US * 1.0 / (DTS_BUDGET_US * m._PACING_HEADROOM) * 1000.0
        smooth_ms = 13 * spacing_ms
        bucket_ms = 9 * 104.9 + 161.0 + 4 * 104.9
        self.assertLessEqual(smooth_ms, bucket_ms * 1.05,
                             "smooth must not be materially slower than the bucket")

    def test_first_fragment_after_a_long_idle_is_not_delayed(self) -> None:
        """A drained window means the train may start immediately — otherwise
        every train would pay a spacing penalty it does not owe."""
        b = _fresh({"LIFETRAC_AIRTIME_PACING": "smooth"},
                   budget_us=DTS_BUDGET_US, window_s=1.0)
        stop = threading.Event()
        t0 = __import__("time").monotonic()
        self.assertTrue(b.admit(FRAG_TOA_US, stop))
        self.assertLess(__import__("time").monotonic() - t0, 0.05,
                        "first admit on a fresh budget must be immediate")


class ModeSelectionTests(unittest.TestCase):

    def test_smooth_is_the_default_after_on_air_verification(self) -> None:
        """Verified 2026-07-30, n=2 per side at 0.4 fps plus a saturated check:
        the idx-10 notch went 59.1/23.9% -> 2.15/2.15% with total loss
        unchanged, and at saturation goodput cost only 2.7% while loss fell
        3.73% -> 2.24%. The headroom-less first attempt is why this test
        exists — do not flip the default without re-running that A/B."""
        b = _fresh({"LIFETRAC_AIRTIME_PACING": None},
                   budget_us=DTS_BUDGET_US, window_s=1.0)
        self.assertTrue(b._smooth)

    def test_bucket_mode_is_reachable_for_ab_against_recorded_evidence(self) -> None:
        b = _fresh({"LIFETRAC_AIRTIME_PACING": "bucket"},
                   budget_us=DTS_BUDGET_US, window_s=1.0)
        self.assertFalse(b._smooth)


if __name__ == "__main__":
    unittest.main()
