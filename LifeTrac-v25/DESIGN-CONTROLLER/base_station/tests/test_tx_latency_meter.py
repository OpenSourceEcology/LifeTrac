"""Unit tests for the S1.2 host-side TX latency meter.

Pure-Python: the meter takes wall-clock ms as parameters, so tests can use a
deterministic synthetic clock and never touch real time.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

# Mirror the import pattern used elsewhere in this test suite: the
# base_station module lives one directory up from tests/.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from tx_latency_meter import TxLatencyMeter, _percentile  # noqa: E402


class PercentileHelperTests(unittest.TestCase):
    def test_empty_returns_none(self) -> None:
        self.assertIsNone(_percentile([], 50))

    def test_min_and_max_bounds(self) -> None:
        self.assertEqual(_percentile([1.0, 2.0, 3.0], 0), 1.0)
        self.assertEqual(_percentile([1.0, 2.0, 3.0], 100), 3.0)

    def test_median_nearest_rank(self) -> None:
        # 5 samples → 50% bucket index = ceil(0.5*5) - 1 = 1 → value 2.0
        self.assertEqual(_percentile([5.0, 1.0, 2.0, 4.0, 3.0], 50), 2.0)

    def test_p99_picks_top(self) -> None:
        samples = [float(i) for i in range(100)]
        # idx = int(0.99*100) - 1 = 98 → 98.0
        self.assertEqual(_percentile(samples, 99), 98.0)


class TxLatencyMeterTests(unittest.TestCase):
    def setUp(self) -> None:
        self.meter = TxLatencyMeter()

    def test_snapshot_empty_until_first_done(self) -> None:
        self.assertEqual(self.meter.snapshot(), {})
        # Even after enqueue/dequeue but no done, snapshot shows queue_age only.
        tok = self.meter.mark_enqueue(prio=0, now_ms=100)
        self.meter.mark_dequeue(tok, now_ms=105)
        snap = self.meter.snapshot()
        self.assertIn(0, snap)
        self.assertEqual(snap[0]["queue_age_p50_ms"], 5.0)
        self.assertEqual(snap[0]["tx_done_p50_ms"], None)
        self.assertEqual(snap[0]["sample_count"], 0)

    def test_full_lifecycle_records_all_three_signals(self) -> None:
        tok = self.meter.mark_enqueue(prio=0, now_ms=0)
        self.meter.mark_dequeue(tok, now_ms=4)
        # actual on-air = 30 - 4 = 26 ms; predicted 25 → delta +1
        self.meter.mark_done(tok, now_ms=30, predicted_toa_ms=25.0)
        snap = self.meter.snapshot()
        self.assertEqual(snap[0]["queue_age_p50_ms"], 4.0)
        self.assertEqual(snap[0]["tx_done_p50_ms"], 30.0)
        self.assertEqual(snap[0]["toa_delta_p50_ms"], 1.0)
        self.assertEqual(snap[0]["sample_count"], 1)

    def test_per_class_isolation(self) -> None:
        # P0 fast and on-budget; P3 slow and over-budget.
        for i in range(3):
            t = self.meter.mark_enqueue(prio=0, now_ms=i * 10)
            self.meter.mark_dequeue(t, now_ms=i * 10 + 1)
            self.meter.mark_done(t, now_ms=i * 10 + 10, predicted_toa_ms=9.0)
        for i in range(3):
            t = self.meter.mark_enqueue(prio=3, now_ms=1000 + i * 10)
            self.meter.mark_dequeue(t, now_ms=1000 + i * 10 + 50)
            self.meter.mark_done(t, now_ms=1000 + i * 10 + 200, predicted_toa_ms=100.0)
        snap = self.meter.snapshot()
        self.assertEqual(snap[0]["tx_done_p50_ms"], 10.0)
        self.assertEqual(snap[0]["queue_age_p50_ms"], 1.0)
        self.assertEqual(snap[3]["tx_done_p50_ms"], 200.0)
        self.assertEqual(snap[3]["queue_age_p50_ms"], 50.0)
        # P3 actual on-air = 150 ms, predicted 100 → +50 ms delta (S1 gate
        # would flag this as a prediction-model failure on P3).
        self.assertEqual(snap[3]["toa_delta_p50_ms"], 50.0)

    def test_p0_start_delay_p99_pickup(self) -> None:
        # 98 fast P0 enqueues (1 ms wait) and 2 slow ones (500 ms wait, the
        # exact gate-busting scenario S1.2 needs to surface). At 100 samples
        # nearest-rank p99 picks index 98 — must land on the slow tail.
        for i in range(98):
            t = self.meter.mark_enqueue(prio=0, now_ms=i * 10)
            self.meter.mark_dequeue(t, now_ms=i * 10 + 1)
            self.meter.mark_done(t, now_ms=i * 10 + 5)
        for j in range(2):
            t = self.meter.mark_enqueue(prio=0, now_ms=10_000 + j * 1000)
            self.meter.mark_dequeue(t, now_ms=10_000 + j * 1000 + 500)
            self.meter.mark_done(t, now_ms=10_000 + j * 1000 + 510)
        snap = self.meter.snapshot()
        self.assertEqual(snap[0]["queue_age_p99_ms"], 500.0)
        self.assertEqual(snap[0]["queue_age_p50_ms"], 1.0)

    def test_missing_done_does_not_corrupt_other_frames(self) -> None:
        # Open a token that we'll abandon.
        abandoned = self.meter.mark_enqueue(prio=0, now_ms=0)
        # Healthy frame on a different class still records cleanly.
        good = self.meter.mark_enqueue(prio=1, now_ms=10)
        self.meter.mark_dequeue(good, now_ms=12)
        self.meter.mark_done(good, now_ms=20)
        snap = self.meter.snapshot()
        self.assertNotIn(0, snap)  # abandoned never reached mark_done or _dequeue
        self.assertEqual(snap[1]["tx_done_p50_ms"], 10.0)
        # Sanity: stale token can still be closed without affecting the
        # healthy class.
        self.meter.mark_done(abandoned, now_ms=999)
        snap2 = self.meter.snapshot()
        self.assertEqual(snap2[0]["tx_done_p50_ms"], 999.0)
        self.assertEqual(snap2[1]["tx_done_p50_ms"], 10.0)

    def test_unknown_token_is_noop(self) -> None:
        # Should not raise, should not produce samples.
        self.meter.mark_dequeue(token=9999, now_ms=100)
        self.meter.mark_done(token=9999, now_ms=100)
        self.assertEqual(self.meter.snapshot(), {})

    def test_inflight_leak_guard_bounded(self) -> None:
        # Open way more than _MAX_INFLIGHT without ever closing them.
        for i in range(TxLatencyMeter._MAX_INFLIGHT * 3):
            self.meter.mark_enqueue(prio=2, now_ms=i)
        # In-flight dict is capped.
        self.assertLessEqual(len(self.meter._inflight), TxLatencyMeter._MAX_INFLIGHT)

    def test_reset_clears_everything(self) -> None:
        t = self.meter.mark_enqueue(prio=0, now_ms=0)
        self.meter.mark_dequeue(t, now_ms=5)
        self.meter.mark_done(t, now_ms=20)
        self.assertNotEqual(self.meter.snapshot(), {})
        self.meter.reset()
        self.assertEqual(self.meter.snapshot(), {})
        self.assertEqual(len(self.meter._inflight), 0)

    def test_rolling_window_caps_memory(self) -> None:
        small = TxLatencyMeter(window=5)
        for i in range(20):
            t = small.mark_enqueue(prio=0, now_ms=i * 10)
            small.mark_dequeue(t, now_ms=i * 10 + 1)
            small.mark_done(t, now_ms=i * 10 + 2)
        snap = small.snapshot()
        self.assertEqual(snap[0]["sample_count"], 5)


class BridgeIntegrationTests(unittest.TestCase):
    """Smoke-test that Bridge owns a meter and the heap entry shape did not
    drift. We do NOT spin up a real Bridge (that path needs serial + crypto
    fixtures); we just import the module and verify the symbol exists in the
    expected place so a future refactor that drops the wiring fails loudly.
    """

    def test_bridge_module_exposes_tx_latency_meter_symbol(self) -> None:
        import lora_bridge  # noqa: WPS433 — lazy import to keep test isolated

        self.assertTrue(
            hasattr(lora_bridge, "TxLatencyMeter"),
            "lora_bridge must re-import TxLatencyMeter so Bridge instances "
            "have a `.tx_latency` attribute (S1.2 wiring).",
        )


if __name__ == "__main__":
    unittest.main()
