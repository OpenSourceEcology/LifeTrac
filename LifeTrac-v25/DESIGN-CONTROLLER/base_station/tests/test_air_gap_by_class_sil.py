"""RS-12.17 — the air-gap per-class helper must be TOTAL.

Leg U (2026-09-15) lost the daemon's stats thread one frame into a 300 s
camera leg:

    File "/work/image_rx_daemon.py", line 1803, in _stats_worker
      for smp in samples:
    TypeError: 'NoneType' object is not iterable

The RS-11.1 `air_gap_by_class` block reused `samples` from the enclosing
scope while sitting inside the PHASE-TELEMETRY branch, so any window with
phase telemetry and no gap samples killed the thread. Nothing logged the
failure as an error afterwards — the daemon simply stopped emitting
`stats:` lines. Because `tools/rs12_leg_report.py` reads every figure on
its loss line from the LAST `stats:` line, the leg was then reported as
"loss 651/652 = 99.8% published=1" when it had in fact published 537 frames
with zero lock losses: a false catastrophe on the campaign's headline
metric.

These cases pin the helper's totality and its formatting so the crash
cannot return.
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from image_rx_daemon import ImageRxDaemon  # noqa: E402

parts = ImageRxDaemon._air_gap_by_class_parts


class AirGapByClassTotality(unittest.TestCase):
    """The exact shapes that crashed the thread."""

    def test_none_returns_empty(self) -> None:
        self.assertEqual(parts(None), [])

    def test_empty_returns_empty(self) -> None:
        self.assertEqual(parts([]), [])

    def test_none_does_not_raise(self) -> None:
        try:
            parts(None)
        except TypeError as exc:                      # pragma: no cover
            self.fail(f"the leg-U crash is back: {exc}")


class AirGapByClassFormatting(unittest.TestCase):
    def test_untagged_samples_default_to_seq(self) -> None:
        # (gap_us, len_b) with no class tag — the pre-RS-11.1 sample shape.
        self.assertEqual(parts([(1000, 10), (3000, 10)]),
                         ["seq: n=2 med=3.0ms"])

    def test_classes_report_in_fixed_order(self) -> None:
        samples = [(120_000, 10, "post_loss"),
                   (50_000, 10, "boundary"),
                   (1_000, 10, "seq"),
                   (3_000, 10, "seq")]
        self.assertEqual(parts(samples),
                         ["seq: n=2 med=3.0ms",
                          "boundary: n=1 med=50.0ms",
                          "post_loss: n=1 med=120.0ms"])

    def test_absent_classes_are_skipped(self) -> None:
        self.assertEqual(parts([(2_000, 10, "boundary")]),
                         ["boundary: n=1 med=2.0ms"])

    def test_unknown_class_is_ignored(self) -> None:
        """Unchanged from the inline version: only the three known classes
        are reported, so a new tag cannot silently widen the log line."""
        self.assertEqual(parts([(2_000, 10, "something_else")]), [])

    def test_median_uses_upper_middle_like_the_original(self) -> None:
        self.assertEqual(parts([(1_000, 1, "seq"), (2_000, 1, "seq"),
                                (9_000, 1, "seq")]),
                         ["seq: n=3 med=2.0ms"])


if __name__ == "__main__":
    unittest.main()
