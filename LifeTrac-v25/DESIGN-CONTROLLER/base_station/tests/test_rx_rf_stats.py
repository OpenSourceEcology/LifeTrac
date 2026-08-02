"""RS-11.6 — healthy-frame RF window in image_rx_daemon.

The RS-11.5 closure flagged this as the missing instrument: only corrupt
packets carried visible RSSI/SNR (crc_dump), so the healthy population's
SNR margin was never measurable. `_note_frag_rf()` accumulates the
window; the 10 s stats block summarizes (min/med/max) and resets. Same
stub-binding pattern as the RS-11.1 classifier tests: the method touches
only `self` attributes.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import image_rx_daemon as rxd  # noqa: E402


class _Stub:
    pass


class RfWindowTests(unittest.TestCase):
    def test_accumulates_both_axes(self) -> None:
        s = _Stub()
        note = rxd.ImageRxDaemon._note_frag_rf
        note(s, -60, 8.5)
        note(s, -72, 3.0)
        note(s, -55, 9.75)
        self.assertEqual(s._rf_rssi, [-60, -72, -55])
        self.assertEqual(s._rf_snr, [8.5, 3.0, 9.75])

    def test_none_values_are_skipped_not_crashed(self) -> None:
        """Pre-F8 firmware or a short URC can yield None on either axis;
        the window must simply skip them."""
        s = _Stub()
        note = rxd.ImageRxDaemon._note_frag_rf
        note(s, None, None)
        note(s, -80, None)
        note(s, None, 2.0)
        self.assertEqual(s._rf_rssi, [-80])
        self.assertEqual(s._rf_snr, [2.0])

    def test_lazy_init_matches_classifier_state_pattern(self) -> None:
        s = _Stub()
        rxd.ImageRxDaemon._note_frag_rf(s, -90, 1.0)
        self.assertTrue(hasattr(s, "_rf_rssi"))
        self.assertTrue(hasattr(s, "_rf_snr"))

    def test_single_axis_windows_accumulate_independently(self) -> None:
        """PR #95 review case: the axes can be independently absent; an
        SNR-only window must still carry its samples (the stats block
        gates on EITHER list and resets both)."""
        s = _Stub()
        note = rxd.ImageRxDaemon._note_frag_rf
        note(s, None, 4.0)
        note(s, None, 6.0)
        self.assertEqual(s._rf_rssi, [])
        self.assertEqual(s._rf_snr, [4.0, 6.0])


if __name__ == "__main__":
    unittest.main()
