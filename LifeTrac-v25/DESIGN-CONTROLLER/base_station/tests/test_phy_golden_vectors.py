import os
import sys
import unittest

# Ensure base_station directory is on sys.path
_HERE = os.path.dirname(os.path.abspath(__file__))
_BASE_STATION = os.path.dirname(_HERE)
if _BASE_STATION not in sys.path:
    sys.path.insert(0, _BASE_STATION)

from lora_proto import PHY_BY_NAME, lora_time_on_air_ms


class TestPhyGoldenVectors(unittest.TestCase):
    """On-air lengths & ToA measured from live TX_DONE_URC values.
    If an estimator or profile edit breaks these, the wire model has
    drifted from the silicon — do NOT relax the deltas (F1 postmortem)."""
    VECTORS = [
        # (profile, on_air_len_B, measured toa_us, source)
        ("image_bw250", 33, 35_968,  # 2026-05-28 tx_burst RFCO/TX_DONE
         "25 B body + 8 B hop hdr"),
        ("image_bw250", 49, 48_768,  # 2026-05-27 air-link-proven run
         "41 B body + 8 B hop hdr"),
    ]

    def test_estimator_matches_firmware_urc(self):
        for name, length, toa_us, note in self.VECTORS:
            est = lora_time_on_air_ms(length, PHY_BY_NAME[name]) * 1000
            self.assertAlmostEqual(est, toa_us, delta=1,
                                   msg=f"{name}/{length}B ({note})")


if __name__ == "__main__":
    unittest.main()
