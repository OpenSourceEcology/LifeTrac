import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_BASE_STATION = os.path.dirname(_HERE)
_FIRMWARE_X8 = os.path.abspath(os.path.join(_BASE_STATION, "..", "firmware", "tractor_x8"))
_X8_HELPER = os.path.abspath(os.path.join(_BASE_STATION, "..", "firmware", "x8_lora_bootloader_helper"))
for p in (_BASE_STATION, _FIRMWARE_X8, _X8_HELPER):
    if p not in sys.path:
        sys.path.insert(0, p)

from lora_proto import (
    PHY_IMAGE_BW250,
    PHY_IMAGE_BW500,
    max_image_fragment_body,
    pack_image_fragments,
    pack_image_fragments_v2,
)
from image_tx_daemon import AirtimeBudget, _PendingFrame
from image_rx_daemon import KeyframeRequester


class TestImageTxRxOptimization(unittest.TestCase):
    def test_max_image_fragment_body(self):
        # Body max is 247 B; at 170 ms cap on BW250 SF7, 200 B body fits (ToA ~164 ms)
        body_size = max_image_fragment_body(PHY_IMAGE_BW250, max_air_ms=170.0)
        self.assertGreaterEqual(body_size, 200)
        self.assertLessEqual(body_size, 247)

    def test_pack_image_fragments_v1_and_v2(self):
        payload = b"X" * 500
        frags_v1 = pack_image_fragments(payload, frag_seq=5)
        self.assertGreater(len(frags_v1), 1)
        self.assertEqual(frags_v1[0][0], 0xFE)  # magic

        frags_v2 = pack_image_fragments_v2(payload, frag_seq=5, copies=2)
        self.assertEqual(len(frags_v2), len(frags_v1) * 2)
        self.assertEqual(frags_v2[0][0], 0xFD)  # magic v2

    def test_airtime_budget(self):
        budget = AirtimeBudget(budget_us=400_000, window_s=1.0)
        # Record 300ms ToA
        budget.record(300_000)
        import threading
        stop = threading.Event()
        # 50ms fits (300ms + 50ms <= 400ms)
        self.assertTrue(budget.admit(50_000, stop))
        # 150ms does not fit right now (300ms + 150ms > 400ms)
        # Check _used()
        self.assertEqual(budget._used(budget._events[0][0]), 300_000)

    def test_keyframe_requester(self):
        published = []

        class MockClient:
            def publish(self, topic, payload, qos=0):
                published.append((topic, payload))

        client = MockClient()
        kf = KeyframeRequester(lambda: client, min_interval_s=1.0)
        kf.poke("test_reason")
        self.assertEqual(len(published), 1)
        self.assertEqual(published[0][0], "lifetrac/v25/cmd/req_keyframe")

        # Second poke within rate-limit interval should be suppressed
        kf.poke("test_reason_2")
        self.assertEqual(len(published), 1)


if __name__ == "__main__":
    unittest.main()
