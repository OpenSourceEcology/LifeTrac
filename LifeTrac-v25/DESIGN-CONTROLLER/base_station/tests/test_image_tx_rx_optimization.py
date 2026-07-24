import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_BASE_STATION = os.path.dirname(_HERE)
_FIRMWARE_X8 = os.path.abspath(os.path.join(_BASE_STATION, "..", "firmware", "tractor_x8"))
_X8_HELPER = os.path.abspath(os.path.join(_BASE_STATION, "..", "firmware", "x8_lora_bootloader_helper"))
for p in (_X8_HELPER, _FIRMWARE_X8, _BASE_STATION):
    # Unconditional re-insert: under `python -m unittest` the cwd (==
    # _BASE_STATION) is already on sys.path, so an `if not in` guard
    # would leave the X8 paths in front and the X8-side image_pipeline
    # package would shadow the base-station one (T3 collision).
    if p in sys.path:
        sys.path.remove(p)
    sys.path.insert(0, p)

from lora_proto import (
    PHY_IMAGE_BW250,
    PHY_IMAGE_BW500,
    max_image_fragment_body,
    pack_image_fragments,
    pack_image_fragments_v2,
    add_parity_fragments,
)
from image_pipeline.reassemble import FragmentReassembler
from image_pipeline.frame_format import TileDeltaFrame, TileBlob, encode_tile_delta_frame
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

    def test_parity_fragments_reconstruction(self):
        # Build a VALID multi-fragment TileDeltaFrame: three ~250 B tiles
        # -> ~780 B payload -> 4 fragments at the 203 B chunk. (Appending
        # raw padding to a valid frame would trip the parser's
        # trailing-bytes check after reconstruction — the frame must be
        # genuinely large, not padded.)
        big = bytes(range(250))
        frame = TileDeltaFrame(frame_kind=1, base_seq=42, grid_w=6, grid_h=4, tile_px=32,
                               changed_indices=[0, 1, 2],
                               tiles=[TileBlob(0, 0, 0, big),
                                      TileBlob(1, 1, 0, big),
                                      TileBlob(2, 2, 0, big)])
        payload = encode_tile_delta_frame(frame)
        frags = pack_image_fragments(payload, frag_seq=12, profile=PHY_IMAGE_BW250, max_air_ms=170.0)
        self.assertGreaterEqual(len(frags), 3)

        frags_with_parity = add_parity_fragments(frags, frag_seq=12, group_len=8)
        # Parity fragments appended after each group of up to 8
        self.assertGreater(len(frags_with_parity), len(frags))

        # Drop a NON-LAST data fragment (idx 1) and feed into reassembler
        ra = FragmentReassembler()
        completed = None
        for f in frags_with_parity:
            if f[0] == 0xFE and f[2] == 1:
                continue
            res = ra.feed(f)
            if res is not None:
                completed = res

        self.assertIsNotNone(completed)
        self.assertEqual(ra.stats.parity_reconstructions, 1)

    def test_parity_skips_short_last_fragment(self):
        # The frame's LAST fragment may be shorter than the XOR width;
        # reconstructing it would append padding the parser rejects
        # (2026-07-24 fix). The reassembler must SKIP it and leave the
        # partial pending (GC/keyframe-request recovers), never complete
        # a corrupt frame.
        big = bytes(range(250))
        frame = TileDeltaFrame(frame_kind=1, base_seq=43, grid_w=6, grid_h=4, tile_px=32,
                               changed_indices=[0, 1, 2],
                               tiles=[TileBlob(0, 0, 0, big),
                                      TileBlob(1, 1, 0, big),
                                      TileBlob(2, 2, 0, big)])
        payload = encode_tile_delta_frame(frame)
        frags = pack_image_fragments(payload, frag_seq=13, profile=PHY_IMAGE_BW250, max_air_ms=170.0)
        last_idx = len(frags) - 1
        self.assertLess(len(frags[last_idx]), len(frags[0]),
                        "test needs a genuinely short last fragment")
        frags_with_parity = add_parity_fragments(frags, frag_seq=13, group_len=8)
        ra = FragmentReassembler()
        completed = None
        for f in frags_with_parity:
            if f[0] == 0xFE and f[2] == last_idx:
                continue                      # lose the short LAST fragment
            res = ra.feed(f)
            if res is not None:
                completed = res
        self.assertIsNone(completed, "must not complete a padded/corrupt frame")
        self.assertEqual(ra.stats.parity_reconstructions, 0)
        self.assertEqual(ra.stats.decode_errors, 0)

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

    def test_rx_daemon_has_kf_req(self):
        from image_rx_daemon import ImageRxDaemon
        d = ImageRxDaemon(uart="loop://", baud="0", mqtt_host="127.0.0.1",
                          mqtt_port=1883, reassembler_timeout_ms=1500)
        self.assertTrue(hasattr(d, "_kf_req"))
        d._kf_req.poke("attribute-exists smoke")   # must not raise

    def test_codec_webp_rawstream(self):
        from image_pipeline.codec_decode import rewrap_webp, transcode_to_webp
        from image_pipeline.frame_format import CODEC_WEBP_RAWSTREAM
        import io
        from PIL import Image
        img = Image.new("RGB", (32, 32), color=(255, 0, 0))
        buf = io.BytesIO()
        img.save(buf, format="WEBP", quality=30)
        full_webp = buf.getvalue()
        # Strip container header (first 20 bytes)
        raw_bitstream = full_webp[20:]
        rewrapped = rewrap_webp(raw_bitstream)
        self.assertEqual(len(rewrapped), len(full_webp))
        decoded_img = Image.open(io.BytesIO(rewrapped))
        self.assertEqual(decoded_img.size, (32, 32))

        res = transcode_to_webp(CODEC_WEBP_RAWSTREAM, raw_bitstream, 32)
        self.assertEqual(res, rewrapped)


if __name__ == "__main__":
    unittest.main()
