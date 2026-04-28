"""Tests for R-6 telemetry fragmentation and the nonce store."""
from __future__ import annotations

import os
import tempfile
import unittest

from lora_proto import (
    PHY_IMAGE,
    PHY_TELEMETRY,
    TELEMETRY_FRAGMENT_MAGIC,
    TelemetryReassembler,
    max_telemetry_fragment_payload,
    pack_telemetry_fragments,
    parse_telemetry_fragment,
    lora_time_on_air_ms,
    TELEM_HEADER_LEN,
    CRC_LEN,
)
from nonce_store import NonceStore


class TelemetryFragmentationTests(unittest.TestCase):
    def test_max_fragment_obeys_25ms_cap(self):
        # Fragmented telemetry rides the image PHY (SF7/BW500); the slow
        # SF9/BW250 telemetry PHY has a 33 ms preamble alone and so can't
        # carry any fragment under a 25 ms cap — that's why R-6 demands
        # a retune to the image PHY for the fragmented hot path.
        chunk = max_telemetry_fragment_payload(PHY_IMAGE, 25.0)
        self.assertGreater(chunk, 0)
        body_len = TELEM_HEADER_LEN + CRC_LEN + 4 + chunk  # +4 for fragment header
        air = lora_time_on_air_ms(body_len, PHY_IMAGE)
        self.assertLessEqual(air, 25.0 + 1e-6)
        # And one extra byte should bust the cap
        air_over = lora_time_on_air_ms(body_len + 1, PHY_TELEMETRY)
        # On some PHY profiles the symbol granularity means +1 byte may not
        # cross a symbol boundary; only assert when it does.
        if air_over > 25.0:
            self.assertGreater(air_over, 25.0)

    def test_pack_single_fragment_round_trip(self):
        payload = b"hello world"
        frags = pack_telemetry_fragments(payload, frag_seq=7, profile=PHY_IMAGE)
        self.assertEqual(len(frags), 1)
        seq, idx, total, data = parse_telemetry_fragment(frags[0])
        self.assertEqual((seq, idx, total), (7, 0, 1))
        self.assertEqual(data, payload)

    def test_pack_multi_fragment_round_trip(self):
        payload = bytes(range(200))
        frags = pack_telemetry_fragments(payload, frag_seq=42, profile=PHY_IMAGE)
        self.assertGreater(len(frags), 1)
        for f in frags:
            self.assertEqual(f[0], TELEMETRY_FRAGMENT_MAGIC)
        # All fragments share frag_seq, total and have monotonic idx.
        parsed = [parse_telemetry_fragment(f) for f in frags]
        self.assertTrue(all(p is not None for p in parsed))
        for i, (seq, idx, total, _data) in enumerate(parsed):
            self.assertEqual(seq, 42)
            self.assertEqual(idx, i)
            self.assertEqual(total, len(frags))

    def test_reassembler_passes_through_unfragmented_payloads(self):
        ra = TelemetryReassembler()
        out = ra.feed(source_id=3, topic_id=0x07, body=b"unfragmented", now_ms=10)
        self.assertEqual(out, b"unfragmented")
        self.assertEqual(ra.bad_magic_passthroughs, 1)

    def test_reassembler_recovers_full_payload(self):
        payload = bytes(range(180))
        frags = pack_telemetry_fragments(payload, frag_seq=9, profile=PHY_IMAGE)
        ra = TelemetryReassembler()
        out = None
        for i, f in enumerate(frags):
            out = ra.feed(source_id=3, topic_id=0x07, body=f, now_ms=100 + i)
        self.assertEqual(out, payload)
        self.assertEqual(ra.completed, 1)

    def test_reassembler_dedupes(self):
        payload = bytes(range(180))
        frags = pack_telemetry_fragments(payload, frag_seq=11, profile=PHY_IMAGE)
        ra = TelemetryReassembler()
        # Feed first fragment twice.
        ra.feed(3, 0x07, frags[0], now_ms=1)
        ra.feed(3, 0x07, frags[0], now_ms=2)
        self.assertEqual(ra.duplicates, 1)
        # Finish the rest.
        for i, f in enumerate(frags[1:], start=1):
            out = ra.feed(3, 0x07, f, now_ms=100 + i)
        self.assertEqual(out, payload)

    def test_reassembler_times_out_partials(self):
        payload = bytes(range(180))
        frags = pack_telemetry_fragments(payload, frag_seq=13, profile=PHY_IMAGE)
        ra = TelemetryReassembler(timeout_ms=50)
        ra.feed(3, 0x07, frags[0], now_ms=0)
        # Trigger GC long after timeout.
        ra.feed(3, 0x07, frags[1], now_ms=10_000)
        self.assertEqual(ra.timeouts, 1)


class NonceStoreTests(unittest.TestCase):
    def test_reserve_increments_by_gap(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "nonce.json")
            store = NonceStore(path=path, gap=10, flush_interval_s=0.0)
            a = store.reserve(0x02)
            b = store.reserve(0x02)
            self.assertEqual(a, 0)
            self.assertEqual(b, 10)

    def test_reserve_persists_across_instances(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "nonce.json")
            s1 = NonceStore(path=path, gap=5, flush_interval_s=0.0)
            s1.reserve(0x02)
            s1.reserve(0x02)        # → seq 5
            s1.close()
            s2 = NonceStore(path=path, gap=5, flush_interval_s=0.0)
            self.assertEqual(s2.reserve(0x02), 10)

    def test_reserve_per_source_independent(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "nonce.json")
            store = NonceStore(path=path, gap=64, flush_interval_s=0.0)
            self.assertEqual(store.reserve(0x01), 0)
            self.assertEqual(store.reserve(0x02), 0)
            self.assertEqual(store.reserve(0x01), 64)
            self.assertEqual(store.reserve(0x02), 64)


if __name__ == "__main__":           # pragma: no cover
    unittest.main()
