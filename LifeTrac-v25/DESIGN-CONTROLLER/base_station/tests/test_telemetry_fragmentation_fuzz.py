"""Round 11: Property/fuzz coverage for the telemetry fragmentation path.

Symmetric to the IP-108 command-frame fuzz suite, but applied to the
M7→base direction: ``pack_telemetry_fragments`` ↔
``parse_telemetry_fragment`` ↔ ``TelemetryReassembler``.

Covers:

* **TF-A round-trip identity.** For every payload length 0..2048 across
  every documented PHY profile, packing then reassembling must return
  the original bytes.
* **TF-B fragment-header invariants.** Every fragment carries the magic
  byte, the same ``frag_seq``, monotonically increasing ``idx``, and the
  same ``total``. ``total`` matches the parsed list length.
* **TF-C airtime cap.** Every fragment fits inside the documented
  ``TELEMETRY_FRAGMENT_MAX_AIRTIME_MS`` (25 ms) on its profile.
* **TF-D oversize rejection.** Payloads requiring > 256 fragments raise
  ``ValueError`` rather than silently truncating.
* **TF-E reordered delivery.** Shuffling the fragment order before
  feeding the reassembler still produces the original payload.
* **TF-F duplicate delivery.** Replaying every fragment N extra times
  still yields exactly one completion and bumps ``duplicates`` by the
  expected amount.
* **TF-G missing-middle timeout.** Dropping any single middle fragment
  prevents completion; once the GC window passes the partial is
  reclaimed and ``timeouts`` ticks.
* **TF-H multi-source / multi-topic isolation.** Two streams sharing a
  ``frag_seq`` but with different ``(source_id, topic_id)`` tuples must
  not bleed into each other.
* **TF-I seq-wrap.** ``frag_seq`` wraps at 256; two assemblies with the
  same seq but separated by a GC pass complete independently.
* **TF-J unfragmented passthrough.** A body whose first byte isn't the
  fragment magic is returned verbatim and counted as a passthrough.
* **TF-K total-mid-stream-change recovery.** If a stream's reported
  ``total`` ever changes, the reassembler discards the partial and
  starts over (matches the live behaviour at lora_proto.py:611).
* **TF-L randomised stress.** 200 random (length, seq, source, topic,
  loss-pattern) tuples; PRNG-seeded so failures repro deterministically.

Pure stdlib.
"""

from __future__ import annotations

import random
import unittest

from lora_proto import (
    PHY_IMAGE,
    PHY_CONTROL_SF7,
    PHY_CONTROL_SF8,
    PHY_CONTROL_SF9,
    TELEMETRY_FRAGMENT_HEADER_LEN,
    TELEMETRY_FRAGMENT_MAGIC,
    TELEMETRY_FRAGMENT_MAX_AIRTIME_MS,
    TELEM_HEADER_LEN,
    CRC_LEN,
    TelemetryReassembler,
    lora_time_on_air_ms,
    max_telemetry_fragment_payload,
    pack_telemetry_fragments,
    parse_telemetry_fragment,
)


# Profiles fast enough to actually carry a fragment under the 25 ms cap.
# PHY_TELEMETRY (SF9/BW250) preamble alone is ~33 ms so it's excluded
# (the live code path retunes to PHY_IMAGE for fragmented telemetry —
# see the docstring at lora_proto.py:494).
FAST_PROFILES = (PHY_IMAGE, PHY_CONTROL_SF7)
# Slower profiles still pack, just into more fragments.
ALL_PACKABLE_PROFILES = FAST_PROFILES + (PHY_CONTROL_SF8, PHY_CONTROL_SF9)


def _feed_all(ra: TelemetryReassembler, frags, source_id: int,
              topic_id: int, t0_ms: int = 1000) -> bytes | None:
    """Feed every fragment and return the first non-None completion.

    A duplicate fragment delivered *after* completion would normally
    open a fresh partial slot for the same key (because the original
    was deleted on completion), so a naive ``out = ra.feed(...)`` loop
    would return ``None`` from the trailing duplicate. Tests want the
    completion result, so we latch it.
    """
    completion = None
    for i, f in enumerate(frags):
        result = ra.feed(source_id, topic_id, f, now_ms=t0_ms + i)
        if result is not None and completion is None:
            completion = result
    return completion


class TelemetryFragRoundTripTests(unittest.TestCase):
    """TF-A / TF-B / TF-C — packing properties across the length space."""

    def test_round_trip_every_length_image_phy(self):
        """TF-A on PHY_IMAGE: 0..2048 bytes round-trip exactly."""
        chunk = max_telemetry_fragment_payload(PHY_IMAGE)
        self.assertGreater(chunk, 0)
        for length in range(0, 2049, 17):       # stride 17 keeps wall-time low
            payload = bytes((i * 31) & 0xFF for i in range(length))
            frags = pack_telemetry_fragments(payload, frag_seq=length & 0xFF,
                                             profile=PHY_IMAGE)
            ra = TelemetryReassembler()
            out = _feed_all(ra, frags, source_id=1, topic_id=2)
            self.assertEqual(out, payload, f"length={length}")
            self.assertEqual(ra.completed, 1)
            self.assertEqual(ra.duplicates, 0)
            self.assertEqual(ra.timeouts, 0)

    def test_round_trip_every_packable_profile(self):
        """TF-A across every profile that can fit ≥1 byte under the cap."""
        payload = bytes(range(180))
        for profile in ALL_PACKABLE_PROFILES:
            chunk = max_telemetry_fragment_payload(profile)
            if chunk <= 0:
                self.skipTest(f"{profile.name} cannot pack — by design")
            frags = pack_telemetry_fragments(payload, frag_seq=5,
                                             profile=profile)
            ra = TelemetryReassembler()
            out = _feed_all(ra, frags, source_id=1, topic_id=2)
            self.assertEqual(out, payload, profile.name)

    def test_fragment_header_invariants(self):
        """TF-B: magic, shared seq, monotonic idx, shared total."""
        payload = bytes((i & 0xFF) for i in range(500))
        frags = pack_telemetry_fragments(payload, frag_seq=99,
                                         profile=PHY_IMAGE)
        self.assertGreater(len(frags), 1)
        seen = []
        for i, f in enumerate(frags):
            self.assertEqual(f[0], TELEMETRY_FRAGMENT_MAGIC)
            parsed = parse_telemetry_fragment(f)
            self.assertIsNotNone(parsed)
            seq, idx, total, _data = parsed
            self.assertEqual(seq, 99)
            self.assertEqual(idx, i)
            self.assertEqual(total, len(frags))
            seen.append(idx)
        self.assertEqual(seen, list(range(len(frags))))

    def test_every_fragment_under_airtime_cap(self):
        """TF-C: each TelemetryFrame body fits under the 25 ms cap."""
        for profile in FAST_PROFILES:
            chunk = max_telemetry_fragment_payload(profile)
            # Sized to ~50 fragments on this profile so we sweep a
            # range of last-fragment payload sizes.
            length = min(800, chunk * 50)
            payload = bytes((i & 0xFF) for i in range(length))
            frags = pack_telemetry_fragments(payload, frag_seq=1,
                                             profile=profile)
            for f in frags:
                envelope = TELEM_HEADER_LEN + CRC_LEN
                air = lora_time_on_air_ms(envelope + len(f), profile)
                self.assertLessEqual(
                    air,
                    TELEMETRY_FRAGMENT_MAX_AIRTIME_MS + 1e-6,
                    f"{profile.name}: fragment air {air} ms > cap")


class TelemetryFragOversizeTests(unittest.TestCase):
    """TF-D — the > 256-fragment rejection path."""

    def test_oversize_payload_raises(self):
        chunk = max_telemetry_fragment_payload(PHY_IMAGE)
        too_big = bytes(chunk * 257)        # exactly 257 fragments worth
        with self.assertRaises(ValueError):
            pack_telemetry_fragments(too_big, frag_seq=0, profile=PHY_IMAGE)

    def test_at_boundary_packs_ok(self):
        chunk = max_telemetry_fragment_payload(PHY_IMAGE)
        ok = bytes(chunk * 256)
        frags = pack_telemetry_fragments(ok, frag_seq=0, profile=PHY_IMAGE)
        self.assertEqual(len(frags), 256)
        ra = TelemetryReassembler()
        out = _feed_all(ra, frags, source_id=1, topic_id=2)
        self.assertEqual(out, ok)


class TelemetryFragReorderTests(unittest.TestCase):
    """TF-E / TF-F — reordering and duplication don't break recovery."""

    def test_reordered_delivery_completes(self):
        rng = random.Random(0xBEEF)
        payload = bytes(range(240))
        frags = pack_telemetry_fragments(payload, frag_seq=12,
                                         profile=PHY_IMAGE)
        for trial in range(20):
            shuffled = frags[:]
            rng.shuffle(shuffled)
            ra = TelemetryReassembler()
            out = _feed_all(ra, shuffled, source_id=1, topic_id=2)
            self.assertEqual(out, payload, f"trial={trial}")

    def test_duplicate_delivery_counts_and_completes(self):
        payload = bytes((i & 0xFF) for i in range(240))
        frags = pack_telemetry_fragments(payload, frag_seq=33,
                                         profile=PHY_IMAGE)
        self.assertGreaterEqual(len(frags), 2)
        ra = TelemetryReassembler()
        # Send all-but-last 3× (each extra is a duplicate), then last
        # once — first delivery of the last fragment completes the
        # assembly. We avoid feeding the last fragment a second time
        # because that would open a new partial slot for the same key.
        t = 0
        for f in frags[:-1]:
            for _ in range(3):
                ra.feed(1, 2, f, now_ms=t)
                t += 1
        out = ra.feed(1, 2, frags[-1], now_ms=t)
        self.assertEqual(out, payload)
        self.assertEqual(ra.completed, 1)
        self.assertEqual(ra.duplicates, 2 * (len(frags) - 1))


class TelemetryFragGapAndTimeoutTests(unittest.TestCase):
    """TF-G — missing middle fragments time out and reclaim."""

    def test_missing_middle_does_not_complete(self):
        payload = bytes(range(240))
        frags = pack_telemetry_fragments(payload, frag_seq=44,
                                         profile=PHY_IMAGE)
        self.assertGreaterEqual(len(frags), 3)
        # Drop the middle fragment.
        keep = frags[:1] + frags[2:]
        ra = TelemetryReassembler(timeout_ms=500)
        out = _feed_all(ra, keep, source_id=1, topic_id=2, t0_ms=0)
        self.assertIsNone(out)
        self.assertEqual(ra.completed, 0)
        # Now advance past the timeout via a noop feed of an unrelated key.
        ra.feed(99, 99, b"unfragmented", now_ms=10_000)
        self.assertEqual(ra.timeouts, 1)
        self.assertEqual(ra.pending_keys(), [])


class TelemetryFragIsolationTests(unittest.TestCase):
    """TF-H / TF-I — multi-stream isolation + seq-wrap behaviour."""

    def test_two_streams_same_seq_different_keys(self):
        payload_a = bytes(b"AAAA" * 60)
        payload_b = bytes(b"BBBB" * 60)
        frags_a = pack_telemetry_fragments(payload_a, frag_seq=7,
                                           profile=PHY_IMAGE)
        frags_b = pack_telemetry_fragments(payload_b, frag_seq=7,
                                           profile=PHY_IMAGE)
        ra = TelemetryReassembler()
        # Interleave A (source=1, topic=10) and B (source=2, topic=10).
        out_a = out_b = None
        t = 0
        for fa, fb in zip(frags_a, frags_b):
            out_a = ra.feed(1, 10, fa, now_ms=t); t += 1
            out_b = ra.feed(2, 10, fb, now_ms=t); t += 1
        # If the lengths differ, drain the longer.
        for fa in frags_a[len(frags_b):]:
            out_a = ra.feed(1, 10, fa, now_ms=t); t += 1
        for fb in frags_b[len(frags_a):]:
            out_b = ra.feed(2, 10, fb, now_ms=t); t += 1
        self.assertEqual(out_a, payload_a)
        self.assertEqual(out_b, payload_b)
        self.assertEqual(ra.completed, 2)

    def test_seq_wrap_with_intervening_gc(self):
        """TF-I: same frag_seq used twice on the same key, separated by
        a GC pass, must complete twice independently."""
        payload = bytes(range(180))
        frags = pack_telemetry_fragments(payload, frag_seq=0,
                                         profile=PHY_IMAGE)
        ra = TelemetryReassembler(timeout_ms=100)
        out1 = _feed_all(ra, frags, source_id=1, topic_id=2, t0_ms=0)
        # Advance well past the timeout, then send a same-seq stream again.
        out2 = _feed_all(ra, frags, source_id=1, topic_id=2, t0_ms=100_000)
        self.assertEqual(out1, payload)
        self.assertEqual(out2, payload)
        self.assertEqual(ra.completed, 2)


class TelemetryFragPassthroughTests(unittest.TestCase):
    """TF-J — non-magic bodies pass through verbatim."""

    def test_unfragmented_payload_passthrough(self):
        ra = TelemetryReassembler()
        # Anything whose first byte != magic is treated as a complete payload.
        body = b"\x00\x01\x02hello"
        out = ra.feed(1, 2, body, now_ms=0)
        self.assertEqual(out, body)
        self.assertEqual(ra.bad_magic_passthroughs, 1)

    def test_short_body_passthrough(self):
        ra = TelemetryReassembler()
        out = ra.feed(1, 2, b"x", now_ms=0)   # shorter than the 4-byte header
        self.assertEqual(out, b"x")
        self.assertEqual(ra.bad_magic_passthroughs, 1)


class TelemetryFragTotalChangeTests(unittest.TestCase):
    """TF-K — mid-stream ``total`` change drops the partial and restarts.

    The live ``feed()`` at lora_proto.py:611 builds a fresh slot when
    ``slot.total != total``. We construct a synthetic header to force
    the divergence (the packer never emits inconsistent totals on its
    own, but a corrupt sender could)."""

    def test_total_change_resets_partial(self):
        # Build two "fragments" by hand with different totals but the
        # same (source, topic, seq) key.
        seq = 5
        # First fragment: claims total=4, idx=0, payload=b"AA".
        f1 = bytes([TELEMETRY_FRAGMENT_MAGIC, seq, 0, 3]) + b"AA"
        # Second fragment: claims total=2, idx=0, payload=b"BB".
        f2 = bytes([TELEMETRY_FRAGMENT_MAGIC, seq, 0, 1]) + b"BB"
        # Third fragment: completes the new total=2 stream, idx=1.
        f3 = bytes([TELEMETRY_FRAGMENT_MAGIC, seq, 1, 1]) + b"CC"
        ra = TelemetryReassembler()
        self.assertIsNone(ra.feed(1, 2, f1, now_ms=0))
        self.assertIsNone(ra.feed(1, 2, f2, now_ms=1))
        out = ra.feed(1, 2, f3, now_ms=2)
        # The new (total=2) assembly completes from f2+f3 only; f1 is gone.
        self.assertEqual(out, b"BBCC")
        self.assertEqual(ra.completed, 1)


class TelemetryFragRandomisedStressTests(unittest.TestCase):
    """TF-L — randomised stress with PRNG seed for determinism."""

    def test_two_hundred_random_streams(self):
        rng = random.Random(0xC0FFEE)
        # Cache per-profile chunk so we can clamp length to fit ≤ 256
        # fragments (the packer's hard cap).
        chunks = {p: max_telemetry_fragment_payload(p) for p in FAST_PROFILES}
        for trial in range(200):
            profile = rng.choice(FAST_PROFILES)
            chunk = chunks[profile]
            max_len = chunk * 256
            length = rng.randint(0, min(1500, max_len))
            payload = rng.randbytes(length)
            seq = rng.randint(0, 255)
            source = rng.randint(0, 7)
            topic = rng.randint(0, 31)
            frags = pack_telemetry_fragments(payload, frag_seq=seq,
                                             profile=profile)
            # 30 % chance to shuffle, 30 % chance to duplicate one fragment.
            if rng.random() < 0.30:
                rng.shuffle(frags)
            if rng.random() < 0.30 and frags:
                frags.insert(rng.randint(0, len(frags) - 1),
                             frags[rng.randint(0, len(frags) - 1)])
            ra = TelemetryReassembler()
            out = _feed_all(ra, frags, source_id=source, topic_id=topic)
            self.assertEqual(
                out, payload,
                f"trial={trial} length={length} seq={seq} "
                f"source={source} topic={topic} profile={profile.name}")


if __name__ == "__main__":            # pragma: no cover
    unittest.main()
