"""
test_d13_d14_codec.py — S5 software-half tests for the 2026-05-18 TX-Power
Adaptation design doc.

Covers:
  * S5.1 — AES-GCM-64 implicit-nonce codec (D13):
      - 12 B on-air overhead (matches CRYPTO_GCM64_IMPLICIT.overhead_bytes)
      - round-trip works
      - tampered tag, wrong source_id, wrong boot_ctr all reject cleanly
      - distinct (src, boot_ctr, seq) tuples produce distinct nonces
        (smoke test against nonce reuse, NIST SP 800-38D §8 mandate)
      - tag truncation is exactly 8 bytes
  * S5.2 — image plaintext + CRC32-truncated framer (D14):
      - 6 B on-air overhead
      - round-trip works
      - tampered payload or CRC rejects
      - short frame rejects
  * S5.3 — host-boundary class-tag enforcer:
      - P0/P1/P2 + unauthenticated profile rejects (§21.3-3)
      - P3 + unauthenticated profile accepted (split-trust)
      - P0/P1/P2 + authenticated profile accepted
      - class-downgrade-only invariant: upgrade rejects, downgrade allowed
      - sentinel: a tampered D14 fragment that decodes cleanly STILL cannot
        be laundered into a P0 path by the enforcer

These tests skip if `cryptography` is not installed (consistent with the
existing test_crypto_vectors.py convention)."""
import unittest

try:
    import cryptography  # noqa: F401
    HAVE_CRYPTOGRAPHY = True
except ImportError:
    HAVE_CRYPTOGRAPHY = False

from lora_proto import (
    CRYPTO_GCM128_EXPLICIT,
    CRYPTO_GCM64_IMPLICIT,
    CRYPTO_IMAGE_PLAIN_CRC32,
    ClassTagViolation,
    GCM64_OVERHEAD,
    GCM64_TAG_LEN,
    IMAGE_PLAIN_OVERHEAD,
    PRIO_P0,
    PRIO_P1,
    PRIO_P2,
    PRIO_P3,
    build_implicit_nonce,
    enforce_class_downgrade_only,
    enforce_class_tag_boundary,
    pack_image_fragment_plain,
    unpack_image_fragment_plain,
)


@unittest.skipUnless(HAVE_CRYPTOGRAPHY, "cryptography library not installed")
class D13Gcm64ImplicitCodecTests(unittest.TestCase):
    KEY = b"\x00" * 16
    SRC = 0x42
    BOOT_CTR = 7

    def _codec(self):
        # Imported lazily so the module loads without cryptography for
        # collection of D14 + enforcer tests that don't need it.
        from lora_proto import (encrypt_frame_gcm64_implicit,
                                decrypt_frame_gcm64_implicit)
        return encrypt_frame_gcm64_implicit, decrypt_frame_gcm64_implicit

    def test_overhead_constant_matches_crypto_profile(self):
        # The codec's on-air overhead MUST equal the predictor's
        # CRYPTO_GCM64_IMPLICIT.overhead_bytes; if these drift the S1.0
        # airtime predictions become wrong.
        self.assertEqual(GCM64_OVERHEAD, CRYPTO_GCM64_IMPLICIT.overhead_bytes)
        self.assertEqual(GCM64_OVERHEAD, 12)
        self.assertEqual(GCM64_TAG_LEN, 8)

    def test_round_trip_short_payload(self):
        enc, dec = self._codec()
        pt = b"hello world"
        wire = enc(self.KEY, self.SRC, self.BOOT_CTR, 123, pt)
        self.assertEqual(len(wire), len(pt) + GCM64_OVERHEAD)
        out = dec(self.KEY, self.SRC, self.BOOT_CTR, wire)
        self.assertEqual(out, (123, pt))

    def test_round_trip_empty_payload(self):
        enc, dec = self._codec()
        wire = enc(self.KEY, self.SRC, self.BOOT_CTR, 0, b"")
        self.assertEqual(len(wire), GCM64_OVERHEAD)   # just seq + tag
        self.assertEqual(dec(self.KEY, self.SRC, self.BOOT_CTR, wire), (0, b""))

    def test_round_trip_control_frame_sized_payload(self):
        # 16 B ControlFrame is the §17 worked example. Verify the codec
        # produces the predicted 28 B on-air size (16 + 12 overhead).
        enc, dec = self._codec()
        pt = bytes(range(16))
        wire = enc(self.KEY, self.SRC, self.BOOT_CTR, 0xDEAD_BEEF, pt)
        self.assertEqual(len(wire), 28)
        self.assertEqual(dec(self.KEY, self.SRC, self.BOOT_CTR, wire),
                         (0xDEAD_BEEF, pt))

    def test_tampered_tag_rejects(self):
        enc, dec = self._codec()
        wire = bytearray(enc(self.KEY, self.SRC, self.BOOT_CTR, 1, b"data"))
        wire[-1] ^= 0x01   # flip a tag bit
        self.assertIsNone(dec(self.KEY, self.SRC, self.BOOT_CTR, bytes(wire)))

    def test_tampered_ciphertext_rejects(self):
        enc, dec = self._codec()
        wire = bytearray(enc(self.KEY, self.SRC, self.BOOT_CTR, 1, b"data"))
        # Flip a ciphertext byte (between seq[:4] and tag[-8:])
        wire[4] ^= 0x01
        self.assertIsNone(dec(self.KEY, self.SRC, self.BOOT_CTR, bytes(wire)))

    def test_wrong_source_id_rejects(self):
        # Implicit nonce binds the frame to (src, boot_ctr). A frame from
        # src=0x42 must NOT decrypt under src=0x43.
        enc, dec = self._codec()
        wire = enc(self.KEY, self.SRC, self.BOOT_CTR, 1, b"data")
        self.assertIsNone(dec(self.KEY, self.SRC + 1, self.BOOT_CTR, wire))

    def test_wrong_boot_ctr_rejects(self):
        # boot_ctr roll guarantees nonce uniqueness across reboots; an
        # RX with the wrong boot_ctr cannot accept stale frames.
        enc, dec = self._codec()
        wire = enc(self.KEY, self.SRC, self.BOOT_CTR, 1, b"data")
        self.assertIsNone(dec(self.KEY, self.SRC, self.BOOT_CTR + 1, wire))

    def test_short_frame_rejects_without_exception(self):
        _, dec = self._codec()
        # Any input shorter than seq(4)+tag(8) must return None, not raise.
        for short_len in range(GCM64_OVERHEAD):
            with self.subTest(short_len=short_len):
                self.assertIsNone(
                    dec(self.KEY, self.SRC, self.BOOT_CTR, b"\x00" * short_len)
                )

    def test_implicit_nonce_is_12_bytes_and_unique_per_seq(self):
        # AES-GCM nonces MUST be 12 B and MUST be unique under a given key
        # (NIST SP 800-38D §8). Verify both invariants at the construction
        # layer so test failures pinpoint the contract violation directly.
        seen = set()
        for seq in range(0, 1000, 7):
            n = build_implicit_nonce(self.SRC, self.BOOT_CTR, seq)
            self.assertEqual(len(n), 12)
            self.assertNotIn(n, seen)
            seen.add(n)
        # Varying boot_ctr also produces distinct nonces (cross-reboot).
        n1 = build_implicit_nonce(self.SRC, 1, 100)
        n2 = build_implicit_nonce(self.SRC, 2, 100)
        self.assertNotEqual(n1, n2)
        # Varying src too (cross-tractor).
        n3 = build_implicit_nonce(self.SRC + 1, 1, 100)
        self.assertNotEqual(n1, n3)


class D14ImagePlainCrc32FramerTests(unittest.TestCase):
    """D14 framer has zero crypto deps — runs unconditionally."""

    def test_overhead_constant_matches_crypto_profile(self):
        self.assertEqual(IMAGE_PLAIN_OVERHEAD,
                         CRYPTO_IMAGE_PLAIN_CRC32.overhead_bytes)
        self.assertEqual(IMAGE_PLAIN_OVERHEAD, 6)

    def test_round_trip(self):
        payload = b"\x01\x02\x03\x04\x05\x06\x07\x08"
        wire = pack_image_fragment_plain(99, payload)
        self.assertEqual(len(wire), len(payload) + IMAGE_PLAIN_OVERHEAD)
        self.assertEqual(unpack_image_fragment_plain(wire), (99, payload))

    def test_round_trip_empty_payload(self):
        wire = pack_image_fragment_plain(0, b"")
        self.assertEqual(len(wire), IMAGE_PLAIN_OVERHEAD)
        self.assertEqual(unpack_image_fragment_plain(wire), (0, b""))

    def test_tampered_payload_rejects(self):
        wire = bytearray(pack_image_fragment_plain(1, b"AAAA"))
        wire[4] ^= 0x01
        self.assertIsNone(unpack_image_fragment_plain(bytes(wire)))

    def test_tampered_crc_rejects(self):
        wire = bytearray(pack_image_fragment_plain(1, b"AAAA"))
        wire[-1] ^= 0x01
        self.assertIsNone(unpack_image_fragment_plain(bytes(wire)))

    def test_short_frame_rejects(self):
        for short_len in range(IMAGE_PLAIN_OVERHEAD):
            with self.subTest(short_len=short_len):
                self.assertIsNone(
                    unpack_image_fragment_plain(b"\x00" * short_len)
                )

    def test_seq_field_is_4_bytes_big_endian(self):
        # Wire format pin: first 4 bytes are seq big-endian. Locking this
        # so a future endianness flip becomes a noisy test failure.
        wire = pack_image_fragment_plain(0x01020304, b"x")
        self.assertEqual(wire[:4], b"\x01\x02\x03\x04")


class HostBoundaryClassTagEnforcerTests(unittest.TestCase):

    def test_p0_p1_p2_with_unauthenticated_profile_rejects(self):
        # §21.3-3: kinetic-effector classes MUST carry a MAC.
        for cls in (PRIO_P0, PRIO_P1, PRIO_P2):
            with self.subTest(cls=cls):
                with self.assertRaises(ClassTagViolation):
                    enforce_class_tag_boundary(cls, CRYPTO_IMAGE_PLAIN_CRC32)

    def test_p3_with_unauthenticated_profile_accepted(self):
        # Whole point of D14 split-trust: P3 image fragments may be
        # unauthenticated because they have zero kinetic authority.
        enforce_class_tag_boundary(PRIO_P3, CRYPTO_IMAGE_PLAIN_CRC32)

    def test_all_classes_with_authenticated_profiles_accepted(self):
        for cls in (PRIO_P0, PRIO_P1, PRIO_P2, PRIO_P3):
            for profile in (CRYPTO_GCM128_EXPLICIT, CRYPTO_GCM64_IMPLICIT):
                with self.subTest(cls=cls, profile=profile.name):
                    enforce_class_tag_boundary(cls, profile)

    def test_class_downgrade_allowed(self):
        # P0 → P1/P2/P3 is fine (reduces privilege).
        for dst in (PRIO_P0, PRIO_P1, PRIO_P2, PRIO_P3):
            with self.subTest(dst=dst):
                enforce_class_downgrade_only(PRIO_P0, dst)

    def test_class_upgrade_rejected(self):
        # P3 → P0/P1/P2 must reject (would raise privilege).
        for dst in (PRIO_P0, PRIO_P1, PRIO_P2):
            with self.subTest(dst=dst):
                with self.assertRaises(ClassTagViolation):
                    enforce_class_downgrade_only(PRIO_P3, dst)

    def test_forged_d14_fragment_cannot_be_laundered_into_p0(self):
        # End-to-end sentinel: even a perfectly-valid D14 fragment (which
        # the unpacker happily accepts because the CRC matches) cannot
        # be relabeled as P0 at the host boundary. This is the core
        # safety property that makes D14 split-trust acceptable.
        wire = pack_image_fragment_plain(42, b"\xde\xad\xbe\xef")
        out = unpack_image_fragment_plain(wire)
        self.assertIsNotNone(out)
        # Attacker now tries to feed this fragment into a P0 path:
        with self.assertRaises(ClassTagViolation):
            enforce_class_tag_boundary(PRIO_P0, CRYPTO_IMAGE_PLAIN_CRC32)
        # And tries to relabel its declared class from P3 to P0:
        with self.assertRaises(ClassTagViolation):
            enforce_class_downgrade_only(PRIO_P3, PRIO_P0)


if __name__ == "__main__":
    unittest.main()
