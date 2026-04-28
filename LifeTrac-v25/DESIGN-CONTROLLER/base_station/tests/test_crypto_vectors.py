"""
test_crypto_vectors.py — LifeTrac v25 AES-128-GCM golden-vector regression.

Per DECISIONS.md D-C2 the firmware uses MbedTLS (Portenta H7) and rweather
Crypto (MKR WAN 1310). Both must produce byte-for-byte identical output to
the Python `cryptography.hazmat.primitives.ciphers.aead.AESGCM` reference
for the lp_encrypt / lp_decrypt wrappers in firmware/common/lora_proto/.

This test verifies the *Python reference* matches the committed JSON
vectors, locking the spec. The matching firmware-side host check lives at
firmware/bench/crypto_vectors/host_check.c — running it under CI is
gated on having mbedtls-dev installed, which we do not require for the
Python test suite, so it is invoked separately by `make crypto-check`
inside that bench directory.

If `cryptography` is not installed the test is skipped (the runtime base
station depends on it indirectly via paho-mqtt+TLS, but our minimal dev
shell does not always have it).
"""

import json
import unittest
from pathlib import Path

VECTORS_PATH = (
    Path(__file__).resolve().parent.parent.parent
    / "firmware" / "bench" / "crypto_vectors" / "vectors.json"
)


class CryptoVectorsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            from cryptography.hazmat.primitives.ciphers.aead import AESGCM  # noqa: F401
        except ImportError:
            raise unittest.SkipTest("cryptography library not installed")
        cls.AESGCM = AESGCM
        cls.vectors = [v for v in json.loads(VECTORS_PATH.read_text())
                       if "key" in v]   # skip the leading metadata block

    def test_python_matches_committed_vectors(self):
        """Lock the spec: every vector with concrete ct/tag must round-trip."""
        for v in self.vectors:
            if v.get("ct_tag_combined_check"):
                continue   # synthetic vector, validated dynamically below
            with self.subTest(name=v["name"]):
                key   = bytes.fromhex(v["key"])
                nonce = bytes.fromhex(v["nonce"])
                pt    = bytes.fromhex(v["pt"])
                aad   = bytes.fromhex(v["aad"]) if v["aad"] else None
                gcm = self.AESGCM(key)
                got = gcm.encrypt(nonce, pt, aad)        # ct || tag
                want = bytes.fromhex(v["ct"]) + bytes.fromhex(v["tag"])
                self.assertEqual(
                    got.hex(), want.hex(),
                    f"vector {v['name']} ciphertext+tag drifted from spec",
                )
                # Round-trip: decrypt must recover the plaintext or raise.
                rec = gcm.decrypt(nonce, got, aad)
                self.assertEqual(rec, pt)

    def test_synthetic_lifetrac_frame_self_consistency(self):
        """For vectors marked ct_tag_combined_check we just prove that the
        Python AESGCM impl is internally consistent on the LifeTrac-shaped
        input (right key/nonce/pt sizes, no AAD). The firmware host check
        compares its output bytes to the Python reference live."""
        for v in self.vectors:
            if not v.get("ct_tag_combined_check"):
                continue
            with self.subTest(name=v["name"]):
                key   = bytes.fromhex(v["key"])
                nonce = bytes.fromhex(v["nonce"])
                pt    = bytes.fromhex(v["pt"])
                self.assertEqual(len(key), 16)
                self.assertEqual(len(nonce), 12)
                gcm = self.AESGCM(key)
                ct_tag = gcm.encrypt(nonce, pt, None)
                self.assertEqual(len(ct_tag), len(pt) + 16)
                rec = gcm.decrypt(nonce, ct_tag, None)
                self.assertEqual(rec, pt)


if __name__ == "__main__":
    unittest.main()
