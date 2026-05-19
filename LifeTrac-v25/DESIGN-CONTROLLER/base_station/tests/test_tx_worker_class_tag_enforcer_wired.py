"""Tests for the S5.3b host-boundary class-tag enforcer wiring in
`lora_bridge._tx_worker`. Two complementary checks:

  1. Source-text presence test: the enforcer call is actually wired into
     the TX worker, with the audit + drop scaffold around it. Cheap
     regression guard against an accidental revert during a future
     S5.1b cutover edit.

  2. Behavioural pairing test: for every (frame_type, opcode, topic_id)
     combination the bridge currently emits, the classify_priority →
     enforce_class_tag_boundary pairing against CRYPTO_PROFILE_DEFAULT
     does not raise. (Today's default is GCM-128 explicit → has_mac=True
     → nothing trips. This test is the SENTINEL that flips loud the day
     someone changes CRYPTO_PROFILE_DEFAULT to an unauthenticated profile
     without auditing the dispatch path.)
"""
from __future__ import annotations

import pathlib
import sys
import unittest

_PKG = pathlib.Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

import lora_proto as L  # noqa: E402


class TxWorkerEnforcerWiringTests(unittest.TestCase):
    BRIDGE_PATH = _PKG / "lora_bridge.py"

    def test_tx_worker_calls_enforce_class_tag_boundary(self) -> None:
        src = self.BRIDGE_PATH.read_text(encoding="utf-8")
        self.assertIn("enforce_class_tag_boundary(", src,
                      "S5.3b: TX boundary enforcer call missing from "
                      "lora_bridge.py — was it reverted?")

    def test_violation_is_audit_logged_and_frame_dropped(self) -> None:
        src = self.BRIDGE_PATH.read_text(encoding="utf-8")
        # The drop path must (a) log to audit with the canonical event
        # name and (b) `continue` so the frame never reaches encrypt+TX.
        self.assertIn("class_tag_violation", src)
        self.assertIn("ClassTagViolation", src)
        # Belt-and-suspenders: the enforce call must be physically before
        # the encrypt call inside the same try block, otherwise a
        # violation could go on-air anyway.
        enf_idx = src.index("enforce_class_tag_boundary(")
        enc_idx = src.index("wire = encrypt_frame(")
        self.assertLess(enf_idx, enc_idx,
                        "enforcer must run BEFORE encrypt_frame on TX path")

    def test_default_profile_admits_every_current_priority_class(self) -> None:
        # Sentinel: with today's CRYPTO_PROFILE_DEFAULT (GCM-128 explicit,
        # has_mac=True), all four PRIO_P* classes are admissible. If this
        # ever flips, _tx_worker's drop path will start firing on
        # legitimate traffic and the cutover (S5.1b) needs an audit.
        for prio in (L.PRIO_P0, L.PRIO_P1, L.PRIO_P2, L.PRIO_P3):
            with self.subTest(prio=prio):
                L.enforce_class_tag_boundary(prio, L.CRYPTO_PROFILE_DEFAULT)

    def test_classify_then_enforce_pairing_for_all_tx_paths(self) -> None:
        # Walks every (frame_type, opcode, topic_id) tuple the bridge
        # could currently dispatch and asserts the pairing does not
        # raise under the default profile. Mirrors _tx_worker's exact
        # classification call signature.
        from lora_proto import (
            FT_CONTROL, FT_HEARTBEAT, FT_COMMAND, FT_TELEMETRY,
            CMD_ESTOP, CMD_LINK_TUNE, CMD_REQ_KEYFRAME, CMD_CAMERA_SELECT,
            IMAGE_TOPIC_IDS,
        )
        cases = [
            (FT_CONTROL, None, None),
            (FT_HEARTBEAT, None, None),
            (FT_COMMAND, CMD_ESTOP, None),
            (FT_COMMAND, CMD_LINK_TUNE, None),
            (FT_COMMAND, CMD_REQ_KEYFRAME, None),
            (FT_COMMAND, CMD_CAMERA_SELECT, None),
            (FT_COMMAND, 0xFF, None),   # unknown opcode → P1 default
            (FT_TELEMETRY, None, 0x10),  # non-image telemetry → P2
        ]
        for tid in sorted(IMAGE_TOPIC_IDS):
            cases.append((FT_TELEMETRY, None, tid))
        for frame_type, opcode, topic_id in cases:
            prio = L.classify_priority(frame_type, opcode, topic_id)
            with self.subTest(frame_type=frame_type, opcode=opcode,
                              topic_id=topic_id, prio=prio):
                L.enforce_class_tag_boundary(prio, L.CRYPTO_PROFILE_DEFAULT)

    def test_p0_under_plaintext_profile_is_rejected(self) -> None:
        # The whole point of S5.3b: if the cutover ever paired P0 with the
        # D14 plaintext profile, the enforcer MUST raise so _tx_worker's
        # drop path takes effect rather than encrypting nothing.
        with self.assertRaises(L.ClassTagViolation):
            L.enforce_class_tag_boundary(L.PRIO_P0,
                                          L.CRYPTO_IMAGE_PLAIN_CRC32)


if __name__ == "__main__":
    unittest.main()
