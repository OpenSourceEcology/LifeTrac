"""Round 22: Camera back-channel keyframe round-trip SIL (W4-08 / IP-103, IP-104).

W4-08 verification surface
==========================

End-to-end keyframe-on-demand path:

    web_ui  ──MQTT──▶  lora_bridge  ──LoRa──▶  M7 (tractor_h7)
                                                    │
                                                    └─UART KISS─▶  X8 camera_service

The HIL acceptance criterion is < 200 ms wall-clock from web-UI click to
keyframe on the X8 encoder, with the LoRa leg < 100 ms. The bench-only
residual is the cross-MCU UART round-trip on real silicon — but the
following slices are pure logic / wire-format / arithmetic and CI-gate
nicely:

* The bridge's MQTT ``cmd/req_keyframe`` → ``pack_command(seq, CMD_REQ_KEYFRAME)``
  translation must produce a byte-identical FT_COMMAND frame whose
  opcode is ``0x62`` and whose CRC validates round-trip.
* That frame must encrypt + KISS-encode + KISS-decode + decrypt + parse
  back to the same plaintext, with the dispatcher recognizing opcode
  ``0x62`` as ``CMD_REQ_KEYFRAME``.
* The same opcode byte, when wrapped by the M7's
  ``send_cmd_to_x8()`` (`[X8_CMD_TOPIC, opcode, args...]`) and KISS-
  encoded over the H747↔X8 UART, must arrive at
  ``dispatch_back_channel`` with ``force_key_evt`` set.
* The boot-PHY (LADDER[0] = SF7/BW250/CR4-5) airtime for the encrypted
  CMD_REQ_KEYFRAME frame must fit inside the < 100 ms LoRa-leg budget
  with margin (the 200 ms end-to-end budget is then dominated by the
  TX queue + UART + encode loop wake-up).

What this does NOT cover (bench-only residual)
----------------------------------------------

* Real SX1276 IRQ → M7 startTransmit → on-air symbol timing.
* Real H747 Serial1 baud-rate stability under M4 load.
* Camera-encoder I-frame produce time on the actual X8 silicon.
* MQTT broker hop latency on the real Linux container.
"""

from __future__ import annotations

import math
import os
import sys
import threading
import unittest
from pathlib import Path

# Make the X8 service importable from the base_station tests (matches
# the sys.path tweak in test_back_channel_dispatch.py).
_X8_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "firmware", "tractor_x8"))
if _X8_DIR not in sys.path:
    sys.path.insert(0, _X8_DIR)

from camera_service import (  # noqa: E402
    CMD_REQ_KEYFRAME as X8_CMD_REQ_KEYFRAME,
    X8_CMD_TOPIC,
    dispatch_back_channel,
)

from lora_proto import (  # noqa: E402
    CMD_REQ_KEYFRAME,
    CRC_LEN,
    FT_COMMAND,
    HEADER_LEN,
    PHY_CONTROL_SF7,
    PROTO_VERSION,
    SRC_BASE,
    KissDecoder,
    decrypt_frame,
    encrypt_frame,
    kiss_encode,
    lora_time_on_air_ms,
    pack_command,
    parse_header,
    verify_crc,
)


# ---------------------------------------------------------------------------
# Wire / timing constants pinned to DECISIONS.md + LORA_PROTOCOL.md
# ---------------------------------------------------------------------------

# IP-103 / IP-104: opcode-only command, no args.
EXPECTED_OPCODE = 0x62
EXPECTED_X8_TOPIC = 0xC0

# Boot PHY (LADDER[0]). Pinned by test_boot_phy_sil.py — duplicated here
# only as a tripwire so a future ladder edit shows up in BOTH tests.
EXPECTED_BOOT_SF = 7
EXPECTED_BOOT_BW_KHZ = 250

# W4-08 acceptance budgets from HIL_RUNBOOK.md.
LORA_LEG_BUDGET_MS = 100.0
END_TO_END_BUDGET_MS = 200.0

# Test fleet key — fixed 16 bytes so the test is deterministic. NOT the
# production key (which lives in a Docker secret per IP-008).
TEST_KEY = bytes.fromhex("000102030405060708090a0b0c0d0e0f")


def _kiss_roundtrip(payload: bytes) -> bytes:
    """KISS-encode, then KISS-decode the resulting frame, return payload.

    Models the M7 ``Serial1.write(kiss)`` → X8 ``_back_channel_reader()``
    UART path in pure Python.
    """
    wire = kiss_encode(payload)
    dec = KissDecoder()
    decoded: list[bytes] = []
    for byte in wire:
        for frame in dec.feed(byte):
            decoded.append(frame)
    if len(decoded) != 1:
        raise AssertionError(f"KISS roundtrip yielded {len(decoded)} frames, want 1")
    return decoded[0]


def _bridge_pack_req_keyframe(seq: int) -> bytes:
    """Inline copy of the bridge's ``/cmd/req_keyframe`` → frame translation.

    See ``lora_bridge.Bridge._on_mqtt_message`` (the ``elif msg.topic.endswith
    ("/cmd/req_keyframe")`` arm). Reproduced here to avoid pulling in
    paho.mqtt + pyserial as test-time hard deps.
    """
    return pack_command(seq, CMD_REQ_KEYFRAME)


def _m7_wrap_for_x8(opcode: int, args: bytes = b"") -> bytes:
    """Inline copy of M7 ``send_cmd_to_x8()`` body framing.

    See ``firmware/tractor_h7/tractor_h7.ino`` ``send_cmd_to_x8()`` —
    the body laid down before KISS-encode is ``[X8_CMD_TOPIC, opcode,
    args...]``. The KISS encode itself is exercised separately so a
    failure is attributable.
    """
    if len(args) > 32:
        raise ValueError("X8 args limited to 32 bytes (matches firmware guard)")
    return bytes([X8_CMD_TOPIC, opcode]) + args


# ---------------------------------------------------------------------------
# KR-A: Constant pinning across the three node sources.
# ---------------------------------------------------------------------------

class KR_A_ConstantPinning(unittest.TestCase):
    """The opcode byte must agree on all three nodes; the X8 topic byte
    must agree between firmware (M7 ``send_cmd_to_x8``) and X8
    (``dispatch_back_channel``)."""

    def test_KR_A_opcode_matches_lora_proto(self) -> None:
        self.assertEqual(CMD_REQ_KEYFRAME, EXPECTED_OPCODE,
                         "lora_proto.CMD_REQ_KEYFRAME must equal 0x62 (LORA_PROTOCOL.md)")

    def test_KR_A2_opcode_matches_camera_service(self) -> None:
        self.assertEqual(X8_CMD_REQ_KEYFRAME, EXPECTED_OPCODE,
                         "camera_service.CMD_REQ_KEYFRAME must equal lora_proto.CMD_REQ_KEYFRAME")
        self.assertEqual(X8_CMD_REQ_KEYFRAME, CMD_REQ_KEYFRAME,
                         "Python-side opcode constants drifted between modules")

    def test_KR_A3_x8_topic_byte_pinned(self) -> None:
        self.assertEqual(X8_CMD_TOPIC, EXPECTED_X8_TOPIC,
                         "X8_CMD_TOPIC must equal 0xC0 (matches send_cmd_to_x8 in tractor_h7.ino)")


# ---------------------------------------------------------------------------
# KR-B: Bridge MQTT → FT_COMMAND wire format.
# ---------------------------------------------------------------------------

class KR_B_BridgeMqttTranslation(unittest.TestCase):
    """The bridge's translation from ``/cmd/req_keyframe`` to an on-air
    frame must produce a well-formed, opcode-only FT_COMMAND."""

    def test_KR_B_frame_layout_is_minimal_command(self) -> None:
        # 5B header + 1B opcode + 0B args + 2B CRC = 8 bytes total.
        frame = _bridge_pack_req_keyframe(seq=0x1234)
        self.assertEqual(len(frame), HEADER_LEN + 1 + CRC_LEN,
                         f"REQ_KEYFRAME plaintext frame must be 8 B, got {len(frame)}")

    def test_KR_B2_header_fields(self) -> None:
        frame = _bridge_pack_req_keyframe(seq=0xABCD)
        hdr = parse_header(frame)
        self.assertIsNotNone(hdr)
        self.assertEqual(hdr.version, PROTO_VERSION)
        self.assertEqual(hdr.source_id, SRC_BASE,
                         "bridge-originated commands must source-id as SRC_BASE")
        self.assertEqual(hdr.frame_type, FT_COMMAND,
                         "REQ_KEYFRAME must be FT_COMMAND, not FT_CONTROL")
        self.assertEqual(hdr.sequence_num, 0xABCD,
                         "header seq must echo the bridge-reserved nonce_seq")

    def test_KR_B3_opcode_byte_is_at_offset_5(self) -> None:
        frame = _bridge_pack_req_keyframe(seq=0)
        # Layout: [version, src, ftype, seq_lo, seq_hi, OPCODE, crc_lo, crc_hi]
        self.assertEqual(frame[HEADER_LEN], EXPECTED_OPCODE,
                         f"opcode byte must be at offset {HEADER_LEN} and equal 0x62")

    def test_KR_B4_crc_validates(self) -> None:
        for seq in (0, 1, 0x7FFF, 0xFFFE, 0xFFFF):
            with self.subTest(seq=seq):
                frame = _bridge_pack_req_keyframe(seq=seq)
                self.assertTrue(verify_crc(frame),
                                f"CRC failed for REQ_KEYFRAME at seq=0x{seq:04x}")


# ---------------------------------------------------------------------------
# KR-C: LoRa air-side round-trip — encrypt + KISS + decode + decrypt.
# ---------------------------------------------------------------------------

class KR_C_LoRaAirRoundTrip(unittest.TestCase):
    """The bridge's frame must survive the encrypt→KISS-encode→KISS-
    decode→decrypt path and arrive intact at the M7's parser."""

    def test_KR_C_full_air_round_trip_preserves_opcode(self) -> None:
        seq = 0x4242
        plaintext = _bridge_pack_req_keyframe(seq=seq)
        wire = encrypt_frame(TEST_KEY, SRC_BASE, seq, plaintext)
        kiss = kiss_encode(wire)

        # Receiver side (mirrors M7 SX1276 RX → KissDecoder.feed loop).
        dec = KissDecoder()
        recovered: list[bytes] = []
        for byte in kiss:
            for frame in dec.feed(byte):
                recovered.append(frame)
        self.assertEqual(len(recovered), 1, "KISS reassembly must yield exactly 1 frame")

        decrypted = decrypt_frame(TEST_KEY, recovered[0])
        self.assertIsNotNone(decrypted, "AES-GCM decrypt failed on round-trip")
        self.assertEqual(decrypted, plaintext,
                         "decrypted plaintext must equal the bridge-side frame")

        # Parser side: header + opcode + CRC.
        hdr = parse_header(decrypted)
        self.assertIsNotNone(hdr)
        self.assertEqual(hdr.frame_type, FT_COMMAND)
        self.assertEqual(decrypted[HEADER_LEN], EXPECTED_OPCODE,
                         "opcode byte must survive air round-trip unchanged")
        self.assertTrue(verify_crc(decrypted),
                        "CRC must still validate after air round-trip")

    def test_KR_C2_kiss_resists_fend_in_payload(self) -> None:
        # The CRC byte CAN happen to be 0xC0 (KISS_FEND). Loop seq values
        # until we find one whose encrypted ciphertext contains 0xC0
        # somewhere — that exercises the FESC/TFEND escape path.
        found_fend = False
        for seq in range(256):
            pt = _bridge_pack_req_keyframe(seq=seq)
            wire = encrypt_frame(TEST_KEY, SRC_BASE, seq, pt)
            if 0xC0 in wire:
                found_fend = True
                kiss = kiss_encode(wire)
                # Every literal 0xC0 in the payload must have been escaped,
                # so the only un-escaped 0xC0 are the two frame delimiters.
                # That means raw count of 0xC0 in wire bytes plus 2 delimiters
                # >= count of 0xC0 in encoded bytes (escape replaces 0xC0
                # with 0xDB 0xDC, so encoded count of 0xC0 is exactly 2).
                self.assertEqual(kiss.count(0xC0), 2,
                                 "KISS encode failed to escape inline 0xC0 bytes")
                # And the round-trip still recovers the original.
                self.assertEqual(_kiss_roundtrip(wire), wire)
                break
        self.assertTrue(found_fend,
                        "test setup error: no seq in 0..255 produced a 0xC0 in ciphertext")


# ---------------------------------------------------------------------------
# KR-D: M7 → X8 UART path arrives at dispatch_back_channel.
# ---------------------------------------------------------------------------

class KR_D_M7ToX8UartPath(unittest.TestCase):
    """The byte sequence the M7 hands to ``Serial1.write(kiss)`` must
    end up triggering ``force_key_evt`` on the X8."""

    def setUp(self) -> None:
        self.evt = threading.Event()

    def test_KR_D_m7_wrap_round_trip_sets_keyframe(self) -> None:
        body = _m7_wrap_for_x8(EXPECTED_OPCODE)  # opcode-only
        self.assertEqual(body, bytes([EXPECTED_X8_TOPIC, EXPECTED_OPCODE]),
                         "M7 send_cmd_to_x8() body framing drifted")
        # KISS over the H747↔X8 UART, then dispatch on the X8 side.
        recovered = _kiss_roundtrip(body)
        dispatch_back_channel(recovered, self.evt)
        self.assertTrue(self.evt.is_set(),
                        "M7-wrapped + KISS-roundtripped REQ_KEYFRAME failed to "
                        "force a keyframe at the X8 dispatcher")

    def test_KR_D2_no_keyframe_if_topic_byte_corrupted(self) -> None:
        # Sanity-check the negative path so a no-op dispatcher would fail
        # the positive test for the wrong reason.
        body = bytes([0xC1, EXPECTED_OPCODE])  # wrong topic byte
        recovered = _kiss_roundtrip(body)
        dispatch_back_channel(recovered, self.evt)
        self.assertFalse(self.evt.is_set(),
                         "wrong topic byte must not force a keyframe")


# ---------------------------------------------------------------------------
# KR-E: LoRa-leg airtime budget at boot PHY.
# ---------------------------------------------------------------------------

class KR_E_LoRaAirtimeBudget(unittest.TestCase):
    """The encrypted CMD_REQ_KEYFRAME frame at LADDER[0] (SF7/BW250) must
    fit the < 100 ms LoRa leg budget with margin so the 200 ms end-to-end
    budget is achievable."""

    @staticmethod
    def _encrypted_len_bytes(plaintext_len: int) -> int:
        # GCM nonce (12) + ciphertext (== plaintext_len) + GCM tag (16).
        # Mirrors lora_proto.encrypted_payload_len() but inlined so a
        # change in either side trips the test.
        return 12 + plaintext_len + 16

    def test_KR_E_boot_phy_pinned(self) -> None:
        # Tripwire — if the boot PHY changes, this whole test class
        # needs re-validation.
        self.assertEqual(PHY_CONTROL_SF7.sf, EXPECTED_BOOT_SF)
        self.assertEqual(PHY_CONTROL_SF7.bw_khz, EXPECTED_BOOT_BW_KHZ)

    def test_KR_E2_req_keyframe_airtime_under_lora_leg_budget(self) -> None:
        plaintext = _bridge_pack_req_keyframe(seq=0x0001)
        on_air_len = self._encrypted_len_bytes(len(plaintext))
        toa_ms = lora_time_on_air_ms(on_air_len, profile=PHY_CONTROL_SF7)
        self.assertLess(
            toa_ms, LORA_LEG_BUDGET_MS,
            f"REQ_KEYFRAME on-air time {toa_ms:.2f} ms exceeds < {LORA_LEG_BUDGET_MS:.0f} ms "
            f"LoRa-leg budget at boot PHY (SF7/BW250). Frame size: "
            f"plaintext={len(plaintext)} B, on-air={on_air_len} B."
        )

    def test_KR_E3_airtime_has_margin_for_end_to_end_budget(self) -> None:
        # End-to-end budget is 200 ms. Decompose: TOA + TX queue head-of-
        # line + UART KISS @ 115200 + dispatcher wake-up. We model:
        #   - TX queue head-of-line: at most 1× a typical FT_CONTROL frame
        #     (the M7 sends ~25 Hz control frames; worst case is a fresh
        #     ControlFrame just hit the radio when REQ_KEYFRAME enqueued).
        #     Use the same SF7/BW250 PHY for the worst case.
        #   - UART KISS: 0xC0 [topic][opcode] 0xC0 = 4 bytes at 115200 bps
        #     ≈ 0.35 ms.
        #   - Dispatcher wake-up + force_key_evt → encode loop pickup:
        #     bound at 50 ms (encode loop iteration period; TODO: tighten
        #     in a follow-up by polling the event with a shorter timeout).
        plaintext = _bridge_pack_req_keyframe(seq=0)
        on_air_len = self._encrypted_len_bytes(len(plaintext))
        req_toa = lora_time_on_air_ms(on_air_len, profile=PHY_CONTROL_SF7)

        # Worst-case head-of-line: an in-flight FT_CONTROL (16 B plaintext).
        ctrl_pt_len = 16  # CTRL_FRAME_LEN
        ctrl_air_len = self._encrypted_len_bytes(ctrl_pt_len)
        ctrl_toa = lora_time_on_air_ms(ctrl_air_len, profile=PHY_CONTROL_SF7)

        uart_ms = (4 * 10) / 115200.0 * 1000.0   # 4 bytes, 10 bits/byte (8N1)
        encode_loop_wake_ms = 50.0

        modeled = req_toa + ctrl_toa + uart_ms + encode_loop_wake_ms
        self.assertLess(
            modeled, END_TO_END_BUDGET_MS,
            f"modeled end-to-end {modeled:.2f} ms exceeds < {END_TO_END_BUDGET_MS:.0f} ms "
            f"budget. Breakdown: REQ TOA={req_toa:.2f}, CTRL TOA={ctrl_toa:.2f}, "
            f"UART={uart_ms:.2f}, encode-wake={encode_loop_wake_ms:.2f}."
        )

    def test_KR_E4_airtime_increases_monotonically_down_the_ladder(self) -> None:
        # Same payload, three rungs — the SF8 / SF9 fall-back rungs must
        # take longer (monotone) so the bench observation that "step-down
        # makes the keyframe path slower" matches the model.
        from lora_proto import PHY_CONTROL_SF8, PHY_CONTROL_SF9

        plaintext = _bridge_pack_req_keyframe(seq=0)
        on_air_len = self._encrypted_len_bytes(len(plaintext))
        toas = [
            lora_time_on_air_ms(on_air_len, profile=PHY_CONTROL_SF7),
            lora_time_on_air_ms(on_air_len, profile=PHY_CONTROL_SF8),
            lora_time_on_air_ms(on_air_len, profile=PHY_CONTROL_SF9),
        ]
        self.assertEqual(toas, sorted(toas),
                         f"airtime did not increase monotonically across the ladder: {toas}")


# ---------------------------------------------------------------------------
# KR-F: Idempotency / re-entrancy of the round-trip.
# ---------------------------------------------------------------------------

class KR_F_RoundTripIdempotency(unittest.TestCase):
    """Repeated REQ_KEYFRAME presses (a likely user behavior — mash the
    button if the first didn't visibly trigger) must remain idempotent
    end-to-end and never decrement the encryption nonce."""

    def test_KR_F_repeated_press_uses_distinct_nonces(self) -> None:
        # The bridge increments seq for every TX. Confirm distinct
        # nonces would be used end-to-end (replay-window safe).
        seqs = [s for s in range(10)]
        wires = [encrypt_frame(TEST_KEY, SRC_BASE, s, _bridge_pack_req_keyframe(s))
                 for s in seqs]
        # Each wire frame must start with a unique 12-byte GCM nonce.
        nonces = {bytes(w[:12]) for w in wires}
        self.assertEqual(len(nonces), len(seqs),
                         "repeated REQ_KEYFRAME presses produced duplicate nonces")

    def test_KR_F2_dispatcher_idempotent_under_repeat(self) -> None:
        evt = threading.Event()
        body = _m7_wrap_for_x8(EXPECTED_OPCODE)
        for _ in range(5):
            recovered = _kiss_roundtrip(body)
            dispatch_back_channel(recovered, evt)
        self.assertTrue(evt.is_set())


if __name__ == "__main__":
    unittest.main()
