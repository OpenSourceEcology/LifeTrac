"""Strict-path 0xFB command frames (LoRa-only tractor control plane).

2026-07-25: the base→tractor control plane must ride the LoRa link
itself (no LAN/IP path to the tractor in the field). These tests pin the
command codec plus its non-collision with the image fragment magics.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from lora_proto import (  # noqa: E402
    COMMAND_FRAME_MAGIC,
    CMD_OP_REQ_KEYFRAME,
    CMD_OP_ENCODE_MODE,
    CMD_OP_RADIO_PROFILE,
    CMD_OP_RADIO_PROFILE_ACK,
    CMD_OP_RADIO_PROFILE_CONF,
    CMD_OP_ENCODE_MODE_ACK,
    TELEMETRY_FRAGMENT_MAGIC,
    TELEMETRY_FRAGMENT_MAGIC_V2,
    TELEMETRY_FRAGMENT_MAGIC_PARITY,
    pack_command_frame,
    parse_command_frame,
    parse_telemetry_fragment,
)


class CommandFrameTests(unittest.TestCase):
    def test_magic_below_fragment_magics(self):
        # Receivers dispatch on the first byte BEFORE the reassembler;
        # the magic must never collide with any fragment magic.
        self.assertEqual(COMMAND_FRAME_MAGIC, 0xFB)
        self.assertNotIn(COMMAND_FRAME_MAGIC, (
            TELEMETRY_FRAGMENT_MAGIC, TELEMETRY_FRAGMENT_MAGIC_V2,
            TELEMETRY_FRAGMENT_MAGIC_PARITY))

    def test_round_trip_all_opcodes(self):
        cases = [
            (CMD_OP_REQ_KEYFRAME, b""),
            (CMD_OP_ENCODE_MODE, bytes([6])),
            (CMD_OP_RADIO_PROFILE, bytes([2])),
            (CMD_OP_RADIO_PROFILE_ACK, bytes([1])),
            (CMD_OP_RADIO_PROFILE_CONF, bytes([2])),
            (CMD_OP_ENCODE_MODE_ACK, b'{"effective": 1}'),
        ]
        for opcode, args in cases:
            body = pack_command_frame(opcode, args)
            self.assertEqual(body[0], COMMAND_FRAME_MAGIC)
            parsed = parse_command_frame(body)
            self.assertIsNotNone(parsed)
            self.assertEqual(parsed, (opcode, args))

    def test_parse_rejects_non_command(self):
        self.assertIsNone(parse_command_frame(b""))
        self.assertIsNone(parse_command_frame(b"\xfb"))          # magic only
        self.assertIsNone(parse_command_frame(b"\xfe\x01\x02"))  # fragment
        # Right magic, unknown opcode → None (forward compatibility).
        self.assertIsNone(parse_command_frame(bytes([0xFB, 0x00])))
        self.assertIsNone(parse_command_frame(bytes([0xFB, 0xFF, 1, 2])))

    def test_pack_validates(self):
        with self.assertRaises(ValueError):
            pack_command_frame(0x00)
        with self.assertRaises(ValueError):
            pack_command_frame(CMD_OP_ENCODE_MODE_ACK, b"x" * 201)

    def test_command_frames_invisible_to_fragment_parser(self):
        # A command frame must never parse as a telemetry/image fragment.
        body = pack_command_frame(CMD_OP_RADIO_PROFILE, bytes([2]))
        self.assertIsNone(parse_telemetry_fragment(body))


if __name__ == "__main__":
    unittest.main()
