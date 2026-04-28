"""IP-108 follow-up: property/fuzz tests for the FT_COMMAND wire format.

The original IP-108 finding was that ``lp_make_command`` wrote the CRC
at offset ``HEADER_LEN + 1 + arg_len`` while the C ``CommandFrame``
struct exposed a fixed ``crc16`` field at ``HEADER_LEN + 1 + 8``.  For
``arg_len < 8`` those two offsets disagreed, so the on-the-wire CRC
landed *inside* ``arg[]`` and any code reading the struct member saw
stale memory.

This suite locks down the Python wire-format side that the bridge
emits, which is what the M7 actually parses, for every legal
``arg_len`` (0..8). Catches:

* CRC drift (stale ``crc16`` field bug)
* tail-byte non-determinism (used to pick up stack garbage on the C side)
* over-long args being silently truncated instead of refused
* off-by-one in the body length
"""

from __future__ import annotations

import struct
import unittest

from lora_proto import (
    CRC_LEN,
    FT_COMMAND,
    HEADER_LEN,
    PROTO_VERSION,
    SRC_BASE,
    crc16_ccitt,
    pack_command,
    parse_header,
)


# Deterministic per-arg-len corpus. Hypothesis would be nicer but the
# rest of the suite is stdlib-only and we want CI to stay zero-deps.
_OPCODE_CORPUS = [0x00, 0x01, 0x03, 0x04, 0x10, 0x55, 0x62, 0x63, 0xAA, 0xFF]
_SEQ_CORPUS = [0, 1, 0x7F, 0xFF, 0x100, 0x7FFF, 0xFFFE, 0xFFFF]
_SOURCE_CORPUS = [0x01, SRC_BASE, 0x10, 0xFE]


def _arg_corpus(n: int) -> list[bytes]:
    """All-zeros, all-ones, ascending, descending, and a 0xC0/0xDB pair
    (KISS escape boundaries) for every n in [0, 8]."""
    if n == 0:
        return [b""]
    return [
        bytes(n),
        bytes([0xFF] * n),
        bytes(range(n)),
        bytes(range(n, 0, -1)),
        # KISS escape bytes inside args used to break the C parser when
        # the parser ran on the *escaped* buffer instead of the decoded
        # body. Keep a fixture that exercises both.
        (b"\xc0\xdb" * ((n + 1) // 2))[:n],
    ]


class CommandFrameFuzzTests(unittest.TestCase):
    """Exhaustive sweep over (arg_len, opcode, seq, source, args)."""

    def test_pack_command_layout_for_every_arg_len(self) -> None:
        for arg_len in range(0, 9):
            for args in _arg_corpus(arg_len):
                with self.subTest(arg_len=arg_len, args=args.hex()):
                    frame = pack_command(seq=0x1234, opcode=0x62, args=args)

                    # 5-byte header + 1-byte opcode + arg_len + 2-byte CRC.
                    expected_len = HEADER_LEN + 1 + arg_len + CRC_LEN
                    self.assertEqual(len(frame), expected_len)

                    # Header decodes cleanly and reports our SRC_BASE.
                    hdr = parse_header(frame)
                    self.assertIsNotNone(hdr)
                    assert hdr is not None  # narrowing for mypy/pyright
                    self.assertEqual(hdr.version, PROTO_VERSION)
                    self.assertEqual(hdr.source_id, SRC_BASE)
                    self.assertEqual(hdr.frame_type, FT_COMMAND)
                    self.assertEqual(hdr.sequence_num, 0x1234)

                    # Opcode follows the header.
                    self.assertEqual(frame[HEADER_LEN], 0x62)

                    # Args round-trip byte-for-byte at the correct offset.
                    arg_off = HEADER_LEN + 1
                    self.assertEqual(frame[arg_off:arg_off + arg_len], args)

                    # CRC sits *immediately after* the args, not at any
                    # fixed offset that assumes arg_len == 8. Recomputing
                    # over the body must match the trailer.
                    body = frame[:expected_len - CRC_LEN]
                    crc = struct.unpack("<H", frame[-CRC_LEN:])[0]
                    self.assertEqual(crc, crc16_ccitt(body),
                                     "CRC drift on the wire — IP-108 regression")

    def test_pack_command_parameter_sweep(self) -> None:
        for opcode in _OPCODE_CORPUS:
            for seq in _SEQ_CORPUS:
                for source in _SOURCE_CORPUS:
                    args = bytes(range(4))
                    frame = pack_command(seq=seq, opcode=opcode,
                                         args=args, source_id=source)
                    hdr = parse_header(frame)
                    self.assertIsNotNone(hdr)
                    assert hdr is not None
                    self.assertEqual(hdr.frame_type, FT_COMMAND)
                    self.assertEqual(hdr.source_id, source & 0xFF)
                    self.assertEqual(hdr.sequence_num, seq & 0xFFFF)
                    self.assertEqual(frame[HEADER_LEN], opcode & 0xFF)

                    # CRC validates.
                    body = frame[:-CRC_LEN]
                    expected_crc = crc16_ccitt(body)
                    actual_crc = struct.unpack("<H", frame[-CRC_LEN:])[0]
                    self.assertEqual(actual_crc, expected_crc)

    def test_pack_command_refuses_oversize_args(self) -> None:
        with self.assertRaises(ValueError):
            pack_command(seq=0, opcode=0, args=b"\x00" * 9)

    def test_pack_command_no_trailing_pad(self) -> None:
        """Wire format must not pad short args out to 8 bytes — the M7
        parser uses ``arg_len = body_len - HEADER_LEN - 1 - CRC_LEN``
        and would mis-frame a padded payload."""
        for arg_len in range(0, 9):
            frame = pack_command(seq=0, opcode=0, args=bytes([0x5A] * arg_len))
            self.assertEqual(len(frame), HEADER_LEN + 1 + arg_len + CRC_LEN)

    def test_single_bit_flip_breaks_crc(self) -> None:
        """Sanity: any single-byte mutation invalidates the trailer."""
        frame = bytearray(pack_command(seq=0, opcode=0x62, args=b"\xde\xad\xbe\xef"))
        for i in range(len(frame) - CRC_LEN):
            mutated = bytearray(frame)
            mutated[i] ^= 0x01
            body = bytes(mutated[:-CRC_LEN])
            crc = struct.unpack("<H", bytes(mutated[-CRC_LEN:]))[0]
            self.assertNotEqual(crc, crc16_ccitt(body),
                                f"CRC failed to detect bit-flip at offset {i}")


if __name__ == "__main__":
    unittest.main()
