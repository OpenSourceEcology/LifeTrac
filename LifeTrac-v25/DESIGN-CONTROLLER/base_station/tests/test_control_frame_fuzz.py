"""Round 8: property/fuzz tests for the FT_CONTROL wire format.

Companion to ``test_command_frame_fuzz`` (IP-108). That suite locked
down the *command* encoder; this one does the same for ``pack_control``,
which is what the handheld emits at 20 Hz and is the safety-critical
hot path on the M7.

Properties asserted:

* ``pack_control`` always emits exactly ``CTRL_FRAME_LEN`` (16) bytes.
* ``parse_header`` recovers ``version=PROTO_VERSION``, ``frame_type=
  FT_CONTROL``, the ``source_id`` we asked for, and the low 16 bits of
  ``seq``.
* ``crc16_ccitt`` over the body matches the trailing 2 bytes for every
  permutation of stick / button / flag / hb / source / seq we sweep.
* Stick channels are clipped to the signed-int8 range ``[-127, 127]``
  rather than wrapping mod 256 (the analog-stick safety invariant: a
  rail-to-rail joystick must never alias to the *opposite* direction).
* ``buttons`` is masked to 16 bits and ``flags`` / ``hb`` to 8 bits, so
  callers passing wider values can't smash adjacent fields.
* Single-bit flips anywhere in the packed frame are caught by CRC.

Pure stdlib, mirrors the IP-108 fixture style.
"""

from __future__ import annotations

import struct
import unittest

from lora_proto import (
    CRC_LEN,
    CTRL_FRAME_LEN,
    FT_CONTROL,
    HEADER_LEN,
    PROTO_VERSION,
    SRC_BASE,
    SRC_HANDHELD,
    crc16_ccitt,
    pack_control,
    parse_header,
    verify_crc,
)


_STICK_CORPUS = [-200, -128, -127, -64, -1, 0, 1, 64, 127, 128, 200]
_BUTTONS_CORPUS = [0, 1, 0x00FF, 0x5A5A, 0xA5A5, 0xFFFF, 0x10000, 0xDEADBEEF]
_FLAGS_CORPUS = [0, 1, 0x55, 0xAA, 0xFF, 0x100, 0xFFFF]
_HB_CORPUS = [0, 1, 0x7F, 0x80, 0xFF, 0x100]
_SEQ_CORPUS = [0, 1, 0x7F, 0xFF, 0x100, 0x7FFF, 0xFFFE, 0xFFFF, 0x10000, 0x1FFFF]
_SOURCE_CORPUS = [SRC_HANDHELD, SRC_BASE, 0x10, 0xFE]


def _clip_i8_ref(v: int) -> int:
    """Reference clip mirroring ``lora_proto._clip_i8`` so the test does
    not depend on the private helper."""
    return max(-127, min(127, int(v)))


class TestControlFrameFuzz(unittest.TestCase):

    def test_length_and_header_invariants(self) -> None:
        # Sweep header-affecting fields: source × seq. Sticks/buttons
        # don't change framing so we keep them at zero here.
        for src in _SOURCE_CORPUS:
            for seq in _SEQ_CORPUS:
                frame = pack_control(seq, 0, 0, 0, 0, source_id=src)
                with self.subTest(src=src, seq=seq):
                    self.assertEqual(len(frame), CTRL_FRAME_LEN)
                    hdr = parse_header(frame)
                    self.assertIsNotNone(hdr)
                    assert hdr is not None  # type narrow
                    self.assertEqual(hdr.version, PROTO_VERSION)
                    self.assertEqual(hdr.frame_type, FT_CONTROL)
                    self.assertEqual(hdr.source_id, src)
                    self.assertEqual(hdr.sequence_num, seq & 0xFFFF)
                    self.assertTrue(verify_crc(frame))

    def test_crc_locked_at_offset_14(self) -> None:
        # Body = 14 bytes, CRC = trailing 2 bytes. If anyone re-orders
        # ``pack_control``'s struct, this will catch it before the C
        # side silently drops the frame.
        frame = pack_control(0x1234, 1, 2, 3, 4, buttons=0xBEEF, flags=0x5A, hb=0x77)
        body = frame[: CTRL_FRAME_LEN - CRC_LEN]
        crc = struct.unpack("<H", frame[-CRC_LEN:])[0]
        self.assertEqual(crc, crc16_ccitt(body))
        self.assertEqual(len(body), 14)

    def test_stick_clipping_no_aliasing(self) -> None:
        # The safety invariant: a wildly out-of-range stick value must
        # NEVER decode to the opposite sign. ``_clip_i8`` clamps to
        # [-127, 127]; this test asserts the on-wire bytes match the
        # clamp rather than wrapping mod 256.
        for lhx in _STICK_CORPUS:
            for lhy in _STICK_CORPUS:
                for rhx in (_STICK_CORPUS[0], 0, _STICK_CORPUS[-1]):
                    for rhy in (_STICK_CORPUS[0], 0, _STICK_CORPUS[-1]):
                        frame = pack_control(0, lhx, lhy, rhx, rhy)
                        with self.subTest(lhx=lhx, lhy=lhy, rhx=rhx, rhy=rhy):
                            self.assertTrue(verify_crc(frame))
                            decoded = struct.unpack(
                                "<bbbb", frame[HEADER_LEN : HEADER_LEN + 4]
                            )
                            self.assertEqual(
                                decoded,
                                (
                                    _clip_i8_ref(lhx),
                                    _clip_i8_ref(lhy),
                                    _clip_i8_ref(rhx),
                                    _clip_i8_ref(rhy),
                                ),
                            )
                            # Sign-preservation check — the actual safety
                            # property. A positive input must never
                            # decode negative and vice-versa (zero maps
                            # to zero).
                            for raw, dec in zip((lhx, lhy, rhx, rhy), decoded):
                                if raw > 0:
                                    self.assertGreater(dec, 0)
                                elif raw < 0:
                                    self.assertLess(dec, 0)
                                else:
                                    self.assertEqual(dec, 0)

    def test_buttons_flags_hb_masked(self) -> None:
        # Wide values must be masked, not allowed to spill into
        # neighbouring fields. We assert by re-decoding the full body
        # struct and comparing against the documented mask.
        for buttons in _BUTTONS_CORPUS:
            for flags in _FLAGS_CORPUS:
                for hb in _HB_CORPUS:
                    frame = pack_control(0, 0, 0, 0, 0, buttons=buttons,
                                         flags=flags, hb=hb)
                    with self.subTest(buttons=buttons, flags=flags, hb=hb):
                        self.assertTrue(verify_crc(frame))
                        # Body layout after sticks: buttons (H), flags
                        # (B), hb (B), pad (B).
                        b, f, h, pad = struct.unpack(
                            "<HBBB",
                            frame[HEADER_LEN + 4 : HEADER_LEN + 4 + 5],
                        )
                        self.assertEqual(b, buttons & 0xFFFF)
                        self.assertEqual(f, flags & 0xFF)
                        self.assertEqual(h, hb & 0xFF)
                        self.assertEqual(pad, 0)

    def test_seq_wraps_modulo_16_bits(self) -> None:
        for seq in _SEQ_CORPUS:
            frame = pack_control(seq, 0, 0, 0, 0)
            with self.subTest(seq=seq):
                hdr = parse_header(frame)
                assert hdr is not None
                self.assertEqual(hdr.sequence_num, seq & 0xFFFF)
                self.assertTrue(verify_crc(frame))

    def test_single_bit_flips_caught_by_crc(self) -> None:
        # IP-108's symmetric concern: a single bit flip *anywhere* in
        # the body must be caught. We don't flip inside the CRC field
        # itself because that would just verify CRC self-consistency.
        base = pack_control(0xABCD, 12, -34, 56, -78,
                            buttons=0xBEEF, flags=0x5A, hb=0x77,
                            source_id=SRC_HANDHELD)
        self.assertTrue(verify_crc(base))
        body_len = CTRL_FRAME_LEN - CRC_LEN
        for byte_idx in range(body_len):
            for bit in range(8):
                mutated = bytearray(base)
                mutated[byte_idx] ^= 1 << bit
                with self.subTest(byte=byte_idx, bit=bit):
                    self.assertFalse(
                        verify_crc(bytes(mutated)),
                        f"CRC failed to catch flip at byte {byte_idx} bit {bit}",
                    )

    def test_default_source_is_base(self) -> None:
        # Regression: ``source_id`` defaults to ``SRC_BASE`` for the
        # bridge. Drift here would make the M7 misroute frames into
        # the wrong replay-window slot.
        frame = pack_control(0, 0, 0, 0, 0)
        hdr = parse_header(frame)
        assert hdr is not None
        self.assertEqual(hdr.source_id, SRC_BASE)


if __name__ == "__main__":
    unittest.main()
