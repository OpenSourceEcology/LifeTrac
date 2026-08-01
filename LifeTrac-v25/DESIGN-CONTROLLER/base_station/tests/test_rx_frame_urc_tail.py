"""F8 — parse_rx_frame's additive phase-telemetry tail.

The L072 appends 8 bytes after payload[len] on every RX_FRAME_URC:
{u8 phase_flags, u8 profile_id, u8 hop_idx, u8 slot_offset_ms, u32le epoch}.
The len byte excludes the tail, which is what makes it additive: legacy
parsers slice by len and keep working; the extended parser reads the tail
when present and reports None on all five keys when the firmware predates
F8 — so consumers can distinguish "old firmware" (None) from "no valid
hop header" (zeros with phase_valid False).

Also pins byte-for-byte parity between the v2 parser (canonical, imported
by both daemons) and the v1 legacy duplicate.
"""

import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "firmware", "x8_lora_bootloader_helper"))

from method_h_stage2_tx_probe_v2 import parse_rx_frame as parse_v2  # noqa: E402
from method_h_stage2_tx_probe import parse_rx_frame as parse_v1     # noqa: E402


def _legacy(rx: bytes) -> bytes:
    return struct.pack("<Bbh I".replace(" ", ""), len(rx), -3, -91,
                       0x11223344) + rx


def _tail(flags=0x01, profile=2, hop=37, slotoff=215, epoch=0x0A0B0C0D):
    return struct.pack("<BBBBI", flags, profile, hop, slotoff, epoch)


class TailParsingTests(unittest.TestCase):

    def test_pre_f8_firmware_yields_none(self) -> None:
        d = parse_v2(_legacy(b"\xaa\xbb\xcc"))
        self.assertEqual(d["payload"], b"\xaa\xbb\xcc")
        for k in ("phase_valid", "profile_id", "hop_idx",
                  "slot_offset_ms", "epoch"):
            self.assertIsNone(d[k], f"{k} must be None on legacy frames")

    def test_extended_frame_parses_the_tail(self) -> None:
        d = parse_v2(_legacy(b"\xaa\xbb\xcc") + _tail())
        self.assertEqual(d["payload"], b"\xaa\xbb\xcc",
                         "len byte excludes the tail — payload unchanged")
        self.assertTrue(d["phase_valid"])
        self.assertEqual(d["profile_id"], 2)
        self.assertEqual(d["hop_idx"], 37)
        self.assertEqual(d["slot_offset_ms"], 215,
                         "F7 straddle values (>199) must survive")
        self.assertEqual(d["epoch"], 0x0A0B0C0D)

    def test_zero_tail_reads_as_invalid_not_none(self) -> None:
        d = parse_v2(_legacy(b"\xaa") + _tail(flags=0, profile=0, hop=0,
                                              slotoff=0, epoch=0))
        self.assertIs(d["phase_valid"], False,
                      "zero flags = valid F8 firmware, no hop header")
        self.assertEqual(d["epoch"], 0)

    def test_partial_tail_treated_as_legacy(self) -> None:
        d = parse_v2(_legacy(b"\xaa\xbb") + b"\x01\x02\x25")
        self.assertIsNone(d["phase_valid"],
                          "a <8-byte remainder is not a tail")

    def test_truncated_payload_still_raises(self) -> None:
        with self.assertRaises(ValueError):
            parse_v2(_legacy(b"\xaa\xbb\xcc")[:-1])

    def test_v1_and_v2_agree(self) -> None:
        for frame in (_legacy(b"\xaa\xbb\xcc"),
                      _legacy(b"") + _tail(),
                      _legacy(b"\xaa" * 247) + _tail(epoch=0xFFFFFFFF)):
            self.assertEqual(parse_v1(frame), parse_v2(frame))


if __name__ == "__main__":
    unittest.main()
