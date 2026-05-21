"""W2-02 P0c (2026-05-20) v2 redundancy wire-format reassembler tests.

Validates:
    R1. v1 (0xFE) and v2 (0xFD) wire formats coexist in one stream.
    R2. v2 fragments dedup by (frag_seq, frag_idx) using copy_idx bitmask;
        the first copy fills the slot, subsequent copies of the same idx
        increment v2_redundant_copies_seen without corrupting the slot.
    R3. Loss of *all but one* copy per fragment still completes the frame
        (the redundancy actually saves the frame).
    R4. Malformed v2 redundancy bytes (copy_idx >= total_copies, total_copies=0)
        are rejected and counted in v2_bad_header.
    R5. Exact-duplicate (frag_seq, frag_idx, copy_idx) re-deliveries land
        in duplicate_fragments (not v2_redundant_copies_seen).

The test deliberately bypasses TileDeltaFrame decoding: it feeds a payload
that the v1 decoder will fail to parse, then asserts on the reassembler's
post-completion stats (decode_errors increments, but parts ARE assembled).
This isolates the wire-format/dedup logic from the WebP/tile codec.
"""
from __future__ import annotations

import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_BS = os.path.abspath(os.path.join(_HERE, ".."))
if _BS not in sys.path:
    sys.path.insert(0, _BS)

from image_pipeline.reassemble import (   # noqa: E402
    FRAGMENT_HEADER_LEN,
    FRAGMENT_HEADER_LEN_V2,
    FRAGMENT_MAGIC,
    FRAGMENT_MAGIC_V2,
    FragmentReassembler,
)


def _v1(frag_seq: int, idx: int, total: int, body: bytes) -> bytes:
    return bytes([FRAGMENT_MAGIC, frag_seq & 0xFF, idx & 0xFF,
                  (total - 1) & 0xFF]) + body


def _v2(frag_seq: int, idx: int, total: int, total_copies: int,
        copy_idx: int, body: bytes) -> bytes:
    red = ((total_copies & 0x0F) << 4) | (copy_idx & 0x0F)
    return bytes([FRAGMENT_MAGIC_V2, frag_seq & 0xFF, idx & 0xFF,
                  (total - 1) & 0xFF, red]) + body


class TestV2RedundancyWireFormat(unittest.TestCase):

    def test_R1_v1_and_v2_coexist(self):
        # Two completely separate frames with different magics.
        ras = FragmentReassembler()
        # v1 frame at frag_seq=10
        for idx in range(2):
            ras.feed(_v1(10, idx, total=2, body=b"X" * 8))
        # v2 frame at frag_seq=11, redundancy 2
        for idx in range(2):
            for c in range(2):
                ras.feed(_v2(11, idx, total=2, total_copies=2,
                             copy_idx=c, body=b"Y" * 8))
        self.assertEqual(ras.stats.v2_fragments_seen, 4)
        # Each idx of the v2 frame got 2 copies; first fills, second is
        # counted as redundant.
        self.assertEqual(ras.stats.v2_redundant_copies_seen, 2)
        # Each frame "completes" then fails to decode (body is not a real
        # TileDeltaFrame). That's intentional -- we're asserting on the
        # wire-format dedup, not on the codec.
        self.assertEqual(ras.stats.decode_errors, 2)

    def test_R2_v2_dedup_first_copy_wins(self):
        ras = FragmentReassembler()
        # Single-fragment v2 frame; copy 1 arrives first.
        ras.feed(_v2(20, 0, total=1, total_copies=2, copy_idx=1, body=b"AB"))
        # Copy 0 arrives next -- should NOT overwrite, must be counted
        # as v2_redundant_copies_seen.
        ras.feed(_v2(20, 0, total=1, total_copies=2, copy_idx=0, body=b"CD"))
        self.assertEqual(ras.stats.v2_fragments_seen, 2)
        self.assertEqual(ras.stats.v2_redundant_copies_seen, 1)
        self.assertEqual(ras.stats.duplicate_fragments, 0)

    def test_R3_redundancy_saves_lost_copies(self):
        ras = FragmentReassembler()
        # 3-fragment v2 frame with redundancy 2. Lose copy_idx=0 of every
        # fragment; only copy_idx=1 arrives. Frame must still complete.
        for idx in range(3):
            ras.feed(_v2(30, idx, total=3, total_copies=2, copy_idx=1,
                         body=bytes([idx]) * 4))
        self.assertEqual(ras.stats.v2_fragments_seen, 3)
        # No redundant copies (we dropped them on the air).
        self.assertEqual(ras.stats.v2_redundant_copies_seen, 0)
        # Frame "completes" -> decode_errors increments (bogus payload).
        self.assertEqual(ras.stats.decode_errors, 1)
        # Slot was cleared on completion.
        self.assertEqual(ras.pending_frag_seqs(), [])

    def test_R4_bad_v2_header_rejected(self):
        ras = FragmentReassembler()
        # total_copies = 0 (invalid).
        ras.feed(bytes([FRAGMENT_MAGIC_V2, 40, 0, 0, 0x00]) + b"junk")
        # copy_idx >= total_copies (2 of 2 -> copy_idx 2 invalid).
        ras.feed(_v2(41, 0, total=1, total_copies=2, copy_idx=2, body=b"x"))
        self.assertEqual(ras.stats.v2_bad_header, 2)
        # Both got counted as v2_fragments_seen and as decode_errors.
        self.assertEqual(ras.stats.v2_fragments_seen, 2)
        self.assertEqual(ras.stats.decode_errors, 2)
        # No slot was ever opened.
        self.assertEqual(ras.pending_frag_seqs(), [])

    def test_R5_exact_dup_copy_idx_counted_as_duplicate(self):
        ras = FragmentReassembler()
        # Same (frag_seq, frag_idx, copy_idx) twice -- this is a
        # genuine air dup, not redundancy.
        ras.feed(_v2(50, 0, total=2, total_copies=2, copy_idx=0, body=b"A" * 4))
        ras.feed(_v2(50, 0, total=2, total_copies=2, copy_idx=0, body=b"A" * 4))
        self.assertEqual(ras.stats.duplicate_fragments, 1)
        # Not counted as redundant-copy because the bitmask bit was
        # already set for copy 0.
        self.assertEqual(ras.stats.v2_redundant_copies_seen, 0)


if __name__ == "__main__":
    unittest.main()
