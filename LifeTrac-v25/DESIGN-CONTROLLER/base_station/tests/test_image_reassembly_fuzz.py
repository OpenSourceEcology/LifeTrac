"""Round 13 — Image-pipeline fragmentation/reassembly property + fuzz suite.

Symmetric companion to:

- ``test_command_frame_fuzz.py`` (IP-108, operator → tractor command frame),
- ``test_telemetry_fragmentation_fuzz.py`` (Round 11, M7 → base
  telemetry frame fragmentation, magic ``0xFE``, source/topic keyed).

This suite covers the *third* fragmenter in the system: the on-air
TileDeltaFrame chunker that ``camera_service.py`` will start using once
the M7↔SX1276 SPI driver lands. The reassembler is in
[``image_pipeline/reassemble.py``](../image_pipeline/reassemble.py) and
the wire format is documented in its module docstring:

    fragment header (4 bytes, big-endian):
        u8  magic = 0xFE
        u8  frag_seq        ; rolls 0..255
        u8  frag_idx        ; 0..(total-1)
        u8  total_minus1    ; total fragments == this+1

The existing ``ReassemblerTests`` in ``test_image_pipeline.py`` are
three smoke tests (passthrough, two-fragment, GC). They don't pin the
trickier properties — duplicate handling, reorder safety, mid-stream
``total`` change recovery, multi-stream isolation, seq-wrap, malformed
header rejection, or randomised stress. This file adds those.

Properties asserted (IF-A … IF-L):

- IF-A  Round-trip identity for arbitrary frame sizes split N ways.
- IF-B  Fragment-header invariants (magic, monotonic idx, shared
        ``total``, ``frag_seq`` constant within a stream).
- IF-C  Bad-magic body is treated as a complete frame and counts as a
        passthrough — does not corrupt any in-flight assembly.
- IF-D  ``frag_idx >= total`` is rejected and increments
        ``decode_errors``.
- IF-E  Reordered fragments still reassemble.
- IF-F  Duplicate fragments increment ``duplicate_fragments`` and never
        complete twice.
- IF-G  Missing-middle fragment times out via the injected clock and
        increments ``timeouts``.
- IF-H  Two streams keyed by distinct ``frag_seq`` complete
        independently when interleaved.
- IF-I  Seq-wrap 0..255 → 0 across an intervening GC works.
- IF-J  Truncated header (< 4 bytes) starting with the magic bumps
        ``decode_errors`` rather than raising.
- IF-K  Mid-stream ``total`` change discards the partial and completes
        on the new total.
- IF-L  200-trial PRNG-seeded stress: random frame size, random split
        count, optional shuffle, optional duplicate.
"""

from __future__ import annotations

import random
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from image_pipeline.frame_format import (                  # noqa: E402
    TileBlob,
    TileDeltaFrame,
    encode_tile_delta_frame,
)
from image_pipeline.reassemble import (                    # noqa: E402
    FRAGMENT_HEADER_LEN,
    FRAGMENT_MAGIC,
    FragmentReassembler,
)


# --------------------------------------------------------------------------
# Synthetic frame factory — tile blobs are arbitrary bytes; the
# reassembler doesn't decode WebP, just hands the parser the joined
# payload. We size the tiles so the wire frame lands at predictable
# byte counts for the split logic.
# --------------------------------------------------------------------------
def _make_frame(seq: int, n_tiles_changed: int, blob_size: int = 8) -> bytes:
    indices = list(range(n_tiles_changed))
    tiles = [
        TileBlob(
            index=i,
            tx=i % 12,
            ty=i // 12,
            blob=bytes([(i + b) & 0xFF for b in range(blob_size)]) or b"\x00",
        )
        for i in indices
    ]
    frame = TileDeltaFrame(
        frame_kind=1,            # keyframe
        base_seq=seq & 0xFF,
        grid_w=12,
        grid_h=8,
        tile_px=32,
        changed_indices=indices,
        tiles=tiles,
    )
    return encode_tile_delta_frame(frame)


def _split_into_fragments(wire: bytes, frag_seq: int,
                          n_frags: int) -> list[bytes]:
    """Chunk *wire* into *n_frags* fragments with the 4-byte header that
    `FragmentReassembler` expects."""
    if not (1 <= n_frags <= 256):
        raise ValueError(f"n_frags out of range: {n_frags}")
    chunk = max(1, (len(wire) + n_frags - 1) // n_frags)
    pieces: list[bytes] = []
    cursor = 0
    for _ in range(n_frags - 1):
        pieces.append(wire[cursor:cursor + chunk])
        cursor += chunk
    pieces.append(wire[cursor:])
    # If the last piece would be empty (wire shorter than expected) drop
    # the n_frags down so we don't emit a zero-length fragment.
    while pieces and pieces[-1] == b"":
        pieces.pop()
    total = len(pieces)
    return [
        bytes([FRAGMENT_MAGIC, frag_seq & 0xFF, idx, total - 1]) + body
        for idx, body in enumerate(pieces)
    ]


def _feed_all(ras: FragmentReassembler, frags) -> object:
    """Feed *frags* in order, latching the FIRST non-None completion.

    Same trick as the telemetry-fuzz ``_feed_all``: a duplicate fragment
    delivered after completion opens a fresh partial slot for the same
    ``frag_seq`` (the original was deleted on completion), so the loop's
    last assignment would shadow the real result.
    """
    out = None
    for f in frags:
        result = ras.feed(f)
        if out is None and result is not None:
            out = result
    return out


# --------------------------------------------------------------------------
# IF-A — round-trip identity
# --------------------------------------------------------------------------
class RoundTripIdentityTests(unittest.TestCase):
    def test_if_a_various_sizes_various_splits(self) -> None:
        for n_tiles in (1, 2, 5, 12, 32, 96):
            wire = _make_frame(seq=n_tiles, n_tiles_changed=n_tiles)
            for n_frags in (1, 2, 3, 5, 8):
                if n_frags > len(wire):
                    continue
                ras = FragmentReassembler()
                frags = _split_into_fragments(wire, frag_seq=n_tiles,
                                              n_frags=n_frags)
                frame = _feed_all(ras, frags)
                self.assertIsNotNone(
                    frame,
                    f"reassembly failed for n_tiles={n_tiles} n_frags={n_frags}")
                self.assertEqual(frame.changed_indices, list(range(n_tiles)))
                self.assertEqual(ras.stats.completed_frames, 1)


# --------------------------------------------------------------------------
# IF-B — fragment-header invariants
# --------------------------------------------------------------------------
class FragmentHeaderInvariantsTests(unittest.TestCase):
    def test_if_b_invariants(self) -> None:
        wire = _make_frame(seq=42, n_tiles_changed=20)
        for n_frags in (1, 2, 4, 8, 16):
            frags = _split_into_fragments(wire, frag_seq=42, n_frags=n_frags)
            self.assertEqual(len(frags), n_frags)
            seen_idx = []
            for i, f in enumerate(frags):
                self.assertGreaterEqual(len(f), FRAGMENT_HEADER_LEN)
                self.assertEqual(f[0], FRAGMENT_MAGIC, "magic byte")
                self.assertEqual(f[1], 42, "frag_seq constant per stream")
                self.assertEqual(f[2], i, "monotonic idx 0..total-1")
                self.assertEqual(f[3], n_frags - 1, "total_minus1 constant")
                seen_idx.append(f[2])
            self.assertEqual(seen_idx, list(range(n_frags)))


# --------------------------------------------------------------------------
# IF-C — bad-magic passthrough
# IF-J — truncated header but starts with magic
# --------------------------------------------------------------------------
class PassthroughAndTruncationTests(unittest.TestCase):
    def test_if_c_bad_magic_is_complete_frame(self) -> None:
        wire = _make_frame(seq=0, n_tiles_changed=3)
        # First byte of a real TileDeltaFrame is frame_kind (0 or 1),
        # never 0xFE — perfect passthrough candidate.
        self.assertNotEqual(wire[0], FRAGMENT_MAGIC)
        ras = FragmentReassembler()
        frame = ras.feed(wire)
        self.assertIsNotNone(frame)
        self.assertEqual(ras.stats.bad_magic_passthroughs, 1)
        self.assertEqual(ras.stats.completed_frames, 1)

    def test_if_c_passthrough_does_not_disturb_in_flight(self) -> None:
        ras = FragmentReassembler()
        wire = _make_frame(seq=0, n_tiles_changed=8)
        frags = _split_into_fragments(wire, frag_seq=0, n_frags=4)
        # Inject a passthrough between fragments — it must not corrupt
        # the in-flight assembly.
        passthru = _make_frame(seq=99, n_tiles_changed=2)
        self.assertIsNone(ras.feed(frags[0]))
        passthru_frame = ras.feed(passthru)
        self.assertIsNotNone(passthru_frame)
        self.assertEqual(passthru_frame.base_seq, 99)
        # Continue the original stream; should still complete.
        self.assertIsNone(ras.feed(frags[1]))
        self.assertIsNone(ras.feed(frags[2]))
        last = ras.feed(frags[3])
        self.assertIsNotNone(last)
        self.assertEqual(last.base_seq, 0)
        self.assertEqual(ras.stats.completed_frames, 2)
        self.assertEqual(ras.stats.bad_magic_passthroughs, 1)

    def test_if_j_truncated_header_with_magic(self) -> None:
        ras = FragmentReassembler()
        # 0xFE prefix but only 2 bytes total — too short for the
        # 4-byte fragment header.
        result = ras.feed(bytes([FRAGMENT_MAGIC, 0x00]))
        self.assertIsNone(result)
        self.assertEqual(ras.stats.decode_errors, 1)
        self.assertEqual(ras.stats.bad_magic_passthroughs, 0)


# --------------------------------------------------------------------------
# IF-D — frag_idx >= total
# --------------------------------------------------------------------------
class MalformedFragmentTests(unittest.TestCase):
    def test_if_d_idx_geq_total_rejected(self) -> None:
        ras = FragmentReassembler()
        # total_minus1 = 1 → total = 2; idx = 5 is illegal.
        bogus = bytes([FRAGMENT_MAGIC, 0, 5, 1]) + b"junk"
        self.assertIsNone(ras.feed(bogus))
        self.assertEqual(ras.stats.decode_errors, 1)


# --------------------------------------------------------------------------
# IF-E — reorder
# --------------------------------------------------------------------------
class ReorderTests(unittest.TestCase):
    def test_if_e_reorder(self) -> None:
        wire = _make_frame(seq=11, n_tiles_changed=24)
        rng = random.Random(0xBEEF1)
        for trial in range(20):
            frags = _split_into_fragments(wire, frag_seq=11, n_frags=6)
            order = list(range(len(frags)))
            rng.shuffle(order)
            ras = FragmentReassembler()
            shuffled = [frags[i] for i in order]
            frame = _feed_all(ras, shuffled)
            self.assertIsNotNone(frame, f"trial {trial} order={order}")
            self.assertEqual(frame.base_seq, 11)


# --------------------------------------------------------------------------
# IF-F — duplicate
# --------------------------------------------------------------------------
class DuplicateTests(unittest.TestCase):
    def test_if_f_duplicate_counted_no_double_complete(self) -> None:
        wire = _make_frame(seq=7, n_tiles_changed=12)
        frags = _split_into_fragments(wire, frag_seq=7, n_frags=4)
        ras = FragmentReassembler()
        # Send each non-final fragment 3× then the final once — same
        # pattern as the telemetry fuzz TF-F.
        sequence = []
        for f in frags[:-1]:
            sequence.extend([f, f, f])
        sequence.append(frags[-1])
        completed_frames = []
        for f in sequence:
            r = ras.feed(f)
            if r is not None:
                completed_frames.append(r)
        self.assertEqual(len(completed_frames), 1, "exactly one completion")
        # Each non-final fragment had 2 duplicates after the original,
        # so 2 * (N-1) duplicates total.
        self.assertEqual(ras.stats.duplicate_fragments,
                         2 * (len(frags) - 1))
        self.assertEqual(ras.stats.completed_frames, 1)


# --------------------------------------------------------------------------
# IF-G — missing-middle timeout
# --------------------------------------------------------------------------
class MissingMiddleTimeoutTests(unittest.TestCase):
    def test_if_g_missing_middle_then_gc(self) -> None:
        clock = [0]
        ras = FragmentReassembler(timeout_ms=100, clock_ms=lambda: clock[0])
        wire = _make_frame(seq=3, n_tiles_changed=8)
        frags = _split_into_fragments(wire, frag_seq=3, n_frags=4)
        # Drop frags[2] (a middle fragment).
        for f in (frags[0], frags[1], frags[3]):
            self.assertIsNone(ras.feed(f))
        # Advance the clock past the timeout and feed an unrelated
        # fragment to trigger GC.
        clock[0] = 5_000
        unrelated = bytes([FRAGMENT_MAGIC, 99, 0, 0]) + b"x"
        # frag_seq=99, total=1, single-frag stream that completes
        # immediately as junk-payload — but its decode will fail (not a
        # valid TileDeltaFrame), incrementing decode_errors. The
        # important assertion is the timeout counter for frag_seq=3.
        ras.feed(unrelated)
        self.assertEqual(ras.stats.timeouts, 1)


# --------------------------------------------------------------------------
# IF-H — multi-stream isolation
# --------------------------------------------------------------------------
class MultiStreamIsolationTests(unittest.TestCase):
    def test_if_h_two_streams_interleaved(self) -> None:
        wire_a = _make_frame(seq=10, n_tiles_changed=6)
        wire_b = _make_frame(seq=20, n_tiles_changed=10)
        frags_a = _split_into_fragments(wire_a, frag_seq=10, n_frags=3)
        frags_b = _split_into_fragments(wire_b, frag_seq=20, n_frags=4)
        # Interleave them deterministically.
        interleaved = []
        a_iter, b_iter = iter(frags_a), iter(frags_b)
        for pair in zip(a_iter, b_iter):
            interleaved.extend(pair)
        # Tail of whichever is longer.
        interleaved.extend(a_iter)
        interleaved.extend(b_iter)
        ras = FragmentReassembler()
        completed = []
        for f in interleaved:
            r = ras.feed(f)
            if r is not None:
                completed.append(r)
        self.assertEqual(len(completed), 2)
        seqs = sorted(r.base_seq for r in completed)
        self.assertEqual(seqs, [10, 20])


# --------------------------------------------------------------------------
# IF-I — seq-wrap with intervening GC
# --------------------------------------------------------------------------
class SeqWrapTests(unittest.TestCase):
    def test_if_i_seq_wrap_after_gc(self) -> None:
        clock = [0]
        ras = FragmentReassembler(timeout_ms=100,
                                  clock_ms=lambda: clock[0])

        wire1 = _make_frame(seq=255, n_tiles_changed=4)
        frags1 = _split_into_fragments(wire1, frag_seq=255, n_frags=2)
        # Send only the first fragment, then let it time out.
        self.assertIsNone(ras.feed(frags1[0]))
        clock[0] = 10_000

        # Re-use frag_seq=0 (after a wrap) for a fresh complete stream.
        wire2 = _make_frame(seq=0, n_tiles_changed=6)
        frags2 = _split_into_fragments(wire2, frag_seq=0, n_frags=3)
        frame = _feed_all(ras, frags2)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.base_seq, 0)
        # frags1[0] was GC'd by the time wire2 started arriving.
        self.assertGreaterEqual(ras.stats.timeouts, 1)


# --------------------------------------------------------------------------
# IF-K — mid-stream total change
# --------------------------------------------------------------------------
class TotalChangeTests(unittest.TestCase):
    def test_if_k_total_change_restarts_partial(self) -> None:
        ras = FragmentReassembler()
        wire = _make_frame(seq=1, n_tiles_changed=4)
        # Original split says total=4 (total_minus1=3) but only feed idx=0…
        partial = bytes([FRAGMENT_MAGIC, 1, 0, 3]) + wire[:5]
        self.assertIsNone(ras.feed(partial))
        # …then producer changes its mind and re-emits the same stream
        # as a 2-fragment carve-up (total_minus1=1). The reassembler
        # must drop the partial, start fresh, and complete on the
        # second frag of the new split.
        midpoint = len(wire) // 2
        new_a = bytes([FRAGMENT_MAGIC, 1, 0, 1]) + wire[:midpoint]
        new_b = bytes([FRAGMENT_MAGIC, 1, 1, 1]) + wire[midpoint:]
        self.assertIsNone(ras.feed(new_a))
        frame = ras.feed(new_b)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.base_seq, 1)


# --------------------------------------------------------------------------
# IF-L — randomised stress
# --------------------------------------------------------------------------
class RandomisedStressTests(unittest.TestCase):
    def test_if_l_prng_stress(self) -> None:
        rng = random.Random(0xC0FFEE13)
        for trial in range(200):
            n_tiles = rng.randint(1, 96)
            wire = _make_frame(seq=trial & 0xFF, n_tiles_changed=n_tiles)
            max_frags = min(8, len(wire))
            n_frags = rng.randint(1, max(1, max_frags))
            frag_seq = rng.randint(0, 255)
            frags = _split_into_fragments(wire, frag_seq=frag_seq,
                                          n_frags=n_frags)
            # Optional shuffle.
            if rng.random() < 0.30 and len(frags) > 1:
                rng.shuffle(frags)
            # Optional duplicate (insert before the last fragment so we
            # don't shadow the completion the way we would by inserting
            # after).
            if rng.random() < 0.30 and len(frags) >= 2:
                dup = frags[rng.randrange(len(frags) - 1)]
                frags.insert(len(frags) - 1, dup)

            ras = FragmentReassembler()
            frame = _feed_all(ras, frags)
            self.assertIsNotNone(
                frame,
                f"trial {trial} n_tiles={n_tiles} n_frags={n_frags} "
                f"frag_seq={frag_seq}")
            self.assertEqual(frame.base_seq, trial & 0xFF)


if __name__ == "__main__":                                  # pragma: no cover
    unittest.main()
