"""RS-12.16 — the reassembler's monotonic fragment-loss counters.

`ReassemblyStats.fragments_expected` / `fragments_missing` feed link_stats
and, through it, the web UI's AutoRadioPolicy loss-rate input. Their
contract, pinned here:

  * `expected` is booked when a frame is FINALIZED — completed (either
    completion path, incl. parity) or timed out — as the frame's total.
  * `missing` is booked ONLY on timeout, as total − fragments present.
  * a parity-RECONSTRUCTED fragment is present, so it is not missing: the
    counters measure the loss that actually cost a frame, after parity.
  * a parity-only partial (total unknown, == 0) books nothing on timeout.
  * both are monotonic across frames, so any consumer can take a rate
    over any window by differencing.

Fragment wire format is the 4-byte header the fuzz suite documents:
[magic, frag_seq, frag_idx, total_minus1] + body; parity fragments carry
[magic_parity, frag_seq, group_start, group_len] + xor_body. Payloads here
are arbitrary bytes — decode outcome is irrelevant to the counters, which
are booked before decode.
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from image_pipeline.reassemble import (  # noqa: E402
    FragmentReassembler, FRAGMENT_MAGIC, FRAGMENT_MAGIC_PARITY)


def _frags(seq: int, bodies: list[bytes]) -> list[bytes]:
    total = len(bodies)
    return [bytes([FRAGMENT_MAGIC, seq & 0xFF, idx, total - 1]) + b
            for idx, b in enumerate(bodies)]


def _parity(seq: int, start: int, length: int, bodies: list[bytes]) -> bytes:
    width = max(len(b) for b in bodies)
    acc = bytearray(width)
    for b in bodies:
        for i, x in enumerate(b):
            acc[i] ^= x
    return bytes([FRAGMENT_MAGIC_PARITY, seq & 0xFF, start, length]) + bytes(acc)


class _Clock:
    def __init__(self) -> None:
        self.ms = 1_000

    def __call__(self) -> int:
        return self.ms


class LossCounters(unittest.TestCase):
    def setUp(self) -> None:
        self.clock = _Clock()
        self.ras = FragmentReassembler(timeout_ms=100, clock_ms=self.clock)
        self.st = self.ras.stats

    def _tick_past_timeout(self) -> None:
        """GC runs at the top of every feed(); advance the clock past the
        timeout and feed an unrelated fragment to trigger it."""
        self.clock.ms += 500
        self.ras.feed(_frags(200, [b"\x01" * 8, b"\x02" * 8])[0])   # new partial

    def test_fresh_stats_are_zero(self) -> None:
        self.assertEqual((self.st.fragments_expected, self.st.fragments_missing), (0, 0))

    def test_completed_frame_books_expected_only(self) -> None:
        for f in _frags(1, [b"a" * 10, b"b" * 10, b"c" * 10, b"d" * 4]):
            self.ras.feed(f)
        self.assertEqual(self.st.fragments_expected, 4)
        self.assertEqual(self.st.fragments_missing, 0)
        self.assertEqual(self.st.timeouts, 0)

    def test_timeout_books_expected_and_missing(self) -> None:
        f = _frags(2, [b"a" * 10, b"b" * 10, b"c" * 10, b"d" * 10])
        for x in (f[0], f[1], f[3]):            # idx 2 never arrives
            self.ras.feed(x)
        self.assertEqual(self.st.fragments_expected, 0)   # not finalized yet
        self._tick_past_timeout()
        self.assertEqual(self.st.timeouts, 1)
        self.assertEqual(self.st.fragments_expected, 4)
        self.assertEqual(self.st.fragments_missing, 1)

    def test_all_fragments_lost_but_one(self) -> None:
        f = _frags(3, [b"a" * 10] * 6)
        self.ras.feed(f[0])
        self._tick_past_timeout()
        self.assertEqual((self.st.fragments_expected, self.st.fragments_missing), (6, 5))

    def test_parity_reconstructed_fragment_is_present_not_missing(self) -> None:
        bodies = [b"\x11" * 12, b"\x22" * 12, b"\x33" * 12]
        f = _frags(4, bodies)
        self.ras.feed(f[0])
        self.ras.feed(f[2])                       # idx 1 lost on air
        self.ras.feed(_parity(4, 0, 3, bodies))   # ...and rebuilt from parity
        self.assertEqual(self.st.parity_reconstructions, 1)
        self.assertEqual(self.st.timeouts, 0)
        self.assertEqual(self.st.fragments_expected, 3)
        self.assertEqual(self.st.fragments_missing, 0)

    def test_parity_only_partial_books_nothing_on_timeout(self) -> None:
        """A parity fragment with no data fragment has no known total."""
        self.ras.feed(_parity(5, 0, 3, [b"\x01" * 8, b"\x02" * 8, b"\x03" * 8]))
        self._tick_past_timeout()
        self.assertEqual(self.st.timeouts, 1)
        self.assertEqual((self.st.fragments_expected, self.st.fragments_missing), (0, 0))

    def test_counters_are_monotonic_across_frames(self) -> None:
        for f in _frags(6, [b"a" * 10, b"b" * 10]):
            self.ras.feed(f)
        f = _frags(7, [b"a" * 10, b"b" * 10, b"c" * 10])
        self.ras.feed(f[0])
        self._tick_past_timeout()
        for x in _frags(8, [b"z" * 5]):
            self.ras.feed(x)
        self.assertEqual(self.st.fragments_expected, 2 + 3 + 1)
        self.assertEqual(self.st.fragments_missing, 0 + 2 + 0)
        # the leg-U style rate a consumer would take: 2 missing of 6
        self.assertAlmostEqual(self.st.fragments_missing / self.st.fragments_expected, 2 / 6)


if __name__ == "__main__":
    unittest.main()
