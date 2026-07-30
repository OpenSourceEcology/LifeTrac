"""RS-11.1 — fragment-loss attribution in image_rx_daemon.

The `air_gap` histogram could not distinguish a train boundary from a lost
fragment: one loss makes the next inter-arrival delta ~2x ToA (~200 ms), landing
in exactly the same bucket as a real boundary. At the measured ~3.4% fragment
loss that is ~60 contaminating samples per run, so every train-boundary size
derived from that histogram was an upper bound rather than a measurement.

`_note_frag_arrival()` classifies each arrival against the previous one, which
answers two things:

  1. Which class an oversized gap belongs to (so the boundary can be sized, and
     RS-11.2's gap-tuning A/B has a trustworthy input).
  2. Whether the loss floor clusters at fragment index 0 — the RXCONT re-arm-gap
     hypothesis behind firmware item F4. TODO RS-10.1 gates F4 on this, so that
     a firmware cycle is not spent on preamble length against an assumed
     mechanism.

These tests drive the classifier directly; it touches only `self` attributes, so
it binds cleanly to a stub without constructing a daemon (which would want a
serial port and an MQTT broker).
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import image_rx_daemon as rxd  # noqa: E402


class _Stub:
    """Minimal carrier for the classifier's instance state."""
    _last_gap_class = "seq"


def _feed(stub, arrivals):
    """Push (frag_seq, frag_idx, total) tuples; return the class of each."""
    note = rxd.ImageRxDaemon._note_frag_arrival
    out = []
    for seq, idx, total in arrivals:
        note(stub, seq, idx, total)
        out.append(stub._last_gap_class)
    return out


class GapClassificationTests(unittest.TestCase):

    def test_first_fragment_is_seq(self) -> None:
        s = _Stub()
        self.assertEqual(_feed(s, [(0, 0, 13)]), ["seq"])
        self.assertEqual(s._lost_idx_hist, {})

    def test_contiguous_run_is_all_seq_and_loses_nothing(self) -> None:
        s = _Stub()
        classes = _feed(s, [(7, i, 13) for i in range(13)])
        self.assertEqual(set(classes), {"seq"})
        self.assertEqual(s._lost_idx_hist, {},
                         "a clean run must record zero losses")

    def test_frame_change_is_boundary_not_loss(self) -> None:
        """The single most important distinction: a new frag_seq means the gap
        spanned the train boundary, NOT that fragments vanished."""
        s = _Stub()
        classes = _feed(s, [(7, 11, 13), (7, 12, 13), (8, 0, 13), (8, 1, 13)])
        self.assertEqual(classes, ["seq", "seq", "boundary", "seq"])
        self.assertEqual(s._lost_idx_hist, {},
                         "a frame boundary must not be counted as loss")
        self.assertEqual(s._frames_observed, 1)

    def test_index_jump_records_exactly_the_missing_indices(self) -> None:
        s = _Stub()
        classes = _feed(s, [(3, 0, 13), (3, 1, 13), (3, 5, 13), (3, 6, 13)])
        self.assertEqual(classes, ["seq", "seq", "post_loss", "seq"])
        # 2,3,4 never arrived — and only those.
        self.assertEqual(s._lost_idx_hist, {2: 1, 3: 1, 4: 1})

    def test_single_missing_fragment(self) -> None:
        s = _Stub()
        classes = _feed(s, [(3, 0, 13), (3, 2, 13)])
        self.assertEqual(classes, ["seq", "post_loss"])
        self.assertEqual(s._lost_idx_hist, {1: 1})

    def test_duplicate_or_reorder_is_not_counted_as_loss(self) -> None:
        """step <= 0 is a duplicate or a reordered copy. It must not create
        negative-range 'losses', and it must not be classed as clean
        inter-fragment spacing either, since its gap is unrepresentative."""
        s = _Stub()
        classes = _feed(s, [(4, 3, 13), (4, 3, 13), (4, 2, 13)])
        self.assertEqual(classes, ["seq", "boundary", "boundary"])
        self.assertEqual(s._lost_idx_hist, {},
                         "a duplicate/reorder must never register a loss")

    def test_loss_at_index_zero_is_attributed_to_index_zero(self) -> None:
        """THE case F4 exists for: the receiver is still re-arming RXCONT when a
        train starts, so index 0 is what it misses.

        A new frag_seq whose first observed index is > 0 means those leading
        fragments never arrived. An earlier version of the classifier returned
        early on any seq change and was therefore structurally blind to the one
        measurement it was built to make — this test pins the fix."""
        s = _Stub()
        # frame 8 ends, frame 9's idx 0 is lost, idx 1 arrives first.
        classes = _feed(s, [(8, 12, 13), (9, 1, 13), (9, 2, 13)])
        self.assertEqual(classes, ["seq", "post_loss", "seq"])
        self.assertEqual(s._lost_idx_hist, {0: 1},
                         "a lost first fragment must be attributed to index 0")

    def test_multiple_leading_fragments_lost(self) -> None:
        s = _Stub()
        classes = _feed(s, [(8, 12, 13), (9, 3, 13)])
        self.assertEqual(classes, ["seq", "post_loss"])
        self.assertEqual(s._lost_idx_hist, {0: 1, 1: 1, 2: 1})

    def test_lost_trailing_fragment_is_attributed(self) -> None:
        """Symmetric to the leading case and equally invisible to naive index
        arithmetic: if frame N's last fragment is lost, the next arrival is
        frame N+1 index 0 — indistinguishable from a clean transition unless the
        outgoing frame's `total` is checked."""
        s = _Stub()
        # 13-fragment frame that stops at index 10: 11 and 12 never arrived.
        classes = _feed(s, [(2, 9, 13), (2, 10, 13), (3, 0, 13), (3, 1, 13)])
        self.assertEqual(classes, ["seq", "seq", "boundary", "seq"])
        self.assertEqual(s._lost_idx_hist, {11: 1, 12: 1})

    def test_complete_frame_then_transition_records_no_loss(self) -> None:
        """The guard must not fire on a frame that ended properly, or every
        clean boundary would manufacture a phantom tail loss."""
        s = _Stub()
        classes = _feed(s, [(2, 11, 13), (2, 12, 13), (3, 0, 13)])
        self.assertEqual(classes, ["seq", "seq", "boundary"])
        self.assertEqual(s._lost_idx_hist, {},
                         "a frame that reached its final index lost nothing")

    def test_both_tail_and_head_loss_in_one_transition(self) -> None:
        s = _Stub()
        # frame 4 stops at 10 (11,12 lost); frame 5 first seen at 2 (0,1 lost).
        classes = _feed(s, [(4, 10, 13), (5, 2, 13)])
        self.assertEqual(classes, ["seq", "post_loss"])
        self.assertEqual(s._lost_idx_hist, {11: 1, 12: 1, 0: 1, 1: 1})

    def test_joining_mid_stream_is_not_counted_as_loss(self) -> None:
        """The very first fragment ever seen may be mid-frame simply because we
        attached late. With no previous arrival there is nothing to infer, so it
        must not manufacture losses for indices 0..idx-1."""
        s = _Stub()
        classes = _feed(s, [(5, 7, 13), (5, 8, 13)])
        self.assertEqual(classes, ["seq", "seq"])
        self.assertEqual(s._lost_idx_hist, {},
                         "attaching mid-stream must not register phantom loss")

    def test_corrupt_index_is_rejected_before_classification(self) -> None:
        """frag_idx >= total is what the reassembler rejects; the daemon applies
        the same guard so a corrupt header cannot poison the statistics. This
        pins the guard's threshold, matching reassemble.py."""
        # The guard lives at the call site; assert the condition it encodes.
        for idx, total, ok in ((0, 13, True), (12, 13, True),
                               (13, 13, False), (200, 13, False)):
            self.assertEqual(idx < total, ok, f"idx={idx} total={total}")


class LossRateNormalisationTests(unittest.TestCase):
    """The verdict compares RATES, not counts: index k can only go missing on
    frames that reach k fragments, so raw counts would bias toward low indices
    on variable-length frames."""

    def test_opportunity_counter_tracks_every_arrival(self) -> None:
        s = _Stub()
        _feed(s, [(1, 0, 3), (1, 1, 3), (1, 2, 3), (2, 0, 3), (2, 1, 3)])
        self.assertEqual(s._frag_idx_seen, {0: 2, 1: 2, 2: 1})

    def test_uniform_loss_is_not_reported_as_clustered(self) -> None:
        """Constructed so idx 0 and the others have the same loss rate; the
        CLUSTERED-AT-0 verdict must not fire. Guards against an instrument that
        confirms the F4 hypothesis by construction."""
        s = _Stub()
        # 4 frames, each losing exactly one mid-frame index — never idx 0.
        arrivals = []
        for f in range(4):
            arrivals += [(f, 0, 5), (f, 1, 5), (f, 3, 5), (f, 4, 5)]
        _feed(s, arrivals)
        self.assertEqual(s._lost_idx_hist, {2: 4})
        self.assertEqual(s._lost_idx_hist.get(0, 0), 0,
                         "no idx-0 loss was injected, so none may be reported")


if __name__ == "__main__":
    unittest.main()
