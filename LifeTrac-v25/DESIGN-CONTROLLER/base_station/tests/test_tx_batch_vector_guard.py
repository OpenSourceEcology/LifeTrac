"""RS-13 guard (2026-10-10): image_tx_daemon batching vs single-fragment frames.

With LIFETRAC_TX_BATCH=1 (the radio-monitor harness default) ``_batch_more``
priced the FIRST frame of a would-be batch with the 4 B container overhead,
although a batch of one is sent unbatched. Every encode-to-fit frame of
240..243 B (BW500) therefore looked like 2 fragments, and a pair of full
single-fragment frames was "improved" into a 492 B, 3-fragment train
(2 fragments unbatched). For VECTOR (codec 6) that also couples the loss of
consecutive scene frames: one lost fragment drops both, with their DELs
(RS-13 anomaly A16). Pinned here:

* a codec-6 frame is never batched, as the first frame or as a follower;
* a lone frame is priced at its own size, so full single-fragment tile frames
  are no longer paired into a longer train;
* batching never costs more fragments than sending the frames separately,
  while small frames still batch (RS-3.1) and >1-fragment frames keep the
  runt amortization the guard was written for (2026-07-26).
"""

from __future__ import annotations

import os
import queue
import sys
import unittest
from unittest import mock

_HERE = os.path.dirname(os.path.abspath(__file__))
_BASE_STATION = os.path.dirname(_HERE)
_FIRMWARE_X8 = os.path.abspath(os.path.join(_BASE_STATION, "..", "firmware", "tractor_x8"))
_X8_HELPER = os.path.abspath(os.path.join(_BASE_STATION, "..", "firmware",
                                          "x8_lora_bootloader_helper"))
for _p in (_X8_HELPER, _FIRMWARE_X8, _BASE_STATION):
    # Same unconditional re-insert as test_image_tx_rx_optimization: keep the
    # base-station image_pipeline in front of any X8-side namesake.
    if _p in sys.path:
        sys.path.remove(_p)
    sys.path.insert(0, _p)

import image_tx_daemon as txd  # noqa: E402
from image_pipeline.frame_format import (  # noqa: E402
    CODEC_VECTOR, CODEC_WEBP_LUMA, FRAME_BATCH_MAGIC, unpack_frame_batch)
from lora_proto import (  # noqa: E402
    IMAGE_FRAG_AIR_CAP_MS, PHY_IMAGE_BW250, PHY_IMAGE_BW500, pack_image_fragments)

_PHY = {0: PHY_IMAGE_BW250, 2: PHY_IMAGE_BW500}


def _frame(n: int, codec: int, kind: int = 0) -> bytes:
    """An n-byte TileDeltaFrame-shaped payload (6 B header + filler)."""
    assert n >= 6
    return bytes([kind, 1, 12, 8, 32, codec]) + b"\x55" * (n - 6)


def vec(n: int, kind: int = 0) -> bytes:
    return _frame(n, CODEC_VECTOR, kind)


def tile(n: int, kind: int = 0) -> bytes:
    return _frame(n, CODEC_WEBP_LUMA, kind)


def _air_frags(payload: bytes, profile: int) -> int:
    """Fragments the daemon would put on air for this payload (v1 path)."""
    return len(pack_image_fragments(payload, 1, _PHY[profile], IMAGE_FRAG_AIR_CAP_MS))


class _Harness:
    """Just the state _batch_more touches, without opening a radio."""

    def __init__(self, profile: int, queued=()):
        self.d = txd.ImageTxDaemon.__new__(txd.ImageTxDaemon)
        self.d._q = queue.Queue()
        self.d._carry = None
        self.d._active_profile = profile
        for i, p in enumerate(queued):
            self.d._q.put(txd._PendingFrame(seq=2 + i, payload=p, enqueued_ms=0))

    def batch(self, first: bytes):
        with mock.patch.object(txd, "TX_BATCH", 1):
            out = self.d._batch_more(txd._PendingFrame(seq=1, payload=first, enqueued_ms=0))
        return out

    def next_out(self) -> bytes:
        """The frame the TX worker takes next: the carry, else the queue head
        (a VECTOR first frame returns before touching the queue)."""
        if self.d._carry is not None:
            return self.d._carry.payload
        return self.d._q.queue[0].payload


class VectorNeverBatched(unittest.TestCase):

    def test_two_full_vector_frames_stay_two_single_fragment_trains(self):
        # The reviewed case: two 243 B VECTOR deltas at DTS (body 243 B).
        h = _Harness(2, [vec(243)])
        out = h.batch(vec(243))
        self.assertEqual(out.payload, vec(243), "VECTOR frame was batched")
        self.assertEqual(h.next_out(), vec(243), "follower must stay next, in order")
        self.assertEqual(_air_frags(out.payload, 2), 1)
        self.assertEqual(_air_frags(h.next_out(), 2), 1)

    def test_small_vector_frames_are_not_batched_either(self):
        # Even two frames that would share one fragment: each VECTOR frame is
        # its own train (loss coupling, A16), never a batch member.
        h = _Harness(2, [vec(60)])
        out = h.batch(vec(60))
        self.assertEqual(out.payload, vec(60))
        self.assertEqual(h.next_out(), vec(60))

    def test_vector_follower_is_carried_behind_a_tile_frame(self):
        h = _Harness(2, [vec(60), tile(40)])
        out = h.batch(tile(50))
        self.assertEqual(out.payload, tile(50))
        self.assertEqual(h.d._carry.payload, vec(60))
        self.assertEqual(h.d._q.qsize(), 1, "nothing behind the carry consumed")

    def test_vector_epoch_start_keyframe_unbatched(self):
        h = _Harness(2, [vec(80)])
        out = h.batch(vec(200, kind=1))
        self.assertEqual(out.payload, vec(200, kind=1))

    def test_vector_detector(self):
        self.assertEqual(txd._CODEC_VECTOR, CODEC_VECTOR)
        self.assertTrue(txd._is_vector_frame(vec(10)))
        self.assertTrue(txd._is_vector_frame(vec(10, kind=1)))
        self.assertFalse(txd._is_vector_frame(tile(10)))
        self.assertFalse(txd._is_vector_frame(vec(10)[:5]))          # truncated header
        self.assertFalse(txd._is_vector_frame(bytes([FRAME_BATCH_MAGIC, 2, 0, 0, 0, 6])))


class LoneFramePricing(unittest.TestCase):

    def test_full_single_fragment_tile_frames_not_paired_bw500(self):
        h = _Harness(2, [tile(243)])
        out = h.batch(tile(243))
        self.assertEqual(out.payload, tile(243), "two 1-fragment frames became a 3-fragment train")
        self.assertEqual(h.d._carry.payload, tile(243))

    def test_full_single_fragment_tile_frames_not_paired_bw250(self):
        h = _Harness(0, [tile(203)])
        out = h.batch(tile(203))
        self.assertEqual(out.payload, tile(203))
        self.assertEqual(_air_frags(out.payload, 0), 1)

    def test_small_frames_still_batch_into_one_fragment(self):
        h = _Harness(2, [tile(100)])
        out = h.batch(tile(100))
        self.assertEqual(out.payload[0], FRAME_BATCH_MAGIC)
        self.assertEqual(unpack_frame_batch(out.payload), [tile(100), tile(100)])
        self.assertEqual(_air_frags(out.payload, 2), 1)
        self.assertIsNone(h.d._carry)

    def test_multi_fragment_pairs_keep_runt_amortization(self):
        # The 2026-07-26 design case: 246 B frames (2 fragments each alone)
        # pair into 3 fragments and triple into 4.
        h = _Harness(2, [tile(246), tile(246)])
        out = h.batch(tile(246))
        self.assertEqual(len(unpack_frame_batch(out.payload)), 3)
        self.assertEqual(_air_frags(out.payload, 2), 4)

    def test_batch_never_costs_more_than_sending_follower_alone(self):
        # 486 B (2 fragments) + 243 B (1) passes per-frame parity (4/2 <= 2/1)
        # but is 4 fragments batched against 3 separate.
        h = _Harness(2, [tile(243)])
        out = h.batch(tile(486))
        self.assertEqual(out.payload, tile(486))
        self.assertEqual(h.d._carry.payload, tile(243))

    def test_pairs_never_use_more_fragments_than_unbatched(self):
        # Property over frame sizes at both bodies: whatever _batch_more does
        # with a pair, the air cost is at most the unbatched cost.
        sizes = list(range(6, 300, 7)) + [199, 200, 203, 204, 239, 240, 243, 244, 246, 486]
        worse = []
        for profile in (0, 2):
            for a in sizes:
                for b in sizes:
                    h = _Harness(profile, [tile(b)])
                    out = h.batch(tile(a))
                    cost = _air_frags(out.payload, profile)
                    if h.d._carry is not None:
                        cost += _air_frags(h.d._carry.payload, profile)
                    alone = _air_frags(tile(a), profile) + _air_frags(tile(b), profile)
                    if cost > alone:
                        worse.append((profile, a, b, cost, alone))
        self.assertEqual(worse, [], f"batched pairs costing more air: {worse[:8]}")

    def test_batching_off_is_untouched(self):
        h = _Harness(2, [tile(100)])
        with mock.patch.object(txd, "TX_BATCH", 0):
            out = h.d._batch_more(txd._PendingFrame(seq=1, payload=tile(100), enqueued_ms=0))
        self.assertEqual(out.payload, tile(100))
        self.assertEqual(h.d._q.qsize(), 1)


if __name__ == "__main__":
    unittest.main()
