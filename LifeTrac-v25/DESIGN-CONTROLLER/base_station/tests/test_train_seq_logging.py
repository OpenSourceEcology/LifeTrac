"""Train-seq surfacing on the publish line (PR #111).

`train_seq_of()` is the only bridge between a completing fragment's
header and the `published ... seq=` log line that makes TX-seq <->
RX-train joins possible from standard logs. Pin its contract for every
fragment layout, and pin the log-line prefix contract that
tools/bulk_loss_boundary.py regex-matches.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import image_rx_daemon as rxd  # noqa: E402
from image_pipeline.reassemble import (  # noqa: E402
    FRAGMENT_MAGIC,
    FRAGMENT_MAGIC_PARITY,
    FRAGMENT_MAGIC_V2,
)


class TrainSeqOfTests(unittest.TestCase):
    def test_v1_fragment_seq(self) -> None:
        raw = bytes([FRAGMENT_MAGIC, 27, 3, 12]) + b"payload"
        self.assertEqual(rxd.train_seq_of(raw), 27)

    def test_v2_fragment_seq(self) -> None:
        raw = bytes([FRAGMENT_MAGIC_V2, 255, 0, 0]) + b"x"
        self.assertEqual(rxd.train_seq_of(raw), 255)

    def test_parity_fragment_seq(self) -> None:
        raw = bytes([FRAGMENT_MAGIC_PARITY, 0, 1, 12]) + b"x"
        self.assertEqual(rxd.train_seq_of(raw), 0)

    def test_batched_frames_share_the_completing_fragment_seq(self) -> None:
        # RS-3.1 batching completes a LIST of frames from ONE fragment
        # train; the daemon derives the seq once from the completing
        # fragment and reuses it for every published frame. The helper
        # must be a pure function of the raw fragment for that to hold.
        raw = bytes([FRAGMENT_MAGIC, 91, 12, 12]) + b"tail"
        self.assertEqual(rxd.train_seq_of(raw), rxd.train_seq_of(raw))
        self.assertEqual(rxd.train_seq_of(raw), 91)

    def test_unfragmented_passthrough_is_minus_one(self) -> None:
        # Any magic outside the three fragment layouts carries no seq.
        self.assertEqual(rxd.train_seq_of(b"\x00\x29payload"), -1)
        self.assertEqual(rxd.train_seq_of(b"\xf0\x29payload"), -1)

    def test_short_payloads_are_minus_one(self) -> None:
        self.assertEqual(rxd.train_seq_of(b""), -1)
        self.assertEqual(rxd.train_seq_of(bytes([FRAGMENT_MAGIC])), -1)


class PublishLinePrefixContractTests(unittest.TestCase):
    def test_bulk_loss_boundary_regex_still_matches(self) -> None:
        # The publish line grew a seq= field; the "published frame_id="
        # prefix is load-bearing for tools/bulk_loss_boundary.py. Build
        # the line exactly as the daemon formats it and run the tool's
        # own regex against it.
        tools_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__)))), "tools")
        sys.path.insert(0, tools_dir)
        try:
            import bulk_loss_boundary  # noqa: E402
        finally:
            sys.path.remove(tools_dir)
        line = "published frame_id=%d seq=%d %d B → %s" % (
            0, 27, 239, "lifetrac/v25/video/tile_delta")
        self.assertIsNotNone(bulk_loss_boundary.PUB_RE.search(line))


if __name__ == "__main__":
    unittest.main()
