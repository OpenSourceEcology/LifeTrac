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

    def test_unfragmented_passthrough_is_minus_one(self) -> None:
        # Any magic outside the three fragment layouts carries no seq.
        self.assertEqual(rxd.train_seq_of(b"\x00\x29payload"), -1)
        self.assertEqual(rxd.train_seq_of(b"\xf0\x29payload"), -1)

    def test_short_payloads_are_minus_one(self) -> None:
        self.assertEqual(rxd.train_seq_of(b""), -1)
        self.assertEqual(rxd.train_seq_of(bytes([FRAGMENT_MAGIC])), -1)


class _PublishRecorder:
    """Stub self for ImageRxDaemon._publish_completed (the stub-binding
    pattern used by the RS-11.1/11.6 daemon tests)."""

    def __init__(self) -> None:
        import threading
        self._lock = threading.Lock()
        self.published = []          # (frame_id, seq) per _publish call
        self.cleared = []

    def _publish(self, payload, frame_id, seq=-1):
        self.published.append((frame_id, seq))

    def _clear_pending(self, op, why):
        self.cleared.append(op)


class _FakeFrame:
    codec = 0
    frame_kind = 0


class BatchedPublicationSeqTests(unittest.TestCase):
    def test_every_publish_in_a_batch_gets_the_completing_seq(self) -> None:
        # RS-3.1 batching completes a LIST of frames from ONE fragment
        # train. Drive the real publication path with a 3-frame batch and
        # verify every _publish call receives the completing fragment's
        # seq — not just the first, and not divergent values.
        stub = _PublishRecorder()
        raw = bytes([FRAGMENT_MAGIC, 91, 12, 12]) + b"tail"
        batch = [_FakeFrame(), _FakeFrame(), _FakeFrame()]
        orig = rxd.encode_tile_delta_frame
        rxd.encode_tile_delta_frame = lambda f: b"payload"
        try:
            rxd.ImageRxDaemon._publish_completed(stub, batch, raw)
        finally:
            rxd.encode_tile_delta_frame = orig
        self.assertEqual(stub.published,
                         [(0, 91), (0, 91), (0, 91)])

    def test_single_frame_completion_gets_its_seq(self) -> None:
        stub = _PublishRecorder()
        raw = bytes([FRAGMENT_MAGIC_V2, 200, 0, 0]) + b"x"
        orig = rxd.encode_tile_delta_frame
        rxd.encode_tile_delta_frame = lambda f: b"payload"
        try:
            rxd.ImageRxDaemon._publish_completed(stub, _FakeFrame(), raw)
        finally:
            rxd.encode_tile_delta_frame = orig
        self.assertEqual(stub.published, [(0, 200)])

    def test_keyframe_in_batch_clears_pending_and_still_publishes(self) -> None:
        stub = _PublishRecorder()
        raw = bytes([FRAGMENT_MAGIC, 5, 1, 1]) + b"x"
        kf = _FakeFrame()
        kf.frame_kind = 1
        batch = [_FakeFrame(), kf]
        orig = rxd.encode_tile_delta_frame
        rxd.encode_tile_delta_frame = lambda f: b"payload"
        try:
            rxd.ImageRxDaemon._publish_completed(stub, batch, raw)
        finally:
            rxd.encode_tile_delta_frame = orig
        self.assertEqual(stub.published, [(0, 5), (0, 5)])
        self.assertEqual(len(stub.cleared), 1)


class PublishLinePrefixContractTests(unittest.TestCase):
    def test_bulk_loss_boundary_regex_still_matches(self) -> None:
        # The publish line grew a seq= field; the "published frame_id="
        # prefix is load-bearing for tools/bulk_loss_boundary.py. Drive
        # the REAL ImageRxDaemon._publish with a stub MQTT client,
        # capture the log record it actually emits, and run the tool's
        # own regex against that emitted text — so a prefix/field-order
        # change in production cannot leave this test green. The pattern
        # is lifted from the tool's source text rather than importing
        # the module — the tool pulls in numpy, which the protocol-gate
        # CI env does not install.
        import re
        import threading
        import types

        tool_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__)))), "tools",
            "bulk_loss_boundary.py")
        with open(tool_path, encoding="utf-8") as fh:
            src = fh.read()
        m = re.search(r'PUB_RE = re\.compile\(r"(.*?)"\)', src)
        self.assertIsNotNone(
            m, "PUB_RE definition not found in bulk_loss_boundary.py — "
               "its publish-line contract moved; update this test")

        class _Client:
            def publish(self, topic, payload, qos=0, retain=False):
                return types.SimpleNamespace(rc=0)

        stub = types.SimpleNamespace(
            _client=_Client(),
            _lock=threading.Lock(),
            stats=types.SimpleNamespace(publish_errors=0,
                                        reassembled_frames_published=0))
        with self.assertLogs(rxd.LOG, level="INFO") as cm:
            rxd.ImageRxDaemon._publish(stub, b"x" * 239, 0, 27)
        emitted = [r.getMessage() for r in cm.records
                   if "published" in r.getMessage()]
        self.assertEqual(len(emitted), 1)
        self.assertIsNotNone(re.compile(m.group(1)).search(emitted[0]))
        self.assertIn("seq=27", emitted[0])


if __name__ == "__main__":
    unittest.main()
