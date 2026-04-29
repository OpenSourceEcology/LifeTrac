"""Round 24: IP-102 nonce-seq thread-through SIL.

Claude addendum A-1 / Gemini 1.1 flagged that ``web_ui.ws_control``
packs a ``ControlFrame`` with its own seq counter, then
``lora_bridge._on_mqtt_message`` originally called ``self._tx(...)``
without ``nonce_seq=``. That caused the bridge to *reserve a fresh
seq* for the AEAD nonce while the cleartext header carried a different
seq from the web-ui counter. Inner header seq and outer GCM nonce seq
diverged, which:

* breaks the tractor-side replay window (the inner-header seq the
  tractor checks against ``ReplayWindow`` no longer matches the seq
  baked into the GCM nonce that authenticated the frame), and
* defeats the IP-101 ``NonceStore`` persistence guarantee (the value
  reserved from the store does not appear in the bytes the tractor
  sees on-air).

Round 14-history landed the fix: every ``_on_mqtt_message`` arm
reserves exactly one seq, stamps it into the cleartext header
(``_restamp_control`` for the existing 16-byte FT_CONTROL frame from
``web_ui``, ``pack_command(seq, ...)`` for the opcode-only commands),
and threads that *same* seq into ``_tx(..., nonce_seq=seq)`` so
``encrypt_frame`` re-uses it for the GCM nonce.

This SIL pins that thread-through end-to-end without standing up a
broker / serial port. We use ``Bridge.__new__(Bridge)`` to skip the
hardware-touching constructor, hand-wire only the fields the four
``_on_mqtt_message`` arms touch, and pop frames straight off the
internal priority queue. For each frame we then parse the cleartext
header, build the GCM nonce that ``_tx_worker`` would use, and assert
seq agreement.

Acceptance (per IP-102 spec):

* publish 100 control messages → AEAD nonce seq == cleartext header
  seq for every frame;
* tractor-side replay window (driven from the same per-frame seqs)
  records zero false-rejects.
"""

from __future__ import annotations

import heapq
import itertools
import struct
import threading
import unittest
from pathlib import Path
from unittest import mock

# Make the base_station package importable regardless of cwd.
import sys
_BS = Path(__file__).resolve().parents[1]
if str(_BS) not in sys.path:
    sys.path.insert(0, str(_BS))

import lora_bridge  # noqa: E402
from lora_bridge import Bridge  # noqa: E402
from lora_proto import (  # noqa: E402
    CRC_LEN,
    CTRL_FRAME_LEN,
    FT_COMMAND,
    FT_CONTROL,
    HEADER_LEN,
    PROTO_VERSION,
    ReplayWindow,
    SRC_BASE,
    build_nonce,
    crc16_ccitt,
    pack_control,
    parse_header,
    verify_crc,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

class _FakeMqttMsg:
    """Stand-in for ``paho.mqtt.client.MQTTMessage`` (only ``.topic`` + ``.payload``)."""

    __slots__ = ("topic", "payload")

    def __init__(self, topic: str, payload: bytes) -> None:
        self.topic = topic
        self.payload = payload


def _make_bridge() -> Bridge:
    """Construct a Bridge instance without touching MQTT, serial, or threads.

    We hand-wire ONLY the fields the four ``_on_mqtt_message`` arms and the
    ``_tx`` priority-queue path read or write. The TX worker thread is NOT
    started; tests pop the queue directly so there's no encryption /
    serial-write race to worry about.
    """
    b = Bridge.__new__(Bridge)
    # Audit + counters.
    b.audit = mock.MagicMock()
    b.tx_seq = 0
    b.lock = threading.Lock()
    b.nonce_store = None  # exercise the in-memory counter branch
    # Replay map (per-source); not populated until something is checked.
    b._replay = {}
    # Priority TX queue (we read it back instead of starting the worker).
    b._tx_queue = []
    b._tx_cv = threading.Condition()
    b._tx_counter = itertools.count()
    b._stop_evt = threading.Event()
    return b


def _drain_queue(b: Bridge) -> list[tuple[int, int, bytes]]:
    """Pop everything off ``_tx_queue`` in priority + FIFO order."""
    out: list[tuple[int, int, bytes]] = []
    with b._tx_cv:
        while b._tx_queue:
            _prio, _ord, item = heapq.heappop(b._tx_queue)
            out.append(item)
    return out


def _make_control_payload(seq: int, lhx: int = 10, lhy: int = -20,
                          rhx: int = 30, rhy: int = -40,
                          buttons: int = 0x1234, flags: int = 0x05,
                          hb: int = 0x07) -> bytes:
    # web_ui.ws_control packs a fully-formed CTRL frame; we replicate that
    # so _restamp_control has a valid CRC to recompute against.
    return pack_control(seq, lhx, lhy, rhx, rhy,
                        buttons=buttons, flags=flags, hb=hb)


# ---------------------------------------------------------------------------
# NS-A: control path — _restamp_control + _tx nonce-seq agreement.
# ---------------------------------------------------------------------------

class NS_A_ControlPath(unittest.TestCase):
    """``cmd/control`` — bridge restamps inbound seq, MUST thread its own
    reserved seq into both the cleartext header and the AEAD nonce."""

    def test_NS_A_single_message_seq_threadthrough(self) -> None:
        b = _make_bridge()
        # web_ui sends a frame with its own (different!) counter — the bridge
        # must overwrite the seq, not honor the inbound value.
        web_ui_seq = 0xBEEF
        msg = _FakeMqttMsg("lifetrac/v25/cmd/control",
                           _make_control_payload(web_ui_seq))
        b._on_mqtt_message(None, None, msg)
        items = _drain_queue(b)
        self.assertEqual(len(items), 1, "exactly one frame must be enqueued")
        source_id, queued_seq, pt = items[0]
        self.assertEqual(source_id, SRC_BASE)
        # Bridge owns the seq counter; first reservation is 0 (in-memory branch).
        self.assertEqual(queued_seq, 0,
                         "bridge must use its OWN reserved seq, not the inbound web_ui seq")
        hdr = parse_header(pt)
        self.assertIsNotNone(hdr)
        self.assertEqual(hdr.sequence_num, queued_seq,
                         "cleartext header seq MUST equal the seq passed to "
                         "_tx(nonce_seq=...) — IP-102")
        self.assertNotEqual(hdr.sequence_num, web_ui_seq,
                            "bridge must NOT honor the inbound web_ui seq verbatim — "
                            "that was the original IP-102 bug")
        # CRC must be valid after restamp.
        self.assertTrue(verify_crc(pt),
                        "_restamp_control must recompute CRC after seq overwrite")

    def test_NS_A2_nonce_seq_matches_header_seq_in_gcm_nonce(self) -> None:
        # The whole point of IP-102 is that the GCM nonce's seq bytes
        # (offset 1..3) match the cleartext header seq bytes (offset 3..5).
        # We rebuild the nonce here exactly as _tx_worker would.
        b = _make_bridge()
        b._on_mqtt_message(None, None,
                           _FakeMqttMsg("lifetrac/v25/cmd/control",
                                        _make_control_payload(0xCAFE)))
        (source_id, queued_seq, pt), = _drain_queue(b)
        nonce = build_nonce(source_id, queued_seq, now_s=0,
                            random_tail=b"\x00" * 5)
        # Nonce layout: [source_id | seq_lo | seq_hi | t(4) | rand(5)]
        self.assertEqual(nonce[0], SRC_BASE)
        nonce_seq = struct.unpack("<H", nonce[1:3])[0]
        header_seq = struct.unpack("<H", pt[3:5])[0]
        self.assertEqual(
            nonce_seq, header_seq,
            "GCM nonce seq bytes MUST equal cleartext-header seq bytes; "
            "divergence is the IP-102 regression class.")
        self.assertEqual(nonce_seq, queued_seq)

    def test_NS_A3_hundred_messages_all_seqs_agree_and_are_unique(self) -> None:
        # Acceptance criterion verbatim: "publish 100 control messages,
        # asserts AEAD nonce seq == cleartext header seq for every frame".
        b = _make_bridge()
        for i in range(100):
            b._on_mqtt_message(
                None, None,
                _FakeMqttMsg("lifetrac/v25/cmd/control",
                             _make_control_payload((i * 37) & 0xFFFF)))
        items = _drain_queue(b)
        self.assertEqual(len(items), 100)
        seen_seqs: set[int] = set()
        for source_id, queued_seq, pt in items:
            hdr = parse_header(pt)
            self.assertEqual(hdr.sequence_num, queued_seq,
                             "header seq diverged from queued nonce seq")
            self.assertEqual(hdr.source_id, SRC_BASE)
            self.assertEqual(hdr.frame_type, FT_CONTROL)
            self.assertTrue(verify_crc(pt))
            seen_seqs.add(queued_seq)
        self.assertEqual(len(seen_seqs), 100,
                         "100 reservations must produce 100 distinct seqs")
        # And monotonic within the 16-bit window (no wrap in this test).
        self.assertEqual(sorted(seen_seqs), list(range(100)))

    def test_NS_A4_no_replay_false_rejects_for_100_fresh_seqs(self) -> None:
        # Acceptance criterion verbatim: "asserts no replay-window false-rejects."
        # Drive a tractor-side ReplayWindow with the seqs the bridge would
        # actually emit; every frame must be accepted as fresh.
        b = _make_bridge()
        for _ in range(100):
            b._on_mqtt_message(
                None, None,
                _FakeMqttMsg("lifetrac/v25/cmd/control",
                             _make_control_payload(0)))
        items = _drain_queue(b)
        window = ReplayWindow()
        for _src, queued_seq, _pt in items:
            self.assertTrue(
                window.check_and_update(queued_seq),
                f"replay window falsely rejected fresh seq={queued_seq} — "
                "a thread-through bug would surface here.")

    def test_NS_A5_malformed_control_payload_does_not_burn_seq(self) -> None:
        # Wrong length / bad CRC must be dropped BEFORE _reserve_tx_seq —
        # otherwise an attacker spamming garbage would advance the nonce
        # counter and accelerate persistent-store wear (and on a NonceStore
        # the cost is real disk I/O per reservation).
        b = _make_bridge()
        # Wrong length.
        b._on_mqtt_message(None, None,
                           _FakeMqttMsg("lifetrac/v25/cmd/control", b"\x00" * 5))
        # Right length, broken CRC.
        bad = bytearray(_make_control_payload(0))
        bad[-1] ^= 0xFF
        b._on_mqtt_message(None, None,
                           _FakeMqttMsg("lifetrac/v25/cmd/control", bytes(bad)))
        self.assertEqual(_drain_queue(b), [],
                         "malformed payloads must not enqueue a frame")
        self.assertEqual(b.tx_seq, 0,
                         "malformed payloads must not advance tx_seq — "
                         "rejection happens before _reserve_tx_seq")


# ---------------------------------------------------------------------------
# NS-B: opcode-only command paths (estop / camera_select / req_keyframe).
# ---------------------------------------------------------------------------

class NS_B_CommandPaths(unittest.TestCase):
    """``cmd/estop``, ``cmd/camera_select``, ``cmd/req_keyframe`` — each
    builds its own ``pack_command(seq, opcode)`` frame and MUST pass that
    same seq into ``_tx(nonce_seq=...)``."""

    def _assert_thread_through(self, b: Bridge, expected_opcode: int,
                               expected_seq: int) -> bytes:
        items = _drain_queue(b)
        self.assertEqual(len(items), 1)
        source_id, queued_seq, pt = items[0]
        self.assertEqual(source_id, SRC_BASE)
        self.assertEqual(queued_seq, expected_seq)
        hdr = parse_header(pt)
        self.assertEqual(hdr.frame_type, FT_COMMAND)
        self.assertEqual(hdr.sequence_num, queued_seq,
                         "command-path header seq must equal nonce_seq (IP-102)")
        # Opcode lives at offset HEADER_LEN.
        self.assertEqual(pt[HEADER_LEN], expected_opcode)
        self.assertTrue(verify_crc(pt))
        return pt

    def test_NS_B_estop_threads_seq(self) -> None:
        b = _make_bridge()
        b._on_mqtt_message(None, None,
                           _FakeMqttMsg("lifetrac/v25/cmd/estop", b""))
        # CMD_ESTOP opcode value comes from lora_proto.
        from lora_proto import CMD_ESTOP
        self._assert_thread_through(b, CMD_ESTOP, expected_seq=0)

    def test_NS_B2_camera_select_threads_seq(self) -> None:
        b = _make_bridge()
        b._on_mqtt_message(None, None,
                           _FakeMqttMsg("lifetrac/v25/cmd/camera_select",
                                        bytes([0x02])))
        from lora_proto import CMD_CAMERA_SELECT
        pt = self._assert_thread_through(b, CMD_CAMERA_SELECT, expected_seq=0)
        # Camera-id arg byte must follow the opcode untouched.
        self.assertEqual(pt[HEADER_LEN + 1], 0x02)

    def test_NS_B3_req_keyframe_threads_seq(self) -> None:
        b = _make_bridge()
        b._on_mqtt_message(None, None,
                           _FakeMqttMsg("lifetrac/v25/cmd/req_keyframe", b""))
        from lora_proto import CMD_REQ_KEYFRAME
        self._assert_thread_through(b, CMD_REQ_KEYFRAME, expected_seq=0)

    def test_NS_B4_camera_select_invalid_id_does_not_burn_seq(self) -> None:
        b = _make_bridge()
        b._on_mqtt_message(None, None,
                           _FakeMqttMsg("lifetrac/v25/cmd/camera_select",
                                        bytes([0x99])))
        self.assertEqual(_drain_queue(b), [])
        self.assertEqual(b.tx_seq, 0,
                         "invalid camera_id must reject before _reserve_tx_seq")

    def test_NS_B5_camera_select_wrong_length_does_not_burn_seq(self) -> None:
        b = _make_bridge()
        b._on_mqtt_message(None, None,
                           _FakeMqttMsg("lifetrac/v25/cmd/camera_select",
                                        b"\x00\x01"))
        self.assertEqual(_drain_queue(b), [])
        self.assertEqual(b.tx_seq, 0)


# ---------------------------------------------------------------------------
# NS-C: cross-arm interleave — every command type shares ONE counter.
# ---------------------------------------------------------------------------

class NS_C_CrossArmInterleave(unittest.TestCase):
    """All four ``_on_mqtt_message`` arms must draw from the SAME seq
    counter so two concurrent arms cannot reserve the same value."""

    def test_NS_C_interleaved_arms_produce_strictly_monotonic_seqs(self) -> None:
        b = _make_bridge()
        topics = [
            ("lifetrac/v25/cmd/control", _make_control_payload(0xAAAA)),
            ("lifetrac/v25/cmd/estop", b""),
            ("lifetrac/v25/cmd/req_keyframe", b""),
            ("lifetrac/v25/cmd/camera_select", bytes([0x01])),
            ("lifetrac/v25/cmd/control", _make_control_payload(0xBBBB)),
            ("lifetrac/v25/cmd/req_keyframe", b""),
        ]
        for t, p in topics:
            b._on_mqtt_message(None, None, _FakeMqttMsg(t, p))
        items = _drain_queue(b)
        # Items come back in priority order, not enqueue order, so we need
        # to verify uniqueness + 0..N-1 coverage rather than a sequence.
        seqs = sorted(it[1] for it in items)
        self.assertEqual(seqs, list(range(len(topics))),
                         "shared counter must yield 0..N-1 across all arms")
        # Every queued frame's cleartext header must still match its seq.
        for source_id, queued_seq, pt in items:
            hdr = parse_header(pt)
            self.assertEqual(hdr.sequence_num, queued_seq,
                             "cross-arm interleave broke header/nonce thread-through")

    def test_NS_C2_reserve_tx_seq_increments_and_wraps_at_16bit(self) -> None:
        # If tx_seq ever escapes its 16-bit mask the GCM nonce's 2-byte
        # seq slot truncates and IP-102's invariant (header seq ==
        # nonce seq) silently breaks at wrap-around.
        b = _make_bridge()
        b.tx_seq = 0xFFFE
        s1 = b._reserve_tx_seq()
        s2 = b._reserve_tx_seq()
        s3 = b._reserve_tx_seq()
        self.assertEqual((s1, s2, s3), (0xFFFE, 0xFFFF, 0))
        self.assertEqual(b.tx_seq, 1,
                         "tx_seq must wrap at 16 bits to match the nonce seq slot")


# ---------------------------------------------------------------------------
# NS-D: source-grep tripwire — every _tx call from the bridge must
# either pass nonce_seq or be the explicit "fresh seq" path.
# ---------------------------------------------------------------------------

class NS_D_SourceGrepTripwire(unittest.TestCase):
    """Defend against a future refactor that drops ``nonce_seq=`` from one
    of the ``_on_mqtt_message`` arms."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.src = Path(lora_bridge.__file__).read_text(encoding="utf-8")

    def test_NS_D_on_mqtt_message_calls_tx_with_nonce_seq(self) -> None:
        # Capture the body of _on_mqtt_message and assert every self._tx(
        # call inside it passes nonce_seq=.
        import re
        m = re.search(
            r"def\s+_on_mqtt_message\s*\([^)]*\)\s*:\n"
            r"(?P<body>(?:[ \t]+[^\n]*\n|\n)+?)(?=^\s{0,4}def\s)",
            self.src,
            re.MULTILINE,
        )
        self.assertIsNotNone(m, "_on_mqtt_message body not found")
        body = m.group("body")
        tx_calls = re.findall(r"self\._tx\([^)]*\)", body)
        self.assertGreaterEqual(len(tx_calls), 4,
                                "expected at least 4 _tx() calls in _on_mqtt_message "
                                "(control + estop + camera_select + req_keyframe)")
        for call in tx_calls:
            self.assertIn(
                "nonce_seq=", call,
                f"_on_mqtt_message _tx call missing nonce_seq=: {call!r} — "
                "this is the original IP-102 bug surface."
            )


if __name__ == "__main__":
    unittest.main()
