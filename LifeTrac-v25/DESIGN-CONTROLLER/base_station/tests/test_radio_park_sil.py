"""2026-10-24 (RS-12.21) — the firmware park command, host side.

`HostLink.radio_park()` (firmware/x8_lora_bootloader_helper/
method_g_stage1_probe.py) sends HOST_TYPE_RADIO_PARK_REQ (0x04) and sorts
the answer: RADIO_PARK_URC (0x84: a status byte, then the RegOpMode
readback), ERR_PROTO UNKNOWN_TYPE (an image from before the command), any
other ERR_PROTO, or nothing within the timeout.

`bench_tools/radio_park.py` parks through it when the firmware answers
(persistent: the host_parked flag gates every RX arm path until a host RXCONT
write, a profile activation or a reset) and otherwise falls back to the
STANDBY -> SLEEP register writes it has always made (transient: the scan
walker re-arms RXCONT until the scan fails out into FAILED). Its last line
says which path parked the radio and whether the park is persistent; its
PARK_OK / PARK_TRANSIENT / PARK_FAIL lines and exit codes 0/2/3/4/5 keep
their meaning. 2026-10-24 review: the VER reply's capability bit 7
(HOST_CAP_RADIO_PARK) picks the path -- an image that advertises the command
is asked again on silence (a late answer counts) and never gets the register
writes, which cannot hold on it -- and the settle outlasts the firmware's
slowest re-arm, the 2 s scan dwell.

Everything runs against a scripted L072 (FakeParkL072) that stands in for
`os` under the real HostLink and models the firmware's RX owners re-arming
RXCONT one scan dwell (SX1276_RX_SCAN_DWELL_MS, read from the firmware
header) after the radio left RX -- no hardware, no radio. The firmware's own
unpark path (a host raw RXCONT write) is pinned by check-radio-park
(firmware/murata_l072/bench/host_proto/radio_park.c), not here.
"""
import contextlib
import io
import json
import os
import re
import sys
import threading
import time
import unittest
from unittest import mock

_BS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_DC_DIR = os.path.dirname(_BS_DIR)
_HELPER_DIR = os.path.join(_DC_DIR, "firmware", "x8_lora_bootloader_helper")
_BT_DIR = os.path.join(_HELPER_DIR, "bench_tools")
for _p in (_HELPER_DIR, _BT_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import method_g_stage1_probe as mg  # noqa: E402
import radio_park  # noqa: E402

REG_OP_MODE = 0x01
RXCONT, STANDBY, SLEEP = 0x85, 0x81, 0x80
CAP_SPLIT, CAP_OLD = 0xFF, 0x7F        # VER cap_bitmap with / without bit 7


def _fw_define(header: str, name: str) -> int:
    """A `#define NAME <n>U` of the L072 firmware headers."""
    path = os.path.join(_DC_DIR, "firmware", "murata_l072", "include", header)
    with open(path, encoding="utf-8") as fh:
        m = re.search(rf"#define\s+{name}\s+(\d+)U", fh.read())
    return int(m.group(1))


# The firmware's slowest RX re-arm: the scan walker's next step, at most one
# channel dwell after the radio left RX (2 s since RS-12.21 d2), and a slot.
DWELL_S = _fw_define("sx1276_rx_scan_policy.h", "SX1276_RX_SCAN_DWELL_MS") / 1000.0
SLOT_S = _fw_define("sx1276_fhss_clock.h", "SX1276_FHSS_SLOT_MS") / 1000.0


def err_proto(offending_type: int, err_code: int, detail: int = 0) -> bytes:
    """The 5-byte ERR_PROTO payload host_uart_send_err_proto() builds."""
    return bytes([offending_type, 1, err_code, detail & 0xFF, detail >> 8])


class FakeParkL072:
    """Stands in for `os` inside method_g_stage1_probe. Answers VER, register
    read/write and RADIO_PARK_REQ the way host_cmd.c does for the image `fw`:

      split       answers the park command: sets host_parked, SLEEP, URC
                  {PARKED or ALREADY_PARKED, RegOpMode};
      leaky       the same, but the gate misses an arm path (a defect);
      refuse      URC {REFUSED, RegOpMode}, radio unchanged;
      old         ERR_PROTO UNKNOWN_TYPE (host_cmd.c dispatch default);
      bad_length  ERR_PROTO BAD_LENGTH (knows the command, not the frame);
      silent      no answer at all.

    `walking`: an RX owner (scan walker, retune, slot follower) still has
    work, so advance() -- the firmware's clock, moved by radio_park's sleeps
    -- re-arms RXCONT DWELL_S after the radio left RX (the worst phase of the
    walker's dwell) unless the park gate holds. A host REG_WRITE RegOpMode
    RXCONT clears the park, as sx1276_rx_note_external_arm() does. `cap`:
    the VER reply's capability bitmap (default: bit 7 for an image that
    answers the command); `late`: answer the park command only when the next
    host frame arrives (after the caller's timeout)."""
    O_RDWR = O_NOCTTY = O_NONBLOCK = 0

    def __init__(self, fw="split", opmode=RXCONT, walking=True, cap=None):
        self._lock = threading.Lock()
        self._out = bytearray()        # L072 -> host
        self._in = bytearray()         # host -> L072 (partial frame)
        self.host_frames = []
        self.fw = fw
        self.opmode = opmode
        self.walking = walking
        self.cap = cap
        self.parked = False
        self.before_park = []          # [(type, payload, seq)] ahead of the reply
        self.answer_ver = True
        self.answer_reg_read = True
        self.ver_payload = None        # a VER_URC payload as is (None: by cap)
        self.late = False
        self._late_seq = None
        self.now = 0.0                 # firmware clock, s (radio_park's sleeps)
        self.left_rx_at = None         # when the radio last left RX

    # -- os surface used by HostLink --
    def open(self, *_a, **_k):
        return 99

    def close(self, _fd):
        pass

    def read(self, _fd, n):
        with self._lock:
            if not self._out:
                raise BlockingIOError
            chunk = bytes(self._out[:n])
            del self._out[:n]
            return chunk

    def write(self, _fd, data):
        self._in.extend(data)
        while True:
            try:
                start = self._in.index(0)
                end = self._in.index(0, start + 1)
            except ValueError:
                break
            body = bytes(self._in[start + 1:end])
            del self._in[:end + 1]
            if body:
                self._handle(mg.parse_frame(body))
        return len(data)

    # -- scripting --
    def push(self, ftype, payload, seq=0):
        with self._lock:
            self._out.extend(mg.build_frame(ftype, seq, payload))

    def _set_opmode(self, value):
        if (self.opmode & 0x07) == 0x05 and (value & 0x07) != 0x05:
            self.left_rx_at = self.now
        self.opmode = value

    def advance(self, s):
        self.now += s
        if (self.walking and (not self.parked or self.fw == "leaky")
                and (self.opmode & 0x07) != 0x05 and self.left_rx_at is not None
                and self.now - self.left_rx_at >= DWELL_S):
            self.opmode = RXCONT

    def ver_cap(self):
        if self.cap is not None:
            return self.cap
        return CAP_SPLIT if self.fw in ("split", "leaky", "refuse") else CAP_OLD

    def sent(self, ftype):
        return [f for f in self.host_frames if f["type"] == ftype]

    def _handle(self, fr):
        self.host_frames.append(fr)
        t, seq, p = fr["type"], fr["seq"], fr["payload"]
        if self._late_seq is not None:          # the delayed park answer first
            late, self._late_seq = self._late_seq, None
            self._park_reply(late)
        if t == mg.HOST_TYPE_VER_REQ:
            if self.answer_ver:
                self.push(mg.HOST_TYPE_VER_URC,
                          self.ver_payload if self.ver_payload is not None else
                          bytes(8) + self.ver_cap().to_bytes(4, "little"), seq)
        elif t == mg.HOST_TYPE_REG_READ_REQ:
            if self.answer_reg_read:
                self.push(mg.HOST_TYPE_REG_DATA_URC, bytes(
                    [p[0], self.opmode if p[0] == REG_OP_MODE else 0]), seq)
        elif t == 0x31:                                     # REG_WRITE_REQ
            if p[0] == REG_OP_MODE:
                self._set_opmode(p[1])
                if (p[1] & 0x07) == 0x05:
                    self.parked = False
            self.push(0xB1, bytes(p[:2]), seq)
        elif t == mg.HOST_TYPE_RADIO_PARK_REQ:
            for ftype, payload, s in self.before_park:
                self.push(ftype, payload, s)
            if self.late:
                self._late_seq = seq
            else:
                self._park_reply(seq)
        else:
            self.push(mg.HOST_TYPE_ERR_PROTO_URC, err_proto(
                t, mg.HOST_ERR_PROTO_UNKNOWN_TYPE, t), seq)

    def _park_reply(self, seq):
        t = mg.HOST_TYPE_RADIO_PARK_REQ
        if self.fw == "old":
            self.push(mg.HOST_TYPE_ERR_PROTO_URC, err_proto(
                t, mg.HOST_ERR_PROTO_UNKNOWN_TYPE, t), seq)
        elif self.fw == "bad_length":
            self.push(mg.HOST_TYPE_ERR_PROTO_URC, err_proto(
                t, mg.HOST_ERR_PROTO_BAD_LENGTH, 0), seq)
        elif self.fw == "refuse":
            self.push(mg.HOST_TYPE_RADIO_PARK_URC, bytes(
                [mg.RADIO_PARK_STATUS_REFUSED, self.opmode]), seq)
        elif self.fw in ("split", "leaky"):
            status = (mg.RADIO_PARK_STATUS_ALREADY_PARKED if self.parked
                      else mg.RADIO_PARK_STATUS_PARKED)
            self.parked = True
            self._set_opmode(SLEEP)
            self.push(mg.HOST_TYPE_RADIO_PARK_URC,
                      bytes([status, self.opmode]), seq)
        # "silent": nothing


class _FakeTime:
    """radio_park's `time`: its only use is sleep(), which moves the fake
    L072's clock (FakeParkL072.advance) by the time slept."""

    def __init__(self, fake):
        self.fake = fake
        self.slept = []

    def sleep(self, s):
        self.slept.append(s)
        self.fake.advance(s)


class _Case(unittest.TestCase):
    def setUp(self):
        self.fake = FakeParkL072()
        for p in (mock.patch.object(mg, "os", self.fake),
                  mock.patch.object(mg, "configure_uart", lambda *_a: None)):
            p.start()
            self.addCleanup(p.stop)
        self.stdout = io.StringIO()
        quiet = contextlib.redirect_stdout(self.stdout)
        quiet.__enter__()
        self.addCleanup(quiet.__exit__, None, None, None)

    def link(self):
        return mg.HostLink("/dev/fake", "921600")


# ---------------------------------------------------------------------------
# 1. HostLink.radio_park
# ---------------------------------------------------------------------------
class HostLinkRadioPark(_Case):
    def test_split_image_parks(self):
        link = self.link()
        seq = link.seq                                 # the next send's seq
        got = link.radio_park()
        self.assertEqual(got, {"answer": "urc", "status": 0,
                               "status_name": "PARKED", "opmode": SLEEP,
                               "detail": "", "seq": seq, "answer_seq": seq})
        req = self.fake.sent(mg.HOST_TYPE_RADIO_PARK_REQ)
        self.assertEqual([(f["payload"], f["ver"]) for f in req],
                         [(b"", mg.HOST_PROTOCOL_VER)])          # empty payload
        self.assertEqual(link.radio_park()["status_name"], "ALREADY_PARKED")

    def test_old_image_says_unknown_type(self):
        self.fake.fw = "old"
        got = self.link().radio_park()
        self.assertEqual(got["answer"], "unknown_type")
        self.assertIn("offending_type=0x04", got["detail"])
        self.assertIn("err_code=0x02(UNKNOWN_TYPE)", got["detail"])
        self.assertEqual((got["status"], got["opmode"]), (None, None))
        self.assertEqual(self.fake.opmode, RXCONT)                # untouched

    def test_other_err_proto_is_not_unknown_type(self):
        self.fake.fw = "bad_length"
        got = self.link().radio_park()
        self.assertEqual(got["answer"], "err_proto")
        self.assertIn("err_code=0x03(BAD_LENGTH)", got["detail"])

    def test_no_reply_within_the_timeout(self):
        self.fake.fw = "silent"
        t0 = time.monotonic()
        link = self.link()
        seq = link.seq
        got = link.radio_park(timeout=0.3)
        self.assertLess(time.monotonic() - t0, 2.0)
        self.assertEqual(got, {"answer": "no_reply", "status": None,
                               "status_name": None, "opmode": None,
                               "detail": "", "seq": seq, "answer_seq": None})

    def test_a_late_answer_to_an_earlier_try(self):
        """2026-10-24 review: the answer to a try that timed out lands during
        the next one. Named in earlier_seqs it is the answer; otherwise it is
        dropped as stale, as before."""
        self.fake.late = True                 # answers when the next frame lands
        link = self.link()
        first = link.radio_park(timeout=0.3)
        self.assertEqual(first["answer"], "no_reply")
        got = link.radio_park(timeout=0.5, earlier_seqs=[first["seq"]])
        self.assertEqual((got["answer"], got["status_name"], got["answer_seq"]),
                         ("urc", "PARKED", first["seq"]))
        self.assertNotEqual(got["seq"], first["seq"])

    def test_a_late_answer_not_asked_for_is_dropped(self):
        self.fake.late = True
        link = self.link()
        first = link.radio_park(timeout=0.3)
        got = link.radio_park(timeout=0.3)     # the first's answer lands: stale
        self.assertEqual((first["answer"], got["answer"]), ("no_reply", "no_reply"))
        self.assertEqual(list(link.urc_queue), [])

    def test_refused_and_short_payloads(self):
        self.fake.fw = "refuse"
        got = self.link().radio_park()
        self.assertEqual((got["answer"], got["status"], got["status_name"],
                          got["opmode"]), ("urc", 2, "REFUSED", RXCONT))
        link = self.link()
        seq = link.seq                                 # the next send's seq
        self.fake.fw = "silent"
        self.fake.before_park = [(mg.HOST_TYPE_RADIO_PARK_URC, b"\x00", seq)]
        got = link.radio_park(timeout=0.5)
        self.assertEqual((got["status"], got["status_name"], got["opmode"]),
                         (0, "PARKED", None))
        seq = link.seq
        self.fake.before_park = [(mg.HOST_TYPE_RADIO_PARK_URC, b"", seq)]
        got = link.radio_park(timeout=0.5)
        self.assertEqual((got["answer"], got["status"], got["status_name"]),
                         ("urc", None, None))
        seq = link.seq
        self.fake.before_park = [(mg.HOST_TYPE_RADIO_PARK_URC, b"\x07\x80", seq)]
        self.assertEqual(link.radio_park(timeout=0.5)["status_name"], "UNKNOWN_0x07")

    def test_other_frames_are_requeued_in_order_and_seen_once(self):
        """URCs that land before the answer go back on urc_queue as
        request() leaves them; a FAULT among them reaches on_fault_urc once;
        a stale RADIO_PARK_URC (an earlier seq) is dropped, an ERR_PROTO for
        another seq is kept."""
        link = self.link()
        faults = []
        link.on_fault_urc = lambda fr, ids: faults.append(fr["payload"][:2])
        fault = bytes([0x0D, 0xA3]) + bytes(22)
        stats = bytes(8)
        other_err = err_proto(0x30, mg.HOST_ERR_PROTO_BAD_LENGTH, 1)
        self.fake.before_park = [
            (mg.HOST_TYPE_FAULT_URC, fault, 0),
            (mg.HOST_TYPE_RADIO_PARK_URC, bytes([0, SLEEP]), 0x7777),   # stale
            (mg.HOST_TYPE_STATS_URC, stats, 0),
            (mg.HOST_TYPE_ERR_PROTO_URC, other_err, 0x6666),
        ]
        got = link.radio_park()
        self.assertEqual(got["answer"], "urc")
        self.assertEqual([(f["type"], bytes(f["payload"])) for f in link.urc_queue], [
            (mg.HOST_TYPE_FAULT_URC, fault), (mg.HOST_TYPE_STATS_URC, stats),
            (mg.HOST_TYPE_ERR_PROTO_URC, other_err)])
        self.assertEqual(faults, [b"\x0d\xa3"])
        again = link.read_frames(0.2)                     # the queued copies
        self.assertEqual(len(again), 3)
        self.assertEqual(faults, [b"\x0d\xa3"])           # not seen twice

    def test_queued_frames_before_the_call_survive(self):
        link = self.link()
        self.fake.push(mg.HOST_TYPE_STATS_URC, bytes(8))
        link.urc_queue.extend(link.read_frames(0.2))     # parked by a request()
        self.assertEqual(len(link.urc_queue), 1)
        self.assertEqual(link.radio_park()["answer"], "urc")
        self.assertEqual([f["type"] for f in link.urc_queue], [mg.HOST_TYPE_STATS_URC])


# ---------------------------------------------------------------------------
# 2. bench_tools/radio_park.py
# ---------------------------------------------------------------------------
class RadioParkTool(_Case):
    def run_tool(self, **fake):
        for k, v in fake.items():
            setattr(self.fake, k, v)
        self.time = _FakeTime(self.fake)
        with mock.patch.object(radio_park, "time", self.time), \
                mock.patch.object(radio_park, "PARK_CMD_TIMEOUT_S", 0.3):
            rc = radio_park.main(["/dev/fake"])
        last = self.stdout.getvalue().strip().splitlines()[-1]
        verdict, _sp, blob = last.partition(" ")
        return rc, verdict, json.loads(blob)

    def reg_writes(self):
        return [bytes(f["payload"]) for f in self.fake.sent(0x31)]

    def test_split_image_parks_persistently(self):
        rc, verdict, info = self.run_tool(fw="split", walking=True)
        self.assertEqual((rc, verdict), (0, "PARK_OK"))
        self.assertEqual(info["path"], "firmware")
        self.assertIs(info["persistent"], True)
        self.assertEqual(info["park_status"], "PARKED")
        self.assertEqual((info["opmode_readback"], info["opmode_after_settle"]),
                         ("0x80", "0x80"))
        self.assertEqual(info["settle_s"], radio_park.SETTLE_S)
        self.assertIn("RXCONT write", info["holds"])
        self.assertNotIn("park_cmd", info)
        self.assertEqual(self.reg_writes(), [])           # no register writes
        self.assertIn(radio_park.SETTLE_S, self.time.slept)
        # the walker had work through the settle and the gate held
        self.assertEqual(self.fake.opmode, SLEEP)
        self.assertEqual((info["cap_bitmap"], info["park_tries"]), ("0x000000FF", 1))
        self.assertNotIn("late", info)

    def test_split_image_already_parked(self):
        self.fake.parked, self.fake.opmode = True, SLEEP
        rc, verdict, info = self.run_tool(fw="split")
        self.assertEqual((rc, verdict, info["park_status"], info["persistent"]),
                         (0, "PARK_OK", "ALREADY_PARKED", True))

    def test_old_image_scan_failed_out_parks_by_register(self):
        rc, verdict, info = self.run_tool(fw="old", walking=False)
        self.assertEqual((rc, verdict), (0, "PARK_OK"))
        self.assertEqual(info["path"], "register")
        self.assertIs(info["persistent"], False)
        self.assertEqual(info["park_cmd"], "unknown_type")
        self.assertIn("UNKNOWN_TYPE", info["park_cmd_err"])
        self.assertIn("until the scan fails out", info["holds"])
        self.assertNotIn("park_status", info)
        self.assertEqual(self.reg_writes(), [bytes([REG_OP_MODE, STANDBY]),
                                             bytes([REG_OP_MODE, SLEEP])])
        self.assertEqual((info["opmode_readback"], info["opmode_after_settle"]),
                         ("0x80", "0x80"))
        self.assertEqual((info["cap_bitmap"], info["park_tries"]), ("0x0000007F", 1))

    def test_old_image_scan_walking_is_transient(self):
        rc, verdict, info = self.run_tool(fw="old", walking=True)
        self.assertEqual((rc, verdict), (5, "PARK_TRANSIENT"))
        self.assertEqual((info["path"], info["persistent"], info["park_cmd"]),
                         ("register", False, "unknown_type"))
        self.assertEqual((info["opmode_readback"], info["opmode_after_settle"]),
                         ("0x80", "0x85"))

    def test_settle_outlasts_the_slowest_rearm(self):
        """2026-10-24 review: SETTLE_S must outlast a scan channel dwell (2 s
        since RS-12.21 d2, read from the firmware header) and a slot, or a
        walker that re-arms late in the settle reads PARK_OK -- as the 1.5 s
        settle did against the 2 s dwell (shown with it patched back)."""
        self.assertGreaterEqual(radio_park.SETTLE_S, DWELL_S + SLOT_S)
        with mock.patch.object(radio_park, "SETTLE_S", 1.5):
            rc, verdict, _info = self.run_tool(fw="old", walking=True)
        self.assertEqual((rc, verdict), (0, "PARK_OK"))   # the walker not yet back

    def test_split_image_silent_is_a_fail_with_no_register_write(self):
        """2026-10-24 review: an image that advertises the command (VER cap bit
        7) is asked PARK_CMD_TRIES times; unanswered it is a PARK_FAIL that made
        no register write -- one cannot hold on it (its scan restarts after a
        final RX_SCAN_FAILED and re-arms RX every dwell)."""
        rc, verdict, info = self.run_tool(fw="silent", cap=CAP_SPLIT, walking=True)
        self.assertEqual((rc, verdict), (4, "PARK_FAIL"))
        self.assertEqual((info["path"], info["persistent"], info["park_cmd"],
                          info["park_tries"], info["cap_bitmap"]),
                         ("firmware", False, "no_reply", radio_park.PARK_CMD_TRIES,
                          "0x000000FF"))
        self.assertIn("did not answer it in 3 tries; a register SLEEP write cannot "
                      "hold on this image", info["reason"])
        self.assertEqual(len(self.fake.sent(mg.HOST_TYPE_RADIO_PARK_REQ)),
                         radio_park.PARK_CMD_TRIES)
        self.assertEqual(self.reg_writes(), [])
        self.assertEqual(self.fake.opmode, RXCONT)                # untouched
        # one that says UNKNOWN_TYPE although it advertises the command: no retry
        self.fake.host_frames.clear()
        self.stdout.seek(0)
        self.stdout.truncate()
        rc, verdict, info = self.run_tool(fw="old", cap=CAP_SPLIT)
        self.assertEqual((rc, verdict, info["park_cmd"], info["park_tries"]),
                         (4, "PARK_FAIL", "unknown_type", 1))
        self.assertIn("refused it in 1 try", info["reason"])
        self.assertIn("UNKNOWN_TYPE", info["park_cmd_err"])
        self.assertEqual(self.reg_writes(), [])

    def test_split_image_late_answer_parks(self):
        """2026-10-24 review: the PARKED answer to a try that timed out lands
        during the next try and is the answer (the split image is parked); the
        old tool drained it and fell back to the register path."""
        rc, verdict, info = self.run_tool(fw="split", late=True, walking=True)
        self.assertEqual((rc, verdict), (0, "PARK_OK"))
        self.assertEqual((info["path"], info["persistent"], info["park_status"],
                          info["park_tries"], info["late"]),
                         ("firmware", True, "PARKED", 2, True))
        self.assertEqual(self.reg_writes(), [])
        self.assertEqual(self.fake.opmode, SLEEP)

    def test_short_ver_reply_is_read_as_an_image_without_the_command(self):
        self.fake.ver_payload = bytes(11)
        rc, verdict, info = self.run_tool(fw="old", walking=False)
        self.assertEqual((rc, verdict, info["path"], info["cap_bitmap"]),
                         (0, "PARK_OK", "register", None))

    def test_no_reply_falls_back_to_the_register_path(self):
        # an image whose VER bitmap lacks bit 7 (the fake's default for "silent")
        rc, verdict, info = self.run_tool(fw="silent", walking=False)
        self.assertEqual((rc, verdict, info["path"], info["park_cmd"]),
                         (0, "PARK_OK", "register", "no_reply"))
        self.assertNotIn("park_cmd_err", info)
        self.assertEqual(len(self.reg_writes()), 2)

    def test_other_err_proto_falls_back_with_its_text(self):
        rc, verdict, info = self.run_tool(fw="bad_length", walking=False)
        self.assertEqual((rc, verdict, info["path"], info["park_cmd"]),
                         (0, "PARK_OK", "register", "err_proto"))
        self.assertIn("BAD_LENGTH", info["park_cmd_err"])

    def test_refused_is_a_fail_and_touches_nothing(self):
        rc, verdict, info = self.run_tool(fw="refuse")
        self.assertEqual((rc, verdict), (4, "PARK_FAIL"))
        self.assertEqual((info["path"], info["park_status"], info["persistent"],
                          info["opmode_readback"]),
                         ("firmware", "REFUSED", False, "0x85"))
        self.assertEqual(self.reg_writes(), [])
        self.assertNotIn(radio_park.SETTLE_S, self.time.slept)

    def test_a_leaky_gate_is_transient_on_the_firmware_path(self):
        rc, verdict, info = self.run_tool(fw="leaky", walking=True)
        self.assertEqual((rc, verdict), (5, "PARK_TRANSIENT"))
        self.assertEqual((info["path"], info["persistent"]), ("firmware", False))
        self.assertEqual((info["opmode_readback"], info["opmode_after_settle"]),
                         ("0x80", "0x85"))

    def test_ver_failure_keeps_exit_2(self):
        rc, verdict, info = self.run_tool(answer_ver=False)
        self.assertEqual((rc, verdict, info["stage"]), (2, "PARK_FAIL", "ver"))
        self.assertEqual(self.fake.sent(mg.HOST_TYPE_RADIO_PARK_REQ), [])

    def test_readback_failure_is_exit_3(self):
        rc, verdict, info = self.run_tool(fw="split", answer_reg_read=False)
        self.assertEqual((rc, verdict, info["stage"], info["path"]),
                         (3, "PARK_FAIL", "opmode", "firmware"))

    def test_park_line_is_last(self):
        """leg_post.sh / power_up_guard.sh read `tail -1` and match PARK_OK*."""
        self.run_tool(fw="split")
        lines = self.stdout.getvalue().strip().splitlines()
        self.assertTrue(lines[-1].startswith("PARK_OK {"), lines[-1])
        self.assertEqual(sum(ln.startswith("PARK_") for ln in lines), 1)


if __name__ == "__main__":
    unittest.main()
