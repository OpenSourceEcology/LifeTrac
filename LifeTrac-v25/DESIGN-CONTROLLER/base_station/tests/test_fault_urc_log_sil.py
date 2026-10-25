"""2026-10-24 — every FAULT_URC logged once, in one format, by both daemons.

Leg 6 of the 2026-10-24 FHSS round (archive radio_monitor_20261024_104056
_d28bc2ca): the base never locked the FHSS grid, and the leading hypothesis
(its cold-start scan ran out into the absorbing RX_SCAN_FAILED state) could
not be confirmed — image_rx_daemon dropped FAULT URCs at DEBUG, and the
only LOG line for one was image_tx_daemon's pipelined-TX `FAULT during
pipelined TX:` (wait_for_tx_done and drain_boot only print() them to
stdout, untimestamped). Both daemons now install `install_fault_urc_logger`
on their HostLink: one `FAULT_URC:` line per FAULT_URC, produced by
`format_fault_urc_line`, at INFO for BENIGN_FAULT_CODES and WARNING
otherwise, wherever the reader takes it off the wire. The legacy lines
stay, so tools count `FAULT_URC: code=0x.. sub=0x.. name=` lines only.

2026-10-24, later (the 0x0D split, FHSS_SCAN_FAILED_OPTIONS.md section 7):
RX_SCAN_FAILED keeps 0x0D and TX_FIFO_RB_MISMATCH moves to 0x0F. The decoder
names a 0x0D RX_SCAN_FAILED whenever its sub is a scan encoding (the
in-flight check survives as `basis=`, the only trace of the pre-split
ambiguity) and the pre-split FIFO fault (`fw=old`) otherwise; it no longer
writes AMBIGUOUS.

Contract pinned here:
  * the formatter on REAL payloads from the committed archives, incl. the
    0x0D decode (RX_SCAN_FAILED with its basis / the pre-split
    TX_FIFO_RB_MISMATCH), 0x0F, and the L072 uptime (payload bytes 20-23);
  * the line is seen by the case-sensitive FAULT greps the tools use, and
    is told apart from its legacy twins by `name=`;
  * the scan sub-byte model matches the firmware (sx1276_rx_scan_fail.h,
    host_types.h, a port of sx1276_rx_scan_fail_eval);
  * the legacy `format_fault_payload` text is byte-identical (tools grep it);
  * HostLink's observer sees each FAULT once (also through request()'s
    re-queue), tracks in-flight tx_ids (a resent tx_id, an ERR_PROTO
    refusal, staleness on time.monotonic()), and changes no frame a caller
    gets;
  * both daemons' FAULT paths, driven through a real HostLink on a scripted
    L072 (no hardware): boot drain, request(), command TX wait, RX loop /
    command drain, pipelined TX — and the non-FAULT handling around them.
"""
import contextlib
import glob
import io
import logging
import os
import re
import struct
import sys
import threading
import time
import unittest
from unittest import mock

_BS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_DC_DIR = os.path.dirname(_BS_DIR)
_HELPER_DIR = os.path.join(_DC_DIR, "firmware", "x8_lora_bootloader_helper")
_X8_DIR = os.path.join(_DC_DIR, "firmware", "tractor_x8")
_FW_INC = os.path.join(_DC_DIR, "firmware", "murata_l072", "include")
_EVIDENCE = os.path.join(_DC_DIR, "bench-evidence")
for _p in (_BS_DIR, _HELPER_DIR, _X8_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import method_g_stage1_probe as mg  # noqa: E402
import method_h_stage2_tx_probe_v2 as mh  # noqa: E402
import image_rx_daemon  # noqa: E402
from image_pipeline.reassemble import FRAGMENT_MAGIC  # noqa: E402
from lora_proto import (  # noqa: E402
    PHY_IMAGE_BW250, CMD_OP_REQ_KEYFRAME, CMD_OP_PROBE, pack_command_frame)

# Same paho stub the other tractor-daemon tests use.
try:
    import paho.mqtt.client  # noqa: F401
    _HAVE_PAHO = True
except ImportError:
    _HAVE_PAHO = False
if _HAVE_PAHO:
    with mock.patch("paho.mqtt.client.Client") as _mqtt_class:
        _mqtt_class.return_value.connect = mock.MagicMock()
        _mqtt_class.return_value.loop_start = mock.MagicMock()
        import image_tx_daemon  # noqa: E402
else:
    image_tx_daemon = None


def fault(code, sub, uptime_ms=0, pc=0, lr=0, psr=0, bfar=0) -> bytes:
    """A 24-byte FAULT_URC payload, laid out as host_cmd.c builds it."""
    return struct.pack("<BBH5I", code, sub, 0, pc, lr, psr, bfar, uptime_ms)


# Real FAULT_URC payloads, copied from the committed tractor logs.
# radio_monitor_20261024_104056_d28bc2ca/tx_daemon.log (leg 6, 2b_f3):
LEG6_ATTEMPT0 = bytes.fromhex("0d20000000000000000000000000000000000000dc840000")
LEG6_ATTEMPT2 = bytes.fromhex("0d020000000000000000000000000000000000003e6f0100")
# 2026-09-29 leg X lines (pinned in bench_tools/test_run_live_radio_monitor.ps1):
LEGX_ATTEMPT0 = bytes.fromhex("0d0000000000000000000000000000000000000084830000")
LEGX_FINAL = bytes.fromhex("0d8300000000000000000000000000000000000016e30100")
SETTLE_RX_INACTIVE = bytes.fromhex("0a00000000000000000000000000000000000000c50b0000")

PS1_FAULT_RE = re.compile(r"code=0x([0-9A-Fa-f]{2}) sub=0x([0-9A-Fa-f]{2})")
# The one-count key for tools (format_fault_urc_line docstring).
NEW_LINE_RE = re.compile(r"\bFAULT_URC: code=0x[0-9A-F]{2} sub=0x[0-9A-F]{2} name=\w")


def fault_urc_lines(records):
    return [r for r in records if r.getMessage().startswith("FAULT_URC:")]


# ---------------------------------------------------------------------------
# 1. The formatter
# ---------------------------------------------------------------------------
class FormatterOnArchivedPayloads(unittest.TestCase):
    def test_leg6_attempt0_is_a_scan_failure(self):
        """sub 0x20 = attempt 0, hw_suspect; only fragment 0 (tx_id 0x00)
        was ever in flight on the 1-fragment camera trains."""
        self.assertEqual(
            mg.format_fault_urc_line(LEG6_ATTEMPT0, frozenset({0x00})),
            "FAULT_URC: code=0x0D sub=0x20 name=RX_SCAN_FAILED attempt=0 "
            "final=0 warm=0 hw_suspect=1 crc_seen=0 basis=tx_id_not_in_flight "
            "tx_inflight=0x00 uptime_ms=34012 "
            "raw=0d20000000000000000000000000000000000000dc840000")

    def test_leg6_attempt2_is_60_s_later(self):
        line = mg.format_fault_urc_line(LEG6_ATTEMPT2, frozenset({0x00}))
        self.assertIn("name=RX_SCAN_FAILED attempt=2 final=0 warm=0 "
                      "hw_suspect=0 crc_seen=0", line)
        self.assertIn("uptime_ms=94014", line)
        # two 30 s scan windows after attempt 0: the attempt-1 URC (~64 s)
        # between them is in neither leg-6 log (sub 0x02 proves it was
        # sent); whether a final (~124 s) was ever emitted is unknown -- a
        # LOCK in between would have reset the retry count.
        self.assertEqual(94014 - 34012, 60002)

    def test_final_attempt(self):
        line = mg.format_fault_urc_line(LEGX_FINAL, frozenset())
        self.assertIn("name=RX_SCAN_FAILED attempt=3 final=1", line)
        self.assertIn("tx_inflight=- uptime_ms=123670 ", line)

    def test_sub_equal_to_an_in_flight_tx_id_keeps_its_basis(self):
        """Leg X's sub 0x00 against fragment 0 in flight. A scan encoding is
        RX_SCAN_FAILED after the 0x0D split (PR #166 named it AMBIGUOUS);
        basis=tx_id_in_flight keeps what still matters on a pre-split image:
        this URC could also be fragment 0's FIFO readback mismatch."""
        self.assertEqual(
            mg.format_fault_urc_line(LEGX_ATTEMPT0, frozenset({0x00})),
            "FAULT_URC: code=0x0D sub=0x00 name=RX_SCAN_FAILED attempt=0 final=0 "
            "warm=0 hw_suspect=0 crc_seen=0 basis=tx_id_in_flight "
            "tx_inflight=0x00 uptime_ms=33668 "
            "raw=0d0000000000000000000000000000000000000084830000")

    def test_no_tx_context_is_still_a_scan_failure(self):
        line = mg.format_fault_urc_line(LEG6_ATTEMPT0)
        self.assertIn("name=RX_SCAN_FAILED attempt=0 final=0 warm=0 "
                      "hw_suspect=1 crc_seen=0 basis=no_tx_context tx_inflight=? ",
                      line)

    def test_the_decoder_never_writes_ambiguous(self):
        for sub in range(256):
            for ctx in (None, frozenset(), frozenset({sub})):
                d = mg.decode_fault_0x0d(sub, ctx)
                self.assertIn(d["name"], ("RX_SCAN_FAILED", "TX_FIFO_RB_MISMATCH"))
                self.assertEqual(d["name"] == "RX_SCAN_FAILED",
                                 mg.rx_scan_fail_sub_valid(sub), hex(sub))
                self.assertEqual(d["old_fw"], d["name"] == "TX_FIFO_RB_MISMATCH")
                self.assertNotIn("AMBIGUOUS",
                                 mg.format_fault_urc_line(fault(0x0D, sub), ctx))

    def test_non_scan_sub_is_a_pre_split_fifo_mismatch_whatever_the_context(self):
        # 0xFE/0xFD: the command-frame tx_ids (hw_suspect+crc_seen both set);
        # 0x03: attempt 3 without `final`; 0x84: final with attempt 4. No
        # image after the 0x0D split sends these as 0x0D: fw=old says so.
        for sub in (0xFE, 0xFD, 0x03, 0x84, 0x30, 0x05):
            for ctx in (None, frozenset(), frozenset({sub})):
                line = mg.format_fault_urc_line(fault(0x0D, sub, 7), ctx)
                self.assertIn(f"name=TX_FIFO_RB_MISMATCH tx_id=0x{sub:02X} "
                              "fw=old basis=not_scan_encoding", line, (sub, ctx))
                self.assertNotIn("attempt=", line)

    def test_0x0f_is_the_fifo_mismatch(self):
        """After the split TX_FIFO_RB_MISMATCH is 0x0F, sub = the host's
        tx_id; the code says it all (no basis, no fw= token), whatever the
        sub would read as under 0x0D."""
        for sub in (0x00, 0x02, 0x83, 0xFD, 0xFE):
            for ctx in (None, frozenset(), frozenset({sub})):
                line = mg.format_fault_urc_line(fault(0x0F, sub, 180000), ctx)
                self.assertEqual(
                    line, f"FAULT_URC: code=0x0F sub=0x{sub:02X} "
                    f"name=TX_FIFO_RB_MISMATCH tx_id=0x{sub:02X} uptime_ms=180000 "
                    f"raw={fault(0x0F, sub, 180000).hex()}", (sub, ctx))
        self.assertEqual(mg.HOST_FAULT_CODE_TX_FIFO_RB_MISMATCH, 0x0F)
        self.assertEqual(mg.HOST_FAULT_CODE_TX_FIFO_RB_MISMATCH_PRE_SPLIT, 0x0D)
        self.assertEqual(mg.HOST_FAULT_CODE_RX_SCAN_FAILED, 0x0D)
        # the legacy text needs no decode: byte-identical shape
        self.assertEqual(mg.format_fault_payload(fault(0x0F, 0x02, 9)),
                         f"code=0x0F sub=0x02 raw={fault(0x0F, 0x02, 9).hex()}")

    def test_other_codes(self):
        self.assertEqual(
            mg.format_fault_urc_line(SETTLE_RX_INACTIVE),
            "FAULT_URC: code=0x0A sub=0x00 name=HOST_RX_INACTIVE "
            "reason=no_ingress_during_probe_window uptime_ms=3013 "
            "raw=0a00000000000000000000000000000000000000c50b0000")
        self.assertIn("name=HOST_RX_SEEN lanes=LPUART1 uptime_ms=812 ",
                      mg.format_fault_urc_line(fault(0x09, 0x01, 812)))
        self.assertIn("name=HOST_PARSE_ERROR parse_err_count_hint=1 ",
                      mg.format_fault_urc_line(fault(0x0B, 0x01)))
        self.assertIn("name=HOST_DIAG_MARK marks=VER_REQ_PARSED|VER_URC_SENT ",
                      mg.format_fault_urc_line(fault(0x0C, 0x09)))
        self.assertIn("name=TX_DONE_EARLY tx_id=0x05 ",
                      mg.format_fault_urc_line(fault(0x0E, 0x05)))
        self.assertIn("name=RADIO_OPMODE_DRIFT opmode=0x80 ",
                      mg.format_fault_urc_line(fault(0x04, 0x80)))
        self.assertIn("name=CLOCK_HSE_FAILED clock_source_id=1 ",
                      mg.format_fault_urc_line(fault(0x08, 0x01)))
        self.assertIn("code=0x7F sub=0x00 name=UNKNOWN uptime_ms=0 ",
                      mg.format_fault_urc_line(fault(0x7F, 0x00)))

    def test_replayed_cpu_fault_carries_its_registers(self):
        line = mg.format_fault_urc_line(
            fault(0x01, 0x00, 12, pc=0xDEADBEEF, lr=0x11, psr=0x61000000))
        self.assertIn("name=HARDFAULT pc=0xDEADBEEF lr=0x00000011 "
                      "psr=0x61000000 bfar=0x00000000 uptime_ms=12 ", line)
        self.assertNotIn("pc=", mg.format_fault_urc_line(LEG6_ATTEMPT0))

    def test_short_payloads(self):
        self.assertEqual(mg.format_fault_urc_line(b"\x0d"),
                         "FAULT_URC: short_payload len=1 raw=0d")
        self.assertEqual(mg.format_fault_urc_line(b"\x0e\x05"),
                         "FAULT_URC: code=0x0E sub=0x05 name=TX_DONE_EARLY "
                         "tx_id=0x05 uptime_ms=? raw=0e05")

    def test_keeps_the_ps1_grep_shape(self):
        """run_live_radio_monitor.ps1 Test-InjectFaultLine reads
        `code=0x.. sub=0x..` out of any line containing FAULT."""
        line = mg.format_fault_urc_line(LEGX_FINAL, frozenset())
        self.assertTrue(line.startswith("FAULT_URC: "), line)
        m = PS1_FAULT_RE.search(line)
        self.assertEqual((m.group(1), m.group(2)), ("0D", "83"))

    def test_seen_by_the_case_sensitive_tool_greps(self):
        """Review 2026-10-24: a lower-case `fault_urc:` tag was invisible to
        tools/rs12_leg_report.py FAULT_RE and to the 10-24 rescore.py
        (`"FAULT" in l`) -- on the base log, the only FAULT line there is."""
        rs12_fault_re = re.compile(
            r"\bFAULT\w*\b.*?\bcode=0x([0-9A-Fa-f]{2}) sub=0x([0-9A-Fa-f]{2})")
        line = ("2026-10-24 10:43:07,512 WARNING image_rx_daemon: "
                + mg.format_fault_urc_line(fault(0x0D, 0xA3, 124020),
                                           frozenset()))
        self.assertIn("FAULT", line)
        self.assertEqual(rs12_fault_re.search(line).groups(), ("0D", "A3"))
        self.assertRegex(line, NEW_LINE_RE)

    def test_told_apart_from_its_legacy_twins(self):
        """One URC can leave a legacy line beside the new one (pipelined
        LOG line, stdout `FAULT_URC during ...`); only the new one carries
        `name=` right after `code=.. sub=..`."""
        desc = mg.format_fault_payload(LEG6_ATTEMPT0)
        for twin in ("FAULT during pipelined TX: " + desc,
                     "FAULT_URC during TX wait: " + desc,
                     "FAULT_URC during settle (expected for W1-9 lenient "
                     "init): " + desc,
                     "INFO: benign FAULT_URC: " + desc,
                     "FAULT_URC: " + desc):          # method_g main()'s print
            self.assertNotRegex(twin, NEW_LINE_RE)
        self.assertRegex(mg.format_fault_urc_line(LEG6_ATTEMPT0), NEW_LINE_RE)

    def test_log_levels(self):
        self.assertEqual(mh.fault_urc_log_level(fault(0x09, 1)), logging.INFO)
        self.assertEqual(mh.fault_urc_log_level(fault(0x0C, 1)), logging.INFO)
        for code in (0x01, 0x0A, 0x0B, 0x0D, 0x0E, 0x0F):
            self.assertEqual(mh.fault_urc_log_level(fault(code, 0)),
                             logging.WARNING, code)
        self.assertEqual(mh.fault_urc_log_level(b""), logging.WARNING)


class LegacyFaultTextUnchanged(unittest.TestCase):
    """format_fault_payload was refactored to share its sub decode; its text
    is grepped by bench tooling and must not move by a byte."""

    def test_golden_branches(self):
        self.assertEqual(mg.format_fault_payload(SETTLE_RX_INACTIVE),
                         "code=0x0A sub=0x00 reason=no_ingress_during_probe_"
                         "window raw=0a00000000000000000000000000000000000000"
                         "c50b0000")
        p = fault(0x09, 0x03)
        self.assertEqual(mg.format_fault_payload(p),
                         f"code=0x09 sub=0x03 lanes=LPUART1,USART1 raw={p.hex()}")
        p = fault(0x09, 0x00)
        self.assertEqual(mg.format_fault_payload(p),
                         f"code=0x09 sub=0x00 lanes=none raw={p.hex()}")
        p = fault(0x0B, 0x07)
        self.assertEqual(mg.format_fault_payload(p),
                         f"code=0x0B sub=0x07 parse_err_count_hint=7 raw={p.hex()}")
        p = fault(0x0C, 0x1F)
        self.assertEqual(mg.format_fault_payload(p),
                         "code=0x0C sub=0x1F marks=VER_REQ_PARSED|FRAME_PARSE_ERR|"
                         f"VER_REQ_DISPATCHED|VER_URC_SENT|AT_VER_DISPATCHED raw={p.hex()}")
        p = fault(0x0C, 0x00)
        self.assertEqual(mg.format_fault_payload(p),
                         f"code=0x0C sub=0x00 marks=none raw={p.hex()}")
        self.assertEqual(mg.format_fault_payload(LEGX_FINAL),
                         f"code=0x0D sub=0x83 raw={LEGX_FINAL.hex()}")
        self.assertEqual(mg.format_fault_payload(b"\x0d"), "0d")

    def test_every_archived_pipelined_fault_line(self):
        """Re-render every `FAULT during pipelined TX:` line in the committed
        tractor logs from its raw bytes: same text, and the new line's
        code/sub agree."""
        seen = 0
        for path in sorted(glob.glob(os.path.join(
                _EVIDENCE, "radio_monitor_*", "tx_daemon.log"))):
            with open(path, encoding="utf-8-sig", errors="replace") as fh:
                for raw_line in fh:
                    marker = "FAULT during pipelined TX: "
                    if marker not in raw_line:
                        continue
                    desc = raw_line.split(marker, 1)[1].strip()
                    payload = bytes.fromhex(desc.rsplit("raw=", 1)[1])
                    self.assertEqual(mg.format_fault_payload(payload), desc, path)
                    new = mg.format_fault_urc_line(payload, frozenset())
                    self.assertEqual(PS1_FAULT_RE.search(new).groups(),
                                     PS1_FAULT_RE.search(desc).groups())
                    seen += 1
        # the six 2026-10-24 archives alone hold six such lines
        self.assertGreaterEqual(seen, 6)


# ---------------------------------------------------------------------------
# 2. The scan sub-byte model against the firmware sources
# ---------------------------------------------------------------------------
def _scan_fail_eval(retries, got_any_irq, crc_seen, ever_locked, max_retries=3):
    """Port of radio/sx1276_rx_scan_fail.c sx1276_rx_scan_fail_eval()."""
    is_final = retries >= max_retries
    hw_suspect = (not got_any_irq) and (not crc_seen)
    attempt = min(retries, 0x0F)
    return ((0x80 if is_final else 0) | (0x40 if ever_locked else 0)
            | (0x20 if hw_suspect else 0) | (0x10 if crc_seen else 0) | attempt)


class ScanEncodingMatchesFirmware(unittest.TestCase):
    def _defines(self, header):
        with open(os.path.join(_FW_INC, header), encoding="utf-8") as fh:
            text = fh.read()
        return {m.group(1): int(m.group(2), 0) for m in re.finditer(
            r"#define\s+(\w+)\s+(0x[0-9A-Fa-f]+|\d+)U?\b", text)}

    def test_scan_fail_header(self):
        d = self._defines("sx1276_rx_scan_fail.h")
        self.assertEqual(d["SX1276_RX_SCAN_MAX_RETRIES"], mg.RX_SCAN_MAX_RETRIES)
        self.assertEqual(1 << d["SX1276_RX_SCAN_FAIL_SUB_BIT_FINAL"],
                         mg.RX_SCAN_FAIL_SUB_FINAL)
        self.assertEqual(1 << d["SX1276_RX_SCAN_FAIL_SUB_BIT_WARM"],
                         mg.RX_SCAN_FAIL_SUB_WARM)
        self.assertEqual(1 << d["SX1276_RX_SCAN_FAIL_SUB_BIT_HW_SUSPECT"],
                         mg.RX_SCAN_FAIL_SUB_HW_SUSPECT)
        self.assertEqual(1 << d["SX1276_RX_SCAN_FAIL_SUB_BIT_CRC_SEEN"],
                         mg.RX_SCAN_FAIL_SUB_CRC_SEEN)
        self.assertEqual(d["SX1276_RX_SCAN_FAIL_SUB_MASK_ATTEMPT"],
                         mg.RX_SCAN_FAIL_SUB_ATTEMPT_MASK)

    def test_host_types_codes_and_payload(self):
        d = self._defines("host_types.h")
        codes = {k[len("HOST_FAULT_CODE_"):]: v for k, v in d.items()
                 if k.startswith("HOST_FAULT_CODE_")}
        # the 0x0D split (2026-10-24): the double booking is gone, every
        # code is unique and the host names each one as the firmware does
        self.assertEqual(codes["RX_SCAN_FAILED"], 0x0D)
        self.assertEqual(codes["TX_FIFO_RB_MISMATCH"], 0x0F)
        self.assertEqual(len(set(codes.values())), len(codes), codes)
        for name, value in codes.items():
            self.assertEqual(mg.HOST_FAULT_CODE_NAMES.get(value), name, name)
        self.assertEqual(set(mg.HOST_FAULT_CODE_NAMES), set(codes.values()))
        self.assertEqual(d["HOST_FAULT_URC_PAYLOAD_LEN"], 24)
        self.assertEqual(d["HOST_TYPE_FAULT_URC"], mg.HOST_TYPE_FAULT_URC)
        self.assertEqual(d["HOST_TYPE_TX_FRAME_REQ"], mg.HOST_TYPE_TX_FRAME_REQ)
        self.assertEqual(d["HOST_TYPE_TX_DONE_URC"], mg.HOST_TYPE_TX_DONE_URC)
        # RS-12.21 park command (HostLink.radio_park, bench_tools/radio_park.py)
        self.assertEqual(d["HOST_TYPE_RADIO_PARK_REQ"], mg.HOST_TYPE_RADIO_PARK_REQ)
        self.assertEqual(d["HOST_TYPE_RADIO_PARK_URC"], mg.HOST_TYPE_RADIO_PARK_URC)
        self.assertEqual((mg.HOST_TYPE_RADIO_PARK_REQ, mg.HOST_TYPE_RADIO_PARK_URC),
                         (0x04, 0x84))
        # 2026-10-24 review: the capability bit radio_park.py and
        # rs12_leg_report.py read the image by, and that the image sets it
        with open(os.path.join(_FW_INC, "host_types.h"), encoding="utf-8") as fh:
            text = fh.read()
        bit = re.search(r"#define\s+HOST_CAP_RADIO_PARK\s+\(1UL << (\d+)\)", text)
        self.assertEqual(1 << int(bit.group(1)), mg.HOST_CAP_RADIO_PARK)
        self.assertRegex(text, r"HOST_CAP_TX_FRAME \| \\\s*\n\s*HOST_CAP_RADIO_PARK\)")
        tools = os.path.join(_DC_DIR, "tools")
        if tools not in sys.path:
            sys.path.insert(0, tools)
        import rs12_leg_report as rep
        self.assertEqual(rep.HOST_CAP_RADIO_PARK, mg.HOST_CAP_RADIO_PARK)

    def test_every_emittable_sub_is_a_scan_encoding(self):
        emitted = set()
        for retries in range(0, mg.RX_SCAN_MAX_RETRIES + 1):
            for irq in (False, True):
                for crc in (False, True):
                    for warm in (False, True):
                        sub = _scan_fail_eval(retries, irq, crc, warm)
                        emitted.add(sub)
                        d = mg.decode_fault_0x0d(sub, frozenset())
                        self.assertEqual(d["name"], "RX_SCAN_FAILED", hex(sub))
                        self.assertEqual(d["basis"], "tx_id_not_in_flight")
                        # a TX with tx_id == sub in flight moves the basis only
                        busy = mg.decode_fault_0x0d(sub, frozenset({sub}))
                        self.assertEqual((busy["name"], busy["basis"], busy["scan"]),
                                         ("RX_SCAN_FAILED", "tx_id_in_flight",
                                          d["scan"]))
                        self.assertEqual(d["scan"]["attempt"], retries)
                        self.assertEqual(d["scan"]["warm"], int(warm))
                        self.assertEqual(d["scan"]["crc_seen"], int(crc))
                        self.assertEqual(d["scan"]["hw_suspect"],
                                         int(not irq and not crc))
                        self.assertEqual(d["scan"]["final"],
                                         int(retries == mg.RX_SCAN_MAX_RETRIES))
        valid = {s for s in range(256) if mg.rx_scan_fail_sub_valid(s)}
        self.assertEqual(valid, emitted | {0x80})     # + the NULL-input marker
        self.assertEqual(len(valid), 25)

    def test_every_archived_0x0d_sub_reads_as_a_scan_failure(self):
        # 2026-09/10 tractor logs: 0x00-0x02, 0x20/0x21, 0x40-0x42, 0x60-0x62,
        # 0x83, 0xA3, 0xC3, 0xE3 -- all scan encodings (none needs a tx_id).
        for sub in (0x00, 0x01, 0x02, 0x20, 0x21, 0x40, 0x41, 0x42, 0x60,
                    0x61, 0x62, 0x83, 0xA3, 0xC3, 0xE3):
            self.assertTrue(mg.rx_scan_fail_sub_valid(sub), hex(sub))


# ---------------------------------------------------------------------------
# 3. A real HostLink on a scripted L072 (no hardware)
# ---------------------------------------------------------------------------
HOST_TYPE_RX_FRAME_URC = 0x91


class FakeL072:
    """Stands in for `os` inside method_g_stage1_probe: the HostLink's fd
    reads/writes land here. Host frames are parsed and answered (VER, CFG
    set, register read/write, TX -> optional scripted URCs then TX_DONE);
    push() queues spontaneous URCs."""
    O_RDWR = O_NOCTTY = O_NONBLOCK = 0

    def __init__(self):
        self._lock = threading.Lock()
        self._out = bytearray()        # L072 -> host
        self._in = bytearray()         # host -> L072 (partial frame)
        self.host_frames = []
        self.before_reply = {}         # req type -> [(type, payload)] first
        self.on_tx = {}                # tx_id -> [(type, payload)] before TX_DONE
        self.park_tx = False           # True: TX_FRAME_REQ draws no reply
        self.refuse_tx = 0             # next N TX_FRAME_REQs: ERR_PROTO FORBIDDEN
        self.ver_payload = bytes(12)   # VER_URC: proto..reserved, cap_bitmap (LE)

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

    def _handle(self, fr):
        self.host_frames.append(fr)
        t, seq, p = fr["type"], fr["seq"], fr["payload"]
        for ftype, payload in self.before_reply.pop(t, []):
            self.push(ftype, payload)
        if t == 0x01:                                       # VER_REQ
            self.push(0x81, self.ver_payload, seq)
        elif t == 0x20:                                     # CFG_SET_REQ
            self.push(0xA0, bytes([p[0], 0]), seq)
        elif t == 0x30:                                     # REG_READ_REQ
            self.push(0xB0, bytes([p[0], 0x85]), seq)       # RXCONT
        elif t == 0x31:                                     # REG_WRITE_REQ
            self.push(0xB1, bytes(p[:2]), seq)
        elif t == mg.HOST_TYPE_TX_FRAME_REQ:
            tx_id = p[0]
            if self.park_tx:
                return
            if self.refuse_tx:                  # host_cmd.c handle_tx_frame
                self.refuse_tx -= 1
                self.push(mg.HOST_TYPE_ERR_PROTO_URC, bytes(
                    [t, 1, mg.HOST_ERR_PROTO_FORBIDDEN, 0, 0]), seq)
                return
            for ftype, payload in self.on_tx.get(tx_id, []):
                self.push(ftype, payload)
            self.push(mg.HOST_TYPE_TX_DONE_URC,
                      bytes([tx_id, 0]) + struct.pack("<I", 100_000) + b"\x0e")


def rx_frame_urc(data: bytes) -> bytes:
    return bytes([len(data), 8]) + struct.pack("<hI", -60, 1234) + data


class _WireCase(unittest.TestCase):
    """Patches the HostLink transport onto a FakeL072 for each test."""

    def setUp(self):
        self.fake = FakeL072()
        for p in (mock.patch.object(mg, "os", self.fake),
                  mock.patch.object(mg, "configure_uart", lambda *_a: None)):
            p.start()
            self.addCleanup(p.stop)
        self.stdout = io.StringIO()            # the daemons' print() lines
        self._quiet = contextlib.redirect_stdout(self.stdout)
        self._quiet.__enter__()
        self.addCleanup(self._quiet.__exit__, None, None, None)

    def link(self, observe=True):
        link = mg.HostLink("/dev/fake", "921600")
        self.calls = []
        if observe:
            link.on_fault_urc = lambda fr, ids: self.calls.append(
                (fr["payload"][1], ids))
        return link


class HostLinkObserver(_WireCase):
    def test_default_link_is_untouched(self):
        link = self.link(observe=False)
        self.assertIsNone(link.on_fault_urc)
        link.send(mg.HOST_TYPE_TX_FRAME_REQ, b"\x03\x01x")
        self.assertEqual(link._tx_unacked, {})
        self.assertEqual([f["type"] for f in link.read_frames(0.2)],
                         [mg.HOST_TYPE_TX_DONE_URC])

    def test_observer_changes_no_frame_a_caller_gets(self):
        frames = {}
        for observe in (False, True):
            self.fake.push(0xF1, LEG6_ATTEMPT0)
            self.fake.push(HOST_TYPE_RX_FRAME_URC, rx_frame_urc(b"abc"))
            self.fake.push(0xC1, bytes(8))
            frames[observe] = self.link(observe).read_frames(0.2)
        self.assertEqual(frames[False], frames[True])
        self.assertEqual(len(frames[True]), 3)
        self.assertEqual(len(self.calls), 1)

    def test_fault_inside_request_is_seen_once(self):
        """request() re-queues the FAULT it is not waiting for; the queued
        copy that read_frames() hands back later must not fire again."""
        link = self.link()
        self.fake.before_reply[0x01] = [(0xF1, LEG6_ATTEMPT0)]
        link.request(0x01, 0x81, timeout=1.0)
        self.assertEqual(len(self.calls), 1)
        again = link.read_frames(0.2)                   # the urc_queue copy
        self.assertEqual([f["type"] for f in again], [0xF1])
        self.assertEqual(len(self.calls), 1)

    def test_tx_in_flight_until_its_tx_done(self):
        link = self.link()
        self.fake.on_tx[0x03] = [(0xF1, fault(0x0D, 0x03))]
        link.send(mg.HOST_TYPE_TX_FRAME_REQ, b"\x03\x01x")
        self.assertEqual(link.tx_ids_in_flight(), frozenset({0x03}))
        link.read_frames(0.2)
        self.assertEqual(self.calls, [(0x03, frozenset({0x03}))])
        self.assertEqual(link.tx_ids_in_flight(), frozenset())

    def test_lost_tx_done_ages_out(self):
        link = self.link()
        self.fake.park_tx = True
        link.send(mg.HOST_TYPE_TX_FRAME_REQ, b"\x00\x01a")
        self.assertEqual(link.tx_ids_in_flight(), frozenset({0x00}))
        later = time.monotonic() + mg.TX_INFLIGHT_STALE_S + 0.5
        self.assertEqual(link.tx_ids_in_flight(now=later), frozenset())
        self.assertEqual(link._tx_unacked, {})

    def test_staleness_ignores_a_wall_clock_step(self):
        """Review 2026-10-24: the boards' wall clocks are wrong and an NTP
        sync steps them; staleness runs on time.monotonic()."""
        link = self.link()
        self.fake.park_tx = True
        link.send(mg.HOST_TYPE_TX_FRAME_REQ, b"\x00\x01a")
        stepped = time.time() + 19 * 86400              # 10-05 -> 10-24
        with mock.patch.object(mg.time, "time", return_value=stepped):
            self.assertEqual(link.tx_ids_in_flight(), frozenset({0x00}))

    def test_resent_tx_id_stays_in_flight_until_both_tx_dones(self):
        """Review 2026-10-24 (dup tx_id): the pipelined watchdog / serial
        retry resend a tx_id while the L072 may still hold the first copy.
        The first TX_DONE must not retire both: a pre-split FIFO fault on
        the resend (code 0x0D, sub = tx_id 0x00, also a scan encoding)
        keeps basis=tx_id_in_flight, not tx_id_not_in_flight."""
        link = self.link(observe=False)
        lines = []
        link.on_fault_urc = lambda fr, ids: lines.append(
            mg.format_fault_urc_line(bytes(fr["payload"]), ids))
        self.fake.park_tx = True
        link.send(mg.HOST_TYPE_TX_FRAME_REQ, b"\x00\x01a")   # original
        link.send(mg.HOST_TYPE_TX_FRAME_REQ, b"\x00\x01a")   # resend
        done = bytes([0x00, 0]) + struct.pack("<I", 150_000) + b"\x0e"
        self.fake.push(mg.HOST_TYPE_TX_DONE_URC, done)       # the original
        self.fake.push(0xF1, fault(0x0D, 0x00, 40_000))      # resend's FIFO
        self.fake.push(mg.HOST_TYPE_TX_DONE_URC, done)       # the resend
        self.fake.push(0xF1, fault(0x0D, 0x00, 70_000))      # nothing in flight
        while len(lines) < 2:
            self.assertTrue(link.read_frames(0.2))
        self.assertIn("sub=0x00 name=RX_SCAN_FAILED attempt=0", lines[0])
        self.assertIn("basis=tx_id_in_flight tx_inflight=0x00 "
                      "uptime_ms=40000", lines[0])
        self.assertIn("sub=0x00 name=RX_SCAN_FAILED attempt=0", lines[1])
        self.assertIn("basis=tx_id_not_in_flight tx_inflight=- "
                      "uptime_ms=70000", lines[1])
        self.assertEqual(link._tx_unacked, {})

    def test_send_drops_its_tx_ids_stale_sends(self):
        """A resend after the stale window does not inherit the lost copy,
        so its own TX_DONE retires the tx_id (and the list cannot grow)."""
        link = self.link()
        self.fake.park_tx = True
        link.send(mg.HOST_TYPE_TX_FRAME_REQ, b"\x00\x01a")
        future = time.monotonic() + mg.TX_INFLIGHT_STALE_S + 0.5
        with mock.patch.object(mg.time, "monotonic", return_value=future):
            link.send(mg.HOST_TYPE_TX_FRAME_REQ, b"\x00\x01a")
        self.assertEqual(len(link._tx_unacked[0x00]), 1)
        self.fake.push(mg.HOST_TYPE_TX_DONE_URC,
                       bytes([0x00, 0]) + struct.pack("<I", 1) + b"\x0e")
        link.read_frames(0.2)
        self.assertEqual(link._tx_unacked, {})

    def test_err_proto_refusal_retires_that_request(self):
        """host_cmd.c: a refused TX_FRAME_REQ gets ERR_PROTO (its seq,
        payload[0] = 0x10) and never a TX_DONE -- it is not in flight."""
        link = self.link()
        self.fake.refuse_tx = 1
        link.send(mg.HOST_TYPE_TX_FRAME_REQ, b"\x00\x01a")   # refused
        link.send(mg.HOST_TYPE_TX_FRAME_REQ, b"\x01\x01b")   # radiates
        self.fake.park_tx = True
        link.send(mg.HOST_TYPE_TX_FRAME_REQ, b"\x02\x01c")   # parked
        got = []
        while len(got) < 2:
            got += [f["type"] for f in link.read_frames(0.2)]
        self.assertEqual(got, [mg.HOST_TYPE_ERR_PROTO_URC,
                               mg.HOST_TYPE_TX_DONE_URC])
        self.assertEqual(link.tx_ids_in_flight(), frozenset({0x02}))
        # an ERR_PROTO for another request type, or an unknown seq, retires
        # nothing
        self.fake.push(mg.HOST_TYPE_ERR_PROTO_URC, bytes([0x30, 1, 3, 1, 0]),
                       link._tx_unacked[0x02][0][0])
        self.fake.push(mg.HOST_TYPE_ERR_PROTO_URC, bytes([0x10, 1, 8, 0, 0]),
                       0x7777)
        self.assertEqual(len(link.read_frames(0.2)), 2)
        self.assertEqual(link.tx_ids_in_flight(), frozenset({0x02}))

    def test_observer_error_never_breaks_the_reader(self):
        link = self.link()
        link.on_fault_urc = mock.Mock(side_effect=RuntimeError("boom"))
        self.fake.push(0xF1, LEG6_ATTEMPT0)
        self.fake.push(HOST_TYPE_RX_FRAME_URC, rx_frame_urc(b"abc"))
        self.assertEqual([f["type"] for f in link.read_frames(0.2)],
                         [0xF1, HOST_TYPE_RX_FRAME_URC])
        link.on_fault_urc.assert_called_once()

    def test_install_fault_urc_logger(self):
        link = self.link(observe=False)
        log = logging.getLogger("test_fault_urc_log_sil.install")
        mh.install_fault_urc_logger(link, log)
        self.fake.push(0xF1, fault(0x09, 0x01, 812))
        self.fake.push(0xF1, LEG6_ATTEMPT0)
        with self.assertLogs(log, level="INFO") as cap:
            link.read_frames(0.2)
        self.assertEqual([(r.levelno, r.getMessage()) for r in cap.records], [
            (logging.INFO, mg.format_fault_urc_line(fault(0x09, 0x01, 812),
                                                    frozenset())),
            (logging.WARNING, mg.format_fault_urc_line(LEG6_ATTEMPT0,
                                                       frozenset())),
        ])


# ---------------------------------------------------------------------------
# 4. The daemons
# ---------------------------------------------------------------------------
_OPEN_ENV = {"LIFETRAC_SKIP_RESET_REQ": "1", "LIFETRAC_SKIP_PHY_CONTRACT": "1",
             "LIFETRAC_TX_POWER_DBM": ""}


class _DaemonCase(_WireCase):
    module = None
    logger = None

    def setUp(self):
        super().setUp()
        for p in (mock.patch.dict(os.environ, _OPEN_ENV),
                  mock.patch.object(self.module,
                                    "configure_regulatory_profile_if_needed",
                                    lambda *_a, **_k: None)):
            p.start()
            self.addCleanup(p.stop)

    def open_link(self, d):
        with self.assertLogs(self.logger, level="DEBUG") as cap:
            link = d._open_link()
        return link, cap.records

    def assert_one_fault_line(self, records, expect, level=logging.WARNING):
        hits = [r for r in fault_urc_lines(records) if expect in r.getMessage()]
        self.assertEqual(len(hits), 1, [r.getMessage() for r in records])
        self.assertEqual(hits[0].levelno, level)
        return hits[0].getMessage()

    def assert_one_count(self, records, n):
        """The tools' counting rule over one docker log (LOG records plus
        the print()ed stdout): the `name=` key matches once per FAULT_URC,
        whatever legacy twins sit beside it."""
        text = ([r.getMessage() for r in records]
                + self.stdout.getvalue().splitlines())
        self.assertEqual(sum(bool(NEW_LINE_RE.search(t)) for t in text), n,
                         text)


class BaseDaemonFaultPaths(_DaemonCase):
    module = image_rx_daemon
    logger = "image_rx_daemon"

    def daemon(self):
        return image_rx_daemon.ImageRxDaemon(
            uart="/dev/fake", baud="921600", mqtt_host="127.0.0.1",
            mqtt_port=1883, reassembler_timeout_ms=1500)

    def test_open_link_logs_boot_and_request_faults(self):
        """A boot-drain FAULT and one that lands inside the VER request are
        each logged exactly once, from _open_link on."""
        self.fake.push(0xF1, fault(0x09, 0x01, 812))
        self.fake.before_reply[0x01] = [(0xF1, fault(0x0D, 0x21, 64012))]
        link, records = self.open_link(self.daemon())
        self.assertIsNotNone(link.on_fault_urc)
        self.assert_one_fault_line(
            records, "code=0x09 sub=0x01 name=HOST_RX_SEEN lanes=LPUART1",
            logging.INFO)
        msg = self.assert_one_fault_line(records, "code=0x0D sub=0x21")
        self.assertIn("name=RX_SCAN_FAILED attempt=1 final=0 warm=0 "
                      "hw_suspect=1 crc_seen=0 basis=tx_id_not_in_flight "
                      "tx_inflight=- uptime_ms=64012", msg)
        self.assertEqual(len(fault_urc_lines(records)), 2)

    def test_open_link_logs_the_capability_bitmap(self):
        """2026-10-24 review: the VER warm-up line names the image by its
        capability bitmap (bit 7 HOST_CAP_RADIO_PARK: the RS-12.21 image, whose
        scan restarts after a final RX_SCAN_FAILED), in the shape
        rs12_leg_report.py's acquisition rule reads; a short VER payload logs
        cap=?."""
        tools = os.path.join(_DC_DIR, "tools")
        if tools not in sys.path:
            sys.path.insert(0, tools)
        import rs12_leg_report as rep
        for payload, cap in ((bytes(8) + (0xFF).to_bytes(4, "little"), "0x000000FF"),
                             (bytes(8) + (0x7F).to_bytes(4, "little"), "0x0000007F"),
                             (bytes(11), "?")):
            with self.subTest(cap=cap):
                self.fake.ver_payload = payload
                _link, records = self.open_link(self.daemon())
                msgs = [r.getMessage() for r in records
                        if r.getMessage().startswith("L072 VER warm-up ok")]
                self.assertEqual(msgs, [f"L072 VER warm-up ok (attempt 1) cap={cap}"])
                m = rep.VER_CAP_RE.search(msgs[0])
                if cap == "?":
                    self.assertIsNone(m)
                else:
                    self.assertEqual(int(m.group(1), 16) & rep.HOST_CAP_RADIO_PARK,
                                     int(cap, 16) & 0x80)

    def test_command_tx_fifo_mismatch(self):
        """A pre-split image's FIFO fault on a base command (code 0x0D, sub
        = tx_id 0xFD), and the same fault from a split image (code 0x0F)."""
        for code, expect in (
                (0x0D, "code=0x0D sub=0xFD name=TX_FIFO_RB_MISMATCH tx_id=0xFD "
                       "fw=old basis=not_scan_encoding tx_inflight=0xFD "
                       "uptime_ms=5000"),
                (0x0F, "code=0x0F sub=0xFD name=TX_FIFO_RB_MISMATCH tx_id=0xFD "
                       "uptime_ms=5000")):
            with self.subTest(code=hex(code)):
                self.stdout.seek(0)
                self.stdout.truncate()
                d = self.daemon()
                link, _ = self.open_link(d)
                self.fake.on_tx[0xFD] = [(0xF1, fault(code, 0xFD, 5000))]
                body = pack_command_frame(CMD_OP_REQ_KEYFRAME)
                with self.assertLogs(self.logger, level="DEBUG") as cap:
                    d._send_command_frame(link, body, copies=1)
                self.assert_one_fault_line(cap.records, expect)
                # the base's command TX wait also print()s a stdout twin
                # (review 2026-10-24): one docker log, one count on `name=`
                self.assertIn("FAULT_URC during TX wait: " + mg.format_fault_payload(
                    fault(code, 0xFD, 5000)), self.stdout.getvalue())
                self.assert_one_count(cap.records, 1)
                # non-FAULT handling around it: TX_DONE still completes the send
                self.assertTrue(any("command TX opcode=0x60 copy=1/1 OK (on air)"
                                    in r.getMessage() for r in cap.records))
                self.assertEqual(getattr(d, "_cmd_tx_ok", 0), 1)

    def test_rx_loop_logs_fault_and_still_handles_frames(self):
        """The RX loop used to drop FAULTs at DEBUG; now the observer logs
        it, while the DEBUG line and the RX_FRAME path are unchanged."""
        d = self.daemon()
        link, _ = self.open_link(d)
        d._open_link = lambda: link
        self.fake.push(0xF1, fault(0x0D, 0xA3, 124020))
        frag = bytes([FRAGMENT_MAGIC, 5, 0, 1]) + b"half-a-frame"
        self.fake.push(HOST_TYPE_RX_FRAME_URC, rx_frame_urc(frag))
        worker = threading.Thread(target=d._rx_worker, daemon=True)
        with self.assertLogs(self.logger, level="DEBUG") as cap:
            worker.start()
            deadline = time.time() + 5.0
            while time.time() < deadline and d.stats.rx_frames_seen < 1:
                time.sleep(0.02)
            d._stop.set()
            worker.join(5.0)
        self.assertFalse(worker.is_alive())
        self.assert_one_fault_line(
            cap.records, "code=0x0D sub=0xA3 name=RX_SCAN_FAILED attempt=3 "
            "final=1 warm=0 hw_suspect=1 crc_seen=0 basis=tx_id_not_in_flight "
            "tx_inflight=- uptime_ms=124020")
        self.assertTrue(any(r.levelno == logging.DEBUG and r.getMessage()
                            == "ignoring host frame type=0xf1"
                            for r in cap.records))
        self.assertEqual(d.stats.rx_frames_seen, 1)
        self.assertEqual(d.stats.rx_decode_errors, 0)


@unittest.skipUnless(image_tx_daemon is not None,
                     "paho-mqtt required to import image_tx_daemon")
class TractorDaemonFaultPaths(_DaemonCase):
    module = image_tx_daemon
    logger = "image_tx_daemon"

    def daemon(self):
        return image_tx_daemon.ImageTxDaemon(
            uart="/dev/fake", baud="921600", mqtt_host="127.0.0.1",
            mqtt_port=1883, inter_cycle_s=0.05, max_queue_depth=4)

    def test_open_link_logs_boot_fault(self):
        self.fake.push(0xF1, fault(0x0C, 0x09, 400))
        self.fake.before_reply[0x01] = [(0xF1, LEG6_ATTEMPT0)]
        link, records = self.open_link(self.daemon())
        self.assertIsNotNone(link.on_fault_urc)
        self.assert_one_fault_line(
            records, "code=0x0C sub=0x09 name=HOST_DIAG_MARK", logging.INFO)
        self.assert_one_fault_line(records, "code=0x0D sub=0x20 "
                                   "name=RX_SCAN_FAILED attempt=0")

    def test_pipelined_tx_scan_and_fifo_faults(self):
        """Fragments 0 and 1 in flight (depth 2): a scan fault (sub 0x20)
        reads as RX_SCAN_FAILED; a 0x0D whose sub is fragment 1's tx_id
        reads as RX_SCAN_FAILED too, with basis=tx_id_in_flight (a pre-split
        image could have meant fragment 1's FIFO fault; PR #166 named it
        AMBIGUOUS); a split image's 0x0F on fragment 1 is the FIFO fault.
        The historical pipelined line is still written verbatim and the
        TX_DONE accounting is unchanged."""
        d = self.daemon()
        link, _ = self.open_link(d)
        scan = fault(0x0D, 0x20, 34012)
        busy = fault(0x0D, 0x01, 34200)
        fifo = fault(0x0F, 0x01, 34201)
        self.fake.on_tx[0x00] = [(0xF1, scan)]
        self.fake.on_tx[0x01] = [(0xF1, busy), (0xF1, fifo)]
        frame = image_tx_daemon._PendingFrame(seq=7, payload=b"x",
                                              enqueued_ms=0)
        with self.assertLogs(self.logger, level="DEBUG") as cap:
            d._tx_fragments_pipelined(link, frame, [b"a" * 20, b"b" * 20],
                                      PHY_IMAGE_BW250, 4, 1)
        self.assert_one_fault_line(
            cap.records, "code=0x0D sub=0x20 name=RX_SCAN_FAILED attempt=0 "
            "final=0 warm=0 hw_suspect=1 crc_seen=0 basis=tx_id_not_in_flight "
            "tx_inflight=0x00,0x01 uptime_ms=34012")
        self.assert_one_fault_line(
            cap.records, "code=0x0D sub=0x01 name=RX_SCAN_FAILED attempt=1 final=0 "
            "warm=0 hw_suspect=0 crc_seen=0 basis=tx_id_in_flight "
            "tx_inflight=0x01 uptime_ms=34200")
        self.assert_one_fault_line(
            cap.records, "code=0x0F sub=0x01 name=TX_FIFO_RB_MISMATCH tx_id=0x01 "
            "uptime_ms=34201")
        legacy = [r.getMessage() for r in cap.records
                  if r.getMessage().startswith("FAULT during pipelined TX: ")]
        self.assertEqual(legacy, [
            "FAULT during pipelined TX: " + mg.format_fault_payload(scan),
            "FAULT during pipelined TX: " + mg.format_fault_payload(busy),
            "FAULT during pipelined TX: " + mg.format_fault_payload(fifo)])
        self.assert_one_count(cap.records, 3)
        self.assertEqual((d.frames_tx_ok, d.fragments_tx_ok,
                          d.fragments_tx_fail), (1, 2, 0))

    def test_command_tx_wait_logs_fault(self):
        """The serial wait (wait_for_tx_done) used to only print() it."""
        d = self.daemon()
        link, _ = self.open_link(d)
        self.fake.on_tx[0xFE] = [(0xF1, LEGX_FINAL)]
        with self.assertLogs(self.logger, level="DEBUG") as cap:
            d._send_command_frame(link, pack_command_frame(CMD_OP_PROBE,
                                                           bytes(6)), copies=1)
        self.assert_one_fault_line(
            cap.records, "code=0x0D sub=0x83 name=RX_SCAN_FAILED attempt=3 "
            "final=1 warm=0 hw_suspect=0 crc_seen=0 basis=tx_id_not_in_flight "
            "tx_inflight=0xFE uptime_ms=123670")
        self.assertIn("FAULT_URC during TX wait: "
                      + mg.format_fault_payload(LEGX_FINAL),
                      self.stdout.getvalue())
        self.assert_one_count(cap.records, 1)
        self.assertEqual(sum(f["type"] == mg.HOST_TYPE_TX_FRAME_REQ
                             for f in self.fake.host_frames), 1)

    def test_command_drain_logs_fault_and_returns_rx(self):
        """_drain_rx_frames used to discard non-RX URCs silently."""
        d = self.daemon()
        link, _ = self.open_link(d)
        cmd = pack_command_frame(CMD_OP_REQ_KEYFRAME)
        self.fake.push(0xF1, LEG6_ATTEMPT2)
        self.fake.push(HOST_TYPE_RX_FRAME_URC, rx_frame_urc(cmd))
        with self.assertLogs(self.logger, level="DEBUG") as cap:
            got = d._drain_rx_frames(link, 0.3)
        self.assertEqual(got, [cmd])
        self.assert_one_fault_line(
            cap.records, "code=0x0D sub=0x02 name=RX_SCAN_FAILED attempt=2 "
            "final=0 warm=0 hw_suspect=0 crc_seen=0 basis=tx_id_not_in_flight "
            "tx_inflight=- uptime_ms=94014")


if __name__ == "__main__":
    unittest.main()
