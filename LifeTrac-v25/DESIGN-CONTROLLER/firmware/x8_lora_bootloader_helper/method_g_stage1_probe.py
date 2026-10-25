#!/usr/bin/env python3
"""
method_g_stage1_probe.py - Stage 1 custom-firmware probe for Method G.

Talks to the custom murata_l072 firmware over /dev/ttymxc3 using the
binary COBS-framed host protocol at 921600 8N1. The probe is intended to
exercise the first Phase 1 gate from DESIGN-LORAFIRMWARE/03:

  - observe BOOT_URC when available
  - request VER_URC and STATS_URC
  - read the SX1276 version register and a full 0x00..0x70 register map

Exit codes:
  0 = Stage 1 critical checks passed
  1 = Protocol responded but one or more critical checks failed
  2 = Fatal transport / timeout failure
"""

from __future__ import annotations

import argparse
import os
import struct
import subprocess
import sys
import time


DEV_DEFAULT = "/dev/ttymxc3"
BAUD_DEFAULT = "921600"

HOST_PROTOCOL_VER = 1

HOST_TYPE_VER_REQ = 0x01
HOST_TYPE_REG_READ_REQ = 0x30
HOST_TYPE_STATS_DUMP_REQ = 0x41

HOST_TYPE_VER_URC = 0x81
HOST_TYPE_REG_DATA_URC = 0xB0
HOST_TYPE_STATS_URC = 0xC1
HOST_TYPE_BOOT_URC = 0xF0
HOST_TYPE_FAULT_URC = 0xF1
HOST_TYPE_READY_URC = 0xF2
HOST_TYPE_ERR_PROTO_URC = 0xFE

# 2026-10-24 (RS-12.21, the firmware park command; include/host_types.h).
# RADIO_PARK_REQ (empty payload) sets the L072's host_parked flag, which
# gates EVERY RX arm path (scan walker, gamma-1 retune, slot follower, the
# post-TX re-arm in sx1276_tx.c, the scan restarts), and puts the modem to
# SLEEP. The flag holds until a host raw RXCONT write (REG_WRITE RegOpMode),
# a profile activation or a reset; a TX_FRAME_REQ while parked is refused
# (ERR_PROTO). The reply, RADIO_PARK_URC (same seq): {u8 status
# (RADIO_PARK_STATUS_*), u8 RegOpMode readback}. Firmware from before the
# command answers ERR_PROTO UNKNOWN_TYPE (host_cmd.c dispatch default) and
# leaves the radio alone. HostLink.radio_park() sends it and sorts the answer.
HOST_TYPE_RADIO_PARK_REQ = 0x04
HOST_TYPE_RADIO_PARK_URC = 0x84
RADIO_PARK_STATUS_PARKED = 0
RADIO_PARK_STATUS_ALREADY_PARKED = 1
RADIO_PARK_STATUS_REFUSED = 2
RADIO_PARK_STATUS_NAMES = {
    RADIO_PARK_STATUS_PARKED: "PARKED",
    RADIO_PARK_STATUS_ALREADY_PARKED: "ALREADY_PARKED",
    RADIO_PARK_STATUS_REFUSED: "REFUSED",
}
# The firmware answers a host request within one main-loop pass; old
# firmware's UNKNOWN_TYPE refusal is as quick. 1 s is the "no reply" bound.
RADIO_PARK_TIMEOUT_S = 1.0
# VER_URC / READY_URC cap_bitmap bit 7 (host_types.h HOST_CAP_RADIO_PARK): the
# image answers RADIO_PARK_REQ. The same image moved TX_FIFO_RB_MISMATCH to
# 0x0F (the 0x0D split, below) and restarts its scan after a final
# RX_SCAN_FAILED (scan option (b)), so a host SLEEP write cannot hold on it:
# bench_tools/radio_park.py parks it by the command only (2026-10-24 review).
HOST_CAP_RADIO_PARK = 0x80

HOST_FAULT_CODE_HOST_RX_SEEN = 0x09
HOST_FAULT_CODE_HOST_RX_INACTIVE = 0x0A
HOST_FAULT_CODE_HOST_PARSE_ERROR = 0x0B
HOST_FAULT_CODE_HOST_DIAG_MARK = 0x0C

# 2026-10-24 (FAULT URC logging): the rest of host_types.h's fault codes, for
# the `FAULT_URC:` line (format_fault_urc_line).
# 2026-10-24 (the 0x0D split, FHSS_SCAN_FAILED_OPTIONS.md section 7): 0x0D
# was DOUBLE-BOOKED -- RX_SCAN_FAILED (radio/sx1276_rx.c, sub = the
# sx1276_rx_scan_fail.h bit field; since 2026-05-20) and TX_FIFO_RB_MISMATCH
# (radio/sx1276_tx.c, sub = that TX's tx_id, emitted just BEFORE its
# TX_DONE_URC; RS-11.5, 2026-08-02). From the split RX_SCAN_FAILED keeps 0x0D
# and TX_FIFO_RB_MISMATCH moves to 0x0F. An image from before the split still
# sends its FIFO fault as 0x0D (..._PRE_SPLIT below). VER_URC's version and
# name do not tell the images apart (0.0.0 / "dev"), but its capability
# bitmap does: the image with the split also added the park command and sets
# HOST_CAP_RADIO_PARK (above). decode_fault_0x0d() is called where that bitmap
# is not at hand (the daemons' FAULT_URC hook), so it reads every 0x0D by its
# sub-byte, which is exact on a split image too: every 0x0D it sends is a scan
# fault with a valid sub, so "fw=old" can only mark a pre-split image.
HOST_FAULT_CODE_HARDFAULT = 0x01
HOST_FAULT_CODE_NMI = 0x02
HOST_FAULT_CODE_RADIO_INIT_FAIL = 0x03
HOST_FAULT_CODE_RADIO_OPMODE_DRIFT = 0x04
HOST_FAULT_CODE_RX_CRC_DISABLED = 0x05
HOST_FAULT_CODE_HOST_DMA_OVERRUN = 0x06
HOST_FAULT_CODE_STACK_GUARD = 0x07
HOST_FAULT_CODE_CLOCK_HSE_FAILED = 0x08
HOST_FAULT_CODE_RX_SCAN_FAILED = 0x0D
HOST_FAULT_CODE_TX_DONE_EARLY = 0x0E
HOST_FAULT_CODE_TX_FIFO_RB_MISMATCH = 0x0F
HOST_FAULT_CODE_TX_FIFO_RB_MISMATCH_PRE_SPLIT = 0x0D

HOST_FAULT_CODE_NAMES = {
    HOST_FAULT_CODE_HARDFAULT: "HARDFAULT",
    HOST_FAULT_CODE_NMI: "NMI",
    HOST_FAULT_CODE_RADIO_INIT_FAIL: "RADIO_INIT_FAIL",
    HOST_FAULT_CODE_RADIO_OPMODE_DRIFT: "RADIO_OPMODE_DRIFT",
    HOST_FAULT_CODE_RX_CRC_DISABLED: "RX_CRC_DISABLED",
    HOST_FAULT_CODE_HOST_DMA_OVERRUN: "HOST_DMA_OVERRUN",
    HOST_FAULT_CODE_STACK_GUARD: "STACK_GUARD",
    HOST_FAULT_CODE_CLOCK_HSE_FAILED: "CLOCK_HSE_FAILED",
    HOST_FAULT_CODE_HOST_RX_SEEN: "HOST_RX_SEEN",
    HOST_FAULT_CODE_HOST_RX_INACTIVE: "HOST_RX_INACTIVE",
    HOST_FAULT_CODE_HOST_PARSE_ERROR: "HOST_PARSE_ERROR",
    HOST_FAULT_CODE_HOST_DIAG_MARK: "HOST_DIAG_MARK",
    # format_fault_urc_line names a 0x0D by decode_fault_0x0d(), not here
    HOST_FAULT_CODE_RX_SCAN_FAILED: "RX_SCAN_FAILED",
    HOST_FAULT_CODE_TX_DONE_EARLY: "TX_DONE_EARLY",
    HOST_FAULT_CODE_TX_FIFO_RB_MISMATCH: "TX_FIFO_RB_MISMATCH",
}

# RX_SCAN_FAILED `sub` layout + retry budget, mirrored from
# include/sx1276_rx_scan_fail.h (pinned against the header by
# base_station/tests/test_fault_urc_log_sil.py).
RX_SCAN_FAIL_SUB_FINAL = 0x80        # retries exhausted, absorbing FAILED
RX_SCAN_FAIL_SUB_WARM = 0x40         # this boot has LOCKED at least once
RX_SCAN_FAIL_SUB_HW_SUSPECT = 0x20   # window saw no DIO0 IRQ and no CRC error
RX_SCAN_FAIL_SUB_CRC_SEEN = 0x10     # window saw >= 1 payload-CRC error
RX_SCAN_FAIL_SUB_ATTEMPT_MASK = 0x0F
RX_SCAN_MAX_RETRIES = 3              # SX1276_RX_SCAN_MAX_RETRIES (4 attempts)

# TX bookkeeping for the 0x0D decode's basis (HostLink.on_fault_urc): the
# request that radiates, and the URC that retires it (payload[0] = tx_id in
# both). An ERR_PROTO_URC whose payload[0] is TX_FRAME_REQ retires the
# request with its seq too: the L072 refused it (host_cmd.c; also every TX
# while the radio is parked, RS-12.21), so no TX_DONE_URC follows.
HOST_TYPE_TX_FRAME_REQ = 0x10
HOST_TYPE_TX_DONE_URC = 0x90
# A tx_id with an unretired TX_FRAME_REQ counts as in flight until this long
# after its NEWEST send (time.monotonic()) -- above every host TX timeout
# (the daemons give up after 2-3 s), so a lost TX_DONE ages out instead of
# pinning its tx_id "in flight" (which would give every later 0x0D on it
# basis=tx_id_in_flight).
TX_INFLIGHT_STALE_S = 5.0

HOST_ERR_PROTO_BAD_VERSION = 0x01
HOST_ERR_PROTO_UNKNOWN_TYPE = 0x02
HOST_ERR_PROTO_BAD_LENGTH = 0x03
HOST_ERR_PROTO_BAD_CRC = 0x04
HOST_ERR_PROTO_BAD_COBS = 0x05
HOST_ERR_PROTO_TOO_LARGE = 0x06
HOST_ERR_PROTO_QUEUE_FULL = 0x07
HOST_ERR_PROTO_FORBIDDEN = 0x08

HOST_RX_SEEN_FLAG_LPUART = 0x01
HOST_RX_SEEN_FLAG_USART1 = 0x02

HOST_DIAG_MARK_VER_REQ_PARSED = 0x01
HOST_DIAG_MARK_FRAME_PARSE_ERR = 0x02
HOST_DIAG_MARK_VER_REQ_DISPATCHED = 0x04
HOST_DIAG_MARK_VER_URC_SENT = 0x08
HOST_DIAG_MARK_AT_VER_DISPATCHED = 0x10

SX1276_REG_VERSION = 0x42
REGISTER_DUMP_END = 0x70


def configure_uart(dev: str, baud: str) -> None:
    result = subprocess.run(
        [
            "stty",
            "-F",
            dev,
            baud,
            "cs8",
            "-parenb",
            "-cstopb",
            "raw",
            "-echo",
            "-ixon",
            "-ixoff",
            "-ixany",
            "-crtscts",
        ],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or f"stty failed with rc={result.returncode}")


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_index = 0
    code = 1
    for byte in data:
        if byte == 0:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
            continue
        out.append(byte)
        code += 1
        if code == 0xFF:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
    out[code_index] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    index = 0
    while index < len(data):
        code = data[index]
        if code == 0:
            raise ValueError("zero byte inside COBS payload")
        index += 1
        for _ in range(1, code):
            if index >= len(data):
                raise ValueError("truncated COBS payload")
            out.append(data[index])
            index += 1
        if code != 0xFF and index < len(data):
            out.append(0)
    return bytes(out)


def build_frame(frame_type: int, seq: int, payload: bytes = b"", flags: int = 0) -> bytes:
    header = struct.pack("<BBBHH", HOST_PROTOCOL_VER, frame_type, flags, seq, len(payload))
    inner = header + payload
    inner += struct.pack("<H", crc16_ccitt(inner))
    return b"\x00" + cobs_encode(inner) + b"\x00"


def parse_frame(raw: bytes) -> dict:
    inner = cobs_decode(raw)
    if len(inner) < 9:
        raise ValueError("inner frame too short")
    body = inner[:-2]
    got_crc = struct.unpack("<H", inner[-2:])[0]
    calc_crc = crc16_ccitt(body)
    if got_crc != calc_crc:
        raise ValueError(f"CRC mismatch got=0x{got_crc:04X} calc=0x{calc_crc:04X}")
    ver, frame_type, flags, seq, payload_len = struct.unpack("<BBBHH", body[:7])
    payload = body[7:]
    if len(payload) != payload_len:
        raise ValueError("payload length mismatch")
    return {
        "ver": ver,
        "type": frame_type,
        "flags": flags,
        "seq": seq,
        "payload": payload,
    }


class HostLink:
    def __init__(self, dev: str, baud: str):
        configure_uart(dev, baud)
        self.fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.rx_buf = bytearray()
        self.seq = 1
        # Frames received but not consumed by the matching request() call.
        # Drained first by subsequent read_frames()/request()/wait_*() so URCs
        # that arrive in the middle of a request/response cannot be lost.
        # 2026-05-12 W1-9e fix: previously these frames were kept only in a
        # local 'pending' list inside request() and silently discarded on
        # return, which dropped TX_DONE_URC arriving during post-TX_FRAME_REQ
        # diagnostic REG_READ_REQs.
        self.urc_queue = []
        # 2026-10-24 (FAULT URC logging): optional observer, called as
        # on_fault_urc(frame, tx_ids_in_flight) ONCE per FAULT_URC, when
        # read_frames() takes it off the wire -- so a fault consumed inside
        # request()/drain_boot()/drain_pending()/wait_for_tx_done() is still
        # seen, and one request() re-queues is not seen twice. While it is
        # set, send()/read_frames() also track TX_FRAME_REQs awaiting their
        # TX_DONE_URC (the 0x0D decode's context). None (the default) keeps
        # the historical reader exactly: no tracking, no callback.
        self.on_fault_urc = None
        # tx_id -> [(seq, time.monotonic()), ...]: its unretired sends,
        # oldest first. A list, not one slot per tx_id: the daemons resend a
        # tx_id (pipelined watchdog, serial retry) while the L072 may still
        # hold the first copy, and one TX_DONE must not retire both.
        self._tx_unacked = {}

    def tx_ids_in_flight(self, now: "float | None" = None) -> frozenset:
        """tx_ids with a TX_FRAME_REQ whose TX_DONE_URC (or ERR_PROTO
        refusal) has not been read, while the newest such send is younger
        than TX_INFLIGHT_STALE_S on time.monotonic() (the boards' wall
        clocks step). Only maintained while on_fault_urc is set (otherwise
        always empty)."""
        now = time.monotonic() if now is None else now
        for tx_id in [k for k, sends in self._tx_unacked.items()
                      if now - sends[-1][1] > TX_INFLIGHT_STALE_S]:
            del self._tx_unacked[tx_id]
        return frozenset(self._tx_unacked)

    def _tx_retire(self, tx_id: int, index: int) -> None:
        sends = self._tx_unacked.get(tx_id)
        if sends:
            del sends[index]
            if not sends:
                del self._tx_unacked[tx_id]

    def _observe_frame(self, frame: dict) -> None:
        # on_fault_urc bookkeeping (2026-10-24); never raises into the reader.
        payload = frame["payload"]
        if frame["type"] == HOST_TYPE_TX_DONE_URC:
            # the OLDEST unretired send of that tx_id: the L072 serves its
            # TX ring head-first, and which copy a TX_DONE answers cannot be
            # told -- a resent tx_id stays in flight (0x0D basis=
            # tx_id_in_flight)
            if payload:
                self._tx_retire(payload[0], 0)
            return
        if frame["type"] == HOST_TYPE_ERR_PROTO_URC:
            if payload and payload[0] == HOST_TYPE_TX_FRAME_REQ:
                for tx_id, sends in list(self._tx_unacked.items()):
                    hit = [i for i, (seq, _t) in enumerate(sends)
                           if seq == frame["seq"]]
                    if hit:
                        self._tx_retire(tx_id, hit[0])
                        break
            return
        try:
            self.on_fault_urc(frame, self.tx_ids_in_flight())
        except Exception:
            pass

    def close(self) -> None:
        os.close(self.fd)

    def write_all(self, data: bytes, timeout: float = 1.0) -> None:
        deadline = time.time() + timeout
        offset = 0
        while offset < len(data):
            try:
                written = os.write(self.fd, data[offset:])
            except BlockingIOError:
                written = 0

            if written > 0:
                offset += written
                continue

            if time.time() >= deadline:
                raise TimeoutError(f"serial write timeout after {offset}/{len(data)} bytes")

            time.sleep(0.005)

    def send_ascii(self, text: str) -> None:
        self.write_all(text.encode("ascii"))

    def read_raw(self, timeout: float = 0.5, max_bytes: int = 256) -> bytes:
        deadline = time.time() + timeout
        data = bytearray()
        while time.time() < deadline and len(data) < max_bytes:
            try:
                chunk = os.read(self.fd, max_bytes - len(data))
            except BlockingIOError:
                chunk = b""
            if chunk:
                data.extend(chunk)
                deadline = time.time() + 0.1
                continue
            time.sleep(0.01)
        return bytes(data)

    def drain_raw(self) -> bytes:
        data = bytearray()
        if self.rx_buf:
            data.extend(self.rx_buf)
            self.rx_buf.clear()
        while True:
            try:
                chunk = os.read(self.fd, 1024)
            except BlockingIOError:
                break
            if not chunk:
                break
            data.extend(chunk)
        return bytes(data)

    def send(self, frame_type: int, payload: bytes = b"", flags: int = 0) -> int:
        seq = self.seq
        self.seq = (self.seq + 1) & 0xFFFF
        if self.seq == 0:
            self.seq = 1
        self.write_all(build_frame(frame_type, seq, payload, flags))
        if (self.on_fault_urc is not None
                and frame_type == HOST_TYPE_TX_FRAME_REQ and payload):
            # 0x0D decode context; sends already stale are dropped here so a
            # lost TX_DONE cannot make this tx_id's list grow
            now = time.monotonic()
            self._tx_unacked[payload[0]] = [
                s for s in self._tx_unacked.get(payload[0], ())
                if now - s[1] <= TX_INFLIGHT_STALE_S] + [(seq, now)]
        return seq

    def read_frames(self, timeout: float):
        deadline = time.time() + timeout
        frames = []
        # 2026-05-12 W1-9e fix: drain any frames stashed by previous request()
        # calls before reading new bytes from the wire.
        # 2026-05-12 W1-9b refinement: if the queue has frames, return them
        # immediately (caller may want them). If the caller wants more, they
        # call read_frames again — which will then proceed to the wire-read
        # path below since the queue is now empty.
        if self.urc_queue:
            frames.extend(self.urc_queue)
            self.urc_queue.clear()
            return frames
        while time.time() < deadline:
            try:
                chunk = os.read(self.fd, 1024)
            except BlockingIOError:
                chunk = b""
            if chunk:
                self.rx_buf.extend(chunk)
                while True:
                    try:
                        start = self.rx_buf.index(0)
                    except ValueError:
                        self.rx_buf.clear()
                        break
                    if start > 0:
                        del self.rx_buf[:start]
                    try:
                        end = self.rx_buf.index(0, 1)
                    except ValueError:
                        break
                    payload = bytes(self.rx_buf[1:end])
                    del self.rx_buf[: end + 1]
                    if not payload:
                        continue
                    try:
                        frame = parse_frame(payload)
                    except ValueError as exc:
                        print(f"WARNING: dropped malformed frame: {exc}")
                        continue
                    frames.append(frame)
                    if (self.on_fault_urc is not None and frame["type"]
                            in (HOST_TYPE_FAULT_URC, HOST_TYPE_TX_DONE_URC,
                                HOST_TYPE_ERR_PROTO_URC)):
                        self._observe_frame(frame)
                if frames:
                    return frames
            time.sleep(0.01)
        return frames

    def request(self, req_type: int, rsp_type: int, payload: bytes = b"", timeout: float = 1.0) -> dict:
        seq = self.send(req_type, payload)
        deadline = time.time() + timeout
        # 2026-05-12 W1-9b fix: drain urc_queue ONCE up front. Subsequent
        # iterations must read from the wire — otherwise read_frames() keeps
        # returning the same queued (non-matching) frames every iteration
        # without ever pulling new bytes, and request() spins until timeout.
        pending = list(self.urc_queue)
        self.urc_queue.clear()
        unmatched_local = []
        while True:
            if not pending:
                if time.time() >= deadline:
                    if unmatched_local:
                        self.urc_queue.extend(unmatched_local)
                    raise TimeoutError(
                        f"timeout waiting for response type 0x{rsp_type:02X} "
                        f"to req 0x{req_type:02X}"
                    )
                pending = self.read_frames(0.1)
            keep = []
            matched = None
            for idx, frame in enumerate(pending):
                if frame["type"] == rsp_type and (frame["seq"] == seq or rsp_type == HOST_TYPE_VER_URC):
                    matched = (idx, frame)
                    break
                if frame["type"] == HOST_TYPE_ERR_PROTO_URC and frame["seq"] == seq:
                    self.urc_queue.extend(unmatched_local + keep + pending[idx + 1:])
                    details = format_err_proto_payload(frame["payload"])
                    raise RuntimeError(f"ERR_PROTO for req 0x{req_type:02X}: {details}")
                # Stale type-match (response from prior request that was
                # given up on) — discard so it doesn't cycle in urc_queue.
                if frame["type"] == rsp_type:
                    print(
                        f"INFO: discarding stale type=0x{frame['type']:02X} "
                        f"seq={frame['seq']} (waiting for seq={seq})"
                    )
                    continue
                keep.append(frame)
            if matched is not None:
                idx, frame = matched
                leftovers = keep + pending[idx + 1:]
                self.urc_queue.extend(unmatched_local + leftovers)
                return frame
            # No match in this batch — keep unrelated frames locally,
            # then drop back to the wire-read path.
            if keep:
                unmatched_local.extend(keep)
            pending = []

    def radio_park(self, timeout: float = RADIO_PARK_TIMEOUT_S,
                   earlier_seqs=()) -> dict:
        """Send RADIO_PARK_REQ (RS-12.21, 2026-10-24) and wait up to
        `timeout` s for the L072's answer to THAT request (its seq) -- or to
        one of `earlier_seqs`, the seqs of this caller's earlier tries that
        drew no reply (2026-10-24 review: a late answer to an earlier try is
        the answer, not a stale frame).

        Not request(): an old image's ERR_PROTO is an answer here (it says
        which firmware this is), not an error to raise. Returns
        {"answer", "status", "status_name", "opmode", "detail", "seq",
        "answer_seq"}: "seq" the seq this call sent, "answer_seq" the seq the
        answer carried (None for "no_reply");
          "urc"           RADIO_PARK_URC: status (RADIO_PARK_STATUS_*; None
                          on an empty payload), status_name, opmode (the
                          RegOpMode readback byte; None when absent);
          "unknown_type"  ERR_PROTO UNKNOWN_TYPE: an image from before the
                          command -- the radio was not touched;
          "err_proto"     any other ERR_PROTO for it: the image knows the
                          command but refused the frame (detail = the
                          decoded payload, format_err_proto_payload);
          "no_reply"      nothing within `timeout`.
        Frames that answer something else go back on urc_queue in arrival
        order, as request() leaves them (a FAULT_URC among them has already
        been seen by on_fault_urc, once); a RADIO_PARK_URC with any other seq
        (the late answer to a call that gave up and was not named in
        `earlier_seqs`) is dropped. Raises only what send() raises (a serial
        write timeout)."""
        seq = self.send(HOST_TYPE_RADIO_PARK_REQ)
        mine = {seq, *earlier_seqs}
        deadline = time.time() + timeout
        out = {"answer": "no_reply", "status": None, "status_name": None,
               "opmode": None, "detail": "", "seq": seq, "answer_seq": None}
        pending = list(self.urc_queue)
        self.urc_queue.clear()
        kept = []
        while True:
            for idx, frame in enumerate(pending):
                ftype = frame["type"]
                if ftype == HOST_TYPE_RADIO_PARK_URC and frame["seq"] not in mine:
                    continue
                if (ftype not in (HOST_TYPE_RADIO_PARK_URC,
                                  HOST_TYPE_ERR_PROTO_URC)
                        or frame["seq"] not in mine):
                    kept.append(frame)
                    continue
                self.urc_queue.extend(kept + pending[idx + 1:])
                out["answer_seq"] = frame["seq"]
                payload = bytes(frame["payload"])
                if ftype == HOST_TYPE_RADIO_PARK_URC:
                    status = payload[0] if payload else None
                    out.update(
                        answer="urc", status=status,
                        status_name=(None if status is None else
                                     RADIO_PARK_STATUS_NAMES.get(
                                         status, f"UNKNOWN_0x{status:02X}")),
                        opmode=payload[1] if len(payload) >= 2 else None)
                else:
                    # ERR_PROTO payload: offending_type, offending_ver,
                    # err_code, detail (format_err_proto_payload)
                    unknown = (len(payload) >= 3
                               and payload[2] == HOST_ERR_PROTO_UNKNOWN_TYPE)
                    out.update(answer="unknown_type" if unknown else "err_proto",
                               detail=format_err_proto_payload(payload))
                return out
            pending = []
            if time.time() >= deadline:
                self.urc_queue.extend(kept)
                return out
            pending = self.read_frames(0.1)


def parse_boot(payload: bytes) -> dict:
    if len(payload) < 6:
        raise ValueError("BOOT_URC payload too short")
    return {
        "reset_cause": payload[0],
        "radio_ok": payload[1],
        "radio_version": payload[2],
        "proto_ver": payload[3],
        "schema_ver": payload[4],
        "clock_source_id": payload[5],
    }


def parse_version(payload: bytes) -> dict:
    if len(payload) < 12:
        raise ValueError("VER_URC payload too short")
    proto_ver, schema_ver, major, minor, patch, name_len, git_len, _reserved = payload[:8]
    cap_bitmap = struct.unpack("<I", payload[8:12])[0]
    start = 12
    name = payload[start:start + name_len].decode(errors="replace")
    start += name_len
    git_sha = payload[start:start + git_len].decode(errors="replace")
    return {
        "proto_ver": proto_ver,
        "schema_ver": schema_ver,
        "major": major,
        "minor": minor,
        "patch": patch,
        "name": name,
        "git_sha": git_sha,
        "cap_bitmap": cap_bitmap,
    }


def parse_stats(payload: bytes) -> dict:
    labels = [
        "host_dropped",
        "host_errors",
        "host_queue_full",
        "host_irq_idle",
        "host_irq_ht",
        "host_irq_tc",
        "host_irq_te",
        "radio_dio0",
        "radio_dio1",
        "radio_dio2",
        "radio_dio3",
        "radio_crc_err",
        "radio_rx_ok",
        "radio_tx_ok",
        "radio_tx_abort_lbt",
        "radio_tx_abort_airtime",
        "radio_state",
        "host_rx_bytes",
        "host_rx_lpuart_bytes",
        "host_rx_usart1_bytes",
        "host_parse_ok",
        "host_parse_err",
        "host_uart_err_lpuart",
        "host_uart_err_usart1",
        # v1.2 (W1-7 RX) additive: per-flag UART error counters + RX ring overflow.
        # Older firmware (96-byte payload) simply omits these — handled by the
        # `offset + 4 <= len(payload)` guard below.
        "host_uart_pe_lpuart",
        "host_uart_fe_lpuart",
        "host_uart_ne_lpuart",
        "host_uart_ore_lpuart",
        "host_uart_pe_usart1",
        "host_uart_fe_usart1",
        "host_uart_ne_usart1",
        "host_uart_ore_usart1",
        "host_rx_ring_ovf",
        # RS-11.5 (2026-08-02) additive: TX FIFO readback discriminator.
        "tx_fifo_rb_ok",
        "tx_fifo_rb_bad",
        "tx_done_early",
        # RS-12 (2026-09-12) additive: URC-path loss accounting.
        "rx_urc_lost",
        "rx_pretx_drained",
        # RS-12.10 (2026-09-12) additive: FIFO skip + TX deaf window.
        "rx_fifo_skip",
        "tx_deaf_max_us",
        "tx_deaf_sum_us",
        "tx_done_to_rearm_max_us",
        # RS-12.15 v2 (2026-09-14) additive: FHSS clock authority.
        "fhss_dec_aligned",
        "fhss_dec_snapped",
        "fhss_dec_rej_not_init",
        "fhss_dec_rej_bad_hop",
        "fhss_dec_rej_epoch_drift",
        "fhss_dec_rej_locked_out",
        "clk_demotion_reset",
        "clk_demotion_kept",
        "tx_first_anchor",
        "tx_stream_streak_max",
    ]
    stats = {}
    for index, label in enumerate(labels):
        offset = index * 4
        if offset + 4 <= len(payload):
            stats[label] = struct.unpack("<I", payload[offset:offset + 4])[0]
    return stats


def dump_registers(link: HostLink, start: int = 0x00, end: int = REGISTER_DUMP_END) -> dict:
    regs = {}
    for addr in range(start, end + 1):
        frame = link.request(HOST_TYPE_REG_READ_REQ, HOST_TYPE_REG_DATA_URC, bytes([addr]), timeout=0.4)
        payload = frame["payload"]
        if len(payload) != 2 or payload[0] != addr:
            raise RuntimeError(f"bad REG_DATA payload for 0x{addr:02X}: {payload.hex()}")
        regs[addr] = payload[1]
    return regs


def print_register_dump(regs: dict) -> None:
    print("SX1276 register dump:")
    for row_base in range(0x00, REGISTER_DUMP_END + 1, 0x10):
        cols = []
        for addr in range(row_base, min(row_base + 0x10, REGISTER_DUMP_END + 1)):
            cols.append(f"{regs[addr]:02X}")
        print(f"  0x{row_base:02X}: {' '.join(cols)}")


def print_raw_bytes(label: str, data: bytes) -> None:
    if not data:
        print(f"{label}: no bytes observed")
        return

    printable = ''.join(chr(byte) if 0x20 <= byte <= 0x7E else '.' for byte in data)
    print(f"{label}: {len(data)} byte(s)")
    print(f"  hex: {data.hex()}")
    print(f"  txt: {printable}")


def _fault_sub_detail(code: int, sub: int) -> str:
    """The decoded-`sub` token format_fault_payload() has always printed for
    the four host-transport codes, or "" for any other code. Shared with
    format_fault_urc_line() so the two cannot drift (2026-10-24)."""
    if code == HOST_FAULT_CODE_HOST_RX_SEEN:
        lanes = []
        if sub & HOST_RX_SEEN_FLAG_LPUART:
            lanes.append("LPUART1")
        if sub & HOST_RX_SEEN_FLAG_USART1:
            lanes.append("USART1")
        lane_text = ",".join(lanes) if lanes else "none"
        return f"lanes={lane_text}"

    if code == HOST_FAULT_CODE_HOST_RX_INACTIVE:
        return "reason=no_ingress_during_probe_window"

    if code == HOST_FAULT_CODE_HOST_PARSE_ERROR:
        return f"parse_err_count_hint={sub}"

    if code == HOST_FAULT_CODE_HOST_DIAG_MARK:
        marks = []
        if sub & HOST_DIAG_MARK_VER_REQ_PARSED:
            marks.append("VER_REQ_PARSED")
        if sub & HOST_DIAG_MARK_FRAME_PARSE_ERR:
            marks.append("FRAME_PARSE_ERR")
        if sub & HOST_DIAG_MARK_VER_REQ_DISPATCHED:
            marks.append("VER_REQ_DISPATCHED")
        if sub & HOST_DIAG_MARK_VER_URC_SENT:
            marks.append("VER_URC_SENT")
        if sub & HOST_DIAG_MARK_AT_VER_DISPATCHED:
            marks.append("AT_VER_DISPATCHED")
        marks_text = "|".join(marks) if marks else "none"
        return f"marks={marks_text}"

    return ""


def format_fault_payload(payload: bytes) -> str:
    # Output is byte-identical to the pre-2026-10-24 version: bench tooling
    # greps `code=0x.. sub=0x..` out of these lines (run_live_radio_monitor
    # .ps1 Test-InjectFaultLine) and tests pin it on archived payloads.
    if len(payload) < 2:
        return payload.hex()

    code = payload[0]
    sub = payload[1]
    detail = _fault_sub_detail(code, sub)
    if detail:
        return f"code=0x{code:02X} sub=0x{sub:02X} {detail} raw={payload.hex()}"
    return f"code=0x{code:02X} sub=0x{sub:02X} raw={payload.hex()}"


def rx_scan_fail_sub_valid(sub: int) -> bool:
    """True when `sub` is a byte sx1276_rx_scan_fail_eval() can emit.

    With SX1276_RX_SCAN_MAX_RETRIES = 3 the firmware sets `final` exactly
    when the attempt nibble reaches 3 (retries reset only on LOCK or
    sx1276_rx_scan_reset; option (b)'s restarts of a node without
    originator authority keep the count at 3, so every later 30 s attempt
    is final with attempt 3 again), and `hw_suspect` only when `crc_seen`
    is clear -- 25 of the 256 values (incl. the NULL-input marker 0x80).
    Every other 0x0D sub can only be the TX_FIFO_RB_MISMATCH tx_id of an
    image from before the 0x0D split.
    """
    if sub == RX_SCAN_FAIL_SUB_FINAL:          # defensive NULL-input marker
        return True
    if (sub & RX_SCAN_FAIL_SUB_HW_SUSPECT) and (sub & RX_SCAN_FAIL_SUB_CRC_SEEN):
        return False
    attempt = sub & RX_SCAN_FAIL_SUB_ATTEMPT_MASK
    if sub & RX_SCAN_FAIL_SUB_FINAL:
        return attempt == RX_SCAN_MAX_RETRIES
    return attempt < RX_SCAN_MAX_RETRIES


def decode_fault_0x0d(sub: int, tx_ids_in_flight=None) -> dict:
    """Resolve a code-0x0D FAULT_URC (2026-10-24, leg-6 lesson; the 0x0D
    split, FHSS_SCAN_FAILED_OPTIONS.md section 7).

    0x0D is RX_SCAN_FAILED; TX_FIFO_RB_MISMATCH moved to 0x0F. An image
    from before the split sends both as 0x0D. Its VER capability bitmap says
    which image it is (HOST_CAP_RADIO_PARK set: the split), but the FAULT_URC
    hooks that call this do not have it, so the sub-byte decides (on a split
    image every 0x0D sub is a valid scan encoding, so the first case below is
    then the only one):
      * a sub sx1276_rx_scan_fail_eval() can emit (rx_scan_fail_sub_valid)
        -> RX_SCAN_FAILED, whatever the TX state. `basis` keeps the in-flight
        check of PR #166, which still says something about a pre-split image
        only (its FIFO fault fires on a TX's TX_DONE, before that TX's
        TX_DONE_URC, i.e. while it is in flight):
          tx_id_not_in_flight  no TX with tx_id == sub awaited its TX_DONE:
                               a scan fault on any image;
          tx_id_in_flight      one did: on a pre-split image this URC could
                               also be that TX's FIFO readback mismatch (e.g.
                               sub 0x00 against fragment 0; PR #166's daemons
                               named it AMBIGUOUS). Read the board's
                               tx_fifo_rb_bad counter (0 in every archived
                               reading) if it matters;
          no_tx_context        the caller passed None;
      * any other sub -> TX_FIFO_RB_MISMATCH of a pre-split image (tx_id =
        sub, basis=not_scan_encoding, old_fw True): no image after the split
        sends 0x0D with it.
    Returns {"name", "basis", "scan" (dict or None), "tx_id" (int or None),
    "old_fw" (bool)}: "scan" is set for RX_SCAN_FAILED, "tx_id" and
    "old_fw" for the pre-split FIFO fault.
    """
    if rx_scan_fail_sub_valid(sub):
        scan = {
            "attempt": sub & RX_SCAN_FAIL_SUB_ATTEMPT_MASK,
            "final": int(bool(sub & RX_SCAN_FAIL_SUB_FINAL)),
            "warm": int(bool(sub & RX_SCAN_FAIL_SUB_WARM)),
            "hw_suspect": int(bool(sub & RX_SCAN_FAIL_SUB_HW_SUSPECT)),
            "crc_seen": int(bool(sub & RX_SCAN_FAIL_SUB_CRC_SEEN)),
        }
        if tx_ids_in_flight is None:
            basis = "no_tx_context"
        elif sub in tx_ids_in_flight:
            basis = "tx_id_in_flight"
        else:
            basis = "tx_id_not_in_flight"
        return {"name": "RX_SCAN_FAILED", "basis": basis, "scan": scan,
                "tx_id": None, "old_fw": False}
    return {"name": "TX_FIFO_RB_MISMATCH", "basis": "not_scan_encoding",
            "scan": None, "tx_id": sub, "old_fw": True}


# Upper case on purpose: the case-sensitive FAULT greps (rs12_leg_report.py
# FAULT_RE, the 10-24 rescore.py `"FAULT" in l`) must see this line.
FAULT_URC_LOG_TAG = "FAULT_URC:"


def format_fault_urc_line(payload: bytes, tx_ids_in_flight=None) -> str:
    """The ONE `FAULT_URC:` log line both strict-path daemons write per
    FAULT_URC (2026-10-24). Space-separated key=value tokens, in order:

      FAULT_URC: code=0xCC sub=0xSS name=NAME [detail...] uptime_ms=N raw=HEX

    `code=0xCC sub=0xSS` stays adjacent (format_fault_payload's grep shape).
    Count FAULTs on `FAULT_URC: code=0x.. sub=0x.. name=` alone: the same URC
    can also leave a legacy line in the same log -- the tractor's `FAULT
    during pipelined TX:`, or the untimestamped stdout `FAULT_URC during TX
    wait:` / `FAULT_URC during settle` (wait_for_tx_done, drain_boot; both
    daemons) -- and no legacy line carries `name=`.
    NAME is HOST_FAULT_CODE_NAMES[code], UNKNOWN, or for 0x0D
    RX_SCAN_FAILED or (a pre-split image's FIFO fault) TX_FIFO_RB_MISMATCH
    (decode_fault_0x0d). Logs written by PR #166's daemons (2026-10-24,
    before the 0x0D split) can also hold name=AMBIGUOUS; this function no
    longer writes it.
    Detail tokens: 0x0D -> attempt= final= warm= hw_suspect= crc_seen= (an
    RX_SCAN_FAILED), or tx_id=0xSS fw=old (the pre-split FIFO fault), then
    basis= and tx_inflight= (comma list, '-' none, '?' unknown); 0x0E and
    0x0F -> tx_id=; 0x04 -> opmode=; 0x08 -> clock_source_id=; 0x09-0x0C ->
    the format_fault_payload token; pc= lr= psr= bfar= when any is non-zero
    or the code is a CPU fault (a replayed record from the previous boot).
    uptime_ms is payload[20:24] -- the L072's platform_now_ms() when the URC
    was SENT (host_cmd.c), '?' on a short payload.
    """
    raw = payload.hex()
    if len(payload) < 2:
        return f"{FAULT_URC_LOG_TAG} short_payload len={len(payload)} raw={raw}"
    code = payload[0]
    sub = payload[1]
    parts = [f"{FAULT_URC_LOG_TAG} code=0x{code:02X} sub=0x{sub:02X}"]
    if code == HOST_FAULT_CODE_RX_SCAN_FAILED:
        d = decode_fault_0x0d(sub, tx_ids_in_flight)
        parts.append(f"name={d['name']}")
        if d["scan"] is not None:
            parts.append("attempt={attempt} final={final} warm={warm} "
                         "hw_suspect={hw_suspect} crc_seen={crc_seen}"
                         .format(**d["scan"]))
        if d["tx_id"] is not None:
            parts.append(f"tx_id=0x{d['tx_id']:02X}")
        if d["old_fw"]:
            parts.append("fw=old")
        parts.append(f"basis={d['basis']}")
        if tx_ids_in_flight is None:
            parts.append("tx_inflight=?")
        else:
            parts.append("tx_inflight=" + (",".join(
                f"0x{t:02X}" for t in sorted(tx_ids_in_flight)) or "-"))
    else:
        parts.append(f"name={HOST_FAULT_CODE_NAMES.get(code, 'UNKNOWN')}")
        detail = _fault_sub_detail(code, sub)
        if detail:
            parts.append(detail)
        elif code in (HOST_FAULT_CODE_TX_DONE_EARLY,
                      HOST_FAULT_CODE_TX_FIFO_RB_MISMATCH):
            parts.append(f"tx_id=0x{sub:02X}")
        elif code == HOST_FAULT_CODE_RADIO_OPMODE_DRIFT:
            parts.append(f"opmode=0x{sub:02X}")
        elif code == HOST_FAULT_CODE_CLOCK_HSE_FAILED:
            parts.append(f"clock_source_id={sub}")
    if len(payload) >= 24:
        pc, lr, psr, bfar, uptime_ms = struct.unpack("<5I", payload[4:24])
        if (pc or lr or psr or bfar or code in (
                HOST_FAULT_CODE_HARDFAULT, HOST_FAULT_CODE_NMI,
                HOST_FAULT_CODE_STACK_GUARD)):
            parts.append(f"pc=0x{pc:08X} lr=0x{lr:08X} psr=0x{psr:08X} "
                         f"bfar=0x{bfar:08X}")
        parts.append(f"uptime_ms={uptime_ms}")
    else:
        parts.append("uptime_ms=?")
    parts.append(f"raw={raw}")
    return " ".join(parts)


def format_err_proto_payload(payload: bytes) -> str:
    # Firmware emits ERR_PROTO payload as 5 bytes:
    #   off  size  field
    #   ---- ----  -------------------------
    #    0    1   offending_type
    #    1    1   offending_ver
    #    2    1   err_code
    #    3    2   detail (uint16, little-endian)
    # Older callers also produce 6-byte payloads with a reserved gap byte;
    # accept both so legacy logs continue to decode.
    if len(payload) < 5:
        return payload.hex()

    offending_type = payload[0]
    offending_ver = payload[1]
    err_code = payload[2]
    if len(payload) >= 6:
        # Legacy 6-byte layout with reserved gap at payload[3].
        detail = struct.unpack("<H", payload[4:6])[0]
    else:
        # Current 5-byte layout: detail is bytes [3:5].
        detail = struct.unpack("<H", payload[3:5])[0]
    err_names = {
        HOST_ERR_PROTO_BAD_VERSION: "BAD_VERSION",
        HOST_ERR_PROTO_UNKNOWN_TYPE: "UNKNOWN_TYPE",
        HOST_ERR_PROTO_BAD_LENGTH: "BAD_LENGTH",
        HOST_ERR_PROTO_BAD_CRC: "BAD_CRC",
        HOST_ERR_PROTO_BAD_COBS: "BAD_COBS",
        HOST_ERR_PROTO_TOO_LARGE: "TOO_LARGE",
        HOST_ERR_PROTO_QUEUE_FULL: "QUEUE_FULL",
        HOST_ERR_PROTO_FORBIDDEN: "FORBIDDEN",
    }
    err_name = err_names.get(err_code, "UNKNOWN")
    return (
        f"offending_type=0x{offending_type:02X} "
        f"offending_ver={offending_ver} "
        f"err_code=0x{err_code:02X}({err_name}) "
        f"detail={detail} raw={payload.hex()}"
    )


def format_ready_payload(payload: bytes) -> str:
    if len(payload) < 8:
        return payload.hex()

    ready_flags = payload[0]
    proto = payload[1]
    schema = payload[2]
    at_shell = payload[3]
    cap = struct.unpack("<I", payload[4:8])[0]
    return (
        f"flags=0x{ready_flags:02X} "
        f"proto={proto} schema={schema} "
        f"at_shell={at_shell} cap=0x{cap:08X}"
    )


def decode_frames_from_raw(data: bytes) -> list:
    frames = []
    index = 0
    while index < len(data):
        if data[index] != 0:
            index += 1
            continue

        end = data.find(b"\x00", index + 1)
        if end < 0:
            break

        payload = data[index + 1:end]
        index = end + 1
        if not payload:
            continue

        try:
            frames.append(parse_frame(payload))
        except ValueError:
            continue

    return frames


def format_frame_summary(frame: dict) -> str:
    frame_type = frame["type"]
    payload = frame["payload"]

    if frame_type == HOST_TYPE_BOOT_URC:
        try:
            boot = parse_boot(payload)
            details = (
                f"reset_cause={boot['reset_cause']} "
                f"radio_ok={boot['radio_ok']} "
                f"radio_version=0x{boot['radio_version']:02X} "
                f"proto={boot['proto_ver']} schema={boot['schema_ver']} "
                f"clock_source_id={boot['clock_source_id']}"
            )
        except ValueError:
            details = payload.hex()
        return f"type=0x{frame_type:02X}(BOOT_URC) seq={frame['seq']} {details}"

    if frame_type == HOST_TYPE_READY_URC:
        return (
            f"type=0x{frame_type:02X}(READY_URC) seq={frame['seq']} "
            f"{format_ready_payload(payload)}"
        )

    if frame_type == HOST_TYPE_FAULT_URC:
        return (
            f"type=0x{frame_type:02X}(FAULT_URC) seq={frame['seq']} "
            f"{format_fault_payload(payload)}"
        )

    if frame_type == HOST_TYPE_ERR_PROTO_URC:
        return (
            f"type=0x{frame_type:02X}(ERR_PROTO_URC) seq={frame['seq']} "
            f"{format_err_proto_payload(payload)}"
        )

    if frame_type == HOST_TYPE_STATS_URC:
        stats = parse_stats(payload)
        return (
            f"type=0x{frame_type:02X}(STATS_URC) seq={frame['seq']} "
            f"host_rx_bytes={stats.get('host_rx_bytes', 0)} "
            f"host_parse_err={stats.get('host_parse_err', 0)} "
            f"host_errors={stats.get('host_errors', 0)}"
        )

    return (
        f"type=0x{frame_type:02X} seq={frame['seq']} "
        f"payload_len={len(payload)} raw={payload.hex()}"
    )


def print_decoded_frames(label: str, data: bytes) -> None:
    frames = decode_frames_from_raw(data)
    if not frames:
        return

    print(f"{label}: decoded {len(frames)} host frame(s)")
    for frame in frames:
        print(f"  {format_frame_summary(frame)}")


def run_ascii_diagnostics(link: HostLink) -> None:
    print("ASCII fallback diagnostics:")
    pre_drain = link.drain_raw()
    print_raw_bytes("  pre-drain", pre_drain)
    print_decoded_frames("  pre-drain", pre_drain)

    for cmd in ("ATI\r\n", "AT+VER?\r\n"):
        link.send_ascii(cmd)
        time.sleep(0.1)
        reply = link.read_raw(timeout=0.6, max_bytes=256)
        print_raw_bytes(f"  reply to {cmd.strip()}", reply)
        print_decoded_frames(f"  reply to {cmd.strip()}", reply)


def print_uart_flag_counters(label: str, stats: dict) -> None:
    """v1.3 Phase D2: emit per-flag UART error counters from the 132-byte STATS payload.

    Older firmware (96-byte payload) simply omits these keys, in which case
    parse_stats() leaves them out of the dict and we print a note.
    """
    new_keys = (
        "host_uart_pe_lpuart",
        "host_uart_fe_lpuart",
        "host_uart_ne_lpuart",
        "host_uart_ore_lpuart",
        "host_uart_pe_usart1",
        "host_uart_fe_usart1",
        "host_uart_ne_usart1",
        "host_uart_ore_usart1",
        "host_rx_ring_ovf",
    )
    if not any(k in stats for k in new_keys):
        print(f"{label} per-flag: <not present in payload (legacy 96B firmware)>")
        return
    print(
        f"{label} per-flag LPUART1: "
        f"PE={stats.get('host_uart_pe_lpuart', 0)} "
        f"FE={stats.get('host_uart_fe_lpuart', 0)} "
        f"NE={stats.get('host_uart_ne_lpuart', 0)} "
        f"ORE={stats.get('host_uart_ore_lpuart', 0)}"
    )
    print(
        f"{label} per-flag USART1:  "
        f"PE={stats.get('host_uart_pe_usart1', 0)} "
        f"FE={stats.get('host_uart_fe_usart1', 0)} "
        f"NE={stats.get('host_uart_ne_usart1', 0)} "
        f"ORE={stats.get('host_uart_ore_usart1', 0)}"
    )
    print(f"{label} ring overflow:  rx_ring_ovf={stats.get('host_rx_ring_ovf', 0)}")


def attempt_stats_dump_on_failure(link: HostLink) -> None:
    """v1.3 Phase D2: best-effort STATS capture after VER_REQ timeout.

    Drains any pending frames first, then issues STATS_DUMP_REQ with a
    generous timeout so the per-flag UART error counters reach stdout.
    """
    print("Phase D2: attempting STATS_DUMP_REQ after failure")
    pending = link.read_frames(0.2)
    for frame in pending:
        if frame["type"] == HOST_TYPE_STATS_URC:
            stats = parse_stats(frame["payload"])
            print(f"  drained late STATS_URC seq={frame['seq']} payload_len={len(frame['payload'])}")
            print_uart_flag_counters("  drained STATS_URC", stats)
        elif frame["type"] == HOST_TYPE_FAULT_URC:
            print(f"  drained FAULT_URC: {format_fault_payload(frame['payload'])}")
        elif frame["type"] == HOST_TYPE_ERR_PROTO_URC:
            print(f"  drained ERR_PROTO_URC: {format_err_proto_payload(frame['payload'])}")
    frame = link.request(HOST_TYPE_STATS_DUMP_REQ, HOST_TYPE_STATS_URC, timeout=2.0)
    payload = frame["payload"]
    stats = parse_stats(payload)
    print(f"  STATS_URC(D2) payload_len={len(payload)} seq={frame['seq']}")
    print(
        "  STATS_URC(D2): "
        f"host_rx_bytes={stats.get('host_rx_bytes', 0)} "
        f"host_rx_lpuart={stats.get('host_rx_lpuart_bytes', 0)} "
        f"host_rx_usart1={stats.get('host_rx_usart1_bytes', 0)} "
        f"host_parse_ok={stats.get('host_parse_ok', 0)} "
        f"host_parse_err={stats.get('host_parse_err', 0)} "
        f"uart_err_lpuart={stats.get('host_uart_err_lpuart', 0)} "
        f"uart_err_usart1={stats.get('host_uart_err_usart1', 0)}"
    )
    print_uart_flag_counters("  STATS_URC(D2)", stats)


def main() -> int:
    parser = argparse.ArgumentParser(description="Method G Stage 1 custom-firmware probe")
    parser.add_argument("--dev", default=DEV_DEFAULT, help=f"UART device (default: {DEV_DEFAULT})")
    parser.add_argument("--baud", default=BAUD_DEFAULT, help=f"UART baud (default: {BAUD_DEFAULT})")
    parser.add_argument("--boot-timeout", type=float, default=1.0, help="time to wait for BOOT_URC")
    args = parser.parse_args()

    print("=" * 60)
    print("Method G Stage 1 Probe")
    print(f"Device: {args.dev} @ {args.baud} 8N1")
    print("Expected target: custom murata_l072 firmware host link")
    print("=" * 60)

    try:
        link = HostLink(args.dev, args.baud)
    except Exception as exc:
        print(f"FATAL: cannot open/configure {args.dev}: {exc}")
        return 2

    try:
        boot_info = None
        for frame in link.read_frames(args.boot_timeout):
            if frame["type"] == HOST_TYPE_BOOT_URC:
                boot_info = parse_boot(frame["payload"])
            elif frame["type"] == HOST_TYPE_STATS_URC:
                stats = parse_stats(frame["payload"])
                print(
                    "STATS_URC(init): "
                    f"host_rx_bytes={stats.get('host_rx_bytes', 0)} "
                    f"host_rx_lpuart={stats.get('host_rx_lpuart_bytes', 0)} "
                    f"host_rx_usart1={stats.get('host_rx_usart1_bytes', 0)} "
                    f"host_parse_ok={stats.get('host_parse_ok', 0)} "
                    f"host_parse_err={stats.get('host_parse_err', 0)} "
                    f"uart_err_lpuart={stats.get('host_uart_err_lpuart', 0)} "
                    f"uart_err_usart1={stats.get('host_uart_err_usart1', 0)}"
                )
                print_uart_flag_counters("STATS_URC(init)", stats)
            elif frame["type"] == HOST_TYPE_FAULT_URC:
                print(f"FAULT_URC: {format_fault_payload(frame['payload'])}")
            elif frame["type"] == HOST_TYPE_READY_URC:
                print(f"READY_URC: {format_ready_payload(frame['payload'])}")
            elif frame["type"] == HOST_TYPE_ERR_PROTO_URC:
                print(f"ERR_PROTO_URC(init): {format_err_proto_payload(frame['payload'])}")

        if boot_info is not None:
            print(
                "BOOT_URC: "
                f"reset_cause={boot_info['reset_cause']} "
                f"radio_ok={boot_info['radio_ok']} "
                f"radio_version=0x{boot_info['radio_version']:02X} "
                f"proto={boot_info['proto_ver']} "
                f"schema={boot_info['schema_ver']} "
                f"clock_source_id={boot_info['clock_source_id']}"
            )
        else:
            print("BOOT_URC: not observed during initial window (continuing with active queries)")

        version = parse_version(link.request(HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC, timeout=1.0)["payload"])
        print(
            "VER_URC: "
            f"name={version['name']} "
            f"version={version['major']}.{version['minor']}.{version['patch']} "
            f"git={version['git_sha']} "
            f"cap=0x{version['cap_bitmap']:08X}"
        )

        stats = parse_stats(link.request(HOST_TYPE_STATS_DUMP_REQ, HOST_TYPE_STATS_URC, timeout=1.0)["payload"])
        print(
            "STATS_URC: "
            f"radio_state={stats.get('radio_state', 0)} "
            f"radio_rx_ok={stats.get('radio_rx_ok', 0)} "
            f"radio_tx_ok={stats.get('radio_tx_ok', 0)} "
            f"host_errors={stats.get('host_errors', 0)} "
            f"host_rx_bytes={stats.get('host_rx_bytes', 0)} "
            f"host_rx_lpuart={stats.get('host_rx_lpuart_bytes', 0)} "
            f"host_rx_usart1={stats.get('host_rx_usart1_bytes', 0)} "
            f"host_parse_err={stats.get('host_parse_err', 0)} "
            f"uart_err_lpuart={stats.get('host_uart_err_lpuart', 0)} "
            f"uart_err_usart1={stats.get('host_uart_err_usart1', 0)}"
        )
        print_uart_flag_counters("STATS_URC", stats)

        regs = dump_registers(link)
        version_reg = regs[SX1276_REG_VERSION]
        print(f"REG 0x{SX1276_REG_VERSION:02X} VERSION = 0x{version_reg:02X}")
        print_register_dump(regs)

        checks = []
        if boot_info is not None:
            checks.append(("clock_source_id == 0", boot_info["clock_source_id"] == 0))
            checks.append(("radio_ok == 1", boot_info["radio_ok"] == 1))
        checks.append(("ver name present", bool(version["name"])))
        checks.append(("sx1276 version reg valid", version_reg not in (0x00, 0xFF)))

        print()
        print("Critical checks:")
        failed = 0
        for label, ok in checks:
            status = "PASS" if ok else "FAIL"
            print(f"  [{status}] {label}")
            if not ok:
                failed += 1

        print()
        if failed:
            print(f"VERDICT: FAIL ({failed} critical check(s) failed)")
            return 1

        print("VERDICT: PASS (Stage 1 protocol and register-access checks passed)")
        return 0
    except Exception as exc:
        # v1.3 Phase D2: on any failure (most importantly VER_REQ timeout)
        # try to drain a STATS_URC so the per-flag UART error counters in
        # the 132-byte payload land in stdout. Best-effort; never raises.
        try:
            attempt_stats_dump_on_failure(link)
        except Exception as stats_exc:
            print(f"Phase D2 stats-on-failure capture failed: {stats_exc}")
        try:
            run_ascii_diagnostics(link)
        except Exception as diag_exc:
            print(f"ASCII fallback diagnostics failed: {diag_exc}")
        print(f"FATAL: probe failed: {exc}")
        return 2
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())