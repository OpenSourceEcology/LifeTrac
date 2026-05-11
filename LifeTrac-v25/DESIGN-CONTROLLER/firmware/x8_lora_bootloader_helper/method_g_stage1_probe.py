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

HOST_FAULT_CODE_HOST_RX_SEEN = 0x09
HOST_FAULT_CODE_HOST_RX_INACTIVE = 0x0A
HOST_FAULT_CODE_HOST_PARSE_ERROR = 0x0B
HOST_FAULT_CODE_HOST_DIAG_MARK = 0x0C

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
                        frames.append(parse_frame(payload))
                    except ValueError as exc:
                        print(f"WARNING: dropped malformed frame: {exc}")
                        continue
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
        while True:
            if not pending:
                if time.time() >= deadline:
                    raise TimeoutError(
                        f"timeout waiting for response type 0x{rsp_type:02X} "
                        f"to req 0x{req_type:02X}"
                    )
                pending = self.read_frames(0.1)
            keep = []
            matched = None
            for idx, frame in enumerate(pending):
                if frame["type"] == rsp_type and frame["seq"] == seq:
                    matched = (idx, frame)
                    break
                if frame["type"] == HOST_TYPE_ERR_PROTO_URC and frame["seq"] == seq:
                    self.urc_queue.extend(keep + pending[idx + 1:])
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
                if leftovers:
                    self.urc_queue.extend(leftovers)
                return frame
            # No match in this batch — keep unrelated frames for later
            # consumers, then drop back to the wire-read path.
            if keep:
                self.urc_queue.extend(keep)
            pending = []


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


def format_fault_payload(payload: bytes) -> str:
    if len(payload) < 2:
        return payload.hex()

    code = payload[0]
    sub = payload[1]
    if code == HOST_FAULT_CODE_HOST_RX_SEEN:
        lanes = []
        if sub & HOST_RX_SEEN_FLAG_LPUART:
            lanes.append("LPUART1")
        if sub & HOST_RX_SEEN_FLAG_USART1:
            lanes.append("USART1")
        lane_text = ",".join(lanes) if lanes else "none"
        return f"code=0x{code:02X} sub=0x{sub:02X} lanes={lane_text} raw={payload.hex()}"

    if code == HOST_FAULT_CODE_HOST_RX_INACTIVE:
        return f"code=0x{code:02X} sub=0x{sub:02X} reason=no_ingress_during_probe_window raw={payload.hex()}"

    if code == HOST_FAULT_CODE_HOST_PARSE_ERROR:
        return f"code=0x{code:02X} sub=0x{sub:02X} parse_err_count_hint={sub} raw={payload.hex()}"

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
        return f"code=0x{code:02X} sub=0x{sub:02X} marks={marks_text} raw={payload.hex()}"

    return f"code=0x{code:02X} sub=0x{sub:02X} raw={payload.hex()}"


def format_err_proto_payload(payload: bytes) -> str:
    if len(payload) < 6:
        return payload.hex()

    offending_type = payload[0]
    offending_ver = payload[1]
    err_code = payload[2]
    detail = struct.unpack("<H", payload[4:6])[0]
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