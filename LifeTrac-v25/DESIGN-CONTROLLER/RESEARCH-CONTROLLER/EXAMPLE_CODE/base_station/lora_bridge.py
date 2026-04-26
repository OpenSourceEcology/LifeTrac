"""lora_bridge.py — LoRa <-> MQTT bridge running on the Portenta X8 Linux side.

DRAFT FOR REVIEW. Not run yet.

The X8's H747 co-MCU runs base.ino (a thin RadioLib wrapper) and pipes
KISS-framed bytes to/from this script over UART. We:

  - Decode KISS frames, decrypt with AES-128-GCM, parse the LoraHeader.
  - Republish telemetry + control-source state on Mosquitto under
    `lifetrac/v25/...` topics.
  - Subscribe to control commands (from web_ui.py via MQTT) and TX them.

Run:
    python lora_bridge.py /dev/ttymxc0
"""

from __future__ import annotations

import argparse
import logging
import os
import struct
import sys
import threading
import time
from dataclasses import dataclass

import paho.mqtt.client as mqtt
import serial
from cryptography.hazmat.primitives.ciphers.aead import AESGCM

# ---- shared with lora_proto.h ------------------------------------------
PROTO_VERSION = 0x01
SRC_HANDHELD, SRC_BASE, SRC_TRACTOR, SRC_AUTONOMY = 0x01, 0x02, 0x03, 0x04
FT_CONTROL, FT_TELEMETRY, FT_COMMAND, FT_HEARTBEAT = 0x10, 0x20, 0x30, 0x40

KISS_FEND, KISS_FESC, KISS_TFEND, KISS_TFESC = 0xC0, 0xDB, 0xDC, 0xDD

# 16-byte pre-shared fleet key. PROVISIONED ELSEWHERE.
FLEET_KEY = bytes(16)


# ---- KISS framing ------------------------------------------------------
def kiss_encode(data: bytes) -> bytes:
    out = bytearray([KISS_FEND])
    for b in data:
        if b == KISS_FEND:
            out += bytes([KISS_FESC, KISS_TFEND])
        elif b == KISS_FESC:
            out += bytes([KISS_FESC, KISS_TFESC])
        else:
            out.append(b)
    out.append(KISS_FEND)
    return bytes(out)


class KissDecoder:
    def __init__(self) -> None:
        self.buf = bytearray()
        self.in_frame = False
        self.escape = False

    def feed(self, b: int):
        """Yield complete frames as bytes."""
        if b == KISS_FEND:
            if self.in_frame and self.buf:
                frame = bytes(self.buf)
                self.buf.clear()
                self.in_frame = False
                self.escape = False
                yield frame
            else:
                self.in_frame = True
                self.buf.clear()
                self.escape = False
            return
        if not self.in_frame:
            return
        if self.escape:
            if b == KISS_TFEND:
                b = KISS_FEND
            elif b == KISS_TFESC:
                b = KISS_FESC
            self.escape = False
        elif b == KISS_FESC:
            self.escape = True
            return
        self.buf.append(b)


# ---- frame parsing -----------------------------------------------------
@dataclass
class LoraHeader:
    version: int
    source_id: int
    frame_type: int
    sequence_num: int


HEADER_LEN = 5
CTRL_FRAME_LEN = 16
HB_FRAME_LEN = 10
TELEM_HEADER_LEN = HEADER_LEN + 2   # + topic_id + payload_len
TELEM_MAX_PAYLOAD = 128
CRC_LEN = 2


def parse_header(pt: bytes) -> LoraHeader | None:
    if len(pt) < HEADER_LEN:
        return None
    return LoraHeader(*struct.unpack("<BBBH", pt[:HEADER_LEN]))


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT (poly 0x1021, init 0xFFFF) — matches lora_proto.c."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def verify_crc(frame: bytes) -> bool:
    """Last 2 bytes are little-endian CRC over the preceding bytes."""
    if len(frame) < CRC_LEN + 1:
        return False
    body, tail = frame[:-CRC_LEN], frame[-CRC_LEN:]
    expected = crc16_ccitt(body)
    actual = struct.unpack("<H", tail)[0]
    return expected == actual


def decrypt(onair: bytes) -> bytes | None:
    if len(onair) < 12 + 16:
        return None
    nonce, ct = onair[:12], onair[12:]
    try:
        return AESGCM(FLEET_KEY).decrypt(nonce, ct, None)
    except Exception:
        return None


def encrypt(seq: int, source_id: int, pt: bytes) -> bytes:
    nonce = (
        bytes([source_id])
        + struct.pack("<H", seq & 0xFFFF)
        + struct.pack("<I", int(time.time()) & 0xFFFFFFFF)
        + os.urandom(5)
    )
    ct = AESGCM(FLEET_KEY).encrypt(nonce, pt, None)
    return nonce + ct


# ---- MQTT topic mapping -----------------------------------------------
TOPIC_BY_ID = {
    0x01: "lifetrac/v25/telemetry/gps",
    0x02: "lifetrac/v25/telemetry/engine",
    0x03: "lifetrac/v25/telemetry/battery",
    0x04: "lifetrac/v25/telemetry/hydraulics",
    0x05: "lifetrac/v25/telemetry/mode",
    0x06: "lifetrac/v25/telemetry/errors",
    0x10: "lifetrac/v25/control/source_active",
    0x20: "lifetrac/v25/video/thumbnail",
}


# ---- bridge -----------------------------------------------------------
class Bridge:
    def __init__(self, port: str, mqtt_host: str = "localhost") -> None:
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        self.dec = KissDecoder()
        self.mqtt = mqtt.Client(client_id="lora_bridge")
        self.mqtt.on_message = self._on_mqtt_message
        self.mqtt.connect(mqtt_host, 1883)
        self.mqtt.subscribe("lifetrac/v25/cmd/control")  # from web_ui
        self.mqtt.subscribe("lifetrac/v25/cmd/estop")
        self.mqtt.loop_start()
        self.tx_seq = 0
        self.lock = threading.Lock()

    # ---- inbound: serial → MQTT -----------------------------------
    def run(self) -> None:
        logging.info("bridge running on %s", self.ser.port)
        while True:
            chunk = self.ser.read(256)
            for b in chunk:
                for frame in self.dec.feed(b):
                    self._handle_air(frame)

    def _handle_air(self, onair: bytes) -> None:
        pt = decrypt(onair)
        if pt is None:
            logging.debug("decrypt failed (%d bytes)", len(onair))
            return
        hdr = parse_header(pt)
        if hdr is None or hdr.version != PROTO_VERSION:
            return

        if hdr.frame_type == FT_TELEMETRY:
            # Layout: [hdr(5)|topic_id(1)|payload_len(1)|payload(N)|crc16(2)]
            if len(pt) < TELEM_HEADER_LEN + CRC_LEN:
                logging.debug("telemetry too short: %d", len(pt))
                return
            topic_id = pt[HEADER_LEN]
            payload_len = pt[HEADER_LEN + 1]
            if payload_len > TELEM_MAX_PAYLOAD:
                logging.debug("telemetry payload_len %d > max", payload_len)
                return
            expected_total = TELEM_HEADER_LEN + payload_len + CRC_LEN
            if len(pt) < expected_total:
                logging.debug("telemetry truncated: have %d need %d",
                              len(pt), expected_total)
                return
            frame = pt[:expected_total]
            if not verify_crc(frame):
                logging.warning("telemetry CRC failed (topic=%#x)", topic_id)
                return
            payload = frame[TELEM_HEADER_LEN : TELEM_HEADER_LEN + payload_len]
            topic = TOPIC_BY_ID.get(topic_id, f"lifetrac/v25/raw/{topic_id:02x}")
            self.mqtt.publish(topic, payload, qos=0, retain=False)

        elif hdr.frame_type == FT_HEARTBEAT:
            if len(pt) < HB_FRAME_LEN:
                return
            if not verify_crc(pt[:HB_FRAME_LEN]):
                logging.warning("heartbeat CRC failed (src=%#x)", hdr.source_id)
                return
            self.mqtt.publish(
                "lifetrac/v25/status/heartbeat",
                struct.pack("<BH", hdr.source_id, hdr.sequence_num),
            )

        elif hdr.frame_type == FT_CONTROL:
            # Bridge does not normally re-publish control frames, but if it
            # does (debug/forensic), validate first.
            if len(pt) < CTRL_FRAME_LEN or not verify_crc(pt[:CTRL_FRAME_LEN]):
                return
            self.mqtt.publish(
                f"lifetrac/v25/raw/control/{hdr.source_id:02x}",
                pt[:CTRL_FRAME_LEN], qos=0, retain=False,
            )

    # ---- outbound: MQTT → serial ----------------------------------
    def _on_mqtt_message(self, _client, _userdata, msg):
        if msg.topic.endswith("/cmd/control"):
            # Expect a fully-formed 16-byte ControlFrame from web_ui.py.
            # Validate length + CRC before transmitting — otherwise a malformed
            # payload from the LAN side becomes a wasted air packet.
            if len(msg.payload) != CTRL_FRAME_LEN:
                logging.warning("cmd/control wrong length: %d", len(msg.payload))
                return
            if not verify_crc(msg.payload):
                logging.warning("cmd/control CRC failed")
                return
            self._tx(SRC_BASE, msg.payload)
        elif msg.topic.endswith("/cmd/estop"):
            # FT_COMMAND payload: [hdr(5) | opcode(1) | args(...) | crc16(2)]
            # opcode 0x01 = CMD_ESTOP. We let the bridge stamp seq=0 here; the
            # tractor's replay window tolerates seq=0 as the wakeup case, and
            # the latched-fault is idempotent.
            body = struct.pack("<BBBHB",
                               PROTO_VERSION, SRC_BASE, FT_COMMAND, 0, 0x01)
            crc = crc16_ccitt(body)
            cmd = body + struct.pack("<H", crc)
            self._tx(SRC_BASE, cmd)

    def _tx(self, source_id: int, pt: bytes) -> None:
        with self.lock:
            seq = self.tx_seq
            self.tx_seq = (self.tx_seq + 1) & 0xFFFF
        wire = encrypt(seq, source_id, pt)
        self.ser.write(kiss_encode(wire))


def main() -> None:
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")
    p = argparse.ArgumentParser()
    p.add_argument("port", help="serial device for the H747 link, e.g. /dev/ttymxc0")
    p.add_argument("--mqtt", default="localhost")
    args = p.parse_args()
    Bridge(args.port, args.mqtt).run()


if __name__ == "__main__":
    main()
