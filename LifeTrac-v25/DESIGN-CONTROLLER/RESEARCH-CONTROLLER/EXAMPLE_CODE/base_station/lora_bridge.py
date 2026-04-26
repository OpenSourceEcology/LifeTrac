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


def parse_header(pt: bytes) -> LoraHeader | None:
    if len(pt) < 5:
        return None
    return LoraHeader(*struct.unpack("<BBBH", pt[:5]))


def decrypt(onair: bytes) -> bytes | None:
    if len(onair) < 12 + 16:
        return None
    nonce, ct = onair[:12], onair[12:]
    try:
        return AESGCM(FLEET_KEY).decrypt(nonce, ct, None)
    except Exception:
        return None


def encrypt(seq: int, source_id: int, pt: bytes) -> bytes:
    nonce = bytes([source_id]) + struct.pack("<H", seq) + struct.pack("<I", int(time.time())) + os.urandom(5)
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
        if hdr.frame_type == FT_TELEMETRY and len(pt) >= 7:
            topic_id, payload_len = pt[5], pt[6]
            payload = pt[7 : 7 + payload_len]
            topic = TOPIC_BY_ID.get(topic_id, f"lifetrac/v25/raw/{topic_id:02x}")
            self.mqtt.publish(topic, payload, qos=0, retain=False)
        elif hdr.frame_type == FT_HEARTBEAT and len(pt) >= 8:
            self.mqtt.publish(
                "lifetrac/v25/status/heartbeat",
                struct.pack("<BH", hdr.source_id, hdr.sequence_num),
            )

    # ---- outbound: MQTT → serial ----------------------------------
    def _on_mqtt_message(self, _client, _userdata, msg):
        if msg.topic.endswith("/cmd/control"):
            # Expect a 16-byte ControlFrame from web_ui.py.
            self._tx(SRC_BASE, msg.payload)
        elif msg.topic.endswith("/cmd/estop"):
            cmd = bytes([PROTO_VERSION, SRC_BASE, FT_COMMAND]) + struct.pack("<H", 0) + bytes([0x01]) + bytes(8) + bytes(2)
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
