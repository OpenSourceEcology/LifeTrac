"""lora_bridge.py — LoRa <-> MQTT bridge running on the Portenta X8 Linux side.

DRAFT FOR REVIEW. Not run yet.

The v25 master plan pins the base station to Linux-on-X8 control of the SX1276
over SPI. This bridge currently keeps the serial transport path as a bench
fallback while the in-tree SPI driver is filled in. We:

  - Decode KISS frames, decrypt with AES-128-GCM, parse the LoraHeader.
  - Republish telemetry + control-source state on Mosquitto under
    `lifetrac/v25/...` topics.
  - Subscribe to control commands (from web_ui.py via MQTT) and TX them.

Run:
    python lora_bridge.py /dev/ttymxc0
"""

from __future__ import annotations

import argparse
import heapq
import itertools
import json
import logging
import struct
import threading
import time

import paho.mqtt.client as mqtt
import serial

from audit_log import DEFAULT_PATH as AUDIT_LOG_DEFAULT_PATH, AuditLog
from link_monitor import EncodeModeController, RollingAirtimeLedger
from lora_proto import (
    CMD_CAMERA_SELECT,
    CMD_ESTOP,
    CMD_LINK_TUNE,
    CTRL_FRAME_LEN,
    CRC_LEN,
    FT_COMMAND,
    FT_CONTROL,
    FT_HEARTBEAT,
    FT_TELEMETRY,
    HB_FRAME_LEN,
    HEADER_LEN,
    KissDecoder,
    PROTO_VERSION,
    ReplayWindow,
    SRC_BASE,
    TELEM_HEADER_LEN,
    TELEM_MAX_PAYLOAD,
    attribute_phy,
    classify_priority,
    decrypt_frame,
    encrypt_frame,
    kiss_encode,
    pack_command,
    parse_header,
    topic_name,
    verify_crc,
)

# Airtime alarm thresholds per LORA_IMPLEMENTATION.md \u00a77.
U_TELEMETRY_ALARM = 0.30
U_TOTAL_ALARM = 0.60
# How often to poll the ledger + emit CMD_ENCODE_MODE if needed.
AIRTIME_POLL_INTERVAL_S = 1.0

# 16-byte pre-shared fleet key. PROVISIONED ELSEWHERE.
FLEET_KEY = bytes(16)

# ---- bridge -----------------------------------------------------------
class Bridge:
    def __init__(self, port: str, mqtt_host: str = "localhost",
                 audit_path: str = AUDIT_LOG_DEFAULT_PATH) -> None:
        # Audit log first — every other constructor step may want to record.
        self.audit = AuditLog(audit_path)
        self.audit.record("bridge_start", port=port, mqtt_host=mqtt_host)
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        self.dec = KissDecoder()
        self.mqtt = mqtt.Client(client_id="lora_bridge")
        self.mqtt.on_message = self._on_mqtt_message
        self.mqtt.connect(mqtt_host, 1883)
        self.mqtt.subscribe("lifetrac/v25/cmd/control")  # from web_ui
        self.mqtt.subscribe("lifetrac/v25/cmd/estop")
        self.mqtt.subscribe("lifetrac/v25/cmd/camera_select")
        self.mqtt.loop_start()
        self.tx_seq = 0
        self.lock = threading.Lock()
        # Per-source replay defence (mirrors LpReplayWindow on the tractor).
        # Keyed by header.source_id so a noisy peer can't poison another peer.
        self._replay: dict[int, ReplayWindow] = {}
        # Airtime ledger + encode-mode controller (LORA_IMPLEMENTATION.md \u00a74, \u00a77).
        self.ledger = RollingAirtimeLedger(window_ms=10_000)
        self.encode_ctrl = EncodeModeController(required_windows=3)
        # Priority TX queue (LORA_IMPLEMENTATION.md §4). All TX goes through a
        # single dedicated worker so the serial write is never racy and so
        # P0 frames preempt anything queued behind them.
        self._tx_queue: list[tuple[int, int, bytes]] = []
        self._tx_cv = threading.Condition()
        self._tx_counter = itertools.count()
        self._stop_evt = threading.Event()
        self._tx_thread = threading.Thread(
            target=self._tx_worker, daemon=True, name="tx"
        )
        self._tx_thread.start()
        self._airtime_thread = threading.Thread(
            target=self._airtime_worker, daemon=True, name="airtime"
        )
        self._airtime_thread.start()

    # ---- inbound: serial → MQTT -----------------------------------
    def run(self) -> None:
        logging.info("bridge running on %s", self.ser.port)
        while True:
            chunk = self.ser.read(256)
            for b in chunk:
                for frame in self.dec.feed(b):
                    self._handle_air(frame)

    def _handle_air(self, onair: bytes) -> None:
        pt = decrypt_frame(FLEET_KEY, onair)
        if pt is None:
            logging.debug("decrypt failed (%d bytes)", len(onair))
            self.audit.log_gcm_reject(onair_len=len(onair))
            return
        hdr = parse_header(pt)
        if hdr is None or hdr.version != PROTO_VERSION:
            self.audit.record("bad_header",
                              version=(hdr.version if hdr else None),
                              pt_len=len(pt))
            return
        # Per-source replay window check. The C side does this on the tractor
        # with an LpReplayWindow; we mirror it here so a base-side replay
        # doesn't clobber telemetry derived from the cleartext.
        if not self._replay_check(hdr.source_id, hdr.sequence_num):
            self.audit.log_replay_reject(
                source_id=hdr.source_id, seq=hdr.sequence_num)
            return
        self.audit.record("rx",
                          source_id=hdr.source_id,
                          frame_type=hdr.frame_type,
                          seq=hdr.sequence_num,
                          pt_len=len(pt))

        # Attribute received airtime to the appropriate PHY profile so the
        # ledger reflects the full channel utilization, not just our own TX.
        topic_id = pt[HEADER_LEN] if hdr.frame_type == FT_TELEMETRY and len(pt) > HEADER_LEN else None
        self.ledger.record(_now_ms(), attribute_phy(hdr.frame_type, topic_id),
                           cleartext_len=len(pt), encrypted=True)

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
            self.mqtt.publish(topic_name(topic_id), payload, qos=0, retain=False)

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

        elif hdr.frame_type == FT_COMMAND:
            # Inbound commands from the tractor. The only one we currently
            # care about is CMD_LINK_TUNE — surface it on MQTT so the radio
            # supervisor (today: the serial firmware on the X8 carrier;
            # tomorrow: the in-tree SPI driver per MASTER_PLAN.md §8.17) can
            # retune the base-side radio. We do NOT retune here because the
            # current bridge talks to the radio over an opaque KISS serial link.
            if len(pt) < HEADER_LEN + 4:
                return
            opcode = pt[HEADER_LEN]
            if opcode == CMD_LINK_TUNE:
                target_sf = pt[HEADER_LEN + 1]
                target_bw_code = pt[HEADER_LEN + 2]
                reason = pt[HEADER_LEN + 3]
                logging.info("CMD_LINK_TUNE rx: SF=%d bw_code=%d reason=%#x",
                             target_sf, target_bw_code, reason)
                self.audit.record("link_tune",
                                  target_sf=target_sf,
                                  target_bw_code=target_bw_code,
                                  reason=reason)
                self.mqtt.publish(
                    "lifetrac/v25/control/link_tune",
                    json.dumps({
                        "target_sf": target_sf,
                        "target_bw_code": target_bw_code,
                        "reason": reason,
                    }).encode(),
                    qos=0, retain=True,
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
            # FT_COMMAND payload: [hdr(5) | opcode(1) | args(...) | crc16(2)].
            # E-stop is latched and idempotent, but it still gets a fresh
            # sequence number so the tractor replay window does not discard it.
            seq = self._reserve_tx_seq()
            cmd = pack_command(seq, CMD_ESTOP)
            self._tx(SRC_BASE, cmd, nonce_seq=seq)
        elif msg.topic.endswith("/cmd/camera_select"):
            # opcode 0x03 = CMD_CAMERA_SELECT, payload = 1B camera_id
            # (0=auto, 1=front, 2=rear, 3=implement, 4=crop). See LORA_PROTOCOL.md.
            if len(msg.payload) != 1:
                logging.warning("cmd/camera_select wrong length: %d", len(msg.payload))
                return
            cam_id = msg.payload[0]
            if cam_id > 0x04:
                logging.warning("cmd/camera_select unknown id: %#x", cam_id)
                return
            seq = self._reserve_tx_seq()
            cmd = pack_command(seq, CMD_CAMERA_SELECT, bytes([cam_id]))
            self._tx(SRC_BASE, cmd, nonce_seq=seq)

    def _reserve_tx_seq(self) -> int:
        with self.lock:
            seq = self.tx_seq
            self.tx_seq = (self.tx_seq + 1) & 0xFFFF
            return seq

    def _replay_check(self, source_id: int, seq: int) -> bool:
        """Per-source 64-frame replay window check. Returns True if fresh.

        Lazily allocates a ReplayWindow on first sight of a source so unknown
        peers (e.g. a future SRC_AUTONOMY) don't need pre-registration.
        """
        with self.lock:
            window = self._replay.get(source_id)
            if window is None:
                window = ReplayWindow()
                self._replay[source_id] = window
            return window.check_and_update(seq)

    def _tx(self, source_id: int, pt: bytes, nonce_seq: int | None = None) -> None:
        """Enqueue a cleartext frame for transmission.

        The encrypt + write happens on the dedicated TX worker so the serial
        port has exactly one writer (no interleaved bytes) and so P0 traffic
        always wins against any queued P2/P3.
        """
        seq = self._reserve_tx_seq() if nonce_seq is None else nonce_seq
        # Classify priority from the cleartext frame header.
        frame_type = pt[2] if len(pt) > 2 else FT_COMMAND
        opcode = pt[HEADER_LEN] if frame_type == FT_COMMAND and len(pt) > HEADER_LEN else None
        topic_id = pt[HEADER_LEN] if frame_type == FT_TELEMETRY and len(pt) > HEADER_LEN else None
        prio = classify_priority(frame_type, opcode, topic_id)
        item = (source_id, seq, pt)
        with self._tx_cv:
            heapq.heappush(self._tx_queue,
                           (prio, next(self._tx_counter), item))
            self._tx_cv.notify()

    def _tx_worker(self) -> None:
        """Single TX writer: pops highest-priority frame, encrypts, writes serial,
        records airtime. Per LORA_IMPLEMENTATION.md §4 priority queue.
        """
        while not self._stop_evt.is_set():
            with self._tx_cv:
                while not self._tx_queue and not self._stop_evt.is_set():
                    self._tx_cv.wait(timeout=0.5)
                if self._stop_evt.is_set():
                    return
                _prio, _ord, (source_id, seq, pt) = heapq.heappop(self._tx_queue)
            try:
                wire = encrypt_frame(FLEET_KEY, source_id, seq, pt)
                # Attribute TX airtime by frame type (and topic for FT_TELEMETRY).
                frame_type = pt[2] if len(pt) > 2 else FT_COMMAND
                topic_id = pt[HEADER_LEN] if frame_type == FT_TELEMETRY and len(pt) > HEADER_LEN else None
                self.ledger.record(_now_ms(), attribute_phy(frame_type, topic_id),
                                   cleartext_len=len(pt), encrypted=True)
                self.ser.write(kiss_encode(wire))
                self.audit.record("tx",
                                  source_id=source_id,
                                  frame_type=frame_type,
                                  seq=seq,
                                  pt_len=len(pt))
            except Exception:
                logging.exception("tx worker write failed")
                self.audit.record("tx_error", source_id=source_id, seq=seq)

    # ---- airtime worker: poll ledger, emit CMD_ENCODE_MODE, publish 0x10 ----
    def _airtime_worker(self) -> None:
        """Periodically observe utilization, emit CMD_ENCODE_MODE on rung change,
        publish the airtime triple for the operator UI, and log alarm thresholds.

        Per LORA_IMPLEMENTATION.md \u00a74 + \u00a77.
        """
        last_alarm_state = (False, False)
        while not self._stop_evt.wait(AIRTIME_POLL_INTERVAL_S):
            try:
                util = self.ledger.utilization(_now_ms())
                # 1. Mode-change detection (3-window hysteresis lives in EncodeModeController).
                new_mode = self.encode_ctrl.observe(util)
                if new_mode is not None:
                    seq = self._reserve_tx_seq()
                    cmd = self.encode_ctrl.command_frame(seq, new_mode)
                    logging.info("CMD_ENCODE_MODE \u2192 %s (U_image=%.1f%%)",
                                 new_mode.name, util.image * 100)
                    self.audit.record("encode_mode_change",
                                      mode=new_mode.name,
                                      u_image=round(util.image, 4),
                                      u_telemetry=round(util.telemetry, 4),
                                      u_total=round(util.total, 4))
                    self._tx(SRC_BASE, cmd, nonce_seq=seq)
                # 2. Publish airtime triple alongside the source-active topic
                #    so the UI has one place to read everything link-related.
                #    NOTE: topic 0x10 (`control/source_active`) is owned by the
                #    tractor's 1 Hz active-source byte, so airtime lands on a
                #    sibling retained topic to avoid clobbering it.
                payload = json.dumps({
                    "u_image": round(util.image, 4),
                    "u_telemetry": round(util.telemetry, 4),
                    "u_total": round(util.total, 4),
                    "encode_mode": self.encode_ctrl.mode.name,
                }).encode()
                self.mqtt.publish("lifetrac/v25/control/link_airtime",
                                  payload, qos=0, retain=True)
                # 3. Alarm logging \u2014 squelch repeat lines so the journal stays useful.
                tel_alarm = util.telemetry > U_TELEMETRY_ALARM
                tot_alarm = util.total > U_TOTAL_ALARM
                if (tel_alarm, tot_alarm) != last_alarm_state:
                    if tel_alarm:
                        logging.warning("U_telemetry %.1f%% exceeds %.0f%% alarm",
                                        util.telemetry * 100, U_TELEMETRY_ALARM * 100)
                    if tot_alarm:
                        logging.warning("U_total %.1f%% exceeds %.0f%% alarm \u2014 P0 headroom at risk",
                                        util.total * 100, U_TOTAL_ALARM * 100)
                    self.audit.record("airtime_alarm",
                                      u_telemetry=round(util.telemetry, 4),
                                      u_total=round(util.total, 4),
                                      telemetry_alarm=tel_alarm,
                                      total_alarm=tot_alarm)
                    last_alarm_state = (tel_alarm, tot_alarm)
            except Exception:
                logging.exception("airtime worker iteration failed")


def _now_ms() -> int:
    return int(time.monotonic() * 1000)


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
