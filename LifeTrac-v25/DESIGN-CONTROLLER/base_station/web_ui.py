"""web_ui.py — FastAPI operator console for the LifeTrac v25 base station.

DRAFT FOR REVIEW. Not run yet.

Serves a single HTML page with on-screen virtual joysticks AND USB-gamepad
support via the browser Gamepad API. Pushes ControlFrames to lora_bridge.py
through MQTT (`lifetrac/v25/cmd/control`) at 20 Hz, and streams telemetry to
the page over a WebSocket.

Run:
    uvicorn web_ui:app --host 0.0.0.0 --port 8080
"""

from __future__ import annotations

import asyncio
import json
import logging
import struct
from pathlib import Path
from typing import Any

import paho.mqtt.client as mqtt
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

WEB_DIR = Path(__file__).parent / "web"

app = FastAPI(title="LifeTrac v25 Base Station")
app.mount("/static", StaticFiles(directory=WEB_DIR), name="static")

# ---- MQTT plumbing -----------------------------------------------------
mqtt_client = mqtt.Client(client_id="web_ui")
mqtt_client.connect("localhost", 1883)
mqtt_client.loop_start()

telemetry_subscribers: set[WebSocket] = set()


def _on_telemetry(_c, _u, msg):
    payload = {"topic": msg.topic, "data": msg.payload.hex()}
    for ws in list(telemetry_subscribers):
        asyncio.run_coroutine_threadsafe(
            ws.send_text(json.dumps(payload)), app.state.loop
        )


mqtt_client.message_callback_add("lifetrac/v25/telemetry/+", _on_telemetry)
mqtt_client.message_callback_add("lifetrac/v25/status/+", _on_telemetry)
mqtt_client.subscribe("lifetrac/v25/telemetry/#")
mqtt_client.subscribe("lifetrac/v25/status/#")


# ---- ControlFrame packing — MUST match lora_proto.h ControlFrame -------
PROTO_VERSION = 0x01
SRC_BASE = 0x02
FT_CONTROL = 0x10


def _clip_i8(v: int) -> int:
    if v < -127:
        return -127
    if v > 127:
        return 127
    return v


def pack_control(seq: int, lhx: int, lhy: int, rhx: int, rhy: int,
                 buttons: int, flags: int, hb: int) -> bytes:
    """Returns 16-byte ControlFrame, CRC included.

    Layout matches the C ControlFrame in lora_proto.h byte-for-byte:
      [ver(1)|src(1)|type(1)|seq(2) | lhx|lhy|rhx|rhy(4) | btns(2) | flags(1)
       | hb(1) | reserved=0(1) | crc16(2) ] = 16 bytes.
    """
    body = struct.pack(
        "<BBBHbbbbHBBB",
        PROTO_VERSION, SRC_BASE, FT_CONTROL, seq & 0xFFFF,
        _clip_i8(lhx), _clip_i8(lhy), _clip_i8(rhx), _clip_i8(rhy),
        buttons & 0xFFFF, flags & 0xFF, hb & 0xFF,
        0,  # reserved — must be zero on TX
    )
    # CRC-16/CCITT (poly 0x1021, init 0xFFFF) over body.
    crc = 0xFFFF
    for b in body:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return body + struct.pack("<H", crc)


# ---- routes ------------------------------------------------------------
@app.on_event("startup")
async def _startup():
    app.state.loop = asyncio.get_event_loop()


@app.get("/", response_class=HTMLResponse)
async def index():
    return (WEB_DIR / "index.html").read_text(encoding="utf-8")


@app.websocket("/ws/telemetry")
async def ws_telemetry(ws: WebSocket):
    await ws.accept()
    telemetry_subscribers.add(ws)
    try:
        while True:
            await ws.receive_text()  # keepalive pings; we don't care about content
    except WebSocketDisconnect:
        pass
    finally:
        telemetry_subscribers.discard(ws)


@app.websocket("/ws/control")
async def ws_control(ws: WebSocket):
    """Receives JSON {lhx,lhy,rhx,rhy,buttons,flags} at up to 50 Hz from the
    browser. Forwards as a packed ControlFrame to the LoRa bridge over MQTT.
    Server-side rate-limits to 20 Hz to match the air-side cadence."""
    await ws.accept()
    seq = 0
    hb = 0
    last_tx = 0.0
    try:
        while True:
            raw = await ws.receive_text()
            now = asyncio.get_event_loop().time()
            if now - last_tx < 0.05:    # 20 Hz
                continue
            last_tx = now
            try:
                msg: dict[str, Any] = json.loads(raw)
            except json.JSONDecodeError:
                continue
            frame = pack_control(
                seq=seq,
                lhx=int(msg.get("lhx", 0)),
                lhy=int(msg.get("lhy", 0)),
                rhx=int(msg.get("rhx", 0)),
                rhy=int(msg.get("rhy", 0)),
                buttons=int(msg.get("buttons", 0)),
                flags=int(msg.get("flags", 0)),
                hb=hb,
            )
            mqtt_client.publish("lifetrac/v25/cmd/control", frame, qos=0)
            seq = (seq + 1) & 0xFFFF
            hb = (hb + 1) & 0xFF
    except WebSocketDisconnect:
        pass


@app.post("/api/estop")
async def estop():
    mqtt_client.publish("lifetrac/v25/cmd/estop", b"", qos=1)
    return {"ok": True}


# Camera selection: maps human-readable name -> CMD_CAMERA_SELECT payload byte.
# See LORA_PROTOCOL.md § Command frame opcodes.
_CAMERA_IDS = {
    "auto":      0x00,
    "front":     0x01,
    "rear":      0x02,
    "implement": 0x03,
    "crop":      0x04,
}


@app.post("/api/camera/select")
async def camera_select(payload: dict):
    """Switch which camera the tractor uses for LoRa thumbnails / WebRTC.

    Body: {"camera": "auto"|"front"|"rear"|"implement"|"crop"}
    Publishes a 1-byte payload to lifetrac/v25/cmd/camera_select; lora_bridge
    wraps it in an FT_COMMAND/CMD_CAMERA_SELECT frame and TXes it over LoRa.
    """
    name = str(payload.get("camera", "")).lower()
    if name not in _CAMERA_IDS:
        return {"ok": False, "error": f"unknown camera '{name}'"}
    cam_id = _CAMERA_IDS[name]
    mqtt_client.publish("lifetrac/v25/cmd/camera_select", bytes([cam_id]), qos=1)
    return {"ok": True, "camera": name, "id": cam_id}
