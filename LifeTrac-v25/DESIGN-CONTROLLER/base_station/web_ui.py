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
import threading
from pathlib import Path
from typing import Any

import paho.mqtt.client as mqtt
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

from lora_proto import SRC_AUTONOMY, SRC_BASE, SRC_HANDHELD, SRC_NONE, pack_control

WEB_DIR = Path(__file__).parent / "web"

app = FastAPI(title="LifeTrac v25 Base Station")
app.mount("/static", StaticFiles(directory=WEB_DIR), name="static")

# ---- MQTT plumbing -----------------------------------------------------
mqtt_client = mqtt.Client(client_id="web_ui")
mqtt_client.connect("localhost", 1883)
mqtt_client.loop_start()

telemetry_subscribers: set[WebSocket] = set()
active_source = "none"
active_source_lock = threading.Lock()


SOURCE_BY_ID = {
    0x00: "none",
    SRC_HANDHELD: "handheld",
    SRC_BASE: "base",
    0x03: "autonomy",  # priority enum value used by heartbeat/source telemetry
    SRC_AUTONOMY: "autonomy",
    SRC_NONE: "none",
}


def _source_name(value: Any) -> str | None:
    if isinstance(value, str):
        name = value.strip().lower()
        if name in {"none", "handheld", "base", "autonomy"}:
            return name
        return None
    if isinstance(value, int):
        return SOURCE_BY_ID.get(value)
    return None


def _set_active_source(name: str) -> None:
    global active_source
    with active_source_lock:
        active_source = name


def _base_controls_allowed() -> bool:
    with active_source_lock:
        return active_source not in {"handheld", "autonomy"}


def _decode_payload(topic: str, payload: bytes) -> Any:
    if topic.endswith("/active_camera") and len(payload) == 1:
        return payload[0]
    if topic.endswith("/source_active") and len(payload) == 1:
        return _source_name(payload[0]) or payload[0]
    stripped = payload.lstrip()
    if stripped.startswith((b"{", b"[")):
        try:
            return json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            pass
    return payload.hex()


def _on_mqtt_message(_c, _u, msg):
    data = _decode_payload(msg.topic, msg.payload)
    if msg.topic.endswith("/source_active"):
        candidate = None
        if isinstance(data, dict):
            candidate = _source_name(data.get("active_source"))
        else:
            candidate = _source_name(data)
        if candidate is not None:
            _set_active_source(candidate)

    payload = {"topic": msg.topic, "data": data}
    loop = getattr(app.state, "loop", None)
    if loop is None:
        return
    for ws in list(telemetry_subscribers):
        asyncio.run_coroutine_threadsafe(
            ws.send_text(json.dumps(payload)), loop
        )


mqtt_client.on_message = _on_mqtt_message
mqtt_client.subscribe("lifetrac/v25/telemetry/#")
mqtt_client.subscribe("lifetrac/v25/status/#")
mqtt_client.subscribe("lifetrac/v25/video/#")
mqtt_client.subscribe("lifetrac/v25/control/#")


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
            if not _base_controls_allowed():
                continue
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
