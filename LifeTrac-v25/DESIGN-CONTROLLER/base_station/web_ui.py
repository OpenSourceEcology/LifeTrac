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
import hmac
import json
import logging
import os
import secrets
import threading
import time
from collections import defaultdict
from pathlib import Path
from typing import Any

import paho.mqtt.client as mqtt
from fastapi import Cookie, Depends, FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect, status
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from lora_proto import SRC_AUTONOMY, SRC_BASE, SRC_HANDHELD, SRC_NONE, pack_control

WEB_DIR = Path(__file__).parent / "web"

# ---- PIN auth (MASTER_PLAN.md §8.5 / DECISIONS.md D-E1) -----------------
# Single shared PIN, LAN-only, plain HTTP for v25. Telemetry views are public;
# control/E-stop/camera-select require the PIN. The hardware E-stop on the
# base is auth-independent.
LIFETRAC_PIN = os.environ.get("LIFETRAC_PIN", "").strip()  # 4-6 digits, set at first boot
SESSION_TTL_S = 30 * 60          # 30 min idle timeout
LOCKOUT_FAILS = 5                # wrong PINs before lockout
LOCKOUT_S     = 60               # seconds of cool-off

_sessions: dict[str, float] = {}             # token -> last_used_ts
_sessions_lock = threading.Lock()
_fail_counts: dict[str, tuple[int, float]] = defaultdict(lambda: (0, 0.0))   # ip -> (n_fails, lockout_until)
_fail_lock = threading.Lock()


def _new_session_token() -> str:
    return secrets.token_urlsafe(32)


def _session_valid(token: str | None) -> bool:
    if not token:
        return False
    now = time.monotonic()
    with _sessions_lock:
        last = _sessions.get(token)
        if last is None:
            return False
        if now - last > SESSION_TTL_S:
            _sessions.pop(token, None)
            return False
        _sessions[token] = now
        return True


def _client_ip(request: Request) -> str:
    return (request.client.host if request.client else "unknown")


def _check_lockout(ip: str) -> float:
    """Return seconds remaining in lockout, or 0 if not locked."""
    now = time.monotonic()
    with _fail_lock:
        n_fails, locked_until = _fail_counts[ip]
        if locked_until > now:
            return locked_until - now
        return 0.0


def _record_failure(ip: str) -> None:
    now = time.monotonic()
    with _fail_lock:
        n_fails, _ = _fail_counts[ip]
        n_fails += 1
        if n_fails >= LOCKOUT_FAILS:
            _fail_counts[ip] = (0, now + LOCKOUT_S)
            logging.warning("web_ui: PIN lockout for %s (%d fails)", ip, n_fails)
        else:
            _fail_counts[ip] = (n_fails, 0.0)


def _record_success(ip: str) -> None:
    with _fail_lock:
        _fail_counts[ip] = (0, 0.0)


def _require_session(session: str | None = Cookie(default=None)) -> str:
    if not _session_valid(session):
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED,
                            detail="PIN required")
    return session   # type: ignore[return-value]


app = FastAPI(title="LifeTrac v25 Base Station")
app.mount("/static", StaticFiles(directory=WEB_DIR), name="static")

# ---- MQTT plumbing -----------------------------------------------------
mqtt_client = mqtt.Client(client_id="web_ui")
mqtt_client.connect("localhost", 1883)
mqtt_client.loop_start()

telemetry_subscribers: set[WebSocket] = set()
image_subscribers: set[WebSocket] = set()
state_subscribers: set[WebSocket] = set()
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
    # Image frames stream as raw bytes on a separate WS so we don't pay
    # the JSON-encode cost on every fragment. The reassembled canvas is
    # published by image_pipeline/reassemble.py on
    # `lifetrac/v25/video/canvas` as a single WebP/PNG blob the browser
    # can draw directly via createImageBitmap.
    if msg.topic.endswith("/video/canvas"):
        loop_img = getattr(app.state, "loop", None)
        if loop_img is not None:
            for ws in list(image_subscribers):
                asyncio.run_coroutine_threadsafe(
                    ws.send_bytes(msg.payload), loop_img
                )
        return

    if msg.topic.endswith("/video/tile_delta") or msg.topic.endswith("/cmd/image_frame"):
        _ingest_tile_delta(bytes(msg.payload))
        return

    data = _decode_payload(msg.topic, msg.payload)
    if msg.topic.endswith("/source_active"):
        candidate = None
        if isinstance(data, dict):
            candidate = _source_name(data.get("active_source"))
        else:
            candidate = _source_name(data)
        if candidate is not None:
            _set_active_source(candidate)
    _maybe_capture_params(msg.topic, data)

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
mqtt_client.subscribe("lifetrac/v25/params/changed")


# ---- image-pipeline state publisher (bucket A wiring) ------------------
# Reassemble TileDeltaFrame messages → Canvas → StatePublisher snapshots.
# Snapshot pushes happen ~2 Hz (every 500 ms) plus immediately after every
# canvas update. Browser modules in /static/img/ subscribe to /ws/state.
from image_pipeline.canvas import Canvas               # noqa: E402
from image_pipeline.reassemble import FragmentReassembler  # noqa: E402
from image_pipeline.state_publisher import StatePublisher  # noqa: E402

_image_canvas = Canvas()
_image_reassembler = FragmentReassembler()
_image_publisher = StatePublisher(canvas=_image_canvas)
_image_lock = threading.Lock()


def _publish_state_snapshot() -> None:
    loop = getattr(app.state, "loop", None)
    if loop is None or not state_subscribers:
        return
    snap = json.dumps(_image_publisher.snapshot())
    for ws in list(state_subscribers):
        asyncio.run_coroutine_threadsafe(ws.send_text(snap), loop)


def _ingest_tile_delta(payload: bytes) -> None:
    with _image_lock:
        frame = _image_reassembler.feed(payload)
        if frame is None:
            return
        update = _image_canvas.apply(frame)
        if update.request_keyframe:
            _image_publisher.needs_keyframe = True
            _image_publisher.last_keyframe_reason = update.reason
            try:
                mqtt_client.publish("lifetrac/v25/cmd/req_keyframe",
                                    update.reason.encode()[:64], qos=1)
            except Exception:
                pass
        else:
            _image_publisher.needs_keyframe = False
            _image_publisher.last_keyframe_reason = ""
    _publish_state_snapshot()


# ---- params cache (proxied from tractor X8 over MQTT) ------------------
# The authoritative store lives on the tractor X8's params_service.py and
# publishes a retained snapshot on `lifetrac/v25/params/changed`. We mirror
# that snapshot here so the operator UI can render the current state
# without round-tripping every page load. Writes are sent as a JSON patch
# on `lifetrac/v25/params/set`; the tractor side validates and applies.
_params_cache: dict[str, Any] = {}
_params_lock = threading.Lock()


def _maybe_capture_params(topic: str, data: Any) -> None:
    if topic.endswith("/params/changed") and isinstance(data, dict):
        with _params_lock:
            _params_cache.clear()
            _params_cache.update(data)


# ---- routes ------------------------------------------------------------
@app.on_event("startup")
async def _startup():
    app.state.loop = asyncio.get_event_loop()


@app.get("/", response_class=HTMLResponse)
async def index(session: str | None = Cookie(default=None)):
    # Redirect to the PIN page when there is no valid session — keeps the
    # operator UI behind D-E1 auth without bolting auth into the static SPA.
    if not _session_valid(session):
        return HTMLResponse(
            status_code=status.HTTP_303_SEE_OTHER,
            content="",
            headers={"Location": "/login"},
        )
    return (WEB_DIR / "index.html").read_text(encoding="utf-8")


@app.get("/login", response_class=HTMLResponse)
async def login_page():
    return (WEB_DIR / "login.html").read_text(encoding="utf-8")


@app.get("/settings", response_class=HTMLResponse)
async def settings_page(session: str | None = Cookie(default=None)):
    if not _session_valid(session):
        return HTMLResponse(status_code=status.HTTP_303_SEE_OTHER,
                            content="", headers={"Location": "/login"})
    return (WEB_DIR / "settings.html").read_text(encoding="utf-8")


@app.get("/audit", response_class=HTMLResponse)
async def audit_page(session: str | None = Cookie(default=None)):
    if not _session_valid(session):
        return HTMLResponse(status_code=status.HTTP_303_SEE_OTHER,
                            content="", headers={"Location": "/login"})
    return (WEB_DIR / "audit.html").read_text(encoding="utf-8")


@app.get("/diagnostics", response_class=HTMLResponse)
async def diagnostics_page(session: str | None = Cookie(default=None)):
    """Link-health page: airtime %, SF rung history, FHSS heatmap, link loss."""
    if not _session_valid(session):
        return HTMLResponse(status_code=status.HTTP_303_SEE_OTHER,
                            content="", headers={"Location": "/login"})
    return (WEB_DIR / "diagnostics.html").read_text(encoding="utf-8")


@app.get("/map", response_class=HTMLResponse)
async def map_page(session: str | None = Cookie(default=None)):
    """Live GPS map with offline tile fallback."""
    if not _session_valid(session):
        return HTMLResponse(status_code=status.HTTP_303_SEE_OTHER,
                            content="", headers={"Location": "/login"})
    return (WEB_DIR / "map.html").read_text(encoding="utf-8")


@app.get("/api/audit")
async def api_audit(limit: int = 200,
                    _session: str = Depends(_require_session)):
    """Return the last `limit` lines of the active audit JSONL file.

    Cheap tail: read the file in 64 KiB chunks from the end until we have
    `limit` newlines or hit BOF. Avoids loading multi-MB files into RAM
    just to render the last screenful in the operator UI.
    """
    from audit_log import DEFAULT_PATH as _AUDIT_PATH
    import os
    path = os.environ.get("LIFETRAC_AUDIT_PATH", _AUDIT_PATH)
    limit = max(1, min(int(limit), 5000))
    try:
        size = os.path.getsize(path)
    except OSError:
        return {"path": path, "lines": [], "error": "audit file not found"}
    chunk = 65536
    out_lines: list[str] = []
    with open(path, "rb") as fp:
        offset = size
        tail = b""
        while offset > 0 and len(out_lines) <= limit:
            read_size = min(chunk, offset)
            offset -= read_size
            fp.seek(offset)
            tail = fp.read(read_size) + tail
            out_lines = tail.split(b"\n")
        # drop trailing empty (final newline) and decode the last `limit`
        decoded = [ln.decode("utf-8", errors="replace")
                   for ln in out_lines if ln.strip()]
        decoded = decoded[-limit:]
    parsed: list[Any] = []
    for ln in decoded:
        try:
            parsed.append(json.loads(ln))
        except json.JSONDecodeError:
            parsed.append({"raw": ln})
    return {"path": path, "lines": parsed}


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


@app.websocket("/ws/image")
async def ws_image(ws: WebSocket):
    """Binary frame stream of the reassembled tile-delta canvas.

    Each message is a single complete image blob (WebP today, possibly
    JPEG later) ready for `createImageBitmap` on the browser side.
    Unauthenticated reads are allowed: the canvas is operationally the
    same surface as the dashboard telemetry and FastAPI's cookie Depends
    doesn't run for WebSockets anyway.
    """
    await ws.accept()
    image_subscribers.add(ws)
    try:
        while True:
            await ws.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        image_subscribers.discard(ws)


@app.websocket("/ws/state")
async def ws_state(ws: WebSocket):
    """Authoritative server-side image state (tiles + age + badge + detections).

    Per IMAGE_PIPELINE.md §6.1 the browser MUST consume this rather than
    computing display state locally. JSON snapshots are pushed on every
    canvas update plus a 2 Hz keepalive.
    """
    await ws.accept()
    state_subscribers.add(ws)
    try:
        # Send an initial snapshot immediately so a freshly-connected
        # client can render before the next tile arrives.
        await ws.send_text(json.dumps(_image_publisher.snapshot()))
        while True:
            await asyncio.sleep(0.5)
            try:
                await ws.send_text(json.dumps(_image_publisher.snapshot()))
            except Exception:
                break
    except WebSocketDisconnect:
        pass
    finally:
        state_subscribers.discard(ws)


@app.post("/api/health/refusal")
async def api_health_refusal(request: Request):
    """Browser badge_renderer.js posts here when it refuses a tile.

    We append to the audit log via the same channel as MQTT events so
    refusal events become part of the black-box record.
    """
    try:
        body = await request.json()
    except Exception:
        body = {}
    try:
        mqtt_client.publish("lifetrac/v25/control/badge_refusal",
                            json.dumps(body).encode(), qos=1)
    except Exception:
        pass
    return {"ok": True}


@app.post("/api/audit/view_mode")
async def api_audit_view_mode(request: Request):
    """Browser raw_mode_toggle.js posts here whenever the operator flips
    raw mode. Logged to the black-box audit trail."""
    try:
        body = await request.json()
    except Exception:
        body = {}
    try:
        mqtt_client.publish("lifetrac/v25/control/view_mode",
                            json.dumps(body).encode(), qos=1)
    except Exception:
        pass
    return {"ok": True}


@app.websocket("/ws/control")
async def ws_control(ws: WebSocket):
    """Receives JSON {lhx,lhy,rhx,rhy,buttons,flags} at up to 50 Hz from the
    browser. Forwards as a packed ControlFrame to the LoRa bridge over MQTT.
    Server-side rate-limits to 20 Hz to match the air-side cadence.

    Auth: requires a valid `session` cookie (set by POST /api/login). A
    request without one is closed with 4401 immediately so the browser
    cannot stream control frames before unlocking.
    """
    # FastAPI doesn't run the cookie Depends for WebSockets, so we check by hand.
    token = ws.cookies.get("session")
    if not _session_valid(token):
        await ws.close(code=4401)
        return
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
async def estop(_session: str = Depends(_require_session)):
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
async def camera_select(payload: dict, _session: str = Depends(_require_session)):
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


# ---- PIN login routes (MASTER_PLAN.md §8.5) ----------------------------
@app.post("/api/login")
async def api_login(payload: dict, request: Request):
    if not LIFETRAC_PIN:
        # No PIN configured — refuse to grant access rather than fail-open.
        raise HTTPException(status_code=503,
                            detail="LIFETRAC_PIN not configured on server")
    ip = _client_ip(request)
    locked = _check_lockout(ip)
    if locked > 0:
        raise HTTPException(status_code=429,
                            detail=f"locked out, retry in {int(locked)}s")
    submitted = str(payload.get("pin", ""))
    if not hmac.compare_digest(submitted, LIFETRAC_PIN):
        _record_failure(ip)
        raise HTTPException(status_code=401, detail="invalid PIN")
    _record_success(ip)
    token = _new_session_token()
    with _sessions_lock:
        _sessions[token] = time.monotonic()
    resp = JSONResponse({"ok": True})
    # HttpOnly + SameSite=strict per MASTER_PLAN.md §8.5. Secure flag omitted
    # because v25 is plain HTTP on LAN — set `secure=True` if a future
    # deployment puts nginx + TLS in front.
    resp.set_cookie("session", token, httponly=True, samesite="strict",
                    max_age=SESSION_TTL_S, path="/")
    return resp


@app.post("/api/logout")
async def api_logout(session: str | None = Cookie(default=None)):
    if session:
        with _sessions_lock:
            _sessions.pop(session, None)
    resp = JSONResponse({"ok": True})
    resp.delete_cookie("session", path="/")
    return resp


@app.get("/api/session")
async def api_session(session: str | None = Cookie(default=None)):
    return {"authenticated": _session_valid(session)}


# ---- Coral / accelerator settings (DECISIONS.md D-CORAL-1..3) ----------
# Lazy-imported so the dev box can run web_ui without the image_pipeline
# package on PYTHONPATH (e.g. for the auth tests).
def _accel_get():
    from image_pipeline.accel_select import get_accelerator
    return get_accelerator()


def _settings_get():
    from settings_store import get_store
    return get_store()


@app.get("/api/settings/accel")
async def api_accel_get(_session: str = Depends(_require_session)):
    """Return current accelerator state for the operator settings panel.

    Shape: {present, usable, enabled, kind, last_error, detected_at, active}.
    See image_pipeline/accel_select.py for the meaning of each field.
    """
    return _accel_get().state().to_dict()


@app.post("/api/settings/accel")
async def api_accel_set(payload: dict,
                        _session: str = Depends(_require_session)):
    """Operator master toggle for the optional Coral Edge TPU.

    Body: {"enabled": bool}. Persisted to base_settings.json so the choice
    survives reboot. Takes effect within one inference call (per-call
    `is_active()` check in superres.py / detect.py).
    """
    if "enabled" not in payload:
        raise HTTPException(status_code=400, detail="missing 'enabled' bool")
    enabled = bool(payload["enabled"])
    _settings_get().set("image.coral_enabled", enabled)
    state = _accel_get().state()
    return {"ok": True, "enabled": enabled, "active": state.is_active()}


@app.post("/api/settings/accel/refresh")
async def api_accel_refresh(_session: str = Depends(_require_session)):
    """Force a re-detection pass. Covers the 'I just plugged it in / yanked
    it' case without waiting for the 30 s background poll.
    """
    return _accel_get().refresh().to_dict()


# ---- Tractor params proxy (TODO bucket C item 4) -----------------------
@app.get("/api/params")
async def api_params_get(_session: str = Depends(_require_session)):
    """Return the cached tractor parameter snapshot.

    The cache is fed by the retained `lifetrac/v25/params/changed` MQTT
    topic that the tractor X8 `params_service.py` publishes. Empty until
    the broker delivers the first retained payload.
    """
    with _params_lock:
        return dict(_params_cache)


@app.post("/api/params")
async def api_params_set(payload: dict,
                         _session: str = Depends(_require_session)):
    """Forward a JSON patch to the tractor X8 params_service.

    Body is a partial dict shaped like the snapshot from GET /api/params.
    The tractor side validates and rejects unknown keys, so we send the
    patch as-is and rely on the next retained `params/changed` message to
    confirm the apply.
    """
    if not isinstance(payload, dict):
        raise HTTPException(status_code=400,
                            detail="body must be a JSON object")
    mqtt_client.publish("lifetrac/v25/params/set",
                        json.dumps(payload), qos=1)
    return {"ok": True, "submitted": payload}
