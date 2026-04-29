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
from fastapi.responses import HTMLResponse, JSONResponse, PlainTextResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, Field, ValidationError, field_validator

from lora_proto import SRC_AUTONOMY, SRC_BASE, SRC_HANDHELD, SRC_NONE, pack_control


# ---- IP-203: input validation models -----------------------------------
# Every LAN-side surface (WebSocket payloads + JSON bodies) goes through one
# of these models. Range-checks here are tighter than the on-air protocol so
# malformed inputs never reach the LoRa transmitter.

# Allow-list of params the operator may set via /api/params. Anything else
# is rejected with HTTP 400 \u2014 prevents a compromised browser from poking
# at internal MQTT topics that happen to start with "params.".
_ALLOWED_PARAM_KEYS = frozenset({
    "image.coral_enabled",
    "image.encode_quality",
    "image.tile_quality",
    "ui.theme",
    "ui.language",
})

MAX_PARAM_VALUE_LEN = 256        # any string value over this is dropped
MAX_PARAMS_BODY_BYTES = 4096     # whole /api/params POST cap


class ControlMsg(BaseModel):
    """WebSocket payload from /ws/control."""
    model_config = ConfigDict(extra="forbid")
    lhx: int = Field(0, ge=-127, le=127)
    lhy: int = Field(0, ge=-127, le=127)
    rhx: int = Field(0, ge=-127, le=127)
    rhy: int = Field(0, ge=-127, le=127)
    buttons: int = Field(0, ge=0, le=0xFFFF)
    flags: int = Field(0, ge=0, le=0xFF)


class CameraSelectBody(BaseModel):
    model_config = ConfigDict(extra="forbid")
    camera: str = Field(..., min_length=1, max_length=16)


class LoginBody(BaseModel):
    model_config = ConfigDict(extra="forbid")
    pin: str = Field(..., min_length=1, max_length=16)


class AccelToggleBody(BaseModel):
    model_config = ConfigDict(extra="forbid")
    enabled: bool


class ParamsBody(BaseModel):
    """POST /api/params body — only allow-listed keys, scalar values only."""
    model_config = ConfigDict(extra="forbid")
    params: dict[str, Any] = Field(default_factory=dict)

    @field_validator("params")
    @classmethod
    def _check_keys(cls, v: dict[str, Any]) -> dict[str, Any]:
        for k, val in v.items():
            if k not in _ALLOWED_PARAM_KEYS:
                raise ValueError(f"unknown parameter {k!r}")
            if isinstance(val, str) and len(val) > MAX_PARAM_VALUE_LEN:
                raise ValueError(f"value for {k!r} too long")
            if isinstance(val, (list, dict, set, tuple)):
                raise ValueError(f"value for {k!r} must be a scalar")
        return v

WEB_DIR = Path(__file__).parent / "web"

# ---- PIN auth (MASTER_PLAN.md §8.5 / DECISIONS.md D-E1) -----------------
# Single shared PIN, LAN-only, plain HTTP for v25. Telemetry views are public;
# control/E-stop/camera-select require the PIN. The hardware E-stop on the
# base is auth-independent.
def _load_operator_pin() -> str:
    """Load the operator PIN from a Docker secret file or env var.

    Source order (matches IP-002 / IP-008 secret-file pattern):
      1. ``LIFETRAC_PIN_FILE``  — preferred, file is mounted by Docker.
      2. ``LIFETRAC_PIN``       — fallback, mostly for dev / unit tests.
    Empty / unreadable values fall through to "" so the runtime check in
    ``api_login`` returns 503 (refuse-to-grant) rather than fail-open.
    """
    path = os.environ.get("LIFETRAC_PIN_FILE", "").strip()
    if path:
        try:
            return open(path, "r", encoding="utf-8").read().strip()
        except OSError as exc:
            logging.warning("LIFETRAC_PIN_FILE=%r unreadable: %s", path, exc)
    return os.environ.get("LIFETRAC_PIN", "").strip()


LIFETRAC_PIN = _load_operator_pin()    # 4-6 digits, set at first boot
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
    """Return the best-effort client IP for audit + lockout (IP-206).

    Trusts ``X-Forwarded-For`` only when the immediate peer is in
    ``LIFETRAC_TRUSTED_PROXIES`` (comma-separated CIDRs / exact IPs). When
    not trusted, falls back to the TCP peer address. Trims to the leftmost
    entry of XFF (the original client) per RFC 7239 §5.2 guidance.
    """
    peer = request.client.host if request.client else "unknown"
    trusted = os.environ.get("LIFETRAC_TRUSTED_PROXIES", "").strip()
    if not trusted:
        return peer
    allowed = {t.strip() for t in trusted.split(",") if t.strip()}
    if peer not in allowed:
        return peer
    xff = request.headers.get("x-forwarded-for")
    if not xff:
        return peer
    first = xff.split(",", 1)[0].strip()
    return first or peer


def _check_lockout(ip: str) -> float:
    """Return seconds remaining in lockout, or 0 if not locked."""
    now = time.monotonic()
    with _fail_lock:
        n_fails, locked_until = _fail_counts[ip]
        if locked_until > now:
            return locked_until - now
        return 0.0


# §E (Round 9): module-level singleton AuditLog so PIN failures don't open a
# new file handle per event (which would leak a `ResourceWarning` on every
# rejected login). Lazily constructed so unit-test environments without a
# writable audit dir don't pay for setup until the first event.
_AUDIT_LOG: "object | None" = None
_AUDIT_LOG_LOCK = threading.Lock()


def _get_audit_log():
    global _AUDIT_LOG
    if _AUDIT_LOG is not None:
        return _AUDIT_LOG
    with _AUDIT_LOG_LOCK:
        if _AUDIT_LOG is None:
            try:
                from audit_log import AuditLog, DEFAULT_PATH as _AUDIT_PATH
                _AUDIT_LOG = AuditLog(_AUDIT_PATH)
            except Exception:                          # pragma: no cover
                _AUDIT_LOG = False  # sentinel: tried and failed
    return _AUDIT_LOG if _AUDIT_LOG is not False else None


def _record_failure(ip: str) -> None:
    now = time.monotonic()
    with _fail_lock:
        n_fails, _ = _fail_counts[ip]
        n_fails += 1
        if n_fails >= LOCKOUT_FAILS:
            _fail_counts[ip] = (0, now + LOCKOUT_S)
            logging.warning("web_ui: PIN lockout for %s (%d fails)", ip, n_fails)
            # IP-207: also persist to the audit log so SOC tooling can alert.
            try:
                al = _get_audit_log()
                if al is not None:
                    al.record("pin_lockout", ip=ip, fails=n_fails)
            except Exception:                          # pragma: no cover
                pass
        else:
            _fail_counts[ip] = (n_fails, 0.0)
            logging.info("web_ui: PIN failure for %s (%d/%d)",
                         ip, n_fails, LOCKOUT_FAILS)
            # IP-207: also audit individual failures so brute-force attempts
            # show up in the same stream as crypto rejects.
            try:
                al = _get_audit_log()
                if al is not None:
                    al.record("pin_failure", ip=ip, fails=n_fails)
            except Exception:                          # pragma: no cover
                pass


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


def _connect_mqtt_with_retry() -> None:
    """IP-201: connect to the broker with bounded retries instead of letting
    a transient DNS failure crash the entire web UI at module import time.
    The broker is named ``mosquitto`` inside docker-compose; the container
    may finish coming up after we do.
    """
    host = os.environ.get("LIFETRAC_MQTT_HOST", "localhost")
    deadline = time.monotonic() + 30.0
    backoff = 0.5
    while True:
        try:
            mqtt_client.connect(host, 1883)
            return
        except (OSError, ConnectionError) as exc:
            if time.monotonic() >= deadline:
                logging.error("mqtt: giving up on %s after 30s (%s)", host, exc)
                raise
            logging.warning("mqtt: %s not ready (%s); retry in %.1fs",
                            host, exc, backoff)
            time.sleep(backoff)
            backoff = min(backoff * 2, 5.0)


_connect_mqtt_with_retry()
mqtt_client.loop_start()

# ---- BC-04: load build configuration -----------------------------------
# Module-level singleton consumed below for MAX_CONTROL_SUBSCRIBERS,
# _CAMERA_IDS filtering, and the boot-time audit-log entry. Best-effort:
# if the loader fails (corrupt TOML, missing file in a dev checkout) we
# fall back to canonical defaults so the web UI still boots and the
# operator sees a degraded-but-running console rather than a crash loop.
# Per BC-XX the env var LIFETRAC_UNIT_ID selects the per-unit override.
try:
    import build_config as _bc
    BUILD: "_bc.BuildConfig | None" = _bc.load(os.environ.get("LIFETRAC_UNIT_ID"))
except Exception as _bc_exc:  # pragma: no cover — dev-checkout fallback
    logging.warning("build_config: load failed (%s); using built-in defaults", _bc_exc)
    BUILD = None


def _audit_config_loaded() -> None:
    """Record the active BuildConfig SHA on every web_ui boot (BC-04).

    The SHA is what later rounds (BC-05 admin form, BC-10 delivery) will
    cross-reference against the file on disk to confirm the running
    process matches the file the operator believes is active.
    """
    if BUILD is None:
        return
    al = _get_audit_log()
    if al is None:
        return
    try:
        al.record(
            "config_loaded",
            component="web_ui",
            unit_id=BUILD.unit_id,
            source_path=str(BUILD.source_path),
            config_sha256=BUILD.config_sha256,
            schema_version=BUILD.schema_version,
        )
    except Exception as exc:  # pragma: no cover
        logging.warning("audit: config_loaded record failed: %s", exc)


_audit_config_loaded()

telemetry_subscribers: set[WebSocket] = set()
image_subscribers: set[WebSocket] = set()
state_subscribers: set[WebSocket] = set()
control_subscribers: set[WebSocket] = set()
# §C (Round 9): single lock guarding all WS subscriber-set mutations.
# The MQTT background thread iterates these sets while the FastAPI event
# loop adds/removes on connect/disconnect; without the lock that race
# fires "Set changed size during iteration" intermittently. We snapshot
# under the lock and iterate the snapshot lock-free.
_subscribers_lock = threading.Lock()
active_source = "none"
active_source_lock = threading.Lock()


def _snapshot_subscribers(pool: set[WebSocket]) -> list[WebSocket]:
    """Return a stable snapshot of *pool* under ``_subscribers_lock`` so the
    caller can iterate without racing against connect/disconnect."""
    with _subscribers_lock:
        return list(pool)


# ---- IP-202 / IP-208: bounded WS fan-out -------------------------------
# Caps on concurrent subscribers per topic so a misbehaving (or malicious)
# client can't OOM the base-station by opening thousands of WSes. A new
# connection over the cap is closed with WS code 4429 ("too many
# connections"), modelled after HTTP 429.
MAX_TELEMETRY_SUBSCRIBERS = 8
MAX_IMAGE_SUBSCRIBERS     = 4
MAX_STATE_SUBSCRIBERS     = 4
# §B (Round 9): /ws/control was previously unbounded — flagged independently
# by the second-pass GPT-5.3-Codex and Gemini reviews. Tractor-side replay
# windows already gate by source_id, but at the WS layer an attacker that
# bypassed PIN auth could still open thousands of sockets and starve the
# event loop. Cap matches the realistic operator + observer scenario
# (one driver + a couple of mirrored sessions). Per BC-04 this is now
# driven from the loaded BuildConfig (`ui.max_control_subscribers`) when
# available, with the historical default of 4 as the dev-checkout fallback.
MAX_CONTROL_SUBSCRIBERS   = BUILD.ui.max_control_subscribers if BUILD is not None else 4
WS_OVER_CAPACITY_CODE     = 4429


async def _admit_ws(ws: WebSocket, pool: set[WebSocket], cap: int,
                    label: str) -> bool:
    """Accept *ws* into *pool* if there's room, else close with 4429.

    Returns True iff the caller should proceed with its receive loop.
    Mutation of *pool* is serialised through ``_subscribers_lock`` so the
    MQTT-thread snapshot in ``_snapshot_subscribers`` is race-free.
    """
    with _subscribers_lock:
        if len(pool) >= cap:
            over_cap = True
        else:
            over_cap = False
    if over_cap:
        try:
            await ws.close(code=WS_OVER_CAPACITY_CODE)
        except Exception:
            pass
        logging.warning("ws/%s: rejecting connection, cap=%d reached", label, cap)
        # Audit so a brute-force WS spammer shows up alongside PIN failures.
        try:
            al = _get_audit_log()
            if al is not None:
                al.record("ws_over_cap", label=label, cap=cap)
        except Exception:                                  # pragma: no cover
            pass
        return False
    await ws.accept()
    with _subscribers_lock:
        pool.add(ws)
    return True


def _discard_subscriber(pool: set[WebSocket], ws: WebSocket) -> None:
    """Remove *ws* from *pool* under the shared lock."""
    with _subscribers_lock:
        pool.discard(ws)


def _ws_send_done(label: str):
    """asyncio future callback that surfaces dropped sends in the log
    instead of silently swallowing exceptions (IP-208)."""
    def _cb(fut):
        exc = fut.exception()
        if exc is not None:
            logging.debug("ws/%s send failed: %r", label, exc)
    return _cb


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
            for ws in _snapshot_subscribers(image_subscribers):
                fut = asyncio.run_coroutine_threadsafe(
                    ws.send_bytes(msg.payload), loop_img
                )
                fut.add_done_callback(_ws_send_done("image"))
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
    for ws in _snapshot_subscribers(telemetry_subscribers):
        fut = asyncio.run_coroutine_threadsafe(
            ws.send_text(json.dumps(payload)), loop
        )
        fut.add_done_callback(_ws_send_done("telemetry"))


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
    if loop is None:
        return
    snap_subs = _snapshot_subscribers(state_subscribers)
    if not snap_subs:
        return
    snap = json.dumps(_image_publisher.snapshot())
    for ws in snap_subs:
        fut = asyncio.run_coroutine_threadsafe(ws.send_text(snap), loop)
        fut.add_done_callback(_ws_send_done("state"))


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


# ---- BC-05 (Round 30): build-config admin page -------------------------
@app.get("/config", response_class=HTMLResponse)
async def config_page(session: str | None = Cookie(default=None)):
    """Operator-facing TOML editor + reload-class diff preview.

    Thin client over the BC-05 endpoints below + the BC-10 watcher
    state route. Keeps PIN-gated so only authenticated operators can
    even read the active config (it carries unit_id, MQTT host, etc.).
    """
    if not _session_valid(session):
        return HTMLResponse(status_code=status.HTTP_303_SEE_OTHER,
                            content="", headers={"Location": "/login"})
    return (WEB_DIR / "config.html").read_text(encoding="utf-8")


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
    if not await _admit_ws(ws, telemetry_subscribers,
                           MAX_TELEMETRY_SUBSCRIBERS, "telemetry"):
        return
    try:
        while True:
            await ws.receive_text()  # keepalive pings; we don't care about content
    except WebSocketDisconnect:
        pass
    finally:
        _discard_subscriber(telemetry_subscribers, ws)


@app.websocket("/ws/image")
async def ws_image(ws: WebSocket):
    """Binary frame stream of the reassembled tile-delta canvas.

    Each message is a single complete image blob (WebP today, possibly
    JPEG later) ready for `createImageBitmap` on the browser side.
    Unauthenticated reads are allowed: the canvas is operationally the
    same surface as the dashboard telemetry and FastAPI's cookie Depends
    doesn't run for WebSockets anyway.
    """
    if not await _admit_ws(ws, image_subscribers,
                           MAX_IMAGE_SUBSCRIBERS, "image"):
        return
    try:
        while True:
            await ws.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        _discard_subscriber(image_subscribers, ws)


@app.websocket("/ws/state")
async def ws_state(ws: WebSocket):
    """Authoritative server-side image state (tiles + age + badge + detections).

    Per IMAGE_PIPELINE.md §6.1 the browser MUST consume this rather than
    computing display state locally. JSON snapshots are pushed on every
    canvas update plus a 2 Hz keepalive.
    """
    if not await _admit_ws(ws, state_subscribers,
                           MAX_STATE_SUBSCRIBERS, "state"):
        return
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
        _discard_subscriber(state_subscribers, ws)


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
    # §B (Round 9): bound the operator-control socket the same way as
    # telemetry/image/state so a credentialled-but-misbehaving client
    # cannot open thousands of sockets and starve the event loop.
    if not await _admit_ws(ws, control_subscribers,
                           MAX_CONTROL_SUBSCRIBERS, "control"):
        return
    seq = 0
    hb = 0
    last_tx = 0.0
    try:
        while True:
            raw = await ws.receive_text()
            # IP-203: cap payload size before parsing. ControlMsg fully
            # populated with extreme integers fits in well under 256 bytes.
            if len(raw) > 512:
                continue
            now = asyncio.get_event_loop().time()
            if now - last_tx < 0.05:    # 20 Hz
                continue
            last_tx = now
            if not _base_controls_allowed():
                continue
            try:
                msg_obj = ControlMsg.model_validate_json(raw)
            except ValidationError:
                # Malformed payload — drop the frame but keep the socket
                # so a flaky browser tab doesn't have to re-auth.
                continue
            frame = pack_control(
                seq=seq,
                lhx=msg_obj.lhx,
                lhy=msg_obj.lhy,
                rhx=msg_obj.rhx,
                rhy=msg_obj.rhy,
                buttons=msg_obj.buttons,
                flags=msg_obj.flags,
                hb=hb,
            )
            mqtt_client.publish("lifetrac/v25/cmd/control", frame, qos=0)
            seq = (seq + 1) & 0xFFFF
            hb = (hb + 1) & 0xFF
    except WebSocketDisconnect:
        pass
    finally:
        _discard_subscriber(control_subscribers, ws)


@app.post("/api/estop")
async def estop(_session: str = Depends(_require_session)):
    mqtt_client.publish("lifetrac/v25/cmd/estop", b"", qos=1)
    return {"ok": True}


# Camera selection: maps human-readable name -> CMD_CAMERA_SELECT payload byte.
# See LORA_PROTOCOL.md § Command frame opcodes.
# Per BC-04 the table is filtered against the loaded BuildConfig so a build
# without (e.g.) a rear or implement camera does not advertise that name on
# the API — attempts to select an absent camera return HTTP 400 with
# "unknown camera" rather than silently sending a CMD_CAMERA_SELECT for a
# camera the tractor doesn't have.
_CAMERA_IDS_FULL = {
    "auto":      0x00,
    "front":     0x01,
    "rear":      0x02,
    "implement": 0x03,
    "crop":      0x04,
}


def _filter_cameras(table: dict[str, int]) -> dict[str, int]:
    if BUILD is None or BUILD.cameras.count == 0:
        # No build config or no cameras at all — keep "auto" only when at
        # least one camera might exist; with count == 0 even auto is
        # meaningless, so drop the table entirely.
        return dict(table) if BUILD is None else {}
    out = {"auto": table["auto"]}
    if BUILD.cameras.front_present:
        out["front"] = table["front"]
    if BUILD.cameras.rear_present:
        out["rear"] = table["rear"]
    if BUILD.cameras.implement_present:
        out["implement"] = table["implement"]
    if BUILD.cameras.crop_health_present:
        out["crop"] = table["crop"]
    return out


_CAMERA_IDS = _filter_cameras(_CAMERA_IDS_FULL)


# ---- BC-10 (Round 29b-alpha): installer bundle download ----------------
# PIN-gated. Returns a plain-text installer bundle of the active BUILD as a
# file download. Operator copies the file to a USB stick, walks it to the
# tractor; the X8-side installer (Round 29b-beta) verifies the embedded SHA
# and applies via atomic-rename + quiescence-gate. The route is GET so it's
# linkable from the (future) /config admin page; no body, no side effects
# other than an audit-log entry recording who downloaded what SHA.
@app.get("/config/download")
async def config_download(_session: str = Depends(_require_session)):
    if BUILD is None:
        raise HTTPException(status_code=503, detail="build_config not loaded")
    try:
        import config_bundle as _cb  # local import keeps base_station optional
        body = BUILD.source_path.read_text(encoding="utf-8")
        bundle = _cb.make_bundle(
            body, BUILD.unit_id, generator="lifetrac-base-ui 1.0"
        )
        text = _cb.serialise(bundle)
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"bundle build failed: {exc}") from exc
    al = _get_audit_log()
    if al is not None:
        try:
            al.record(
                "config_download",
                component="web_ui",
                unit_id=bundle.unit_id,
                config_sha256=bundle.sha256,
            )
        except Exception:  # pragma: no cover
            pass
    return PlainTextResponse(
        content=text,
        headers={
            "Content-Disposition": f'attachment; filename="{bundle.filename}"',
            "Cache-Control": "no-store",
        },
    )


# ---- BC-10 (Round 29b-beta): config watch-and-reload state -------------
# Module-level ConfigWatcher, lazily constructed on first /api/build_config
# poll. Base side has no hydraulics, so quiescence is trivially-true
# (parked >> threshold, no subs, queue empty, engine off). The watcher's
# job here is to surface a "restart pending" banner when the X8 installer
# atomically replaces the on-disk TOML under us; the same module is also
# imported by lora_bridge with a real quiescence callback for tractor-side
# live-reload of UI strings + camera-presence flags.
_WATCHER: "_cw.ConfigWatcher | None" = None  # type: ignore[name-defined]


def _base_quiescence():
    import build_config as _bc
    # Base UI = always quiet. Live changes apply on the next poll.
    return _bc.QuiescenceState(
        parked_seconds=999.0,
        active_control_subscribers=0,
        m7_tx_queue_depth=0,
        engine_idle_or_off=True,
    )


def _get_watcher():
    global _WATCHER
    if _WATCHER is not None:
        return _WATCHER
    if BUILD is None:
        return None
    try:
        import config_watcher as _cw
        _WATCHER = _cw.ConfigWatcher(BUILD, get_quiescence=_base_quiescence)
    except Exception as exc:  # pragma: no cover
        logging.warning("config_watcher: init failed: %s", exc)
        return None
    return _WATCHER


@app.get("/api/build_config/state")
async def build_config_state(_session: str = Depends(_require_session)):
    """Surface the running BuildConfig + any pending-reload state.

    Polls the watcher (cheap: one ``stat()`` + a possible reload) so a
    UI banner can refresh without the operator restarting the daemon.
    Returns a small JSON envelope ``{unit_id, sha256, schema_version,
    restart_pending, last_event, last_event_reason, changed_leaves,
    worst_reload_class}``.
    """
    if BUILD is None:
        raise HTTPException(status_code=503, detail="build_config not loaded")
    watcher = _get_watcher()
    if watcher is None:
        raise HTTPException(status_code=503, detail="config_watcher unavailable")
    event = watcher.poll()
    cfg = watcher.current
    payload = {
        "unit_id": cfg.unit_id,
        "schema_version": cfg.schema_version,
        "sha256": cfg.config_sha256,
        "source_path": str(cfg.source_path),
        "restart_pending": watcher.restart_pending,
        "last_event": event.kind,
        "last_event_reason": event.reason,
    }
    diff = event.diff or watcher.restart_pending_diff
    if diff is not None:
        payload["changed_leaves"] = list(diff.changed)
        payload["worst_reload_class"] = diff.worst
    else:
        payload["changed_leaves"] = []
        payload["worst_reload_class"] = None
    al = _get_audit_log()
    if al is not None and event.kind != "noop":
        try:
            al.record(
                "config_watch_event",
                component="web_ui",
                unit_id=cfg.unit_id,
                event=event.kind,
                reason=event.reason,
                worst_reload_class=payload["worst_reload_class"],
            )
        except Exception:  # pragma: no cover
            pass
    return JSONResponse(payload)


# ---- BC-05 (Round 30): build-config admin API --------------------------
# Three sibling routes the /config admin page drives:
#
#   GET  /api/build_config/source       -> raw TOML body (text/plain)
#   POST /api/build_config/preview-diff -> {ok, changed_leaves, classes,
#                                            worst_reload_class} or {ok:false, error}
#   POST /api/build_config/upload       -> writes atomically + audits;
#                                          returns {ok, sha256, worst_reload_class}
#                                          or {ok:false, error}
#
# Upload bytes are validated by the same ``build_config.load`` path the X8
# installer (BC-10 / Round 29b-beta) uses, so a TOML the editor accepts is
# byte-identical to a TOML the installer would accept. Atomic write goes
# via ``installer_daemon._atomic_write`` so a partial write can never leave
# the running daemon's source file half-rewritten -- the watcher then
# picks up the change on its next poll and either applies live, defers
# on quiescence, or sets restart_pending. Firmware-required diffs are
# rejected at upload time (same policy as the X8 installer).


def _load_body_for_validation(body: str):
    """Schema-validate a TOML body string by writing it to a temp file
    and invoking the canonical loader. Returns the BuildConfig.

    Mirrors :func:`installer_daemon._load_body_via_temp` exactly; kept
    inline so web_ui can degrade gracefully when installer_daemon is
    unimportable in dev checkouts (it's stdlib-only, but the import is
    inside a function so a missing module surfaces as 503 not import-time
    crash).
    """
    import build_config as _bc
    import os as _os
    import tempfile as _tf
    with _tf.NamedTemporaryFile("w", suffix=".toml", delete=False, encoding="utf-8") as tf:
        tf.write(body)
        tmp = tf.name
    prev = _os.environ.get("LIFETRAC_BUILD_CONFIG_PATH")
    _os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = tmp
    try:
        return _bc.load()
    finally:
        if prev is None:
            _os.environ.pop("LIFETRAC_BUILD_CONFIG_PATH", None)
        else:
            _os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = prev
        try:
            _os.unlink(tmp)
        except OSError:
            pass


@app.get("/api/build_config/source", response_class=PlainTextResponse)
async def build_config_source(_session: str = Depends(_require_session)):
    """Return the raw TOML body of the active build config.

    The editor seeds itself from this. PIN-gated -- the body carries
    unit_id, MQTT host, and other identifying info.
    """
    if BUILD is None:
        raise HTTPException(status_code=503, detail="build_config not loaded")
    try:
        return PlainTextResponse(
            BUILD.source_path.read_text(encoding="utf-8"),
            headers={"Cache-Control": "no-store"},
        )
    except OSError as exc:
        raise HTTPException(status_code=500, detail=f"read failed: {exc}") from exc


@app.post("/api/build_config/preview-diff")
async def build_config_preview_diff(request: Request,
                                    _session: str = Depends(_require_session)):
    """Schema-validate a candidate body and report the would-be reload class.

    Body is the raw TOML text (Content-Type: text/plain). No side effects
    -- the editor calls this on "Preview diff" so the operator can see
    what's about to land before they hit Save.
    """
    if BUILD is None:
        raise HTTPException(status_code=503, detail="build_config not loaded")
    body_bytes = await request.body()
    body = body_bytes.decode("utf-8", errors="replace")
    try:
        import build_config as _bc
        candidate = _load_body_for_validation(body)
    except Exception as exc:
        return JSONResponse({"ok": False, "error": str(exc)})
    try:
        diff = _bc.diff_reload_classes(BUILD, candidate)
    except Exception as exc:
        return JSONResponse({"ok": False, "error": f"diff: {exc}"})
    return JSONResponse({
        "ok": True,
        "unit_id": candidate.unit_id,
        "schema_version": candidate.schema_version,
        "candidate_sha256": candidate.config_sha256,
        "running_sha256": BUILD.config_sha256,
        "changed_leaves": list(diff.changed),
        "classes": dict(diff.classes),
        "worst_reload_class": diff.worst,
    })


@app.post("/api/build_config/upload")
async def build_config_upload(request: Request,
                              _session: str = Depends(_require_session)):
    """Schema-validate, refuse firmware-required diffs, atomically write.

    On success the running daemon does NOT immediately swap -- the
    :class:`config_watcher.ConfigWatcher` picks up the change on its
    next poll and decides per the BC-10 contract (live -> apply,
    restart_required -> set restart_pending, deferred if not quiet).
    This endpoint just lands the bytes on disk and audits the upload.
    """
    if BUILD is None:
        raise HTTPException(status_code=503, detail="build_config not loaded")
    body_bytes = await request.body()
    body = body_bytes.decode("utf-8", errors="replace")
    try:
        import build_config as _bc
        candidate = _load_body_for_validation(body)
    except Exception as exc:
        return JSONResponse({"ok": False, "error": str(exc)})
    if candidate.unit_id != BUILD.unit_id:
        return JSONResponse({
            "ok": False,
            "error": (
                f"unit_id mismatch: candidate={candidate.unit_id!r} "
                f"running={BUILD.unit_id!r}"
            ),
        })
    diff = _bc.diff_reload_classes(BUILD, candidate)
    if diff.firmware_required:
        return JSONResponse({
            "ok": False,
            "error": (
                "firmware_required change refused via web admin "
                f"(leaves: {', '.join(diff.changed)}); re-flash via bench"
            ),
            "worst_reload_class": "firmware_required",
            "changed_leaves": list(diff.changed),
        })
    # Atomic write -- delegate to installer_daemon for byte-identity
    # with the X8-side path.
    try:
        import installer_daemon as _id
        _id._atomic_write(BUILD.source_path, body)
    except Exception as exc:
        return JSONResponse({"ok": False, "error": f"write failed: {exc}"})
    al = _get_audit_log()
    if al is not None:
        try:
            al.record(
                "config_upload",
                component="web_ui",
                unit_id=candidate.unit_id,
                config_sha256=candidate.config_sha256,
                worst_reload_class=diff.worst,
                changed_leaves=list(diff.changed),
            )
        except Exception:  # pragma: no cover
            pass
    return JSONResponse({
        "ok": True,
        "sha256": candidate.config_sha256,
        "unit_id": candidate.unit_id,
        "schema_version": candidate.schema_version,
        "worst_reload_class": diff.worst,
        "changed_leaves": list(diff.changed),
    })


@app.post("/api/camera/select")
async def camera_select(body: CameraSelectBody,
                        _session: str = Depends(_require_session)):
    """Switch which camera the tractor uses for LoRa thumbnails / WebRTC.

    Body: {"camera": "auto"|"front"|"rear"|"implement"|"crop"}
    Publishes a 1-byte payload to lifetrac/v25/cmd/camera_select; lora_bridge
    wraps it in an FT_COMMAND/CMD_CAMERA_SELECT frame and TXes it over LoRa.
    """
    name = body.camera.lower()
    if name not in _CAMERA_IDS:
        raise HTTPException(status_code=400,
                            detail=f"unknown camera {name!r}")
    cam_id = _CAMERA_IDS[name]
    mqtt_client.publish("lifetrac/v25/cmd/camera_select", bytes([cam_id]), qos=1)
    return {"ok": True, "camera": name, "id": cam_id}


# ---- PIN login routes (MASTER_PLAN.md §8.5) ----------------------------
@app.post("/api/login")
async def api_login(body: LoginBody, request: Request):
    if not LIFETRAC_PIN:
        # No PIN configured — refuse to grant access rather than fail-open.
        raise HTTPException(status_code=503,
                            detail="LIFETRAC_PIN not configured on server")
    ip = _client_ip(request)
    locked = _check_lockout(ip)
    if locked > 0:
        raise HTTPException(status_code=429,
                            detail=f"locked out, retry in {int(locked)}s")
    submitted = body.pin
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
async def api_accel_set(body: AccelToggleBody,
                        _session: str = Depends(_require_session)):
    """Operator master toggle for the optional Coral Edge TPU.

    Body: {"enabled": bool}. Persisted to base_settings.json so the choice
    survives reboot. Takes effect within one inference call (per-call
    `is_active()` check in superres.py / detect.py).
    """
    enabled = body.enabled
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
async def api_params_set(request: Request,
                         _session: str = Depends(_require_session)):
    """Forward a JSON patch to the tractor X8 params_service.

    Body is a partial dict of allow-listed parameter keys. We size-cap the
    raw body (IP-203) and run it through ``ParamsBody`` so neither a huge
    payload nor an unknown key reaches the tractor.
    """
    raw = await request.body()
    if len(raw) > MAX_PARAMS_BODY_BYTES:
        raise HTTPException(status_code=413,
                            detail=f"body exceeds {MAX_PARAMS_BODY_BYTES} bytes")
    try:
        as_dict = json.loads(raw or b"{}")
    except json.JSONDecodeError as exc:
        raise HTTPException(status_code=400, detail=f"invalid JSON: {exc}")
    # Accept either the raw {key: value} dict or a {"params": {...}} wrapper.
    inner = as_dict.get("params", as_dict) if isinstance(as_dict, dict) else None
    if not isinstance(inner, dict):
        raise HTTPException(status_code=400,
                            detail="body must be a JSON object")
    try:
        body = ParamsBody(params=inner)
    except ValidationError as exc:
        # Pydantic's ``errors()`` may include ValueError instances under
        # ``ctx`` which aren't JSON-serialisable; stringify them.
        raise HTTPException(status_code=400, detail=str(exc))
    mqtt_client.publish("lifetrac/v25/params/set",
                        json.dumps(body.params), qos=1)
    return {"ok": True, "submitted": body.params}
