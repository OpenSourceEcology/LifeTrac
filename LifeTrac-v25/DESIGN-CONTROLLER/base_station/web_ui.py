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
import shutil
import subprocess
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


class ChangePinBody(BaseModel):
    model_config = ConfigDict(extra="forbid")
    current_pin: str = Field(..., min_length=1, max_length=16)
    new_pin: str = Field(..., pattern=r"^\d{4,6}$")


class AccelToggleBody(BaseModel):
    model_config = ConfigDict(extra="forbid")
    enabled: bool


# Allowed values for POST /api/settings/encode_mode. This list is explicit
# operator-selected methods only; there is no auto ladder mode in the UI.
# ``wireframe`` remains excluded from the operator UI; it's kept on-wire for
# backward compatibility but not surfaced in the bench controls.
# 2026-07-26: btc4_per_tile / btc4_per_frame removed from the UI — the
# tractor has no BTC4 encoder and silently clamps both to y_only, so the
# menu entries were placebos. Their EncodeMode wire values (4, 5) stay
# reserved in lora_proto for when an encoder lands.
_ENCODE_MODE_UI_CHOICES = (
    "full",
    "y_only",
    "motion_only",
    "mono_g4",
)


class EncodeModeBody(BaseModel):
    model_config = ConfigDict(extra="forbid")
    # Both fields optional so a quality-only change does not have to
    # restate the mode: the settings page holds a mode value it read at
    # page load, and sending that back would silently revert a mode
    # changed since (from the console pill, the gamepad, or another tab).
    # Omitted mode -> keep the current one. At least one is required.
    mode: str | None = Field(None,
              pattern=r"^(full|y_only|motion_only|mono_g4)$")
    # Optional WebP/codec quality (1-100) carried in the same 0x63 command
    # frame as the mode. Omitted -> keep the current quality override.
    quality: int | None = Field(None, ge=1, le=100)

    @field_validator("quality")
    @classmethod
    def _need_one(cls, v, info):
        if v is None and not info.data.get("mode"):
            raise ValueError("provide at least one of 'mode' or 'quality'")
        return v


class RadioProfileBody(BaseModel):
    model_config = ConfigDict(extra="forbid")
    profile: str = Field(..., pattern=r"^(auto|0|1|2)$")


class BenchRadioBody(BaseModel):
    model_config = ConfigDict(extra="forbid")
    mode: str = Field(..., pattern="^(sleep|wake)$")


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
DEFAULT_OPERATOR_PIN = "1234"
# Persistent store for UI-changed PINs. Lives next to the module so a
# pip-installed deployment writes alongside its own files. Overridable via
# env for tests / containerised deployments that mount a writable volume.
PIN_STORE_PATH = Path(
    os.environ.get(
        "LIFETRAC_PIN_STORE",
        str(Path(__file__).parent / ".operator_pin"),
    )
)


def _read_pin_file(path: str | Path) -> str:
    try:
        return Path(path).read_text(encoding="utf-8").strip()
    except OSError as exc:
        logging.warning("PIN file %r unreadable: %s", str(path), exc)
        return ""


def _load_operator_pin() -> str:
    """Load the operator PIN.

    Priority (first non-empty wins):
      1. ``PIN_STORE_PATH`` (user-changed PIN persisted by settings UI).
      2. ``LIFETRAC_PIN_FILE`` env (Docker secret mount; IP-002 / IP-008).
      3. ``LIFETRAC_PIN`` env (dev / unit tests).
      4. ``DEFAULT_OPERATOR_PIN`` ("1234") so first boot is never lockout.
    """
    if PIN_STORE_PATH.exists():
        pin = _read_pin_file(PIN_STORE_PATH)
        if pin:
            return pin
    path = os.environ.get("LIFETRAC_PIN_FILE", "").strip()
    if path:
        pin = _read_pin_file(path)
        if pin:
            return pin
    env_pin = os.environ.get("LIFETRAC_PIN", "").strip()
    if env_pin:
        return env_pin
    return DEFAULT_OPERATOR_PIN


def _persist_operator_pin(new_pin: str) -> None:
    """Write the new PIN to ``PIN_STORE_PATH`` with restrictive perms.

    Raises ``OSError`` on failure; callers translate to HTTP 500.
    """
    PIN_STORE_PATH.parent.mkdir(parents=True, exist_ok=True)
    tmp = PIN_STORE_PATH.with_suffix(PIN_STORE_PATH.suffix + ".tmp")
    tmp.write_text(new_pin + "\n", encoding="utf-8")
    try:
        os.chmod(tmp, 0o600)
    except OSError:
        pass  # Windows / FAT — best-effort
    os.replace(tmp, PIN_STORE_PATH)


LIFETRAC_PIN = _load_operator_pin()    # 4-6 digits, see _load_operator_pin
SESSION_TTL_S = 30 * 60          # 30 min idle timeout
LOCKOUT_FAILS = 5                # wrong PINs before lockout
LOCKOUT_S     = 60               # seconds of cool-off

# ---- Encode-mode override store ----------------------------------------
# Persists the operator's choice from the settings UI so it survives a
# web_ui restart. The bridge picks it up via the retained MQTT topic
# ``lifetrac/v25/control/encode_mode_override``; we re-publish on startup
# below so a bridge that came up *after* web_ui still gets the pin.
ENCODE_MODE_STORE_PATH = Path(
    os.environ.get(
        "LIFETRAC_ENCODE_MODE_STORE",
        str(Path(__file__).parent / ".encode_mode_override"),
    )
)
_ENCODE_MODE_TOPIC = "lifetrac/v25/control/encode_mode_override"
_encode_mode_runtime_lock = threading.Lock()
_encode_mode_runtime_override = "full"
_encode_mode_runtime_quality: int | None = None   # None = tractor default


def _load_encode_mode_state() -> tuple[str, int | None]:
    """Return persisted (mode, quality), defaulting to ("full", None).

    Store format is JSON ``{"mode": ..., "quality": ...}``; a legacy
    bare-mode line (pre-quality store) is still accepted.
    """
    try:
        if ENCODE_MODE_STORE_PATH.exists():
            raw = ENCODE_MODE_STORE_PATH.read_text(encoding="utf-8").strip()
            mode, quality = "full", None
            try:
                body = json.loads(raw)
                if isinstance(body, dict):
                    mode = str(body.get("mode", "full"))
                    q = body.get("quality")
                    if isinstance(q, int) and 1 <= q <= 100:
                        quality = q
            except json.JSONDecodeError:
                mode = raw           # legacy single-line store
            if mode == "auto":
                mode = "full"
            if mode in _ENCODE_MODE_UI_CHOICES:
                return mode, quality
    except OSError as exc:
        logging.warning("encode_mode override file unreadable: %s", exc)
    return "full", None


def _load_encode_mode_override() -> str:
    """Return persisted mode name, defaulting to ``"full"``."""
    return _load_encode_mode_state()[0]


def _persist_encode_mode_override(mode: str, quality: int | None) -> None:
    """Write mode+quality to ``ENCODE_MODE_STORE_PATH`` atomically."""
    ENCODE_MODE_STORE_PATH.parent.mkdir(parents=True, exist_ok=True)
    tmp = ENCODE_MODE_STORE_PATH.with_suffix(
        ENCODE_MODE_STORE_PATH.suffix + ".tmp")
    body: dict[str, Any] = {"mode": mode}
    if quality is not None:
        body["quality"] = quality
    tmp.write_text(json.dumps(body) + "\n", encoding="utf-8")
    os.replace(tmp, ENCODE_MODE_STORE_PATH)


def _get_runtime_encode_mode_override() -> str:
    with _encode_mode_runtime_lock:
        return _encode_mode_runtime_override


def _get_runtime_encode_state() -> tuple[str, int | None]:
    with _encode_mode_runtime_lock:
        return _encode_mode_runtime_override, _encode_mode_runtime_quality


def _set_runtime_encode_mode_override(mode: str,
                                      quality: int | None = None) -> None:
    global _encode_mode_runtime_override, _encode_mode_runtime_quality
    with _encode_mode_runtime_lock:
        _encode_mode_runtime_override = mode
        if quality is not None:
            _encode_mode_runtime_quality = quality


# ---- Radio-profile selector + auto policy (2026-07-25) -----------------
# Operator picks the regulatory/PHY profile for the image link, or "auto".
# Concrete picks publish retained {"profile": N} on the control topic; the
# image daemons apply it live (co-processor reconfig + PHY re-verify) and
# confirm on lifetrac/v25/status/radio_profile/{tx,rx}. "auto" hands the
# choice to AutoRadioPolicy below, driven by the RX daemon's link_stats.
RADIO_PROFILE_STORE_PATH = Path(
    os.environ.get(
        "LIFETRAC_RADIO_PROFILE_STORE",
        str(Path(__file__).parent / ".radio_profile_override"),
    )
)
_RADIO_PROFILE_TOPIC = "lifetrac/v25/control/radio_profile"
_RADIO_PROFILE_UI_CHOICES = ("auto", "0", "1", "2")
_RADIO_PROFILE_LABELS = {
    "0": "Fixed 915 (bench)",
    "1": "FHSS 50ch BW250",
    "2": "DTS BW500",
    "auto": "Auto",
}
_radio_profile_lock = threading.Lock()
_radio_profile_mode = "2"          # UI selection (may be "auto")
_radio_profile_active = 2          # concrete profile last commanded
# True once a radio_profile pin has been observed on the broker (retained
# replay at connect, or any live publish). Distinguishes "auto restarted,
# the link already has a profile" from "nothing on the wire at all".
_radio_profile_pin_seen = False
_radio_acks: dict[str, dict] = {}  # "tx"/"rx" -> last retained ack
_encode_mode_ack: dict | None = None   # tractor camera_service status
# link_monitor / lora_bridge safe-mode override. Cached here because the
# bridge publishes it retained + edge-only: a browser that connects after
# the edge gets nothing on /ws/telemetry, so the console reads it from
# /api/encode_mode/current instead of waiting for the next transition.
_safe_mode_active = False


def _load_radio_profile() -> str:
    try:
        if RADIO_PROFILE_STORE_PATH.exists():
            val = RADIO_PROFILE_STORE_PATH.read_text(encoding="utf-8").strip()
            if val in _RADIO_PROFILE_UI_CHOICES:
                return val
    except OSError as exc:
        logging.warning("radio_profile store unreadable: %s", exc)
    return "2"


def _persist_radio_profile(mode: str) -> None:
    RADIO_PROFILE_STORE_PATH.parent.mkdir(parents=True, exist_ok=True)
    tmp = RADIO_PROFILE_STORE_PATH.with_suffix(
        RADIO_PROFILE_STORE_PATH.suffix + ".tmp")
    tmp.write_text(mode + "\n", encoding="utf-8")
    os.replace(tmp, RADIO_PROFILE_STORE_PATH)


def _publish_radio_profile(profile: int, source: str) -> bool:
    global _radio_profile_active
    # "ts" lets daemon-side consumers age-gate a stale retained pin
    # (run-V incident class: a stale retained profile re-commanded the
    # tractor mid-run). Consumers without the field treat it as live.
    payload = json.dumps({"profile": int(profile), "source": source,
                          "ts": time.time()})
    info = mqtt_client.publish(_RADIO_PROFILE_TOPIC, payload,
                               retain=True, qos=0)
    global _radio_profile_pin_seen
    with _radio_profile_lock:
        _radio_profile_active = int(profile)
        _radio_profile_pin_seen = True
    logging.info("radio_profile: published profile=%d (%s)", profile, source)
    return bool(info and info.rc == 0)


class AutoRadioPolicy:
    """Pick the fastest profile the environment currently supports.

    Inputs (from the RX daemon's ~2 s link_stats samples):
      * sample age   — a stale/absent sample means the link is down or
                       the RX daemon died; treat as unhealthy.
      * timeout rate — reassembler timeouts accumulating means frames
                       are being lost mid-flight (interference / range).

    Policy (v1, bench-calibrated):
      * Prefer DTS BW500 (profile 2) — measured 1.76 KB/s sustained,
        the fastest link — while the link is healthy.
      * Degrade 2 -> 1 (FHSS 50ch) when unhealthy: hopping trades ~60 %
        of the throughput for interference/frequency diversity.
      * Promote 1 -> 2 only after PROMOTE_AFTER_S of continuous health
        AND at least MIN_SWITCH_GAP_S since the last switch (hysteresis
        + min-dwell so a marginal RF environment cannot flap the link,
        which costs a reconfig outage each time).
      * Profile 0 (fixed 915) is bench-only and never auto-selected.

    Pure state machine — no I/O — so the unit suite can drive it with
    synthetic clocks (see tests/test_web_ui_radio_profile.py).
    """

    STALE_LINK_S     = 20.0
    TIMEOUT_RATE_MAX = 2.5     # timeouts per 10 s window
    PROMOTE_AFTER_S  = 60.0
    MIN_SWITCH_GAP_S = 60.0

    def __init__(self, initial_profile: int = 2, now: float = 0.0):
        self.profile = 2 if initial_profile not in (1, 2) else initial_profile
        self._last_switch_t = now
        self._healthy_since: float | None = None

    def evaluate(self, *, now: float, sample_age_s: float | None,
                 timeouts_per_10s: float) -> int | None:
        """Return a new profile to command, or None to hold."""
        healthy = (sample_age_s is not None
                   and sample_age_s <= self.STALE_LINK_S
                   and timeouts_per_10s <= self.TIMEOUT_RATE_MAX)
        if healthy:
            if self._healthy_since is None:
                self._healthy_since = now
        else:
            self._healthy_since = None
        gap_ok = (now - self._last_switch_t) >= self.MIN_SWITCH_GAP_S
        if self.profile == 2 and not healthy and gap_ok:
            self.profile = 1
            self._last_switch_t = now
            return 1
        if (self.profile == 1 and healthy and gap_ok
                and self._healthy_since is not None
                and (now - self._healthy_since) >= self.PROMOTE_AFTER_S):
            self.profile = 2
            self._last_switch_t = now
            return 2
        return None

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

# Re-publish the persisted encode-mode override (retained) so a bridge
# that came up after web_ui still applies the operator's last choice.
# Best-effort: if the broker is still flaky we'll catch up on the next
# POST /api/settings/encode_mode. (Cycle now persists too, so the store
# always matches the last operator action — the boot re-publish can no
# longer roll back a mode cycled after the last settings save.)
try:
    _initial_mode, _initial_quality = _load_encode_mode_state()
    _set_runtime_encode_mode_override(_initial_mode, _initial_quality)
    _startup_payload: dict[str, Any] = {"mode": _initial_mode,
                                        "ts": time.time()}
    if _initial_quality is not None:
        _startup_payload["quality"] = _initial_quality
    mqtt_client.publish(_ENCODE_MODE_TOPIC,
                        json.dumps(_startup_payload),
                        retain=True, qos=0)
    logging.info("encode_mode_override: republished %s (q=%s) on startup",
                 _initial_mode, _initial_quality)
except Exception as exc:  # pragma: no cover — startup-best-effort
    logging.warning("encode_mode_override: startup republish failed: %s", exc)

# Radio-profile startup: restore the persisted selection. Concrete picks
# republish retained (the operator's explicit persisted choice) so daemons
# that came up after web_ui still converge. "auto" must NOT publish at
# boot: the module-default active profile (2) is not what the policy had
# converged to before the restart, and re-pinning it retained is exactly
# the run-V incident class (base re-commanded a stale profile mid-run).
# Instead the auto worker below creates its policy lazily on the first
# tick, seeded from _radio_profile_active — which by then reflects the
# broker's retained pin via the _on_mqtt_message capture.
_radio_auto_policy: AutoRadioPolicy | None = None
try:
    _radio_profile_mode = _load_radio_profile()
    if _radio_profile_mode in ("0", "1", "2"):
        _publish_radio_profile(int(_radio_profile_mode), "startup")
    else:
        logging.info("radio_profile: auto selected; deferring to the "
                     "broker's retained pin (no boot re-publish)")
    logging.info("radio_profile: restored %r on startup", _radio_profile_mode)
except Exception as exc:  # pragma: no cover — startup-best-effort
    logging.warning("radio_profile: startup restore failed: %s", exc)


def _radio_auto_worker() -> None:
    """Drive AutoRadioPolicy off the RX daemon's link_stats samples.

    Evaluates every 5 s; only acts while the UI selection is "auto".
    The timeout RATE is derived from the monotonic `timeouts` counter
    in successive samples (the counter itself never resets mid-run).
    """
    global _radio_auto_policy
    last_timeouts: int | None = None
    last_eval_t = time.monotonic()
    while True:
        time.sleep(5.0)
        try:
            with _radio_profile_lock:
                mode = _radio_profile_mode
                policy = _radio_auto_policy
            if mode != "auto":
                last_timeouts = None
                continue
            if policy is None:
                # Don't seed until the broker session is up, or a slow
                # connect would let this tick beat the retained replay and
                # seed from the module default — the very thing the
                # no-boot-publish change exists to avoid.
                if not mqtt_client.is_connected():
                    logging.info("radio_profile[auto]: broker not connected "
                                 "yet; deferring policy seed")
                    continue
                # Lazily create the policy after boot, seeded from the
                # profile actually on the wire (retained capture in
                # _on_mqtt_message) — never from the module default.
                with _radio_profile_lock:
                    seed = _radio_profile_active
                    seen = _radio_profile_pin_seen
                policy = AutoRadioPolicy(initial_profile=seed,
                                         now=time.monotonic())
                with _radio_profile_lock:
                    _radio_auto_policy = policy
                if seen:
                    logging.info("radio_profile[auto]: policy seeded from "
                                 "retained pin %d (no boot re-pin)", seed)
                else:
                    # Nothing on the wire at all (fresh broker / first
                    # boot). AutoRadioPolicy.evaluate() only publishes on a
                    # TRANSITION, so without this the daemons would sit on
                    # their env default forever while the policy believed
                    # it was at `seed`. Publishing once here is safe: there
                    # is no live pin for it to override, which is exactly
                    # what made the boot re-pin dangerous in the first place.
                    logging.info("radio_profile[auto]: no retained pin found "
                                 "— seeding the link at profile %d", seed)
                    _publish_radio_profile(seed, "auto-seed")
                last_timeouts = None
                continue
            now = time.monotonic()
            stats = getattr(_image_publisher, "link_stats", None) or {}
            ts = stats.get("ts")
            sample_age_s = (time.time() - ts) if isinstance(
                ts, (int, float)) else None
            timeouts = stats.get("timeouts")
            window_s = max(now - last_eval_t, 1e-3)
            if isinstance(timeouts, int) and last_timeouts is not None:
                rate_per_10s = max(0, timeouts - last_timeouts) / window_s * 10.0
            else:
                rate_per_10s = 0.0
            if isinstance(timeouts, int):
                last_timeouts = timeouts
            last_eval_t = now
            new_profile = policy.evaluate(now=now,
                                          sample_age_s=sample_age_s,
                                          timeouts_per_10s=rate_per_10s)
            if new_profile is not None:
                logging.info(
                    "radio_profile[auto]: %s (sample_age=%s, rate=%.1f/10s)",
                    new_profile, sample_age_s, rate_per_10s)
                _publish_radio_profile(new_profile, "auto")
        except Exception:                                     # pragma: no cover
            logging.exception("radio_profile auto worker crashed; continuing")


threading.Thread(target=_radio_auto_worker, name="radio-auto",
                 daemon=True).start()

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
MAX_STATE_SUBSCRIBERS     = 8
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
    try:
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

        # 2026-07-24 link-speed telemetry: image_rx_daemon publishes a
        # rolling B/s + RSSI/SNR JSON sample on /video/link_stats every
        # ~2 s. Forward into the StatePublisher so /ws/state carries it
        # to the image panel (same seam as the S7.1 link_power glue).
        if msg.topic.endswith("/video/link_stats"):
            try:
                stats = json.loads(msg.payload.decode("utf-8"))
                if isinstance(stats, dict):
                    _image_publisher.link_stats = stats
            except Exception:
                pass
            return

        # 2026-07-25 selector acks: tractor camera_service confirms every
        # applied encoder mode (requested vs effective — surfaces silent
        # clamps); each image daemon confirms radio-profile switches.
        # All retained, so web_ui restarts repopulate the caches.
        # Safe-mode state must be CACHED, not just fanned out live: the
        # bridge publishes it retained and only on edges, so a page loaded
        # after safe mode engaged would otherwise never learn about it and
        # would show the operator's encode pin as if it were in force.
        if msg.topic.endswith("/control/safe_mode_active"):
            global _safe_mode_active
            try:
                body = json.loads(msg.payload.decode("utf-8"))
                _safe_mode_active = bool(body.get("active")) \
                    if isinstance(body, dict) else bool(body)
            except Exception:
                pass
            # fall through: live subscribers still get the telemetry frame
        if msg.topic.endswith("/status/encode_mode"):
            try:
                body = json.loads(msg.payload.decode("utf-8"))
                if isinstance(body, dict):
                    global _encode_mode_ack
                    _encode_mode_ack = body
            except Exception:
                pass
            return
        if "/status/radio_profile/" in msg.topic:
            side = msg.topic.rsplit("/", 1)[-1]
            if side in ("tx", "rx"):
                try:
                    body = json.loads(msg.payload.decode("utf-8"))
                    if isinstance(body, dict):
                        with _radio_profile_lock:
                            _radio_acks[side] = body
                except Exception:
                    pass
            return

        # Bench-only compatibility path: some legacy bring-up scripts publish
        # tile payloads on /cmd/image_frame. Keep this disabled by default so a
        # local synthetic publisher cannot override the authoritative radio path.
        allow_cmd_image = os.environ.get("LIFETRAC_ALLOW_CMD_IMAGE_FRAME", "").strip() == "1"
        if msg.topic.endswith("/video/tile_delta") or (
            allow_cmd_image and msg.topic.endswith("/cmd/image_frame")
        ):
            _ingest_tile_delta(bytes(msg.payload))
            return

        data = _decode_payload(msg.topic, msg.payload)
        # Track the commanded radio profile actually on the wire (retained
        # replay at connect + live publishes, including our own). Keeps
        # _radio_profile_active truthful across restarts so the lazy auto
        # policy seeds from reality, not the module default.
        if msg.topic.endswith("/control/radio_profile") and isinstance(data, dict):
            prof = data.get("profile")
            if isinstance(prof, int) and prof in (0, 1, 2):
                global _radio_profile_active, _radio_profile_pin_seen
                with _radio_profile_lock:
                    _radio_profile_active = prof
                    _radio_profile_pin_seen = True
        if msg.topic.endswith("/source_active"):
            candidate = None
            if isinstance(data, dict):
                candidate = _source_name(data.get("active_source"))
            else:
                candidate = _source_name(data)
            if candidate is not None:
                _set_active_source(candidate)
        maybe_capture = globals().get("_maybe_capture_params")
        if callable(maybe_capture):
            maybe_capture(msg.topic, data)

        # S7.1 LINK-pill cross-process glue: forward the per-direction
        # adapter snapshot published by lora_bridge._airtime_worker on
        # `lifetrac/v25/control/link_power/{direction}` into the
        # StatePublisher so /ws/state carries it to map.js. Bridge and
        # web_ui are separate processes (they share only the broker), so
        # this MQTT subscription is the seam — there is no in-process
        # object handoff. Refusal-on-bad-payload: silently drop anything
        # that isn't a dict on a known direction, per §6.1.
        if msg.topic.startswith("lifetrac/v25/control/link_power/"):
            direction = msg.topic.rsplit("/", 1)[-1]
            if direction in ("uplink", "downlink") and isinstance(data, dict):
                try:
                    _image_publisher.link_power[direction] = data
                except Exception:
                    pass

        payload = {"topic": msg.topic, "data": data}
        loop = getattr(app.state, "loop", None)
        if loop is None:
            return
        for ws in _snapshot_subscribers(telemetry_subscribers):
            fut = asyncio.run_coroutine_threadsafe(
                ws.send_text(json.dumps(payload)), loop
            )
            fut.add_done_callback(_ws_send_done("telemetry"))
    except Exception:
        # Keep the Paho loop thread alive even if one payload is malformed.
        logging.exception("web_ui: mqtt callback crashed for topic %s", getattr(msg, "topic", "?"))


def _subscribe_mqtt_topics() -> None:
    mqtt_client.subscribe("lifetrac/v25/telemetry/#")
    mqtt_client.subscribe("lifetrac/v25/status/#")
    mqtt_client.subscribe("lifetrac/v25/video/#")
    mqtt_client.subscribe("lifetrac/v25/control/#")
    if os.environ.get("LIFETRAC_ALLOW_CMD_IMAGE_FRAME", "").strip() == "1":
        mqtt_client.subscribe("lifetrac/v25/cmd/image_frame")
    mqtt_client.subscribe("lifetrac/v25/params/changed")


def _on_mqtt_connect(_c, _u, _flags, rc):
    if rc == 0:
        _subscribe_mqtt_topics()
    else:
        logging.warning("mqtt: connect callback rc=%s", rc)


mqtt_client.on_message = _on_mqtt_message
mqtt_client.on_connect = _on_mqtt_connect
_subscribe_mqtt_topics()


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

# F10 (2026-08-01): receiver-driven stale-tile reporting. The canvas is
# PERSISTENT — a tile whose update is lost keeps showing the old blob until
# the encoder happens to resend it, and the encoder cannot know (its age
# escalation is encoder-side). The base is the only party that knows what
# arrived, so it reports staleness and the tractor treats the bitmap as
# advisory dirty marks. Semantics note: with tile-delta on a static scene
# the receiver cannot distinguish "static, nothing sent" from "sent and
# lost" — what makes age evidence of loss is the encoder's fair-share
# SWEEP, which re-ships every tile within its rotation period on a healthy
# link. STALE_AFTER_MS must therefore exceed the full sweep rotation
# (n_tiles / LIFETRAC_SWEEP_STEP frames at the current fps); marks are
# advisory, so a false positive merely re-ships one tile early.
_TILE_STALE_ENABLE = os.environ.get("LIFETRAC_TILE_STALE_ENABLE", "1") == "1"
_TILE_STALE_AFTER_MS = int(os.environ.get("LIFETRAC_TILE_STALE_AFTER_MS",
                                          "20000"))
_TILE_STALE_PERIOD_S = float(os.environ.get("LIFETRAC_TILE_STALE_PERIOD_S",
                                            "3.0"))
TILE_STALE_TOPIC = "lifetrac/v25/cmd/tile_stale"

# F11 (2026-08-01, from the F10 acceptance §5 observation): the gap-tolerant
# canvas requests a keyframe on EVERY base_seq gap — including a single lost
# delta frame — and each granted request costs a multi-frame keyframe train
# at the current link budget. F10's stale-tile reporting now detects and
# repairs exactly that damage tile-by-tile at ~zero extra airtime, so the
# per-gap keyframe is largely redundant. Gate it, DEFAULT UNCHANGED (on),
# per the F10 protocol: measure the A/B on air first, then flip in a
# separate commit. Cold start, grid mismatch, and tile-decode errors are
# NOT gated — those are the keyframe request's legitimate jobs.
_KF_ON_SEQ_GAP = os.environ.get("LIFETRAC_KF_ON_SEQ_GAP", "1") == "1"


def compute_stale_tiles(canvas, now_ms: int, stale_after_ms: int) -> list:
    """Pure: indices of tiles that have not refreshed within the horizon.

    Only meaningful once a keyframe has populated the canvas; a tile with
    arrived_ms == 0 after that point never arrived at all and counts as
    stale too.
    """
    if not canvas.has_keyframe:
        return []
    out = []
    for idx, tile in enumerate(canvas._tiles):
        arrived = tile.arrived_ms
        if arrived == 0 or (now_ms - arrived) > stale_after_ms:
            out.append(idx)
    return out


def _tile_stale_worker() -> None:
    from lora_proto import pack_tile_stale
    while True:
        time.sleep(_TILE_STALE_PERIOD_S)
        try:
            with _image_lock:
                canvas = _image_canvas
                now_ms = int(time.monotonic() * 1000)
                stale = compute_stale_tiles(canvas, now_ms,
                                            _TILE_STALE_AFTER_MS)
                n_tiles = canvas.n_tiles
                base_seq = getattr(canvas, "_last_base_seq", None) or 0
            if not stale:
                continue
            body = pack_tile_stale(base_seq, stale, n_tiles)
            mqtt_client.publish(TILE_STALE_TOPIC, body, qos=0)
        except Exception:
            # Advisory path: never let a transient error kill the worker.
            pass


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
    global _image_canvas
    with _image_lock:
        frame = _image_reassembler.feed(payload)
        if frame is None:
            return
        # 2026-05-27: auto-adopt the upstream camera's grid on the first
        # keyframe whose layout differs from our current Canvas. Before
        # this, _image_canvas was hardcoded 12x8@32px and any camera with
        # a different grid (e.g. the LoRa-bridge synthetic 6x4@32px) was
        # rejected by Canvas.apply() with "grid mismatch", leaving every
        # /ws/state snapshot with tiles=[] and the operator canvas black.
        if frame.is_keyframe and (
            (frame.grid_w, frame.grid_h, frame.tile_px)
            != (_image_canvas.grid_w, _image_canvas.grid_h, _image_canvas.tile_px)
        ):
            _image_canvas = Canvas(
                grid_w=frame.grid_w,
                grid_h=frame.grid_h,
                tile_px=frame.tile_px,
            )
            _image_publisher.canvas = _image_canvas
        update = _image_canvas.apply(frame)
        # F11: a request caused ONLY by a base_seq gap is suppressed when
        # the gate is off — the gap's tiles were still applied (gap-tolerant
        # merge) and whatever the lost frames staled is repaired by the 0x6C
        # stale-tile path. Any co-occurring tile error un-suppresses.
        suppress_gap_kf = (update.request_keyframe and update.seq_gap
                           and not update.tile_error and not _KF_ON_SEQ_GAP)
        if suppress_gap_kf:
            logging.info("kf-on-gap suppressed (F11): %s", update.reason)
        if update.request_keyframe and not suppress_gap_kf:
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
    if _TILE_STALE_ENABLE:
        threading.Thread(target=_tile_stale_worker,
                         name="tile-stale", daemon=True).start()
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


MAX_AUDIT_EVENT_BODY_BYTES = 2048   # cap for the two audit-intent POSTs below


async def _bounded_json_body(request: Request) -> dict:
    """Read a small JSON dict body; anything oversized/non-dict becomes {}."""
    try:
        raw = await request.body()
    except Exception:
        return {}
    if len(raw) > MAX_AUDIT_EVENT_BODY_BYTES:
        return {"truncated": True, "size": len(raw)}
    try:
        body = json.loads(raw or b"{}")
    except Exception:
        return {}
    return body if isinstance(body, dict) else {}


@app.post("/api/health/refusal")
async def api_health_refusal(request: Request,
                             _session: str = Depends(_require_session)):
    """Browser badge_renderer.js posts here when it refuses a tile.

    We append to the audit log via the same channel as MQTT events so
    refusal events become part of the black-box record. Session-gated and
    size-capped: this used to relay arbitrary unauthenticated client JSON
    onto the control/# namespace.
    """
    body = await _bounded_json_body(request)
    try:
        mqtt_client.publish("lifetrac/v25/control/badge_refusal",
                            json.dumps(body).encode(), qos=1)
    except Exception:
        pass
    return {"ok": True}


@app.post("/api/audit/view_mode")
async def api_audit_view_mode(request: Request,
                              _session: str = Depends(_require_session)):
    """Browser raw_mode_toggle.js posts here whenever the operator flips
    raw mode. Logged to the black-box audit trail. Session-gated and
    size-capped like /api/health/refusal."""
    body = await _bounded_json_body(request)
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
            try:
                raw = await asyncio.wait_for(ws.receive_text(), timeout=0.15)
            except asyncio.TimeoutError:
                # Deadman threshold exceeded (150 ms)
                if _base_controls_allowed():
                    logging.warning("web_ui: Control frame timeout. Halting vehicle.")
                    frame = pack_control(
                        seq=seq,
                        lhx=0,
                        lhy=0,
                        rhx=0,
                        rhy=0,
                        buttons=0,
                        flags=0,
                        hb=hb,
                    )
                    mqtt_client.publish("lifetrac/v25/cmd/control", frame, qos=0)
                    seq = (seq + 1) & 0xFFFF
                    hb = (hb + 1) & 0xFF
                continue
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
    except Exception as e:
        logging.error("web_ui: websocket error in ws_control: %s", e)
    finally:
        if _base_controls_allowed():
            logging.info("web_ui: Control websocket closed. Halting vehicle.")
            try:
                frame = pack_control(
                    seq=seq,
                    lhx=0,
                    lhy=0,
                    rhx=0,
                    rhy=0,
                    buttons=0,
                    flags=0,
                    hb=hb,
                )
                mqtt_client.publish("lifetrac/v25/cmd/control", frame, qos=0)
            except Exception:
                pass
        _discard_subscriber(control_subscribers, ws)


@app.post("/api/estop")
async def estop(_session: str = Depends(_require_session)):
    """Latch the tractor E-stop via MQTT -> lora_bridge -> LoRa.

    Reports what it can actually prove: whether the qos-1 publish was
    confirmed by the LOCAL broker. Radio delivery has no ack path, so
    ``delivered`` here means "handed to the bridge plane", not "tractor
    stopped" — the frontend words its banner accordingly. Previously this
    returned ``{"ok": true}`` unconditionally, even with MQTT down.
    """
    if not mqtt_client.is_connected():
        return JSONResponse(
            {"ok": False, "delivered": False,
             "detail": "MQTT broker disconnected"},
            status_code=503,
        )
    info = mqtt_client.publish("lifetrac/v25/cmd/estop", b"", qos=1)
    delivered = bool(info and info.rc == mqtt.MQTT_ERR_SUCCESS)
    if delivered and not info.is_published():
        def _wait_confirm() -> bool:
            """True if the broker confirmed; False if it definitely didn't.

            Returns True on paho builds too old to support a bounded wait:
            blocking an E-stop response on an unbounded wait would be
            worse than reporting the enqueue, and reporting FAILED for a
            publish that probably succeeded would train operators to
            ignore the banner.
            """
            try:
                info.wait_for_publish(timeout=1.0)
            except TypeError:      # paho < 1.6 has no timeout kwarg
                return True
            except Exception:
                return False
            return info.is_published()
        delivered = await asyncio.to_thread(_wait_confirm)
    al = _get_audit_log()
    if al is not None:
        try:
            al.record("estop_pressed", component="web_ui",
                      delivered=delivered)
        except Exception:  # pragma: no cover
            pass
    detail = None if delivered else "MQTT publish not confirmed by broker"
    return {"ok": delivered, "delivered": delivered, "detail": detail}


def _run_bench_radio_script(mode: str) -> dict[str, Any]:
    helper_dir = (Path(__file__).resolve().parent / ".." /
                  "firmware" / "x8_lora_bootloader_helper").resolve()
    use_powershell = shutil.which("powershell") is not None
    if use_powershell:
        script_name = "run_radio_sleep.ps1" if mode == "sleep" else "run_radio_wake_rxcont.ps1"
        cmd = [
            "powershell",
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
        ]
    else:
        script_name = "radio_sleep.py" if mode == "sleep" else "w2_02_radio_wake_rxcont.py"
        py = shutil.which("python3") or shutil.which("python")
        if py is None:
            return {
                "ok": False,
                "mode": mode,
                "error": "bench radio helper unavailable: python3/python not found",
                "returncode": -1,
                "output_tail": [],
            }
        cmd = [py]
    script_path = helper_dir / script_name
    if not script_path.exists():
        return {
            "ok": False,
            "mode": mode,
            "error": f"helper script missing: {script_path}",
            "returncode": -1,
            "output_tail": [],
        }
    cmd.append(str(script_path))
    try:
        proc = subprocess.run(
            cmd,
            text=True,
            capture_output=True,
            timeout=180,
            cwd=str(helper_dir),
            check=False,
        )
        merged = ((proc.stdout or "") + "\n" + (proc.stderr or "")).strip()
        lines = [ln for ln in merged.splitlines() if ln.strip()]
        return {
            "ok": proc.returncode == 0,
            "mode": mode,
            "returncode": proc.returncode,
            "output_tail": lines[-40:],
        }
    except subprocess.TimeoutExpired:
        return {
            "ok": False,
            "mode": mode,
            "returncode": 124,
            "error": "bench radio helper timed out",
            "output_tail": [],
        }
    except OSError as exc:
        return {
            "ok": False,
            "mode": mode,
            "returncode": -1,
            "error": f"bench radio helper launch failed: {exc}",
            "output_tail": [],
        }


# One bench-radio helper at a time (they share the physical L072).
_bench_radio_busy = asyncio.Lock()


@app.post("/api/bench/radio")
async def api_bench_radio(body: BenchRadioBody,
                          _session: str = Depends(_require_session)):
    # Bench-only guard so production deployments can hard-disable this switch.
    flag = os.environ.get("LIFETRAC_ENABLE_BENCH_RADIO", "1").strip().lower()
    if flag not in {"1", "true", "yes", "on"}:
        raise HTTPException(status_code=403, detail="bench radio control disabled")

    # The helper shells out to adb/serial probes and can legitimately take
    # minutes; run it off the event loop so the whole UI (including
    # /api/estop) stays responsive while it grinds. Single-flight: the
    # helpers drive the SAME L072 over the same serial port, and moving
    # off the loop removed the accidental serialization that used to
    # prevent two concurrent probes from fighting over the modem.
    if _bench_radio_busy.locked():
        raise HTTPException(status_code=409,
                            detail="bench radio helper already running")
    async with _bench_radio_busy:
        result = await asyncio.to_thread(_run_bench_radio_script, body.mode)
    try:
        mqtt_client.publish(
            "lifetrac/v25/control/bench_radio",
            json.dumps({
                "mode": body.mode,
                "ok": bool(result.get("ok")),
                "returncode": int(result.get("returncode", -1)),
            }).encode(),
            qos=1,
        )
    except Exception:
        pass
    code = 200 if result.get("ok") else 500
    return JSONResponse(result, status_code=code)


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


@app.post("/api/settings/pin")
async def api_change_pin(body: ChangePinBody,
                         _session: str = Depends(_require_session)):
    """Operator-changeable PIN. Requires current PIN + valid session.

    New PIN must be 4-6 digits. Persisted to ``PIN_STORE_PATH`` so it
    survives restart; reload happens in-process so the next login uses
    the new value immediately. All other sessions are invalidated.
    """
    global LIFETRAC_PIN
    if not hmac.compare_digest(body.current_pin, LIFETRAC_PIN):
        raise HTTPException(status_code=401, detail="invalid current PIN")
    try:
        _persist_operator_pin(body.new_pin)
    except OSError as exc:
        logging.error("failed to persist PIN to %s: %s", PIN_STORE_PATH, exc)
        raise HTTPException(status_code=500,
                            detail=f"could not persist PIN: {exc}")
    LIFETRAC_PIN = body.new_pin
    # Invalidate every session except the caller's so other devices must
    # re-authenticate with the new PIN.
    with _sessions_lock:
        keep = _sessions.get(_session)
        _sessions.clear()
        if keep is not None:
            _sessions[_session] = keep
    return {"ok": True}


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


# ---- Encode-mode selection (manual operator control) ---------------------
# The selected mode is applied directly by lora_bridge and remains active
# until the operator selects a new mode.
@app.get("/api/settings/encode_mode")
async def api_encode_mode_get(_session: str = Depends(_require_session)):
    """Return the persisted operator-selected mode + available choices.
    the UI is allowed to surface.
    """
    mode, quality = _load_encode_mode_state()
    return {
        "current": mode,
        "quality": quality,
        "choices": list(_ENCODE_MODE_UI_CHOICES),
    }


@app.get("/api/encode_mode/current")
async def api_encode_mode_current(_session: str = Depends(_require_session)):
    """Return the active runtime override used by pill/gamepad cycling.

    Also carries the confirmation loop (2026-07-25): ``tractor`` is the
    retained ack from camera_service (requested vs effective — non-null
    ``clamped`` means the tractor demoted an unimplemented codec), and
    ``rx_codec_name`` is the codec of the most recently REASSEMBLED
    frame (the implicit over-the-air ack — mode changes force a
    keyframe, so this converges within one frame of a real switch).
    """
    stats = getattr(_image_publisher, "link_stats", None) or {}
    mode, quality = _get_runtime_encode_state()
    return {
        "current": mode,
        "quality": quality,
        "choices": list(_ENCODE_MODE_UI_CHOICES),
        "tractor": _encode_mode_ack,
        "rx_codec": stats.get("rx_codec"),
        "rx_codec_name": stats.get("rx_codec_name"),
        "safe_mode": _safe_mode_active,
    }


@app.post("/api/settings/encode_mode")
async def api_encode_mode_set(body: EncodeModeBody,
                              _session: str = Depends(_require_session)):
    """Set the operator-selected encode mode.

    Persists to disk so the choice survives a web_ui restart, then
    publishes a retained JSON message on ``lifetrac/v25/control/
    encode_mode_override`` so the bridge applies it on the next tick.
    The bridge handler (`lora_bridge._on_mqtt_message`) applies the
    selected mode immediately.
    """
    mode = body.mode or _get_runtime_encode_mode_override()
    return _set_encode_mode_override(mode, persist=True,
                                     quality=body.quality)


# Subset of `_ENCODE_MODE_UI_CHOICES` that the gamepad / pill cycle button
# advances through. Order matches the on-screen pill rotation.
_ENCODE_MODE_CYCLE_ORDER: tuple[str, ...] = (
    "full",
    "y_only",
    "motion_only",
    "mono_g4",
)


def _set_encode_mode_override(mode: str, *, persist: bool,
                              quality: int | None = None) -> dict:
    """Set + publish an operator-selected encode mode (+ optional quality).

    Shared by ``POST /api/settings/encode_mode`` and
    ``POST /api/encode_mode/cycle``. ``quality=None`` keeps the current
    quality override. Returns the JSON body the endpoints surface.
    """
    if mode not in _ENCODE_MODE_UI_CHOICES:
        raise HTTPException(status_code=400,
                            detail=f"unknown encode mode: {mode!r}")
    # Persist BEFORE mutating runtime state: an OSError here must leave
    # the runtime override, the store, and the wire all agreeing on the
    # old value rather than stranding a change only this process knows.
    eff_quality = quality if quality is not None else _get_runtime_encode_state()[1]
    eff_mode = mode
    if persist:
        try:
            _persist_encode_mode_override(eff_mode, eff_quality)
        except OSError as exc:
            logging.error("failed to persist encode_mode override to %s: %s",
                          ENCODE_MODE_STORE_PATH, exc)
            raise HTTPException(status_code=500,
                                detail=f"could not persist override: {exc}")
    _set_runtime_encode_mode_override(eff_mode, eff_quality)
    body: dict[str, Any] = {"mode": eff_mode, "ts": time.time()}
    if eff_quality is not None:
        body["quality"] = eff_quality
    info = mqtt_client.publish(_ENCODE_MODE_TOPIC, json.dumps(body),
                               retain=True, qos=0)
    delivered = bool(info and info.rc == 0)
    return {
        "ok": True,
        "mode": eff_mode,
        "quality": eff_quality,
        "delivered": delivered,   # local broker enqueue only — the real
        "persisted": persist,     # confirmation is the tractor 0x68 ack
    }


@app.post("/api/encode_mode/cycle")
async def api_encode_mode_cycle(_session: str = Depends(_require_session)):
    """Advance the operator-selected encode mode through the cycle order.

    Reads the current runtime override, rotates to the next entry in
    :data:`_ENCODE_MODE_CYCLE_ORDER`, publishes AND persists it (2026-07-26:
    cycling used to skip persistence, so a restart re-pinned the stale
    settings value over the last cycled mode). Wired to the gamepad
    BACK/SELECT button (idx 8) and the web encoder-mode pill.
    """
    current = _get_runtime_encode_mode_override()
    if current in _ENCODE_MODE_CYCLE_ORDER:
        i = _ENCODE_MODE_CYCLE_ORDER.index(current)
        nxt = _ENCODE_MODE_CYCLE_ORDER[(i + 1) % len(_ENCODE_MODE_CYCLE_ORDER)]
    else:
        # Persisted/runtime value is outside the cycle list; snap back
        # to the head of the cycle on the next press.
        nxt = _ENCODE_MODE_CYCLE_ORDER[0]
    result = _set_encode_mode_override(nxt, persist=True)
    result["previous"] = current
    return result


# ---- Radio-profile selector endpoints (2026-07-25) ----------------------
@app.get("/api/settings/radio_profile")
async def api_radio_profile_get(_session: str = Depends(_require_session)):
    """Return the radio-profile selection + both daemons' retained acks.

    ``acks.tx`` / ``acks.rx`` carry the last CONFIRMED profile per side;
    a side whose ack disagrees with ``active_profile`` is mid-switch,
    unreachable, or failed-and-reverted (``ok: false``) — the settings
    UI renders each state distinctly.
    """
    with _radio_profile_lock:
        mode = _radio_profile_mode
        active = _radio_profile_active
        acks = {k: dict(v) for k, v in _radio_acks.items()}
    stats = getattr(_image_publisher, "link_stats", None) or {}
    ts = stats.get("ts")
    age = round(time.time() - ts, 1) if isinstance(ts, (int, float)) else None
    return {
        "current": mode,
        "choices": list(_RADIO_PROFILE_UI_CHOICES),
        "labels": dict(_RADIO_PROFILE_LABELS),
        "active_profile": active,
        "acks": acks,
        "link_stats_age_s": age,
    }


@app.post("/api/settings/radio_profile")
async def api_radio_profile_set(body: RadioProfileBody,
                                _session: str = Depends(_require_session)):
    """Select the image-link radio profile (or hand it to auto policy).

    Concrete picks publish retained ``{"profile": N}`` on
    ``lifetrac/v25/control/radio_profile``; both image daemons apply the
    switch live and ack on ``status/radio_profile/{tx,rx}``. ``auto``
    starts :class:`AutoRadioPolicy` seeded from the last commanded
    profile — it will only ever choose FHSS (1) or DTS (2).
    """
    global _radio_profile_mode, _radio_auto_policy
    try:
        _persist_radio_profile(body.profile)
    except OSError as exc:
        logging.error("failed to persist radio_profile to %s: %s",
                      RADIO_PROFILE_STORE_PATH, exc)
        raise HTTPException(status_code=500,
                            detail=f"could not persist selection: {exc}")
    with _radio_profile_lock:
        _radio_profile_mode = body.profile
    delivered = True
    if body.profile in ("0", "1", "2"):
        with _radio_profile_lock:
            _radio_auto_policy = None
        delivered = _publish_radio_profile(int(body.profile), "operator")
    else:
        with _radio_profile_lock:
            seed = _radio_profile_active
        policy = AutoRadioPolicy(initial_profile=seed, now=time.monotonic())
        with _radio_profile_lock:
            _radio_auto_policy = policy
        # Immediately command the policy's starting profile so "auto"
        # has a defined state even before the first evaluation tick.
        delivered = _publish_radio_profile(policy.profile, "auto-start")
    return {
        "ok": True,
        "current": body.profile,
        "active_profile": _radio_profile_active,
        "delivered": delivered,
    }


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


# ---- Autonomy & Route Planning (BC-06, Round 30) ---------------------
class GeofencePlanRequest(BaseModel):
    """Pydantic model for calculating a Boustrophedon sweep route.
    
    Accepts polygon vertices as standard lat/lon tuples, and optional swath width.
    """
    polygon: list[tuple[float, float]]
    swath_width_m: float = Field(default=2.5, gt=0.5, lt=30.0)

@app.post("/api/autonomy/calculate-sweep")
async def api_calculate_sweep(body: GeofencePlanRequest,
                              _session: str = Depends(_require_session)):
    """Calculate and return Boustrophedon sweep waypoints from custom geofence."""
    import planner
    try:
        waypoints = planner.generate_sweep_coverage(body.polygon, body.swath_width_m)
        return {
            "ok": True,
            "swath_width_m": body.swath_width_m,
            "waypoints": [wp.to_dict() for wp in waypoints]
        }
    except Exception as exc:
        raise HTTPException(status_code=400, detail=str(exc))

