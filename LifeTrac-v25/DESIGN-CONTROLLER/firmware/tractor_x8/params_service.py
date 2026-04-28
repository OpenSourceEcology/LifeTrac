#!/usr/bin/env python3
"""
LifeTrac v25 — tractor X8 parameter service.

Owns the persistent, operator-tunable parameters the tractor needs across
reboots: hydraulic flow setpoint defaults, joystick deadbands, valve timing
calibration, encoder ratios, and the AES key ID (the key itself stays in
the firmware build — see KEY_ROTATION.md).

Per MASTER_PLAN.md §8.10 (X8 Linux sidecar role).

Public surface:
  * Library API: load() / save() / update() — used by other X8 services
    and unit tests.
  * Optional FastAPI sub-app `params_app` (built only when fastapi is
    importable) for the operator console proxy.
  * Optional MQTT publish on every successful update so the M7 firmware
    can reload via the X8↔H747 UART (topic `lifetrac/v25/params/changed`,
    retained QoS 1).

Storage:
  * JSON file at /var/lib/lifetrac/params.json. Atomic via tmp+replace
    so a crashed write cannot corrupt the file.
  * `LIFETRAC_PARAMS_PATH` env var overrides for tests.

Schema (see DEFAULT_PARAMS) is intentionally narrow for v25; new keys
should be added with a default so old config files remain valid.
"""

from __future__ import annotations

import json
import logging
import os
import threading
from pathlib import Path
from typing import Any

LOG = logging.getLogger("params_service")

DEFAULT_PARAMS: dict[str, dict[str, Any]] = {
    "hydraulic": {
        "flow_setpoint_default_pct": 60,
        "valve_min_on_ms": 25,
    },
    "joystick": {
        "deadband_pct": 6,
        "expo_pct": 25,
    },
    "link": {
        "control_phy": "SF7_BW250",         # DECISIONS.md D-A2
        "image_phy":   "SF7_BW500",         # DECISIONS.md D-A3
    },
    "key": {
        "fleet_key_id": 0,                   # see KEY_ROTATION.md
    },
}

PARAMS_PATH = Path(os.environ.get("LIFETRAC_PARAMS_PATH",
                                  "/var/lib/lifetrac/params.json"))
MQTT_TOPIC  = "lifetrac/v25/params/changed"

_lock = threading.Lock()
_mqtt_client = None      # lazily set in main()


# ---- core load/save ----------------------------------------------------

def _deep_merge(base: dict, override: dict) -> dict:
    """Recursively overlay `override` onto `base`, preferring override values
    while keeping defaults for keys override does not mention. Pure function;
    returns a new dict."""
    out: dict[str, Any] = {k: dict(v) if isinstance(v, dict) else v
                           for k, v in base.items()}
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load() -> dict:
    """Return the on-disk params merged on top of DEFAULT_PARAMS so missing
    keys come up with their default values."""
    with _lock:
        if PARAMS_PATH.exists():
            try:
                disk = json.loads(PARAMS_PATH.read_text())
                if not isinstance(disk, dict):
                    raise ValueError("params.json: top level must be a dict")
                return _deep_merge(DEFAULT_PARAMS, disk)
            except (OSError, json.JSONDecodeError, ValueError) as exc:
                LOG.warning("params: load failed (%s) — using defaults", exc)
        return _deep_merge(DEFAULT_PARAMS, {})


def save(params: dict) -> None:
    """Atomic write to params.json + best-effort MQTT publish."""
    with _lock:
        try:
            PARAMS_PATH.parent.mkdir(parents=True, exist_ok=True)
            tmp = PARAMS_PATH.with_suffix(".json.tmp")
            tmp.write_text(json.dumps(params, indent=2, sort_keys=True))
            tmp.replace(PARAMS_PATH)
        except OSError as exc:
            LOG.warning("params: save failed: %s", exc)
            return
    if _mqtt_client is not None:
        try:
            _mqtt_client.publish(MQTT_TOPIC,
                                 json.dumps(params, sort_keys=True),
                                 qos=1, retain=True)
        except Exception as exc:                       # paho raises generic Exception
            LOG.warning("params: MQTT publish failed: %s", exc)


def update(patch: dict) -> dict:
    """Merge `patch` over the current params, persist, and return the new
    full params dict. Validates that `patch` is a dict and all top-level
    keys are known so a typo in the operator console does not silently
    extend the schema."""
    if not isinstance(patch, dict):
        raise ValueError("patch must be a dict")
    unknown = set(patch.keys()) - set(DEFAULT_PARAMS.keys())
    if unknown:
        raise ValueError(f"unknown param sections: {sorted(unknown)}")
    merged = _deep_merge(load(), patch)
    save(merged)
    return merged


# ---- optional FastAPI sub-app -----------------------------------------

try:
    from fastapi import APIRouter, HTTPException

    params_app = APIRouter(prefix="/api/params", tags=["params"])

    @params_app.get("")
    def _get_params() -> dict:
        return load()

    @params_app.post("")
    def _post_params(patch: dict) -> dict:
        try:
            return update(patch)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc))
except ImportError:
    params_app = None      # FastAPI not installed; library API still works


# ---- entry point ------------------------------------------------------

def main() -> None:
    global _mqtt_client
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")
    try:
        import paho.mqtt.client as mqtt
        _mqtt_client = mqtt.Client(client_id="params_service")

        # Apply patches submitted by the base-station web UI via MQTT. We
        # use a dedicated `params/set` topic (NOT the retained `changed`
        # topic) so a subscriber that comes up late doesn't accidentally
        # re-apply a stale write.
        def _on_set(_c, _u, msg):
            try:
                patch = json.loads(msg.payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                LOG.warning("params/set: bad payload (%s)", exc)
                return
            if not isinstance(patch, dict):
                LOG.warning("params/set: payload must be a JSON object")
                return
            try:
                update(patch)
            except ValueError as exc:
                LOG.warning("params/set: rejected (%s)", exc)

        _mqtt_client.on_message = _on_set
        _mqtt_client.connect("localhost", 1883)
        _mqtt_client.subscribe("lifetrac/v25/params/set", qos=1)
        _mqtt_client.loop_start()
        # Re-publish current params on startup so freshly-booted subscribers
        # (e.g. M7 firmware over the UART bridge) can latch the latest state.
        _mqtt_client.publish(MQTT_TOPIC,
                             json.dumps(load(), sort_keys=True),
                             qos=1, retain=True)
    except (ImportError, OSError) as exc:
        LOG.info("params_service: MQTT not available (%s) — library mode only", exc)

    LOG.info("params_service: ready, %d sections", len(load()))
    if _mqtt_client is None:
        # No MQTT loop driving the process; if we exit here systemd would
        # restart-flap. Sleep forever.
        import time
        while True:
            time.sleep(3600)


if __name__ == "__main__":
    main()
