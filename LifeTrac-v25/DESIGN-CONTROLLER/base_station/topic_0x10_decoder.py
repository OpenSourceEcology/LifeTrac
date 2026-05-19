"""S1.3 decoder: structured parser for LoRa TelemetryFrame topic 0x10.

Topic 0x10 (``lifetrac/v25/control/source_active``) historically shipped as a
single-byte payload (the active source enum). The LoRa link evolution
(MASTER_PLAN.md §6 + LORA_IMPLEMENTATION.md §4/§7) adds:

* per-source RSSI/SNR/loss-rate
* current SF rung (from CMD_LINK_TUNE adaptive ladder)
* airtime utilisation triple ``(U_image, U_telemetry, U_total)``
* ``link_unstable`` flag (per LORA_PROTOCOL.md §3 step 4)

These ride as JSON payloads on the same topic. ``web_ui._decode_payload``
already handles "1 byte → enum / JSON → dict" routing; this module sits one
level up and *normalises* the dict form into a typed ``SourceActiveSnapshot``
so consumers (the LINK pill UI, the W2-02 stability harness, future
operator-canvas overlays) don't all duplicate the unknown-field guards.

Pure-Python, no MQTT/PIL deps — fast to test, safe to import from any layer.

The decoder is **forgiving**: any field can be missing or ``None`` (legacy
payloads, partial publishes during boot, intentional fail-closed publishes
from the bridge when no source is active). Unknown JSON keys are preserved
in ``extras`` so a forward-rev payload doesn't lose data on round-trip.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any


# Active-source enum mirrors `lora_proto.SRC_*` constants and
# `web_ui.SOURCE_BY_ID`. Keep in sync if those values ever change.
_SOURCE_NAMES = {
    0x01: "handheld",
    0x02: "base",
    0x03: "tractor",
    0x04: "autonomy",
    0xFF: "none",
}


@dataclass
class PerSourceLink:
    """Snapshot of one source's link metrics. All fields optional."""

    rssi_dbm: float | None = None
    snr_db: float | None = None
    loss_rate: float | None = None
    last_heard_ms: int | None = None


@dataclass
class SourceActiveSnapshot:
    """Normalised topic 0x10 payload.

    Fields mirror the JSON schema used by the bridge's source-active
    publisher; legacy 1-byte payloads decode with ``active_source`` set and
    every other field None.
    """

    active_source: str | None = None
    sf_rung: int | None = None
    link_unstable: bool = False
    u_image: float | None = None
    u_telemetry: float | None = None
    u_total: float | None = None
    sources: dict[str, PerSourceLink] = field(default_factory=dict)
    raw_legacy_byte: int | None = None
    extras: dict[str, Any] = field(default_factory=dict)


# Known top-level JSON keys — anything else goes to ``extras``.
_KNOWN_KEYS = frozenset({
    "active_source",
    "sf_rung",
    "link_unstable",
    "u_image",
    "u_telemetry",
    "u_total",
    "sources",
    # Aliases the bridge has emitted at various points in development.
    "U_image",
    "U_telemetry",
    "U_total",
})


def _resolve_source_name(value: Any) -> str | None:
    if value is None:
        return None
    if isinstance(value, str):
        return value or None
    if isinstance(value, int) and not isinstance(value, bool):
        return _SOURCE_NAMES.get(value)
    return None


def _coerce_float(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _coerce_int(value: Any) -> int | None:
    if value is None or isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, float) and value.is_integer():
        return int(value)
    return None


def _decode_per_source_map(value: Any) -> dict[str, PerSourceLink]:
    """Accept either a dict keyed by name or a list of dicts with a ``name`` key."""
    out: dict[str, PerSourceLink] = {}
    if isinstance(value, dict):
        for name, entry in value.items():
            if isinstance(entry, dict):
                out[str(name)] = PerSourceLink(
                    rssi_dbm=_coerce_float(entry.get("rssi_dbm")),
                    snr_db=_coerce_float(entry.get("snr_db")),
                    loss_rate=_coerce_float(entry.get("loss_rate")),
                    last_heard_ms=_coerce_int(entry.get("last_heard_ms")),
                )
    elif isinstance(value, list):
        for entry in value:
            if isinstance(entry, dict):
                name = entry.get("name") or entry.get("source")
                if isinstance(name, str) and name:
                    out[name] = PerSourceLink(
                        rssi_dbm=_coerce_float(entry.get("rssi_dbm")),
                        snr_db=_coerce_float(entry.get("snr_db")),
                        loss_rate=_coerce_float(entry.get("loss_rate")),
                        last_heard_ms=_coerce_int(entry.get("last_heard_ms")),
                    )
    return out


def decode_topic_0x10(payload: bytes | str | dict) -> SourceActiveSnapshot:
    """Normalise a topic-0x10 payload into a ``SourceActiveSnapshot``.

    Accepts:

    * ``bytes``: a single-byte legacy payload (active-source enum) or a
      UTF-8 JSON dict. Anything else returns an empty snapshot.
    * ``str``: a JSON dict. Anything else returns an empty snapshot.
    * ``dict``: already-decoded JSON form (e.g. from ``web_ui._decode_payload``).

    Never raises — malformed input produces a fail-closed empty snapshot
    rather than a TypeError that would crash the consumer.
    """
    snapshot = SourceActiveSnapshot()

    if isinstance(payload, bytes):
        if len(payload) == 1:
            snapshot.raw_legacy_byte = payload[0]
            snapshot.active_source = _resolve_source_name(payload[0])
            return snapshot
        try:
            payload = payload.decode("utf-8")
        except UnicodeDecodeError:
            return snapshot

    if isinstance(payload, str):
        stripped = payload.lstrip()
        if not stripped or stripped[0] not in "{[":
            return snapshot
        try:
            payload = json.loads(payload)
        except json.JSONDecodeError:
            return snapshot

    if not isinstance(payload, dict):
        return snapshot

    snapshot.active_source = _resolve_source_name(payload.get("active_source"))
    snapshot.sf_rung = _coerce_int(payload.get("sf_rung"))
    snapshot.link_unstable = bool(payload.get("link_unstable", False))
    # Accept both lowercase and the legacy uppercase ``U_*`` keys.
    snapshot.u_image = (
        _coerce_float(payload.get("u_image"))
        if payload.get("u_image") is not None
        else _coerce_float(payload.get("U_image"))
    )
    snapshot.u_telemetry = (
        _coerce_float(payload.get("u_telemetry"))
        if payload.get("u_telemetry") is not None
        else _coerce_float(payload.get("U_telemetry"))
    )
    snapshot.u_total = (
        _coerce_float(payload.get("u_total"))
        if payload.get("u_total") is not None
        else _coerce_float(payload.get("U_total"))
    )
    snapshot.sources = _decode_per_source_map(payload.get("sources"))
    snapshot.extras = {k: v for k, v in payload.items() if k not in _KNOWN_KEYS}
    return snapshot
