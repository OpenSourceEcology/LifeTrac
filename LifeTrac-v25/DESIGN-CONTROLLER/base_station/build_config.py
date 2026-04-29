"""Build-configuration loader for LifeTrac v25 (Round 27 / BC-02).

A LifeTrac build is not bit-for-bit identical across units in the field:
some lack a second camera, some run without an IMU/GPS pair, some are
single-axis. This module owns the **single source of truth** for which
optional capabilities are present on a given unit and what the per-unit
parameters are.

Design contract
---------------

* Schema is canonical JSON Schema (draft-07 subset) at
  ``base_station/config/build_config.schema.json``. Human-readable
  documentation in ``DESIGN-CONTROLLER/CAPABILITY_INVENTORY.md``.
* The loader is **strict**: unknown top-level or nested keys, out-of-range
  values, wrong types, and bad enum tokens all raise
  :class:`BuildConfigError`. There is no silent coercion.
* Fallback chain:

    1. ``LIFETRAC_BUILD_CONFIG_PATH`` env var — explicit absolute file path.
    2. ``base_station/config/build.<unit_id>.toml`` — per-unit override.
    3. ``base_station/config/build.default.toml`` — canonical-BOM fallback.

  Layer 1 wins; layer 3 is the floor and must always exist.
* :func:`load` returns a :class:`BuildConfig` immutable dataclass. The
  raw validated dict is exposed as ``cfg.raw``; deterministic
  ``cfg.config_sha256`` (canonical JSON, sorted keys, no whitespace)
  is what the audit log records on every boot.

Zero third-party dependencies — uses stdlib ``tomllib`` (Python 3.11+),
hand-rolled JSON Schema validator over the small draft-07 subset we
actually use. Fits in <300 LOC by design.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

try:  # Python 3.11+
    import tomllib  # type: ignore[import-not-found]
except ModuleNotFoundError:  # pragma: no cover - exercised only on <3.11 hosts
    import tomli as tomllib  # type: ignore[no-redef]


_HERE = Path(__file__).resolve().parent
CONFIG_DIR = _HERE / "config"
SCHEMA_PATH = CONFIG_DIR / "build_config.schema.json"
DEFAULT_TOML = CONFIG_DIR / "build.default.toml"


class BuildConfigError(ValueError):
    """Raised when a build configuration fails to load or validate."""


# ---------------------------------------------------------------- dataclasses


@dataclass(frozen=True)
class HydraulicConfig:
    track_axis_count: int
    arm_axis_count: int
    proportional_flow: bool
    track_ramp_seconds: float
    arm_ramp_seconds: float


@dataclass(frozen=True)
class SafetyConfig:
    estop_topology: str
    estop_latency_ms_max: int
    modbus_fail_latch_count: int
    m4_watchdog_ms: int


@dataclass(frozen=True)
class CamerasConfig:
    count: int
    front_present: bool
    rear_present: bool
    implement_present: bool
    crop_health_present: bool
    coral_tpu: str


@dataclass(frozen=True)
class SensorsConfig:
    imu_present: bool
    imu_model: str
    gps_present: bool
    gps_model: str
    hyd_pressure_sensor_count: int


@dataclass(frozen=True)
class CommConfig:
    lora_region: str
    cellular_backup_present: bool
    handheld_present: bool


@dataclass(frozen=True)
class UiConfig:
    web_ui_enabled: bool
    max_control_subscribers: int
    pin_required_for_control: bool


@dataclass(frozen=True)
class NetConfig:
    mqtt_host: str
    mqtt_port: int


@dataclass(frozen=True)
class AuxConfig:
    # Round 33 / BC-11: auxiliary attachment hydraulic ports.
    port_count: int
    coupler_type: str
    case_drain_present: bool


@dataclass(frozen=True)
class BuildConfig:
    unit_id: str
    schema_version: int
    hydraulic: HydraulicConfig
    safety: SafetyConfig
    cameras: CamerasConfig
    sensors: SensorsConfig
    comm: CommConfig
    ui: UiConfig
    net: NetConfig
    aux: AuxConfig
    source_path: Path
    raw: Mapping[str, Any] = field(repr=False)

    @property
    def config_sha256(self) -> str:
        """Deterministic hash of the validated config (audit-log identity)."""
        canonical = json.dumps(
            self.raw, sort_keys=True, separators=(",", ":"), ensure_ascii=False
        )
        return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


# ---------------------------------------------------------------- validator


def _validate(value: Any, schema: Mapping[str, Any], path: str) -> None:
    """Tiny JSON-Schema (draft-07) subset validator.

    Only supports the constructs the build-config schema actually uses:
    type (object/integer/number/boolean/string), required, properties,
    additionalProperties (false only), enum, minimum/maximum, pattern,
    minLength/maxLength.
    """
    expected = schema.get("type")
    if expected == "object":
        if not isinstance(value, dict):
            raise BuildConfigError(f"{path}: expected object, got {type(value).__name__}")
        required = schema.get("required", [])
        properties = schema.get("properties", {})
        for key in required:
            if key not in value:
                raise BuildConfigError(f"{path}: missing required key {key!r}")
        if schema.get("additionalProperties") is False:
            extras = set(value) - set(properties)
            if extras:
                raise BuildConfigError(
                    f"{path}: unknown key(s) {sorted(extras)!r}"
                )
        for key, sub in value.items():
            if key in properties:
                _validate(sub, properties[key], f"{path}.{key}" if path else key)
        return

    if expected == "integer":
        # bool is a subclass of int in Python — reject it explicitly.
        if isinstance(value, bool) or not isinstance(value, int):
            raise BuildConfigError(f"{path}: expected integer, got {type(value).__name__}")
    elif expected == "number":
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise BuildConfigError(f"{path}: expected number, got {type(value).__name__}")
    elif expected == "boolean":
        if not isinstance(value, bool):
            raise BuildConfigError(f"{path}: expected boolean, got {type(value).__name__}")
    elif expected == "string":
        if not isinstance(value, str):
            raise BuildConfigError(f"{path}: expected string, got {type(value).__name__}")

    if "enum" in schema and value not in schema["enum"]:
        raise BuildConfigError(
            f"{path}: value {value!r} not in enum {schema['enum']!r}"
        )
    if "minimum" in schema and value < schema["minimum"]:
        raise BuildConfigError(f"{path}: {value!r} < minimum {schema['minimum']!r}")
    if "maximum" in schema and value > schema["maximum"]:
        raise BuildConfigError(f"{path}: {value!r} > maximum {schema['maximum']!r}")
    if "minLength" in schema and len(value) < schema["minLength"]:
        raise BuildConfigError(f"{path}: length {len(value)} < minLength {schema['minLength']}")
    if "maxLength" in schema and len(value) > schema["maxLength"]:
        raise BuildConfigError(f"{path}: length {len(value)} > maxLength {schema['maxLength']}")
    if "pattern" in schema and not re.search(schema["pattern"], value):
        raise BuildConfigError(
            f"{path}: value {value!r} does not match pattern {schema['pattern']!r}"
        )


def load_schema(path: Path = SCHEMA_PATH) -> Mapping[str, Any]:
    """Load and parse the JSON Schema document. Cached only by callers."""
    with path.open("rb") as f:
        return json.loads(f.read().decode("utf-8"))


# ---------------------------------------------------------------- public API


def _resolve_source(unit_id: str | None) -> Path:
    explicit = os.environ.get("LIFETRAC_BUILD_CONFIG_PATH")
    if explicit:
        p = Path(explicit)
        if not p.is_file():
            raise BuildConfigError(
                f"LIFETRAC_BUILD_CONFIG_PATH={explicit!r} does not point to a file"
            )
        return p
    if unit_id:
        candidate = CONFIG_DIR / f"build.{unit_id}.toml"
        if candidate.is_file():
            return candidate
    if not DEFAULT_TOML.is_file():
        raise BuildConfigError(
            f"default build config missing at {DEFAULT_TOML} — repository corrupt"
        )
    return DEFAULT_TOML


def load(unit_id: str | None = None) -> BuildConfig:
    """Load and validate the active build configuration.

    Resolution order: ``LIFETRAC_BUILD_CONFIG_PATH`` env var, then
    ``build.<unit_id>.toml`` if ``unit_id`` is provided and the file
    exists, then the canonical ``build.default.toml``.
    """
    source = _resolve_source(unit_id)
    with source.open("rb") as f:
        raw_bytes = f.read()
    try:
        data = tomllib.loads(raw_bytes.decode("utf-8"))
    except tomllib.TOMLDecodeError as exc:
        raise BuildConfigError(f"{source}: TOML parse error: {exc}") from exc

    schema = load_schema()
    _validate(data, schema, path="")

    return BuildConfig(
        unit_id=data["unit_id"],
        schema_version=data["schema_version"],
        hydraulic=HydraulicConfig(**data["hydraulic"]),
        safety=SafetyConfig(**data["safety"]),
        cameras=CamerasConfig(**data["cameras"]),
        sensors=SensorsConfig(**data["sensors"]),
        comm=CommConfig(**data["comm"]),
        ui=UiConfig(**data["ui"]),
        net=NetConfig(**data["net"]),
        aux=AuxConfig(**data["aux"]),
        source_path=source,
        raw=data,
    )


# ---------------------------------------------------------------- BC-10 hot-reload
#
# Round 29 (BC-10): config delivery + hot-reload contract. Every leaf in the
# schema carries a ``reload_class`` annotation declaring how a change to that
# leaf is meant to land on a running unit:
#
#   * ``live`` -- can be hot-applied without restarting the daemon. UI strings,
#     camera-presence flags, ramp seconds, MQTT host (with reconnect),
#     ``max_control_subscribers``, IMU/GPS presence flags, sensor counts.
#   * ``restart_required`` -- needs a graceful daemon restart. ``lora_region``
#     (radio re-tune), ``estop_topology``, axis counts, model strings (driver
#     init), ``unit_id``, ``web_ui_enabled``, ``cellular_backup_present``.
#   * ``firmware_required`` -- needs a re-flash of one or more sketches.
#     ``schema_version`` (by definition), ``m4_watchdog_ms`` (M4-side literal).
#
# Live-reload of a ``restart_required`` field is a contract violation; the
# delivery installer (BC-10 / Round 29b) and the editor (BC-05 / Round 30)
# both consult :func:`diff_reload_classes` to decide whether to apply, defer,
# or reject. Live-reload also requires the unit to be quiescent
# (:func:`evaluate_quiescence`) so a value change can't interleave with an
# active hydraulic command.

RELOAD_CLASSES: tuple[str, ...] = ("live", "restart_required", "firmware_required")
_RELOAD_CLASS_RANK = {"live": 0, "restart_required": 1, "firmware_required": 2}


def _walk_leaves(
    schema: Mapping[str, Any], prefix: str = ""
) -> "list[tuple[str, Mapping[str, Any]]]":
    """Yield ``(dotted_path, leaf_schema)`` for every non-object property."""
    out: list[tuple[str, Mapping[str, Any]]] = []
    if schema.get("type") == "object":
        for key, sub in schema.get("properties", {}).items():
            out.extend(_walk_leaves(sub, f"{prefix}.{key}" if prefix else key))
        return out
    out.append((prefix, schema))
    return out


def iter_reload_classes(schema: Mapping[str, Any] | None = None) -> dict[str, str]:
    """Return a ``{dotted_path: reload_class}`` map for every schema leaf.

    Raises :class:`BuildConfigError` if any leaf is missing a ``reload_class``
    annotation or carries one outside :data:`RELOAD_CLASSES` -- the schema is
    the contract; an un-annotated leaf is a bug, not a "default to live."
    """
    schema = schema if schema is not None else load_schema()
    result: dict[str, str] = {}
    for path, leaf in _walk_leaves(schema):
        rc = leaf.get("reload_class")
        if rc is None:
            raise BuildConfigError(f"{path}: schema leaf missing reload_class")
        if rc not in RELOAD_CLASSES:
            raise BuildConfigError(
                f"{path}: reload_class {rc!r} not in {RELOAD_CLASSES!r}"
            )
        result[path] = rc
    return result


def _flatten(value: Any, prefix: str = "") -> dict[str, Any]:
    if isinstance(value, dict):
        out: dict[str, Any] = {}
        for k, v in value.items():
            out.update(_flatten(v, f"{prefix}.{k}" if prefix else k))
        return out
    return {prefix: value}


@dataclass(frozen=True)
class ReloadDiff:
    """Outcome of comparing two validated build configs.

    ``changed`` lists the dotted paths whose leaf values differ.
    ``classes`` maps each changed path to its declared reload class.
    ``worst`` is the strictest class present (``live`` < ``restart_required``
    < ``firmware_required``); ``None`` when nothing changed.
    """

    changed: tuple[str, ...]
    classes: Mapping[str, str]
    worst: str | None

    @property
    def is_empty(self) -> bool:
        return not self.changed

    @property
    def restart_required(self) -> bool:
        return self.worst in ("restart_required", "firmware_required")

    @property
    def firmware_required(self) -> bool:
        return self.worst == "firmware_required"


def diff_reload_classes(
    old: BuildConfig, new: BuildConfig, schema: Mapping[str, Any] | None = None
) -> ReloadDiff:
    """Diff two validated configs and classify the change set.

    Both configs must have already been validated against the schema.
    Unknown leaves (present in the data but not the schema) raise
    :class:`BuildConfigError` -- the validator should have caught them, so
    hitting one here means a schema/loader drift bug.
    """
    classes = iter_reload_classes(schema)
    flat_old = _flatten(dict(old.raw))
    flat_new = _flatten(dict(new.raw))
    keys = set(flat_old) | set(flat_new)
    changed: list[str] = []
    diff_classes: dict[str, str] = {}
    for key in sorted(keys):
        if flat_old.get(key) != flat_new.get(key):
            if key not in classes:
                raise BuildConfigError(
                    f"diff_reload_classes: leaf {key!r} not declared in schema"
                )
            changed.append(key)
            diff_classes[key] = classes[key]
    worst: str | None = None
    for rc in diff_classes.values():
        if worst is None or _RELOAD_CLASS_RANK[rc] > _RELOAD_CLASS_RANK[worst]:
            worst = rc
    return ReloadDiff(changed=tuple(changed), classes=diff_classes, worst=worst)


@dataclass(frozen=True)
class QuiescenceState:
    """Snapshot of the runtime conditions live-reload depends on.

    Populated by the daemon (``lora_bridge`` for tractor-side reload,
    ``web_ui`` for base-side reload). Pure data; no side effects.
    """

    parked_seconds: float
    """Seconds the tractor has continuously been in STATE_PARKED."""

    active_control_subscribers: int
    """Count of currently-connected ``/ws/control`` subscribers."""

    m7_tx_queue_depth: int
    """Outstanding M7-side LoRa TX frames queued."""

    engine_idle_or_off: bool
    """True if engine is off or at idle-low; False if under load."""


@dataclass(frozen=True)
class QuiescenceResult:
    """Outcome of evaluating a :class:`QuiescenceState`.

    ``ok`` indicates whether a live-reload is safe right now. ``reason``
    is a short operator-facing string when ``ok`` is False, suitable for
    the OLED status line / web UI banner.
    """

    ok: bool
    reason: str | None

    def __bool__(self) -> bool:  # convenience for ``if quiescence(state):``
        return self.ok


def evaluate_quiescence(
    state: QuiescenceState, *, parked_seconds_min: float = 30.0
) -> QuiescenceResult:
    """Decide whether a live config-reload may proceed.

    All four conditions must hold: the tractor must have been parked for
    at least ``parked_seconds_min`` seconds (default 30s), no ``/ws/control``
    subscribers may be connected, the M7 TX queue must be empty, and the
    engine must be off or at idle-low.

    Returns the first failure reason in operator-facing wording so the OLED
    line / web banner can show a single coherent message rather than a list.
    """
    if state.parked_seconds < parked_seconds_min:
        return QuiescenceResult(
            False,
            f"not parked long enough ({state.parked_seconds:.0f}s < {parked_seconds_min:.0f}s)",
        )
    if state.active_control_subscribers > 0:
        return QuiescenceResult(
            False,
            f"control session active ({state.active_control_subscribers} subscriber(s))",
        )
    if state.m7_tx_queue_depth > 0:
        return QuiescenceResult(
            False, f"radio busy (M7 TX queue depth {state.m7_tx_queue_depth})"
        )
    if not state.engine_idle_or_off:
        return QuiescenceResult(False, "engine under load")
    return QuiescenceResult(True, None)


__all__ = [
    "BuildConfig",
    "BuildConfigError",
    "CamerasConfig",
    "CommConfig",
    "HydraulicConfig",
    "NetConfig",
    "QuiescenceResult",
    "QuiescenceState",
    "ReloadDiff",
    "RELOAD_CLASSES",
    "SafetyConfig",
    "SensorsConfig",
    "UiConfig",
    "CONFIG_DIR",
    "DEFAULT_TOML",
    "SCHEMA_PATH",
    "diff_reload_classes",
    "evaluate_quiescence",
    "iter_reload_classes",
    "load",
    "load_schema",
]
