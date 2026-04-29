"""boot_self_test.py \u2014 BC-12 boot-time config-vs-hardware verification.

At process startup the base-station web_ui and lora_bridge load the
active :class:`BuildConfig`; the firmware-side M4 reports a hardware
inventory over Modbus during the same boot. This module compares the
two and emits a single ``boot_self_test`` audit-log event with a
structured pass/fail report.

The SIL surface (this file) does not actually probe hardware \u2014 the
caller passes a :class:`HardwareInventory` produced by whatever it
believes the M4 reported. The HIL half (deferred until bench rig)
replaces the synthetic inventory with a real Modbus read.

Severity model
--------------
* ``error`` \u2014 axis-count mismatches, missing proportional-capable
  hardware when the config commands ramped flow, missing safety-critical
  sensors. Sets ``report.ok = False``.
* ``warning`` \u2014 cosmetic mismatches (e.g. fewer cameras present than
  configured) that do not endanger the operator. Logged but ``ok``
  stays True.

A finding is a :class:`SelfTestFinding` with ``code`` (machine-stable
identifier) and ``message`` (human prose). Codes are stable across
releases so dashboards can pivot on them.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable

from build_config import BuildConfig

# --- public data classes ------------------------------------------------


@dataclass(frozen=True)
class HardwareInventory:
    """What the M4 (or HIL stub) reports about the physical platform."""
    track_axis_count: int
    arm_axis_count: int
    proportional_capable: bool
    hyd_pressure_sensor_count: int
    imu_present: bool
    gps_present: bool
    camera_count: int
    lora_region: str
    aux_port_count: int
    aux_coupler_type: str
    aux_case_drain_present: bool


@dataclass(frozen=True)
class SelfTestFinding:
    severity: str           # "error" | "warning"
    code: str               # stable machine identifier, e.g. "AXIS_COUNT_TRACK"
    message: str            # human prose
    expected: Any           # what BuildConfig claims
    observed: Any           # what HardwareInventory reports


@dataclass(frozen=True)
class SelfTestReport:
    unit_id: str
    config_sha256: str
    ok: bool
    findings: tuple[SelfTestFinding, ...]
    started_ts: float
    finished_ts: float

    def errors(self) -> tuple[SelfTestFinding, ...]:
        return tuple(f for f in self.findings if f.severity == "error")

    def warnings(self) -> tuple[SelfTestFinding, ...]:
        return tuple(f for f in self.findings if f.severity == "warning")


# --- finding code catalogue --------------------------------------------
#
# Machine-stable codes \u2014 dashboards / runbooks pivot on these strings,
# so they MUST NOT be renamed without a deprecation cycle. The SIL gate
# pins the catalogue.

CODES = (
    "AXIS_COUNT_TRACK",
    "AXIS_COUNT_ARM",
    "PROPORTIONAL_FLOW_UNAVAILABLE",
    "PRESSURE_SENSOR_COUNT",
    "IMU_PRESENCE",
    "GPS_PRESENCE",
    "CAMERA_COUNT_SHORT",
    "CAMERA_COUNT_EXTRA",
    "LORA_REGION_MISMATCH",
    "AUX_PORT_COUNT",
    "AUX_COUPLER_TYPE",
    "AUX_CASE_DRAIN",
)

# Severity classification per code. Errors fail boot; warnings don't.
_ERROR_CODES = frozenset({
    "AXIS_COUNT_TRACK",
    "AXIS_COUNT_ARM",
    "PROPORTIONAL_FLOW_UNAVAILABLE",
    "PRESSURE_SENSOR_COUNT",
    "IMU_PRESENCE",
    "GPS_PRESENCE",
    "LORA_REGION_MISMATCH",
    "AUX_PORT_COUNT",
    "AUX_COUPLER_TYPE",
    "AUX_CASE_DRAIN",
})


def _severity(code: str) -> str:
    return "error" if code in _ERROR_CODES else "warning"


# --- core comparator ---------------------------------------------------


def _compare(config: BuildConfig,
             hw: HardwareInventory) -> Iterable[SelfTestFinding]:
    h, s, c, sn, cm, a = (config.hydraulic, config.safety, config.cameras,
                          config.sensors, config.comm, config.aux)
    # Hydraulic axis counts \u2014 hard error: a missing axis means commanded
    # motion has no actuator, a surplus axis means the config can't drive
    # something the operator can see move.
    if h.track_axis_count != hw.track_axis_count:
        yield SelfTestFinding(
            _severity("AXIS_COUNT_TRACK"), "AXIS_COUNT_TRACK",
            "track axis count differs between build config and hardware",
            h.track_axis_count, hw.track_axis_count,
        )
    if h.arm_axis_count != hw.arm_axis_count:
        yield SelfTestFinding(
            _severity("AXIS_COUNT_ARM"), "AXIS_COUNT_ARM",
            "arm axis count differs between build config and hardware",
            h.arm_axis_count, hw.arm_axis_count,
        )
    # Proportional flow: only a problem if the config commands it but the
    # hardware can't deliver. The reverse (hardware capable, config
    # bang-bang) is fine \u2014 you're just leaving capability on the table.
    if h.proportional_flow and not hw.proportional_capable:
        yield SelfTestFinding(
            _severity("PROPORTIONAL_FLOW_UNAVAILABLE"),
            "PROPORTIONAL_FLOW_UNAVAILABLE",
            "config commands proportional flow but hardware reports bang-bang only",
            True, False,
        )
    if sn.hyd_pressure_sensor_count != hw.hyd_pressure_sensor_count:
        yield SelfTestFinding(
            _severity("PRESSURE_SENSOR_COUNT"), "PRESSURE_SENSOR_COUNT",
            "hydraulic pressure sensor count differs between config and hardware",
            sn.hyd_pressure_sensor_count, hw.hyd_pressure_sensor_count,
        )
    if sn.imu_present != hw.imu_present:
        yield SelfTestFinding(
            _severity("IMU_PRESENCE"), "IMU_PRESENCE",
            "IMU presence differs between config and hardware",
            sn.imu_present, hw.imu_present,
        )
    if sn.gps_present != hw.gps_present:
        yield SelfTestFinding(
            _severity("GPS_PRESENCE"), "GPS_PRESENCE",
            "GPS presence differs between config and hardware",
            sn.gps_present, hw.gps_present,
        )
    # Camera asymmetry: fewer cameras than expected is a warning (you'll
    # see black tiles in the UI but nothing unsafe); more cameras than
    # expected is also a warning (extra hardware, no UI route). Neither
    # endangers the operator so neither fails the boot.
    if hw.camera_count < c.count:
        yield SelfTestFinding(
            _severity("CAMERA_COUNT_SHORT"), "CAMERA_COUNT_SHORT",
            "fewer cameras present than build config expects",
            c.count, hw.camera_count,
        )
    elif hw.camera_count > c.count:
        yield SelfTestFinding(
            _severity("CAMERA_COUNT_EXTRA"), "CAMERA_COUNT_EXTRA",
            "more cameras present than build config expects",
            c.count, hw.camera_count,
        )
    # LoRa region: hard error \u2014 transmitting on the wrong region's
    # frequency plan is illegal in most jurisdictions.
    if cm.lora_region != hw.lora_region:
        yield SelfTestFinding(
            _severity("LORA_REGION_MISMATCH"), "LORA_REGION_MISMATCH",
            "LoRa region in config does not match radio module's region",
            cm.lora_region, hw.lora_region,
        )
    if a.port_count != hw.aux_port_count:
        yield SelfTestFinding(
            _severity("AUX_PORT_COUNT"), "AUX_PORT_COUNT",
            "aux hydraulic port count differs between config and hardware",
            a.port_count, hw.aux_port_count,
        )
    if a.coupler_type != hw.aux_coupler_type:
        yield SelfTestFinding(
            _severity("AUX_COUPLER_TYPE"), "AUX_COUPLER_TYPE",
            "aux coupler type differs between config and hardware",
            a.coupler_type, hw.aux_coupler_type,
        )
    if a.case_drain_present != hw.aux_case_drain_present:
        yield SelfTestFinding(
            _severity("AUX_CASE_DRAIN"), "AUX_CASE_DRAIN",
            "aux case-drain presence differs between config and hardware",
            a.case_drain_present, hw.aux_case_drain_present,
        )


def run_self_test(config: BuildConfig,
                  hardware: HardwareInventory,
                  *,
                  now: Callable[[], float] = time.time) -> SelfTestReport:
    """Compare config against hardware inventory; return a structured report."""
    started = now()
    findings = tuple(_compare(config, hardware))
    finished = now()
    ok = not any(f.severity == "error" for f in findings)
    return SelfTestReport(
        unit_id=config.unit_id,
        config_sha256=config.config_sha256,
        ok=ok,
        findings=findings,
        started_ts=started,
        finished_ts=finished,
    )


def emit_audit(audit, report: SelfTestReport, *, component: str) -> None:
    """Write one ``boot_self_test`` event to the audit log.

    The serialised finding list keeps each finding flat so jq / Pandas
    pipelines on ``audit.jsonl`` can pivot on ``code`` without nested
    object handling.
    """
    audit.record(
        "boot_self_test",
        component=component,
        unit_id=report.unit_id,
        config_sha256=report.config_sha256,
        ok=report.ok,
        error_count=len(report.errors()),
        warning_count=len(report.warnings()),
        findings=[
            {
                "severity": f.severity,
                "code": f.code,
                "message": f.message,
                "expected": f.expected,
                "observed": f.observed,
            }
            for f in report.findings
        ],
        started_ts=report.started_ts,
        finished_ts=report.finished_ts,
    )
