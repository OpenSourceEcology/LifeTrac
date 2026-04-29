"""SIL gate for boot_self_test.py (Round 39 / BC-12).

Pins the boot-time config-vs-hardware verification self-test against
its contract: matching inventory passes; safety-significant mismatches
fail boot; cosmetic mismatches degrade to warnings; the audit-log
event has the canonical shape; the finding-code catalogue is stable.

The HIL half (real Modbus probe of the M4) waits for the bench rig.
"""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

_BS = Path(__file__).resolve().parents[1]
if str(_BS) not in sys.path:
    sys.path.insert(0, str(_BS))

import build_config as _bc  # noqa: E402
import boot_self_test as _bst  # noqa: E402
from audit_log import AuditLog  # noqa: E402


def _matching_inventory(cfg: _bc.BuildConfig) -> _bst.HardwareInventory:
    """Build a HardwareInventory that perfectly matches ``cfg`` \u2014
    handy starting point for "twiddle one field" mismatch tests."""
    return _bst.HardwareInventory(
        track_axis_count=cfg.hydraulic.track_axis_count,
        arm_axis_count=cfg.hydraulic.arm_axis_count,
        proportional_capable=cfg.hydraulic.proportional_flow,
        hyd_pressure_sensor_count=cfg.sensors.hyd_pressure_sensor_count,
        imu_present=cfg.sensors.imu_present,
        gps_present=cfg.sensors.gps_present,
        camera_count=cfg.cameras.count,
        lora_region=cfg.comm.lora_region,
        aux_port_count=cfg.aux.port_count,
        aux_coupler_type=cfg.aux.coupler_type,
        aux_case_drain_present=cfg.aux.case_drain_present,
    )


def _fake_clock():
    """Return a callable returning monotonically-increasing fake times."""
    state = {"t": 1700000000.0}

    def now() -> float:
        state["t"] += 0.001
        return state["t"]
    return now


class _ConfigFixture(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.cfg = _bc.load()


class BC12_A_MatchingInventoryPasses(_ConfigFixture):
    def test_matching_inventory_yields_ok_no_findings(self) -> None:
        report = _bst.run_self_test(self.cfg, _matching_inventory(self.cfg),
                                    now=_fake_clock())
        self.assertTrue(report.ok)
        self.assertEqual(report.findings, ())
        self.assertEqual(report.unit_id, self.cfg.unit_id)
        self.assertEqual(report.config_sha256, self.cfg.config_sha256)
        self.assertGreater(report.finished_ts, report.started_ts)


class BC12_B_AxisCountMismatchFailsBoot(_ConfigFixture):
    def test_track_axis_short_yields_error_and_ok_false(self) -> None:
        hw = replace(_matching_inventory(self.cfg),
                     track_axis_count=self.cfg.hydraulic.track_axis_count - 1)
        report = _bst.run_self_test(self.cfg, hw, now=_fake_clock())
        self.assertFalse(report.ok)
        codes = {f.code for f in report.findings}
        self.assertIn("AXIS_COUNT_TRACK", codes)
        finding = next(f for f in report.findings if f.code == "AXIS_COUNT_TRACK")
        self.assertEqual(finding.severity, "error")
        self.assertEqual(finding.expected, self.cfg.hydraulic.track_axis_count)
        self.assertEqual(finding.observed, hw.track_axis_count)

    def test_arm_axis_extra_yields_error(self) -> None:
        hw = replace(_matching_inventory(self.cfg),
                     arm_axis_count=self.cfg.hydraulic.arm_axis_count + 1)
        report = _bst.run_self_test(self.cfg, hw, now=_fake_clock())
        self.assertFalse(report.ok)
        self.assertIn("AXIS_COUNT_ARM",
                      {f.code for f in report.findings})


class BC12_C_ProportionalFlowAsymmetry(_ConfigFixture):
    def test_config_wants_proportional_hardware_lacks_it_fails(self) -> None:
        # Default config has proportional_flow=True; flip hardware to bang-bang.
        self.assertTrue(self.cfg.hydraulic.proportional_flow,
                        "fixture assumption: default config commands proportional")
        hw = replace(_matching_inventory(self.cfg),
                     proportional_capable=False)
        report = _bst.run_self_test(self.cfg, hw, now=_fake_clock())
        self.assertFalse(report.ok)
        self.assertIn("PROPORTIONAL_FLOW_UNAVAILABLE",
                      {f.code for f in report.findings})

    def test_hardware_capable_but_config_bang_bang_does_not_fail(self) -> None:
        # We synthesise the asymmetric direction by overriding the relevant
        # config field via dataclasses.replace on the frozen dataclass.
        new_hyd = replace(self.cfg.hydraulic, proportional_flow=False)
        cfg2 = replace(self.cfg, hydraulic=new_hyd)
        # Hardware still reports capable.
        hw = _matching_inventory(cfg2)
        hw = replace(hw, proportional_capable=True)
        report = _bst.run_self_test(cfg2, hw, now=_fake_clock())
        self.assertTrue(report.ok)
        self.assertEqual(report.findings, ())


class BC12_D_PresenceAndSensorMismatches(_ConfigFixture):
    def test_imu_presence_mismatch_is_error(self) -> None:
        hw = replace(_matching_inventory(self.cfg),
                     imu_present=not self.cfg.sensors.imu_present)
        report = _bst.run_self_test(self.cfg, hw, now=_fake_clock())
        self.assertFalse(report.ok)
        self.assertIn("IMU_PRESENCE", {f.code for f in report.findings})

    def test_gps_presence_mismatch_is_error(self) -> None:
        hw = replace(_matching_inventory(self.cfg),
                     gps_present=not self.cfg.sensors.gps_present)
        report = _bst.run_self_test(self.cfg, hw, now=_fake_clock())
        self.assertFalse(report.ok)
        self.assertIn("GPS_PRESENCE", {f.code for f in report.findings})

    def test_pressure_sensor_count_mismatch_is_error(self) -> None:
        hw = replace(_matching_inventory(self.cfg),
                     hyd_pressure_sensor_count=
                     self.cfg.sensors.hyd_pressure_sensor_count + 1)
        report = _bst.run_self_test(self.cfg, hw, now=_fake_clock())
        self.assertFalse(report.ok)
        self.assertIn("PRESSURE_SENSOR_COUNT",
                      {f.code for f in report.findings})


class BC12_E_CameraAsymmetryIsWarning(_ConfigFixture):
    def test_fewer_cameras_present_is_warning_not_error(self) -> None:
        hw = replace(_matching_inventory(self.cfg),
                     camera_count=self.cfg.cameras.count + 0)
        # Make hw report one fewer than configured.
        hw = replace(hw, camera_count=max(0, self.cfg.cameras.count - 1))
        # Only meaningful if config has at least one camera.
        self.assertGreaterEqual(self.cfg.cameras.count, 1)
        report = _bst.run_self_test(self.cfg, hw, now=_fake_clock())
        self.assertTrue(report.ok, msg="camera shortage must NOT fail boot")
        codes = {f.code for f in report.findings}
        self.assertIn("CAMERA_COUNT_SHORT", codes)
        f = next(f for f in report.findings if f.code == "CAMERA_COUNT_SHORT")
        self.assertEqual(f.severity, "warning")

    def test_more_cameras_present_is_warning(self) -> None:
        hw = replace(_matching_inventory(self.cfg),
                     camera_count=self.cfg.cameras.count + 1)
        report = _bst.run_self_test(self.cfg, hw, now=_fake_clock())
        self.assertTrue(report.ok)
        self.assertIn("CAMERA_COUNT_EXTRA", {f.code for f in report.findings})


class BC12_F_LoraAndAuxMismatchesAreErrors(_ConfigFixture):
    def test_lora_region_mismatch_fails(self) -> None:
        hw = replace(_matching_inventory(self.cfg), lora_region="eu868")
        report = _bst.run_self_test(self.cfg, hw, now=_fake_clock())
        self.assertFalse(report.ok)
        self.assertIn("LORA_REGION_MISMATCH",
                      {f.code for f in report.findings})

    def test_aux_port_count_mismatch_fails(self) -> None:
        hw = replace(_matching_inventory(self.cfg),
                     aux_port_count=self.cfg.aux.port_count + 1)
        report = _bst.run_self_test(self.cfg, hw, now=_fake_clock())
        self.assertFalse(report.ok)
        self.assertIn("AUX_PORT_COUNT", {f.code for f in report.findings})

    def test_aux_coupler_type_mismatch_fails(self) -> None:
        hw = replace(_matching_inventory(self.cfg),
                     aux_coupler_type="iso_5675")
        # Default config is "none"; ensure they differ.
        self.assertNotEqual(self.cfg.aux.coupler_type, "iso_5675")
        report = _bst.run_self_test(self.cfg, hw, now=_fake_clock())
        self.assertFalse(report.ok)
        self.assertIn("AUX_COUPLER_TYPE", {f.code for f in report.findings})


class BC12_G_AuditEventShape(_ConfigFixture):
    def test_emit_audit_writes_canonical_event(self) -> None:
        hw = replace(_matching_inventory(self.cfg),
                     track_axis_count=self.cfg.hydraulic.track_axis_count - 1,
                     camera_count=self.cfg.cameras.count + 1)
        report = _bst.run_self_test(self.cfg, hw, now=_fake_clock())
        with tempfile.TemporaryDirectory() as td:
            log_path = Path(td) / "audit.jsonl"
            audit = AuditLog(path=str(log_path))
            try:
                _bst.emit_audit(audit, report, component="web_ui")
            finally:
                audit.close()
            lines = [l for l in log_path.read_text(encoding="utf-8").splitlines()
                     if l.strip()]
            self.assertEqual(len(lines), 1)
            rec = json.loads(lines[0])
        self.assertEqual(rec["event"], "boot_self_test")
        self.assertEqual(rec["component"], "web_ui")
        self.assertEqual(rec["unit_id"], self.cfg.unit_id)
        self.assertEqual(rec["config_sha256"], self.cfg.config_sha256)
        self.assertFalse(rec["ok"])
        self.assertEqual(rec["error_count"], 1)
        self.assertEqual(rec["warning_count"], 1)
        self.assertEqual(len(rec["findings"]), 2)
        for f in rec["findings"]:
            self.assertEqual(set(f.keys()),
                             {"severity", "code", "message",
                              "expected", "observed"})


class BC12_H_FindingCodeCatalogueIsStable(unittest.TestCase):
    """Pin the machine-stable code list. Renames are caught here so we
    notice them before dashboards / runbooks break."""

    EXPECTED = (
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

    def test_code_catalogue_is_exactly_the_expected_set(self) -> None:
        self.assertEqual(_bst.CODES, self.EXPECTED)


if __name__ == "__main__":
    unittest.main()
