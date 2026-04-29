"""SIL gate for ``lifetrac-config self-test`` (Round 41 / BC-12C).

Round 41 wires the Round 39 boot_self_test comparator into the
``lifetrac-config`` CLI so a depot operator can dry-run the same
self-test on the install bench (or post-incident, against a captured
inventory dump) without booting a real tractor.

Test classes use the BC12C_* prefix so MASTER_TEST_PROGRAM.md $5
maps cleanly back to BC-12 (CLI half).
"""

from __future__ import annotations

import io
import json
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

_BS = Path(__file__).resolve().parents[1]
_REPO = _BS.parents[1]
_TOOLS = _REPO / "tools"
_DEFAULT_TOML = _BS / "config" / "build.default.toml"

if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))
if str(_BS) not in sys.path:
    sys.path.insert(0, str(_BS))

import lifetrac_config as lc  # noqa: E402
import build_config as _bc  # noqa: E402
import boot_self_test as _bst  # noqa: E402


def _matching_inventory_dict(cfg: _bc.BuildConfig) -> dict:
    return {
        "track_axis_count": cfg.hydraulic.track_axis_count,
        "arm_axis_count": cfg.hydraulic.arm_axis_count,
        "proportional_capable": cfg.hydraulic.proportional_flow,
        "hyd_pressure_sensor_count": cfg.sensors.hyd_pressure_sensor_count,
        "imu_present": cfg.sensors.imu_present,
        "gps_present": cfg.sensors.gps_present,
        "camera_count": cfg.cameras.count,
        "lora_region": cfg.comm.lora_region,
        "aux_port_count": cfg.aux.port_count,
        "aux_coupler_type": cfg.aux.coupler_type,
        "aux_case_drain_present": cfg.aux.case_drain_present,
    }


def _run_cli(*argv: str) -> tuple[int, str, str]:
    """Invoke ``lifetrac_config.main`` in-process and capture streams."""
    out = io.StringIO()
    err = io.StringIO()
    with redirect_stdout(out), redirect_stderr(err):
        rc = lc.main(list(argv))
    return rc, out.getvalue(), err.getvalue()


class _TmpDirCase(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.tmp = Path(self._tmp.name)
        self.cfg = _bc.load()

    def _write_inv(self, payload) -> Path:
        p = self.tmp / "inventory.json"
        p.write_text(json.dumps(payload), encoding="utf-8")
        return p


class BC12C_A_MatchingInventoryExitsZero(_TmpDirCase):
    def test_text_format_passes(self) -> None:
        inv = self._write_inv(_matching_inventory_dict(self.cfg))
        rc, stdout, stderr = _run_cli("self-test", str(_DEFAULT_TOML), str(inv))
        self.assertEqual(rc, lc.EXIT_OK, msg=f"stderr={stderr!r}")
        self.assertIn("ok             : True", stdout)
        self.assertIn("errors         : 0", stdout)
        # No findings section when the list is empty.
        self.assertNotIn("findings:", stdout)

    def test_json_format_passes(self) -> None:
        inv = self._write_inv(_matching_inventory_dict(self.cfg))
        rc, stdout, _ = _run_cli("self-test", str(_DEFAULT_TOML),
                                 str(inv), "--format", "json")
        self.assertEqual(rc, lc.EXIT_OK)
        payload = json.loads(stdout)
        self.assertTrue(payload["ok"])
        self.assertEqual(payload["error_count"], 0)
        self.assertEqual(payload["warning_count"], 0)
        self.assertEqual(payload["findings"], [])
        self.assertEqual(payload["unit_id"], self.cfg.unit_id)
        self.assertEqual(payload["config_sha256"], self.cfg.config_sha256)


class BC12C_B_ErrorMismatchExitsNonZero(_TmpDirCase):
    def test_axis_count_mismatch_fails(self) -> None:
        d = _matching_inventory_dict(self.cfg)
        d["track_axis_count"] = d["track_axis_count"] + 1
        inv = self._write_inv(d)
        rc, stdout, _ = _run_cli("self-test", str(_DEFAULT_TOML), str(inv))
        self.assertEqual(rc, lc.EXIT_USAGE)
        self.assertIn("ok             : False", stdout)
        self.assertIn("AXIS_COUNT_TRACK", stdout)
        self.assertIn("[error  ]", stdout)

    def test_axis_count_mismatch_fails_json(self) -> None:
        d = _matching_inventory_dict(self.cfg)
        d["arm_axis_count"] = d["arm_axis_count"] + 1
        inv = self._write_inv(d)
        rc, stdout, _ = _run_cli("self-test", str(_DEFAULT_TOML),
                                 str(inv), "--format", "json")
        self.assertEqual(rc, lc.EXIT_USAGE)
        payload = json.loads(stdout)
        self.assertFalse(payload["ok"])
        self.assertGreaterEqual(payload["error_count"], 1)
        codes = [f["code"] for f in payload["findings"]]
        self.assertIn("AXIS_COUNT_ARM", codes)
        # Each finding carries severity / expected / observed.
        bad = next(f for f in payload["findings"] if f["code"] == "AXIS_COUNT_ARM")
        self.assertEqual(bad["severity"], "error")
        self.assertIn("expected", bad)
        self.assertIn("observed", bad)


class BC12C_C_WarningOnlyKeepsExitZero(_TmpDirCase):
    def test_camera_count_short_warns_but_passes(self) -> None:
        d = _matching_inventory_dict(self.cfg)
        # CAMERA_COUNT_SHORT/EXTRA are warnings per Round 39 contract.
        d["camera_count"] = max(0, d["camera_count"] - 1) if d["camera_count"] > 0 else d["camera_count"] + 1
        # Force a guaranteed mismatch:
        if d["camera_count"] == _matching_inventory_dict(self.cfg)["camera_count"]:
            d["camera_count"] += 1  # extra camera also a warning
        inv = self._write_inv(d)
        rc, stdout, _ = _run_cli("self-test", str(_DEFAULT_TOML),
                                 str(inv), "--format", "json")
        self.assertEqual(rc, lc.EXIT_OK,
                         msg="warning-only findings must not flip exit code")
        payload = json.loads(stdout)
        self.assertTrue(payload["ok"])
        self.assertGreaterEqual(payload["warning_count"], 1)
        self.assertEqual(payload["error_count"], 0)
        warn_codes = [f["code"] for f in payload["findings"]
                      if f["severity"] == "warning"]
        self.assertTrue(any(c.startswith("CAMERA_COUNT_") for c in warn_codes))


class BC12C_D_BadInputsRejected(_TmpDirCase):
    def test_missing_inventory_file(self) -> None:
        rc, _, stderr = _run_cli("self-test", str(_DEFAULT_TOML),
                                 str(self.tmp / "nope.json"))
        self.assertEqual(rc, lc.EXIT_USAGE)
        self.assertIn("inventory file not found", stderr)

    def test_inventory_not_an_object(self) -> None:
        bad = self.tmp / "list.json"
        bad.write_text("[1, 2, 3]", encoding="utf-8")
        rc, _, stderr = _run_cli("self-test", str(_DEFAULT_TOML), str(bad))
        self.assertEqual(rc, lc.EXIT_USAGE)
        self.assertIn("top-level object", stderr)

    def test_inventory_missing_fields(self) -> None:
        bad = self.tmp / "partial.json"
        bad.write_text(json.dumps({"track_axis_count": 2}), encoding="utf-8")
        rc, _, stderr = _run_cli("self-test", str(_DEFAULT_TOML), str(bad))
        self.assertEqual(rc, lc.EXIT_USAGE)
        self.assertIn("inventory shape mismatch", stderr)

    def test_inventory_unparseable(self) -> None:
        bad = self.tmp / "garbage.json"
        bad.write_text("{not valid json", encoding="utf-8")
        rc, _, stderr = _run_cli("self-test", str(_DEFAULT_TOML), str(bad))
        self.assertEqual(rc, lc.EXIT_USAGE)
        self.assertIn("cannot parse inventory JSON", stderr)


class BC12C_E_SourceContainsSubcommand(unittest.TestCase):
    def test_self_test_registered(self) -> None:
        # Tripwire: a refactor that drops the subparser registration
        # fails this gate even if test_*_CLI tests get accidentally
        # mocked-out.
        src = (_TOOLS / "lifetrac_config.py").read_text(encoding="utf-8")
        self.assertIn('"self-test"', src)
        self.assertIn("def cmd_self_test", src)
        self.assertIn("set_defaults(func=cmd_self_test)", src)


if __name__ == "__main__":
    unittest.main()
