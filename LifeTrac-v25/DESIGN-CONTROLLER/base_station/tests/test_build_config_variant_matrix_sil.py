"""SIL gate for Round 32 -- IP: BC-07 (variant-matrix end-to-end).

BC-07 was scoped as: a SIL exercising 4 representative configurations
(canonical / no-camera / no-IMU+GPS / single-axis) end-to-end through
``web_ui`` and ``lora_bridge`` with ``LIFETRAC_BUILD_CONFIG_PATH``
pointed at fixture TOMLs. The point is to catch any consumer that
silently uses a hard-coded constant (``MAX_CONTROL_SUBSCRIBERS = 4``,
``CAMERA_FRONT = 1``, ``TRACK_AXIS_COUNT = 2``, etc.) instead of the
loaded ``BuildConfig`` -- a class of bug that BC-04 closed but that
only the canonical TOML exercises today.

The four variants (synthesised in :func:`_variant_toml` from the
canonical default) are kept narrow and orthogonal so a regression on
any one consumer surface points at exactly one variant:

* **canonical** -- baseline (identical to ``build.default.toml``).
* **no-camera** -- ``cameras.count = 0`` + every ``*_present = false``;
  the camera-select endpoint must have an empty ``_CAMERA_IDS``
  table; preview-diff against canonical lights up the four camera
  leaves as live changes; codegen emits ``LIFETRAC_CAMERAS_COUNT 0``
  + every ``*_PRESENT 0``.
* **no-imu-gps** -- ``imu_present = false`` + ``gps_present = false``
  + ``hyd_pressure_sensor_count = 0``; `_CAMERA_IDS` still populated;
  codegen emits the present-flags as ``0`` and keeps the model
  strings (re-enabling the IMU later just needs a present-flip).
* **single-axis** -- ``track_axis_count = 1`` + ``arm_axis_count = 0``
  + ``proportional_flow = false`` + ``track_ramp_seconds = 0.5``;
  classic Microtrac shape; verifies the loader accepts the schema
  minimums and codegen emits the smaller numbers verbatim.

Every variant is round-tripped through:

1. **Loader** -- ``build_config.load()`` with ``LIFETRAC_BUILD_CONFIG_PATH``
   pointed at the fixture; assert no exception, the dataclass values
   match the override.
2. **web_ui module-import** -- assert ``BUILD`` mirrors the variant,
   ``MAX_CONTROL_SUBSCRIBERS`` reads from ``BUILD.ui``, and the
   camera ID table reflects the variant's ``cameras.*_present``.
3. **Codegen** -- emit the C header against the variant; assert the
   bool / int leaves landed at the variant's values; assert the SHA
   in the header matches ``cfg.config_sha256``.
4. **Diff against canonical** -- ``diff_reload_classes(canonical,
   variant)`` returns the expected leaf set + worst reload class.

A final cross-cutting case asserts the variant SHAs are pairwise
distinct so ``config_loaded`` audit entries can disambiguate the four
fleet shapes from a single log file.
"""

from __future__ import annotations

import importlib
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


_BS = Path(__file__).resolve().parents[1]
_DEFAULT_TOML = _BS / "config" / "build.default.toml"

if str(_BS) not in sys.path:
    sys.path.insert(0, str(_BS))


# ------------------------------------------------------------- variant table
# Each variant is expressed as a list of (find, replace) pairs against
# the canonical default. We use string substitution rather than a TOML
# AST so the alignment stays canonical and the fixture stays diff-able
# against the default. ``_assert_overrides_landed`` catches a typo in
# the find side that would otherwise produce a no-op.

_NO_CAMERA: tuple[tuple[str, str], ...] = (
    ("count                = 1",                  "count                = 0"),
    ("front_present        = true",               "front_present        = false"),
)

_NO_IMU_GPS: tuple[tuple[str, str], ...] = (
    ("imu_present                = true",         "imu_present                = false"),
    ("gps_present                = true",         "gps_present                = false"),
    ("hyd_pressure_sensor_count  = 2",            "hyd_pressure_sensor_count  = 0"),
)

_SINGLE_AXIS: tuple[tuple[str, str], ...] = (
    ("track_axis_count    = 2",                   "track_axis_count    = 1"),
    ("arm_axis_count      = 2",                   "arm_axis_count      = 0"),
    ("proportional_flow   = true",                "proportional_flow   = false"),
    ("track_ramp_seconds  = 2.0",                 "track_ramp_seconds  = 0.5"),
)

_VARIANTS: dict[str, tuple[tuple[str, str], ...]] = {
    "canonical":   (),
    "no_camera":   _NO_CAMERA,
    "no_imu_gps":  _NO_IMU_GPS,
    "single_axis": _SINGLE_AXIS,
}


def _variant_toml(tmpdir: Path, name: str) -> Path:
    """Write the named variant's TOML into ``tmpdir`` and return its path."""
    text = _DEFAULT_TOML.read_text(encoding="utf-8")
    overrides = _VARIANTS[name]
    for old, new in overrides:
        if old not in text:
            raise AssertionError(
                f"variant {name!r}: override anchor {old!r} not found in default "
                f"-- canonical TOML alignment changed?"
            )
        if old == new:
            raise AssertionError(f"variant {name!r}: no-op override {old!r}")
        text = text.replace(old, new)
    out = tmpdir / f"build.{name}.toml"
    out.write_text(text, encoding="utf-8")
    return out


def _import_web_ui(toml_path: Path):
    """Drop cached modules and import web_ui pinned to ``toml_path``."""
    for mod in ("web_ui", "build_config"):
        sys.modules.pop(mod, None)
    env = {
        "LIFETRAC_BUILD_CONFIG_PATH": str(toml_path),
        "LIFETRAC_ALLOW_UNCONFIGURED_KEY": "1",
        "LIFETRAC_PIN": "1234",
    }
    with mock.patch.dict(os.environ, env, clear=False), \
            mock.patch("paho.mqtt.client.Client") as cls:
        instance = mock.MagicMock()
        instance.connect.return_value = None
        cls.return_value = instance
        return importlib.import_module("web_ui")


def _import_loader(toml_path: Path):
    sys.modules.pop("build_config", None)
    os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = str(toml_path)
    return importlib.import_module("build_config")


# ------------------------------------------------------------- shared setup


class _VariantBase(unittest.TestCase):
    """Materialise all four variants once per test method."""

    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="lt-bc07-"))
        self.paths: dict[str, Path] = {
            name: _variant_toml(self.tmp, name) for name in _VARIANTS
        }


# ------------------------------------------------------------- BC07_A


class BC07_A_LoaderAcceptsEveryVariant(_VariantBase):

    def test_no_camera(self) -> None:
        bc = _import_loader(self.paths["no_camera"])
        cfg = bc.load()
        self.assertEqual(cfg.cameras.count, 0)
        self.assertFalse(cfg.cameras.front_present)
        self.assertFalse(cfg.cameras.rear_present)

    def test_no_imu_gps(self) -> None:
        bc = _import_loader(self.paths["no_imu_gps"])
        cfg = bc.load()
        self.assertFalse(cfg.sensors.imu_present)
        self.assertFalse(cfg.sensors.gps_present)
        self.assertEqual(cfg.sensors.hyd_pressure_sensor_count, 0)

    def test_single_axis(self) -> None:
        bc = _import_loader(self.paths["single_axis"])
        cfg = bc.load()
        self.assertEqual(cfg.hydraulic.track_axis_count, 1)
        self.assertEqual(cfg.hydraulic.arm_axis_count, 0)
        self.assertFalse(cfg.hydraulic.proportional_flow)
        self.assertAlmostEqual(cfg.hydraulic.track_ramp_seconds, 0.5)

    def test_canonical_baseline_loads(self) -> None:
        bc = _import_loader(self.paths["canonical"])
        cfg = bc.load()
        # Canonical baseline must match the canonical default's SHA
        # exactly -- our test fixture is byte-identical to the default.
        self.assertEqual(
            self.paths["canonical"].read_text(encoding="utf-8"),
            _DEFAULT_TOML.read_text(encoding="utf-8"),
        )


# ------------------------------------------------------------- BC07_B


class BC07_B_WebUiConsumesVariant(_VariantBase):

    def test_no_camera_collapses_camera_table(self) -> None:
        web_ui = _import_web_ui(self.paths["no_camera"])
        # _CAMERA_IDS has one base entry ("auto" -> 0). Every
        # *_present-gated entry must be filtered out.
        self.assertNotIn("front", web_ui._CAMERA_IDS)
        self.assertNotIn("rear", web_ui._CAMERA_IDS)
        self.assertNotIn("implement", web_ui._CAMERA_IDS)
        self.assertNotIn("crop", web_ui._CAMERA_IDS)

    def test_canonical_camera_table(self) -> None:
        web_ui = _import_web_ui(self.paths["canonical"])
        # Default has front_present=true only.
        self.assertIn("front", web_ui._CAMERA_IDS)
        self.assertNotIn("rear", web_ui._CAMERA_IDS)

    def test_single_axis_max_subscribers_unchanged(self) -> None:
        # single_axis variant doesn't touch ui.max_control_subscribers,
        # so MAX_CONTROL_SUBSCRIBERS must equal the canonical default
        # (and NOT a hard-coded constant).
        web_ui = _import_web_ui(self.paths["single_axis"])
        self.assertEqual(web_ui.MAX_CONTROL_SUBSCRIBERS,
                         web_ui.BUILD.ui.max_control_subscribers)
        self.assertEqual(web_ui.BUILD.hydraulic.track_axis_count, 1)


# ------------------------------------------------------------- BC07_C


class BC07_C_CodegenForEveryVariant(_VariantBase):

    def _emit(self, path: Path) -> tuple[str, object]:
        bc = _import_loader(path)
        sys.modules.pop("build_config_codegen", None)
        cg = importlib.import_module("build_config_codegen")
        cfg = bc.load()
        return cg.emit_header(cfg), cfg

    def test_no_camera_header_zeros_present_flags(self) -> None:
        text, cfg = self._emit(self.paths["no_camera"])
        self.assertIn("#define LIFETRAC_CAMERAS_COUNT 0", text)
        self.assertIn("#define LIFETRAC_CAMERAS_FRONT_PRESENT 0", text)
        self.assertIn("#define LIFETRAC_CAMERAS_REAR_PRESENT 0", text)
        self.assertIn(f'"{cfg.config_sha256}"', text)

    def test_no_imu_gps_header_zeros_present_flags(self) -> None:
        text, _ = self._emit(self.paths["no_imu_gps"])
        self.assertIn("#define LIFETRAC_SENSORS_IMU_PRESENT 0", text)
        self.assertIn("#define LIFETRAC_SENSORS_GPS_PRESENT 0", text)
        self.assertIn("#define LIFETRAC_SENSORS_HYD_PRESSURE_SENSOR_COUNT 0", text)
        # Model strings must persist (re-enable later by flipping
        # present back to true; we don't want a flag-day rename).
        self.assertIn('#define LIFETRAC_SENSORS_IMU_MODEL "bno086"', text)

    def test_single_axis_header_carries_smaller_numbers(self) -> None:
        text, _ = self._emit(self.paths["single_axis"])
        self.assertIn("#define LIFETRAC_HYDRAULIC_TRACK_AXIS_COUNT 1", text)
        self.assertIn("#define LIFETRAC_HYDRAULIC_ARM_AXIS_COUNT 0", text)
        self.assertIn("#define LIFETRAC_HYDRAULIC_PROPORTIONAL_FLOW 0", text)
        self.assertIn("#define LIFETRAC_HYDRAULIC_TRACK_RAMP_SECONDS 0.5f", text)


# ------------------------------------------------------------- BC07_D


class BC07_D_ReloadClassDiffPerVariant(_VariantBase):

    def _diff(self, variant: str):
        bc = _import_loader(self.paths["canonical"])
        canonical = bc.load()
        bc = _import_loader(self.paths[variant])
        candidate = bc.load()
        return bc.diff_reload_classes(canonical, candidate), canonical, candidate

    def test_canonical_self_diff_empty(self) -> None:
        diff, _, _ = self._diff("canonical")
        self.assertEqual(list(diff.changed), [])
        self.assertTrue(diff.is_empty)

    def test_no_camera_diff_lights_up_camera_leaves_only(self) -> None:
        diff, _, _ = self._diff("no_camera")
        # Every changed leaf must be in the cameras section.
        for leaf in diff.changed:
            self.assertTrue(leaf.startswith("cameras."),
                            msg=f"unexpected non-camera leaf changed: {leaf!r}")
        # Worst class for camera diffs is live (BC-10 annotation).
        self.assertEqual(diff.worst, "live")
        self.assertFalse(diff.firmware_required)

    def test_single_axis_diff_is_restart_required(self) -> None:
        diff, _, _ = self._diff("single_axis")
        # track_axis_count + arm_axis_count carry restart_required;
        # ramp_seconds is live; so worst should bubble to
        # restart_required.
        self.assertEqual(diff.worst, "restart_required")
        self.assertIn("hydraulic.track_axis_count", diff.changed)
        self.assertIn("hydraulic.arm_axis_count", diff.changed)


# ------------------------------------------------------------- BC07_E


class BC07_E_DistinctShasAcrossVariants(_VariantBase):

    def test_pairwise_distinct_shas(self) -> None:
        shas: dict[str, str] = {}
        for name, path in self.paths.items():
            bc = _import_loader(path)
            shas[name] = bc.load().config_sha256
        # All four variants must produce distinct SHAs so a single
        # audit-log line carrying config_sha256 disambiguates the
        # fleet shape that booted.
        self.assertEqual(len(set(shas.values())), len(shas),
                         msg=f"SHA collision: {shas}")
        # And canonical's SHA must equal a fresh load of the default.
        bc = _import_loader(_DEFAULT_TOML)
        self.assertEqual(shas["canonical"], bc.load().config_sha256)


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
