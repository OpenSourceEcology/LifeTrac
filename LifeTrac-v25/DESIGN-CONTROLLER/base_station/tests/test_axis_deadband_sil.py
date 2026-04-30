"""Round 52 / BC-29 (K-D3): SIL coverage for configurable axis deadband.

The build-config leaf ``ui.axis_deadband`` is an integer (range
``0..32``) that supplies the firmware ``AXIS_DEADBAND`` constant via
the codegen-emitted header. ``13`` (default \u2248 10% of int8 full
scale) is byte-for-byte identity vs the pre-Round-52 behaviour. A
build can widen for jittery operators or shrink for fine arms work.

The deadband is consumed inside the M7 firmware in four places:
``axis_active()`` (per-axis activity test used by the BC-21 mixed-
mode-skip rule), the per-coil activation block (drive LF/LR/RF/RR,
boom up/down, bucket curl/dump), the BC-24 spin-turn detection, and
the flow-set-point computation. All four sites read the same
constant, so a single macro substitution keeps them in sync.

Test classes:

* ``DB_A_SchemaTests`` \u2014 schema accepts in-range integers, rejects
  out-of-range / wrong-type / missing-leaf cases, and pins the
  required-list update.
* ``DB_B_DefaultIdentityTests`` \u2014 ``build.default.toml`` carries
  ``axis_deadband = 13`` and the loaded config exposes it as the int
  13. Codegen emits ``LIFETRAC_UI_AXIS_DEADBAND 13`` for the default.
* ``DB_C_OverrideRoundTripTests`` \u2014 a temp toml that authors
  ``axis_deadband = 5`` round-trips through ``build_config.load()``
  and the codegen header emits ``LIFETRAC_UI_AXIS_DEADBAND 5``;
  similarly for the upper-bound value 32.
* ``DB_D_FirmwareTripwireTests`` \u2014 firmware contains the BC-29
  marker, sources ``AXIS_DEADBAND`` from the macro (no hard-coded
  ``= 13`` literal anywhere), and continues to reference the
  constant in ``axis_active()``, the per-coil activation block,
  BC-24 spin-turn detection, and the flow-set-point computation.
* ``DB_E_DocsTripwireTests`` \u2014 schema, default toml, and
  CAPABILITY_INVENTORY all carry the new leaf.

Pure stdlib.
"""

from __future__ import annotations

import os
import pathlib
import sys
import tempfile
import unittest

_THIS = pathlib.Path(__file__).resolve()
_BS = _THIS.parents[1]
sys.path.insert(0, str(_BS))

import build_config  # noqa: E402
import build_config_codegen  # noqa: E402

_DEFAULT_TOML = _BS / "config" / "build.default.toml"
_SCHEMA_PATH = _BS / "config" / "build_config.schema.json"
_FIRMWARE_PATH = (_THIS.parents[3] / "DESIGN-CONTROLLER"
                  / "firmware" / "tractor_h7" / "tractor_h7.ino")
_INVENTORY_PATH = _THIS.parents[3] / "DESIGN-CONTROLLER" / "CAPABILITY_INVENTORY.md"


def _write_temp_toml(replacements: list[tuple[str, str]]) -> pathlib.Path:
    text = _DEFAULT_TOML.read_text(encoding="utf-8")
    for old, new in replacements:
        if old not in text:
            raise AssertionError(f"replacement target not found: {old!r}")
        text = text.replace(old, new)
    tf = tempfile.NamedTemporaryFile(
        "w", suffix=".toml", delete=False, encoding="utf-8")
    tf.write(text)
    tf.close()
    return pathlib.Path(tf.name)


class _EnvIsolated(unittest.TestCase):
    """Base class: scrubs LIFETRAC_BUILD_CONFIG_PATH around each test."""

    def setUp(self) -> None:
        self._prev = os.environ.pop("LIFETRAC_BUILD_CONFIG_PATH", None)
        self._tmp_files: list[pathlib.Path] = []

    def tearDown(self) -> None:
        os.environ.pop("LIFETRAC_BUILD_CONFIG_PATH", None)
        if self._prev is not None:
            os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = self._prev
        for p in self._tmp_files:
            p.unlink(missing_ok=True)


# ---------------------------------------------------------------------------
# DB-A \u2014 schema validation
# ---------------------------------------------------------------------------


class DB_A_SchemaTests(_EnvIsolated):

    def test_required_list_includes_axis_deadband(self):
        schema = build_config.load_schema()
        ui_required = schema["properties"]["ui"]["required"]
        self.assertIn("axis_deadband", ui_required)

    def test_in_range_values_accepted(self):
        for value in (0, 1, 13, 20, 32):
            with self.subTest(value=value):
                path = _write_temp_toml([
                    ("axis_deadband             = 13",
                     f"axis_deadband             = {value}"),
                ])
                self._tmp_files.append(path)
                os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = str(path)
                cfg = build_config.load()
                self.assertEqual(cfg.ui.axis_deadband, value)

    def test_out_of_range_low_rejected(self):
        path = _write_temp_toml([
            ("axis_deadband             = 13",
             "axis_deadband             = -1"),
        ])
        self._tmp_files.append(path)
        os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = str(path)
        with self.assertRaises(build_config.BuildConfigError):
            build_config.load()

    def test_out_of_range_high_rejected(self):
        path = _write_temp_toml([
            ("axis_deadband             = 13",
             "axis_deadband             = 33"),
        ])
        self._tmp_files.append(path)
        os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = str(path)
        with self.assertRaises(build_config.BuildConfigError):
            build_config.load()

    def test_wrong_type_rejected(self):
        path = _write_temp_toml([
            ("axis_deadband             = 13",
             'axis_deadband             = "13"'),
        ])
        self._tmp_files.append(path)
        os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = str(path)
        with self.assertRaises(build_config.BuildConfigError):
            build_config.load()

    def test_float_rejected(self):
        # Schema demands integer; TOML 13.0 is a different type.
        path = _write_temp_toml([
            ("axis_deadband             = 13",
             "axis_deadband             = 13.0"),
        ])
        self._tmp_files.append(path)
        os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = str(path)
        with self.assertRaises(build_config.BuildConfigError):
            build_config.load()


# ---------------------------------------------------------------------------
# DB-B \u2014 default-value identity
# ---------------------------------------------------------------------------


class DB_B_DefaultIdentityTests(_EnvIsolated):

    def test_default_toml_authors_thirteen(self):
        text = _DEFAULT_TOML.read_text(encoding="utf-8")
        self.assertIn("axis_deadband             = 13", text)

    def test_default_load_yields_int_thirteen(self):
        cfg = build_config.load()
        self.assertEqual(cfg.ui.axis_deadband, 13)
        self.assertIsInstance(cfg.ui.axis_deadband, int)

    def test_default_codegen_emits_thirteen(self):
        cfg = build_config.load()
        text = build_config_codegen.emit_header(cfg)
        self.assertIn("LIFETRAC_UI_AXIS_DEADBAND 13", text)

    def test_default_reload_class_is_restart_required(self):
        classes = build_config.iter_reload_classes()
        self.assertEqual(classes["ui.axis_deadband"], "restart_required")


# ---------------------------------------------------------------------------
# DB-C \u2014 override round-trip
# ---------------------------------------------------------------------------


class DB_C_OverrideRoundTripTests(_EnvIsolated):

    def _emit_for(self, value: int) -> str:
        path = _write_temp_toml([
            ("axis_deadband             = 13",
             f"axis_deadband             = {value}"),
        ])
        self._tmp_files.append(path)
        os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = str(path)
        cfg = build_config.load()
        self.assertEqual(cfg.ui.axis_deadband, value)
        return build_config_codegen.emit_header(cfg)

    def test_low_override_round_trips(self):
        text = self._emit_for(5)
        self.assertIn("LIFETRAC_UI_AXIS_DEADBAND 5", text)
        # And the legacy 13 value must NOT appear in the macro line.
        self.assertNotIn("LIFETRAC_UI_AXIS_DEADBAND 13", text)

    def test_zero_override_round_trips(self):
        text = self._emit_for(0)
        self.assertIn("LIFETRAC_UI_AXIS_DEADBAND 0", text)

    def test_upper_bound_override_round_trips(self):
        text = self._emit_for(32)
        self.assertIn("LIFETRAC_UI_AXIS_DEADBAND 32", text)


# ---------------------------------------------------------------------------
# DB-D \u2014 firmware tripwires
# ---------------------------------------------------------------------------


class DB_D_FirmwareTripwireTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        cls.fw = _FIRMWARE_PATH.read_text(encoding="utf-8")

    def test_bc29_marker_present(self):
        self.assertIn("BC-29", self.fw)

    def test_axis_deadband_sourced_from_macro(self):
        # The const init line must read from the codegen macro \u2014 a
        # regression that hard-codes 13 again would silently divorce
        # firmware from the schema.
        self.assertIn("LIFETRAC_UI_AXIS_DEADBAND", self.fw)
        self.assertIn(
            "static const int8_t AXIS_DEADBAND  = (int8_t)LIFETRAC_UI_AXIS_DEADBAND;",
            self.fw,
            "AXIS_DEADBAND must be initialised from the codegen macro")

    def test_no_legacy_hard_coded_thirteen(self):
        # The pre-Round-52 line ``static const int8_t AXIS_DEADBAND  = 13;``
        # must be gone. Match a tight pattern so an unrelated literal
        # ``13`` elsewhere in the firmware doesn't false-positive.
        self.assertNotIn(
            "static const int8_t AXIS_DEADBAND  = 13;", self.fw)
        self.assertNotIn(
            "static const int8_t AXIS_DEADBAND = 13;", self.fw)

    def test_axis_deadband_still_consumed_in_axis_active(self):
        self.assertIn("(v > AXIS_DEADBAND) || (v < -AXIS_DEADBAND)", self.fw)

    def test_axis_deadband_still_consumed_in_coil_block(self):
        # Pin the four track / two arm / two bucket activation patterns.
        for pattern in (
            "left_track  >  AXIS_DEADBAND",
            "left_track  < -AXIS_DEADBAND",
            "right_track >  AXIS_DEADBAND",
            "right_track < -AXIS_DEADBAND",
            "arms        >  AXIS_DEADBAND",
            "arms        < -AXIS_DEADBAND",
        ):
            with self.subTest(pattern=pattern):
                self.assertIn(pattern, self.fw)

    def test_axis_deadband_still_consumed_in_spin_turn_detect(self):
        self.assertIn(
            "left_track  >  AXIS_DEADBAND && right_track < -AXIS_DEADBAND",
            self.fw)

    def test_axis_deadband_still_consumed_in_flow_setpoint(self):
        self.assertIn("if (mag > AXIS_DEADBAND)", self.fw)
        self.assertIn("(mag - AXIS_DEADBAND)", self.fw)


# ---------------------------------------------------------------------------
# DB-E \u2014 docs / schema tripwires
# ---------------------------------------------------------------------------


class DB_E_DocsTripwireTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        cls.schema = _SCHEMA_PATH.read_text(encoding="utf-8")
        cls.toml = _DEFAULT_TOML.read_text(encoding="utf-8")
        cls.inv = _INVENTORY_PATH.read_text(encoding="utf-8")

    def test_schema_documents_new_leaf(self):
        self.assertIn("axis_deadband", self.schema)
        # Pin range bounds so a regression that loosens them trips here.
        self.assertIn('"minimum": 0, "maximum": 32', self.schema)

    def test_default_toml_carries_leaf_and_doc_comment(self):
        self.assertIn("axis_deadband             = 13", self.toml)
        self.assertIn("BC-29", self.toml)

    def test_inventory_documents_new_leaf(self):
        self.assertIn("ui.axis_deadband", self.inv)


if __name__ == "__main__":
    unittest.main()
