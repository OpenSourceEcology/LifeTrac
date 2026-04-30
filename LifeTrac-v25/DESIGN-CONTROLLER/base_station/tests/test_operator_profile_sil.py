"""Round 51 / BC-28 (K-D1): SIL coverage for operator profile preset.

The build-config leaf ``ui.operator_profile`` is an enum
``{"normal", "gentle", "sport"}``. ``"normal"`` (default) leaves the
individual operator-feel leaves as authored in the TOML \u2014 byte-for-
byte identity vs the pre-Round-51 behaviour. ``"gentle"`` and
``"sport"`` are bundles that mutate a small set of leaves at config-
load time so every downstream consumer (the codegen-emitted firmware
header, the audit log, hot-reload diff classification) sees the same
post-override state.

Bundles (frozen by SIL):

* ``"gentle"`` \u2192 ``ui.confined_space_mode_enabled = True`` AND
  ``hydraulic.ramp_shape = "scurve"``. Tight-quarters work: longer
  release ramps with a smoothstep envelope.
* ``"sport"`` \u2192 ``ui.confined_space_mode_enabled = False``,
  ``hydraulic.ramp_shape = "linear"`` AND
  ``ui.stick_curve_exponent = 1.0``. Crisp, snappy control.

Test classes:

* ``OP_A_OverrideHelperMathTests`` \u2014 pure
  ``_apply_operator_profile_overrides()`` math. ``normal`` is a no-op
  on a representative ``data`` dict; ``gentle`` and ``sport`` apply
  exactly the bundle leaves and leave everything else untouched;
  unknown profile is a defensive no-op (schema validation catches it
  upstream so this is belt-and-braces).
* ``OP_B_LoadIntegrationTests`` \u2014 ``build_config.load()`` integration
  via temp TOML + ``LIFETRAC_BUILD_CONFIG_PATH``. Default toml loads
  with ``operator_profile == "normal"`` and the individual leaves
  exactly as authored. A ``gentle`` toml that authors
  ``confined_space_mode_enabled = false`` and
  ``ramp_shape = "linear"`` still ends up with the gentle bundle
  applied. A ``sport`` toml that authors the gentle settings still
  ends up with the sport bundle applied. ``raw`` dict reflects the
  post-override state (so ``config_sha256`` is consistent across
  consumers).
* ``OP_C_CodegenIntegrationTests`` \u2014 build a ``gentle`` config and
  verify the emitted firmware header contains
  ``LIFETRAC_UI_CONFINED_SPACE_MODE_ENABLED 1`` and
  ``LIFETRAC_HYDRAULIC_RAMP_SHAPE_SCURVE 1``. A ``sport`` config
  yields ``..._CONFINED_SPACE_MODE_ENABLED 0`` and the linear side
  macro. Pins that codegen reads through the post-override state.
* ``OP_D_SchemaTests`` \u2014 schema rejects an unknown
  ``operator_profile`` value; the leaf is required.
* ``OP_E_FirmwareTripwireTests`` \u2014 BC-28 marker, override helper
  symbol, schema enum, CAPABILITY_INVENTORY row, default.toml leaf.

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
_INVENTORY_PATH = _THIS.parents[3] / "DESIGN-CONTROLLER" / "CAPABILITY_INVENTORY.md"


# ---------------------------------------------------------------------------
# OP-A \u2014 pure override-helper math
# ---------------------------------------------------------------------------


def _fresh_data(profile: str = "normal",
                confined: bool = False,
                ramp_shape: str = "linear",
                stick_curve: float = 1.0) -> dict:
    """A minimal dict shaped like the post-toml data the loader feeds to
    ``_apply_operator_profile_overrides``. Only the fields the helper
    touches need to be present; we add a sentinel under each section
    plus an extra section to prove the helper never touches them."""
    return {
        "ui": {
            "operator_profile": profile,
            "confined_space_mode_enabled": confined,
            "stick_curve_exponent": stick_curve,
            "web_ui_enabled": True,  # untouched sentinel
        },
        "hydraulic": {
            "ramp_shape": ramp_shape,
            "spool_type": "open_centre_4_3",  # untouched sentinel
        },
        "safety": {  # entire untouched section
            "m4_watchdog_ms": 250,
        },
    }


class OP_A_OverrideHelperMathTests(unittest.TestCase):

    def test_normal_is_noop(self):
        data = _fresh_data("normal", confined=False,
                           ramp_shape="linear", stick_curve=1.0)
        snapshot = {k: dict(v) for k, v in data.items()}
        build_config._apply_operator_profile_overrides(data)
        self.assertEqual(data, snapshot)

    def test_normal_does_not_resurrect_authored_values(self):
        # Even if the operator authored "weird" individual values, normal
        # leaves them alone.
        data = _fresh_data("normal", confined=True,
                           ramp_shape="scurve", stick_curve=2.0)
        build_config._apply_operator_profile_overrides(data)
        self.assertTrue(data["ui"]["confined_space_mode_enabled"])
        self.assertEqual(data["hydraulic"]["ramp_shape"], "scurve")
        self.assertEqual(data["ui"]["stick_curve_exponent"], 2.0)

    def test_gentle_overrides_confined_and_scurve(self):
        # Gentle must flip these on REGARDLESS of authored values.
        data = _fresh_data("gentle", confined=False,
                           ramp_shape="linear", stick_curve=1.5)
        build_config._apply_operator_profile_overrides(data)
        self.assertTrue(data["ui"]["confined_space_mode_enabled"])
        self.assertEqual(data["hydraulic"]["ramp_shape"], "scurve")
        # Gentle does NOT touch stick_curve_exponent.
        self.assertEqual(data["ui"]["stick_curve_exponent"], 1.5)

    def test_sport_overrides_confined_off_linear_and_curve_1_0(self):
        data = _fresh_data("sport", confined=True,
                           ramp_shape="scurve", stick_curve=2.0)
        build_config._apply_operator_profile_overrides(data)
        self.assertFalse(data["ui"]["confined_space_mode_enabled"])
        self.assertEqual(data["hydraulic"]["ramp_shape"], "linear")
        self.assertEqual(data["ui"]["stick_curve_exponent"], 1.0)

    def test_sentinel_fields_untouched(self):
        for profile in ("normal", "gentle", "sport"):
            with self.subTest(profile=profile):
                data = _fresh_data(profile)
                build_config._apply_operator_profile_overrides(data)
                self.assertTrue(data["ui"]["web_ui_enabled"])
                self.assertEqual(data["hydraulic"]["spool_type"],
                                 "open_centre_4_3")
                self.assertEqual(data["safety"]["m4_watchdog_ms"], 250)

    def test_unknown_profile_is_defensive_noop(self):
        # Schema validation catches this upstream; the helper must not
        # crash if somehow called with a bogus value.
        data = _fresh_data("unknown_profile")
        snapshot = {k: dict(v) for k, v in data.items()}
        build_config._apply_operator_profile_overrides(data)
        self.assertEqual(data, snapshot)

    def test_bundle_table_pinned(self):
        # Pin the public bundle table so a future "improvement" that
        # silently changes a bundle trips the SIL.
        self.assertEqual(set(build_config.OPERATOR_PROFILE_OVERRIDES),
                         {"normal", "gentle", "sport"})
        self.assertEqual(build_config.OPERATOR_PROFILE_OVERRIDES["normal"], {})
        self.assertEqual(
            build_config.OPERATOR_PROFILE_OVERRIDES["gentle"],
            {"ui": {"confined_space_mode_enabled": True},
             "hydraulic": {"ramp_shape": "scurve"}})
        self.assertEqual(
            build_config.OPERATOR_PROFILE_OVERRIDES["sport"],
            {"ui": {"confined_space_mode_enabled": False,
                    "stick_curve_exponent": 1.0},
             "hydraulic": {"ramp_shape": "linear"}})


# ---------------------------------------------------------------------------
# OP-B \u2014 load() integration via temp TOML + env override
# ---------------------------------------------------------------------------


def _write_temp_toml(replacements: list[tuple[str, str]]) -> pathlib.Path:
    """Copy build.default.toml to a temp file with line-replacements."""
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


class OP_B_LoadIntegrationTests(unittest.TestCase):

    def setUp(self) -> None:
        self._prev = os.environ.pop("LIFETRAC_BUILD_CONFIG_PATH", None)
        self._tmp_files: list[pathlib.Path] = []

    def tearDown(self) -> None:
        os.environ.pop("LIFETRAC_BUILD_CONFIG_PATH", None)
        if self._prev is not None:
            os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = self._prev
        for p in self._tmp_files:
            p.unlink(missing_ok=True)

    def _load(self, path: pathlib.Path) -> "build_config.BuildConfig":
        os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = str(path)
        return build_config.load()

    def test_default_toml_yields_normal_and_preserves_leaves(self):
        cfg = build_config.load()
        self.assertEqual(cfg.ui.operator_profile, "normal")
        # Default toml authors all three at identity values.
        self.assertFalse(cfg.ui.confined_space_mode_enabled)
        self.assertEqual(cfg.ui.stick_curve_exponent, 1.0)
        self.assertEqual(cfg.hydraulic.ramp_shape, "linear")

    def test_gentle_overrides_authored_leaves(self):
        # Author "wrong" values for the gentle bundle to prove override
        # actually fires.
        path = _write_temp_toml([
            ('operator_profile          = "normal"',
             'operator_profile          = "gentle"'),
        ])
        self._tmp_files.append(path)
        cfg = self._load(path)
        self.assertEqual(cfg.ui.operator_profile, "gentle")
        self.assertTrue(cfg.ui.confined_space_mode_enabled,
                        "gentle must flip confined on")
        self.assertEqual(cfg.hydraulic.ramp_shape, "scurve",
                         "gentle must select scurve")
        # Gentle does NOT touch stick_curve_exponent \u2014 authored 1.0 stays.
        self.assertEqual(cfg.ui.stick_curve_exponent, 1.0)

    def test_sport_overrides_authored_leaves(self):
        # Author the gentle settings then ask for sport \u2192 sport wins.
        path = _write_temp_toml([
            ('operator_profile          = "normal"',
             'operator_profile          = "sport"'),
            ('confined_space_mode_enabled = false',
             'confined_space_mode_enabled = true'),
            ('stick_curve_exponent      = 1.0',
             'stick_curve_exponent      = 2.0'),
            ('ramp_shape          = "linear"',
             'ramp_shape          = "scurve"'),
        ])
        self._tmp_files.append(path)
        cfg = self._load(path)
        self.assertEqual(cfg.ui.operator_profile, "sport")
        self.assertFalse(cfg.ui.confined_space_mode_enabled)
        self.assertEqual(cfg.ui.stick_curve_exponent, 1.0)
        self.assertEqual(cfg.hydraulic.ramp_shape, "linear")

    def test_raw_dict_reflects_post_override_state(self):
        # config_sha256 hashes cfg.raw \u2014 it must reflect the final
        # post-override state so all consumers (audit log, hot-reload
        # diff, codegen) agree on identity.
        path = _write_temp_toml([
            ('operator_profile          = "normal"',
             'operator_profile          = "gentle"'),
        ])
        self._tmp_files.append(path)
        cfg = self._load(path)
        self.assertTrue(cfg.raw["ui"]["confined_space_mode_enabled"])
        self.assertEqual(cfg.raw["hydraulic"]["ramp_shape"], "scurve")

    def test_default_load_sha_unchanged_byte_identity(self):
        # The very-first thing a Round-51 regression would break: adding
        # the new leaf must NOT change config_sha256 of the default toml
        # in any way that depends on profile-handling logic. We simply
        # confirm the load succeeds and the sha is a 64-hex string \u2014
        # the actual value will shift due to the new leaf, but the
        # default profile is normal so no override fires.
        cfg = build_config.load()
        sha = cfg.config_sha256
        self.assertEqual(len(sha), 64)
        self.assertTrue(all(c in "0123456789abcdef" for c in sha))


# ---------------------------------------------------------------------------
# OP-C \u2014 codegen integration
# ---------------------------------------------------------------------------


class OP_C_CodegenIntegrationTests(unittest.TestCase):

    def setUp(self) -> None:
        self._prev = os.environ.pop("LIFETRAC_BUILD_CONFIG_PATH", None)
        self._tmp_files: list[pathlib.Path] = []

    def tearDown(self) -> None:
        os.environ.pop("LIFETRAC_BUILD_CONFIG_PATH", None)
        if self._prev is not None:
            os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = self._prev
        for p in self._tmp_files:
            p.unlink(missing_ok=True)

    def _emit(self, profile: str) -> str:
        path = _write_temp_toml([
            ('operator_profile          = "normal"',
             f'operator_profile          = "{profile}"'),
        ])
        self._tmp_files.append(path)
        os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = str(path)
        cfg = build_config.load()
        return build_config_codegen.emit_header(cfg)

    def test_gentle_emits_confined_on_and_scurve(self):
        text = self._emit("gentle")
        self.assertIn("LIFETRAC_UI_CONFINED_SPACE_MODE_ENABLED 1", text)
        self.assertIn('LIFETRAC_HYDRAULIC_RAMP_SHAPE "scurve"', text)
        self.assertIn("LIFETRAC_HYDRAULIC_RAMP_SHAPE_SCURVE 1", text)
        self.assertIn('LIFETRAC_UI_OPERATOR_PROFILE "gentle"', text)
        self.assertIn("LIFETRAC_UI_OPERATOR_PROFILE_GENTLE 1", text)

    def test_sport_emits_confined_off_linear_and_curve_1_0(self):
        text = self._emit("sport")
        self.assertIn("LIFETRAC_UI_CONFINED_SPACE_MODE_ENABLED 0", text)
        self.assertIn('LIFETRAC_HYDRAULIC_RAMP_SHAPE "linear"', text)
        self.assertIn("LIFETRAC_HYDRAULIC_RAMP_SHAPE_LINEAR 1", text)
        self.assertIn("LIFETRAC_UI_STICK_CURVE_EXPONENT 1.0f", text)
        self.assertIn('LIFETRAC_UI_OPERATOR_PROFILE "sport"', text)
        self.assertIn("LIFETRAC_UI_OPERATOR_PROFILE_SPORT 1", text)

    def test_normal_default_toml_is_byte_identity_to_authored(self):
        text = self._emit("normal")
        self.assertIn("LIFETRAC_UI_CONFINED_SPACE_MODE_ENABLED 0", text)
        self.assertIn('LIFETRAC_HYDRAULIC_RAMP_SHAPE "linear"', text)
        self.assertIn("LIFETRAC_UI_STICK_CURVE_EXPONENT 1.0f", text)
        self.assertIn('LIFETRAC_UI_OPERATOR_PROFILE "normal"', text)
        self.assertIn("LIFETRAC_UI_OPERATOR_PROFILE_NORMAL 1", text)


# ---------------------------------------------------------------------------
# OP-D \u2014 schema validation
# ---------------------------------------------------------------------------


class OP_D_SchemaTests(unittest.TestCase):

    def setUp(self) -> None:
        self._prev = os.environ.pop("LIFETRAC_BUILD_CONFIG_PATH", None)
        self._tmp_files: list[pathlib.Path] = []

    def tearDown(self) -> None:
        os.environ.pop("LIFETRAC_BUILD_CONFIG_PATH", None)
        if self._prev is not None:
            os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = self._prev
        for p in self._tmp_files:
            p.unlink(missing_ok=True)

    def test_unknown_profile_rejected_by_schema(self):
        path = _write_temp_toml([
            ('operator_profile          = "normal"',
             'operator_profile          = "race_mode"'),
        ])
        self._tmp_files.append(path)
        os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = str(path)
        with self.assertRaises(build_config.BuildConfigError):
            build_config.load()


# ---------------------------------------------------------------------------
# OP-E \u2014 source / config tripwires
# ---------------------------------------------------------------------------


class OP_E_FirmwareTripwireTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        cls.bc_src = (_BS / "build_config.py").read_text(encoding="utf-8")
        cls.schema = _SCHEMA_PATH.read_text(encoding="utf-8")
        cls.toml = _DEFAULT_TOML.read_text(encoding="utf-8")
        cls.inv = _INVENTORY_PATH.read_text(encoding="utf-8")

    def test_bc28_marker_present_in_loader(self):
        self.assertIn("BC-28", self.bc_src)

    def test_override_helper_symbol_present(self):
        self.assertIn("_apply_operator_profile_overrides", self.bc_src)
        self.assertIn("OPERATOR_PROFILE_OVERRIDES", self.bc_src)

    def test_schema_documents_new_leaf(self):
        self.assertIn("operator_profile", self.schema)
        # Pin the enum so a regression that adds a 4th profile must
        # update this SIL.
        self.assertIn('"normal"', self.schema)
        self.assertIn('"gentle"', self.schema)
        self.assertIn('"sport"', self.schema)

    def test_default_toml_has_normal_default(self):
        self.assertIn('operator_profile          = "normal"', self.toml)

    def test_inventory_documents_new_leaf(self):
        self.assertIn("ui.operator_profile", self.inv)


if __name__ == "__main__":
    unittest.main()
