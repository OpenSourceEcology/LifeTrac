"""Build-configuration loader SIL test.

Round 27 -- IP: BC-01 (capability inventory) + BC-02 (loader / schema /
default-TOML round trip).

Pure source-parse + in-process load test. Exercises the loader at
``DESIGN-CONTROLLER/base_station/build_config.py`` against the schema at
``base_station/config/build_config.schema.json`` and the canonical TOML
at ``base_station/config/build.default.toml``, with synthesised TOMLs
written into a temp directory for fault-injection cases.

Defended invariants
-------------------

BC-A (schema well-formedness): the JSON Schema document parses, declares
    object/required/properties, and every nested section is an object
    with ``additionalProperties: false`` so unknown keys are rejected.

BC-B (loader fallback chain): the loader resolves
    ``LIFETRAC_BUILD_CONFIG_PATH`` ahead of per-unit overrides ahead of
    the canonical default, and produces a populated :class:`BuildConfig`
    whose nested dataclasses match the TOML.

BC-C (strict validation): unknown keys, out-of-range integers, wrong
    types, bad enum tokens, and pattern-violating ``unit_id`` strings
    all raise :class:`BuildConfigError`.

BC-D (deterministic config_sha256): the digest is stable across whitespace
    re-orderings of the input TOML, changes when a leaf value changes,
    and matches the explicit canonical-JSON formula.

BC-E (inventory parity): every property declared in the schema appears as
    an ``id`` token in ``DESIGN-CONTROLLER/CAPABILITY_INVENTORY.md``,
    and every ``id`` token in the inventory tables appears in the schema.
    No third-place drift between the loader, the schema, and the doc.
"""

from __future__ import annotations

import hashlib
import importlib
import json
import os
import re
import sys
import unittest
from pathlib import Path
from unittest import mock


_BS = Path(__file__).resolve().parents[1]
_REPO_ROOT = _BS.parents[1]
_SCHEMA_PATH = _BS / "config" / "build_config.schema.json"
_DEFAULT_TOML = _BS / "config" / "build.default.toml"
_INVENTORY = _BS.parent / "CAPABILITY_INVENTORY.md"

if str(_BS) not in sys.path:
    sys.path.insert(0, str(_BS))


def _import_loader():
    if "build_config" in sys.modules:
        return importlib.reload(sys.modules["build_config"])
    return importlib.import_module("build_config")


# ---------------------------------------------------------------- BC_A


class BC_A_SchemaWellFormedness(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        cls.schema = json.loads(_SCHEMA_PATH.read_text(encoding="utf-8"))

    def test_schema_top_level_is_strict_object(self) -> None:
        self.assertEqual(self.schema.get("type"), "object")
        self.assertIs(self.schema.get("additionalProperties"), False)
        self.assertIn("required", self.schema)
        self.assertIn("properties", self.schema)

    def test_every_nested_section_is_strict(self) -> None:
        for name, sub in self.schema["properties"].items():
            if sub.get("type") == "object":
                with self.subTest(section=name):
                    self.assertIs(
                        sub.get("additionalProperties"), False,
                        msg=f"section {name!r} must reject unknown keys",
                    )
                    self.assertIn("required", sub)
                    self.assertIn("properties", sub)

    def test_required_top_level_keys_match_default_toml(self) -> None:
        # Sanity: the canonical default must satisfy the listed top-level required keys.
        try:
            import tomllib
        except ModuleNotFoundError:  # pragma: no cover
            import tomli as tomllib  # type: ignore
        data = tomllib.loads(_DEFAULT_TOML.read_text(encoding="utf-8"))
        for key in self.schema["required"]:
            with self.subTest(key=key):
                self.assertIn(key, data)


# ---------------------------------------------------------------- BC_B


class BC_B_LoaderFallbackChain(unittest.TestCase):

    def setUp(self) -> None:
        os.environ.pop("LIFETRAC_BUILD_CONFIG_PATH", None)
        self.bc = _import_loader()

    def tearDown(self) -> None:
        os.environ.pop("LIFETRAC_BUILD_CONFIG_PATH", None)

    def test_default_load_returns_canonical(self) -> None:
        cfg = self.bc.load()
        self.assertEqual(cfg.source_path, _DEFAULT_TOML)
        self.assertEqual(cfg.unit_id, "lifetrac-001")
        self.assertEqual(cfg.hydraulic.track_axis_count, 2)
        self.assertTrue(cfg.cameras.front_present)
        self.assertEqual(cfg.cameras.coral_tpu, "mini_pcie")

    def test_unit_id_override_wins_over_default(self) -> None:
        per_unit = self.bc.CONFIG_DIR / "build.lifetrac-test-unit.toml"
        per_unit.write_text(
            _DEFAULT_TOML.read_text(encoding="utf-8").replace(
                'unit_id        = "lifetrac-001"',
                'unit_id        = "lifetrac-test-unit"',
            ).replace("count                = 1", "count                = 0")
            .replace("front_present        = true", "front_present        = false"),
            encoding="utf-8",
        )
        try:
            cfg = self.bc.load(unit_id="lifetrac-test-unit")
            self.assertEqual(cfg.source_path, per_unit)
            self.assertEqual(cfg.unit_id, "lifetrac-test-unit")
            self.assertEqual(cfg.cameras.count, 0)
            self.assertFalse(cfg.cameras.front_present)
        finally:
            per_unit.unlink(missing_ok=True)

    def test_env_var_wins_over_unit_id_and_default(self) -> None:
        per_unit = self.bc.CONFIG_DIR / "build.lifetrac-shadowed.toml"
        per_unit.write_text(_DEFAULT_TOML.read_text(encoding="utf-8"), encoding="utf-8")
        env_file = self.bc.CONFIG_DIR / "build.env-target.toml"
        env_file.write_text(
            _DEFAULT_TOML.read_text(encoding="utf-8").replace(
                'mqtt_host = "localhost"', 'mqtt_host = "from-env"',
            ),
            encoding="utf-8",
        )
        try:
            with mock.patch.dict(os.environ, {"LIFETRAC_BUILD_CONFIG_PATH": str(env_file)}):
                cfg = self.bc.load(unit_id="lifetrac-shadowed")
            self.assertEqual(cfg.source_path, env_file)
            self.assertEqual(cfg.net.mqtt_host, "from-env")
        finally:
            per_unit.unlink(missing_ok=True)
            env_file.unlink(missing_ok=True)

    def test_env_var_pointing_at_missing_file_raises(self) -> None:
        with mock.patch.dict(
            os.environ,
            {"LIFETRAC_BUILD_CONFIG_PATH": str(_BS / "no-such-file.toml")},
        ):
            with self.assertRaises(self.bc.BuildConfigError):
                self.bc.load()


# ---------------------------------------------------------------- BC_C


class BC_C_StrictValidation(unittest.TestCase):

    def setUp(self) -> None:
        self.bc = _import_loader()
        self.tmp = self.bc.CONFIG_DIR / "build.test-validation.toml"

    def tearDown(self) -> None:
        self.tmp.unlink(missing_ok=True)
        os.environ.pop("LIFETRAC_BUILD_CONFIG_PATH", None)

    def _load_with(self, body: str):
        self.tmp.write_text(body, encoding="utf-8")
        with mock.patch.dict(os.environ, {"LIFETRAC_BUILD_CONFIG_PATH": str(self.tmp)}):
            return self.bc.load()

    def test_unknown_top_level_key_rejected(self) -> None:
        body = _DEFAULT_TOML.read_text(encoding="utf-8") + "\nbogus = 1\n"
        with self.assertRaises(self.bc.BuildConfigError) as ctx:
            self._load_with(body)
        self.assertIn("bogus", str(ctx.exception))

    def test_unknown_nested_key_rejected(self) -> None:
        body = _DEFAULT_TOML.read_text(encoding="utf-8").replace(
            "[hydraulic]", "[hydraulic]\nnot_a_real_key = 7"
        )
        with self.assertRaises(self.bc.BuildConfigError) as ctx:
            self._load_with(body)
        self.assertIn("not_a_real_key", str(ctx.exception))

    def test_out_of_range_integer_rejected(self) -> None:
        body = _DEFAULT_TOML.read_text(encoding="utf-8").replace(
            "track_axis_count    = 2", "track_axis_count    = 99"
        )
        with self.assertRaises(self.bc.BuildConfigError):
            self._load_with(body)

    def test_wrong_type_rejected(self) -> None:
        body = _DEFAULT_TOML.read_text(encoding="utf-8").replace(
            "proportional_flow   = true", 'proportional_flow   = "yes"'
        )
        with self.assertRaises(self.bc.BuildConfigError):
            self._load_with(body)

    def test_bad_enum_rejected(self) -> None:
        body = _DEFAULT_TOML.read_text(encoding="utf-8").replace(
            'estop_topology          = "psr_monitored_dual"',
            'estop_topology          = "no_estop_at_all"',
        )
        with self.assertRaises(self.bc.BuildConfigError) as ctx:
            self._load_with(body)
        self.assertIn("enum", str(ctx.exception))

    def test_unit_id_pattern_enforced(self) -> None:
        body = _DEFAULT_TOML.read_text(encoding="utf-8").replace(
            'unit_id        = "lifetrac-001"',
            'unit_id        = "Has Spaces!"',
        )
        with self.assertRaises(self.bc.BuildConfigError) as ctx:
            self._load_with(body)
        self.assertIn("pattern", str(ctx.exception))

    def test_missing_required_section_rejected(self) -> None:
        # Strip out [comm] entirely.
        text = _DEFAULT_TOML.read_text(encoding="utf-8")
        body = re.sub(r"\n\[comm\][^\[]+", "\n", text)
        with self.assertRaises(self.bc.BuildConfigError) as ctx:
            self._load_with(body)
        self.assertIn("comm", str(ctx.exception))


# ---------------------------------------------------------------- BC_D


class BC_D_DeterministicSha256(unittest.TestCase):

    def setUp(self) -> None:
        os.environ.pop("LIFETRAC_BUILD_CONFIG_PATH", None)
        self.bc = _import_loader()
        self.tmp = self.bc.CONFIG_DIR / "build.test-sha.toml"

    def tearDown(self) -> None:
        self.tmp.unlink(missing_ok=True)
        os.environ.pop("LIFETRAC_BUILD_CONFIG_PATH", None)

    def _load_body(self, body: str):
        self.tmp.write_text(body, encoding="utf-8")
        with mock.patch.dict(os.environ, {"LIFETRAC_BUILD_CONFIG_PATH": str(self.tmp)}):
            return self.bc.load()

    def test_sha_matches_canonical_json_formula(self) -> None:
        cfg = self.bc.load()
        canonical = json.dumps(cfg.raw, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
        expected = hashlib.sha256(canonical.encode("utf-8")).hexdigest()
        self.assertEqual(cfg.config_sha256, expected)

    def test_sha_invariant_to_whitespace_and_section_order(self) -> None:
        a = self._load_body(_DEFAULT_TOML.read_text(encoding="utf-8")).config_sha256
        # Re-order [net] before [ui] and add blank lines / extra spaces — same data.
        text = _DEFAULT_TOML.read_text(encoding="utf-8")
        ui_block = re.search(r"\n\[ui\][^\[]+", text).group(0)
        net_block = re.search(r"\n\[net\][^\[]+", text).group(0)
        rearranged = (
            text.replace(ui_block, "").replace(net_block, "")
            + "\n\n\n"
            + net_block
            + "\n\n"
            + ui_block
        )
        b = self._load_body(rearranged).config_sha256
        self.assertEqual(a, b)

    def test_sha_changes_when_any_leaf_changes(self) -> None:
        a = self.bc.load().config_sha256
        b = self._load_body(
            _DEFAULT_TOML.read_text(encoding="utf-8").replace(
                "track_ramp_seconds  = 2.0", "track_ramp_seconds  = 2.5"
            )
        ).config_sha256
        self.assertNotEqual(a, b)


# ---------------------------------------------------------------- BC_E


_INVENTORY_ID_RE = re.compile(r"`([a-z][a-z0-9_]*(?:\.[a-z][a-z0-9_]*)*)`")


def _walk_schema_ids(schema: dict) -> set[str]:
    out: set[str] = set()
    for top, sub in schema.get("properties", {}).items():
        if sub.get("type") == "object":
            for nested in sub.get("properties", {}):
                out.add(f"{top}.{nested}")
        else:
            out.add(top)
    return out


def _inventory_ids() -> set[str]:
    text = _INVENTORY.read_text(encoding="utf-8")
    # Match every backticked dotted id whose first segment is one of our sections,
    # plus the bare identity ids.
    sections = ("hydraulic", "safety", "cameras", "sensors", "comm", "ui", "net", "aux")
    out: set[str] = set()
    for tok in _INVENTORY_ID_RE.findall(text):
        if "." in tok and tok.split(".", 1)[0] in sections:
            out.add(tok)
        elif tok in {"unit_id", "schema_version"}:
            out.add(tok)
    return out


class BC_E_InventoryParity(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        cls.schema = json.loads(_SCHEMA_PATH.read_text(encoding="utf-8"))

    def test_inventory_doc_exists(self) -> None:
        self.assertTrue(_INVENTORY.is_file(), msg=f"missing {_INVENTORY}")

    def test_every_schema_property_has_inventory_row(self) -> None:
        schema_ids = _walk_schema_ids(self.schema)
        inventory_ids = _inventory_ids()
        missing = schema_ids - inventory_ids
        self.assertFalse(
            missing,
            msg=f"schema properties missing from CAPABILITY_INVENTORY.md: {sorted(missing)}",
        )

    def test_no_orphan_inventory_rows(self) -> None:
        schema_ids = _walk_schema_ids(self.schema)
        inventory_ids = _inventory_ids()
        orphans = inventory_ids - schema_ids
        self.assertFalse(
            orphans,
            msg=f"inventory rows with no schema property: {sorted(orphans)}",
        )


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
