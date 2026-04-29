"""Round 33 / BC-11 SIL gate: first-class auxiliary attachment ports.

Pins the contract that the new ``[aux]`` section behaves identically to
every other top-level section in the schema:

* the loader exposes an ``AuxConfig`` dataclass and populates it from
  the canonical TOML;
* the codegen walks ``aux`` and emits ``LIFETRAC_AUX_*`` macros plus
  enum side-macros for ``coupler_type``;
* every leaf carries a ``reload_class`` annotation and the BC-10 diff
  helper classifies a change to any aux leaf as ``restart_required``
  (NOT ``live`` -- aux ports are a hardware capability, the firmware
  must re-init PWM channels and the attachment-permit gate);
* the schema's ``required`` list mentions ``aux`` so a missing section
  is rejected by the loader;
* the canonical default is the conservative shape (``port_count = 0``,
  ``coupler_type = "none"``, ``case_drain_present = false``) so a
  build that doesn't declare aux gets the safe answer.

These are pure-Python regressions; no bench hardware required.
"""

from __future__ import annotations

import importlib
import json
import sys
import tempfile
import unittest
from pathlib import Path

_BS = Path(__file__).resolve().parents[1]
if str(_BS) not in sys.path:
    sys.path.insert(0, str(_BS))

import build_config as _bc  # type: ignore[import-not-found]
import build_config_codegen as _cg  # type: ignore[import-not-found]


_DEFAULT_TOML = _BS / "config" / "build.default.toml"
_SCHEMA_PATH = _BS / "config" / "build_config.schema.json"


def _reload_loader():
    importlib.reload(_bc)
    importlib.reload(_cg)
    return _bc, _cg


# ============================================================ BC11_A schema


class BC11_A_SchemaDeclaresAux(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        cls.schema = json.loads(_SCHEMA_PATH.read_text(encoding="utf-8"))

    def test_aux_listed_in_top_level_required(self) -> None:
        self.assertIn("aux", self.schema["required"])

    def test_aux_section_object_with_three_required_leaves(self) -> None:
        aux = self.schema["properties"]["aux"]
        self.assertEqual(aux["type"], "object")
        self.assertFalse(aux.get("additionalProperties", True))
        self.assertEqual(
            sorted(aux["required"]),
            sorted(["port_count", "coupler_type", "case_drain_present"]),
        )

    def test_every_aux_leaf_carries_restart_required_reload_class(self) -> None:
        leaves = self.schema["properties"]["aux"]["properties"]
        for name, leaf in leaves.items():
            with self.subTest(leaf=name):
                self.assertEqual(
                    leaf.get("reload_class"), "restart_required",
                    msg=f"aux.{name} should be restart_required (hardware capability)",
                )

    def test_coupler_type_enum_includes_safe_none(self) -> None:
        ct = self.schema["properties"]["aux"]["properties"]["coupler_type"]
        self.assertEqual(ct["type"], "string")
        self.assertIn("none", ct["enum"])
        self.assertIn("iso_5675", ct["enum"])
        self.assertIn("flat_face", ct["enum"])


# ============================================================ BC11_B loader


class BC11_B_LoaderExposesAuxConfig(unittest.TestCase):

    def test_aux_dataclass_has_three_fields(self) -> None:
        from dataclasses import fields
        names = {f.name for f in fields(_bc.AuxConfig)}
        self.assertEqual(names, {"port_count", "coupler_type", "case_drain_present"})

    def test_canonical_default_aux_is_conservative(self) -> None:
        cfg = _bc.load()
        self.assertEqual(cfg.aux.port_count, 0)
        self.assertEqual(cfg.aux.coupler_type, "none")
        self.assertFalse(cfg.aux.case_drain_present)

    def test_loader_rejects_missing_aux_section(self) -> None:
        body = _DEFAULT_TOML.read_text(encoding="utf-8")
        # Strip the entire [aux] block (3 data lines + heading + trailing blank).
        head, _, tail = body.partition("[aux]\n")
        self.assertNotEqual(tail, "", msg="canonical default must contain [aux] block")
        # Find the next [section] header and resume from there.
        next_section_pos = tail.find("\n[")
        self.assertGreater(next_section_pos, 0, msg="no following section after [aux]")
        rebuilt = head + tail[next_section_pos + 1 :]
        self.assertNotIn("[aux]", rebuilt, msg="strip-aux fixture failed (tripwire)")
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "no-aux.toml"
            p.write_text(rebuilt, encoding="utf-8")
            import os
            os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = str(p)
            try:
                with self.assertRaises(_bc.BuildConfigError) as ctx:
                    _bc.load()
                self.assertIn("aux", str(ctx.exception))
            finally:
                os.environ.pop("LIFETRAC_BUILD_CONFIG_PATH", None)


# ============================================================ BC11_C codegen


class BC11_C_CodegenEmitsAuxMacros(unittest.TestCase):

    def setUp(self) -> None:
        self.cfg = _bc.load()
        self.text = _cg.emit_header(self.cfg)

    def test_aux_in_codegen_sections_tuple(self) -> None:
        self.assertIn("aux", _cg._SECTIONS)

    def test_every_aux_leaf_emits_a_macro(self) -> None:
        self.assertIn("/* [aux] */", self.text)
        self.assertIn("#define LIFETRAC_AUX_PORT_COUNT 0", self.text)
        self.assertIn('#define LIFETRAC_AUX_COUPLER_TYPE "none"', self.text)
        self.assertIn("#define LIFETRAC_AUX_CASE_DRAIN_PRESENT 0", self.text)

    def test_coupler_type_emits_enum_side_macros(self) -> None:
        # Active value (none) -> 1; others -> 0.
        self.assertIn("#define LIFETRAC_AUX_COUPLER_TYPE_NONE 1", self.text)
        self.assertIn("#define LIFETRAC_AUX_COUPLER_TYPE_ISO_5675 0", self.text)
        self.assertIn("#define LIFETRAC_AUX_COUPLER_TYPE_FLAT_FACE 0", self.text)


# ============================================================ BC11_D reload


class BC11_D_AuxLeafChangesAreRestartRequired(unittest.TestCase):

    def setUp(self) -> None:
        self.base = _bc.load()
        self.classes = _bc.iter_reload_classes()

    def test_iter_reload_classes_includes_every_aux_leaf(self) -> None:
        for leaf in ("port_count", "coupler_type", "case_drain_present"):
            with self.subTest(leaf=leaf):
                self.assertEqual(
                    self.classes.get(f"aux.{leaf}"), "restart_required"
                )

    def test_diff_aux_change_classifies_restart_required(self) -> None:
        new_raw = json.loads(json.dumps(self.base.raw))
        new_raw["aux"]["port_count"] = 2
        new_raw["aux"]["coupler_type"] = "flat_face"
        new_raw["aux"]["case_drain_present"] = True

        # Round-trip through the validator + reconstruct (mirrors the
        # delivery test pattern).
        from dataclasses import replace
        _bc._validate(new_raw, _bc.load_schema(), path="")
        new_cfg = replace(
            self.base,
            aux=_bc.AuxConfig(**new_raw["aux"]),
            raw=new_raw,
        )
        diff = _bc.diff_reload_classes(self.base, new_cfg)
        self.assertEqual(
            sorted(diff.changed),
            sorted(["aux.port_count", "aux.coupler_type", "aux.case_drain_present"]),
        )
        self.assertEqual(diff.worst, "restart_required")


# ============================================================ BC11_E header


class BC11_E_FirmwareHeaderInSyncWithAux(unittest.TestCase):
    """The on-disk header must already contain the aux macros (drift gate)."""

    def test_committed_header_contains_aux_macros(self) -> None:
        header = (_BS.parents[0] / "firmware" / "common" / "lifetrac_build_config.h").read_text(
            encoding="ascii"
        )
        self.assertIn("/* [aux] */", header)
        self.assertIn("#define LIFETRAC_AUX_PORT_COUNT 0", header)
        self.assertIn('#define LIFETRAC_AUX_COUPLER_TYPE "none"', header)
        self.assertIn("#define LIFETRAC_AUX_CASE_DRAIN_PRESENT 0", header)
        self.assertIn("#define LIFETRAC_AUX_COUPLER_TYPE_NONE 1", header)


if __name__ == "__main__":
    unittest.main()
