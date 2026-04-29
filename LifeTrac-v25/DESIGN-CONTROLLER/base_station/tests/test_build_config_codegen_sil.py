"""SIL gate for Round 31 -- IP: BC-03 (firmware codegen).

Round 31 lands the codegen path for the firmware build:
``base_station/build_config_codegen.py`` walks the validated
:class:`build_config.BuildConfig` and emits a deterministic C header
(``DESIGN-CONTROLLER/firmware/common/lifetrac_build_config.h``) that
the M4/M7 sketches ``#include`` in place of their hand-edited
``#define`` blocks. ``tools/lifetrac-config codegen`` exposes the
same path on the command line with a ``--check`` mode for CI drift
detection.

BC03_A_HeaderShape
    Header guard, identity macros, and one canonical leaf per section
    are present and well-formed. Trailing newline, ASCII-only,
    LF-terminated.

BC03_B_LeafParity
    Every scalar leaf the JSON Schema declares as a property of every
    section appears as a ``#define`` in the emitted header. New schema
    leaves added without a codegen update fail this gate.

BC03_C_TypeFormatting
    ``int`` -> bare integer; ``bool`` -> ``1`` / ``0``; ``float`` ->
    decimal with trailing ``f``; ``str`` -> double-quoted; enum
    side macros emitted with the active option set to ``1`` and all
    other options set to ``0``.

BC03_D_Determinism
    Two emissions of the same config produce byte-identical output.
    The on-disk canonical header in ``firmware/common/`` matches a
    fresh emission against ``build.default.toml`` (the CI gate that
    catches "I changed the schema and forgot to regenerate").

BC03_E_LegacyAliases
    Every legacy alias in
    :data:`build_config_codegen._LEGACY_ALIASES` is emitted and
    refers to a macro that exists earlier in the header (so the C
    preprocessor can resolve the alias).

BC03_F_CliCodegen
    ``lifetrac-config codegen --out X`` writes a header byte-identical
    to a direct :func:`emit_header` call. ``--check`` returns 0 when
    the on-disk header matches and non-zero on drift.

BC03_G_SourceTripwire
    web_ui-style: codegen module exports + CLI subcommand + canonical
    on-disk header all carry the documented Round 31 / BC-03 markers.
"""

from __future__ import annotations

import importlib
import json
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

_BS = Path(__file__).resolve().parents[1]
_REPO = _BS.parents[1]  # LifeTrac-v25 root (DESIGN-CONTROLLER -> LifeTrac-v25)
_CONFIG_DIR = _BS / "config"
_DEFAULT_TOML = _CONFIG_DIR / "build.default.toml"
_SCHEMA_PATH = _CONFIG_DIR / "build_config.schema.json"
_FIRMWARE_HEADER = _REPO / "DESIGN-CONTROLLER" / "firmware" / "common" / "lifetrac_build_config.h"
_TOOL = _REPO / "tools" / "lifetrac_config.py"


def _import():
    sys.modules.pop("build_config", None)
    sys.modules.pop("build_config_codegen", None)
    bc = importlib.import_module("build_config")
    cg = importlib.import_module("build_config_codegen")
    return bc, cg


# ------------------------------------------------------------- BC03_A


class BC03_A_HeaderShape(unittest.TestCase):

    def setUp(self) -> None:
        self.bc, self.cg = _import()
        self.cfg = self.bc.load()
        self.text = self.cg.emit_header(self.cfg)

    def test_header_guard_present(self) -> None:
        self.assertIn(f"#ifndef {self.cg.HEADER_GUARD}", self.text)
        self.assertIn(f"#define {self.cg.HEADER_GUARD}", self.text)
        self.assertIn(f"#endif /* {self.cg.HEADER_GUARD} */", self.text)

    def test_identity_macros(self) -> None:
        self.assertIn(f'#define LIFETRAC_UNIT_ID "{self.cfg.unit_id}"', self.text)
        self.assertIn(f"#define LIFETRAC_SCHEMA_VERSION {self.cfg.schema_version}",
                      self.text)
        self.assertIn(f'#define LIFETRAC_CONFIG_SHA256_HEX "{self.cfg.config_sha256}"',
                      self.text)
        self.assertIn(
            f'#define LIFETRAC_CONFIG_SHA256_HEX_SHORT "{self.cfg.config_sha256[:8]}"',
            self.text,
        )

    def test_ascii_lf_trailing_newline(self) -> None:
        self.assertTrue(self.text.endswith("\n"))
        self.assertNotIn("\r", self.text)
        # ASCII-only (no BOM, no high-bit chars).
        self.text.encode("ascii")  # raises UnicodeEncodeError on drift

    def test_no_trailing_whitespace(self) -> None:
        for i, line in enumerate(self.text.splitlines(), start=1):
            self.assertEqual(line, line.rstrip(),
                             msg=f"line {i} has trailing whitespace: {line!r}")


# ------------------------------------------------------------- BC03_B


class BC03_B_LeafParity(unittest.TestCase):

    def test_every_schema_leaf_emitted(self) -> None:
        bc, cg = _import()
        cfg = bc.load()
        text = cg.emit_header(cfg)
        with _SCHEMA_PATH.open("rb") as f:
            schema = json.loads(f.read().decode("utf-8"))
        missing: list[str] = []
        for section in ("hydraulic", "safety", "cameras",
                        "sensors", "comm", "ui", "net"):
            props = schema["properties"][section]["properties"]
            for leaf in props:
                macro = f"LIFETRAC_{section.upper()}_{leaf.upper()}"
                if f"#define {macro} " not in text:
                    missing.append(macro)
        self.assertEqual(missing, [],
                         msg=f"codegen missing macros for: {missing}")


# ------------------------------------------------------------- BC03_C


class BC03_C_TypeFormatting(unittest.TestCase):

    def setUp(self) -> None:
        self.bc, self.cg = _import()
        self.cfg = self.bc.load()
        self.text = self.cg.emit_header(self.cfg)

    def test_int_bare_literal(self) -> None:
        self.assertIn("#define LIFETRAC_HYDRAULIC_TRACK_AXIS_COUNT 2", self.text)
        self.assertIn("#define LIFETRAC_NET_MQTT_PORT 1883", self.text)

    def test_bool_one_or_zero(self) -> None:
        self.assertIn("#define LIFETRAC_HYDRAULIC_PROPORTIONAL_FLOW 1", self.text)
        self.assertIn("#define LIFETRAC_CAMERAS_REAR_PRESENT 0", self.text)

    def test_float_trailing_f_suffix(self) -> None:
        # Match e.g. "#define LIFETRAC_HYDRAULIC_TRACK_RAMP_SECONDS 2.0f"
        m = re.search(
            r"#define LIFETRAC_HYDRAULIC_TRACK_RAMP_SECONDS (\S+)",
            self.text,
        )
        self.assertIsNotNone(m)
        self.assertTrue(m.group(1).endswith("f"),
                        msg=f"float not f-suffixed: {m.group(1)!r}")
        self.assertIn(".", m.group(1),
                      msg=f"float missing decimal point: {m.group(1)!r}")

    def test_string_double_quoted(self) -> None:
        self.assertIn('#define LIFETRAC_NET_MQTT_HOST "localhost"', self.text)

    def test_enum_side_macros_active_one_others_zero(self) -> None:
        # Default: estop_topology = "psr_monitored_dual"
        self.assertIn(
            "#define LIFETRAC_SAFETY_ESTOP_TOPOLOGY_PSR_MONITORED_DUAL 1",
            self.text,
        )
        self.assertIn(
            "#define LIFETRAC_SAFETY_ESTOP_TOPOLOGY_PSR_MONITORED_SINGLE 0",
            self.text,
        )
        self.assertIn(
            "#define LIFETRAC_SAFETY_ESTOP_TOPOLOGY_HARDWIRED_ONLY 0",
            self.text,
        )

    def test_enum_value_string_macro_also_present(self) -> None:
        self.assertIn(
            '#define LIFETRAC_SAFETY_ESTOP_TOPOLOGY "psr_monitored_dual"',
            self.text,
        )


# ------------------------------------------------------------- BC03_D


class BC03_D_Determinism(unittest.TestCase):

    def test_two_emissions_byte_identical(self) -> None:
        bc, cg = _import()
        cfg = bc.load()
        a = cg.emit_header(cfg)
        b = cg.emit_header(cfg)
        self.assertEqual(a, b)

    def test_canonical_header_matches_default_toml(self) -> None:
        """The on-disk firmware/common/lifetrac_build_config.h must be a
        byte-for-byte rendering of the canonical default TOML. CI gate.
        """
        bc, cg = _import()
        cfg = bc.load()
        fresh = cg.emit_header(cfg)
        on_disk = _FIRMWARE_HEADER.read_text(encoding="ascii")
        self.assertEqual(
            on_disk, fresh,
            msg=("firmware/common/lifetrac_build_config.h is stale; "
                 "regenerate with `python tools/lifetrac_config.py codegen "
                 "DESIGN-CONTROLLER/base_station/config/build.default.toml "
                 "--out DESIGN-CONTROLLER/firmware/common/lifetrac_build_config.h`"),
        )


# ------------------------------------------------------------- BC03_E


class BC03_E_LegacyAliases(unittest.TestCase):

    def test_each_legacy_alias_emitted_and_resolvable(self) -> None:
        bc, cg = _import()
        cfg = bc.load()
        text = cg.emit_header(cfg)
        for legacy, canonical in cg._LEGACY_ALIASES:
            self.assertIn(f"#define {legacy} {canonical}", text,
                          msg=f"legacy alias {legacy} not emitted")
            # The canonical macro must be defined earlier in the file
            # (i.e. the alias is resolvable when the header is included).
            self.assertIn(f"#define {canonical} ", text,
                          msg=f"alias target {canonical} not defined")
            li_alias = text.index(f"#define {legacy} {canonical}")
            li_target = text.index(f"#define {canonical} ")
            self.assertLess(li_target, li_alias,
                            msg=f"{canonical} defined AFTER {legacy} (bad order)")


# ------------------------------------------------------------- BC03_F


class BC03_F_CliCodegen(unittest.TestCase):

    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="lt-bc03-cli-"))
        self.out = self.tmp / "lifetrac_build_config.h"

    def _run(self, *args: str) -> subprocess.CompletedProcess:
        return subprocess.run(
            [sys.executable, str(_TOOL), *args],
            capture_output=True, text=True,
        )

    def test_codegen_writes_byte_identical_to_emit(self) -> None:
        r = self._run("codegen", str(_DEFAULT_TOML), "--out", str(self.out))
        self.assertEqual(r.returncode, 0, msg=r.stderr)
        self.assertTrue(self.out.is_file())
        bc, cg = _import()
        cfg = bc.load()
        expected = cg.emit_header(cfg)
        self.assertEqual(self.out.read_text(encoding="ascii"), expected)

    def test_check_mode_clean(self) -> None:
        r1 = self._run("codegen", str(_DEFAULT_TOML), "--out", str(self.out))
        self.assertEqual(r1.returncode, 0, msg=r1.stderr)
        r2 = self._run("codegen", str(_DEFAULT_TOML), "--out", str(self.out),
                       "--check")
        self.assertEqual(r2.returncode, 0, msg=r2.stderr)
        self.assertIn("matches canonical", r2.stdout)

    def test_check_mode_drift_fails(self) -> None:
        # Write a deliberately stale header.
        self.out.write_text("/* stale */\n", encoding="ascii")
        r = self._run("codegen", str(_DEFAULT_TOML), "--out", str(self.out),
                      "--check")
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("stale", r.stderr)

    def test_canonical_firmware_header_passes_check(self) -> None:
        """``--check`` against the on-disk canonical header must pass.

        This is the CI gate: a schema or default-TOML edit that doesn't
        also regenerate the firmware header turns red here.
        """
        r = self._run("codegen", str(_DEFAULT_TOML),
                      "--out", str(_FIRMWARE_HEADER), "--check")
        self.assertEqual(r.returncode, 0, msg=r.stderr or r.stdout)


# ------------------------------------------------------------- BC03_G


class BC03_G_SourceTripwire(unittest.TestCase):

    def test_codegen_module_exports(self) -> None:
        _, cg = _import()
        for name in ("emit_header", "write_header", "GENERATOR",
                     "HEADER_GUARD", "MACRO_PREFIX"):
            self.assertIn(name, cg.__all__, msg=name)

    def test_codegen_module_carries_round_marker(self) -> None:
        src = (_BS / "build_config_codegen.py").read_text(encoding="utf-8")
        self.assertIn("Round 31", src)
        self.assertIn("BC-03", src)

    def test_cli_carries_codegen_subcommand(self) -> None:
        src = _TOOL.read_text(encoding="utf-8")
        self.assertIn("def cmd_codegen", src)
        self.assertIn('"codegen"', src)
        self.assertIn("--check", src)
        self.assertIn("BC-03", src)

    def test_canonical_header_carries_generator_marker(self) -> None:
        text = _FIRMWARE_HEADER.read_text(encoding="ascii")
        self.assertIn("AUTOGENERATED", text)
        self.assertIn("Round 31", text)
        self.assertIn("BC-03", text)
        self.assertIn("DO NOT EDIT", text)


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
