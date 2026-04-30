"""SIL gate for ``lifetrac-config diff --format`` (Round 42 / BC-15).

Round 42 wires a machine-readable JSON form into the existing BC-10
``diff_reload_classes`` CLI subcommand so non-Python consumers (notably
``hil/dispatch.ps1`` and ad-hoc bench-laptop shell pipelines) can
decide whether a candidate config requires a service restart or a
firmware reflash without re-implementing the schema-aware comparator.

Test classes use the BC15_* prefix so MASTER_TEST_PROGRAM.md $5 maps
cleanly back to BC-15 (CLI half).
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


def _run_cli(*argv: str) -> tuple[int, str, str]:
    """Invoke ``lifetrac_config.main`` in-process and capture streams."""
    out = io.StringIO()
    err = io.StringIO()
    with redirect_stdout(out), redirect_stderr(err):
        rc = lc.main(list(argv))
    return rc, out.getvalue(), err.getvalue()


def _mutate_default(tmp: Path, **overrides: str) -> Path:
    """Copy the canonical default TOML and apply naive line-level overrides.

    ``overrides`` maps a left-hand-side key prefix (e.g. ``"track_ramp_seconds"``)
    to the replacement value rendered as a TOML literal (e.g. ``"3.0"``). The
    first matching ``key = `` line is rewritten. Sufficient for the small
    number of well-known leaves used by this gate.
    """
    text = _DEFAULT_TOML.read_text(encoding="utf-8")
    out_lines: list[str] = []
    pending = dict(overrides)
    for line in text.splitlines():
        stripped = line.lstrip()
        replaced = False
        for key, value in list(pending.items()):
            if stripped.startswith(f"{key} ") or stripped.startswith(f"{key}="):
                indent = line[: len(line) - len(stripped)]
                out_lines.append(f"{indent}{key} = {value}")
                pending.pop(key)
                replaced = True
                break
        if not replaced:
            out_lines.append(line)
    if pending:
        raise AssertionError(f"_mutate_default: never matched keys {sorted(pending)}")
    p = tmp / "build.candidate.toml"
    p.write_text("\n".join(out_lines) + "\n", encoding="utf-8")
    return p


class _TmpDirCase(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.tmp = Path(self._tmp.name)


class BC15_A_TextFormatUnchanged(_TmpDirCase):
    """Default ``--format text`` output is byte-identical to pre-BC-15."""

    def test_no_diff_text(self) -> None:
        rc, stdout, stderr = _run_cli(
            "diff", str(_DEFAULT_TOML), "--against", str(_DEFAULT_TOML)
        )
        self.assertEqual(rc, lc.EXIT_OK, msg=f"stderr={stderr!r}")
        self.assertEqual(stdout.strip(), "OK  configs identical (no diff)")

    def test_live_change_text(self) -> None:
        cand = _mutate_default(self.tmp, track_ramp_seconds="3.0")
        rc, stdout, _ = _run_cli("diff", str(_DEFAULT_TOML), "--against", str(cand))
        self.assertEqual(rc, lc.EXIT_OK)
        self.assertIn("hydraulic.track_ramp_seconds", stdout)
        self.assertIn("live", stdout)
        self.assertIn("--> reload required: live", stdout)

    def test_explicit_text_flag_matches_default(self) -> None:
        cand = _mutate_default(self.tmp, track_ramp_seconds="3.0")
        rc1, out1, _ = _run_cli("diff", str(_DEFAULT_TOML), "--against", str(cand))
        rc2, out2, _ = _run_cli(
            "diff", str(_DEFAULT_TOML), "--against", str(cand), "--format", "text"
        )
        self.assertEqual(rc1, rc2)
        self.assertEqual(out1, out2)


class BC15_B_JsonFormatShape(_TmpDirCase):
    """``--format json`` emits the documented canonical payload."""

    def test_no_diff_json(self) -> None:
        rc, stdout, _ = _run_cli(
            "diff", str(_DEFAULT_TOML), "--against", str(_DEFAULT_TOML),
            "--format", "json",
        )
        self.assertEqual(rc, lc.EXIT_OK)
        payload = json.loads(stdout)
        self.assertEqual(payload["changed"], [])
        self.assertEqual(payload["classes"], {})
        self.assertIsNone(payload["worst"])
        self.assertTrue(payload["is_empty"])
        self.assertFalse(payload["restart_required"])
        self.assertFalse(payload["firmware_required"])

    def test_live_change_json(self) -> None:
        cand = _mutate_default(self.tmp, track_ramp_seconds="3.0")
        rc, stdout, _ = _run_cli(
            "diff", str(_DEFAULT_TOML), "--against", str(cand), "--format", "json"
        )
        self.assertEqual(rc, lc.EXIT_OK)
        payload = json.loads(stdout)
        self.assertEqual(payload["changed"], ["hydraulic.track_ramp_seconds"])
        self.assertEqual(
            payload["classes"], {"hydraulic.track_ramp_seconds": "live"}
        )
        self.assertEqual(payload["worst"], "live")
        self.assertFalse(payload["is_empty"])
        self.assertFalse(payload["restart_required"])
        self.assertFalse(payload["firmware_required"])

    def test_restart_required_json(self) -> None:
        cand = _mutate_default(self.tmp, track_axis_count="1")
        rc, stdout, _ = _run_cli(
            "diff", str(_DEFAULT_TOML), "--against", str(cand), "--format", "json"
        )
        self.assertEqual(rc, lc.EXIT_OK)
        payload = json.loads(stdout)
        self.assertEqual(payload["worst"], "restart_required")
        self.assertTrue(payload["restart_required"])
        self.assertFalse(payload["firmware_required"])
        self.assertEqual(
            payload["classes"]["hydraulic.track_axis_count"], "restart_required"
        )

    def test_firmware_required_json(self) -> None:
        cand = _mutate_default(self.tmp, m4_watchdog_ms="300")
        rc, stdout, _ = _run_cli(
            "diff", str(_DEFAULT_TOML), "--against", str(cand), "--format", "json"
        )
        self.assertEqual(rc, lc.EXIT_OK)
        payload = json.loads(stdout)
        self.assertEqual(payload["worst"], "firmware_required")
        self.assertTrue(payload["restart_required"])
        self.assertTrue(payload["firmware_required"])

    def test_worst_promotes_across_multiple_changes(self) -> None:
        cand = _mutate_default(
            self.tmp, track_ramp_seconds="3.0", m4_watchdog_ms="300"
        )
        rc, stdout, _ = _run_cli(
            "diff", str(_DEFAULT_TOML), "--against", str(cand), "--format", "json"
        )
        self.assertEqual(rc, lc.EXIT_OK)
        payload = json.loads(stdout)
        self.assertEqual(payload["worst"], "firmware_required")
        self.assertEqual(set(payload["changed"]), {
            "hydraulic.track_ramp_seconds", "safety.m4_watchdog_ms",
        })

    def test_json_is_canonical_sorted_compact(self) -> None:
        cand = _mutate_default(self.tmp, track_ramp_seconds="3.0")
        rc, stdout, _ = _run_cli(
            "diff", str(_DEFAULT_TOML), "--against", str(cand), "--format", "json"
        )
        self.assertEqual(rc, lc.EXIT_OK)
        line = stdout.strip()
        # Single line, no whitespace after separators (sorted-keys compact form
        # consistent with `dump-json`).
        self.assertNotIn("\n", line)
        self.assertNotIn(", ", line)
        self.assertNotIn(": ", line)
        self.assertEqual(json.dumps(json.loads(line), sort_keys=True,
                                     separators=(",", ":")), line)


class BC15_C_ArgparseSurface(unittest.TestCase):
    def test_invalid_format_rejected(self) -> None:
        # argparse on a choice violation calls parser.exit(2, ...) which
        # raises SystemExit; verify the wiring fails closed and tags the
        # offending value in stderr.
        err = io.StringIO()
        with redirect_stderr(err):
            with self.assertRaises(SystemExit) as ctx:
                lc.main([
                    "diff", str(_DEFAULT_TOML), "--against", str(_DEFAULT_TOML),
                    "--format", "yaml",
                ])
        self.assertEqual(ctx.exception.code, 2)
        self.assertIn("invalid choice", err.getvalue().lower())


if __name__ == "__main__":
    unittest.main()
