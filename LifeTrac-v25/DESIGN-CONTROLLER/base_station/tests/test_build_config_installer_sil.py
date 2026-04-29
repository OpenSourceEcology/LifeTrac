"""SIL gate for Round 29b-alpha -- IP: BC-10 (delivery half, alpha).

Pins the installer-bundle envelope, the four CLI subcommands' contracts,
and the base-side ``/config/download`` route. The X8-side installer
daemon, OLED + LED feedback, and the daemon watch-and-reload loop are
Round 29b-beta and are gated separately.

BC10b_A_BundleEnvelope
    ``make_bundle`` produces a Bundle whose body SHA matches what
    ``parse`` recomputes. Filename matches the documented pattern.
    serialise/parse round-trip preserves every field. Tampering with
    the body bytes after serialise breaks the SHA check on parse.
    Wrong ``bundle_version`` is refused. Wrong ``unit_id`` pattern is
    refused. Filename mismatch (rename) is refused.

BC10b_B_CLI
    ``validate`` exits 0 on the canonical TOML, exits 2 on a TOML with
    an out-of-range value, and prints the SHA on success. ``bundle``
    writes a file named per the documented pattern whose body SHA
    matches the embedded header SHA. ``verify`` accepts a bundle the
    bundler just produced and rejects one with a flipped body byte.
    ``diff`` returns reload-class output when configs differ and a
    no-diff message when they don't.

BC10b_C_DownloadRoute
    GET ``/config/download`` requires a session (401 without one),
    returns the bundle as a text/plain attachment with the documented
    Content-Disposition filename, and emits a ``config_download``
    audit-log entry. The downloaded body parses + verifies cleanly
    via the bundle module (no transport-layer mangling).

BC10b_D_SourceTripwire
    The CLI script exists at ``tools/lifetrac_config.py``, exposes the
    four subcommands by name, and the web_ui contains the
    ``/config/download`` route + ``config_download`` audit verb.
"""

from __future__ import annotations

import importlib
import io
import json
import os
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from unittest import mock

from build_config import load  # type: ignore[import-not-found]
from config_bundle import (  # type: ignore[import-not-found]
    BODY_SENTINEL,
    BUNDLE_VERSION,
    FILENAME_RE,
    BundleError,
    body_sha256,
    make_bundle,
    parse,
    parse_filename,
    serialise,
    verify_filename_matches,
)

_BS = Path(__file__).resolve().parents[1]
_REPO = _BS.parents[1]
_CLI = _REPO / "tools" / "lifetrac_config.py"
_DEFAULT_TOML = _BS / "config" / "build.default.toml"


# --------------------------------------------------------------- helpers


def _read_default_body() -> str:
    return _DEFAULT_TOML.read_text(encoding="utf-8")


def _run_cli(*argv: str) -> "subprocess.CompletedProcess[str]":
    return subprocess.run(
        [sys.executable, str(_CLI), *argv],
        capture_output=True,
        text=True,
        check=False,
    )


def _import_web_ui_with_default():
    """Re-import web_ui pinned to the canonical TOML for this test."""
    for mod in ("web_ui", "build_config"):
        sys.modules.pop(mod, None)
    env = dict(os.environ)
    env["LIFETRAC_BUILD_CONFIG_PATH"] = str(_DEFAULT_TOML)
    env["LIFETRAC_PIN"] = "1234"
    env.pop("LIFETRAC_PIN_FILE", None)
    with mock.patch.dict(os.environ, env, clear=True), \
         mock.patch("paho.mqtt.client.Client") as cls:
        cls.return_value.connect.return_value = None
        cls.return_value.loop_start.return_value = None
        web_ui = importlib.import_module("web_ui")
    return web_ui


# --------------------------------------------------------------- BC10b_A


class BC10b_A_BundleEnvelope(unittest.TestCase):
    """make_bundle / serialise / parse round-trip with anti-tamper checks."""

    def test_make_then_parse_round_trip(self) -> None:
        body = _read_default_body()
        b = make_bundle(body, "lifetrac-001", generator="test 0.0", created="2026-01-01T00:00:00Z")
        text = serialise(b)
        round_tripped = parse(text)
        self.assertEqual(round_tripped.unit_id, "lifetrac-001")
        self.assertEqual(round_tripped.sha256, b.sha256)
        self.assertEqual(round_tripped.bundle_version, BUNDLE_VERSION)
        self.assertEqual(round_tripped.body, body if body.endswith("\n") else body + "\n")
        self.assertEqual(round_tripped.generator, "test 0.0")
        self.assertEqual(round_tripped.created, "2026-01-01T00:00:00Z")

    def test_filename_matches_pattern(self) -> None:
        b = make_bundle(_read_default_body(), "lifetrac-001")
        m = FILENAME_RE.match(b.filename)
        self.assertIsNotNone(m, msg=f"{b.filename!r} did not match {FILENAME_RE.pattern}")
        self.assertEqual(m.group("unit_id"), "lifetrac-001")
        self.assertEqual(m.group("sha8"), b.sha256[:8])

    def test_body_tamper_breaks_sha(self) -> None:
        b = make_bundle(_read_default_body(), "lifetrac-001")
        text = serialise(b)
        # Replace a TOML-body-only string (header doesn't carry "[hydraulic]").
        tampered = text.replace("[hydraulic]", "[Hydraulic]", 1)
        self.assertNotEqual(tampered, text, msg="tamper substitution no-op")
        with self.assertRaises(BundleError):
            parse(tampered)

    def test_unsupported_bundle_version_refused(self) -> None:
        b = make_bundle(_read_default_body(), "lifetrac-001")
        text = serialise(b).replace(
            f"# bundle_version: {BUNDLE_VERSION}",
            "# bundle_version: 999",
        )
        with self.assertRaises(BundleError):
            parse(text)

    def test_bad_unit_id_pattern_refused(self) -> None:
        with self.assertRaises(BundleError):
            make_bundle(_read_default_body(), "Bad_ID!")

    def test_filename_unit_id_mismatch_refused(self) -> None:
        b = make_bundle(_read_default_body(), "lifetrac-001")
        with self.assertRaises(BundleError):
            verify_filename_matches(b, f"lifetrac-config-lifetrac-002-{b.sha8}.toml")

    def test_filename_sha_mismatch_refused(self) -> None:
        b = make_bundle(_read_default_body(), "lifetrac-001")
        with self.assertRaises(BundleError):
            verify_filename_matches(b, f"lifetrac-config-lifetrac-001-deadbeef.toml")

    def test_parse_filename_helper(self) -> None:
        unit, sha8 = parse_filename("lifetrac-config-lifetrac-001-12345678.toml")
        self.assertEqual(unit, "lifetrac-001")
        self.assertEqual(sha8, "12345678")
        with self.assertRaises(BundleError):
            parse_filename("not-a-bundle.toml")

    def test_missing_sentinel_refused(self) -> None:
        with self.assertRaises(BundleError):
            parse("# LifeTrac v25\n# unit_id: lifetrac-001\n[hydraulic]\n")


# --------------------------------------------------------------- BC10b_B


class BC10b_B_CLI(unittest.TestCase):
    """tools/lifetrac_config.py validate / bundle / verify / diff."""

    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)

    def tearDown(self) -> None:
        self._tmp.cleanup()

    def test_validate_canonical_passes(self) -> None:
        result = _run_cli("validate", str(_DEFAULT_TOML))
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("OK", result.stdout)
        self.assertIn("config_sha256:", result.stdout)
        self.assertIn("unit_id:", result.stdout)

    def test_validate_out_of_range_rejected(self) -> None:
        # Write a copy of the canonical TOML with one out-of-range value.
        body = _read_default_body().replace(
            "modbus_fail_latch_count = 10", "modbus_fail_latch_count = 999"
        )
        self.assertNotEqual(body, _read_default_body(), msg="replacement no-op")
        bad = self.tmp / "build.bad.toml"
        bad.write_text(body, encoding="utf-8")
        result = _run_cli("validate", str(bad))
        self.assertEqual(result.returncode, 2, msg=result.stdout + result.stderr)
        self.assertIn("ERROR", result.stderr)

    def test_bundle_writes_named_file_with_matching_sha(self) -> None:
        result = _run_cli("bundle", str(_DEFAULT_TOML), "-o", str(self.tmp))
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        files = list(self.tmp.glob("lifetrac-config-*.toml"))
        self.assertEqual(len(files), 1)
        bundle = parse(files[0].read_text(encoding="utf-8"))
        self.assertEqual(bundle.sha256, body_sha256(bundle.body))
        # Filename agrees with header (the round-trip contract).
        verify_filename_matches(bundle, files[0].name)

    def test_verify_accepts_bundler_output(self) -> None:
        _run_cli("bundle", str(_DEFAULT_TOML), "-o", str(self.tmp))
        bundle_path = next(self.tmp.glob("lifetrac-config-*.toml"))
        result = _run_cli("verify", str(bundle_path))
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("OK", result.stdout)

    def test_verify_rejects_tampered_body(self) -> None:
        _run_cli("bundle", str(_DEFAULT_TOML), "-o", str(self.tmp))
        bundle_path = next(self.tmp.glob("lifetrac-config-*.toml"))
        text = bundle_path.read_text(encoding="utf-8")
        tampered = text.replace(
            "max_control_subscribers   = 4", "max_control_subscribers   = 5", 1
        )
        self.assertNotEqual(text, tampered, msg="tamper replacement no-op")
        bundle_path.write_text(tampered, encoding="utf-8")
        result = _run_cli("verify", str(bundle_path))
        self.assertEqual(result.returncode, 2, msg=result.stdout)
        self.assertIn("ERROR", result.stderr)

    def test_diff_identical_configs_reports_empty(self) -> None:
        result = _run_cli(
            "diff", str(_DEFAULT_TOML), "--against", str(_DEFAULT_TOML)
        )
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("identical", result.stdout)

    def test_diff_changed_configs_reports_reload_class(self) -> None:
        body = _read_default_body().replace(
            "max_control_subscribers   = 4", "max_control_subscribers   = 8"
        )
        self.assertNotEqual(body, _read_default_body(), msg="replacement no-op")
        changed = self.tmp / "build.changed.toml"
        changed.write_text(body, encoding="utf-8")
        result = _run_cli(
            "diff", str(_DEFAULT_TOML), "--against", str(changed)
        )
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("ui.max_control_subscribers", result.stdout)
        self.assertIn("live", result.stdout)

    def test_diff_restart_required_change(self) -> None:
        body = _read_default_body().replace(
            'lora_region              = "us915"', 'lora_region              = "eu868"'
        )
        self.assertNotEqual(body, _read_default_body(), msg="replacement no-op")
        changed = self.tmp / "build.eu.toml"
        changed.write_text(body, encoding="utf-8")
        result = _run_cli(
            "diff", str(_DEFAULT_TOML), "--against", str(changed)
        )
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("comm.lora_region", result.stdout)
        self.assertIn("restart_required", result.stdout)


# --------------------------------------------------------------- BC10b_C


class BC10b_C_DownloadRoute(unittest.TestCase):
    """GET /config/download is PIN-gated, returns a parseable bundle, audits."""

    def setUp(self) -> None:
        from fastapi.testclient import TestClient  # local import (optional dep)

        self.web_ui = _import_web_ui_with_default()
        self.client = TestClient(self.web_ui.app)

    def _login(self) -> None:
        r = self.client.post("/api/login", json={"pin": "1234"})
        self.assertEqual(r.status_code, 200, msg=r.text)

    def test_unauthenticated_returns_401(self) -> None:
        r = self.client.get("/config/download")
        self.assertEqual(r.status_code, 401)

    def test_authenticated_returns_parseable_bundle(self) -> None:
        self._login()
        r = self.client.get("/config/download")
        self.assertEqual(r.status_code, 200, msg=r.text)
        cd = r.headers.get("content-disposition", "")
        self.assertIn("attachment", cd)
        self.assertIn("lifetrac-config-lifetrac-001-", cd)
        bundle = parse(r.text)
        self.assertEqual(bundle.unit_id, "lifetrac-001")
        # SHA in header matches body bytes -- no transport mangling.
        self.assertEqual(body_sha256(bundle.body), bundle.sha256)

    def test_emits_audit_entry(self) -> None:
        self._login()
        recorded: list[tuple[str, dict]] = []

        class _FakeAudit:
            def record(self, verb, **fields):  # noqa: D401
                recorded.append((verb, fields))

        with mock.patch.object(self.web_ui, "_get_audit_log", return_value=_FakeAudit()):
            r = self.client.get("/config/download")
        self.assertEqual(r.status_code, 200)
        verbs = [v for v, _ in recorded]
        self.assertIn("config_download", verbs)
        # The audit entry carries the SHA so post-mortem can correlate.
        download_record = next(f for v, f in recorded if v == "config_download")
        self.assertEqual(download_record["unit_id"], "lifetrac-001")
        self.assertEqual(len(download_record["config_sha256"]), 64)


# --------------------------------------------------------------- BC10b_D


class BC10b_D_SourceTripwire(unittest.TestCase):
    """CLI script exists with the four subcommands; web_ui carries the route."""

    def test_cli_script_exists(self) -> None:
        self.assertTrue(_CLI.is_file(), msg=f"missing {_CLI}")

    def test_cli_subcommands_present(self) -> None:
        text = _CLI.read_text(encoding="utf-8")
        for marker in ("cmd_validate", "cmd_bundle", "cmd_verify", "cmd_diff", "BC-10"):
            self.assertIn(marker, text, msg=f"CLI missing {marker!r}")

    def test_web_ui_carries_download_route(self) -> None:
        text = (_BS / "web_ui.py").read_text(encoding="utf-8")
        for marker in ('"/config/download"', "config_download", "config_bundle", "BC-10"):
            self.assertIn(marker, text, msg=f"web_ui missing {marker!r}")

    def test_config_bundle_module_exists(self) -> None:
        path = _BS / "config_bundle.py"
        self.assertTrue(path.is_file(), msg=f"missing {path}")
        text = path.read_text(encoding="utf-8")
        for marker in ("BUNDLE_VERSION", "BODY_SENTINEL", "make_bundle", "parse", "verify_filename_matches"):
            self.assertIn(marker, text)


if __name__ == "__main__":
    unittest.main()
