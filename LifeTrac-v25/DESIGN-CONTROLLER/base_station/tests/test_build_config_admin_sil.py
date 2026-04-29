"""SIL gate for Round 30 -- IP: BC-05 (web admin form for build config).

Round 30 lands the operator-facing TOML editor on the base UI:
``GET /config`` (the page itself), ``GET /api/build_config/source``
(seeds the textarea), ``POST /api/build_config/preview-diff`` (no
side effects -- shows would-be reload class), and
``POST /api/build_config/upload`` (schema-validate, refuse
firmware-required, atomic write, audit). The X8-style atomic-write +
firmware-required-refusal contract from BC-10 is reused here byte-for-
byte (``installer_daemon._atomic_write`` + same ``diff_reload_classes``
classifier), so a TOML the editor accepts is the same set of bytes the
X8 installer would accept.

BC05_A_PageRouting
    ``GET /config`` requires a session (303 redirect when unauth, the
    HTML body contains the documented widget IDs when authed).

BC05_B_SourceEndpoint
    ``GET /api/build_config/source`` returns the raw TOML body
    verbatim (no transport mangling) and 401s without a session.

BC05_C_PreviewDiff
    Identical body returns no changed leaves. Live-class change
    classifies as live. Restart-required change classifies as
    restart_required. Schema violation returns ``ok: false`` with an
    error string. No write happens (target file unchanged).

BC05_D_Upload
    Identical body upload still schema-validates and audits with a
    no-leaves diff. Live-class upload writes the new bytes atomically
    and audits with worst_reload_class == "live". Schema violation
    returns ``ok: false`` and target untouched. unit_id mismatch
    rejected. Firmware-required upload rejected without writing.

BC05_E_SourceTripwire
    web_ui carries the four BC-05 endpoints + ``config_upload`` audit
    verb + ``BC-05`` marker; the ``/config`` HTML page references the
    documented sibling routes.
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
_WEB_DIR = _BS / "web"


def _read_default_body() -> str:
    return _DEFAULT_TOML.read_text(encoding="utf-8")


def _import_web_ui_with(toml_path: Path):
    """Re-import web_ui pinned to ``toml_path`` for this test.

    Pops watcher + build_config from sys.modules so the watcher's
    initial fingerprint is anchored at the right file.
    """
    for mod in ("web_ui", "build_config", "config_watcher", "installer_daemon"):
        sys.modules.pop(mod, None)
    env = dict(os.environ)
    env["LIFETRAC_BUILD_CONFIG_PATH"] = str(toml_path)
    env["LIFETRAC_PIN"] = "1234"
    env.pop("LIFETRAC_PIN_FILE", None)
    with mock.patch.dict(os.environ, env, clear=True), \
         mock.patch("paho.mqtt.client.Client") as cls:
        cls.return_value.connect.return_value = None
        cls.return_value.loop_start.return_value = None
        web_ui = importlib.import_module("web_ui")
    return web_ui


def _client_logged_in(toml_path: Path):
    from fastapi.testclient import TestClient
    web_ui = _import_web_ui_with(toml_path)
    client = TestClient(web_ui.app)
    r = client.post("/api/login", json={"pin": "1234"})
    assert r.status_code == 200, r.text
    return client, web_ui


# ------------------------------------------------------------- BC05_A


class BC05_A_PageRouting(unittest.TestCase):

    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="lt-bc05-page-"))
        self.toml = self.tmp / "build.toml"
        self.toml.write_text(_read_default_body(), encoding="utf-8")

    def test_config_page_requires_session(self) -> None:
        from fastapi.testclient import TestClient
        web_ui = _import_web_ui_with(self.toml)
        client = TestClient(web_ui.app)
        r = client.get("/config", follow_redirects=False)
        # 303 redirect to /login (same pattern as /settings, /audit, etc.)
        self.assertEqual(r.status_code, 303)
        self.assertEqual(r.headers.get("location"), "/login")

    def test_config_page_authed_returns_widget_ids(self) -> None:
        client, _ = _client_logged_in(self.toml)
        r = client.get("/config")
        self.assertEqual(r.status_code, 200)
        body = r.text
        for widget in ("editor", "preview-btn", "upload-btn",
                       "diff-banner", "watcher-pill", "meta-sha"):
            self.assertIn(f'id="{widget}"', body, msg=widget)


# ------------------------------------------------------------- BC05_B


class BC05_B_SourceEndpoint(unittest.TestCase):

    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="lt-bc05-src-"))
        self.toml = self.tmp / "build.toml"
        self.toml.write_text(_read_default_body(), encoding="utf-8")

    def test_source_requires_session(self) -> None:
        from fastapi.testclient import TestClient
        web_ui = _import_web_ui_with(self.toml)
        client = TestClient(web_ui.app)
        r = client.get("/api/build_config/source")
        self.assertEqual(r.status_code, 401)

    def test_source_returns_verbatim_body(self) -> None:
        client, _ = _client_logged_in(self.toml)
        r = client.get("/api/build_config/source")
        self.assertEqual(r.status_code, 200)
        self.assertEqual(r.text, _read_default_body())
        self.assertEqual(r.headers.get("cache-control"), "no-store")


# ------------------------------------------------------------- BC05_C


class BC05_C_PreviewDiff(unittest.TestCase):

    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="lt-bc05-prev-"))
        self.toml = self.tmp / "build.toml"
        self.toml.write_text(_read_default_body(), encoding="utf-8")

    def test_identical_body_no_changed_leaves(self) -> None:
        client, _ = _client_logged_in(self.toml)
        r = client.post("/api/build_config/preview-diff",
                        content=_read_default_body(),
                        headers={"Content-Type": "text/plain"})
        self.assertEqual(r.status_code, 200, r.text)
        d = r.json()
        self.assertTrue(d["ok"])
        self.assertEqual(d["changed_leaves"], [])
        self.assertIsNone(d["worst_reload_class"])
        self.assertEqual(d["candidate_sha256"], d["running_sha256"])

    def test_live_change_classified_live(self) -> None:
        client, _ = _client_logged_in(self.toml)
        body = _read_default_body().replace(
            "max_control_subscribers   = 4",
            "max_control_subscribers   = 6",
        )
        self.assertNotEqual(body, _read_default_body(), msg="ms no-op")
        r = client.post("/api/build_config/preview-diff", content=body,
                        headers={"Content-Type": "text/plain"})
        d = r.json()
        self.assertTrue(d["ok"])
        self.assertEqual(d["worst_reload_class"], "live")
        self.assertIn("ui.max_control_subscribers", d["changed_leaves"])

    def test_restart_required_change_classified(self) -> None:
        client, _ = _client_logged_in(self.toml)
        body = _read_default_body().replace(
            'lora_region              = "us915"',
            'lora_region              = "eu868"',
        )
        self.assertNotEqual(body, _read_default_body(), msg="lora no-op")
        r = client.post("/api/build_config/preview-diff", content=body,
                        headers={"Content-Type": "text/plain"})
        d = r.json()
        self.assertTrue(d["ok"])
        self.assertEqual(d["worst_reload_class"], "restart_required")

    def test_schema_violation_returns_ok_false_no_write(self) -> None:
        client, _ = _client_logged_in(self.toml)
        body = _read_default_body().replace(
            "track_axis_count    = 2",
            "track_axis_count    = 7",
        )
        self.assertNotEqual(body, _read_default_body(), msg="schema no-op")
        before = self.toml.read_text(encoding="utf-8")
        r = client.post("/api/build_config/preview-diff", content=body,
                        headers={"Content-Type": "text/plain"})
        d = r.json()
        self.assertFalse(d["ok"])
        self.assertIn("error", d)
        # File on disk untouched -- preview is no-side-effects.
        self.assertEqual(self.toml.read_text(encoding="utf-8"), before)


# ------------------------------------------------------------- BC05_D


class BC05_D_Upload(unittest.TestCase):

    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="lt-bc05-up-"))
        self.toml = self.tmp / "build.toml"
        self.toml.write_text(_read_default_body(), encoding="utf-8")

    def test_identical_upload_succeeds_no_changed_leaves(self) -> None:
        client, _ = _client_logged_in(self.toml)
        r = client.post("/api/build_config/upload",
                        content=_read_default_body(),
                        headers={"Content-Type": "text/plain"})
        d = r.json()
        self.assertTrue(d["ok"])
        self.assertEqual(d["changed_leaves"], [])
        self.assertIsNone(d["worst_reload_class"])

    def test_live_upload_atomically_rewrites_target(self) -> None:
        client, _ = _client_logged_in(self.toml)
        body = _read_default_body().replace(
            "max_control_subscribers   = 4",
            "max_control_subscribers   = 6",
        )
        self.assertNotEqual(body, _read_default_body(), msg="ms no-op")
        r = client.post("/api/build_config/upload", content=body,
                        headers={"Content-Type": "text/plain"})
        d = r.json()
        self.assertTrue(d["ok"], msg=d)
        self.assertEqual(d["worst_reload_class"], "live")
        # Bytes on disk are exactly what we sent.
        self.assertEqual(self.toml.read_text(encoding="utf-8"), body)

    def test_schema_violation_upload_rejected_no_write(self) -> None:
        client, _ = _client_logged_in(self.toml)
        body = _read_default_body().replace(
            "track_axis_count    = 2",
            "track_axis_count    = 7",
        )
        self.assertNotEqual(body, _read_default_body(), msg="schema no-op")
        before = self.toml.read_text(encoding="utf-8")
        r = client.post("/api/build_config/upload", content=body,
                        headers={"Content-Type": "text/plain"})
        d = r.json()
        self.assertFalse(d["ok"])
        self.assertEqual(self.toml.read_text(encoding="utf-8"), before)

    def test_unit_id_mismatch_upload_rejected(self) -> None:
        client, _ = _client_logged_in(self.toml)
        body = _read_default_body().replace(
            'unit_id        = "lifetrac-001"',
            'unit_id        = "lifetrac-zzz"',
        )
        self.assertNotEqual(body, _read_default_body(), msg="unit no-op")
        before = self.toml.read_text(encoding="utf-8")
        r = client.post("/api/build_config/upload", content=body,
                        headers={"Content-Type": "text/plain"})
        d = r.json()
        self.assertFalse(d["ok"])
        self.assertIn("unit_id", d["error"])
        self.assertEqual(self.toml.read_text(encoding="utf-8"), before)

    def test_firmware_required_upload_rejected_no_write(self) -> None:
        client, _ = _client_logged_in(self.toml)
        body = _read_default_body().replace(
            "m4_watchdog_ms          = 200",
            "m4_watchdog_ms          = 250",
        )
        self.assertNotEqual(body, _read_default_body(), msg="watchdog no-op")
        before = self.toml.read_text(encoding="utf-8")
        r = client.post("/api/build_config/upload", content=body,
                        headers={"Content-Type": "text/plain"})
        d = r.json()
        self.assertFalse(d["ok"])
        self.assertEqual(d["worst_reload_class"], "firmware_required")
        self.assertEqual(self.toml.read_text(encoding="utf-8"), before)

    def test_upload_requires_session(self) -> None:
        from fastapi.testclient import TestClient
        web_ui = _import_web_ui_with(self.toml)
        client = TestClient(web_ui.app)
        r = client.post("/api/build_config/upload",
                        content=_read_default_body(),
                        headers={"Content-Type": "text/plain"})
        self.assertEqual(r.status_code, 401)


# ------------------------------------------------------------- BC05_E


class BC05_E_SourceTripwire(unittest.TestCase):

    def test_web_ui_carries_admin_routes(self) -> None:
        src = (_BS / "web_ui.py").read_text(encoding="utf-8")
        for needle in (
            '@app.get("/config"',
            "/api/build_config/source",
            "/api/build_config/preview-diff",
            "/api/build_config/upload",
            "config_upload",
            "BC-05",
        ):
            self.assertIn(needle, src, msg=needle)

    def test_config_html_references_sibling_routes(self) -> None:
        page = (_WEB_DIR / "config.html").read_text(encoding="utf-8")
        for needle in (
            "/api/build_config/state",
            "/api/build_config/source",
            "/api/build_config/preview-diff",
            "/api/build_config/upload",
            "/config/download",
        ):
            self.assertIn(needle, page, msg=needle)


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
