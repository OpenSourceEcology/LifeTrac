"""Tests for the PWA service-worker and manifest routes.

Covers the /sw.js and /manifest.json root-level endpoints added to
enable Progressive Web App features on the base-station operator console.
"""

from __future__ import annotations

import importlib
import json
import os
import sys
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

try:
    import paho.mqtt.client  # noqa: F401
    import fastapi           # noqa: F401
    from fastapi.testclient import TestClient  # noqa: F401
except ImportError:
    raise unittest.SkipTest("paho-mqtt + fastapi required for web_ui PWA tests")


class PwaRoutesTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        os.environ["LIFETRAC_PIN"] = "424242"
        with mock.patch("paho.mqtt.client.Client") as mqtt_class:
            instance = mqtt_class.return_value
            instance.connect = mock.MagicMock()
            instance.loop_start = mock.MagicMock()
            instance.subscribe = mock.MagicMock()
            instance.publish = mock.MagicMock()
            import web_ui
            importlib.reload(web_ui)
            cls.client = TestClient(web_ui.app)

    # ---- service worker ------------------------------------------------

    def test_sw_js_returns_200(self):
        r = self.client.get("/sw.js")
        self.assertEqual(r.status_code, 200)

    def test_sw_js_content_type_is_javascript(self):
        r = self.client.get("/sw.js")
        self.assertIn("javascript", r.headers.get("content-type", ""))

    def test_sw_js_service_worker_allowed_header_is_root(self):
        """Service-Worker-Allowed must be '/' so the SW can control all pages."""
        r = self.client.get("/sw.js")
        self.assertEqual(r.headers.get("service-worker-allowed"), "/")

    def test_sw_js_cache_control_prevents_caching(self):
        """The SW file itself must never be cached by the browser."""
        r = self.client.get("/sw.js")
        cc = r.headers.get("cache-control", "")
        self.assertIn("no-cache", cc)

    def test_sw_js_body_contains_cache_name(self):
        r = self.client.get("/sw.js")
        self.assertIn("CACHE_NAME", r.text)

    def test_sw_js_never_intercepts_api_or_ws(self):
        """The SW source must explicitly skip /api/ and /ws/ paths."""
        r = self.client.get("/sw.js")
        self.assertIn("/api/", r.text)
        self.assertIn("/ws/", r.text)

    # ---- manifest -------------------------------------------------------

    def test_manifest_returns_200(self):
        r = self.client.get("/manifest.json")
        self.assertEqual(r.status_code, 200)

    def test_manifest_content_type(self):
        r = self.client.get("/manifest.json")
        ct = r.headers.get("content-type", "")
        self.assertIn("manifest+json", ct)

    def test_manifest_required_fields(self):
        r = self.client.get("/manifest.json")
        m = r.json()
        self.assertIn("name", m)
        self.assertIn("short_name", m)
        self.assertIn("start_url", m)
        self.assertIn("display", m)
        self.assertIn("icons", m)

    def test_manifest_start_url_is_root(self):
        r = self.client.get("/manifest.json")
        self.assertEqual(r.json()["start_url"], "/")

    def test_manifest_icons_reference_static_path(self):
        r = self.client.get("/manifest.json")
        icons = r.json().get("icons", [])
        self.assertTrue(len(icons) >= 1, "manifest must declare at least one icon")
        for icon in icons:
            self.assertTrue(
                icon["src"].startswith("/static/"),
                f"icon src should be under /static/: {icon['src']}",
            )

    def test_manifest_display_is_standalone_or_fullscreen(self):
        r = self.client.get("/manifest.json")
        self.assertIn(r.json()["display"], ("standalone", "fullscreen", "minimal-ui"))

    # ---- offline page via static mount ----------------------------------

    def test_offline_html_accessible(self):
        """The offline fallback page must be reachable without a session cookie."""
        r = self.client.get("/static/offline.html")
        self.assertEqual(r.status_code, 200)
        self.assertIn("text/html", r.headers.get("content-type", ""))

    def test_offline_html_has_reload_button(self):
        r = self.client.get("/static/offline.html")
        self.assertIn("reload", r.text.lower())

    # ---- icon accessible -----------------------------------------------

    def test_icon_svg_accessible(self):
        r = self.client.get("/static/icons/icon.svg")
        self.assertEqual(r.status_code, 200)
        self.assertIn("svg", r.headers.get("content-type", "").lower())

    # ---- /cert.pem download endpoint ------------------------------------

    def test_cert_pem_returns_404_when_no_cert(self):
        """Returns 404 with a hint when no TLS cert has been generated."""
        import web_ui as _wu
        orig = _wu.CERT_PATH
        try:
            _wu.CERT_PATH = Path("/nonexistent/path/cert.pem")
            r = self.client.get("/cert.pem")
            self.assertEqual(r.status_code, 404)
        finally:
            _wu.CERT_PATH = orig

    def test_cert_pem_returns_pem_file_when_cert_exists(self):
        """Returns the certificate file with correct headers when present."""
        import tempfile
        import web_ui as _wu
        orig = _wu.CERT_PATH
        try:
            with tempfile.NamedTemporaryFile(suffix=".pem", delete=False) as f:
                f.write(b"-----BEGIN CERTIFICATE-----\nFAKE\n-----END CERTIFICATE-----\n")
                tmp_cert = Path(f.name)
            _wu.CERT_PATH = tmp_cert
            r = self.client.get("/cert.pem")
            self.assertEqual(r.status_code, 200)
            self.assertIn("pem", r.headers.get("content-type", "").lower())
            self.assertIn("attachment", r.headers.get("content-disposition", "").lower())
            self.assertIn("lifetrac-cert.pem", r.headers.get("content-disposition", ""))
            self.assertIn(b"BEGIN CERTIFICATE", r.content)
        finally:
            _wu.CERT_PATH = orig
            tmp_cert.unlink(missing_ok=True)

    def test_cert_pem_never_caches(self):
        """Cache-Control must be no-store so stale certs are never used."""
        import tempfile
        import web_ui as _wu
        orig = _wu.CERT_PATH
        try:
            with tempfile.NamedTemporaryFile(suffix=".pem", delete=False) as f:
                f.write(b"-----BEGIN CERTIFICATE-----\nFAKE\n-----END CERTIFICATE-----\n")
                tmp_cert = Path(f.name)
            _wu.CERT_PATH = tmp_cert
            r = self.client.get("/cert.pem")
            self.assertIn("no-store", r.headers.get("cache-control", ""))
        finally:
            _wu.CERT_PATH = orig
            tmp_cert.unlink(missing_ok=True)

    # ---- /setup page ----------------------------------------------------

    def test_setup_page_accessible_without_session(self):
        """/setup must be reachable without a login cookie."""
        r = self.client.get("/setup")
        self.assertEqual(r.status_code, 200)
        self.assertIn("text/html", r.headers.get("content-type", ""))

    def test_setup_page_injects_https_port(self):
        """The server must inject the HTTPS port into the page."""
        import web_ui as _wu
        orig = _wu.HTTPS_PORT
        try:
            _wu.HTTPS_PORT = 9443
            r = self.client.get("/setup")
            self.assertIn("9443", r.text)
        finally:
            _wu.HTTPS_PORT = orig

    def test_setup_page_cert_available_true_when_cert_exists(self):
        """cert_available flag is 'true' when the cert file is present."""
        import tempfile
        import web_ui as _wu
        orig = _wu.CERT_PATH
        try:
            with tempfile.NamedTemporaryFile(suffix=".pem", delete=False) as f:
                f.write(b"-----BEGIN CERTIFICATE-----\nFAKE\n-----END CERTIFICATE-----\n")
                tmp_cert = Path(f.name)
            _wu.CERT_PATH = tmp_cert
            r = self.client.get("/setup")
            self.assertIn("true", r.text)
        finally:
            _wu.CERT_PATH = orig
            tmp_cert.unlink(missing_ok=True)

    def test_setup_page_cert_available_false_when_no_cert(self):
        """cert_available flag is 'false' when no cert has been generated."""
        import web_ui as _wu
        orig = _wu.CERT_PATH
        try:
            _wu.CERT_PATH = Path("/nonexistent/path/cert.pem")
            r = self.client.get("/setup")
            self.assertIn("false", r.text)
        finally:
            _wu.CERT_PATH = orig

    def test_setup_page_not_cached(self):
        """Setup page must not be cached (cert state may change)."""
        r = self.client.get("/setup")
        self.assertIn("no-store", r.headers.get("cache-control", ""))

    def test_setup_page_contains_cert_download_link(self):
        r = self.client.get("/setup")
        self.assertIn("/cert.pem", r.text)

    def test_setup_page_contains_android_ios_instructions(self):
        r = self.client.get("/setup")
        self.assertIn("Android", r.text)
        self.assertIn("iPhone", r.text)


class AutoCertTests(unittest.TestCase):
    """Tests for _ensure_self_signed_cert() first-boot auto-generation."""

    def _import_wu(self):
        import web_ui as _wu
        return _wu

    def test_no_op_when_cert_exists(self):
        """Does nothing when CERT_PATH already points to an existing file."""
        import tempfile
        _wu = self._import_wu()
        orig_cert = _wu.CERT_PATH
        orig_key = _wu.KEY_PATH
        try:
            with tempfile.NamedTemporaryFile(suffix=".pem", delete=False) as f:
                f.write(b"-----BEGIN CERTIFICATE-----\nFAKE\n-----END CERTIFICATE-----\n")
                tmp_cert_path = Path(f.name)
            _wu.CERT_PATH = tmp_cert_path
            _wu.KEY_PATH = Path("/nonexistent/key.pem")
            with mock.patch("subprocess.run") as mock_run:
                _wu._ensure_self_signed_cert()
                mock_run.assert_not_called()
        finally:
            _wu.CERT_PATH = orig_cert
            _wu.KEY_PATH = orig_key
            tmp_cert_path.unlink(missing_ok=True)

    def test_no_op_when_key_exists(self):
        """Does nothing when KEY_PATH already points to an existing file."""
        import tempfile
        _wu = self._import_wu()
        orig_cert = _wu.CERT_PATH
        orig_key = _wu.KEY_PATH
        try:
            with tempfile.NamedTemporaryFile(suffix=".pem", delete=False) as f:
                f.write(b"-----BEGIN RSA PRIVATE KEY-----\nFAKE\n-----END RSA PRIVATE KEY-----\n")
                tmp_key_path = Path(f.name)
            _wu.CERT_PATH = Path("/nonexistent/cert.pem")
            _wu.KEY_PATH = tmp_key_path
            with mock.patch("subprocess.run") as mock_run:
                _wu._ensure_self_signed_cert()
                mock_run.assert_not_called()
        finally:
            _wu.CERT_PATH = orig_cert
            _wu.KEY_PATH = orig_key
            tmp_key_path.unlink(missing_ok=True)

    def test_calls_openssl_when_no_cert_or_key(self):
        """Calls openssl when neither cert nor key exists."""
        import tempfile
        _wu = self._import_wu()
        orig_cert = _wu.CERT_PATH
        orig_key = _wu.KEY_PATH
        try:
            with tempfile.TemporaryDirectory() as tmp_dir:
                _wu.CERT_PATH = Path(tmp_dir) / "cert.pem"
                _wu.KEY_PATH = Path(tmp_dir) / "key.pem"
                mock_result = mock.MagicMock()
                mock_result.returncode = 0
                with mock.patch("subprocess.run", return_value=mock_result) as mock_run:
                    _wu._ensure_self_signed_cert()
                    mock_run.assert_called_once()
                    cmd = mock_run.call_args[0][0]
                    self.assertEqual(cmd[0], "openssl")
                    self.assertIn("-x509", cmd)
                    self.assertIn(str(_wu.CERT_PATH), cmd)
                    self.assertIn(str(_wu.KEY_PATH), cmd)
        finally:
            _wu.CERT_PATH = orig_cert
            _wu.KEY_PATH = orig_key

    def test_openssl_subj_uses_hostname(self):
        """The -subj CN must match the current hostname."""
        import socket
        import tempfile
        _wu = self._import_wu()
        orig_cert = _wu.CERT_PATH
        orig_key = _wu.KEY_PATH
        try:
            with tempfile.TemporaryDirectory() as tmp_dir:
                _wu.CERT_PATH = Path(tmp_dir) / "cert.pem"
                _wu.KEY_PATH = Path(tmp_dir) / "key.pem"
                mock_result = mock.MagicMock()
                mock_result.returncode = 0
                with mock.patch("subprocess.run", return_value=mock_result) as mock_run:
                    _wu._ensure_self_signed_cert()
                    cmd = mock_run.call_args[0][0]
                    subj_idx = cmd.index("-subj") + 1
                    self.assertIn(socket.gethostname(), cmd[subj_idx])
        finally:
            _wu.CERT_PATH = orig_cert
            _wu.KEY_PATH = orig_key

    def test_graceful_when_openssl_missing(self):
        """Logs a warning and returns without raising when openssl is absent."""
        import tempfile
        _wu = self._import_wu()
        orig_cert = _wu.CERT_PATH
        orig_key = _wu.KEY_PATH
        try:
            with tempfile.TemporaryDirectory() as tmp_dir:
                _wu.CERT_PATH = Path(tmp_dir) / "cert.pem"
                _wu.KEY_PATH = Path(tmp_dir) / "key.pem"
                with mock.patch("subprocess.run", side_effect=FileNotFoundError("openssl")):
                    # Must not raise
                    _wu._ensure_self_signed_cert()
        finally:
            _wu.CERT_PATH = orig_cert
            _wu.KEY_PATH = orig_key

    def test_graceful_on_subprocess_error(self):
        """Logs a warning and returns without raising on unexpected subprocess errors."""
        import tempfile
        _wu = self._import_wu()
        orig_cert = _wu.CERT_PATH
        orig_key = _wu.KEY_PATH
        try:
            with tempfile.TemporaryDirectory() as tmp_dir:
                _wu.CERT_PATH = Path(tmp_dir) / "cert.pem"
                _wu.KEY_PATH = Path(tmp_dir) / "key.pem"
                with mock.patch("subprocess.run", side_effect=OSError("permission denied")):
                    _wu._ensure_self_signed_cert()
        finally:
            _wu.CERT_PATH = orig_cert
            _wu.KEY_PATH = orig_key

    def test_key_permissions_set_to_600_on_success(self):
        """Private key gets chmod 600 after generation."""
        import tempfile
        import stat
        _wu = self._import_wu()
        orig_cert = _wu.CERT_PATH
        orig_key = _wu.KEY_PATH
        try:
            with tempfile.TemporaryDirectory() as tmp_dir:
                cert_path = Path(tmp_dir) / "cert.pem"
                key_path = Path(tmp_dir) / "key.pem"
                _wu.CERT_PATH = cert_path
                _wu.KEY_PATH = key_path

                def _fake_openssl(cmd, **_kw):
                    # Create the files openssl would normally create
                    cert_path.write_text("FAKE CERT\n")
                    key_path.write_text("FAKE KEY\n")
                    r = mock.MagicMock()
                    r.returncode = 0
                    return r

                with mock.patch("subprocess.run", side_effect=_fake_openssl):
                    _wu._ensure_self_signed_cert()

                mode = oct(stat.S_IMODE(os.stat(key_path).st_mode))
                self.assertEqual(mode, oct(0o600))
        finally:
            _wu.CERT_PATH = orig_cert
            _wu.KEY_PATH = orig_key


if __name__ == "__main__":
    unittest.main()
