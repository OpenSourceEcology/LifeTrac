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


if __name__ == "__main__":
    unittest.main()
