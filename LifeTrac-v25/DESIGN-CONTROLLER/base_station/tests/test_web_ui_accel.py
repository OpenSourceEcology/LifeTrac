"""Tests for /api/settings/accel routes (Coral toggle)."""

from __future__ import annotations

import os
import sys
import tempfile
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
    raise unittest.SkipTest("paho-mqtt + fastapi required for accel UI tests")


class AccelSettingsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        os.environ["LIFETRAC_PIN"] = "131313"
        cls._tmp = tempfile.NamedTemporaryFile(
            suffix=".json", delete=False, mode="w"
        )
        cls._tmp.write("{}")
        cls._tmp.close()
        os.environ["LIFETRAC_BASE_SETTINGS"] = cls._tmp.name

        with mock.patch("paho.mqtt.client.Client") as mqtt_class:
            instance = mqtt_class.return_value
            instance.connect = mock.MagicMock()
            instance.loop_start = mock.MagicMock()
            instance.subscribe = mock.MagicMock()
            instance.publish = mock.MagicMock()

            import importlib
            import settings_store
            importlib.reload(settings_store)
            from image_pipeline import accel_select
            importlib.reload(accel_select)
            accel_select.reset_for_tests()

            # Substitute a fake accelerator so detection is deterministic.
            cls._fake_state = {"present": True, "kind": "coral_m2",
                               "warmup_ok": True}

            def fake_detect():
                return (cls._fake_state["present"], cls._fake_state["kind"])

            def fake_warmup():
                return (cls._fake_state["warmup_ok"], None)

            fake_accel = accel_select.Accelerator(
                enabled_loader=lambda: bool(
                    settings_store.get_store().get("image.coral_enabled", True)),
                detect_fn=fake_detect,
                warmup_fn=fake_warmup,
            )
            with accel_select._singleton_lock:
                accel_select._singleton = fake_accel
            fake_accel.refresh()

            import web_ui
            importlib.reload(web_ui)
            cls.web_ui = web_ui
            cls.client = TestClient(web_ui.app)

    @classmethod
    def tearDownClass(cls) -> None:
        try:
            os.unlink(cls._tmp.name)
        except OSError:
            pass

    def setUp(self) -> None:
        # Reset auth state.
        with self.web_ui._sessions_lock:
            self.web_ui._sessions.clear()
        with self.web_ui._fail_lock:
            self.web_ui._fail_counts.clear()
        # Always start each test with operator switch ON.
        from settings_store import get_store
        get_store().set("image.coral_enabled", True)
        AccelSettingsTests._fake_state["present"] = True
        AccelSettingsTests._fake_state["warmup_ok"] = True
        from image_pipeline.accel_select import get_accelerator
        get_accelerator().refresh()

    def _login(self) -> None:
        r = self.client.post("/api/login", json={"pin": "131313"})
        self.assertEqual(r.status_code, 200)

    # ---- tests ----
    def test_get_requires_auth(self):
        r = self.client.get("/api/settings/accel")
        self.assertEqual(r.status_code, 401)

    def test_get_returns_full_shape_when_authed(self):
        self._login()
        r = self.client.get("/api/settings/accel")
        self.assertEqual(r.status_code, 200)
        body = r.json()
        for k in ("present", "usable", "enabled", "kind",
                  "last_error", "detected_at", "active"):
            self.assertIn(k, body, f"missing field {k}")
        self.assertTrue(body["present"])
        self.assertTrue(body["active"])
        self.assertEqual(body["kind"], "coral_m2")

    def test_post_toggle_off_persists_and_deactivates(self):
        self._login()
        r = self.client.post("/api/settings/accel", json={"enabled": False})
        self.assertEqual(r.status_code, 200)
        body = r.json()
        self.assertFalse(body["enabled"])
        self.assertFalse(body["active"])
        # Round-trip: GET reflects the new value.
        r2 = self.client.get("/api/settings/accel")
        self.assertFalse(r2.json()["enabled"])
        self.assertFalse(r2.json()["active"])

    def test_post_refresh_runs_redetect(self):
        self._login()
        # Simulate hot-unplug between detection passes.
        AccelSettingsTests._fake_state["present"] = False
        r = self.client.post("/api/settings/accel/refresh")
        self.assertEqual(r.status_code, 200)
        body = r.json()
        self.assertFalse(body["present"])
        self.assertFalse(body["active"])

    def test_post_toggle_requires_auth(self):
        r = self.client.post("/api/settings/accel", json={"enabled": False})
        self.assertEqual(r.status_code, 401)
        r2 = self.client.post("/api/settings/accel/refresh")
        self.assertEqual(r2.status_code, 401)


if __name__ == "__main__":
    unittest.main()
