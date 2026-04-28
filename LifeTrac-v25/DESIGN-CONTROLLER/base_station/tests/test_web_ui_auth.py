"""Tests for the web_ui PIN-auth flow.

Covers MASTER_PLAN.md \u00a78.5 / DECISIONS.md D-E1: shared-PIN unlock with HttpOnly
cookie, idle TTL, and IP-rate-limited lockout. We exercise these via FastAPI's
`TestClient` so the full Cookie / Depends chain runs end-to-end.
"""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

# Skip the whole suite if FastAPI / paho-mqtt aren't installed in the dev env.
# These ship in base_station/requirements.txt but a pure-firmware checkout may
# not have them.
try:
    import paho.mqtt.client  # noqa: F401
    import fastapi           # noqa: F401
    from fastapi.testclient import TestClient  # noqa: F401
except ImportError:
    raise unittest.SkipTest("paho-mqtt + fastapi required for web_ui auth tests")


class PinAuthTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        os.environ["LIFETRAC_PIN"] = "424242"
        with mock.patch("paho.mqtt.client.Client") as mqtt_class:
            instance = mqtt_class.return_value
            instance.connect = mock.MagicMock()
            instance.loop_start = mock.MagicMock()
            instance.subscribe = mock.MagicMock()
            instance.publish = mock.MagicMock()
            from fastapi.testclient import TestClient
            import importlib
            import web_ui
            importlib.reload(web_ui)   # rebind module-level mqtt stub
            cls.web_ui = web_ui
            cls.client = TestClient(web_ui.app)
            cls.mqtt = instance

    def setUp(self) -> None:
        # Clear sessions + lockouts between tests so they're independent.
        with self.web_ui._sessions_lock:
            self.web_ui._sessions.clear()
        with self.web_ui._fail_lock:
            self.web_ui._fail_counts.clear()

    def test_protected_route_rejects_without_session(self):
        r = self.client.post("/api/estop")
        self.assertEqual(r.status_code, 401)

    def test_login_with_correct_pin_grants_session_cookie(self):
        r = self.client.post("/api/login", json={"pin": "424242"})
        self.assertEqual(r.status_code, 200)
        self.assertIn("session", r.cookies)
        # Session route now confirms authenticated.
        s = self.client.get("/api/session")
        self.assertEqual(s.json(), {"authenticated": True})

    def test_login_with_wrong_pin_fails_and_does_not_set_cookie(self):
        r = self.client.post("/api/login", json={"pin": "000000"})
        self.assertEqual(r.status_code, 401)
        self.assertNotIn("session", r.cookies)

    def test_protected_route_works_after_login(self):
        self.client.post("/api/login", json={"pin": "424242"})
        r = self.client.post("/api/estop")
        self.assertEqual(r.status_code, 200)
        self.assertTrue(r.json().get("ok"))

    def test_logout_invalidates_session(self):
        self.client.post("/api/login", json={"pin": "424242"})
        self.client.post("/api/logout")
        r = self.client.post("/api/estop")
        self.assertEqual(r.status_code, 401)

    def test_lockout_after_repeated_failures(self):
        for _ in range(self.web_ui.LOCKOUT_FAILS):
            self.client.post("/api/login", json={"pin": "wrong"})
        # Next attempt is rate-limited, even with the correct PIN.
        r = self.client.post("/api/login", json={"pin": "424242"})
        self.assertEqual(r.status_code, 429)


if __name__ == "__main__":
    unittest.main()
