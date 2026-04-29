"""Tests for IP-203 input validation and IP-202 subscriber bounds.

Exercises the Pydantic models bolted onto the LAN-side surfaces and the
4429-on-overflow behaviour of the bounded WS subscriber sets.
"""

from __future__ import annotations

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
    raise unittest.SkipTest("paho-mqtt + fastapi required for web_ui tests")


def _bootstrap_web_ui():
    os.environ["LIFETRAC_PIN"] = "424242"
    os.environ["LIFETRAC_ALLOW_UNCONFIGURED_KEY"] = "1"
    with mock.patch("paho.mqtt.client.Client") as mqtt_class:
        instance = mqtt_class.return_value
        instance.connect = mock.MagicMock()
        instance.loop_start = mock.MagicMock()
        instance.subscribe = mock.MagicMock()
        instance.publish = mock.MagicMock()
        import importlib
        import web_ui
        importlib.reload(web_ui)
        return web_ui, TestClient(web_ui.app), instance


class ValidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.web_ui, cls.client, cls.mqtt = _bootstrap_web_ui()

    def setUp(self) -> None:
        with self.web_ui._sessions_lock:
            self.web_ui._sessions.clear()
        with self.web_ui._fail_lock:
            self.web_ui._fail_counts.clear()

    def _login(self):
        r = self.client.post("/api/login", json={"pin": "424242"})
        self.assertEqual(r.status_code, 200, r.text)

    # ---- /api/login --------------------------------------------------
    def test_login_rejects_extra_fields(self):
        r = self.client.post("/api/login",
                             json={"pin": "424242", "evil": True})
        self.assertEqual(r.status_code, 422)

    def test_login_rejects_missing_pin(self):
        r = self.client.post("/api/login", json={})
        self.assertEqual(r.status_code, 422)

    def test_login_rejects_oversized_pin(self):
        r = self.client.post("/api/login", json={"pin": "x" * 64})
        self.assertEqual(r.status_code, 422)

    # ---- /api/camera/select ------------------------------------------
    def test_camera_select_rejects_extra_fields(self):
        self._login()
        r = self.client.post("/api/camera/select",
                             json={"camera": "front", "x": 1})
        self.assertEqual(r.status_code, 422)

    def test_camera_select_rejects_unknown_name(self):
        self._login()
        r = self.client.post("/api/camera/select",
                             json={"camera": "wormhole"})
        self.assertEqual(r.status_code, 400)

    def test_camera_select_accepts_known(self):
        self._login()
        r = self.client.post("/api/camera/select", json={"camera": "front"})
        self.assertEqual(r.status_code, 200)
        self.assertEqual(r.json()["camera"], "front")

    # ---- /api/settings/accel -----------------------------------------
    def test_accel_set_requires_bool(self):
        self._login()
        r = self.client.post("/api/settings/accel", json={"enabled": "yes"})
        # Pydantic v2 coerces "yes"/"no" booleans loosely; "yes" is fine,
        # but a plain string outside that table should 422.
        r2 = self.client.post("/api/settings/accel",
                              json={"enabled": "definitely"})
        self.assertEqual(r2.status_code, 422)

    def test_accel_set_rejects_extra(self):
        self._login()
        r = self.client.post("/api/settings/accel",
                             json={"enabled": True, "wat": 1})
        self.assertEqual(r.status_code, 422)

    # ---- /api/params -------------------------------------------------
    def test_params_set_rejects_unknown_key(self):
        self._login()
        r = self.client.post("/api/params",
                             json={"params": {"hax.field": "1"}})
        self.assertEqual(r.status_code, 400)

    def test_params_set_rejects_oversize_body(self):
        self._login()
        big = "x" * (self.web_ui.MAX_PARAMS_BODY_BYTES + 1)
        r = self.client.post(
            "/api/params",
            content=big,
            headers={"content-type": "application/json"},
        )
        self.assertEqual(r.status_code, 413)

    def test_params_set_rejects_oversize_value(self):
        self._login()
        long_val = "y" * (self.web_ui.MAX_PARAM_VALUE_LEN + 1)
        r = self.client.post(
            "/api/params",
            json={"params": {"ui.theme": long_val}},
        )
        self.assertEqual(r.status_code, 400)

    def test_params_set_accepts_allowed_key(self):
        self._login()
        r = self.client.post(
            "/api/params",
            json={"params": {"image.coral_enabled": True}},
        )
        self.assertEqual(r.status_code, 200, r.text)


class SubscriberBoundTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.web_ui, cls.client, cls.mqtt = _bootstrap_web_ui()

    def setUp(self) -> None:
        self.web_ui.telemetry_subscribers.clear()
        self.web_ui.image_subscribers.clear()
        self.web_ui.state_subscribers.clear()
        self.web_ui.control_subscribers.clear()
        # PIN auth required for /ws/control. Establish a session cookie
        # on the test client so the cap test reaches the admit gate
        # rather than bouncing on 4401.
        import os
        os.environ["LIFETRAC_PIN"] = "424242"
        self.client.post("/api/login", json={"pin": "424242"})

    def test_telemetry_subscriber_cap(self):
        cap = self.web_ui.MAX_TELEMETRY_SUBSCRIBERS
        clients = []
        try:
            for _ in range(cap):
                ws = self.client.websocket_connect("/ws/telemetry")
                ws.__enter__()
                clients.append(ws)
            # Next one should be rejected with 4429.
            from starlette.websockets import WebSocketDisconnect as SWD
            with self.assertRaises(SWD) as ctx:
                with self.client.websocket_connect("/ws/telemetry"):
                    pass
            self.assertEqual(ctx.exception.code,
                             self.web_ui.WS_OVER_CAPACITY_CODE)
        finally:
            for ws in clients:
                try:
                    ws.__exit__(None, None, None)
                except Exception:
                    pass

    def test_control_subscriber_cap(self):
        # §B (Round 9): /ws/control was previously unbounded.
        cap = self.web_ui.MAX_CONTROL_SUBSCRIBERS
        clients = []
        try:
            for _ in range(cap):
                ws = self.client.websocket_connect("/ws/control")
                ws.__enter__()
                clients.append(ws)
            from starlette.websockets import WebSocketDisconnect as SWD
            with self.assertRaises(SWD) as ctx:
                with self.client.websocket_connect("/ws/control"):
                    pass
            self.assertEqual(ctx.exception.code,
                             self.web_ui.WS_OVER_CAPACITY_CODE)
        finally:
            for ws in clients:
                try:
                    ws.__exit__(None, None, None)
                except Exception:
                    pass

    def test_control_rejected_without_session(self):
        # Drop the auth cookie established in setUp, then expect 4401.
        self.client.cookies.clear()
        from starlette.websockets import WebSocketDisconnect as SWD
        with self.assertRaises(SWD) as ctx:
            with self.client.websocket_connect("/ws/control"):
                pass
        self.assertEqual(ctx.exception.code, 4401)


if __name__ == "__main__":
    unittest.main()
