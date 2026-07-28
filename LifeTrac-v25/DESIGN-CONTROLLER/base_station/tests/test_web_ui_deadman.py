"""Tests for the base station control WebSocket deadman switch logic.

Validates that missing or delayed control stream updates triggers automated
vehicle zeroization and safe-state halting on MQTT topics.
"""

from __future__ import annotations

import os
import sys
import unittest
import json
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

try:
    import fastapi
    from starlette.websockets import WebSocketDisconnect
    from tests.test_web_ui_validation import _bootstrap_web_ui
except ImportError:
    raise unittest.SkipTest("fastapi/starlette required for deadman tests")


class ControlDeadmanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.web_ui, cls.client, cls.mqtt = _bootstrap_web_ui()

    def setUp(self) -> None:
        self.web_ui.control_subscribers.clear()
        self.mqtt.publish.reset_mock()
        # Ensure we login to be authenticated
        os.environ["LIFETRAC_PIN"] = "424242"
        self.client.post("/api/login", json={"pin": "424242"})

    def _wait_for(self, pred, msg: str, timeout_s: float = 2.0) -> None:
        """Poll ``pred`` until true or ``timeout_s`` elapses, then assert.

        The web_ui control socket is served on the event loop while the test
        drives it through a portal, so every observable here is eventually-
        consistent. Polling keeps the test fast on a healthy run and still
        fails a genuine regression.
        """
        import time as _t
        deadline = _t.monotonic() + timeout_s
        while _t.monotonic() < deadline:
            if pred():
                return
            _t.sleep(0.01)
        self.assertTrue(pred(), msg)

    def test_deadman_trigger_on_timeout(self):
        # We manually mock active_source to mimic allowed local base controls
        with mock.patch("web_ui._base_controls_allowed", return_value=True):
            # Connect to ws control
            with self.client.websocket_connect("/ws/control") as ws:
                # Send one valid stream packet
                ws.send_text(json.dumps({
                    "lhx": 50,
                    "lhy": -50,
                    "rhx": 20,
                    "rhy": 0,
                    "buttons": 4,
                    "flags": 0
                }))
                
                # Check that a command was published to MQTT.
                # 2026-07-27: this used to assert immediately after
                # send_text() and flaked ~1 run in 3 (observed pass/fail/pass
                # across identical runs). send_text only queues into the
                # TestClient portal; whether the server's event loop has
                # drained it yet is a race. Wait for the observable instead
                # of assuming it — bounded so a real regression still fails.
                self._wait_for(lambda: self.mqtt.publish.called,
                               "no control frame published within 2 s")
                last_call_args = self.mqtt.publish.call_args_list[-1]
                topic, payload = last_call_args[0][0], last_call_args[0][1]
                self.assertEqual(topic, "lifetrac/v25/cmd/control")
                
                # Wait past the 150 ms deadman threshold for the zeroized
                # halt frame. Polled rather than a fixed sleep so a loaded
                # CI box cannot fail a working deadman (and so a healthy run
                # finishes as soon as the frame appears).
                def _zeroized_pushed() -> bool:
                    for c in self.mqtt.publish.call_args_list:
                        topic_called, frame_payload = c[0][0], c[0][1]
                        if topic_called != "lifetrac/v25/cmd/control":
                            continue
                        # Frame format: <BBBHbbbbHBBB + CRC16; lhx, lhy,
                        # rhx, rhy live at byte offsets 5..8.
                        if len(frame_payload) >= 14:
                            lhx, lhy, rhx, rhy = frame_payload[5:9]
                            if lhx == 0 and lhy == 0 and rhx == 0 and rhy == 0:
                                return True
                    return False

                self._wait_for(
                    _zeroized_pushed,
                    "Deadman halt slice failed to zeroize controls on timeout")

    def test_deadman_trigger_on_disconnect(self):
        with mock.patch("web_ui._base_controls_allowed", return_value=True):
            with self.client.websocket_connect("/ws/control") as ws:
                ws.send_text(json.dumps({
                    "lhx": 50,
                    "lhy": -50,
                    "rhx": 20,
                    "rhy": 0,
                    "buttons": 4,
                    "flags": 0
                }))
            # Scope exited -> WebSocketDisconnect tripped inside endpoint.
            # Verify a zeroized halt frame was published on close/disconnect.
            calls = self.mqtt.publish.call_args_list
            zeroized_on_close = False
            for c in calls:
                topic_called, frame_payload = c[0][0], c[0][1]
                if topic_called == "lifetrac/v25/cmd/control" and len(frame_payload) >= 14:
                    lhx, lhy, rhx, rhy = frame_payload[5:9]
                    if lhx == 0 and lhy == 0 and rhx == 0 and rhy == 0:
                        zeroized_on_close = True
            self.assertTrue(zeroized_on_close, "Halt was not initiated upon WebSocket disconnection")


if __name__ == "__main__":
    unittest.main()
