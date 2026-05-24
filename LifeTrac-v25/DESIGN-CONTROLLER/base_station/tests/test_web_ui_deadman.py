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
                
                # Check that a command was published to MQTT
                self.assertTrue(self.mqtt.publish.called)
                last_call_args = self.mqtt.publish.call_args_list[-1]
                topic, payload = last_call_args[0][0], last_call_args[0][1]
                self.assertEqual(topic, "lifetrac/v25/cmd/control")
                
                # Settle and wait past the timeout threshold (150 ms) to trigger deadman
                import time
                time.sleep(0.2)
                
                # We expect the background thread/loop to execute wait_for and trigger:
                # checking if a zeroized halt frame was published as deadman fallback.
                calls = self.mqtt.publish.call_args_list
                zeroized_pushed = False
                for c in calls:
                    topic_called, frame_payload = c[0][0], c[0][1]
                    if topic_called == "lifetrac/v25/cmd/control":
                        # Unpack the control frame bytes to verify zeroization
                        # Frame format: <BBBHbbbbHBBB + CRC16
                        # Byte offsets for lhx, lhy, rhx, rhy are 5, 6, 7, 8
                        if len(frame_payload) >= 14:
                            lhx, lhy, rhx, rhy = frame_payload[5:9]
                            if lhx == 0 and lhy == 0 and rhx == 0 and rhy == 0:
                                zeroized_pushed = True
                                break
                self.assertTrue(zeroized_pushed, "Deadman halt slice failed to zeroize controls on timeout")

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
