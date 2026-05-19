"""S7.1 LINK-pill cross-process glue test.

Pins that web_ui's MQTT subscriber forwards
``lifetrac/v25/control/link_power/{direction}`` payloads — published by
``lora_bridge._airtime_worker`` (S6.2 host half / S7.2 schema) — into
``_image_publisher.link_power`` so they appear on ``/ws/state`` for
``map.js::renderLinkPill`` (S7.1).

Bridge and web_ui are separate processes that share only the MQTT broker,
so this MQTT subscription IS the seam — there's no in-process handoff.
"""

from __future__ import annotations

import json
import os
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

try:
    import paho.mqtt.client  # noqa: F401
    import fastapi           # noqa: F401
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
        return web_ui


def _mqtt_msg(topic: str, payload: bytes) -> SimpleNamespace:
    return SimpleNamespace(topic=topic, payload=payload)


_SAMPLE_UP = {
    "direction": "uplink",
    "state": "MARGIN_LIMITED",
    "action": "RAISE_POWER",
    "value": 15,
    "reason": "snr_below_floor",
    "power_dbm": 15,
    "sf_rung": 0,
    "snr_ewma": -6.2,
    "per": 0.04,
    "per_sample_count": 100,
}
_SAMPLE_DN = dict(_SAMPLE_UP,
                   direction="downlink",
                   state="NORMAL",
                   action="NONE",
                   value=0,
                   reason="",
                   power_dbm=14,
                   snr_ewma=+8.4,
                   per=0.002)


class LinkPowerGlueTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.web_ui = _bootstrap_web_ui()

    def setUp(self) -> None:
        # Reset between tests so one test's residue can't pass another's
        # assertion by accident.
        self.web_ui._image_publisher.link_power["uplink"] = None
        self.web_ui._image_publisher.link_power["downlink"] = None

    def test_uplink_payload_lands_in_state_publisher(self):
        msg = _mqtt_msg("lifetrac/v25/control/link_power/uplink",
                        json.dumps(_SAMPLE_UP).encode())
        self.web_ui._on_mqtt_message(None, None, msg)
        self.assertEqual(self.web_ui._image_publisher.link_power["uplink"],
                         _SAMPLE_UP)
        self.assertIsNone(self.web_ui._image_publisher.link_power["downlink"])

    def test_downlink_payload_lands_independently(self):
        msg_up = _mqtt_msg("lifetrac/v25/control/link_power/uplink",
                           json.dumps(_SAMPLE_UP).encode())
        msg_dn = _mqtt_msg("lifetrac/v25/control/link_power/downlink",
                           json.dumps(_SAMPLE_DN).encode())
        self.web_ui._on_mqtt_message(None, None, msg_up)
        self.web_ui._on_mqtt_message(None, None, msg_dn)
        self.assertEqual(self.web_ui._image_publisher.link_power["uplink"]["state"],
                         "MARGIN_LIMITED")
        self.assertEqual(self.web_ui._image_publisher.link_power["downlink"]["state"],
                         "NORMAL")

    def test_unknown_direction_is_dropped_silently(self):
        msg = _mqtt_msg("lifetrac/v25/control/link_power/sidelink",
                        json.dumps(_SAMPLE_UP).encode())
        self.web_ui._on_mqtt_message(None, None, msg)
        self.assertIsNone(self.web_ui._image_publisher.link_power["uplink"])
        self.assertIsNone(self.web_ui._image_publisher.link_power["downlink"])

    def test_non_dict_payload_is_dropped_silently(self):
        # Hex-fallback (non-JSON payload) returns a string from
        # _decode_payload — must not crash and must not poison state.
        msg = _mqtt_msg("lifetrac/v25/control/link_power/uplink",
                        b"\xde\xad\xbe\xef")
        self.web_ui._on_mqtt_message(None, None, msg)
        self.assertIsNone(self.web_ui._image_publisher.link_power["uplink"])

    def test_snapshot_carries_link_power_after_glue(self):
        msg = _mqtt_msg("lifetrac/v25/control/link_power/uplink",
                        json.dumps(_SAMPLE_UP).encode())
        self.web_ui._on_mqtt_message(None, None, msg)
        snap = self.web_ui._image_publisher.snapshot()
        self.assertEqual(snap["link_power"]["uplink"], _SAMPLE_UP)
        self.assertIsNone(snap["link_power"]["downlink"])


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
