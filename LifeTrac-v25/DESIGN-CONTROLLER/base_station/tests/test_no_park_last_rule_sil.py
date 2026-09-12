"""RS-12.13 — length-conditional NO_PARK_LAST hold (SIL).

The strict hold removes the penultimate-of-N lock on long synth/keyframe
trains (13 fragments: 3.2 % → 1.5 %) at ~38 ms per held train. On the
camera path trains are 1–2 fragments, the "penultimate" is the first
fragment, and the loss there was the base's own command timing (RS-12.11),
which the hold never touched — holding a 2-fragment train costs 16 % of
its airtime for nothing. `final_hold_applies` gates the hold on train
length so enabling it no longer taxes the camera path.
"""
import os
import sys
import unittest
from unittest import mock

_BS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_X8_DIR = os.path.join(os.path.dirname(_BS_DIR), "firmware", "tractor_x8")
sys.path.insert(0, _BS_DIR)
sys.path.insert(0, _X8_DIR)

# The tractor daemon parses its environment at import and pulls in paho for
# the MQTT feed; stub the client the way the web_ui tests do so a bare CI
# checkout without a broker can import it.
try:
    import paho.mqtt.client  # noqa: F401
    _HAVE_PAHO = True
except ImportError:
    _HAVE_PAHO = False

if _HAVE_PAHO:
    with mock.patch("paho.mqtt.client.Client") as _mqtt_class:
        _mqtt_class.return_value.connect = mock.MagicMock()
        _mqtt_class.return_value.loop_start = mock.MagicMock()
        import image_tx_daemon  # noqa: E402
else:
    image_tx_daemon = None


@unittest.skipUnless(image_tx_daemon is not None, "paho-mqtt required to import image_tx_daemon")
class FinalHoldRule(unittest.TestCase):
    def rule(self, no_park_last, n_frags, min_frags=3):
        return image_tx_daemon.final_hold_applies(no_park_last, n_frags, min_frags)

    def test_hold_off_never_holds(self):
        for n in (1, 2, 3, 13):
            self.assertFalse(self.rule(False, n))

    def test_camera_trains_are_never_held(self):
        self.assertFalse(self.rule(True, 1))
        self.assertFalse(self.rule(True, 2))

    def test_long_trains_hold(self):
        self.assertTrue(self.rule(True, 3))
        self.assertTrue(self.rule(True, 13))

    def test_min_frags_is_inclusive_and_tunable(self):
        self.assertTrue(self.rule(True, 2, min_frags=2))
        self.assertFalse(self.rule(True, 2, min_frags=3))
        self.assertTrue(self.rule(True, 1, min_frags=1))   # min 1 = the pre-RS-12.13 hold

    def test_default_min_frags_is_three(self):
        self.assertEqual(image_tx_daemon.NO_PARK_LAST_MIN_FRAGS, 3)


if __name__ == "__main__":
    unittest.main()
