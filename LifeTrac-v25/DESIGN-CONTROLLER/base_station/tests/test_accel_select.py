"""Tests for base_station/image_pipeline/accel_select.py."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

# base_station on sys.path so `from image_pipeline.accel_select import ...`
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from image_pipeline import accel_select  # noqa: E402
from image_pipeline.accel_select import Accelerator, AcceleratorState  # noqa: E402


def _detect_absent() -> tuple[bool, str]:
    return False, "none"


def _detect_present_m2() -> tuple[bool, str]:
    return True, "coral_m2"


def _warmup_ok() -> tuple[bool, str | None]:
    return True, None


def _warmup_fail() -> tuple[bool, str | None]:
    return False, "pycoral not importable: ImportError"


class AcceleratorStateTest(unittest.TestCase):
    def test_is_active_requires_all_three(self):
        s = AcceleratorState(present=True, usable=True, enabled=True)
        self.assertTrue(s.is_active())
        self.assertFalse(AcceleratorState(present=False, usable=True, enabled=True).is_active())
        self.assertFalse(AcceleratorState(present=True, usable=False, enabled=True).is_active())
        self.assertFalse(AcceleratorState(present=True, usable=True, enabled=False).is_active())

    def test_to_dict_includes_active(self):
        s = AcceleratorState(present=True, usable=True, enabled=True, kind="coral_m2")
        d = s.to_dict()
        self.assertTrue(d["active"])
        self.assertEqual(d["kind"], "coral_m2")


class AcceleratorRefreshTest(unittest.TestCase):
    def test_absent_hardware_path(self):
        a = Accelerator(enabled_loader=lambda: True,
                        detect_fn=_detect_absent,
                        warmup_fn=_warmup_ok)
        s = a.refresh()
        self.assertFalse(s.present)
        self.assertFalse(s.usable)
        self.assertFalse(s.is_active())
        self.assertEqual(s.kind, "none")

    def test_present_but_warmup_fails(self):
        a = Accelerator(enabled_loader=lambda: True,
                        detect_fn=_detect_present_m2,
                        warmup_fn=_warmup_fail)
        s = a.refresh()
        self.assertTrue(s.present)
        self.assertFalse(s.usable)
        self.assertFalse(s.is_active())
        self.assertIn("pycoral", (s.last_error or ""))

    def test_present_usable_enabled_active(self):
        a = Accelerator(enabled_loader=lambda: True,
                        detect_fn=_detect_present_m2,
                        warmup_fn=_warmup_ok)
        s = a.refresh()
        self.assertTrue(s.is_active())
        self.assertEqual(s.kind, "coral_m2")

    def test_operator_disabled_short_circuits(self):
        # Hardware fine, operator turned it off.
        flag = {"on": False}
        a = Accelerator(enabled_loader=lambda: flag["on"],
                        detect_fn=_detect_present_m2,
                        warmup_fn=_warmup_ok)
        a.refresh()
        self.assertFalse(a.is_active())
        # Flip operator switch — no refresh needed.
        flag["on"] = True
        self.assertTrue(a.is_active())
        flag["on"] = False
        self.assertFalse(a.is_active())

    def test_hot_unplug_flips_active_within_one_refresh(self):
        plugged = {"in": True}

        def detect():
            return (True, "coral_usb") if plugged["in"] else (False, "none")

        a = Accelerator(enabled_loader=lambda: True,
                        detect_fn=detect, warmup_fn=_warmup_ok)
        self.assertTrue(a.refresh().is_active())
        plugged["in"] = False
        s = a.refresh()
        self.assertFalse(s.is_active())
        self.assertFalse(s.present)

    def test_on_change_fires_only_on_active_transition(self):
        events: list[bool] = []
        a = Accelerator(enabled_loader=lambda: True,
                        detect_fn=_detect_present_m2,
                        warmup_fn=_warmup_ok)
        a.on_change(lambda s: events.append(s.is_active()))
        a.refresh()                # False -> True : fires
        a.refresh()                # True -> True  : silent
        self.assertEqual(events, [True])


class SingletonTest(unittest.TestCase):
    def setUp(self):
        accel_select.reset_for_tests()

    def tearDown(self):
        accel_select.reset_for_tests()

    def test_get_accelerator_returns_same_instance(self):
        a = accel_select.get_accelerator()
        b = accel_select.get_accelerator()
        self.assertIs(a, b)


if __name__ == "__main__":
    unittest.main()
