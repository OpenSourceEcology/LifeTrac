"""W2-04: link-adaptive byte budget via ``CMD_LINK_PROFILE``.

Pins three behaviours:

1. :class:`camera_service.LinkBudget` recomputes its byte cap when the
   base station reports a new ``(n_fragments, phy_index)`` pair.
2. ``CMD_LINK_PROFILE`` over the back-channel mutates the shared
   :class:`LinkBudget` and forces a keyframe so the new cap takes effect
   on the next encode.
3. Malformed / out-of-range payloads are rejected without crashing the
   reader thread or corrupting the previous budget.

Pure-Python; no PIL needed.
"""

from __future__ import annotations

import importlib.util
import os
import sys
import threading
import unittest


_HERE = os.path.dirname(__file__)
_X8_DIR = os.path.normpath(os.path.join(
    _HERE, "..", "..", "firmware", "tractor_x8"))
_BS_DIR = os.path.normpath(os.path.join(_HERE, ".."))

if _BS_DIR not in sys.path:
    sys.path.insert(0, _BS_DIR)
if _X8_DIR not in sys.path:
    sys.path.insert(0, _X8_DIR)

import camera_service  # noqa: E402
from image_pipeline.fragment import max_payload_for_n_fragments  # noqa: E402
from lora_proto import PHY_BY_NAME  # noqa: E402


def _kiss_frame(opcode: int, *args: int) -> bytes:
    return bytes([camera_service.X8_CMD_TOPIC, opcode, *args])


class LinkBudgetUpdateTests(unittest.TestCase):

    def test_image_phy_recomputes_byte_cap(self) -> None:
        budget = camera_service.LinkBudget()
        ok = budget.update(n_fragments=4, profile_index=0)  # "image"
        expected = max_payload_for_n_fragments(4, profile=PHY_BY_NAME["image"])
        self.assertTrue(ok)
        self.assertEqual(budget.bytes, expected)
        self.assertEqual(budget.profile_name, "image")
        self.assertEqual(budget.n_fragments, 4)

    def test_telemetry_phy_returns_smaller_cap(self) -> None:
        budget_img = camera_service.LinkBudget()
        budget_img.update(n_fragments=4, profile_index=0)
        budget_tel = camera_service.LinkBudget()
        budget_tel.update(n_fragments=4, profile_index=1)  # "telemetry"
        # Telemetry PHY (SF9) carries fewer bytes per fragment than image
        # PHY (SF7); same fragment count must yield a smaller cap.
        self.assertLess(budget_tel.bytes, budget_img.bytes)

    def test_out_of_range_phy_index_rejected(self) -> None:
        budget = camera_service.LinkBudget()
        budget.update(n_fragments=4, profile_index=0)
        prior_bytes = budget.bytes
        ok = budget.update(n_fragments=4, profile_index=99)
        self.assertFalse(ok)
        self.assertEqual(budget.bytes, prior_bytes)
        self.assertEqual(budget.profile_name, "image")

    def test_zero_fragments_rejected(self) -> None:
        budget = camera_service.LinkBudget()
        budget.update(n_fragments=4, profile_index=0)
        prior_bytes = budget.bytes
        self.assertFalse(budget.update(n_fragments=0, profile_index=0))
        self.assertEqual(budget.bytes, prior_bytes)

    def test_initial_state_has_no_budget(self) -> None:
        budget = camera_service.LinkBudget()
        self.assertIsNone(budget.bytes)
        self.assertIsNone(budget.n_fragments)
        self.assertIsNone(budget.profile_name)


class CmdLinkProfileDispatchTests(unittest.TestCase):

    def test_dispatch_updates_budget_and_forces_keyframe(self) -> None:
        evt = threading.Event()
        budget = camera_service.LinkBudget()
        camera_service.dispatch_back_channel(
            _kiss_frame(camera_service.CMD_LINK_PROFILE, 4, 0),
            evt, link_budget=budget)
        self.assertTrue(evt.is_set())
        self.assertIsNotNone(budget.bytes)
        self.assertEqual(budget.profile_name, "image")
        self.assertEqual(budget.n_fragments, 4)

    def test_dispatch_truncated_args_ignored(self) -> None:
        evt = threading.Event()
        budget = camera_service.LinkBudget(bytes_=999, n_fragments=2,
                                           profile_name="image")
        # Only n_fragments byte present, profile_index missing.
        camera_service.dispatch_back_channel(
            _kiss_frame(camera_service.CMD_LINK_PROFILE, 4),
            evt, link_budget=budget)
        # State must not have been corrupted.
        self.assertEqual(budget.bytes, 999)
        self.assertEqual(budget.n_fragments, 2)

    def test_dispatch_without_link_budget_is_silent(self) -> None:
        evt = threading.Event()
        # Must not raise even though no LinkBudget was passed.
        camera_service.dispatch_back_channel(
            _kiss_frame(camera_service.CMD_LINK_PROFILE, 4, 0),
            evt)
        # Keyframe still forced (opcode is in the keyframe-forcing set).
        self.assertTrue(evt.is_set())

    def test_dispatch_bad_phy_index_keeps_previous_budget(self) -> None:
        evt = threading.Event()
        budget = camera_service.LinkBudget()
        budget.update(n_fragments=4, profile_index=0)
        prior = budget.bytes
        camera_service.dispatch_back_channel(
            _kiss_frame(camera_service.CMD_LINK_PROFILE, 4, 99),
            evt, link_budget=budget)
        self.assertEqual(budget.bytes, prior)
        # Even on rejection we force a keyframe so a stale budget is at
        # least observable from the next decoded frame on the base side.
        self.assertTrue(evt.is_set())


class ResolveByteBudgetUsesProfileTests(unittest.TestCase):

    def test_explicit_telemetry_profile_picks_smaller_cap(self) -> None:
        from unittest import mock
        with mock.patch.dict(os.environ, {
            "LIFETRAC_FRAGMENT_BUDGET":  "4",
            "LIFETRAC_FRAGMENT_PROFILE": "image",
        }, clear=False):
            cap_img = camera_service._resolve_byte_budget()
        with mock.patch.dict(os.environ, {
            "LIFETRAC_FRAGMENT_BUDGET":  "4",
            "LIFETRAC_FRAGMENT_PROFILE": "telemetry",
        }, clear=False):
            cap_tel = camera_service._resolve_byte_budget()
        self.assertIsNotNone(cap_img)
        self.assertIsNotNone(cap_tel)
        self.assertLess(cap_tel, cap_img)

    def test_unknown_profile_returns_none(self) -> None:
        from unittest import mock
        with mock.patch.dict(os.environ, {
            "LIFETRAC_FRAGMENT_BUDGET":  "4",
            "LIFETRAC_FRAGMENT_PROFILE": "no_such_phy",
        }, clear=False):
            self.assertIsNone(camera_service._resolve_byte_budget())


if __name__ == "__main__":
    unittest.main()
