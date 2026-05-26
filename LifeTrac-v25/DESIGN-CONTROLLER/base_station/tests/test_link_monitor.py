import unittest

from link_monitor import (
    EncodeModeController,
    RollingAirtimeLedger,
    SAFE_MODE_CLEAR_WINDOWS,
    SAFE_MODE_CRITICAL_WINDOWS,
)
from lora_proto import CMD_ENCODE_MODE, EncodeMode, FT_COMMAND, PHY_IMAGE, PHY_TELEMETRY, parse_header, verify_crc


def _u(image_util: float):
    return type("Util", (), {"image": image_util})()


class LinkMonitorTests(unittest.TestCase):
    def test_ledger_tracks_profiles_separately(self):
        ledger = RollingAirtimeLedger(window_ms=10_000)
        image_airtime = ledger.record(0, PHY_IMAGE, cleartext_len=80)
        telem_airtime = ledger.record(100, PHY_TELEMETRY, cleartext_len=16)
        util = ledger.utilization(100)
        self.assertAlmostEqual(util.image, image_airtime / 10_000, places=4)
        self.assertAlmostEqual(util.telemetry, telem_airtime / 10_000, places=4)
        self.assertGreater(util.total, util.image)

    def test_encode_mode_requires_three_windows(self):
        controller = EncodeModeController(required_windows=3)
        high = type("Util", (), {"image": 0.55})()
        self.assertIsNone(controller.observe(high))
        self.assertIsNone(controller.observe(high))
        self.assertEqual(controller.observe(high), EncodeMode.MOTION_ONLY)
        self.assertEqual(controller.mode, EncodeMode.MOTION_ONLY)

    def test_command_frame_uses_encode_mode_opcode(self):
        controller = EncodeModeController(required_windows=1)
        frame = controller.command_frame(42, EncodeMode.Y_ONLY)
        header = parse_header(frame)
        self.assertEqual(header.frame_type, FT_COMMAND)
        self.assertEqual(header.sequence_num, 42)
        self.assertEqual(frame[5], CMD_ENCODE_MODE)
        self.assertEqual(frame[6], EncodeMode.Y_ONLY)
        self.assertTrue(verify_crc(frame))

    def test_safe_mode_engages_and_overrides_operator_pin(self):
        ctrl = EncodeModeController(required_windows=1)
        # Operator pins FULL (the most quality-hungry mode).
        ctrl.set_operator_ceiling(EncodeMode.FULL)
        edges: list[bool] = []
        ctrl.set_safe_mode_callback(edges.append)
        crit = _u(0.97)
        # Below SAFE_MODE_CRITICAL_WINDOWS streak: not yet engaged.
        for _ in range(SAFE_MODE_CRITICAL_WINDOWS - 1):
            ctrl.observe(crit)
            self.assertFalse(ctrl.safe_mode_active)
        ctrl.observe(crit)  # nth critical observation -> engage
        self.assertTrue(ctrl.safe_mode_active)
        self.assertEqual(edges, [True])
        # Hysteresis: one more identical observation lets the new
        # MONO_G4 target commit through the required_windows gate.
        ctrl.observe(crit)
        # Mode is now MONO_G4 even though operator pin is FULL.
        self.assertEqual(ctrl.mode, EncodeMode.MONO_G4)

    def test_safe_mode_clears_after_long_quiet_streak(self):
        ctrl = EncodeModeController(required_windows=1)
        edges: list[bool] = []
        ctrl.set_safe_mode_callback(edges.append)
        # Engage first.
        for _ in range(SAFE_MODE_CRITICAL_WINDOWS):
            ctrl.observe(_u(0.99))
        self.assertTrue(ctrl.safe_mode_active)
        # Now drop quiet: one short of clear window should not release.
        for _ in range(SAFE_MODE_CLEAR_WINDOWS - 1):
            ctrl.observe(_u(0.10))
        self.assertTrue(ctrl.safe_mode_active)
        ctrl.observe(_u(0.10))  # final tick clears
        self.assertFalse(ctrl.safe_mode_active)
        self.assertEqual(edges, [True, False])

    def test_safe_mode_blip_does_not_engage(self):
        ctrl = EncodeModeController(required_windows=1)
        # A single spike then a relief tick must reset the enter counter.
        ctrl.observe(_u(0.99))
        ctrl.observe(_u(0.30))
        for _ in range(SAFE_MODE_CRITICAL_WINDOWS - 1):
            ctrl.observe(_u(0.99))
        # We had (CRITICAL_WINDOWS-1) consecutive crits but the streak
        # was broken before reaching the threshold -> still inactive.
        self.assertFalse(ctrl.safe_mode_active)


if __name__ == "__main__":
    unittest.main()