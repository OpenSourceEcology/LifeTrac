import unittest

from link_monitor import EncodeModeController, RollingAirtimeLedger
from lora_proto import CMD_ENCODE_MODE, EncodeMode, FT_COMMAND, PHY_IMAGE, PHY_TELEMETRY, parse_header, verify_crc


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


if __name__ == "__main__":
    unittest.main()