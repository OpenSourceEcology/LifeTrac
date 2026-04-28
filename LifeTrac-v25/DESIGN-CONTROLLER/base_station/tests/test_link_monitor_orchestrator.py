"""LinkMonitor end-to-end orchestrator test."""
from __future__ import annotations

import unittest

from link_monitor import LinkMonitor
from lora_proto import EncodeMode, PHY_IMAGE


class LinkMonitorOrchestratorTests(unittest.TestCase):
    def test_emits_command_after_three_high_windows(self):
        commands: list[bytes] = []
        statuses: list[dict] = []
        monitor = LinkMonitor(
            publish_command=commands.append,
            publish_status=statuses.append,
            window_ms=1000,
            required_windows=3,
        )
        # Crank image utilization > 80 % to drive the controller toward WIREFRAME.
        # Each call to on_air at PHY_IMAGE adds ~18 ms airtime; we just need to
        # exceed the 0.80 threshold for three consecutive ticks.
        for tick_n in range(3):
            now_ms = tick_n * 1000
            for _ in range(60):                 # 60 * ~18 ms ≈ 1080 ms saturated
                monitor.on_air(now_ms, PHY_IMAGE, cleartext_len=80)
            monitor.tick(now_ms)
        # Third tick should have crossed required_windows; one CMD_ENCODE_MODE emitted.
        self.assertEqual(len(commands), 1)
        # Status should also have been pushed at least once.
        self.assertGreaterEqual(len(statuses), 1)
        self.assertEqual(statuses[-1]["encode_mode"], EncodeMode.WIREFRAME.name)

    def test_quiet_link_stays_full(self):
        commands: list[bytes] = []
        monitor = LinkMonitor(publish_command=commands.append, window_ms=1000)
        for tick_n in range(5):
            monitor.tick(tick_n * 1000)
        self.assertEqual(commands, [])


if __name__ == "__main__":
    unittest.main()
