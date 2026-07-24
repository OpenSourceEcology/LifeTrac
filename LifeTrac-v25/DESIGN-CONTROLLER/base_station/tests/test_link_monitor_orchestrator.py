"""LinkMonitor end-to-end orchestrator test."""
from __future__ import annotations

import unittest

from link_monitor import LinkMonitor
# NB: must use the "image"-named profile (the BW250 alias) — LinkMonitor
# buckets utilization by profile name, so a *_bw500/_bw250 variant would
# land outside the image bucket and the ladder would never trigger.
# Post-F1 the alias is BW250: each 80 B on_air call is ~92 ms (was ~18 ms
# at BW500), still comfortably past the 0.80 threshold this test needs.
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
        # Post-F1 the "image" alias is SF7/BW250: each 80 B on_air call adds
        # ~92 ms airtime, so 5 calls/window ≈ 0.92 utilization — just above
        # the 0.80 threshold. Keep it JUST above: saturating the window (the
        # old 60-call loop → util ≈ 11) makes the ladder walk a further rung
        # each tick (WIREFRAME → MONO_G4), which resets the 3-window
        # hysteresis and no command is ever emitted. The FIRST tick() only
        # primes the window boundary (publishes nothing), so "3 consecutive
        # high windows" = 4 tick() calls: 1 priming + 3 evaluated.
        for tick_n in range(4):
            now_ms = tick_n * 1000
            for _ in range(5):                  # 5 * ~92 ms ≈ 0.92 util
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
