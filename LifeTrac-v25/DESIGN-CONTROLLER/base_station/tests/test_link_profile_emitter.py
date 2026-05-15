"""W2-04: base-side ``LinkProfileEmitter`` -> ``CMD_LINK_PROFILE`` (0x64).

Closes the loop on the X8's :class:`camera_service.LinkBudget` by driving
the opcode from the rolling SNR the bridge harvests off the LoRa modem.
Pins the ladder, the three-window hysteresis, the wire-format command
frame, and the seq/index plumbing.
"""

from __future__ import annotations

import struct
import unittest

from link_monitor import (
    DEFAULT_SNR_LADDER,
    LinkProfileEmitter,
    LinkProfileTarget,
)
from lora_proto import (
    CMD_LINK_PROFILE,
    FT_COMMAND,
    LINK_PHY_NAMES,
    SRC_BASE,
    pack_command,
    verify_crc,
)


def _drive(emitter: LinkProfileEmitter, snr_db: float, *, ticks: int = 1,
           start_ms: int = 0, step_ms: int = 1000) -> list[bytes]:
    """Feed the same SNR ``ticks`` times, return any frames emitted."""
    out: list[bytes] = []
    for i in range(ticks):
        frame = emitter.observe(start_ms + i * step_ms, snr_db)
        if frame is not None:
            out.append(frame)
    return out


class LadderTargetTests(unittest.TestCase):

    def test_default_ladder_is_strictly_descending(self) -> None:
        thresholds = [t for t, _ in DEFAULT_SNR_LADDER]
        self.assertEqual(thresholds, sorted(thresholds, reverse=True))
        self.assertEqual(len(thresholds), len(set(thresholds)))

    def test_high_snr_picks_full_image_budget(self) -> None:
        target = LinkProfileEmitter._target_for(0.0, DEFAULT_SNR_LADDER)
        self.assertEqual(target.phy_name, "image")
        self.assertEqual(target.n_fragments, 8)

    def test_marginal_image_picks_quarter_budget(self) -> None:
        # Between -10 and -7: image PHY, n_fragments=2 rung.
        target = LinkProfileEmitter._target_for(-9.5, DEFAULT_SNR_LADDER)
        self.assertEqual(target.phy_name, "image")
        self.assertEqual(target.n_fragments, 2)

    def test_below_image_floor_retreats_to_telemetry(self) -> None:
        target = LinkProfileEmitter._target_for(-12.0, DEFAULT_SNR_LADDER)
        self.assertEqual(target.phy_name, "telemetry")
        self.assertEqual(target.n_fragments, 4)

    def test_floor_clamps_to_minimum_telemetry_rung(self) -> None:
        target = LinkProfileEmitter._target_for(-50.0, DEFAULT_SNR_LADDER)
        self.assertEqual(target.phy_name, "telemetry")
        self.assertEqual(target.n_fragments, 2)

    def test_constructor_rejects_unsorted_ladder(self) -> None:
        bad = (
            (-3.0, LinkProfileTarget(8, 0)),
            (-3.0, LinkProfileTarget(4, 0)),  # equal threshold — must reject
        )
        with self.assertRaises(ValueError):
            LinkProfileEmitter(ladder=bad)


class HysteresisTests(unittest.TestCase):

    def test_first_observation_does_not_emit(self) -> None:
        e = LinkProfileEmitter(required_windows=3)
        self.assertEqual(_drive(e, 0.0, ticks=1), [])
        self.assertIsNone(e.current)

    def test_required_windows_must_agree_before_emit(self) -> None:
        e = LinkProfileEmitter(required_windows=3)
        # 2 ticks at -9.5 (image quarter) — not enough.
        self.assertEqual(_drive(e, -9.5, ticks=2), [])
        self.assertIsNone(e.current)
        # Third tick commits.
        frames = _drive(e, -9.5, ticks=1, start_ms=2000)
        self.assertEqual(len(frames), 1)
        self.assertEqual(e.current, LinkProfileTarget(2, 0))

    def test_one_bad_window_resets_candidate_count(self) -> None:
        e = LinkProfileEmitter(required_windows=3)
        _drive(e, -9.5, ticks=2)                             # 2 toward "quarter"
        # A burst of better SNR picks a different target — counter resets.
        out = _drive(e, 0.0, ticks=1, start_ms=2000)
        self.assertEqual(out, [])
        self.assertIsNone(e.current)
        # Now we need 2 more fresh ticks at 0.0 to commit "image full"
        # (candidate is already at full with count=1 from the previous tick).
        out = _drive(e, 0.0, ticks=1, start_ms=3000)
        self.assertEqual(out, [])
        out = _drive(e, 0.0, ticks=1, start_ms=4000)
        self.assertEqual(len(out), 1)
        self.assertEqual(e.current, LinkProfileTarget(8, 0))

    def test_steady_state_after_commit_does_not_re_emit(self) -> None:
        e = LinkProfileEmitter(required_windows=2)
        _drive(e, -9.5, ticks=2)                             # commits
        self.assertEqual(e.current, LinkProfileTarget(2, 0))
        # Many more ticks at the same SNR must not re-emit.
        out = _drive(e, -9.5, ticks=50, start_ms=10000)
        self.assertEqual(out, [])

    def test_required_windows_minimum_is_one(self) -> None:
        e = LinkProfileEmitter(required_windows=0)           # clamped to 1
        out = _drive(e, 0.0, ticks=1)
        self.assertEqual(len(out), 1)


class WireFormatTests(unittest.TestCase):

    def test_emitted_frame_is_valid_command_frame(self) -> None:
        e = LinkProfileEmitter(required_windows=1)
        out = _drive(e, 0.0, ticks=1)
        self.assertEqual(len(out), 1)
        frame = out[0]
        self.assertTrue(verify_crc(frame))

    def test_frame_carries_correct_opcode_and_args(self) -> None:
        e = LinkProfileEmitter(required_windows=1)
        out = _drive(e, -9.5, ticks=1)
        frame = out[0]
        # Header layout: version, source, frame_type, seq u16, opcode, args...
        version, source, frame_type = frame[0], frame[1], frame[2]
        self.assertEqual(frame_type, FT_COMMAND)
        self.assertEqual(source, SRC_BASE)
        opcode = frame[5]
        self.assertEqual(opcode, CMD_LINK_PROFILE)
        n_frag, phy_idx = frame[6], frame[7]
        self.assertEqual(n_frag, 2)
        self.assertEqual(phy_idx, 0)  # "image"
        self.assertEqual(LINK_PHY_NAMES[phy_idx], "image")

    def test_seq_increments_across_emits(self) -> None:
        e = LinkProfileEmitter(required_windows=1, seq_start=100)
        out_a = _drive(e, 0.0, ticks=1)                      # commit "image full"
        out_b = _drive(e, -9.5, ticks=1, start_ms=1000)      # commit "image quarter"
        seq_a = struct.unpack("<H", out_a[0][3:5])[0]
        seq_b = struct.unpack("<H", out_b[0][3:5])[0]
        self.assertEqual(seq_a, 101)
        self.assertEqual(seq_b, 102)


class PublisherWiringTests(unittest.TestCase):

    def test_publish_command_called_on_commit(self) -> None:
        published: list[bytes] = []
        e = LinkProfileEmitter(publish_command=published.append,
                               required_windows=1)
        out = _drive(e, 0.0, ticks=1)
        self.assertEqual(out, published)

    def test_publish_command_silent_until_commit(self) -> None:
        published: list[bytes] = []
        e = LinkProfileEmitter(publish_command=published.append,
                               required_windows=3)
        _drive(e, 0.0, ticks=2)
        self.assertEqual(published, [])

    def test_reset_clears_committed_state(self) -> None:
        published: list[bytes] = []
        e = LinkProfileEmitter(publish_command=published.append,
                               required_windows=1)
        _drive(e, 0.0, ticks=1)
        self.assertIsNotNone(e.current)
        e.reset()
        self.assertIsNone(e.current)
        # After reset, next observation at the same SNR must re-emit.
        published.clear()
        _drive(e, 0.0, ticks=1, start_ms=10000)
        self.assertEqual(len(published), 1)


class AuditWiringTests(unittest.TestCase):

    def test_audit_log_called_on_commit(self) -> None:
        calls: list[dict] = []

        class _Audit:
            def log_link_profile_change(self, **kw):
                calls.append(kw)

        e = LinkProfileEmitter(required_windows=1, audit=_Audit())
        _drive(e, -9.5, ticks=1)
        self.assertEqual(len(calls), 1)
        self.assertIsNone(calls[0]["prev"])
        self.assertEqual(calls[0]["new"], {"n_fragments": 2, "phy": "image"})
        self.assertEqual(calls[0]["snr_db"], -9.5)

    def test_audit_records_previous_target_after_first_commit(self) -> None:
        calls: list[dict] = []

        class _Audit:
            def log_link_profile_change(self, **kw):
                calls.append(kw)

        e = LinkProfileEmitter(required_windows=1, audit=_Audit())
        _drive(e, 0.0, ticks=1)                              # image full
        _drive(e, -12.0, ticks=1, start_ms=1000)             # telemetry retreat
        self.assertEqual(len(calls), 2)
        self.assertEqual(calls[1]["prev"], {"n_fragments": 8, "phy": "image"})
        self.assertEqual(calls[1]["new"],  {"n_fragments": 4, "phy": "telemetry"})


class CrossSideContractTests(unittest.TestCase):
    """Both sides must agree on ``LINK_PHY_NAMES`` order so the index space
    on the wire stays byte-identical."""

    def test_link_phy_names_first_two_are_image_and_telemetry(self) -> None:
        # The X8 hard-codes index 0 = image, index 1 = telemetry. Pin both.
        self.assertEqual(LINK_PHY_NAMES[0], "image")
        self.assertEqual(LINK_PHY_NAMES[1], "telemetry")

    def test_link_phy_names_match_x8_side(self) -> None:
        # Round-trip through the X8 module. Without this gate a unilateral
        # rename on either side would silently desynchronize the wire.
        import importlib.util
        import os
        import sys
        x8_dir = os.path.normpath(os.path.join(
            os.path.dirname(__file__), "..", "..",
            "firmware", "tractor_x8"))
        if x8_dir not in sys.path:
            sys.path.insert(0, x8_dir)
        import camera_service                                  # noqa: WPS433
        self.assertEqual(camera_service.LINK_PHY_NAMES, LINK_PHY_NAMES)


if __name__ == "__main__":
    unittest.main()
