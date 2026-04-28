import unittest

from lora_proto import (
    CMD_CAMERA_SELECT,
    CMD_ESTOP,
    CMD_LINK_TUNE,
    CMD_PERSON_APPEARED,
    CMD_REQ_KEYFRAME,
    CTRL_FRAME_LEN,
    FT_COMMAND,
    FT_CONTROL,
    FT_HEARTBEAT,
    FT_TELEMETRY,
    KISS_FEND,
    KISS_FESC,
    KissDecoder,
    PHY_CONTROL_SF7,
    PHY_IMAGE,
    PHY_TELEMETRY,
    PRIO_P0,
    PRIO_P1,
    PRIO_P2,
    PRIO_P3,
    REPLAY_WINDOW_BITS,
    ReplayWindow,
    SRC_BASE,
    attribute_phy,
    classify_priority,
    crc16_ccitt,
    encrypted_payload_len,
    fhss_channel_hz,
    kiss_encode,
    lora_time_on_air_ms,
    pack_control,
    parse_header,
    pick_csma_hop,
    verify_crc,
)


class LoraProtoTests(unittest.TestCase):
    def test_crc16_ccitt_known_vector(self):
        self.assertEqual(crc16_ccitt(b"123456789"), 0x29B1)

    def test_kiss_round_trip_with_escaped_bytes(self):
        payload = bytes([0x01, KISS_FEND, 0x02, KISS_FESC, 0x03])
        encoded = kiss_encode(payload)
        self.assertEqual(encoded[0], KISS_FEND)
        self.assertEqual(encoded[-1], KISS_FEND)
        decoder = KissDecoder()
        frames = []
        for byte in encoded:
            frames.extend(decoder.feed(byte))
        self.assertEqual(frames, [payload])

    def test_pack_control_layout_and_crc(self):
        frame = pack_control(7, 200, -200, 11, -12, buttons=0x0081, flags=1, hb=9)
        self.assertEqual(len(frame), CTRL_FRAME_LEN)
        header = parse_header(frame)
        self.assertIsNotNone(header)
        self.assertEqual(header.source_id, SRC_BASE)
        self.assertEqual(header.frame_type, FT_CONTROL)
        self.assertEqual(header.sequence_num, 7)
        self.assertEqual(frame[5], 127)
        self.assertEqual(frame[6], 129)  # -127 as uint8 on the wire
        self.assertTrue(verify_crc(frame))

    def test_airtime_estimator_is_reasonable(self):
        self.assertGreater(lora_time_on_air_ms(16, PHY_CONTROL_SF7), 15.0)
        self.assertLess(lora_time_on_air_ms(16, PHY_CONTROL_SF7), 35.0)

    def test_encrypted_control_airtime_fits_20hz_cadence(self):
        # DECISIONS.md D-A2: control PHY is SF7/BW250 so the encrypted 44 B
        # ControlFrame fits a 50 ms (20 Hz) cadence with margin.
        airtime_ms = lora_time_on_air_ms(encrypted_payload_len(CTRL_FRAME_LEN), PHY_CONTROL_SF7)
        self.assertLessEqual(airtime_ms, 50.0)
        self.assertGreater(airtime_ms, 30.0)   # sanity: not absurdly small

    def test_image_fragment_fits_25ms_cap_at_bw500(self):
        # DECISIONS.md D-A3: image PHY is SF7/BW500 so a 32 B fragment
        # stays under the 25 ms per-fragment cap (LORA_IMPLEMENTATION.md §4).
        self.assertLessEqual(lora_time_on_air_ms(32, PHY_IMAGE), 25.0)

    def test_fhss_sequence_stays_in_us_915_band(self):
        channels = [fhss_channel_hz(0x12345678, hop) for hop in range(8)]
        self.assertEqual(len(set(channels)), 8)
        self.assertGreaterEqual(min(channels), 902_000_000)
        self.assertLessEqual(max(channels), 928_000_000)

    def test_attribute_phy_routes_telemetry_topics_correctly(self):
        # Image-bearing telemetry topics ride PHY_IMAGE so the bridge ledger
        # tracks image utilization separately for the auto-fallback ladder.
        self.assertIs(attribute_phy(FT_TELEMETRY, 0x25), PHY_IMAGE)
        self.assertIs(attribute_phy(FT_TELEMETRY, 0x28), PHY_IMAGE)
        self.assertIs(attribute_phy(FT_TELEMETRY, 0x29), PHY_IMAGE)
        # Other telemetry rides PHY_TELEMETRY.
        self.assertIs(attribute_phy(FT_TELEMETRY, 0x10), PHY_TELEMETRY)
        # Control / heartbeat / no-topic always ride the control profile.
        self.assertIs(attribute_phy(FT_CONTROL), PHY_CONTROL_SF7)
        self.assertIs(attribute_phy(FT_HEARTBEAT), PHY_CONTROL_SF7)

    def test_replay_window_accepts_first_then_rejects_duplicate(self):
        w = ReplayWindow()
        self.assertTrue(w.check_and_update(100))
        self.assertFalse(w.check_and_update(100))   # exact duplicate

    def test_replay_window_accepts_in_order_advance(self):
        w = ReplayWindow()
        for s in range(50, 60):
            self.assertTrue(w.check_and_update(s))
        # Re-replay any of them — must reject.
        for s in range(50, 60):
            self.assertFalse(w.check_and_update(s))

    def test_replay_window_accepts_out_of_order_within_window(self):
        w = ReplayWindow()
        self.assertTrue(w.check_and_update(200))
        # 150 is 50 frames behind — inside the 64-frame window, never seen.
        self.assertTrue(w.check_and_update(150))
        # Replay of 150 — must reject.
        self.assertFalse(w.check_and_update(150))

    def test_replay_window_rejects_too_old(self):
        w = ReplayWindow()
        self.assertTrue(w.check_and_update(1000))
        # 1000 - 64 = 936; anything <= 936 is outside the window.
        self.assertFalse(w.check_and_update(936))
        self.assertFalse(w.check_and_update(500))

    def test_replay_window_handles_16bit_wrap(self):
        w = ReplayWindow()
        self.assertTrue(w.check_and_update(0xFFF0))
        # Wrap forward to a fresh seq — small unsigned but logically newer.
        self.assertTrue(w.check_and_update(0x0005))
        # Replay of pre-wrap seq — must reject.
        self.assertFalse(w.check_and_update(0xFFF0))

    def test_classify_priority_buckets(self):
        # P0: control + heartbeat + safety/link commands.
        self.assertEqual(classify_priority(FT_CONTROL), PRIO_P0)
        self.assertEqual(classify_priority(FT_HEARTBEAT), PRIO_P0)
        self.assertEqual(classify_priority(FT_COMMAND, opcode=CMD_ESTOP), PRIO_P0)
        self.assertEqual(classify_priority(FT_COMMAND, opcode=CMD_LINK_TUNE), PRIO_P0)
        self.assertEqual(classify_priority(FT_COMMAND, opcode=CMD_PERSON_APPEARED), PRIO_P0)
        # P1: operator commands.
        self.assertEqual(classify_priority(FT_COMMAND, opcode=CMD_CAMERA_SELECT), PRIO_P1)
        self.assertEqual(classify_priority(FT_COMMAND, opcode=CMD_REQ_KEYFRAME), PRIO_P1)
        # P2: non-image telemetry.
        self.assertEqual(classify_priority(FT_TELEMETRY, topic_id=0x04), PRIO_P2)
        self.assertEqual(classify_priority(FT_TELEMETRY, topic_id=0x10), PRIO_P2)
        # P3: image telemetry.
        self.assertEqual(classify_priority(FT_TELEMETRY, topic_id=0x25), PRIO_P3)
        self.assertEqual(classify_priority(FT_TELEMETRY, topic_id=0x28), PRIO_P3)
        self.assertEqual(classify_priority(FT_TELEMETRY, topic_id=0x29), PRIO_P3)

    def test_csma_clean_start_hop_returns_zero_skips(self):
        # Sampler reports every channel idle: should pick start_hop with 0 skips.
        calls = []
        def sampler(hz):
            calls.append(hz)
            return -110   # well below default -90 dBm threshold
        hop, skips = pick_csma_hop(0xDEADBEEF, 42, sampler)
        self.assertEqual(hop, 42)
        self.assertEqual(skips, 0)
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0], fhss_channel_hz(0xDEADBEEF, 42))

    def test_csma_skips_one_busy_channel(self):
        # First probe busy, second clear: should advance one hop and report skips=1.
        readings = iter([-50, -100])
        probed = []
        def sampler(hz):
            probed.append(hz)
            return next(readings)
        hop, skips = pick_csma_hop(0xCAFEBABE, 7, sampler)
        self.assertEqual(hop, 8)
        self.assertEqual(skips, 1)
        self.assertEqual(probed[0], fhss_channel_hz(0xCAFEBABE, 7))
        self.assertEqual(probed[1], fhss_channel_hz(0xCAFEBABE, 8))

    def test_csma_all_busy_falls_through_with_max_skips(self):
        # Every probe busy: must still return the last candidate so the frame
        # is transmitted (control safety > CCA strictness), with skips==max_skips.
        def sampler(_hz):
            return -40
        hop, skips = pick_csma_hop(0x1234, 100, sampler, max_skips=3)
        self.assertEqual(hop, 103)
        self.assertEqual(skips, 3)

    def test_csma_threshold_boundary_is_inclusive_clear(self):
        # RSSI exactly at threshold counts as CLEAR (matches C: rssi <= threshold).
        def sampler(_hz):
            return -90
        hop, skips = pick_csma_hop(0x1, 0, sampler)
        self.assertEqual((hop, skips), (0, 0))


if __name__ == "__main__":
    unittest.main()