"""Codec-6 (VECTOR) frames on the strict image path (VECTOR_SCENE.md §3.1, §7.1).

A VS1 frame is a ``TileDeltaFrame`` whose codec byte is 6 and whose body is
the vector bitstream instead of a changed-tile bitmap. These tests pin the
parser branch, the byte-for-byte pass-through that ``image_rx_daemon``
relies on when it re-encodes completed frames, the F ≤ 197/237 budget that
keeps every frame a single fragment, the F − 1 cap on epoch-start frames
(so the daemon's ``0xFD`` copies stay one fragment at both profiles), the
enum values the two ends share, and the reassembler round trip.
"""
from __future__ import annotations

import os
import sys
import unittest

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_BS_DIR = os.path.dirname(_THIS_DIR)
if _BS_DIR not in sys.path:
    sys.path.insert(0, _BS_DIR)

from lora_proto import (  # noqa: E402
    IMAGE_FRAG_AIR_CAP_MS, LORA_HOP_HDR_LEN, PHY_IMAGE_BW250, PHY_IMAGE_BW500,
    TX_FRAME_BODY_MAX, Badge, EncodeMode, lora_time_on_air_ms,
    max_image_fragment_body, pack_image_fragments, pack_image_fragments_v2,
)
from image_pipeline.frame_format import (  # noqa: E402
    CODEC_VECTOR, FRAME_KIND_DELTA, FRAME_KIND_KEY, HEADER_FIXED_LEN, FrameDecodeError,
    TileDeltaFrame, encode_tile_delta_frame, parse_tile_delta_frame,
)
from image_pipeline.reassemble import FragmentReassembler  # noqa: E402
from image_pipeline.vector_scene import codec as vs  # noqa: E402
from tests.test_vector_codec import worked_scene  # noqa: E402


def vector_frame(body: bytes, key: bool, seq: int = 7) -> TileDeltaFrame:
    return TileDeltaFrame(frame_kind=FRAME_KIND_KEY if key else FRAME_KIND_DELTA,
                          base_seq=seq, grid_w=12, grid_h=8, tile_px=32,
                          codec=CODEC_VECTOR, vector_body=body)


class EnumTests(unittest.TestCase):
    def test_shared_values(self):
        self.assertEqual(CODEC_VECTOR, 6)
        self.assertEqual(int(EncodeMode.VECTOR), 9)
        self.assertEqual((int(Badge.VECTOR), int(Badge.MODEL)), (7, 8))
        self.assertEqual(vs.CODEC_VECTOR, CODEC_VECTOR)


class ParseEncodeTests(unittest.TestCase):
    def setUp(self):
        self.body = vs.encode_frame(vs.Header(True, 1, 3), worked_scene(), 197)

    def test_round_trip_and_byte_for_byte_pass_through(self):
        wire = encode_tile_delta_frame(vector_frame(self.body, key=True))
        self.assertEqual(wire[:HEADER_FIXED_LEN], bytes([1, 7, 12, 8, 32, 6]))
        self.assertEqual(wire[HEADER_FIXED_LEN:], self.body)
        frame = parse_tile_delta_frame(wire)
        self.assertEqual(frame.codec, CODEC_VECTOR)
        self.assertTrue(frame.is_keyframe)
        self.assertEqual((frame.grid_w, frame.grid_h, frame.tile_px), (12, 8, 32))
        self.assertEqual(frame.vector_body, self.body)
        self.assertEqual((frame.tiles, frame.changed_indices), ([], []))
        # what image_rx_daemon._publish_completed does before republishing
        self.assertEqual(encode_tile_delta_frame(frame), wire)
        # and the body still decodes with K agreeing with frame_kind
        decoded = vs.decode_frame(frame.vector_body, frame.frame_kind)
        self.assertEqual(decoded.records, tuple(worked_scene()))

    def test_first_byte_never_collides_with_the_fragment_magics(self):
        for key in (False, True):
            wire = encode_tile_delta_frame(vector_frame(self.body, key))
            self.assertIn(wire[0], (0, 1))
            self.assertNotIn(wire[0], (0xB5, 0xFB, 0xFC, 0xFD, 0xFE))

    def test_short_and_empty_bodies(self):
        with self.assertRaises(FrameDecodeError):
            parse_tile_delta_frame(bytes([1, 7, 12, 8, 32]))
        frame = parse_tile_delta_frame(bytes([0, 7, 12, 8, 32, 6]))
        self.assertEqual(frame.vector_body, b"")
        self.assertEqual(frame.codec, CODEC_VECTOR)

    def test_tile_codecs_still_reject_trailing_bytes(self):
        # the trailing-bytes rule is unchanged for every codec but 6
        with self.assertRaises(FrameDecodeError):
            parse_tile_delta_frame(bytes([1, 7, 12, 8, 32, 0]) + bytes(12) + b"\x01")


class BudgetTests(unittest.TestCase):
    def test_full_frames_fit_one_v1_fragment_at_both_profiles(self):
        for profile, f in ((PHY_IMAGE_BW250, 197), (PHY_IMAGE_BW500, 237)):
            body_max = max_image_fragment_body(profile)
            self.assertEqual(f, body_max - 4 - HEADER_FIXED_LEN)
            body = vs.encode_frame(vs.Header(False, 0, 0), [vs.Del(1)] * ((f * 8 - 13) // 11), f)
            self.assertLessEqual(len(body), f)
            wire = encode_tile_delta_frame(vector_frame(body, key=False))
            frags = pack_image_fragments(wire, 1, profile)
            self.assertEqual(len(frags), 1)
            self.assertLessEqual(len(frags[0]), TX_FRAME_BODY_MAX)
            self.assertLessEqual(lora_time_on_air_ms(len(frags[0]) + LORA_HOP_HDR_LEN, profile),
                                 IMAGE_FRAG_AIR_CAP_MS)

    def test_epoch_start_at_f_minus_1_stays_one_fragment_when_copied(self):
        # §3.1: the daemon's 0xFD copies chunk 1 B smaller than v1 (fixed in #132),
        # so an epoch start capped at F − 1 (196 / 236 B) is still one fragment per copy.
        for profile, f in ((PHY_IMAGE_BW250, 197), (PHY_IMAGE_BW500, 237)):
            cap = f - 1
            body = vs.encode_frame(vs.Header(True, 0, 0), [vs.Del(1)] * ((cap * 8 - 13) // 11), cap)
            self.assertLessEqual(len(body), cap)
            wire = encode_tile_delta_frame(vector_frame(body, key=True))
            for copies in (2, 3):
                frags = pack_image_fragments_v2(wire, 5, profile, IMAGE_FRAG_AIR_CAP_MS, copies=copies)
                self.assertEqual(len(frags), copies, (profile.name, copies))
                for frag in frags:
                    self.assertLessEqual(len(frag), TX_FRAME_BODY_MAX)
                    self.assertLessEqual(lora_time_on_air_ms(len(frag) + LORA_HOP_HDR_LEN, profile),
                                         IMAGE_FRAG_AIR_CAP_MS)


class ReassemblerTests(unittest.TestCase):
    def test_v1_fragment_round_trip_through_the_receiver(self):
        body = vs.encode_frame(vs.Header(True, 2, 4), worked_scene(), 197)
        wire = encode_tile_delta_frame(vector_frame(body, key=True, seq=9))
        rx = FragmentReassembler()
        out = None
        for frag in pack_image_fragments(wire, 3, PHY_IMAGE_BW250):
            out = rx.feed(frag)
        self.assertIsNotNone(out)
        frame = out[0] if isinstance(out, list) else out
        self.assertEqual(frame.codec, CODEC_VECTOR)
        self.assertEqual(frame.vector_body, body)
        self.assertEqual(frame.base_seq, 9)


if __name__ == "__main__":
    unittest.main()
