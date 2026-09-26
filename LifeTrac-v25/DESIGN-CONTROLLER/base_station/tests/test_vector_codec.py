"""Unit tests for the VS1 vector-scene codec (``image_pipeline/vector_scene/codec.py``).

Pins the wire format of ``VECTOR_SCENE.md`` §3.2–3.4 with hand-computed
bit-string goldens (written independently of the codec's own writer), the
record sizes of the §3.3 table, the worked scene of §3.6 (23 records,
1,018 bits, a 129 B body), the Kraft completeness of the prefix code, the
padding and rejection rules of §3.4, the CRC pins behind DIGEST/CONFIRM and
the static palette. A fuzz pass checks that no input makes the decoder raise
anything but ``VsDecodeError`` or apply a frame partially.
"""
from __future__ import annotations

import os
import random
import sys
import unittest

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_BS_DIR = os.path.dirname(_THIS_DIR)
if _BS_DIR not in sys.path:
    sys.path.insert(0, _BS_DIR)

from image_pipeline.vector_scene import codec as vs  # noqa: E402


def bits(s: str) -> bytes:
    """Bit string (spaces ignored) → zero-padded bytes."""
    s = s.replace(" ", "")
    pad = (-len(s)) % 8
    return int(s + "0" * pad, 2).to_bytes((len(s) + pad) // 8, "big") if s else b""


def packed(rec) -> bytes:
    return vs.pack_record(rec).to_bytes()


PAL = vs.Fill(palette=4)                       # 6 bits
RGB_FLAT = vs.Fill(rgb444=0x123)               # 14 bits
RGB_GRAD = vs.Fill(rgb444=0xABC, grad=(5, -1))  # 20 bits
V_PAL = vs.VFill(palette=0, dl=1)              # 9 bits
V_RGB = vs.VFill(rgb444=0x753, dl=-2)          # 17 bits


class PrefixCodeTests(unittest.TestCase):
    def test_kraft_sum_is_exactly_one(self):
        self.assertEqual(vs.kraft_sum(), 1.0)

    def test_prefix_free(self):
        codes = list(vs._PREFIX.values())
        for a in codes:
            for b in codes:
                if a != b:
                    self.assertFalse(b.startswith(a), (a, b))


class Eg2Tests(unittest.TestCase):
    def test_bit_lengths_follow_the_table(self):
        for d, n in ((0, 3), (1, 3), (-1, 3), (-2, 3), (2, 5), (-6, 5), (6, 7),
                     (-14, 7), (14, 9), (-30, 9), (30, 11), (61, 11), (-62, 11)):
            self.assertEqual(vs.eg2_bits(d), n, d)

    def test_golden_codewords(self):
        for d, code in ((0, "100"), (1, "110"), (-1, "101"), (-2, "111"),
                        (2, "01000"), (-6, "01111"), (6, "0010000"),
                        (61, "00001111110"), (-62, "00001111111")):
            w = vs.BitWriter()
            vs._write_eg2(w, d)
            self.assertEqual(w.bit_count, len(code), d)
            self.assertEqual(w.to_bytes(), bits(code), d)

    def test_round_trip_full_range_and_overflow(self):
        for d in range(-62, 62):
            w = vs.BitWriter()
            vs._write_eg2(w, d)
            self.assertEqual(vs._read_eg2(vs.BitReader(w.to_bytes())), d)
        with self.assertRaises(ValueError):
            vs.eg2_bits(62)
        with self.assertRaises(vs.VsDecodeError) as cm:
            vs._read_eg2(vs.BitReader(bits("00000 1000000")))
        self.assertEqual(cm.exception.reason, "eg2_overflow")


class RecordGoldenTests(unittest.TestCase):
    """Bit strings written by hand from the §3.3 field lists."""

    def check(self, rec, s: str):
        self.assertEqual(vs.record_bits(rec), len(s.replace(" ", "")), rec)
        self.assertEqual(packed(rec), bits(s), rec)
        frame = vs.decode_frame(vs.encode_frame(vs.Header(False, 0, 0), [rec], 237))
        self.assertEqual(frame.records, (rec,))

    def test_upd(self):
        self.check(vs.Upd(3, -1, 2), "00 0000011 1111 0010")

    def test_del(self):
        self.check(vs.Del(5), "1011 0000101")

    def test_gshift(self):
        self.check(vs.Gshift(1, -2, 5), "1101 01 11111110 0000101")

    def test_layer_clear(self):
        self.check(vs.LayerClear(2), "111111 0011 10")

    def test_digest(self):
        self.check(vs.Digest(23, 0xD2), "111111 0010 0010111 11010010")

    def test_hzn_resid(self):
        self.check(vs.HznResid(-3, 1), "1010 01 11101 0001")

    def test_hzn_abs(self):
        self.check(vs.HznAbs(64, -1, 0, V_PAL, V_RGB),
                   "1010 00 01000000 111111 0000  0 0000 0001  1 011101010011 1110")

    def test_hzn_colours_and_no_horizon(self):
        self.check(vs.HznColours(V_PAL, V_PAL), "1010 10  0 0000 0001  0 0000 0001")
        self.check(vs.HznNoHorizon(V_RGB, V_RGB),
                   "1010 11  1 011101010011 1110  1 011101010011 1110")

    def test_status(self):
        self.check(vs.Status(2, 30, 0, 64, 3, 7, False, True),
                   "11100 10 0011110 00 1000000 11 111 0 1")

    def test_insert(self):
        self.check(vs.Insert(7, 2, 1, 1, -2), "111111 0000 0000111 0010 01 110 111")

    def test_confirm(self):
        self.check(vs.Confirm(10, (None, 2, None, 3)),
                   "111111 0001 0001010 0011 0101 10 11")

    def test_fills(self):
        self.check(vs.Ucol(9, PAL), "1100 0001001  0 0100 0")
        self.check(vs.Ucol(9, RGB_GRAD), "1100 0001001  1 101010111100 1 101 111")

    def test_tree(self):
        self.check(vs.Tree(40, 10, 3, 1, 2, vs.Fill(palette=3)),
                   "011 0101000 001010 00011 001 010  0 0011 0  0")
        self.check(vs.Tree(40, 10, 3, 1, 2, vs.Fill(palette=3), trunk_h=5),
                   "011 0101000 001010 00011 001 010  0 0011 0  1 101")

    def test_poly(self):
        self.check(vs.Poly(1, 0, ((10, 5), (12, 5), (11, 8)), vs.Fill(palette=2)),
                   "010 0000001 0 000 001010 00101  01000 100  101 01010  0 0010 0")

    def test_edge(self):
        self.check(vs.Edge(60, 1, ((-16, 0), (-13, 2))),
                   "100 0111100 001 0000000 000000 00  01010 01000")

    def test_ext_singletons(self):
        self.check(vs.Gzoom(128), "111111 0110 10000000")
        self.check(vs.Gain(16, 16, 16), "111111 0111 10000 10000 10000")
        self.check(vs.Pal(3, 0x6BE), "111111 0101 011 011010111110")
        self.check(vs.CalRev(0x1234, 0xBEEF),
                   "111111 0100 0001001000110100 1011111011101111")

    def test_hole(self):
        self.check(vs.Hole(1, 2, False, -16, 0, 7),
                   "111111 1000 0000001 10 0 0000000 000000 111")

    def test_anom(self):
        self.check(vs.Anom(5, 1, 2, 0, 7, 0xFFF),
                   "11101 101 000001 00010 000 111 111111111111")

    def test_blob(self):
        self.check(vs.Blob(90, 0, 10, 2, 3, 4, vs.Fill(palette=7)),
                   "11110 1011010 0010000 001010 010 011 100  0 0111 0")

    def test_skyline(self):
        heights = tuple(range(8))
        self.check(vs.Skyline(1, heights, RGB_FLAT),
                   "111110 00 01 " + "".join(f"{h:03b}" for h in heights)
                   + " 1 000100100011 0")


class RecordSizeTests(unittest.TestCase):
    """The sizes of the §3.3 table."""

    def test_fixed_sizes(self):
        cases = {
            vs.Upd(1, 0, 0): 17, vs.Del(1): 11, vs.Gshift(0, 0, 0): 21,
            vs.Status(0, 0, 0, 0, 0, 0, False, False): 30,
            vs.Anom(0, 0, 0, 0, 0, 0): 37, vs.Digest(0, 0): 25, vs.LayerClear(0): 12,
            vs.CalRev(0, 0): 42, vs.Pal(0, 0): 25, vs.Gzoom(0): 18, vs.Gain(0, 0, 0): 25,
            vs.Hole(1, 0, False, 0, 0, 0): 36, vs.HznResid(0, 0): 15,
            vs.HznAbs(0, 0, 0, V_PAL, V_PAL): 42, vs.HznAbs(0, 0, 0, V_RGB, V_RGB): 58,
            vs.HznColours(V_PAL, V_PAL): 24, vs.HznColours(V_RGB, V_RGB): 40,
            vs.HznNoHorizon(V_PAL, V_PAL): 24, vs.HznNoHorizon(V_RGB, V_RGB): 40,
            vs.Ucol(1, PAL): 17, vs.Ucol(1, RGB_GRAD): 31,
            vs.Tree(32, 0, 0, 0, 0, PAL): 34, vs.Tree(32, 0, 0, 0, 0, RGB_GRAD): 48,
            vs.Tree(32, 0, 0, 0, 0, RGB_GRAD, trunk_h=0): 51,
            vs.Blob(88, 0, 0, 0, 0, 0, PAL): 40, vs.Blob(88, 0, 0, 0, 0, 0, RGB_GRAD): 54,
            vs.Skyline(0, (0,) * 8, RGB_FLAT): 48, vs.Skyline(0, (0,) * 12, RGB_FLAT): 60,
            vs.Skyline(0, (0,) * 16, RGB_FLAT): 72, vs.Skyline(0, (0,) * 24, RGB_FLAT): 96,
            vs.Insert(1, 0, 0, 0, 0): 29,
            vs.Confirm(1, (0,) * 16): 69, vs.Confirm(1, (None,)): 22,
        }
        for rec, size in cases.items():
            self.assertEqual(vs.record_bits(rec), size, rec)

    def test_fill_sizes(self):
        self.assertEqual(PAL.bits, 6)
        self.assertEqual(vs.Fill(palette=1, grad=(0, 0)).bits, 12)
        self.assertEqual(RGB_FLAT.bits, 14)
        self.assertEqual(RGB_GRAD.bits, 20)
        self.assertEqual(V_PAL.bits, 9)
        self.assertEqual(V_RGB.bits, 17)

    def test_key_frame_sizes(self):
        abs_key = vs.record_bits(vs.HznAbs(0, 0, 0, V_RGB, V_RGB)) + vs.record_bits(vs.LayerClear(0))
        nh_key = vs.record_bits(vs.HznNoHorizon(V_RGB, V_RGB)) + vs.record_bits(vs.LayerClear(0))
        self.assertEqual((abs_key, nh_key), (70, 52))


def worked_scene() -> list:
    """The §3.6 field-edge scene: 23 records, 1,018 bits."""
    tri = ((10, 10), (12, 16), (14, 22))            # every delta (2, 6) = 5 + 7 bits
    recs = [
        vs.HznAbs(64, 0, 0, V_RGB, V_RGB),                              # 58
        vs.Status(1, 30, 0, 64, 3, 0, False, False),                    # 30
        vs.Poly(1, 0, tri, RGB_GRAD), vs.Poly(2, 0, tri, RGB_GRAD),     # 69 × 2
        vs.Skyline(1, tuple(i % 8 for i in range(12)), RGB_FLAT),       # 60
        vs.Poly(3, 0, tri, RGB_FLAT), vs.Poly(4, 0, tri, RGB_FLAT),
        vs.Poly(5, 0, tri, RGB_FLAT),                                   # 63 × 3
        vs.Tree(32, 5, 5, 2, 3, RGB_FLAT), vs.Tree(33, 20, 6, 2, 3, RGB_FLAT),
        vs.Tree(34, 30, 8, 1, 1, RGB_FLAT),                             # 42 × 3
        vs.Edge(56, 2, ((0, 20), (2, 26), (4, 32), (6, 38))),           # 64
        vs.Edge(57, 0, ((10, 40), (12, 46))), vs.Edge(58, 0, ((30, 40), (32, 46))),  # 40 × 2
        vs.Digest(22, 0xD2),                                            # 25
    ]
    recs += [vs.Insert(1 + (i % 5), i % 3, 0, 2, 0) for i in range(8)]  # 31 × 8
    return recs


class FrameTests(unittest.TestCase):
    def test_header_golden(self):
        body = vs.encode_frame(vs.Header(key=True, age=2, epoch=5, level=0), [vs.Del(5)], 197)
        self.assertEqual(body, bytes([0x24, 0xA5, 0x85]))
        frame = vs.decode_frame(body, frame_kind=1)
        self.assertEqual(frame.header, vs.Header(True, 2, 5, 0))
        self.assertEqual(frame.records, (vs.Del(5),))

    def test_byte0_never_collides_with_the_fragment_magics(self):
        rng = random.Random(3)
        for _ in range(200):
            h = vs.Header(bool(rng.getrandbits(1)), rng.randrange(16), rng.randrange(16), rng.randrange(4))
            body = vs.encode_frame(h, [vs.Del(rng.randrange(1, 128))], 197)
            self.assertLessEqual(body[0], 0x7F)

    def test_worked_scene_bits_and_body(self):
        recs = worked_scene()
        self.assertEqual(len(recs), 23)
        total = sum(vs.record_bits(r) for r in recs)
        self.assertEqual(total, 1018)
        self.assertEqual((1563 - total, 1883 - total), (545, 865))
        body = vs.encode_frame(vs.Header(True, 1, 0), recs, 197)
        self.assertEqual(len(body), 129)
        self.assertEqual(vs.decode_frame(body, frame_kind=1).records, tuple(recs))

    def test_frames_never_exceed_f(self):
        recs = worked_scene()
        for f in (197, 237):
            body = vs.encode_frame(vs.Header(True, 0, 0), recs * 1, f)
            self.assertLessEqual(len(body), f)
        with self.assertRaises(vs.FrameTooLarge):
            vs.encode_frame(vs.Header(True, 0, 0), recs * 2, 197)

    def test_padding_of_0_to_16_bits_at_every_leftover_width(self):
        # Vary the record set so the frame ends at every bit offset within a byte,
        # then append up to two zero bytes: the decoder must stop cleanly each time.
        for k in range(8):
            recs = [vs.Del(1)] * k + [vs.LayerClear(1)]
            base = vs.encode_frame(vs.Header(False, 0, 0), recs, 237)
            for extra in (b"", b"\x00", b"\x00\x00"):
                frame = vs.decode_frame(base + extra)
                self.assertEqual(frame.records, tuple(recs), (k, extra))

    def test_empty_record_list_is_a_valid_frame(self):
        body = vs.encode_frame(vs.Header(False, 3, 9, 2), [], 12)
        self.assertEqual(len(body), 2)
        self.assertEqual(vs.decode_frame(body), vs.Frame(vs.Header(False, 3, 9, 2), ()))


class RejectionTests(unittest.TestCase):
    def reason(self, body: bytes, frame_kind=None) -> str:
        with self.assertRaises(vs.VsDecodeError) as cm:
            vs.decode_frame(body, frame_kind)
        return cm.exception.reason

    def test_marker_version_and_key(self):
        good = vs.encode_frame(vs.Header(True, 0, 0), [vs.Del(1)], 197)
        self.assertEqual(self.reason(bytes([good[0] | 0x80]) + good[1:]), "bad_marker")
        self.assertEqual(self.reason(bytes([0xFE, 0x00, 0x00])), "bad_marker")
        self.assertEqual(self.reason(bytes([good[0] | 0x40]) + good[1:]), "bad_version")
        self.assertEqual(self.reason(good, frame_kind=0), "key_mismatch")
        self.assertEqual(self.reason(b"\x00"), "short")

    def test_truncated_record_with_nonzero_remainder(self):
        good = vs.encode_frame(vs.Header(False, 0, 0), [vs.Gshift(1, -2, 5)], 197)
        self.assertEqual(self.reason(good[:-1]), "truncated")

    def test_reserved_and_range_rules(self):
        hdr = "0 0 0 0000 0000 00"
        self.assertEqual(self.reason(bits(hdr + "111111 1001 00000000")), "reserved_ext")
        self.assertEqual(self.reason(bits(hdr + "111111 1111 00000000")), "reserved_ext")
        # EDGE cls 5
        self.assertEqual(self.reason(bits(hdr + "100 0111100 101 0000000 000000 00 100 100")), "reserved_cls")
        # POLY on a plant id (40)
        self.assertEqual(self.reason(bits(hdr + "010 0101000 0 000 001010 00101 100 100 100 100 0 0010 0")), "bad_id")
        # TREE on a mass id (1); BLOB on an edge id (60)
        self.assertEqual(self.reason(bits(hdr + "011 0000001 001010 00011 001 010 0 0011 0 0")), "bad_id")
        self.assertEqual(self.reason(bits(hdr + "11110 0111100 0010000 001010 010 011 100 0 0111 0")), "bad_id")
        # id 0 on UPD and DEL
        self.assertEqual(self.reason(bits(hdr + "00 0000000 0000 0000 1")), "id_zero")
        self.assertEqual(self.reason(bits(hdr + "1011 0000000 1")), "id_zero")
        # EG2 with five leading zeros inside an INSERT
        self.assertEqual(self.reason(bits(hdr + "111111 0000 0000111 0010 01 000001000000 100")), "eg2_overflow")
        # CONFIRM range running past id 127
        self.assertEqual(self.reason(bits(hdr + "111111 0001 1111111 0001 11 00 00")), "bad_id")

    def test_encoder_refuses_out_of_range_fields(self):
        for bad in (lambda: vs.Upd(0, 0, 0), lambda: vs.Upd(1, 8, 0), lambda: vs.Poly(40, 0, ((0, 0),) * 3, PAL),
                    lambda: vs.Tree(1, 0, 0, 0, 0, PAL), lambda: vs.Edge(56, 5, ((0, 0), (1, 1))),
                    lambda: vs.Fill(), lambda: vs.Fill(palette=1, rgb444=1), lambda: vs.Skyline(0, (0,) * 9, PAL),
                    lambda: vs.Confirm(120, (0,) * 9), lambda: vs.Gshift(0, 128, 0), lambda: vs.Header(False, 16, 0)):
            with self.assertRaises(ValueError):
                rec = bad()
                vs.pack_record(rec)


class FuzzTests(unittest.TestCase):
    def test_random_and_truncated_input_never_raises_anything_else(self):
        rng = random.Random(1234)
        recs = worked_scene()
        valid = vs.encode_frame(vs.Header(True, 1, 0), recs, 197)
        inputs = [bytes(rng.getrandbits(8) for _ in range(rng.randrange(0, 60))) for _ in range(400)]
        inputs += [valid[:n] for n in range(len(valid))]
        inputs += [bytes([0xFE]) + valid[1:], bytes([0xFB, 0x63, 0x09]), bytes([0xB5]) * 10]
        for body in inputs:
            try:
                frame = vs.decode_frame(body)
            except vs.VsDecodeError:
                continue
            # A frame that parses re-encodes to the same records (no partial apply).
            again = vs.encode_frame(frame.header, list(frame.records), 255)
            self.assertEqual(vs.decode_frame(again).records, frame.records)

    def test_bit_flips_are_caught_or_decode_to_valid_records(self):
        rng = random.Random(7)
        recs = worked_scene()
        valid = bytearray(vs.encode_frame(vs.Header(True, 1, 0), recs, 197))
        for _ in range(300):
            flipped = bytearray(valid)
            i = rng.randrange(len(flipped))
            flipped[i] ^= 1 << rng.randrange(8)
            try:
                vs.decode_frame(bytes(flipped))
            except vs.VsDecodeError:
                pass


class CrcAndPaletteTests(unittest.TestCase):
    def test_check_values(self):
        self.assertEqual(vs.crc8(b"123456789"), 0xF4)
        self.assertEqual(vs.crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_digest_golden(self):
        self.assertEqual(vs.digest_crc([(0x2A, 0x1234, 0x5678)], [(0, 0)] * 4, 0x80, (16, 16, 16)), 0xD2)

    def test_state_hash_is_canonical(self):
        a = vs.Insert(1, 0, 1, 2, 0)
        b = vs.Insert(1, 0, 0, -1, 3)
        h1 = vs.state_hash(0, 0, 0x6BE, [a, b], [])
        h2 = vs.state_hash(0, 0, 0x6BE, [b, a], [None, None, None, None])
        self.assertEqual(h1, h2)
        self.assertNotEqual(h1, vs.state_hash(1, 0, 0x6BE, [a, b], []))
        self.assertNotEqual(h1, vs.state_hash(0, 0, 0x6BE, [a, b], [vs.Hole(1, 0, False, 0, 0, 1)]))

    def test_define_hash_changes_with_geometry_only(self):
        p = vs.Poly(1, 0, ((10, 10), (12, 16), (14, 22)), RGB_FLAT)
        q = vs.Poly(1, 0, ((10, 10), (12, 16), (14, 23)), RGB_FLAT)
        self.assertNotEqual(vs.define_hash(p), vs.define_hash(q))
        self.assertEqual(vs.define_hash(p), vs.define_hash(vs.Poly(1, 0, ((10, 10), (12, 16), (14, 22)), RGB_FLAT)))

    def test_static_palette(self):
        self.assertEqual(vs.STATIC_PALETTE, (0x6BE, 0xBBC, 0x252, 0x693, 0xDB6, 0x753, 0x223, 0xFFF))


if __name__ == "__main__":
    unittest.main()
