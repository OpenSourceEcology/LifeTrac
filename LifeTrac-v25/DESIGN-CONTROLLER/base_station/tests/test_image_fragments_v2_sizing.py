"""2026-09-23: v2 (0xFD) duplicate-copy fragment SIZING tests.

`pack_image_fragments_v2` used to build its copies by re-wrapping the
4 B 0xFE fragments from `pack_image_fragments` in the 5 B 0xFD header, so
every copy was 1 B over `max_image_fragment_body()`: 248 B at BW500 (the
L072 refuses length + 8 B hop header > 255, sx1276_tx.c) and 208 B =
171.6 ms at BW250 (over IMAGE_FRAG_AIR_CAP_MS). `image_tx_daemon._pack_for`
takes the copies path for keyframes exactly when
`recent_frag_loss_rate() > 0.005`, i.e. on the degraded link.

Validates:
    S1. A payload of exactly max_image_fragment_body(PHY_IMAGE_BW500) - 4
        bytes (one full v1 fragment at DTS) packs into v2 bodies that are
        all <= TX_FRAME_BODY_MAX and whose on-air time (body +
        LORA_HOP_HDR_LEN) is <= IMAGE_FRAG_AIR_CAP_MS at both
        PHY_IMAGE_BW250 and PHY_IMAGE_BW500.
    S2. The same budget holds for every copy count at payload sizes at and
        just above the v1 and v2 chunk boundaries (fragment count changes),
        and each copy_idx carries the whole payload in order.
    S3. copies == 1 still returns plain v1 (0xFE) fragments; the nibble
        range is enforced.
    S4. Byte-exact round trip of a genuine keyframe through the receiver
        image_rx_daemon runs (image_pipeline.reassemble.FragmentReassembler)
        at both profiles with copies >= 2 and lost copies. The receiver
        joins parts by frag_idx and never infers offsets from a chunk
        size, so the smaller v2 chunk needs no receiver change.
    S5. add_parity_fragments (0xFC, 4 B header) has no such off-by-one:
        a parity fragment is never longer than the v1 fragments it covers.
"""
from __future__ import annotations

import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_BS = os.path.abspath(os.path.join(_HERE, ".."))
if _BS not in sys.path:
    sys.path.insert(0, _BS)

from lora_proto import (   # noqa: E402
    IMAGE_FRAG_AIR_CAP_MS,
    LORA_HOP_HDR_LEN,
    PHY_IMAGE_BW250,
    PHY_IMAGE_BW500,
    TELEMETRY_FRAGMENT_HEADER_LEN,
    TELEMETRY_FRAGMENT_HEADER_LEN_V2,
    TELEMETRY_FRAGMENT_MAGIC_PARITY,
    TELEMETRY_FRAGMENT_MAGIC_V2,
    TX_FRAME_BODY_MAX,
    add_parity_fragments,
    lora_time_on_air_ms,
    max_image_fragment_body,
    pack_image_fragments,
    pack_image_fragments_v2,
    parse_telemetry_fragment,
)
from image_pipeline.frame_format import (   # noqa: E402
    TileBlob,
    TileDeltaFrame,
    encode_tile_delta_frame,
)
from image_pipeline.reassemble import FragmentReassembler   # noqa: E402

_PROFILES = (PHY_IMAGE_BW250, PHY_IMAGE_BW500)
_GRID_W, _GRID_H = 12, 8                        # 96 tiles -> 12 B bitmap
_FRAME_HDR_LEN = 6 + (_GRID_W * _GRID_H + 7) // 8


def _v1_chunk(profile) -> int:
    return max_image_fragment_body(profile) - TELEMETRY_FRAGMENT_HEADER_LEN


def _v2_chunk(profile) -> int:
    return max_image_fragment_body(profile) - TELEMETRY_FRAGMENT_HEADER_LEN_V2


def _payload(n: int) -> bytes:
    return bytes(i & 0xFF for i in range(n))


def _keyframe_of_size(n: int, base_seq: int) -> bytes:
    """A genuine keyframe TileDeltaFrame whose encoding is exactly n bytes
    (header + bitmap, then 1 B size + blob per tile, blobs 1..256 B)."""
    rem = n - _FRAME_HDR_LEN
    k = -(-rem // 257)                          # tiles needed
    base, extra = divmod(rem - k, k)
    tiles = []
    for i in range(k):
        blob_len = base + (1 if i < extra else 0)
        tiles.append(TileBlob(i, i % _GRID_W, i // _GRID_W,
                              bytes((i + j) & 0xFF for j in range(blob_len))))
    frame = TileDeltaFrame(frame_kind=1, base_seq=base_seq, grid_w=_GRID_W,
                           grid_h=_GRID_H, tile_px=32,
                           changed_indices=[t.index for t in tiles],
                           tiles=tiles)
    payload = encode_tile_delta_frame(frame)
    assert len(payload) == n, (len(payload), n)
    return payload


class TestV2CopySizing(unittest.TestCase):

    def _assert_fits(self, frag: bytes, profile) -> None:
        self.assertLessEqual(len(frag), max_image_fragment_body(profile))
        self.assertLessEqual(len(frag), TX_FRAME_BODY_MAX)
        self.assertLessEqual(
            lora_time_on_air_ms(len(frag) + LORA_HOP_HDR_LEN, profile),
            IMAGE_FRAG_AIR_CAP_MS)

    def test_S1_full_dts_v1_body_copies_fit_radio_and_air_cap(self):
        # One full v1 fragment at DTS: 247 B body -> 243 B payload. The
        # re-wrapped copies were 248 B (L072 refuses) / 208 B = 171.6 ms.
        payload = _payload(max_image_fragment_body(PHY_IMAGE_BW500)
                           - TELEMETRY_FRAGMENT_HEADER_LEN)
        for profile in _PROFILES:
            for copies in (2, 3):
                frags = pack_image_fragments_v2(payload, 9, profile,
                                                IMAGE_FRAG_AIR_CAP_MS,
                                                copies=copies)
                self.assertTrue(frags)
                for f in frags:
                    self.assertEqual(f[0], TELEMETRY_FRAGMENT_MAGIC_V2)
                    self._assert_fits(f, profile)

    def test_S2_budget_and_ordering_at_chunk_boundaries(self):
        for profile in _PROFILES:
            c1, c2 = _v1_chunk(profile), _v2_chunk(profile)
            self.assertEqual(c2, c1 - 1)
            for n in (1, c2, c2 + 1, c1, c1 + 1, 2 * c2, 2 * c2 + 1,
                      2 * c1 + 1, 5 * c1):
                payload = _payload(n)
                total = -(-n // c2)
                for copies in (2, 4, 15):
                    frags = pack_image_fragments_v2(
                        payload, 33, profile, IMAGE_FRAG_AIR_CAP_MS,
                        copies=copies)
                    self.assertEqual(len(frags), total * copies)
                    by_copy: dict[int, list[bytes]] = {
                        c: [] for c in range(copies)}
                    for k, f in enumerate(frags):
                        self._assert_fits(f, profile)
                        seq, idx, tot, data = parse_telemetry_fragment(f)
                        self.assertEqual((seq, tot), (33, total))
                        self.assertEqual(idx, k // copies)
                        self.assertEqual(f[4], (copies << 4) | (k % copies))
                        by_copy[k % copies].append(data)
                    for parts in by_copy.values():
                        self.assertEqual(b"".join(parts), payload)

    def test_S3_single_copy_is_plain_v1_and_nibble_range(self):
        payload = _payload(3 * _v1_chunk(PHY_IMAGE_BW250) + 7)
        for profile in _PROFILES:
            self.assertEqual(
                pack_image_fragments_v2(payload, 5, profile, copies=1),
                pack_image_fragments(payload, 5, profile))
        for bad in (0, 16):
            with self.assertRaises(ValueError):
                pack_image_fragments_v2(payload, 5, copies=bad)

    def test_S4_receiver_round_trip_with_lost_copies(self):
        for profile in _PROFILES:
            c1 = _v1_chunk(profile)
            # Exactly one v1 fragment (now a full v2 body + a 1 B runt),
            # one past it, and one past two -- the totals that change.
            for seq, n in enumerate((c1, c1 + 1, 2 * c1 + 1), start=60):
                payload = _keyframe_of_size(n, base_seq=seq)
                for copies in (2, 3):
                    frags = pack_image_fragments_v2(
                        payload, seq, profile, IMAGE_FRAG_AIR_CAP_MS,
                        copies=copies)
                    total = len(frags) // copies
                    # Loss patterns: every copy 0 lost; a different copy
                    # of each fragment lost; nothing lost.
                    for lost in (lambda idx, c: c == 0,
                                 lambda idx, c: c == idx % copies,
                                 lambda idx, c: False):
                        ras = FragmentReassembler()
                        got = None
                        for f in frags:
                            if lost(f[2], f[4] & 0x0F):
                                continue
                            out = ras.feed(f)
                            if out is not None:
                                got = out
                        self.assertIsNotNone(got, (profile.name, n, copies))
                        self.assertTrue(got.is_keyframe)
                        self.assertEqual(encode_tile_delta_frame(got), payload)
                        self.assertEqual(ras.stats.completed_frames, 1)
                        self.assertEqual(ras.stats.decode_errors, 0)
                        self.assertEqual(ras.pending_frag_seqs(), [])
                    # Every copy of one fragment lost: must stay pending.
                    ras = FragmentReassembler()
                    for f in frags:
                        if f[2] == total - 1:
                            continue
                        self.assertIsNone(ras.feed(f))
                    self.assertEqual(ras.pending_frag_seqs(), [seq])

    def test_S5_parity_fragments_never_exceed_v1_body(self):
        for profile in _PROFILES:
            payload = _payload(9 * _v1_chunk(profile) + 5)
            frags = pack_image_fragments(payload, 12, profile)
            with_parity = add_parity_fragments(frags, 12, group_len=8)
            parity = [f for f in with_parity
                      if f[0] == TELEMETRY_FRAGMENT_MAGIC_PARITY]
            self.assertEqual(len(parity), -(-len(frags) // 8))
            for f in parity:
                self.assertLessEqual(len(f), max(len(d) for d in frags))
                self._assert_fits(f, profile)


if __name__ == "__main__":
    unittest.main()
