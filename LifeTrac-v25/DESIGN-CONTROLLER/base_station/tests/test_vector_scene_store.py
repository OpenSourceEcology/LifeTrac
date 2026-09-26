"""Unit tests for the VS1 vector scene store (``image_pipeline/vector_scene_store.py``).

The §8.6 store row of ``VECTOR_SCENE.md``: epoch hand-over (from an ABS and a
NO_HORIZON anchor, with every LAYER_CLEAR range), tractor reboot, the 9-epoch
jump, per-field last-writer-wins with duplicates, reordering and saturated
ages, INSERT permutations and subsets, CONFIRM/DIGEST gating of age resets,
orphans → RESYNC with no uplink, TTL, HOLE slots, lossy ⊆ lossless and
convergence under i.i.d. and burst loss. Frames are built with the codec and
driven with explicit ``rx_ms`` values so every age is deterministic. The
tractor is simulated with the codec directly (``Tractor`` below).
"""
from __future__ import annotations

import itertools
import json
import math
import os
import random
import sys
import unittest

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_BS_DIR = os.path.dirname(_THIS_DIR)
if _BS_DIR not in sys.path:
    sys.path.insert(0, _BS_DIR)

from image_pipeline.vector_scene import codec as vs  # noqa: E402
from image_pipeline.vector_scene_store import (  # noqa: E402
    BADGE_CACHED, BADGE_PREDICTED, BADGE_VECTOR, TTL_FRAMES, IngestResult, VectorSceneStore,
)

SKY = vs.VFill(rgb444=0x6BE, dl=-2)
GND = vs.VFill(rgb444=0x753, dl=1)
FLAT = vs.Fill(rgb444=0x693)                    # "#669933"
GRAD = vs.Fill(rgb444=0x693, grad=(4, 1))       # dir 90° (down), ΔL = +16
PAL2 = vs.Fill(palette=2)                       # dark foliage 0x252
ABS = vs.HznAbs(64, 0, 0, SKY, GND)             # y_px = 0 at x = 192, flat
NOHZ = vs.HznNoHorizon(SKY, GND)
NEUTRAL = ([(0, 0)] * 4, 128, (16, 16, 16))


def key(rng: int = 0, anchor=ABS) -> list:
    """An epoch start: the absolute anchor plus LAYER_CLEAR (§3.5)."""
    return [anchor, vs.LayerClear(rng)]


def tri(id_: int, y: int = 10, grid: int = 0, fill: vs.Fill = FLAT) -> vs.Poly:
    """A mass below the y = 0 horizon (ground group) at cell (10, y)."""
    return vs.Poly(id_, grid, ((10, y), (20, y), (20, y + 10)), fill)


def body(key_: bool, epoch: int, recs, age: int = 1, level: int = 0) -> bytes:
    return vs.encode_frame(vs.Header(key_, age, epoch, level), list(recs), 237)


def shash(rec, dx=0, dy=0, inserts=(), holes=()) -> int:
    """The tractor-side state-hash of a define with the given live state."""
    if isinstance(rec, vs.Edge):
        rgb = 0
    elif isinstance(rec, vs.Anom):
        rgb = rec.rgb444
    else:
        rgb = vs.STATIC_PALETTE[rec.fill.palette] if rec.fill.palette is not None else rec.fill.rgb444
    return vs.state_hash(dx, dy, rgb, list(inserts), list(holes))


def digest(rows, gshifts=None, zoom: int = 128, gain=(16, 16, 16)) -> vs.Digest:
    rows = list(rows)
    return vs.Digest(len(rows), vs.digest_crc(rows, gshifts or [(0, 0)] * 4, zoom, gain))


class StoreCase(unittest.TestCase):
    """Frames every 500 ms with AAAA = 1, so capture = rx − 200."""

    def setUp(self):
        self.st = VectorSceneStore()
        self.rx = 10_000

    def feed(self, recs, epoch: int = 0, key_: bool = False, rx: int | None = None,
             age: int = 1, airtime: float = 0.0) -> IngestResult:
        if rx is None:
            self.rx += 500
        else:
            self.rx = rx
        return self.st.ingest(body(key_, epoch, recs, age), 1 if key_ else 0, self.rx, airtime)

    def snap(self, now: int | None = None) -> dict:
        return self.st.snapshot(self.rx + 100 if now is None else now)

    @staticmethod
    def shapes_of(snap: dict) -> dict:
        return {s["id"]: s for layer in snap["layers"] for s in layer["shapes"] if s["k"] != "hole"}

    def shapes(self) -> dict:
        return self.shapes_of(self.snap())

    def holes(self) -> list:
        return [s for layer in self.snap()["layers"] for s in layer["shapes"] if s["k"] == "hole"]


# ---------------------------------------------------------------- epochs (§3.5)

class EpochTests(StoreCase):
    def test_snapshot_is_none_until_a_frame_is_applied(self):
        self.assertIsNone(self.st.snapshot(0))
        r = self.feed(key() + [tri(1)], epoch=3, key_=True)
        self.assertEqual((r.applied, r.reason, r.epoch_switched, r.records), (True, None, True, 3))
        self.assertEqual(self.snap()["epoch"], 3)
        self.assertEqual(set(self.shapes()), {1})

    def test_same_epoch_key_repeat_is_idempotent(self):
        self.feed(key() + [tri(1)], epoch=3, key_=True)
        r = self.feed(key() + [tri(2)], epoch=3, key_=True)      # the daemon's copy / repeat-once
        self.assertTrue(r.applied)
        self.assertFalse(r.epoch_switched)
        self.assertEqual(set(self.shapes()), {1, 2})
        self.assertEqual(self.st.stats["epochs"], 1)

    def test_tractor_reboot_epoch_3_to_0(self):
        self.feed(key() + [tri(1)], epoch=3, key_=True)
        r = self.feed(key() + [tri(5), digest([(5, vs.define_hash(tri(5)), shash(tri(5)))])],
                      epoch=0, key_=True)                         # 13 "ahead": behind, but K = 1 and later
        self.assertTrue(r.applied and r.epoch_switched)
        self.assertEqual(self.st.stats["epoch_behind"], 0)
        s = self.snap()
        self.assertEqual(s["epoch"], 0)
        self.assertFalse(s["handover"])                            # LAYER_CLEAR 0: nothing carried
        self.assertEqual(set(self.shapes()), {5})
        r = self.feed(key() + [tri(7)], epoch=9, key_=True, rx=self.rx, age=10)   # K = 1 but older capture
        self.assertEqual((r.applied, r.reason), (False, "epoch_behind"))
        self.assertEqual(self.st.stats["epoch_behind"], 1)

    def test_nine_epoch_jump_after_an_outage(self):
        self.feed(key() + [tri(1)], epoch=3, key_=True)
        r = self.feed([tri(2)], epoch=12, rx=self.rx + 5000)       # K = 0, 9 ahead, > 3 s of silence
        self.assertTrue(r.applied and r.epoch_switched)
        self.assertEqual(self.snap()["epoch"], 12)
        self.assertEqual(self.st.stats["epoch_behind"], 0)

    def test_one_to_seven_ahead_switches_without_a_key_across_the_wrap(self):
        self.feed(key() + [tri(1)], epoch=15, key_=True)
        self.assertTrue(self.feed([tri(2)], epoch=0).epoch_switched)          # 1 ahead
        self.assertTrue(self.feed([tri(2)], epoch=7).epoch_switched)          # 7 ahead
        r = self.feed([tri(2)], epoch=15)                                     # 8 ahead within 3 s
        self.assertEqual((r.applied, r.reason), (False, "epoch_behind"))

    def test_behind_frames_are_dropped_and_counted(self):
        self.feed(key() + [tri(1)], epoch=5, key_=True)
        r = self.feed([tri(2), vs.Upd(1, 1, 1)], epoch=4)
        self.assertEqual((r.applied, r.reason, r.records), (False, "epoch_behind", 2))
        self.assertEqual(self.st.stats["epoch_behind"], 1)
        self.assertEqual(self.st.stats["frames_applied"], 1)
        self.assertEqual(self.shapes()[1]["pts"][:2], [80.0, 80.0])          # nothing applied

    def test_third_consecutive_behind_frame_with_increasing_capture_self_heals(self):
        self.feed(key() + [tri(1)], epoch=5, key_=True)
        self.assertFalse(self.feed([tri(2)], epoch=4).applied)
        self.assertFalse(self.feed([tri(2)], epoch=4).applied)
        r = self.feed([tri(2)], epoch=4)
        self.assertTrue(r.applied and r.epoch_switched)
        self.assertEqual(self.st.stats["epoch_behind"], 2)
        self.assertEqual(self.snap()["epoch"], 4)

    def test_behind_run_needs_strictly_increasing_captures_and_resets_on_an_applied_frame(self):
        self.feed(key() + [tri(1)], epoch=5, key_=True)
        rx = self.rx + 100
        for _ in range(4):
            self.assertFalse(self.feed([tri(2)], epoch=4, rx=rx).applied)     # same capture each time
        self.assertFalse(self.feed([tri(2)], epoch=4, rx=rx + 500).applied)
        self.feed([], epoch=5, rx=rx + 600)                                   # an applied frame resets the run
        self.assertFalse(self.feed([tri(2)], epoch=4, rx=rx + 1000).applied)
        self.assertFalse(self.feed([tri(2)], epoch=4, rx=rx + 1500).applied)
        self.assertEqual(self.snap()["epoch"], 5)


# ---------------------------------------------------------------- hand-over (§3.5, §3.3 LAYER_CLEAR)

class HandoverTests(StoreCase):
    OLD = {1, 2, 32, 56, 90}

    def start_epoch0(self):
        self.feed(key() + [tri(1), tri(2, y=20), vs.Tree(32, 5, 5, 2, 3, PAL2),
                           vs.Edge(56, 2, ((0, 20), (2, 26))), vs.Blob(90, 30, 40, 1, 2, 3, PAL2)],
                  epoch=0, key_=True)
        self.t_old = self.rx - 200

    def test_previous_epoch_stays_cached_until_anchor_and_half_the_digest(self):
        self.start_epoch0()
        r = self.feed([tri(3)], epoch=1)                            # epoch start lost: no anchor yet
        self.assertTrue(r.epoch_switched)
        s = self.snap()
        self.assertTrue(s["handover"])
        self.assertEqual((s["epoch"], s["cached_epoch"]), (1, 0))
        shapes = self.shapes_of(s)
        self.assertEqual(set(shapes), self.OLD)
        self.assertTrue(all(sh["badge"] == BADGE_CACHED for sh in shapes.values()))
        self.assertTrue(all(layer["badge"] == BADGE_CACHED for layer in s["layers"]))
        self.assertEqual(s["horizon"]["badge"], BADGE_CACHED)
        self.assertEqual(s["badge"], BADGE_VECTOR)                  # the scene itself is VECTOR data
        self.assertEqual(s["anchor_age_ms"], self.rx + 100 - self.t_old)
        self.feed(key() + [digest([(3, 0, 0), (4, 0, 0)])], epoch=1)   # repeat-once copy; 1 of 2 present
        s = self.snap()
        self.assertFalse(s["handover"])
        self.assertEqual(set(self.shapes_of(s)), {3})
        self.assertEqual(self.shapes_of(s)[3]["badge"], BADGE_VECTOR)
        self.assertEqual(self.st.stats["handovers"], 1)

    def test_less_than_half_the_digest_keeps_the_cache(self):
        self.start_epoch0()
        self.feed(key() + [tri(3), digest([(i, 0, 0) for i in (3, 4, 5)])], epoch=1, key_=True)
        self.assertTrue(self.snap()["handover"])                    # 1 of 3 present
        self.feed([tri(4)], epoch=1)                                # 2 of 3 (and the 2-frame timeout)
        self.assertFalse(self.snap()["handover"])

    def test_handover_from_a_no_horizon_anchor(self):
        self.start_epoch0()
        self.feed(key(anchor=NOHZ) + [tri(3), digest([(3, 0, 0)])], epoch=1, key_=True)
        s = self.snap()
        self.assertFalse(s["handover"])
        self.assertEqual(s["horizon"]["mode"], "none")
        self.assertEqual(set(self.shapes_of(s)), {3})
        self.assertEqual(s["anchor_age_ms"], 300)

    def test_only_the_digest_half_times_out_after_two_frames(self):
        self.start_epoch0()
        self.feed(key() + [tri(3)], epoch=1, key_=True)             # anchor, no DIGEST
        self.assertTrue(self.snap()["handover"])
        self.feed([], epoch=1)
        self.assertFalse(self.snap()["handover"])
        self.assertEqual(set(self.shapes()), {3})

    def test_the_anchor_half_never_times_out(self):
        self.start_epoch0()
        for _ in range(6):
            self.feed([tri(3), digest([(3, 0, 0)])], epoch=1)
        s = self.snap()
        self.assertTrue(s["handover"])
        self.assertEqual(set(self.shapes_of(s)), self.OLD)
        self.assertEqual(s["horizon"]["mode"], "abs")               # the cached anchor is shown

    def test_each_layer_clear_range_carries_the_other_layers_with_ids_and_ages(self):
        carried_by_range = {0: set(), 1: {1, 2}, 2: {1, 2, 32}, 3: {1, 2, 32, 56}}
        for rng, carried in carried_by_range.items():
            with self.subTest(range=rng):
                self.setUp()
                self.start_epoch0()
                self.feed(key(rng) + [tri(3), digest([(i, 0, 0) for i in sorted(carried | {3})])],
                          epoch=1, key_=True)
                s = self.snap()
                shapes = self.shapes_of(s)
                self.assertFalse(s["handover"])
                self.assertEqual(set(shapes), carried | {3})
                for id_ in carried:
                    self.assertEqual(shapes[id_]["age_ms"], self.rx + 100 - self.t_old, id_)
                    self.assertEqual(shapes[id_]["badge"], BADGE_VECTOR)
                self.assertEqual(shapes[3]["age_ms"], 300)
                self.assertEqual(s["horizon"]["mode"], "abs")       # M and L0 are never cleared

    def test_a_carried_shape_keeps_its_shifted_position_and_takes_new_shifts(self):
        self.feed(key() + [tri(1)], epoch=0, key_=True)
        self.feed([vs.Gshift(1, 5, 0)], epoch=0)                    # ground +10 px
        p0 = self.shapes()[1]["pts"]
        self.assertEqual(p0[:2], [90.0, 80.0])
        self.feed(key(3) + [digest([(1, 0, 0)])], epoch=1, key_=True)
        self.assertEqual(self.shapes()[1]["pts"], p0)
        self.feed([vs.Gshift(2, 1, 1)], epoch=1)                    # the new epoch's shift adds on top
        self.assertEqual(self.shapes()[1]["pts"][:2], [92.0, 82.0])

    def test_records_for_a_carriable_cached_shape_apply_to_it(self):
        self.feed(key() + [tri(1)], epoch=0, key_=True)
        self.feed([vs.Upd(1, 2, 0)], epoch=1)                       # before the new epoch's anchor
        self.assertEqual(self.st.stats["orphans"], 0)
        self.feed(key(3), epoch=1)
        self.feed([], epoch=1)
        self.assertEqual(self.shapes()[1]["pts"][:2], [88.0, 80.0])


# ---------------------------------------------------------------- LAYER_CLEAR (§3.3)

class LayerClearTests(StoreCase):
    def setUp(self):
        super().setUp()
        self.feed(key() + [tri(1), vs.Tree(32, 5, 5, 2, 3, PAL2), vs.Edge(56, 2, ((0, 20), (2, 26))),
                           vs.Blob(90, 30, 40, 1, 2, 3, PAL2)], epoch=0, key_=True)

    def test_a_repeat_of_the_same_pair_is_ignored(self):
        self.feed(key(0) + [tri(2)], epoch=0)                       # the repeat-once copy
        self.assertEqual(set(self.shapes()), {1, 2, 32, 56, 90})

    def test_a_different_range_mid_epoch_clears_at_once_and_l0_survives(self):
        self.feed([vs.LayerClear(2)], epoch=0)
        self.assertEqual(set(self.shapes()), {1, 32})
        self.feed([vs.LayerClear(2)], epoch=0)
        self.assertEqual(set(self.shapes()), {1, 32})
        self.feed([vs.LayerClear(0)], epoch=0)
        self.assertEqual(set(self.shapes()), set())
        self.assertEqual(self.snap()["horizon"]["mode"], "abs")


# ---------------------------------------------------------------- per-field LWW (§3.4 rule 5)

class LwwTests(StoreCase):
    @staticmethod
    def canonical(snap: dict) -> dict:
        snap = dict(snap)
        snap.pop("stats")
        snap.pop("bits")
        return snap

    def test_duplicates_and_reordering_converge_to_the_same_snapshot(self):
        t0 = 10_000
        frames = [(t0 + 1000, True, key() + [tri(1), tri(2, y=20)]),
                  (t0 + 1500, False, [vs.Upd(1, 2, -1), vs.Gain(20, 16, 12), vs.Gshift(0, 1, 0)]),
                  (t0 + 2000, False, [vs.Ucol(1, GRAD), vs.Gshift(1, 3, 0), vs.HznResid(2, 0)]),
                  (t0 + 2500, False, [vs.Upd(1, 1, 1), vs.Insert(2, 0, 0, 0, -2), vs.Pal(0, 0xABC)]),
                  (t0 + 3000, False, [vs.Gshift(1, 6, 1), vs.Gzoom(130), vs.Gain(18, 18, 18),
                                      vs.Hole(2, 1, False, 40, 50, 1)])]

        def run(order):
            st = VectorSceneStore()
            for i in order:
                rx, k, recs = frames[i]
                st.ingest(body(k, 0, recs), 1 if k else 0, rx)
            return self.canonical(st.snapshot(t0 + 4000))

        ref = run([0, 1, 2, 3, 4])
        self.assertEqual(self.shapes_of(ref)[1]["age_ms"], 4000 - 2300)
        rng = random.Random(7)
        for _ in range(40):
            order = [1, 2, 3, 4] + [rng.randrange(1, 5) for _ in range(3)]
            rng.shuffle(order)
            order = [0] + order                          # defines first: rule 4 makes the rest orphans
            self.assertEqual(run(order), ref, order)

    def test_same_hash_define_refreshes_only_the_geometry_age(self):
        self.feed(key() + [tri(1)], epoch=0, key_=True)
        self.feed([vs.Upd(1, 2, 0), vs.Insert(1, 0, 0, 0, -2), vs.Ucol(1, GRAD)], epoch=0)
        before = self.shapes()[1]
        self.assertEqual(len(before["pts"]), 8)
        self.feed([tri(1)], epoch=0)                                # carousel re-send, same define-hash
        after = self.shapes()[1]
        self.assertEqual(after["pts"], before["pts"])               # offset and INSERT kept
        self.assertEqual(after["fill"], before["fill"])             # the UCOL colour kept
        self.assertEqual(after["age_ms"], 300)

    def test_new_hash_resets_the_offset_and_clears_inserts(self):
        self.feed(key() + [tri(1)], epoch=0, key_=True)
        self.feed([vs.Upd(1, 2, 0), vs.Insert(1, 0, 0, 0, -2)], epoch=0)
        self.feed([tri(1, y=12)], epoch=0)
        self.assertEqual(self.shapes()[1]["pts"], [80.0, 96.0, 160.0, 96.0, 160.0, 176.0])

    def test_an_older_define_arriving_later_is_ignored(self):
        self.feed(key() + [tri(1)], epoch=0, key_=True)
        rx = self.rx
        self.feed([tri(1, y=12)], epoch=0, rx=rx + 1000)
        self.feed([tri(1, y=14), vs.Ucol(1, GRAD)], epoch=0, rx=rx + 500)   # captured earlier: loses
        s = self.shapes()[1]
        self.assertEqual(s["pts"][1], 96.0)
        self.assertNotIn("c1", s["fill"])

    def test_saturated_age_fills_absent_state_but_never_supersedes_known_state(self):
        self.feed(key() + [tri(1)], epoch=0, key_=True)
        self.feed([vs.Upd(1, 2, 0), vs.Gshift(1, 4, 0)], epoch=0)
        self.assertEqual(self.shapes()[1]["pts"][0], 96.0)          # 80 + 8 (UPD) + 8 (ground)
        r = self.feed([vs.Upd(1, -5, 0), vs.Gshift(1, -8, 0), vs.Gshift(0, 7, 0), tri(9, y=20),
                       vs.Ucol(1, GRAD)], epoch=0, age=15)
        self.assertTrue(r.applied)
        s = self.shapes()
        self.assertEqual(s[1]["pts"][0], 96.0)                      # known offset and shift kept
        self.assertNotIn("c1", s[1]["fill"])                        # known colour kept
        self.assertIn(9, s)                                         # absent shape filled ...
        self.assertGreaterEqual(s[9]["age_ms"], 3000)               # ... and rendered stale
        rows = [(1, vs.define_hash(tri(1)), shash(tri(1), 2, 0)),
                (9, vs.define_hash(tri(9, y=20)), shash(tri(9, y=20)))]
        self.feed([digest(rows, [(7, 0), (4, 0), (0, 0), (0, 0)])], epoch=0)   # far shift was absent: filled
        self.assertTrue(self.snap()["digest_ok"])

    def test_group_transform_at_define_never_double_shifts(self):
        self.feed(key() + [tri(1)], epoch=0, key_=True)             # defined at shift 0
        self.feed([vs.Gshift(1, 10, 0)], epoch=0)                   # ground +20 px
        self.assertEqual(self.shapes()[1]["pts"][:2], [100.0, 80.0])
        self.feed([tri(2, y=20)], epoch=0)                          # redefine-style: at the current position
        self.assertEqual(self.shapes()[2]["pts"][:2], [80.0, 160.0])
        self.feed([vs.Gshift(1, 15, 0), vs.Gshift(2, 0, 1)], epoch=0)   # ground 30, all +2 px in y
        s = self.shapes()
        self.assertEqual(s[1]["pts"][:2], [110.0, 82.0])
        self.assertEqual(s[2]["pts"][:2], [90.0, 162.0])

    def test_group_membership_far_ground_and_l4(self):
        self.feed(key() + [vs.Tree(32, 5, 0, 1, 1, PAL2), tri(1), vs.Blob(90, 30, 40, 1, 2, 3, PAL2)],
                  epoch=0, key_=True)
        self.feed([vs.Gshift(1, 5, 0), vs.Gshift(0, 1, 0), vs.Gshift(3, 2, 0)], epoch=0)
        s = self.shapes()
        self.assertEqual(s[32]["cx"], 42.0)                         # on the horizon: far, +2
        self.assertEqual(s[1]["pts"][0], 90.0)                      # ground, +10
        self.assertEqual(s[90]["cx"], 124.0)                        # L4, +4

    def test_gain_scales_every_colour_and_clips(self):
        self.feed(key() + [tri(1), vs.Tree(32, 5, 5, 2, 3, vs.Fill(rgb444=0x888))], epoch=0, key_=True)
        self.feed([vs.Gain(16, 31, 0)], epoch=0)                    # ×1, ×1.38, ×0.71
        g, b = 2 ** (15 / 32), 2 ** (-16 / 32)
        s = self.shapes()
        self.assertEqual(s[32]["fill"]["c0"], "#%02x%02x%02x" % (136, round(136 * g), round(136 * b)))
        self.assertEqual(s[1]["fill"]["c0"], "#%02x%02x%02x" % (102, min(255, round(153 * g)), round(51 * b)))
        sky_top = (102 - 16, 255 - 16, round(238 * b) - 16)         # base ± ΔL after the gain, clipped
        self.assertEqual(self.snap()["horizon"]["sky"][0], "#%02x%02x%02x" % sky_top)


# ---------------------------------------------------------------- INSERT ring (§3.3)

def _ccw(a, b, c):
    return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])


def _cross(p, q, r, s):
    return _ccw(p, r, s) != _ccw(q, r, s) and _ccw(p, q, r) != _ccw(p, q, s)


def is_simple(flat: list) -> bool:
    pts = [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]
    n = len(pts)
    segs = [(pts[i], pts[(i + 1) % n]) for i in range(n)]
    for i in range(n):
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:
                continue
            if _cross(*segs[i], *segs[j]):
                return False
    return True


class InsertRingTests(unittest.TestCase):
    QUAD = vs.Poly(1, 0, ((10, 10), (30, 10), (30, 30), (10, 30)), FLAT)
    INS = (vs.Insert(1, 0, 0, -4, -3), vs.Insert(1, 0, 1, 4, -3), vs.Insert(1, 1, 0, 4, 0),
           vs.Insert(1, 2, 0, 0, 4))
    # the canonical full ring, tagged with the INSERT each point comes from
    TAGGED = [((80, 80), None), ((128, 56), 0), ((192, 56), 1), ((240, 80), None), ((272, 160), 2),
              ((240, 240), None), ((160, 272), 3), ((80, 240), None)]

    def ring_after(self, inserts, per_frame: int = 1) -> tuple[VectorSceneStore, list]:
        st = VectorSceneStore()
        st.ingest(body(True, 0, key() + [self.QUAD]), 1, 10_000)
        inserts = list(inserts)
        for i in range(0, len(inserts), per_frame):
            st.ingest(body(False, 0, inserts[i:i + per_frame]), 0, 10_500 + 500 * i)
        return st, StoreCase.shapes_of(st.snapshot(20_000))[1]["pts"]

    def test_every_permutation_gives_the_same_ring(self):
        full = [v for p, _ in self.TAGGED for v in p]
        for perm in itertools.permutations(self.INS):
            self.assertEqual(self.ring_after(perm)[1], full, perm)
        self.assertEqual(self.ring_after(self.INS * 2, per_frame=4)[1], full)   # duplicates, whole frames

    def test_every_subset_is_a_simple_sub_ring(self):
        for mask in range(16):
            sub = [ins for i, ins in enumerate(self.INS) if mask >> i & 1]
            expected = [v for p, src in self.TAGGED if src is None or mask >> src & 1 for v in p]
            _, pts = self.ring_after(sub)
            self.assertEqual(pts, expected, mask)
            self.assertTrue(is_simple(pts), mask)

    def test_insert_naming_an_unknown_id_or_edge_is_an_orphan(self):
        st, pts = self.ring_after([vs.Insert(2, 0, 0, 1, 1), vs.Insert(1, 4, 0, 1, 1)])
        self.assertEqual(st.stats["orphans"], 2)
        self.assertEqual(len(pts), 8)


# ---------------------------------------------------------------- CONFIRM / DIGEST (§4.3)

class ConfirmDigestTests(StoreCase):
    DH1, SH1 = vs.define_hash(tri(1)), shash(tri(1))

    def test_confirm_with_matching_tag_and_digest_resets_the_age(self):
        self.feed(key() + [tri(1)], epoch=0, key_=True)
        self.feed([], epoch=0)
        self.feed([], epoch=0)
        self.assertEqual(self.shapes()[1]["age_ms"], 1300)
        self.feed([vs.Confirm(1, (self.SH1 & 3,)), digest([(1, self.DH1, self.SH1)])], epoch=0)
        self.assertEqual(self.shapes()[1]["age_ms"], 300)
        self.assertTrue(self.snap()["digest_ok"])
        self.assertEqual(self.st.stats["orphans"], 0)

    def test_a_lost_upd_followed_by_confirm_does_not_reset_the_age(self):
        self.feed(key() + [tri(1)], epoch=0, key_=True)
        for dx in range(1, 8):                                       # an offset whose 2-bit tag differs
            if shash(tri(1), dx, 0) & 3 != self.SH1 & 3:
                break
        else:
            self.fail("no differing tag")
        sh_t = shash(tri(1), dx, 0)                                  # the tractor's mirror after the lost UPD
        self.feed([], epoch=0)
        self.feed([vs.Confirm(1, (sh_t & 3,))], epoch=0)             # tag alone mismatches
        self.assertEqual(self.shapes()[1]["age_ms"], 1300)
        self.assertEqual(self.st.stats["orphans"], 1)
        # a tag that happens to pass is caught by the DIGEST over the 16-bit state-hashes
        self.feed([vs.Confirm(1, (self.SH1 & 3,)), digest([(1, self.DH1, sh_t)])], epoch=0)
        self.assertEqual(self.shapes()[1]["age_ms"], 1800)
        self.assertFalse(self.snap()["digest_ok"])
        self.assertEqual(self.st.stats["orphans"], 1)               # blocked by the DIGEST, not an orphan
        # the UPD arrives (carousel re-send): the next CONFIRM + DIGEST resets it
        self.feed([vs.Upd(1, dx, 0), vs.Confirm(1, (sh_t & 3,)), digest([(1, self.DH1, sh_t)])], epoch=0)
        self.assertEqual(self.shapes()[1]["age_ms"], 300)
        self.assertTrue(self.snap()["digest_ok"])

    def test_digest_mismatch_blocks_resets_until_a_match(self):
        self.feed(key() + [tri(1)], epoch=0, key_=True)
        wrong = (vs.digest_crc([(1, self.DH1, self.SH1)], *NEUTRAL) + 1) & 0xFF
        self.feed([vs.Digest(1, wrong)], epoch=0)
        self.assertFalse(self.snap()["digest_ok"])
        self.feed([vs.Confirm(1, (self.SH1 & 3,))], epoch=0)
        self.assertEqual(self.shapes()[1]["age_ms"], 1300)
        self.feed([vs.Confirm(1, (self.SH1 & 3,)), digest([(1, self.DH1, self.SH1)])], epoch=0)
        self.assertEqual(self.shapes()[1]["age_ms"], 300)
        self.assertEqual(self.st.stats["digest_mismatch"], 1)

    def test_digest_covers_the_render_state(self):
        self.feed(key() + [tri(1)], epoch=0, key_=True)
        self.feed([vs.Gshift(1, 3, -2), vs.Gzoom(140), vs.Gain(10, 16, 20),
                   digest([(1, self.DH1, self.SH1)])], epoch=0)
        self.assertFalse(self.snap()["digest_ok"])                   # a stale render state mismatches
        self.feed([digest([(1, self.DH1, self.SH1)], [(0, 0), (3, -2), (0, 0), (0, 0)], 140, (10, 16, 20))],
                  epoch=0)
        self.assertTrue(self.snap()["digest_ok"])

    def test_three_consecutive_mismatches_resync_until_the_next_epoch_start(self):
        self.feed(key() + [tri(1)], epoch=0, key_=True)
        wrong = (vs.digest_crc([(1, self.DH1, self.SH1)], *NEUTRAL) + 1) & 0xFF
        self.feed([vs.Digest(1, wrong)], epoch=0)
        self.feed([digest([(1, self.DH1, self.SH1)])], epoch=0)      # a match resets the run
        self.feed([vs.Digest(1, wrong)], epoch=0)
        self.feed([vs.Digest(1, wrong)], epoch=0)
        self.assertFalse(self.st.stats["resync"])
        self.feed([vs.Digest(1, wrong)], epoch=0)
        self.assertTrue(self.st.stats["resync"])
        self.assertTrue(self.snap()["resync"])
        self.assertEqual(self.shapes()[1]["age_ms"], 2800)           # ages are not reset
        self.feed([vs.Confirm(1, (self.SH1 & 3,)), digest([(1, self.DH1, self.SH1)])], epoch=0)
        self.assertEqual(self.shapes()[1]["age_ms"], 3300)           # nor by a matching CONFIRM in resync
        self.feed(key() + [tri(1), digest([(1, self.DH1, self.SH1)])], epoch=1, key_=True)
        self.assertFalse(self.st.stats["resync"])
        self.assertEqual(self.shapes()[1]["age_ms"], 300)


# ---------------------------------------------------------------- orphans → RESYNC (§3.4 rule 4, §4.3)

class OrphanResyncTests(StoreCase):
    def test_orphans_trigger_resync_with_no_uplink_and_the_next_epoch_start_clears_it(self):
        self.feed(key() + [tri(1)], epoch=0, key_=True)
        t_def = self.rx - 200
        for _ in range(3):                                           # 12 orphans in 12 items
            self.feed([vs.Upd(9, 1, 1), vs.Ucol(9, FLAT), vs.Del(9), vs.Hole(9, 0, False, 1, 1, 1)], epoch=0)
        self.assertEqual(self.st.stats["orphans"], 12)
        self.assertTrue(self.st.stats["resync"])
        self.assertTrue(self.snap()["resync"])
        self.assertEqual(self.shapes()[1]["age_ms"], self.rx + 100 - t_def)   # ages keep counting
        self.assertFalse(any(n.startswith(("send", "publish", "request", "uplink")) for n in dir(self.st)))
        self.feed([vs.Confirm(1, (shash(tri(1)) & 3,))], epoch=0)   # no age reset while in resync
        self.assertEqual(self.shapes()[1]["age_ms"], self.rx + 100 - t_def)
        self.feed(key() + [tri(1), digest([(1, vs.define_hash(tri(1)), shash(tri(1)))])], epoch=1, key_=True)
        self.assertFalse(self.st.stats["resync"])
        self.assertFalse(self.snap()["resync"])
        self.assertEqual(self.shapes()[1]["age_ms"], 300)

    def test_the_repeat_once_anchor_of_a_lost_epoch_start_also_clears_it(self):
        self.feed(key() + [tri(1)], epoch=0, key_=True)
        for _ in range(3):
            self.feed([vs.Upd(9, 1, 1), vs.Ucol(9, FLAT), vs.Del(9)], epoch=0)
        self.assertTrue(self.st.stats["resync"])
        self.feed([tri(2)], epoch=1)                                 # the K = 1 copy was lost ...
        self.assertTrue(self.st.stats["resync"])
        self.feed(key() + [tri(2)], epoch=1)                         # ... its repeat carries the anchor
        self.assertFalse(self.st.stats["resync"])

    def test_orphan_rate_at_or_below_20_percent_is_tolerated(self):
        self.feed(key() + [tri(i) for i in range(1, 10)], epoch=0, key_=True)
        self.feed([vs.Upd(i, 1, 1) for i in range(1, 10)] + [vs.Upd(20, 1, 1)], epoch=0)   # 1 of 10
        self.assertFalse(self.st.stats["resync"])
        self.assertEqual(self.st.stats["orphans"], 1)

    def test_orphan_window_is_ten_seconds(self):
        self.feed(key() + [tri(1)], epoch=0, key_=True)
        self.feed([vs.Upd(9, 1, 1)] * 4, epoch=0)                    # 4 orphans, below the item floor
        self.assertFalse(self.st.stats["resync"])
        self.feed([vs.Upd(1, 1, 1)] * 6, epoch=0, rx=self.rx + 11_000)   # the old orphans have aged out
        self.assertFalse(self.st.stats["resync"])
        self.feed([vs.Upd(9, 1, 1)] * 2 + [vs.Upd(1, 1, 1)] * 6, epoch=0)   # 2 of 14 in the window
        self.assertFalse(self.st.stats["resync"])
        self.feed([vs.Upd(9, 1, 1)] * 2, epoch=0)                    # 4 of 16 = 25 %
        self.assertTrue(self.st.stats["resync"])


# ---------------------------------------------------------------- TTL (§4.3)

class TtlTests(StoreCase):
    def test_a_shape_is_dropped_after_20_frames_without_verification(self):
        self.feed(key() + [tri(1), tri(2, y=20)], epoch=0, key_=True)
        tag2 = shash(tri(2)) & 3
        for _ in range(TTL_FRAMES - 1):
            self.feed([vs.Confirm(2, (tag2,))], epoch=0)             # 2 stays verified, 1 does not
            self.assertEqual(set(self.shapes()), {1, 2})
        self.feed([vs.Confirm(2, (tag2,))], epoch=0)
        self.assertEqual(set(self.shapes()), {2})
        self.assertEqual(self.st.stats["ttl_dropped"], 1)

    def test_upd_and_a_same_hash_define_restart_the_clock(self):
        self.feed(key() + [tri(1)], epoch=0, key_=True)
        for _ in range(TTL_FRAMES - 1):
            self.feed([], epoch=0)
        self.feed([vs.Upd(1, 1, 0)], epoch=0)
        for _ in range(TTL_FRAMES - 1):
            self.feed([], epoch=0)
        self.assertEqual(set(self.shapes()), {1})
        self.feed([tri(1)], epoch=0)
        for _ in range(TTL_FRAMES - 1):
            self.feed([], epoch=0)
        self.assertEqual(set(self.shapes()), {1})
        self.feed([], epoch=0)
        self.assertEqual(set(self.shapes()), set())


# ---------------------------------------------------------------- HOLE slots (§3.3)

class HoleTests(StoreCase):
    def test_slot_replace_and_delete_through_the_parent_state_hash(self):
        self.feed(key() + [tri(1)], epoch=0, key_=True)
        dh = vs.define_hash(tri(1))
        h = vs.Hole(1, 0, False, 30, 47, 1)
        self.feed([h], epoch=0)
        holes = self.holes()
        self.assertEqual([(x["parent"], x["slot"], x["cx"], x["cy"], x["r"]) for x in holes], [(1, 0, 120, 188, 8)])
        sh = shash(tri(1), holes=[h])
        self.feed([vs.Confirm(1, (sh & 3,)), digest([(1, dh, sh)])], epoch=0)
        self.assertTrue(self.snap()["digest_ok"])
        self.assertEqual(self.shapes()[1]["age_ms"], 300)
        h2 = vs.Hole(1, 0, False, 32, 40, 2)                        # replaces slot 0
        h3 = vs.Hole(1, 3, False, 20, 20, 0)
        self.feed([h2, h3], epoch=0)
        self.assertEqual([(x["slot"], x["cx"], x["cy"], x["r"]) for x in self.holes()],
                         [(0, 128, 160, 12), (3, 80, 80, 4)])
        sh = shash(tri(1), holes=[h2, None, None, h3])
        self.feed([digest([(1, dh, sh)])], epoch=0)
        self.assertTrue(self.snap()["digest_ok"])
        self.feed([vs.Hole(1, 0, True, 0, 0, 0)], epoch=0)           # delete slot 0
        self.assertEqual([x["slot"] for x in self.holes()], [3])
        self.feed([digest([(1, dh, shash(tri(1), holes=[None, None, None, h3]))])], epoch=0)
        self.assertTrue(self.snap()["digest_ok"])

    def test_holes_follow_the_parent_and_die_with_it(self):
        self.feed(key() + [tri(1), vs.Hole(1, 0, False, 30, 47, 1)], epoch=0, key_=True)
        self.feed([vs.Upd(1, 1, 0), vs.Gshift(1, 2, 0)], epoch=0)
        self.assertEqual(self.holes()[0]["cx"], 128.0)
        self.feed([vs.Del(1)], epoch=0)
        self.assertEqual(self.holes(), [])
        self.feed([vs.Hole(1, 0, False, 30, 47, 1), vs.Hole(32, 0, False, 30, 47, 1)], epoch=0)
        self.assertEqual(self.st.stats["orphans"], 2)


# ---------------------------------------------------------------- snapshot goldens (§7.2, §3.3, §4.4)

class SnapshotGoldenTests(StoreCase):
    def test_shape_keys_and_json(self):
        self.feed(key() + [tri(1), vs.Status(1, 30, 0, 64, 3, 2, False, True), vs.CalRev(0x1234, 0x5678),
                           vs.Anom(1, 22, 25, 3, 2, 0xC86)], epoch=0, key_=True, rx=10_000, airtime=60.0, age=3)
        s = self.snap(10_100)
        json.dumps(s)
        for k in ("v", "epoch", "badge", "lab", "level", "anchor_age_ms", "digest_ok", "resync", "corr_detected",
                  "corr_shown", "min_object", "bits", "horizon", "layers", "masked", "warp", "stats", "status"):
            self.assertIn(k, s)
        self.assertEqual((s["v"], s["badge"], s["lab"], s["level"], s["masked"]), (1, 7, False, 0, []))
        self.assertEqual(s["warp"], {"applied": False, "source": None, "since_capture_ms": 0})
        self.assertEqual(s["anchor_age_ms"], 10_100 - (10_000 - 60 - 600))    # rx − airtime − 200·AAAA
        self.assertEqual((s["corr_detected"], s["corr_shown"]), (2, 1))
        self.assertEqual([layer["id"] for layer in s["layers"]], ["L1", "L2", "L3", "L4"])
        self.assertEqual(s["status"]["arm_deg"], 0)
        self.assertEqual(s["cal_rev"], {"cal": 0x1234, "mask": 0x5678})
        self.assertEqual(s["stats"]["frames_applied"], 1)
        bits = s["bits"]["per_layer"]
        self.assertEqual(sum(bits.values()), s["bits"]["last_frame"])
        self.assertEqual(bits["hdr"], 13)
        self.assertEqual(bits["ctrl"], vs.record_bits(vs.LayerClear(0)) + 30 + 42)

    def test_pixel_conversions(self):
        recs = key() + [tri(1), vs.Poly(2, 1, ((10, 5), (20, 5), (20, 15)), FLAT), vs.Poly(3, 0, ((10, 5), (20, 5), (20, 15)), FLAT),
                        vs.Tree(32, 5, 5, 2, 3, PAL2, trunk_h=2), vs.Edge(56, 2, ((0, 20), (2, 26))),
                        vs.Anom(1, 22, 25, 3, 2, 0xC86), vs.Blob(90, 30, 40, 1, 2, 3, PAL2),
                        vs.Hole(1, 0, False, 30, 47, 1)]
        self.feed(recs, epoch=0, key_=True)
        s = self.shapes()
        self.assertEqual(s[3]["pts"][:2], [80.0, 40.0])             # 8 px grid: cell (10, 5) → (80, 40)
        self.assertEqual(s[2]["pts"], [40.0, 20.0, 80.0, 20.0, 80.0, 60.0])   # 4 px grid
        self.assertEqual((s[32]["cx"], s[32]["cy"], s[32]["rx"], s[32]["ry"]), (40.0, 40.0, 12.0, 16.0))
        self.assertEqual(s[32]["trunk"], {"w": 4, "h": 24, "c0": "#222233"})
        self.assertEqual((s[56]["cls"], s[56]["pts"]), ("fence", [0.0, 80.0, 8.0, 104.0]))
        self.assertEqual((s[81]["k"], s[81]["corridor"], s[81]["box"]), ("anom", True, [176.0, 200.0, 32.0, 24.0]))
        self.assertEqual(s[81]["fill"], {"c0": "#cc8866"})
        self.assertEqual((s[90]["cx"], s[90]["cy"], s[90]["rx"], s[90]["ry"], s[90]["rot_deg"]),
                         (120.0, 160.0, 8.0, 12.0, 67.5))
        hole = self.holes()[0]
        self.assertEqual((hole["cx"], hole["cy"], hole["r"]), (120.0, 188.0, 8.0))

    def test_horizon_goldens(self):
        self.feed(key() + [tri(1)], epoch=0, key_=True)
        h = self.snap()["horizon"]
        self.assertEqual((h["mode"], h["pts"]), ("abs", [[0.0, 0.0], [192.0, 0.0], [384.0, 0.0]]))
        self.assertEqual(h["sky"], ["#56abde", "#76cbfe"])         # base −16 at the top, +16 at the horizon
        self.assertEqual(h["ground"], ["#6f4d2b", "#7f5d3b"])      # base −8 at the horizon, +8 at the bottom
        self.feed([vs.HznAbs(96, 2, 1, SKY, GND)], epoch=0)        # y 64 px, 1°, 4 px sag
        pts = self.snap()["horizon"]["pts"]
        t = math.tan(math.radians(1))
        self.assertAlmostEqual(pts[0][1], 64 - 192 * t + 4, places=2)
        self.assertEqual(pts[1], [192.0, 64.0])
        self.assertAlmostEqual(pts[2][1], 64 + 192 * t + 4, places=2)
        self.feed([vs.HznResid(2, -2)], epoch=0)                   # +4 px, −1°: flat again with the sag
        h = self.snap()["horizon"]
        self.assertEqual((h["mode"], h["pts"]), ("resid", [[0.0, 72.0], [192.0, 68.0], [384.0, 72.0]]))
        self.feed([vs.HznColours(vs.VFill(palette=1, dl=0), vs.VFill(palette=4, dl=0))], epoch=0)
        h = self.snap()["horizon"]
        self.assertEqual((h["sky"], h["ground"]), (["#bbbbcc", "#bbbbcc"], ["#ddbb66", "#ddbb66"]))
        self.assertEqual(h["pts"][1], [192.0, 68.0])
        self.feed([NOHZ], epoch=0)
        h = self.snap()["horizon"]
        self.assertEqual((h["mode"], h["pts"][1]), ("none", [192.0, 128.0]))

    def test_skyline_singleton_and_fill_gradient(self):
        self.feed([vs.HznAbs(128, 0, 0, SKY, GND), vs.LayerClear(0), tri(1, fill=GRAD),
                   vs.Skyline(0, (1,) * 8, PAL2)], epoch=0, key_=True)
        h = self.snap()["horizon"]
        self.assertEqual(h["skyline"][:2], [[24.0, 124.0], [72.0, 124.0]])
        self.assertEqual(h["skyline_fill"], {"c0": "#225522"})
        self.feed([vs.Skyline(1, (2,) * 12, PAL2)], epoch=0)         # replaces the old one
        self.assertEqual(len(self.snap()["horizon"]["skyline"]), 12)
        self.assertEqual(self.snap()["horizon"]["skyline"][0], [16.0, 112.0])
        fill = self.shapes()[1]["fill"]
        self.assertEqual((fill["c0"], fill["c1"]), ("#568923", "#76a943"))
        self.assertEqual(fill["g"], [133.33, 80.0, 133.33, 160.0])   # through the centroid, spanning the extent
        self.feed([vs.Pal(0, 0xABC), vs.Ucol(1, vs.Fill(palette=8))], epoch=0)
        self.assertEqual(self.shapes()[1]["fill"], {"c0": "#aabbcc"})

    def test_gzoom_expands_the_ground_about_the_vanishing_point(self):
        self.feed(key() + [tri(1), vs.Tree(32, 5, 0, 1, 1, PAL2)], epoch=0, key_=True)
        self.feed([vs.Gzoom(132)], epoch=0)                          # u = 4 / 4096
        s = self.shapes()
        u = 4 / 4096
        exp = []
        for x, y in ((80, 80), (160, 80), (160, 160)):
            r = y / (1 - u * y)
            exp += [round(192 + (x - 192) * (r / y), 2), round(r, 2)]
        self.assertEqual(s[1]["pts"], exp)
        self.assertEqual(s[32]["cx"], 40.0)                          # far group: untouched
        self.assertEqual(s[1]["badge"], BADGE_VECTOR)
        self.feed([vs.Gzoom(255)], epoch=0)                          # r'/r > 2: PREDICTED
        self.assertEqual(self.shapes()[1]["badge"], BADGE_PREDICTED)
        self.feed([tri(2, y=20)], epoch=0)                           # defined at the current zoom: no zoom
        self.assertEqual(self.shapes()[2]["pts"][:2], [80.0, 160.0])
        self.feed([vs.Gzoom(128)], epoch=0)                          # reversing contracts
        self.assertLess(self.shapes()[2]["pts"][1], 160.0)

    def test_anom_refined_by_a_later_poly_on_the_same_id(self):
        self.feed(key() + [vs.Anom(1, 22, 25, 3, 2, 0xC86)], epoch=0, key_=True)
        self.feed([vs.Poly(81, 1, ((44, 50), (52, 50), (52, 56)), FLAT)], epoch=0)
        s = self.shapes()
        self.assertEqual(set(s), {81})
        self.assertEqual((s[81]["k"], s[81]["corridor"]), ("poly", True))
        self.assertEqual(self.snap()["corr_shown"], 1)

    def test_status_and_cal_rev_keep_the_latest(self):
        self.feed(key() + [vs.Status(0, 0, 0, 0, 0, 0, False, False), vs.CalRev(1, 1)], epoch=0, key_=True)
        rx = self.rx
        self.feed([vs.Status(2, 60, 2, 70, 3, 5, True, True), vs.CalRev(2, 2)], epoch=0, rx=rx + 1000)
        self.feed([vs.Status(1, 0, 1, 64, 1, 1, False, False), vs.CalRev(3, 3)], epoch=0, rx=rx + 500)
        s = self.snap()
        self.assertEqual((s["status"]["arm_deg"], s["status"]["bkt_deg"], s["status"]["corr_n"]), (30, 9.0, 5))
        self.assertEqual(s["cal_rev"], {"cal": 2, "mask": 2})

    def test_l1_paints_area_descending_then_holes_then_anomalies(self):
        self.feed(key() + [tri(1), vs.Poly(2, 0, ((0, 5), (40, 5), (40, 30), (0, 30)), FLAT),
                           vs.Anom(0, 1, 1, 1, 1, 0xFFF), vs.Hole(1, 0, False, 30, 47, 1)], epoch=0, key_=True)
        kinds = [(s["id"], s["k"]) for s in self.snap()["layers"][0]["shapes"]]
        self.assertEqual(kinds, [(2, "poly"), (1, "poly"), (1, "hole"), (80, "anom")])


# ---------------------------------------------------------------- robustness (§3.4)

class RobustnessTests(unittest.TestCase):
    def test_ingest_never_raises_on_garbage(self):
        rng = random.Random(5)
        st = VectorSceneStore()
        valid = body(True, 0, key() + [tri(1)])
        inputs = [b"", b"\x00", b"\xff" * 10, None, "str", 123, 4.5, [1, 2], bytearray(valid),
                  memoryview(valid), valid[:-1], valid + b"\x01", b"\x40\x00", b"\xfe" * 20]
        inputs += [bytes(rng.randrange(256) for _ in range(rng.randrange(1, 60))) for _ in range(300)]
        for bad in inputs:
            for fk in (0, 1, 7):
                self.assertIsInstance(st.ingest(bad, fk, 10_000), IngestResult)
                st.snapshot(10_000)
        stats = st.stats
        self.assertEqual(stats["frames_rx"], len(inputs) * 3)
        self.assertEqual(stats["frames_bad"] + stats["frames_applied"], stats["frames_rx"])
        self.assertEqual(sum(stats["bad_reasons"].values()), stats["frames_bad"])

    def test_bad_version_and_key_mismatch_are_counted_by_reason(self):
        st = VectorSceneStore()
        good = body(True, 0, key() + [tri(1)])
        self.assertEqual(st.ingest(bytes([good[0] | 0x40]) + good[1:], 1, 10_000).reason, "bad_version")
        self.assertEqual(st.ingest(good, 0, 10_000).reason, "key_mismatch")
        self.assertEqual(st.stats["bad_version"], 1)
        self.assertEqual(st.stats["bad_reasons"], {"bad_version": 1, "key_mismatch": 1})
        self.assertIsNone(st.snapshot(10_000))
        st.ingest(good, 1, 10_000)
        st.reset()
        self.assertIsNone(st.snapshot(10_000))
        self.assertEqual(st.stats["frames_rx"], 0)


# ---------------------------------------------------------------- loss (§4.3, §8.6): a simulated tractor

class Tractor:
    """The tractor's mirror, driven with the codec directly: defines, absolute
    UPD/UCOL/INSERT/HOLE state, cumulative GSHIFT/GZOOM/GAIN, and the CONFIRM
    tags and DIGEST of §3.3 computed from that mirror."""

    def __init__(self, seed: int):
        self.rng = random.Random(seed)
        self.shapes: dict[int, dict] = {}
        self.gs = [(0, 0)] * 4
        self.zoom = 128
        self.gain = (16, 16, 16)
        self.carousel = 0

    def new_define(self, id_: int):
        rng = self.rng
        if id_ in vs.ID_EDGE:
            x, y = rng.randrange(0, 80), rng.randrange(20, 60)
            return vs.Edge(id_, rng.randrange(5), ((x, y), (x + rng.randrange(1, 8), y + rng.randrange(-4, 4))))
        fill = rng.choice([FLAT, GRAD, PAL2, vs.Fill(rgb444=rng.randrange(4096), grad=(rng.randrange(8), 1))])
        if id_ in vs.ID_PLANT:
            return vs.Tree(id_, rng.randrange(2, 45), rng.randrange(0, 12), rng.randrange(8), rng.randrange(8), fill)
        x, y = rng.randrange(2, 30), rng.randrange(8, 24)
        return vs.Poly(id_, 0, ((x, y), (x + rng.randrange(3, 10), y + rng.randrange(-3, 3)),
                                (x + rng.randrange(0, 8), y + rng.randrange(3, 8))), fill)

    def define(self, id_: int, rec) -> None:
        holes = self.shapes[id_]["holes"] if id_ in self.shapes else [None] * 4
        fill = vs.Fill(rgb444=0) if isinstance(rec, vs.Edge) else rec.fill
        self.shapes[id_] = {"def": rec, "off": (0, 0), "fill": fill, "inserts": {}, "holes": holes}

    def state_hash(self, s: dict) -> int:
        f = s["fill"]
        rgb = vs.STATIC_PALETTE[f.palette] if f.palette is not None else f.rgb444
        return vs.state_hash(s["off"][0], s["off"][1], rgb, list(s["inserts"].values()), s["holes"])

    def digest(self) -> vs.Digest:
        rows = [(i, vs.define_hash(s["def"]), self.state_hash(s)) for i, s in self.shapes.items()]
        return vs.Digest(len(rows), vs.digest_crc(rows, self.gs, self.zoom, self.gain))

    def confirms(self) -> list:
        ids, out = sorted(self.shapes), []
        while ids:
            base = ids[0]
            run = [i for i in ids if i < base + 16]
            tags = tuple(self.state_hash(self.shapes[i]) & 3 if i in run else None
                         for i in range(base, run[-1] + 1))
            out.append(vs.Confirm(base, tags))
            ids = ids[len(run):]
        return out

    def resend(self, id_: int) -> list:
        """A re-verified define plus its current UPD/UCOL/INSERT/HOLE state (§4.3)."""
        s = self.shapes[id_]
        recs = [s["def"]]
        if s["off"] != (0, 0):
            recs.append(vs.Upd(id_, *s["off"]))
        if not isinstance(s["def"], vs.Edge) and s["fill"] != s["def"].fill:
            recs.append(vs.Ucol(id_, s["fill"]))
        recs += list(s["inserts"].values())
        recs += [h for h in s["holes"] if h is not None]
        return recs

    def render_state(self) -> list:
        return [vs.Gshift(g, *self.gs[g]) for g in (0, 1, 2)] + [vs.Gzoom(self.zoom), vs.Gain(*self.gain)]

    def change_one(self) -> list:
        """One random change to the mirror; returns the records that carry it."""
        rng = self.rng
        id_ = rng.choice(sorted(self.shapes))
        s, kind = self.shapes[id_], rng.random()
        poly = isinstance(s["def"], vs.Poly)
        if kind < 0.35 and not isinstance(s["def"], vs.Edge):
            s["off"] = (rng.randrange(-8, 8), rng.randrange(-8, 8))
            return [vs.Upd(id_, *s["off"])]
        if kind < 0.5 and poly:
            s["fill"] = vs.Fill(rgb444=rng.randrange(4096))
            return [vs.Ucol(id_, s["fill"])]
        if kind < 0.7 and poly:
            ins = vs.Insert(id_, rng.randrange(3), 0, rng.randrange(-2, 3), rng.randrange(-2, 3))
            s["inserts"][(ins.edge, ins.k)] = ins
            return [ins]
        if kind < 0.85 and poly:
            h = vs.Hole(id_, rng.randrange(2), False, rng.randrange(0, 96), rng.randrange(0, 64), rng.randrange(8))
            s["holes"][h.slot] = h
            return [h]
        self.define(id_, self.new_define(id_))
        return [self.shapes[id_]["def"]]

    def frame(self, recs: list, key_: bool, epoch: int, kappa: float) -> tuple:
        """Size the frame to F like the encoder (§4.1): the carousel takes κ of
        what the mandatory records, CONFIRM and DIGEST leave of the 1,883
        record bits (§4.2 item 7; κ = 25 / 50 / 75 % at V0 / V1 / V2, §4.5.4),
        round-robin over the live shapes, re-verified (§4.3)."""
        tail = self.confirms() + [self.digest()]
        budget = int(kappa * (1883 - 16 - sum(vs.record_bits(r) for r in recs + tail)))
        live = sorted(self.shapes)
        for _ in range(len(live)):
            group = self.resend(live[self.carousel % len(live)])
            cost = sum(vs.record_bits(r) for r in group)
            if cost > budget:
                break
            recs, budget = recs + group, budget - cost
            self.carousel += 1
        return key_, epoch, recs + tail


def scenario(seed: int, n_frames: int = 60, with_del: bool = True, kappa: float = 0.25):
    """Frames (key, epoch, records) from a tractor that repeats its epoch start
    once, re-anchors every 8 frames, sends the render state, the carousel and
    CONFIRM + DIGEST in every frame, and starts a second epoch (the safety
    refresh, LAYER_CLEAR 3) at frame 44 followed by a quiet tail."""
    tr = Tractor(seed)
    rng = tr.rng
    ids = list(range(1, 9)) + [32, 33, 56, 57]
    for i in ids:
        tr.define(i, tr.new_define(i))
    frames = [tr.frame([ABS, vs.LayerClear(0)] + tr.render_state(), True, 0, 1.0),
              tr.frame([ABS, vs.LayerClear(0)] + tr.render_state(), False, 0, 1.0)]     # repeat-once
    repeat: list = []
    epoch = 0
    for f in range(2, n_frames):
        if f == 44:
            epoch = 1
            frames.append(tr.frame([ABS, vs.LayerClear(3)] + tr.render_state(), True, 1, kappa))
            frames.append(tr.frame([ABS, vs.LayerClear(3)] + tr.render_state(), False, 1, kappa))
            continue
        if f == 45:
            continue
        recs = [ABS] if f % 8 == 0 else []
        if f < 44:
            tr.gs[1] = (max(-100, min(100, tr.gs[1][0] + rng.randrange(-3, 4))), tr.gs[1][1])
            tr.gs[2] = (tr.gs[2][0], max(-50, min(50, tr.gs[2][1] + rng.randrange(-1, 2))))
            tr.zoom = max(100, min(156, tr.zoom + rng.randrange(-2, 3)))
            tr.gain = tuple(max(8, min(24, g + rng.randrange(-1, 2))) for g in tr.gain)
            changed = []
            for _ in range(2):
                out = tr.change_one()
                recs += out
                if isinstance(out[0], (vs.Poly, vs.Tree, vs.Edge)):
                    changed.append(out[0].id)
            if with_del and f == 3:
                recs.append(vs.Del(ids[-1]))
                del tr.shapes[ids[-1]]
            if f == 20:
                tr.define(9, tr.new_define(9))
                recs.append(tr.shapes[9]["def"])
                changed.append(9)
            recs += [r for i in repeat if i in tr.shapes for r in tr.resend(i)]   # repeat-once, re-verified
            repeat = changed
        frames.append(tr.frame(recs + tr.render_state(), False, epoch, kappa))
    return tr, frames


def feed_scenario(frames, keep, st: VectorSceneStore | None = None, start: int = 0,
                  stop: int | None = None, digest_log: list | None = None) -> VectorSceneStore:
    """Frame f is captured at 20_000 + 500·f and received 250 ms later (AAAA = 1,
    50 ms of airtime). ``digest_log`` collects ``digest_ok`` after each applied frame."""
    st = st if st is not None else VectorSceneStore()
    for f in range(start, len(frames) if stop is None else stop):
        if keep[f]:
            k, epoch, recs = frames[f]
            cap = 20_000 + 500 * f
            st.ingest(body(k, epoch, recs), 1 if k else 0, cap + 250, 50.0)
            if digest_log is not None:
                digest_log.append(st.snapshot(cap + 300)["digest_ok"])
    return st


def keep_mask(n: int, drop) -> list:
    keep = [not drop(f) for f in range(n)]
    for a in (0, 44):                    # the premise: one copy of each epoch start lands
        if not (keep[a] or keep[a + 1]):
            keep[a + 1] = True
    return keep


class LossTests(unittest.TestCase):
    NOW = 20_000 + 500 * 60 + 300

    def assert_converged(self, tr: Tractor, st: VectorSceneStore, label):
        snap = st.snapshot(self.NOW)
        self.assertFalse(snap["resync"], label)
        self.assertTrue(snap["digest_ok"], label)
        self.assertFalse(snap["handover"], label)
        self.assertEqual(set(StoreCase.shapes_of(snap)), set(tr.shapes), label)
        self.assertEqual(snap["epoch"], 1, label)

    def test_lossless_run_tracks_the_mirror_exactly(self):
        for seed in (1, 2, 3):
            tr, frames = scenario(seed)
            st = feed_scenario(frames, [True] * len(frames))
            self.assert_converged(tr, st, seed)
            self.assertEqual(st.stats["digest_mismatch"], 0, seed)
            self.assertEqual(st.stats["orphans"], 0, seed)
            self.assertEqual(st.stats["frames_applied"], len(frames))

    def test_lossy_is_a_subset_of_lossless(self):
        """Applying a random subset of the frames never yields a shape that the
        lossless run does not have at a newer (or equal) capture time."""
        for seed in (11, 12, 13, 14):
            tr, frames = scenario(seed, with_del=False)
            full = feed_scenario(frames, [True] * len(frames))
            rng = random.Random(seed)
            for trial in range(4):
                p = rng.choice((0.12, 0.3, 0.5))
                keep = [rng.random() >= p for _ in frames]
                lossy = feed_scenario(frames, keep)
                a, b = StoreCase.shapes_of(full.snapshot(self.NOW)), StoreCase.shapes_of(lossy.snapshot(self.NOW))
                for id_, sh in b.items():
                    self.assertIn(id_, a, (seed, trial, id_))
                    self.assertLessEqual(a[id_]["age_ms"], sh["age_ms"], (seed, trial, id_))
                self.assertLessEqual(full.snapshot(self.NOW)["anchor_age_ms"], lossy.snapshot(self.NOW)["anchor_age_ms"])

    def run_lossy(self, seed: int, kappa: float, drop, label) -> VectorSceneStore:
        """Drive a lossy run. The state must converge on the mirror by the end
        of the quiet tail, and the RESYNC chip after the last epoch start must
        follow the 3-consecutive-DIGEST-mismatch rule exactly (§4.3)."""
        tr, frames = scenario(seed, kappa=kappa)
        keep = keep_mask(len(frames), drop)
        st = feed_scenario(frames, keep, stop=46)
        self.assertFalse(st.stats["resync"], label)             # the epoch start (or its repeat) ends a resync
        self.assertEqual(st.snapshot(self.NOW)["epoch"], 1, label)
        log: list = []
        feed_scenario(frames, keep, st=st, start=46, digest_log=log)
        snap = st.snapshot(self.NOW)
        self.assertTrue(snap["digest_ok"], label)                # converged on the mirror
        self.assertFalse(snap["handover"], label)
        self.assertEqual(set(StoreCase.shapes_of(snap)), set(tr.shapes), label)
        run = best = 0
        for ok in log:
            run = run + 1 if ok is False else 0
            best = max(best, run)
        self.assertEqual(snap["resync"], best >= 3, (label, log))
        return st

    def test_convergence_at_12_and_30_percent_iid_loss(self):
        """κ follows the ladder (§4.5.4): 50 % at V1 for 12 % loss, 75 % at V2 for 30 %."""
        repaired = 0
        for p, kappa in ((0.12, 0.5), (0.30, 0.75)):
            for seed in (21, 22, 23, 24, 25, 26):
                rng = random.Random(seed * 7 + int(p * 100))
                st = self.run_lossy(seed, kappa, lambda f: rng.random() < p, (p, seed))
                self.assertGreater(st.stats["frames_applied"], 60 * (1 - p) * 0.7)
                repaired += st.stats["digest_mismatch"]
        self.assertGreater(repaired, 0)                          # loss really did hit state that was repaired

    def test_convergence_under_burst_loss(self):
        for seed in (31, 32, 33):
            st = self.run_lossy(seed, 0.5, lambda f: f % 20 in (3, 4, 5, 6, 7), seed)   # 5 lost in every 20
            self.assertEqual(st.stats["epoch_behind"], 0)
            self.assertEqual(st.stats["ttl_dropped"], 1)         # the DEL at frame 3 is lost: TTL drops it

    def test_lost_epoch_start_repairs_from_the_repeat_and_ages_stay_honest(self):
        tr, frames = scenario(41)
        keep = [True] * len(frames)
        keep[0] = keep[44] = False                                  # only the repeat-once copies land
        st = feed_scenario(frames, keep)
        self.assert_converged(tr, st, "repeat")
        snap = st.snapshot(self.NOW)
        for sh in StoreCase.shapes_of(snap).values():
            self.assertEqual(sh["age_ms"], self.NOW - (20_000 + 500 * 59), sh["id"])   # CONFIRMed every frame
        self.assertEqual(snap["anchor_age_ms"], self.NOW - (20_000 + 500 * 56))        # last ABS at frame 56
        self.assertEqual(st.stats["epochs"], 2)


if __name__ == "__main__":
    unittest.main()
