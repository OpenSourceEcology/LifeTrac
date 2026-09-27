"""Encoder ↔ store synchronisation under the §3.4/§4.3 rules the RS-13.1 bench
found broken on a loss-free path (``bench-evidence/RS_13_vector_scene_2026-09-26``,
anomalies A1–A3): the mirror's shape TTL, the same-hash define as a repeat,
re-sends that carry the current UPD/UCOL, the safety refresh re-stating the
kept masses, the union ΔD of a same-id redefine, and id pressure that never
starts an epoch. Every test drives the real ``VectorEncoder`` into the real
``VectorSceneStore`` through the TileDeltaFrame container, frame by frame,
and reads the store's own verdict (``digest_ok``, orphans, TTL drops, resync).
Needs numpy + cv2 like the other encoder tests; skipped without them.
"""
from __future__ import annotations

import os
import sys
import unittest

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_BS_DIR = os.path.dirname(_THIS_DIR)
_X8_DIR = os.path.normpath(os.path.join(_THIS_DIR, "..", "..", "firmware", "tractor_x8"))
for _p in (_BS_DIR, _X8_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    import numpy as np
    import cv2  # noqa: F401
    _HAVE_CV = True
except ImportError:  # pragma: no cover
    np = None
    _HAVE_CV = False

from image_pipeline.frame_format import CODEC_VECTOR, parse_tile_delta_frame  # noqa: E402
from image_pipeline.vector_scene import codec as vs  # noqa: E402
from image_pipeline.vector_scene_store import TTL_FRAMES, VectorSceneStore  # noqa: E402

if _HAVE_CV:
    from x8_image_pipeline import encode_vector as ev  # noqa: E402

W, H = 96, 64                       # working resolution: the encoder takes it as is
CW, CH = 384, 256
SKY = (120, 170, 235)
GROUND = (120, 96, 60)
GREEN = (40, 140, 40)
RED = (200, 60, 60)
BUDGET = 203
PERIOD_MS = 500


def scene(y0=112.0, blobs=(), rects=(), noise=0.0, seed=0):
    """Sky above the canvas-space horizon row ``y0``, ground below; ``blobs``
    are (cx, cy, r, colour) trees and ``rects`` (x0, y0, x1, y1, colour)
    masses, in working pixels. ``noise`` adds ±noise per pixel *at canvas
    size* (384×256), the way sensor noise reaches the encoder, which averages
    each 4×4 block down to the working image; noise added per working pixel
    would fragment the scene into MAX_REGIONS specks and trip the 40 %
    relabel trigger on every frame."""
    img = np.zeros((H, W, 3), np.uint8)
    line = y0 / (CW / W)
    ys = np.arange(H)[:, None] + 0.5
    above = np.broadcast_to(ys < line, (H, W))
    img[above] = SKY
    img[~above] = GROUND
    yy, xx = np.mgrid[0:H, 0:W]
    for x0, y0_, x1, y1, colour in rects:
        img[y0_:y1 + 1, x0:x1 + 1] = colour
    for cx, cy, r, colour in blobs:
        img[((xx - cx) ** 2 + (yy - cy) ** 2) <= r * r] = colour
    if noise:
        import cv2
        big = cv2.resize(img, (CW, CH), interpolation=cv2.INTER_NEAREST)
        rng = np.random.default_rng(seed)
        img = np.clip(big.astype(np.int16) + rng.integers(-noise, noise + 1, big.shape), 0, 255).astype(np.uint8)
    return img


def records_of(frame, kind):
    return [r for r in frame.records if isinstance(r, kind)]


def confirmed_ids(frame) -> set:
    return {c.base_id + i for c in records_of(frame, vs.Confirm) for i, t in enumerate(c.tags) if t is not None}


class Link:
    """One encoder feeding one store, 500 ms per frame, with per-frame loss."""

    def __init__(self, clock=None):
        self.enc = ev.VectorEncoder(clock=clock) if clock else ev.VectorEncoder()
        self.store = VectorSceneStore(CW, CH)
        self.rx = 10_000
        self.seq = 0
        self.frames: list = []

    def step(self, img, *, lose: bool = False, budget: int = BUDGET, **kw):
        self.seq += 1
        self.rx += PERIOD_MS
        payload = self.enc.frame(img, budget, seq=self.seq, **kw)
        frame = parse_tile_delta_frame(payload)
        assert frame.codec == CODEC_VECTOR
        decoded = vs.decode_frame(frame.vector_body, frame.frame_kind)
        if not lose:
            res = self.store.ingest(frame.vector_body, frame.frame_kind, self.rx, 0.0)
            assert res.applied, res
        self.frames.append(decoded)
        return decoded

    def snap(self) -> dict:
        return self.store.snapshot(self.rx + 100)

    def in_step(self) -> tuple:
        """(digest_ok, resync, orphans, ttl_dropped) — the store's own verdict."""
        st = self.store.stats
        return self.snap()["digest_ok"], st["resync"], st["orphans"], st["ttl_dropped"]

    def ttl_clocks(self) -> tuple:
        """{id: frames_since_verify} at both ends (the store's live set incl. carried cache)."""
        enc = {i: sh.frames_since_verify for i, sh in self.enc._shapes.items()}
        store = {sh.id: sh.frames_since_verify for sh in self.store._live()}
        return enc, store


@unittest.skipUnless(_HAVE_CV, "numpy + cv2 required for the encoder")
class SyncTests(unittest.TestCase):
    BLOBS = [(20, 44, 4, GREEN), (50, 40, 4, GREEN), (80, 48, 4, GREEN)]
    RECTS = [(60, 20, 90, 30, RED)]

    def setUp(self) -> None:
        # The segmenter's k-means++ init draws from OpenCV's process-global
        # RNG (vector_extract.Segmenter), so an unseeded run can quantise a
        # square two ways and shift a centroid by half a pixel between test
        # orders. Seed it per test; assertions on an UPD's exact value still
        # allow the two legitimate roundings.
        import cv2
        cv2.setRNGSeed(0)

    def test_static_noisy_scene_stays_in_step_and_never_churns(self):
        # The invariant, on camera-like noise: every frame agrees, nothing
        # expires, the epoch stands. (±12 at canvas size: above ±20 the 40 %
        # relabel trigger of §4.3 restarts the epoch now and then on a scene
        # this sparse, which is its own subject, not this one.) The A1
        # mechanisms themselves are pinned by the drift, silence and
        # starvation tests below.
        link = Link()
        n = 3 * TTL_FRAMES + 5
        for i in range(n):
            link.step(scene(blobs=self.BLOBS, rects=self.RECTS, noise=12, seed=i))
            if i >= 1:
                self.assertEqual(link.in_step(), (True, False, 0, 0), f"frame {i + 1}")
        self.assertEqual(link.enc.last_stats["ttl_dropped"], 0)
        self.assertEqual(link.enc.last_stats["epochs"], 1)              # no churn either
        self.assertGreaterEqual(records_of(link.frames[-1], vs.Digest)[0].n_live, 4)

    def test_an_unverifiable_shape_expires_on_both_ends_in_step(self):
        # A1: whatever silences a shape, the mirror must expire it on the
        # base's count: the DIGEST that follows the drop no longer names it.
        link = Link()
        img = scene(blobs=self.BLOBS)
        first = link.step(img)
        target = records_of(first, vs.Tree)[0].id
        keep = link.enc._keep_candidates

        def silent(s, region, upd, de, cands, verified):          # the dead zone, by force
            if s.id != target:
                keep(s, region, upd, de, cands, verified)
        link.enc._keep_candidates = silent
        n_live = records_of(first, vs.Digest)[0].n_live
        for i in range(2, TTL_FRAMES + 2):                          # frames 2 .. 21
            frame = link.step(img)
            self.assertEqual(link.in_step()[:2], (True, False), f"frame {i}")
            self.assertEqual(records_of(frame, vs.Digest)[0].n_live, n_live, f"frame {i}")
            self.assertNotIn(target, confirmed_ids(frame))
        self.assertEqual(link.store.stats["ttl_dropped"], 1)           # dropped after frame 21's DIGEST
        self.assertEqual(link.enc.last_stats["ttl_dropped"], 1)
        frame = link.step(img)                                        # frame 22: the region is new again
        self.assertEqual(link.in_step(), (True, False, 0, 1))
        fresh = [t for t in records_of(frame, vs.Tree)]
        self.assertEqual(len(fresh), 1, frame.records)
        self.assertNotEqual(fresh[0].id, target)                      # the freed id is cooling down
        self.assertEqual(records_of(frame, vs.Digest)[0].n_live, n_live)

    def test_shape_back_at_its_define_position_sends_upd_zero_not_a_redefine(self):
        # A2: the capture re-yields the define itself while the mirror holds
        # an offset. A same-hash define would keep the base's offset (§3.4
        # rule 5); the offset must return to 0 by an UPD, and the DIGEST agree.
        link = Link()
        base = [(20, 44, 4, GREEN), (50, 40, 4, GREEN)]
        for _ in range(3):
            first = link.step(scene(blobs=base))
        ids = {t.id for t in records_of(link.frames[0], vs.Tree)}
        moved = link.step(scene(blobs=[base[0], (53, 40, 4, GREEN)]))
        upds = records_of(moved, vs.Upd)
        self.assertEqual([(u.dx, u.dy) for u in upds], [(3, 0)])
        sid = upds[0].id
        self.assertIn(sid, ids)
        self.assertEqual(link.in_step(), (True, False, 0, 0))
        back = link.step(scene(blobs=base))
        self.assertEqual([(u.id, u.dx, u.dy) for u in records_of(back, vs.Upd)], [(sid, 0, 0)])
        self.assertEqual([t for t in records_of(back, vs.Tree) if t.id == sid], [])
        self.assertEqual(link.in_step(), (True, False, 0, 0))
        self.assertEqual(link.store._shapes[sid].off, (0, 0))
        for _ in range(3):
            link.step(scene(blobs=base))
            self.assertEqual(link.in_step(), (True, False, 0, 0))

    def test_same_hash_define_applied_to_the_mirror_is_a_repeat(self):
        # §3.4 rule 5 on the mirror itself: should any path re-apply the live
        # define, the offset and colour stay, like the base keeps them.
        link = Link()
        link.step(scene(blobs=self.BLOBS))
        s = next(iter(link.enc._shapes.values()))
        s.dx, s.dy = 2, -1
        before = link.enc._frame_no
        link.enc._frame_no += 1
        link.enc._apply_define(s.define, s.raster, s.area, None, 0)
        self.assertIs(link.enc._shapes[s.id], s)
        self.assertEqual((s.dx, s.dy), (2, -1))
        self.assertEqual(s.define_frame, before + 1)

    def test_resend_carries_the_current_upd_and_ucol_and_repairs_a_lost_frame(self):
        # A1b / §4.3: "the re-sent define plus its current UPD/UCOL is a real
        # confirmation". Lose the frame that moved one tree and recoloured
        # another: the carousel's next visit must bring the base back.
        link = Link()
        base = [(20, 44, 4, GREEN), (50, 40, 4, GREEN)]
        for _ in range(3):
            link.step(scene(blobs=base))
        ids = sorted(t.id for t in records_of(link.frames[0], vs.Tree))
        changed = [(20, 44, 4, (30, 100, 30)), (53, 40, 4, GREEN)]
        lost = link.step(scene(blobs=changed), lose=True)
        upd = records_of(lost, vs.Upd)
        ucol = records_of(lost, vs.Ucol)
        self.assertEqual(len(upd), 1, lost.records)
        self.assertEqual(len(ucol), 1, lost.records)
        healed_at = None
        for i in range(8):
            frame = link.step(scene(blobs=changed))
            ok = link.in_step()[0]
            if ok and healed_at is None:
                healed_at = i
                recs = list(frame.records)
                # The define comes first, then its UPD / UCOL (one candidate, in order).
                for t in records_of(frame, vs.Tree):
                    at = recs.index(t)
                    following = recs[at + 1:at + 3]
                    if t.id == upd[0].id:
                        self.assertIn(vs.Upd(t.id, upd[0].dx, upd[0].dy), following, recs)
                    if t.id == ucol[0].id:
                        self.assertIn(vs.Ucol(t.id, ucol[0].fill), following, recs)
                self.assertEqual({t.id for t in records_of(frame, vs.Tree)}, set(ids))
        self.assertIsNotNone(healed_at, "the carousel never repaired the lost UPD/UCOL")
        self.assertTrue(link.in_step()[0])
        self.assertEqual(link.store._shapes[upd[0].id].off, (upd[0].dx, upd[0].dy))
        self.assertEqual(link.store._shapes[ucol[0].id].fill, ucol[0].fill)

    def test_safety_refresh_restates_the_kept_masses_and_ends_a_resync(self):
        # A1c / §4.3: "a desynchronised base recovers within one safety
        # period with no uplink". A lost UPD puts the base in resync; the
        # 60 s refresh (range 2, masses kept) must re-state every kept shape
        # with its current UPD so the new epoch's DIGEST matches.
        t = [1000.0]
        link = Link(clock=lambda: t[0])
        base = [(20, 44, 4, GREEN), (50, 40, 4, GREEN)]
        rects = [(10, 20, 40, 30, RED)]
        for _ in range(3):
            link.step(scene(blobs=base, rects=rects))
            t[0] += 0.5
        moved = [(20, 44, 4, GREEN), (53, 40, 4, GREEN)]
        lost = link.step(scene(blobs=moved, rects=rects), lose=True)
        t[0] += 0.5
        upd = records_of(lost, vs.Upd)
        self.assertEqual(len(upd), 1)
        for _ in range(3):                                            # three mismatches: resync
            link.step(scene(blobs=moved, rects=rects))
            t[0] += 0.5
        self.assertEqual(link.in_step()[:2], (False, True))
        n_live = records_of(link.frames[-1], vs.Digest)[0].n_live
        t[0] += 61.0
        refresh = link.step(scene(blobs=moved, rects=rects))
        self.assertTrue(refresh.header.key)
        self.assertEqual(records_of(refresh, vs.LayerClear), [vs.LayerClear(2)])
        self.assertEqual(link.enc.last_stats["epoch_trigger"], "safety")
        self.assertFalse(link.store.stats["resync"])                  # the epoch start ended it
        resent: set = set()
        frame = refresh
        for i in range(5):
            recs = list(frame.records)
            for d in records_of(frame, vs.Tree) + records_of(frame, vs.Poly):
                resent.add(d.id)
                if d.id == upd[0].id:
                    self.assertEqual(recs[recs.index(d) + 1], vs.Upd(d.id, upd[0].dx, upd[0].dy))
            if link.in_step()[0]:
                break
            t[0] += 0.5
            frame = link.step(scene(blobs=moved, rects=rects))
        self.assertEqual(len(resent), n_live, "every kept shape owes a repeat after the refresh")
        self.assertEqual(link.in_step()[:2], (True, False))
        self.assertEqual(link.store._shapes[upd[0].id].off, (upd[0].dx, upd[0].dy))

    def test_jittering_outline_is_confirmed_when_a_redefine_is_not_worth_it(self):
        # A1: geometry fails (IoU < 0.7) but the alternative outline removes
        # no error, so the live shape is kept *and confirmed*, not left silent.
        link = Link()
        img = scene(rects=self.RECTS)
        first = link.step(img)
        pid = records_of(first, vs.Poly)[0].id
        verify = link.enc._verify_geometry
        record = link.enc._region_record

        def fails(s, r, rec_geo, iou, region_map, same_layer):
            return (False, None) if s.id == pid else verify(s, r, rec_geo, iou, region_map, same_layer)

        def nudged(id_, region):
            rec = record(id_, region)
            if id_ == pid and type(rec).__name__ == "Poly":           # the encoder's own codec class
                v = list(rec.vertices)
                v[0] = (v[0][0] + 1, v[0][1])                         # one vertex one cell off: no better
                rec = ev.vs.Poly(rec.id, rec.grid, tuple(v), rec.fill)
                region.raster = None                                  # score the outline's own raster
            return rec
        link.enc._verify_geometry = fails
        link.enc._region_record = nudged
        dh0 = vs.define_hash(records_of(first, vs.Poly)[0])
        for i in range(2, 6):
            frame = link.step(img)
            # Frame 2 may carry the repeat-once of the define; never a new outline.
            self.assertEqual([p for p in records_of(frame, vs.Poly) if p.id == pid and vs.define_hash(p) != dh0],
                             [], f"frame {i}")
            self.assertEqual(link.in_step(), (True, False, 0, 0), f"frame {i}")
            if i >= 3:
                # Verified every frame: a CONFIRM, or the define itself on a
                # carousel frame (every 4th at V0), never silence.
                resent = any(p.id == pid for p in records_of(frame, vs.Poly))
                self.assertTrue(pid in confirmed_ids(frame) or resent, f"frame {i}: {frame.records}")
        self.assertEqual(records_of(frame, vs.Del), [])
        self.assertEqual(link.enc._shapes[pid].frames_since_verify, 0)

    def test_a_better_outline_replaces_the_live_one_under_the_same_id(self):
        # The other half of the union ΔD: a genuine shape change (the rect
        # grows an arm, IoU < 0.7, the UPD path cannot fit it) is a same-id
        # redefine, and both ends agree afterwards.
        link = Link()
        for _ in range(3):
            first = link.step(scene(rects=self.RECTS))
        pid = records_of(link.frames[0], vs.Poly)[0].id
        dh0 = vs.define_hash(records_of(link.frames[0], vs.Poly)[0])
        grown = [(60, 20, 90, 30, RED), (60, 30, 75, 45, RED)]
        frame = link.step(scene(rects=grown))
        redefs = [p for p in records_of(frame, vs.Poly) if p.id == pid]
        self.assertEqual(len(redefs), 1, frame.records)
        self.assertNotEqual(vs.define_hash(redefs[0]), dh0)
        self.assertEqual(records_of(frame, vs.Del), [])
        self.assertEqual(link.in_step(), (True, False, 0, 0))
        for _ in range(3):
            link.step(scene(rects=grown))
            self.assertEqual(link.in_step(), (True, False, 0, 0))

    def test_id_pressure_never_starts_an_epoch_and_the_best_regions_get_ids(self):
        # A3: 40 regions against 31 mass ids restarted the epoch on every
        # non-key frame (82 % epoch starts on the bench). The surplus waits.
        rects = [(2 + 9 * i, 34 + 12 * j, 7 + 9 * i, 40 + 12 * j, RED) for i in range(10) for j in range(2)]
        link = Link()
        first = link.step(scene(rects=rects[:10]))
        self.assertEqual(len(records_of(first, vs.Poly)), 10)
        link.step(scene(rects=rects[:10]))
        many = rects + [(2 + 9 * i, 8 + 7 * j, 7 + 9 * i, 12 + 7 * j, RED) for i in range(10) for j in range(2)]
        for i in range(4):
            frame = link.step(scene(y0=8.0, rects=many))
            self.assertFalse(frame.header.key, f"frame {i + 3}")
            self.assertEqual(link.enc.epoch, 0)
            ids = [r.id for r in records_of(frame, vs.Poly)]
            self.assertEqual(len(ids), len(set(ids)))
            self.assertTrue(all(i in vs.ID_MASS for i in ids))
            self.assertEqual(link.in_step()[:2], (True, False), f"frame {i + 3}")
        live = [s for s in link.enc._shapes.values() if s.layer == "mass"]
        # Every mass id in use, none wasted: 31, or 30 on a frame whose DEL
        # (a value-based eviction, see the newcomer test) frees an id that
        # the next frame's best waiting region takes.
        self.assertIn(len(live), (len(vs.ID_MASS) - 1, len(vs.ID_MASS)))
        self.assertGreater(link.enc.last_stats["waiting"], 0)
        self.assertEqual(link.enc.last_stats["epochs"], 1)
        self.assertEqual(link.enc.last_stats["epoch_trigger"], None)

    def test_epoch_trigger_is_reported(self):
        link = Link()
        link.step(scene(blobs=self.BLOBS))
        self.assertEqual(link.enc.last_stats["epoch_trigger"], "first")
        self.assertEqual(link.enc.last_stats["epochs"], 1)
        link.step(scene(blobs=self.BLOBS))
        self.assertIsNone(link.enc.last_stats["epoch_trigger"])
        link.enc.force_epoch()
        link.step(scene(blobs=self.BLOBS))
        self.assertEqual(link.enc.last_stats["epoch_trigger"], "forced")
        link.step(scene(blobs=self.BLOBS), epoch_start=True)
        self.assertEqual(link.enc.last_stats["epoch_trigger"], "forced")
        self.assertEqual(link.enc.last_stats["epochs"], 3)

    # ---------------------------------------------------------------- review round 2 (C1-C7)

    @staticmethod
    def grid31():
        """31 masses 8×8 on a 13 px pitch (7 columns × 5 rows minus 4): every
        mass id in use, none waiting, 5 px gaps so a 3 px move stays separate."""
        rects = [(1 + 13 * i, 4 + 13 * j, 8 + 13 * i, 11 + 13 * j, RED) for j in range(5) for i in range(7)]
        return rects[:31]

    def test_colour_drift_below_the_fill_resolution_keeps_the_shape_verified(self):
        # A1 path (a): ΔE 12.8 between captures, the same RGB444 FILL. The
        # old rule (ΔE ≤ 6 only) left the shape silent until both ends
        # dropped it and it came back under a new id (a one-frame blink).
        link = Link()
        for _ in range(3):
            first = link.step(scene(rects=[(60, 20, 90, 30, (90, 70, 40))]))
        pid = records_of(link.frames[0], vs.Poly)[0].id
        for i in range(TTL_FRAMES + 4):
            frame = link.step(scene(rects=[(60, 20, 90, 30, (78, 76, 28))]))
            polys = records_of(frame, vs.Poly)
            self.assertTrue(pid in confirmed_ids(frame) or any(p.id == pid for p in polys),
                            f"frame {i + 4}: {frame.records}")
            self.assertEqual(records_of(frame, vs.Del), [])
            self.assertEqual(records_of(frame, vs.Ucol), [])
            self.assertEqual(link.in_step(), (True, False, 0, 0), f"frame {i + 4}")
        self.assertEqual(link.enc.last_stats["ttl_dropped"], 0)
        self.assertEqual(link.enc._shapes[pid].frames_since_verify, 0)

    def test_starved_frames_age_the_mirror_like_the_store(self):
        # C7: at 14 B a frame holds RESID + STATUS and no CONFIRM or DIGEST,
        # so nothing verifies at the base; the mirror's clock must run on
        # what went on the wire, not on what the encoder considered verified.
        link = Link()
        img = scene(blobs=self.BLOBS)
        for _ in range(2):
            first = link.step(img)
        n_live = records_of(link.frames[0], vs.Digest)[0].n_live
        for i in range(TTL_FRAMES + 2):
            frame = link.step(img, budget=14)
            self.assertEqual({type(r).__name__ for r in frame.records}, {"HznResid", "Status"}, frame.records)
            self.assertEqual(link.enc.last_stats["ttl_dropped"], link.store.stats["ttl_dropped"], f"frame {i + 3}")
            self.assertEqual(*link.ttl_clocks())
        self.assertEqual(link.store.stats["ttl_dropped"], n_live)
        for _ in range(3):
            link.step(img)
            self.assertEqual(link.in_step(), (True, False, 0, n_live))
        self.assertEqual(*link.ttl_clocks())

    def test_ucol_does_not_verify_on_either_end(self):
        # §4.3 "20 frames without a verified define, CONFIRM or UPD": a shape
        # recoloured every frame gets UCOLs and no CONFIRM, so both ends expire
        # it on the same frame and re-define it — in step throughout.
        link = Link()
        colours = [(200, 60, 60), (200, 110, 60), (200, 60, 110)]    # ΔE > 6 apart, distinct RGB444, same outline
        first = link.step(scene(rects=[(60, 20, 90, 30, colours[0])]))
        pid = records_of(first, vs.Poly)[0].id
        for i in range(1, TTL_FRAMES + 6):
            frame = link.step(scene(rects=[(60, 20, 90, 30, colours[i % 3])]))
            self.assertEqual(link.in_step()[:2], (True, False), f"frame {i + 1}")
            self.assertEqual(link.enc.last_stats["ttl_dropped"], link.store.stats["ttl_dropped"], f"frame {i + 1}")
            self.assertEqual(*link.ttl_clocks())
        self.assertGreaterEqual(link.store.stats["ttl_dropped"], 1)

    def test_lost_return_to_origin_upd_heals_at_the_next_resend(self):
        # C2: the frame carrying Upd(id, 0, 0) is lost, so the base still
        # shows the shape shifted. A re-send must state the offset even
        # though the mirror's is the define's own (0, 0).
        link = Link()
        base = [(20, 44, 4, GREEN), (50, 40, 4, GREEN)]
        for _ in range(3):
            link.step(scene(blobs=base))
        moved = link.step(scene(blobs=[base[0], (53, 40, 4, GREEN)]))
        sid = records_of(moved, vs.Upd)[0].id
        back = link.step(scene(blobs=base), lose=True)
        self.assertEqual(records_of(back, vs.Upd), [vs.Upd(sid, 0, 0)])
        self.assertEqual(link.store._shapes[sid].off, (3, 0))
        healed = None
        for i in range(8):
            frame = link.step(scene(blobs=base))
            if link.in_step()[0] and healed is None:
                healed = i
                recs = list(frame.records)
                t = next(t for t in records_of(frame, vs.Tree) if t.id == sid)
                self.assertEqual(recs[recs.index(t) + 1], vs.Upd(sid, 0, 0), recs)
        self.assertIsNotNone(healed, "the re-send never stated the offset")
        self.assertEqual(link.store._shapes[sid].off, (0, 0))
        self.assertEqual(link.in_step()[:2], (True, False))

    def test_lost_return_to_define_colour_ucol_heals_at_the_next_resend(self):
        # C2, colour twin: UCOL away and back to the define's fill; the
        # "back" frame is lost. The re-send must state the colour.
        link = Link()
        rect = (60, 20, 90, 30)
        for _ in range(3):
            first = link.step(scene(rects=[rect + ((200, 60, 60),)]))
        pid = records_of(link.frames[0], vs.Poly)[0].id
        fill0 = records_of(link.frames[0], vs.Poly)[0].fill
        away = link.step(scene(rects=[rect + ((200, 110, 60),)]))    # same outline, ΔE > 6, new RGB444
        self.assertEqual(len(records_of(away, vs.Ucol)), 1, away.records)
        back = link.step(scene(rects=[rect + ((200, 60, 60),)]), lose=True)
        self.assertEqual(records_of(back, vs.Ucol), [vs.Ucol(pid, fill0)])
        self.assertNotEqual(link.store._shapes[pid].fill, fill0)
        healed = None
        for i in range(8):
            frame = link.step(scene(rects=[rect + ((200, 60, 60),)]))
            if link.in_step()[0] and healed is None:
                healed = i
                recs = list(frame.records)
                p = next(p for p in records_of(frame, vs.Poly) if p.id == pid)
                self.assertIn(vs.Ucol(pid, fill0), recs[recs.index(p) + 1:recs.index(p) + 3], recs)
        self.assertIsNotNone(healed, "the re-send never stated the colour")
        self.assertEqual(link.store._shapes[pid].fill, fill0)

    def test_define_precedes_its_upd_when_the_first_frame_was_lost(self):
        # C4: frame 1 (the defines) is lost; in frame 2 a tree moves. The
        # repeat of its define must come before the UPD on the wire, or the
        # base orphans the UPD and creates the shape at the wrong offset.
        link = Link()
        base = [(20, 44, 4, GREEN), (50, 40, 4, GREEN)]
        lost = link.step(scene(blobs=base), lose=True)
        ids = {t.id for t in records_of(lost, vs.Tree)}
        frame = link.step(scene(blobs=[base[0], (53, 40, 4, GREEN)]))
        upds = records_of(frame, vs.Upd)
        self.assertEqual(len(upds), 1, frame.records)
        recs = list(frame.records)
        define = next(t for t in records_of(frame, vs.Tree) if t.id == upds[0].id)
        self.assertLess(recs.index(define), recs.index(upds[0]), recs)
        self.assertEqual({t.id for t in records_of(frame, vs.Tree)}, ids)
        self.assertEqual(link.in_step(), (True, False, 0, 0))
        self.assertEqual(link.store._shapes[upds[0].id].off, (upds[0].dx, upds[0].dy))

    def test_lost_del_then_id_reuse_states_the_offset(self):
        # C3: under id pressure a DEL'd id is re-used at once. If the DEL was
        # lost the base still holds the old shape at its offset, and the
        # byte-identical fresh define would keep it (§3.4 rule 5): the define
        # must carry Upd(id, 0, 0).
        t = [1000.0]
        link = Link(clock=lambda: t[0])
        rects = self.grid31()
        for _ in range(3):
            link.step(scene(y0=8.0, rects=rects))
            t[0] += 0.5
        self.assertEqual(records_of(link.frames[-1], vs.Digest)[0].n_live, 31)
        moved = rects[:]
        moved[0] = (rects[0][0] + 3, rects[0][1], rects[0][2] + 3, rects[0][3], RED)
        frame = link.step(scene(y0=8.0, rects=moved))
        t[0] += 0.5
        upds = records_of(frame, vs.Upd)
        self.assertEqual(len(upds), 1, frame.records)
        self.assertIn((upds[0].dx, upds[0].dy), [(2, 0), (3, 0)])    # two legal 8 px quantisations
        xid = upds[0].id
        self.assertEqual(link.in_step(), (True, False, 0, 0))
        gone = link.step(scene(y0=8.0, rects=moved[1:]), lose=True)   # the DEL is lost
        t[0] += 0.5
        self.assertEqual(records_of(gone, vs.Del), [vs.Del(xid)])
        back = link.step(scene(y0=8.0, rects=rects))                  # back at the define's own place
        t[0] += 0.5
        polys = [p for p in records_of(back, vs.Poly) if p.id == xid]
        self.assertEqual(len(polys), 1, back.records)
        recs = list(back.records)
        self.assertEqual(recs[recs.index(polys[0]) + 1], vs.Upd(xid, 0, 0), recs)
        self.assertEqual(link.store._shapes[xid].off, (0, 0))
        self.assertEqual(link.in_step()[:2], (True, False))
        for _ in range(3):
            link.step(scene(y0=8.0, rects=rects))
            t[0] += 0.5
            self.assertEqual(link.in_step()[:2], (True, False))
        t[0] += 61.0
        link.step(scene(y0=8.0, rects=rects))                          # the safety refresh
        for _ in range(4):
            t[0] += 0.5
            link.step(scene(y0=8.0, rects=rects))
        self.assertEqual(link.in_step()[:2], (True, False))
        self.assertEqual(*link.ttl_clocks())

    def test_refresh_recovers_a_busy_scene_after_a_lost_frame(self):
        # C1: 31 masses, one lost UPD → resync at the base; its CONFIRMs stop
        # verifying and the carousel cannot revisit every shape within the
        # TTL, so the base drops shapes. The safety refresh must re-state
        # every kept shape before any CONFIRM or DIGEST names it, and both
        # ends must be in step within a few frames — no orphans on the key.
        t = [1000.0]
        link = Link(clock=lambda: t[0])
        rects = self.grid31()
        for _ in range(3):
            link.step(scene(y0=8.0, rects=rects))
            t[0] += 0.5
        moved = rects[:]
        moved[0] = (rects[0][0] + 3, rects[0][1], rects[0][2] + 3, rects[0][3], RED)
        lost = link.step(scene(y0=8.0, rects=moved), lose=True)
        t[0] += 0.5
        self.assertEqual(len(records_of(lost, vs.Upd)), 1, lost.records)
        for _ in range(3):
            link.step(scene(y0=8.0, rects=moved))
            t[0] += 0.5
        self.assertTrue(link.store.stats["resync"])
        for _ in range(TTL_FRAMES + 4):                                # the base drops shapes meanwhile
            link.step(scene(y0=8.0, rects=moved))
            t[0] += 0.5
        self.assertGreater(link.store.stats["ttl_dropped"], 0)
        self.assertEqual(link.enc.last_stats["ttl_dropped"], 0)
        orphans_before = link.store.stats["orphans"]
        t[0] += 61.0
        refresh = link.step(scene(y0=8.0, rects=moved))
        t[0] += 0.5
        self.assertTrue(refresh.header.key)
        self.assertEqual(records_of(refresh, vs.LayerClear), [vs.LayerClear(2)])
        self.assertEqual(records_of(refresh, vs.Confirm), [])          # nothing named before its repeat
        self.assertEqual(records_of(refresh, vs.Digest), [])
        self.assertEqual(records_of(refresh, vs.Upd), [u for u in records_of(refresh, vs.Upd)
                                                       if any(p.id == u.id for p in records_of(refresh, vs.Poly))])
        self.assertEqual(link.store.stats["orphans"], orphans_before)
        self.assertFalse(link.store.stats["resync"])
        healed = None
        for i in range(8):
            link.step(scene(y0=8.0, rects=moved))
            t[0] += 0.5
            if link.in_step()[:2] == (True, False):
                healed = i
                break
        self.assertIsNotNone(healed, "the refresh did not bring the base back in step")
        self.assertEqual(*link.ttl_clocks())
        self.assertEqual(link.store.stats["resync"], False)
        for _ in range(4):
            link.step(scene(y0=8.0, rects=moved))
            t[0] += 0.5
            self.assertEqual(link.in_step()[:2], (True, False))

    def test_a_valuable_newcomer_evicts_the_weakest_holder(self):
        # C5: 30 low-value masses hold the ids; two dark newcomers appear.
        # One takes the free id, the other must evict the weakest holder
        # (a DEL, then its id) within a few frames — no epoch, in step.
        low = (150, 96, 60)                                            # ground + 30 red: worth little
        small = [(2 + 9 * i, 30 + 9 * j, 7 + 9 * i, 35 + 9 * j, low) for i in range(10) for j in range(3)]
        link = Link()
        for _ in range(3):
            frame = link.step(scene(y0=8.0, rects=small))
        self.assertEqual(records_of(frame, vs.Digest)[0].n_live, 30)
        big = [(20, 10, 33, 19, RED), (50, 10, 63, 19, RED)]
        dels, drawn_red = 0, set()
        for i in range(6):
            frame = link.step(scene(y0=8.0, rects=small + big))
            self.assertFalse(frame.header.key, f"frame {i + 4}")
            dels += len(records_of(frame, vs.Del))
            for p in records_of(frame, vs.Poly):
                if p.fill.rgb444 == 0xC44:
                    drawn_red.add(p.id)
            self.assertEqual(link.in_step()[:2], (True, False), f"frame {i + 4}")
            if len(drawn_red) == 2 and dels == 1:
                break
        self.assertEqual(len(drawn_red), 2, "the second newcomer never got an id")
        self.assertEqual(dels, 1)
        self.assertEqual(link.enc.last_stats["epochs"], 1)
        self.assertEqual(link.enc.last_stats["waiting"], 1)            # the evictee's own region waits now
        self.assertEqual(*link.ttl_clocks())

    # ---------------------------------------------------------------- review round 3

    def test_moving_refresh_at_a_small_budget_restates_without_orphans(self):
        # 1.1: 31 masses at 80 B, all jittering 2 px, one lost UPD → resync
        # and base drops. The refresh must re-state every kept shape as one
        # record ahead of the frame's UPDs (a bare UPD to a shape the base
        # dropped is an orphan) and get both ends in step within the TTL.
        t = [1000.0]
        link = Link(clock=lambda: t[0])
        rects = self.grid31()
        shifted = [(x0 + 2, y0, x1 + 2, y1, c) for x0, y0, x1, y1, c in rects]
        frames = [rects, shifted]
        for i in range(4):
            link.step(scene(y0=8.0, rects=frames[i % 2]), budget=80)
            t[0] += 0.5
        lost = link.step(scene(y0=8.0, rects=frames[0]), budget=80, lose=True)
        t[0] += 0.5
        self.assertGreater(len(records_of(lost, vs.Upd)), 0)
        for i in range(TTL_FRAMES + 6):
            link.step(scene(y0=8.0, rects=frames[(i + 1) % 2]), budget=80)
            t[0] += 0.5
        self.assertTrue(link.store.stats["resync"])
        orphans_before = link.store.stats["orphans"]
        t[0] += 61.0
        refresh = link.step(scene(y0=8.0, rects=frames[0]), budget=80)
        self.assertTrue(refresh.header.key)
        for u in records_of(refresh, vs.Upd):                          # every UPD rides behind its define
            self.assertTrue(any(p.id == u.id for p in records_of(refresh, vs.Poly)), refresh.records)
        self.assertEqual(link.store.stats["orphans"], orphans_before)
        self.assertFalse(link.store.stats["resync"])
        healed = None
        for i in range(TTL_FRAMES):
            t[0] += 0.5
            frame = link.step(scene(y0=8.0, rects=frames[(i + 1) % 2]), budget=80)
            self.assertEqual(link.store.stats["orphans"], orphans_before, f"post {i + 1}: {frame.records}")
            if link.in_step()[:2] == (True, False):
                healed = i
                break
        self.assertIsNotNone(healed, "never back in step after the refresh")
        self.assertEqual(*link.ttl_clocks())

    def test_static_refresh_at_a_tiny_budget_keeps_the_masses(self):
        # 1.2: with the CONFIRM/DIGEST reserve counted over the kept shapes,
        # a 40 B body (or V2's F = 40) left no room for a single repeat and
        # the whole kept layer expired on both ends at every refresh.
        for label, budget, quality in (("40 B", 40, 80), ("V2", 203, 30)):
            with self.subTest(label):
                t = [1000.0]
                link = Link(clock=lambda: t[0])
                rects = self.grid31()
                for _ in range(6):
                    link.step(scene(y0=8.0, rects=rects), quality=quality)
                    t[0] += 0.5
                n_live = records_of(link.frames[-1], vs.Digest)[0].n_live
                self.assertGreaterEqual(n_live, 13)
                t[0] += 61.0
                refresh = link.step(scene(y0=8.0, rects=rects), budget=budget, quality=quality)
                self.assertTrue(refresh.header.key)
                digest_at = None
                for i in range(TTL_FRAMES + 4):
                    t[0] += 0.5
                    frame = link.step(scene(y0=8.0, rects=rects), budget=budget, quality=quality)
                    self.assertEqual(link.store.stats["ttl_dropped"], 0, f"{label} post {i + 1}")
                    self.assertEqual(link.enc.last_stats["ttl_dropped"], 0, f"{label} post {i + 1}")
                    if digest_at is None and records_of(frame, vs.Digest):
                        digest_at = i
                self.assertIsNotNone(digest_at, "no DIGEST resumed within the TTL")
                self.assertLess(digest_at, TTL_FRAMES)
                self.assertEqual(link.in_step()[:2], (True, False))
                self.assertEqual(*link.ttl_clocks())
                # Nothing kept was lost (at V2 the scene is still filling, so it may have grown).
                self.assertGreaterEqual(records_of(frame, vs.Digest)[0].n_live, n_live)

    def test_underlying_mass_does_not_start_an_eviction_cascade(self):
        # 1.3: holders valued against L0 while newcomers score against the
        # mirror: a static scene with a large mass under the grid cascaded
        # for ever (one eviction every second frame). Both must be measured
        # against the mirror without the shape.
        plate = [(48, 28, 95, 63, (60, 50, 40))]                       # a 1680 px dark mass under the right half
        rects = plate + self.grid31()
        link = Link()
        dels_after_settle = 0
        for i in range(60):
            frame = link.step(scene(y0=8.0, rects=rects))
            if i >= 3:
                dels_after_settle += len([d for d in records_of(frame, vs.Del) if d.id in vs.ID_MASS])
            self.assertEqual(link.in_step()[:2], (True, False), f"frame {i + 1}")
        self.assertTrue(any(o.area > 1000 for o in link.enc._shapes.values()), "no large mass under the grid")
        self.assertGreater(link.enc.last_stats["waiting"], 0)          # the mass ids are all in use
        self.assertEqual(dels_after_settle, 0, "an eviction cascade on a static scene")
        self.assertEqual(link.enc.last_stats["epochs"], 1)

    def test_jittering_holder_is_not_evicted_for_a_weaker_newcomer(self):
        # 1.4: a holder valued on its stale mirror raster (before this
        # frame's UPD) looked worthless and a weaker newcomer evicted it
        # every few frames. Value the region it will describe.
        rects = self.grid31()
        newcomer = [(40, 52, 49, 57, RED)]                              # 10×6: below the 2× bar
        link = Link()
        for _ in range(3):
            link.step(scene(y0=8.0, rects=rects))
        jid = None
        dels = []
        for i in range(14):
            dx = [0, 3, 0, -3][i % 4]
            moving = rects[:]
            moving[10] = (rects[10][0] + dx, rects[10][1], rects[10][2] + dx, rects[10][3], RED)
            frame = link.step(scene(y0=8.0, rects=moving + newcomer))
            for u in records_of(frame, vs.Upd):
                jid = u.id
            dels += [d.id for d in records_of(frame, vs.Del)]
            self.assertEqual(link.in_step()[:2], (True, False), f"frame {i + 4}")
        self.assertIsNotNone(jid)
        self.assertNotIn(jid, dels, f"the jittering holder was evicted: {dels}")

    def test_upd_verifies_on_both_ends(self):
        # 1.7: a shape that moves every frame gets an UPD and no CONFIRM; the
        # store verifies on an UPD, so the mirror must too.
        link = Link()
        base = [(20, 44, 4, GREEN), (50, 40, 4, GREEN)]
        link.step(scene(blobs=base))
        for i in range(TTL_FRAMES + 8):
            x = 53 if i % 2 == 0 else 50
            frame = link.step(scene(blobs=[base[0], (x, 40, 4, GREEN)]))
            self.assertEqual(len(records_of(frame, vs.Upd)), 1, f"frame {i + 2}: {frame.records}")
            self.assertEqual(link.in_step(), (True, False, 0, 0), f"frame {i + 2}")
            self.assertEqual(*link.ttl_clocks())
        self.assertEqual(link.enc.last_stats["ttl_dropped"], 0)

    def test_lost_del_and_lost_reuse_define_still_state_the_offset(self):
        # 1.8 / 1.5: the DEL is lost AND the re-use frame (define + Upd(0,0))
        # is lost too: the next re-send must still state the offset, so the
        # fresh shape must remember it ever had one.
        link = Link()
        rects = self.grid31()
        for _ in range(3):
            link.step(scene(y0=8.0, rects=rects))
        moved = rects[:]
        moved[0] = (rects[0][0] + 3, rects[0][1], rects[0][2] + 3, rects[0][3], RED)
        frame = link.step(scene(y0=8.0, rects=moved))
        upds = records_of(frame, vs.Upd)
        self.assertEqual(len(upds), 1, frame.records)
        xid, off = upds[0].id, (upds[0].dx, upds[0].dy)
        gone = link.step(scene(y0=8.0, rects=moved[1:]), lose=True)
        self.assertEqual(records_of(gone, vs.Del), [vs.Del(xid)])
        reuse = link.step(scene(y0=8.0, rects=rects), lose=True)
        recs = list(reuse.records)
        p = next(p for p in records_of(reuse, vs.Poly) if p.id == xid)
        self.assertEqual(recs[recs.index(p) + 1], vs.Upd(xid, 0, 0), recs)
        self.assertEqual(link.store._shapes[xid].off, off)
        healed = None
        for i in range(8):
            frame = link.step(scene(y0=8.0, rects=rects))
            if link.in_step()[0] and healed is None:
                healed = i
                recs = list(frame.records)
                p = next(p for p in records_of(frame, vs.Poly) if p.id == xid)
                self.assertEqual(recs[recs.index(p) + 1], vs.Upd(xid, 0, 0), recs)
        self.assertIsNotNone(healed)
        self.assertEqual(link.store._shapes[xid].off, (0, 0))

    def test_lost_del_then_id_reuse_states_the_colour(self):
        # x20: colour twin of the lost-DEL test: the old shape was UCOL'd,
        # the DEL is lost, the region returns in the define's colour.
        link = Link()
        rects = self.grid31()
        for _ in range(3):
            first = link.step(scene(y0=8.0, rects=rects))
        recoloured = rects[:]
        recoloured[0] = rects[0][:4] + ((200, 110, 60),)
        frame = link.step(scene(y0=8.0, rects=recoloured))
        ucols = records_of(frame, vs.Ucol)
        self.assertEqual(len(ucols), 1, frame.records)
        xid = ucols[0].id
        fill0 = next(p.fill for p in records_of(link.frames[0], vs.Poly) if p.id == xid)
        gone = link.step(scene(y0=8.0, rects=recoloured[1:]), lose=True)
        self.assertEqual(records_of(gone, vs.Del), [vs.Del(xid)])
        back = link.step(scene(y0=8.0, rects=rects))
        recs = list(back.records)
        p = next(p for p in records_of(back, vs.Poly) if p.id == xid)
        self.assertIn(vs.Ucol(xid, fill0), recs[recs.index(p) + 1:recs.index(p) + 3], recs)
        self.assertEqual(link.store._shapes[xid].fill, fill0)
        self.assertEqual(link.in_step()[:2], (True, False))

    def test_owed_shape_deleted_before_its_repeat_does_not_block_the_digest(self):
        # x17: a kept shape whose region vanishes right after the refresh
        # owes nothing any more; the DIGEST must resume once the others are
        # re-stated.
        t = [1000.0]
        link = Link(clock=lambda: t[0])
        rects = self.grid31()
        for _ in range(3):
            link.step(scene(y0=8.0, rects=rects))
            t[0] += 0.5
        t[0] += 61.0
        refresh = link.step(scene(y0=8.0, rects=rects), budget=60)     # small: the key re-states only a few
        self.assertTrue(refresh.header.key)
        self.assertGreater(len(link.enc._restate_owed), 0)
        digest_at = None
        for i in range(TTL_FRAMES):
            t[0] += 0.5
            frame = link.step(scene(y0=8.0, rects=rects[1:]))          # rect 0 is gone: a DEL, still owed
            if records_of(frame, vs.Digest):
                digest_at = i
                break
        self.assertIsNotNone(digest_at, "the DIGEST never resumed")
        self.assertEqual(link.in_step()[:2], (True, False))

    def test_alloc_id_cooldown(self):
        # m06/m07: a freed id is passed over for TTL_FRAMES frames while any
        # other id is free, then reused; when nothing else is free the
        # OLDEST-freed id is the last resort.
        link = Link()
        link.step(scene(blobs=self.BLOBS))
        enc = link.enc
        ids = sorted(enc._shapes)
        a, b = ids[0], ids[1]
        enc._frame_no = 100
        enc._release_id(a)
        enc._frame_no = 105
        enc._release_id(b)
        taken = set(vs.ID_PLANT) - {a, b}
        enc._frame_no = 106
        self.assertEqual(enc._alloc_id("plant", taken), a)               # both cooling: the oldest freed
        enc._frame_no = 100 + TTL_FRAMES
        self.assertEqual(enc._alloc_id("plant", taken - {vs.ID_PLANT[-1]}), vs.ID_PLANT[-1])   # a free id wins
        enc._frame_no = 100 + TTL_FRAMES + 1
        self.assertEqual(enc._alloc_id("plant", taken), a)               # a's cooldown is over


if __name__ == "__main__":
    unittest.main()
