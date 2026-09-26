"""Unit tests for the tractor VS1 encoder (``x8_image_pipeline/encode_vector.py``,
``VECTOR_SCENE.md`` §8.6 row ``test_vector_encoder.py``).

Every scene is synthetic numpy (no image files): a sky/ground split along a
known canvas-space horizon, optional blobs, rectangles and a self-mask. The
tests decode every emitted frame with the shared codec and check the
container rules (6-byte header, body ≤ budget − 6, − 7 on epoch starts,
K = frame_kind, LL from the quality band), the L0 horizon within one 8 px
cell at several tilts and with a curved pre-calibration horizon, NO_HORIZON
on a ground-only view, a raised-bucket edge that must not capture the
horizon, three green blobs → three TREE records and none on pasture, the
static-scene convergence (repeat-once, then CONFIRM + DIGEST), a residual
that never increases, the self-mask rules and the epoch triggers.

Needs cv2 (``opencv-python-headless``); skipped where it is absent so the
stripped CI image still passes.
"""
from __future__ import annotations

import os
import sys
import unittest

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_BS_DIR = os.path.dirname(_THIS_DIR)
_X8_DIR = os.path.normpath(os.path.join(_THIS_DIR, "..", "..", "firmware", "tractor_x8"))
for _d in (_BS_DIR, _X8_DIR):
    if _d not in sys.path:
        sys.path.insert(0, _d)

from image_pipeline.vector_scene import codec as vs  # noqa: E402

try:                                             # pragma: no cover
    import numpy as np
    import cv2  # noqa: F401
    _HAVE_CV2 = True
except ImportError:                              # pragma: no cover
    np = None
    _HAVE_CV2 = False

if _HAVE_CV2:
    from x8_image_pipeline import encode_vector as ev  # noqa: E402
    from x8_image_pipeline import vector_extract as vx  # noqa: E402

W, H = 96, 64                       # working resolution
CW, CH = 384, 256                   # canvas
SKY = (120, 170, 235)
GROUND = (120, 96, 60)
GREEN = (40, 140, 40)
BUDGET = 203                        # p1 FHSS whole-payload budget (§1)


def horizon_truth(x_canvas, y0=112.0, slope=0.0, sag=0.0):
    u = (np.asarray(x_canvas, np.float64) - CW / 2) / (CW / 2)
    return y0 + slope * (CW / 2) * u + sag * u * u


def scene(y0=112.0, slope=0.0, sag=0.0, blobs=(), rects=(), size=(W, H), sky=SKY, ground=GROUND,
          noise=0.0, seed=0):
    """Sky above the canvas-space horizon y = y0 + slope·(x − 192) + sag·u²,
    ground below; ``blobs`` are (cx, cy, r, colour) and ``rects`` are
    (x0, y0, x1, y1, colour), all in the image's own pixels."""
    w, h = size
    img = np.zeros((h, w, 3), np.uint8)
    scale = CW / w
    xs = (np.arange(w) + 0.5) * scale
    line = horizon_truth(xs, y0, slope, sag) / scale
    ys = np.arange(h)[:, None] + 0.5
    above = ys < line[None, :]
    img[above] = sky
    img[~above] = ground
    yy, xx = np.mgrid[0:h, 0:w]
    for x0, y0_, x1, y1, colour in rects:
        img[y0_:y1 + 1, x0:x1 + 1] = colour
    for cx, cy, r, colour in blobs:
        img[((xx - cx) ** 2 + (yy - cy) ** 2) <= r * r] = colour
    if noise:
        rng = np.random.default_rng(seed)
        img = np.clip(img.astype(np.int16) + rng.integers(-noise, noise + 1, img.shape), 0, 255).astype(np.uint8)
    return img


def decode(frame_bytes: bytes):
    """Check the container and return the decoded VS frame."""
    assert len(frame_bytes) >= 6
    hdr = frame_bytes[:6]
    assert tuple(hdr[2:6]) == (12, 8, 32, 6), hdr
    assert hdr[0] in (0, 1)
    return vs.decode_frame(frame_bytes[6:], frame_kind=hdr[0])


def records_of(frame, kind):
    return [r for r in frame.records if isinstance(r, kind)]


def shown_horizon(frame):
    """The horizon the base holds after this frame's HZN record (ABS only)."""
    abs_recs = records_of(frame, vs.HznAbs)
    if not abs_recs:
        return None
    r = abs_recs[-1]
    return vx.horizon_from_codes(r.y, r.ang, r.curv)


@unittest.skipUnless(_HAVE_CV2, "opencv-python-headless not installed")
class ContainerTests(unittest.TestCase):
    def setUp(self):
        self.img = scene(blobs=[(20, 44, 4, GREEN), (50, 40, 4, GREEN), (80, 48, 4, GREEN)])

    def test_header_budget_and_key_bit(self):
        enc = ev.VectorEncoder()
        for i in range(6):
            fb = enc.frame(self.img, BUDGET, seq=i, quality=80)
            self.assertEqual(tuple(fb[1:6]), (i & 0xFF, 12, 8, 32, 6))
            frame = decode(fb)
            self.assertEqual(frame.header.key, bool(fb[0]))
            self.assertEqual(frame.header.epoch, enc.epoch & 15)
            cap = BUDGET - 6 - (1 if fb[0] else 0)
            self.assertLessEqual(len(fb) - 6, cap, (i, len(fb)))
            self.assertLessEqual(fb[6], 0x7F)               # never a fragment magic
        self.assertTrue(enc.last_stats["epoch_start"] is False)

    def test_first_frame_is_an_epoch_start_with_anchor_and_layer_clear(self):
        enc = ev.VectorEncoder()
        frame = decode(enc.frame(self.img, BUDGET))
        self.assertTrue(frame.header.key)
        self.assertIsInstance(frame.records[0], vs.HznAbs)
        self.assertEqual(frame.records[1], vs.LayerClear(0))
        self.assertEqual(len(records_of(frame, vs.Status)), 1)
        self.assertEqual(len(records_of(frame, vs.Digest)), 1)

    def test_level_follows_the_quality_band(self):
        for quality, level in ((100, 0), (80, 0), (60, 0), (59, 1), (40, 1), (39, 2), (20, 2), (19, 3), (1, 3)):
            enc = ev.VectorEncoder()
            fb = enc.frame(self.img, BUDGET, quality=quality)
            self.assertEqual(decode(fb).header.level, level, quality)
            self.assertEqual(enc.last_stats["level"], level)
            body_cap = {0: BUDGET - 6, 1: 100, 2: 40, 3: 20}[level] - 1
            self.assertLessEqual(len(fb) - 6, body_cap, quality)
        self.assertEqual(ev.level_of_quality(0), 3)
        self.assertEqual(ev.level_of_quality(255), 0)

    def test_v3_is_a_beacon_of_anchor_and_status_only(self):
        enc = ev.VectorEncoder()
        for i in range(3):
            frame = decode(enc.frame(self.img, BUDGET, quality=10, seq=i))
            kinds = {type(r) for r in frame.records}
            self.assertTrue(kinds <= {vs.HznAbs, vs.LayerClear, vs.Status}, kinds)
            self.assertIn(vs.Status, kinds)
            self.assertIn(vs.HznAbs, kinds)

    def test_small_budgets_never_exceed_the_body(self):
        enc = ev.VectorEncoder()
        for budget in (8, 12, 20, 40, 60, 100, 203, 243):
            for i in range(3):
                fb = enc.frame(self.img, budget, seq=i)
                self.assertLessEqual(len(fb), budget - (1 if fb[0] else 0))
                if len(fb) > 6:
                    decode(fb)

    def test_age_and_moving_and_status_codes(self):
        enc = ev.VectorEncoder()
        frame = decode(enc.frame(self.img, BUDGET, age_units=3, moving=True))
        self.assertEqual(frame.header.age, 3)
        status = records_of(frame, vs.Status)[0]
        self.assertEqual(status, vs.Status(0, 30, 0, 64, 0, 0, False, True))
        frame = decode(enc.frame(self.img, BUDGET, age_units=99))
        self.assertEqual(frame.header.age, 15)

    def test_last_stats_reports_timings_and_layers(self):
        enc = ev.VectorEncoder()
        enc.frame(self.img, BUDGET)
        st = enc.last_stats
        for key in ("resize", "l0", "l1", "temporal_pack", "total"):
            self.assertIn(key, st["ms"])
            self.assertGreaterEqual(st["ms"][key], 0.0)
        for key in ("hdr", "L0", "L1", "L2", "L3", "ctrl", "pad"):
            self.assertIn(key, st["bits"])
        self.assertIn("residual", st)
        self.assertIn("records", st)
        self.assertEqual(st["bits"]["hdr"], 13)

    def test_any_input_size_is_resized_to_the_working_image(self):
        big = scene(size=(CW, CH), blobs=[(80, 176, 16, GREEN), (200, 160, 16, GREEN), (320, 192, 16, GREEN)])
        enc = ev.VectorEncoder()
        frame = decode(enc.frame(big, BUDGET))
        self.assertEqual(len(records_of(frame, vs.Tree)), 3)
        hz = shown_horizon(frame)
        self.assertLessEqual(abs(hz.line_canvas(np.array([192.0]))[0] - 112.0), 8.0)


@unittest.skipUnless(_HAVE_CV2, "opencv-python-headless not installed")
class HorizonTests(unittest.TestCase):
    def check_line(self, frame, y0, slope, sag, tol=8.0):
        hz = shown_horizon(frame)
        self.assertIsNotNone(hz, "expected an HZN ABS anchor")
        xs = np.array([8.0, 96.0, 192.0, 288.0, 376.0])
        err = np.abs(hz.line_canvas(xs) - horizon_truth(xs, y0, slope, sag))
        self.assertLessEqual(float(err.max()), tol, (y0, slope, sag, err))

    def test_horizon_within_one_cell_at_several_tilts(self):
        for y0, slope in ((112.0, 0.0), (80.0, 0.1), (150.0, -0.15), (112.0, 0.25), (60.0, -0.05)):
            enc = ev.VectorEncoder()
            frame = decode(enc.frame(scene(y0=y0, slope=slope), BUDGET))
            self.check_line(frame, y0, slope, 0.0)

    def test_curved_pre_calibration_horizon(self):
        for sag in (20.0, -16.0):
            enc = ev.VectorEncoder()
            frame = decode(enc.frame(scene(y0=120.0, sag=sag), BUDGET))
            self.check_line(frame, 120.0, 0.0, sag)
            self.assertNotEqual(records_of(frame, vs.HznAbs)[0].curv, 0)

    def test_no_horizon_when_the_view_is_all_ground(self):
        enc = ev.VectorEncoder()
        frame = decode(enc.frame(scene(y0=-40.0), BUDGET))
        self.assertTrue(frame.header.key)
        self.assertIsInstance(frame.records[0], vs.HznNoHorizon)
        self.assertEqual(records_of(frame, vs.HznAbs), [])
        self.assertFalse(enc.last_stats["horizon"]["found"])
        # A NO_HORIZON epoch sends no RESID (§3.3).
        frame = decode(enc.frame(scene(y0=-40.0), BUDGET))
        frame = decode(enc.frame(scene(y0=-40.0), BUDGET))
        self.assertEqual(records_of(frame, vs.HznResid), [])

    def test_raised_bucket_edge_does_not_capture_the_horizon(self):
        # A dark bucket rectangle in the lower half: its straight top edge is
        # the strongest luma edge, but sky is ordered on chroma (§2.4).
        img = scene(y0=112.0, slope=0.05, rects=[(24, 40, 70, 60, (30, 30, 30))])
        enc = ev.VectorEncoder()
        self.check_line(decode(enc.frame(img, BUDGET)), 112.0, 0.05, 0.0)

    def test_resid_follows_a_small_move_and_abs_a_large_one(self):
        enc = ev.VectorEncoder()
        decode(enc.frame(scene(y0=112.0), BUDGET))
        decode(enc.frame(scene(y0=112.0), BUDGET))              # anchor repeat
        frame = decode(enc.frame(scene(y0=120.0), BUDGET))        # +8 px = dy5 code 4
        resid = records_of(frame, vs.HznResid)
        self.assertEqual(len(resid), 1)
        self.assertEqual(resid[0].dy, 4)
        self.assertEqual(records_of(frame, vs.HznAbs), [])
        enc2 = ev.VectorEncoder()
        decode(enc2.frame(scene(y0=112.0), BUDGET))
        decode(enc2.frame(scene(y0=112.0), BUDGET))
        # A 60 px jump exceeds the RESID field: seen twice (§2.4 consistency), then re-anchored as ABS.
        decode(enc2.frame(scene(y0=172.0), BUDGET))
        frame = decode(enc2.frame(scene(y0=172.0), BUDGET))
        self.assertEqual(records_of(frame, vs.HznResid), [])
        self.check_line(frame, 172.0, 0.0, 0.0)

    def test_band_colours_are_measured(self):
        enc = ev.VectorEncoder()
        rec = records_of(decode(enc.frame(scene(), BUDGET)), vs.HznAbs)[0]
        sky_rgb = vx.fill_shown_rgb8(rec.sky)
        gnd_rgb = vx.fill_shown_rgb8(rec.gnd)
        self.assertLess(float(np.abs(sky_rgb - np.array(SKY)).max()), 9.0)    # RGB444 step is 17
        self.assertLess(float(np.abs(gnd_rgb - np.array(GROUND)).max()), 9.0)


@unittest.skipUnless(_HAVE_CV2, "opencv-python-headless not installed")
class LayerTests(unittest.TestCase):
    def test_three_isolated_green_blobs_become_three_trees(self):
        img = scene(blobs=[(20, 44, 4, GREEN), (50, 40, 4, GREEN), (80, 48, 4, GREEN)])
        enc = ev.VectorEncoder()
        frame = decode(enc.frame(img, BUDGET))
        trees = records_of(frame, vs.Tree)
        self.assertEqual(len(trees), 3)
        centres = sorted((t.cx * 8, t.cy * 8) for t in trees)
        for (cx, cy), (tx, ty) in zip(centres, sorted(((80, 176), (200, 160), (320, 192)))):
            self.assertLessEqual(abs(cx - tx), 8)
            self.assertLessEqual(abs(cy - ty), 8)
        for t in trees:
            self.assertIn(t.id, vs.ID_PLANT)
            self.assertLess(float(vx.delta_e76(vx.rgb_to_lab(vx.fill_shown_rgb8(t.fill)), vx.rgb_to_lab(np.array(GREEN, np.float32)))), 6.0)
        self.assertEqual(records_of(frame, vs.Poly), [])

    def test_pasture_gives_no_shapes(self):
        enc = ev.VectorEncoder()
        frame = decode(enc.frame(scene(), BUDGET))
        self.assertEqual(records_of(frame, vs.Tree), [])
        self.assertEqual(records_of(frame, vs.Poly), [])
        self.assertEqual(records_of(frame, vs.Digest)[0].n_live, 0)

    def test_rectangles_become_polygons_with_measured_fills(self):
        # (250, 250, 250) is within ΔE76 ≤ 4 of the white slot; (240, 240, 240) is 5.2 away.
        img = scene(rects=[(10, 36, 40, 56, (200, 60, 60)), (60, 30, 90, 50, (250, 250, 250))])
        enc = ev.VectorEncoder()
        frame = decode(enc.frame(img, BUDGET))
        polys = records_of(frame, vs.Poly)
        self.assertEqual(len(polys), 2)
        for p in polys:
            self.assertIn(p.id, vs.ID_MASS)
            self.assertEqual(p.grid, 0)
            self.assertGreaterEqual(len(p.vertices), 3)
            self.assertLessEqual(len(p.vertices), 10)
        by_x = sorted(polys, key=lambda p: p.vertices[0][0])
        self.assertIsNone(by_x[0].fill.palette)              # (200, 60, 60) is no farm slot
        self.assertEqual(by_x[1].fill.palette, 7)             # near-white → slot 7 within ΔE ≤ 4

    def test_gradient_is_sent_for_a_shaded_region(self):
        img = scene()
        ramp = np.linspace(60, 200, 40).astype(np.uint8)
        img[36:60, 20:60, 0] = ramp[None, :]                    # luma rises left → right
        img[36:60, 20:60, 1] = ramp[None, :]
        img[36:60, 20:60, 2] = ramp[None, :]
        enc = ev.VectorEncoder()
        polys = records_of(decode(enc.frame(img, BUDGET)), vs.Poly)
        self.assertTrue(polys)
        grads = [p.fill.grad for p in polys if p.fill.grad is not None]
        self.assertTrue(grads, polys)
        d, dl = grads[0]
        self.assertIn(d, (0, 7, 1))                              # horizontal
        self.assertNotEqual(dl, 0)


@unittest.skipUnless(_HAVE_CV2, "opencv-python-headless not installed")
class EdgeTests(unittest.TestCase):
    def rut_and_wire(self):
        img = scene(blobs=[(20, 44, 4, GREEN), (50, 40, 4, GREEN), (80, 48, 4, GREEN)])
        import cv2 as _cv2
        _cv2.line(img, (30, 46), (80, 60), (70, 50, 30), 1)          # a rut in the ground
        _cv2.line(img, (0, 8), (95, 12), (60, 60, 70), 1)            # an overhead wire in the sky
        return img

    def test_rut_and_wire_become_edges_with_classes(self):
        enc = ev.VectorEncoder()
        frame = decode(enc.frame(self.rut_and_wire(), BUDGET))
        edges = records_of(frame, vs.Edge)
        self.assertEqual(len(edges), 2, edges)
        for e in edges:
            self.assertIn(e.id, vs.ID_EDGE)
            self.assertLessEqual(len(e.points), 5)
        wire = [e for e in edges if e.cls == 4]
        rut = [e for e in edges if e.cls == 0]
        self.assertEqual((len(wire), len(rut)), (1, 1), edges)
        self.assertTrue(all(7 <= y <= 14 for _, y in wire[0].points), wire)
        xs = sorted(p[0] for p in rut[0].points)
        self.assertLessEqual(abs(xs[0] - 30), 3)
        self.assertLessEqual(abs(xs[-1] - 80), 3)
        self.assertEqual(len(records_of(frame, vs.Tree)), 3)
        self.assertGreater(enc.last_stats["bits"]["L3"], 0)
        self.assertIn("l3", enc.last_stats["ms"])

    def test_static_edges_are_repeated_then_confirmed(self):
        enc = ev.VectorEncoder()
        first = decode(enc.frame(self.rut_and_wire(), BUDGET))
        ids = {e.id for e in records_of(first, vs.Edge)}
        second = decode(enc.frame(self.rut_and_wire(), BUDGET))
        self.assertEqual({e.id for e in records_of(second, vs.Edge)}, ids)
        third = decode(enc.frame(self.rut_and_wire(), BUDGET))
        self.assertEqual(records_of(third, vs.Edge), [])
        self.assertEqual(records_of(third, vs.Del), [])
        confirmed = {c.base_id + i for c in records_of(third, vs.Confirm) for i, t in enumerate(c.tags) if t is not None}
        self.assertTrue(ids <= confirmed, (ids, confirmed))
        self.assertEqual(records_of(third, vs.Digest)[0].n_live, 5)

    def test_polygon_outlines_are_not_edges(self):
        img = scene(blobs=[(20, 44, 4, GREEN), (50, 40, 4, GREEN), (80, 48, 4, GREEN)],
                    rects=[(60, 36, 90, 48, (200, 60, 60))])
        enc = ev.VectorEncoder()
        frame = decode(enc.frame(img, BUDGET))
        self.assertEqual(records_of(frame, vs.Edge), [])
        self.assertEqual(len(records_of(frame, vs.Poly)), 1)


@unittest.skipUnless(_HAVE_CV2, "opencv-python-headless not installed")
class TemporalTests(unittest.TestCase):
    def setUp(self):
        self.img = scene(blobs=[(20, 44, 4, GREEN), (50, 40, 4, GREEN), (80, 48, 4, GREEN)],
                         rects=[(60, 20, 90, 30, (200, 60, 60))])

    def test_static_scene_repeats_then_confirms(self):
        enc = ev.VectorEncoder()
        f1 = enc.frame(self.img, BUDGET, seq=0)
        d1 = decode(f1)
        defines1 = {r.id: r for r in d1.records if isinstance(r, (vs.Poly, vs.Tree, vs.Edge))}
        self.assertEqual(len(defines1), 4)
        f2 = enc.frame(self.img, BUDGET, seq=1)
        d2 = decode(f2)
        # Repeat-once: the same define records (same define-hash) re-verified on this capture,
        # plus the epoch-start anchor repeated (§3.1); DIGEST every frame.
        defines2 = {r.id: r for r in d2.records if isinstance(r, (vs.Poly, vs.Tree, vs.Edge))}
        self.assertEqual(defines2, defines1)
        self.assertEqual({vs.define_hash(r) for r in defines2.values()}, {vs.define_hash(r) for r in defines1.values()})
        self.assertEqual(len(records_of(d2, vs.Digest)), 1)
        self.assertFalse(d2.header.key)
        self.assertLessEqual(len(f2), len(f1))
        f3 = enc.frame(self.img, BUDGET, seq=2)
        d3 = decode(f3)
        self.assertLess(len(f3), len(f1))
        self.assertEqual(records_of(d3, vs.Poly) + records_of(d3, vs.Tree), [])
        confirms = records_of(d3, vs.Confirm)
        self.assertTrue(confirms)
        confirmed = {c.base_id + i for c in confirms for i, t in enumerate(c.tags) if t is not None}
        self.assertEqual(confirmed, set(defines1))
        self.assertEqual(records_of(d3, vs.Digest)[0].n_live, 4)
        self.assertEqual(records_of(d3, vs.HznResid), [vs.HznResid(0, 0)])
        # The DIGEST is the CRC over the mirror; recompute it from the decoded state.
        shapes = [(r.id, vs.define_hash(r), vs.state_hash(0, 0, vx.fill_base_rgb444(r.fill), [], []))
                  for r in defines1.values()]
        self.assertEqual(records_of(d3, vs.Digest)[0].crc, vs.digest_crc(shapes, [(0, 0)] * 4, 128, (16, 16, 16)))
        for c in confirms:
            for i, tag in enumerate(c.tags):
                if tag is not None:
                    rec = defines1[c.base_id + i]
                    self.assertEqual(tag, vs.state_hash(0, 0, vx.fill_base_rgb444(rec.fill), [], []) & 3)

    def test_repeat_skipped_by_the_budget_is_sent_in_the_next_frame(self):
        # §4.3 repeat-once is owed until it is really sent. Frame 2's budget
        # holds exactly the anchor repeat + STATUS + the CONFIRM/DIGEST reserve
        # (28 B whole payload: 22 B body = 163 record bits, 100 used by slots
        # 1–4, 55 reserved, so no 42-bit TREE fits), so the three defines are
        # not repeated there; they must go out in frame 3, not never.
        img = scene(blobs=[(20, 44, 4, GREEN), (50, 40, 4, GREEN), (80, 48, 4, GREEN)])
        enc = ev.VectorEncoder()
        first = decode(enc.frame(img, BUDGET, seq=0))
        ids = {t.id for t in records_of(first, vs.Tree)}
        self.assertEqual(len(ids), 3)
        second = decode(enc.frame(img, 28, seq=1))
        self.assertEqual(records_of(second, vs.Tree), [])
        self.assertEqual(len(records_of(second, vs.HznAbs)), 1)          # the anchor repeat did fit
        offered = [c for c in enc.last_stats["candidates"] if c[0] == 6]
        self.assertEqual(len(offered), 3)
        self.assertFalse(any(c[4] for c in offered), offered)              # offered, not packed
        third = decode(enc.frame(img, BUDGET, seq=2))
        self.assertEqual({t.id for t in records_of(third, vs.Tree)}, ids)
        packed = [c for c in enc.last_stats["candidates"] if c[0] == 6]
        self.assertEqual(len(packed), 3)
        self.assertTrue(all(c[4] for c in packed), packed)
        self.assertEqual(records_of(third, vs.HznAbs), [])                # the anchor repeat was spent in frame 2
        self.assertEqual(records_of(third, vs.HznResid), [vs.HznResid(0, 0)])
        decode(enc.frame(img, BUDGET, seq=3))                              # the κ = 25 % carousel frame
        fifth = decode(enc.frame(img, BUDGET, seq=4))
        self.assertEqual(records_of(fifth, vs.Tree), [])                  # repeated once, not forever
        confirmed = {c.base_id + i for c in records_of(fifth, vs.Confirm) for i, t in enumerate(c.tags) if t is not None}
        self.assertEqual(confirmed, ids)

    def test_anchor_repeat_skipped_by_the_budget_is_sent_in_the_next_frame(self):
        # The same rule for the epoch-start repeat of §3.1: at 12 B the body
        # (6 B = 35 record bits) holds STATUS but not the 70-bit anchor +
        # LAYER_CLEAR, so frame 3 carries the repeat and frame 4 the RESID.
        img = scene(blobs=[(20, 44, 4, GREEN), (50, 40, 4, GREEN), (80, 48, 4, GREEN)])
        enc = ev.VectorEncoder()
        decode(enc.frame(img, BUDGET, seq=0))
        second = decode(enc.frame(img, 12, seq=1))
        self.assertEqual(records_of(second, vs.HznAbs) + records_of(second, vs.LayerClear), [])
        self.assertEqual(len(records_of(second, vs.Status)), 1)
        third = decode(enc.frame(img, BUDGET, seq=2))
        self.assertFalse(third.header.key)
        self.assertEqual(len(records_of(third, vs.HznAbs)), 1)
        self.assertEqual(records_of(third, vs.LayerClear), [vs.LayerClear(0)])
        self.assertEqual(len(records_of(third, vs.Tree)), 3)              # the define repeats too
        fourth = decode(enc.frame(img, BUDGET, seq=3))
        self.assertEqual(records_of(fourth, vs.HznAbs), [])
        self.assertEqual(records_of(fourth, vs.HznResid), [vs.HznResid(0, 0)])

    def test_carousel_frame_every_fourth_at_v0(self):
        enc = ev.VectorEncoder()
        sizes = []
        kinds = []
        for i in range(9):
            frame = decode(enc.frame(self.img, BUDGET, seq=i))
            kinds.append(len(records_of(frame, vs.Poly) + records_of(frame, vs.Tree)))
        # frames 0, 1: defines + repeat; 2: floor; 3: carousel; 4-6: floor; 7: carousel
        self.assertEqual(kinds[2], 0)
        self.assertGreater(kinds[3], 0)
        self.assertEqual(kinds[4:7], [0, 0, 0])
        self.assertGreater(kinds[7], 0)

    def test_residual_never_increases_on_a_static_scene(self):
        img = scene(blobs=[(24, 56, 4, GREEN), (50, 40, 4, GREEN), (46, 30, 3, GREEN)],
                    rects=[(4, 34, 30, 44, (200, 60, 60)), (60, 36, 90, 48, (240, 240, 240)),
                           (34, 50, 60, 62, (30, 30, 30)), (70, 52, 92, 58, (200, 200, 40)),
                           (2, 46, 16, 52, (60, 60, 200)), (10, 6, 30, 14, (200, 60, 60))])
        enc = ev.VectorEncoder()
        residuals = []
        for i in range(12):
            decode(enc.frame(img, 60, seq=i))                  # a tight budget spreads the defines
            residuals.append(enc.last_stats["residual"])
        for a, b in zip(residuals, residuals[1:]):
            self.assertLessEqual(b, a + 1e-6, residuals)
        self.assertLess(residuals[-1], residuals[0])

    def test_upd_ucol_and_del(self):
        base = [(20, 44, 4, GREEN), (50, 40, 4, GREEN)]
        enc = ev.VectorEncoder()
        decode(enc.frame(scene(blobs=base), BUDGET))
        decode(enc.frame(scene(blobs=base), BUDGET))
        decode(enc.frame(scene(blobs=base), BUDGET))
        # Move the second blob right by 3 working px (12 canvas px = UPD code 3).
        frame = decode(enc.frame(scene(blobs=[base[0], (53, 40, 4, GREEN)]), BUDGET))
        upds = records_of(frame, vs.Upd)
        self.assertEqual(len(upds), 1)
        self.assertEqual((upds[0].dx, upds[0].dy), (3, 0))
        # Recolour the first blob within the vegetation class: a UCOL, no redefine.
        frame = decode(enc.frame(scene(blobs=[(20, 44, 4, (30, 100, 30)), (53, 40, 4, GREEN)]), BUDGET))
        ucols = records_of(frame, vs.Ucol)
        self.assertEqual(len(ucols), 1)
        self.assertNotEqual(ucols[0].id, upds[0].id)          # the other blob
        self.assertEqual(len(records_of(frame, vs.Tree)), 0)
        self.assertEqual(records_of(frame, vs.Del), [])
        # Recolour it out of the vegetation class: it leaves the plant layer, so DEL + a mass define.
        frame = decode(enc.frame(scene(blobs=[(20, 44, 4, (200, 60, 60)), (53, 40, 4, GREEN)]), BUDGET))
        self.assertEqual(len(records_of(frame, vs.Del)), 1)
        self.assertEqual(len(records_of(frame, vs.Poly)), 1)
        self.assertIn(records_of(frame, vs.Poly)[0].id, vs.ID_MASS)
        # Remove it: a DEL.
        frame = decode(enc.frame(scene(blobs=[(53, 40, 4, GREEN)]), BUDGET))
        dels = records_of(frame, vs.Del)
        self.assertEqual(len(dels), 1)
        self.assertEqual(records_of(frame, vs.Digest)[0].n_live, 1)

    def test_gain_absorbs_a_global_exposure_step_without_a_ucol_flood(self):
        img = scene(blobs=[(24, 56, 4, GREEN), (50, 40, 4, GREEN)],
                    rects=[(4, 34, 30, 44, (200, 60, 60)), (60, 36, 90, 48, (180, 180, 180)),
                           (34, 50, 60, 62, (60, 60, 200))])
        enc = ev.VectorEncoder()
        for i in range(3):
            frame = decode(enc.frame(img, BUDGET, seq=i))
            self.assertEqual(records_of(frame, vs.Gain), [])
        n_live = records_of(frame, vs.Digest)[0].n_live
        self.assertGreaterEqual(n_live, 5)
        dark = np.clip(img.astype(np.float32) * 0.8, 0, 255).astype(np.uint8)
        frame = decode(enc.frame(dark, BUDGET, seq=3))
        gains = records_of(frame, vs.Gain)
        self.assertEqual(len(gains), 1, frame.records)
        # 2^((g − 16) / 32) = 0.8 → g ≈ 5.7; one code is 2.2 %, so allow ±1.
        for code in (gains[0].r, gains[0].g, gains[0].b):
            self.assertLessEqual(abs(code - 6), 1, gains)
        self.assertEqual(records_of(frame, vs.Ucol), [])
        self.assertEqual(records_of(frame, vs.Del), [])
        self.assertFalse(frame.header.key)
        self.assertEqual(records_of(frame, vs.Digest)[0].n_live, n_live)
        # The DIGEST hashes the render state including GAIN; the next frame is a quiet one.
        frame = decode(enc.frame(dark, BUDGET, seq=4))
        self.assertEqual(records_of(frame, vs.Gain), [])
        self.assertEqual(records_of(frame, vs.Ucol), [])
        confirmed = {c.base_id + i for c in records_of(frame, vs.Confirm) for i, t in enumerate(c.tags) if t is not None}
        self.assertGreaterEqual(len(confirmed), n_live - 1)

    def test_force_epoch_and_epoch_start_argument(self):
        enc = ev.VectorEncoder()
        decode(enc.frame(self.img, BUDGET))
        self.assertEqual(enc.epoch, 0)
        enc.force_epoch()
        frame = decode(enc.frame(self.img, BUDGET))
        self.assertTrue(frame.header.key)
        self.assertEqual(enc.epoch, 1)
        self.assertEqual(frame.records[1], vs.LayerClear(0))
        frame = decode(enc.frame(self.img, BUDGET, epoch_start=True))
        self.assertTrue(frame.header.key)
        self.assertEqual(enc.epoch, 2)
        self.assertEqual(frame.header.epoch, 2)

    def test_safety_refresh_after_60_s_keeps_the_masses(self):
        t = [1000.0]
        enc = ev.VectorEncoder(clock=lambda: t[0])
        decode(enc.frame(self.img, BUDGET))
        decode(enc.frame(self.img, BUDGET))
        t[0] += 30.0
        self.assertFalse(decode(enc.frame(self.img, BUDGET)).header.key)
        t[0] += 31.0
        frame = decode(enc.frame(self.img, BUDGET))
        self.assertTrue(frame.header.key)
        self.assertEqual(frame.records[1], vs.LayerClear(2))
        self.assertEqual(records_of(frame, vs.Digest)[0].n_live, 4)   # carried into the new epoch
        self.assertEqual(enc.epoch, 1)

    def test_relabelled_scene_starts_a_new_epoch(self):
        enc = ev.VectorEncoder()
        decode(enc.frame(self.img, BUDGET))
        decode(enc.frame(self.img, BUDGET))
        other = scene(y0=112.0, rects=[(0, 30, 95, 63, (200, 200, 40))])
        frame = decode(enc.frame(other, BUDGET))
        self.assertTrue(frame.header.key)
        self.assertEqual(enc.epoch, 1)

    def test_id_exhaustion_starts_a_new_epoch(self):
        rects = [(2 + 9 * i, 34 + 12 * j, 7 + 9 * i, 40 + 12 * j, (200, 60, 60))
                 for i in range(10) for j in range(2)]
        enc = ev.VectorEncoder()
        first = decode(enc.frame(scene(rects=rects[:10]), BUDGET))
        self.assertEqual(len(records_of(first, vs.Poly)), 10)
        decode(enc.frame(scene(rects=rects[:10]), BUDGET))
        # 10 live + many new → the mass range 1–31 runs out → epoch start. (Two
        # rows of gap between rectangles: the 3×3 mode filter bridges one.)
        many = rects + [(2 + 9 * i, 8 + 7 * j, 7 + 9 * i, 12 + 7 * j, (200, 60, 60))
                        for i in range(10) for j in range(2)]
        frame = decode(enc.frame(scene(y0=8.0, rects=many), BUDGET))
        self.assertTrue(frame.header.key)
        self.assertEqual(enc.epoch, 1)
        ids = [r.id for r in records_of(frame, vs.Poly)]
        self.assertEqual(len(ids), len(set(ids)))
        self.assertTrue(all(i in vs.ID_MASS for i in ids))


@unittest.skipUnless(_HAVE_CV2, "opencv-python-headless not installed")
class MaskTests(unittest.TestCase):
    def test_no_vertex_inside_the_mask_and_bucket_object_appears(self):
        mask = np.zeros((H, W), bool)
        mask[50:, :40] = True                                # hood: bottom-left, touches the bottom edge
        # An object in the bucket zone (unmasked, bottom centre) next to the hood,
        # and a region that runs into the mask.
        img = scene(rects=[(44, 48, 62, 60, (240, 220, 40)), (0, 36, 50, 63, (200, 60, 60))])
        enc = ev.VectorEncoder(mask=mask)
        seen = []
        for i in range(3):
            frame = decode(enc.frame(img, BUDGET, seq=i))
            for p in records_of(frame, vs.Poly):
                for cx, cy in p.vertices:
                    x, y = min(2 * cx, W - 1), min(2 * cy, H - 1)
                    self.assertFalse(mask[y, x], (p, cx, cy))
            seen += records_of(frame, vs.Poly) + records_of(frame, vs.Tree)
        obj = np.zeros((H, W), bool)
        obj[48:61, 44:63] = True
        covered = False
        for rec in seen:
            ras = vx.raster_of(rec)
            if (ras & obj).sum() >= 0.5 * obj.sum():
                colour = vx.fill_shown_rgb8(rec.fill)
                covered = covered or float(np.abs(colour - np.array((240, 220, 40))).max()) < 20
        self.assertTrue(covered, seen)
        red = [r for r in seen if isinstance(r, vs.Poly) and r.fill.rgb444 is not None
               and vx.rgb444_to_rgb8(r.fill.rgb444)[0] > 150 and vx.rgb444_to_rgb8(r.fill.rgb444)[1] < 100]
        self.assertTrue(red, seen)                          # the region beside the hood still appears

    def test_masked_pixels_are_never_sampled(self):
        mask = np.zeros((H, W), bool)
        mask[44:, 20:76] = True
        img = scene()
        img[44:, 20:76] = (255, 0, 255)                     # loud colour under the mask
        enc = ev.VectorEncoder(mask=mask)
        frame = decode(enc.frame(img, BUDGET))
        self.assertEqual(records_of(frame, vs.Poly) + records_of(frame, vs.Tree), [])
        gnd = vx.fill_shown_rgb8(records_of(frame, vs.HznAbs)[0].gnd)
        self.assertLess(float(np.abs(gnd - np.array(GROUND)).max()), 9.0)

    def test_mask_shape_is_checked(self):
        with self.assertRaises(ValueError):
            ev.VectorEncoder(mask=np.zeros((10, 10), bool))


if __name__ == "__main__":
    unittest.main()
