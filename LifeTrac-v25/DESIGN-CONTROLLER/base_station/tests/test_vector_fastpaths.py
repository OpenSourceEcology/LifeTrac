"""Bit-exactness of the tractor VS1 encoder's fast paths.

The encoder (``x8_image_pipeline/encode_vector.py`` + ``vector_extract.py``)
was sped up without changing a payload byte: every fast path computes the
very numbers the straightforward formulation computes — per-region work on
bounding-box crops, batched colour conversion, label-map painting, memoised
codec queries, an incremental Visvalingam–Whyatt loop. Each test below pins
one such equivalence against that formulation (written out here as the
encoder had it), on the numpy / OpenCV build the tests run on, so a library
change that broke one is caught as a failing test rather than as silently
different wire bytes.

Needs cv2 (``opencv-python-headless``); skipped where it is absent.
"""
from __future__ import annotations

import os
import sys
import unittest
from types import SimpleNamespace

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_BS_DIR = os.path.dirname(_THIS_DIR)
_X8_DIR = os.path.normpath(os.path.join(_THIS_DIR, "..", "..", "firmware", "tractor_x8"))
for _d in (_BS_DIR, _X8_DIR):
    if _d not in sys.path:
        sys.path.insert(0, _d)

try:                                             # pragma: no cover
    import numpy as np
    import cv2
    _HAVE_CV2 = True
except ImportError:                              # pragma: no cover
    np = None
    _HAVE_CV2 = False

if _HAVE_CV2:
    from x8_image_pipeline import encode_vector as ev  # noqa: E402
    from x8_image_pipeline import vector_extract as vx  # noqa: E402


def same_bits(a, b) -> bool:
    a, b = np.asarray(a), np.asarray(b)
    return a.dtype == b.dtype and a.shape == b.shape and a.tobytes() == b.tobytes()


def blobs(shape, seed, sigma=2.0):
    rng = np.random.default_rng(seed)
    return (cv2.GaussianBlur(rng.random(shape).astype(np.float32), (0, 0), sigma) > 0.5).astype(np.uint8)


def vw_reference(pts, max_n):
    """The closed-ring Visvalingam–Whyatt loop as the encoder had it."""
    pts = [tuple(p) for p in pts]
    while len(pts) > max_n:
        n = len(pts)
        best, best_area = 0, None
        for i in range(n):
            (x0, y0), (x1, y1), (x2, y2) = pts[i - 1], pts[i], pts[(i + 1) % n]
            area = abs((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0))
            if best_area is None or area < best_area:
                best, best_area = i, area
        del pts[best]
    return np.array(pts, dtype=np.float32)


def open_vw_reference(pts, max_n):
    pts = [tuple(p) for p in pts]
    while len(pts) > max_n:
        best, best_area = 1, None
        for i in range(1, len(pts) - 1):
            (x0, y0), (x1, y1), (x2, y2) = pts[i - 1], pts[i], pts[i + 1]
            area = abs((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0))
            if best_area is None or area < best_area:
                best, best_area = i, area
        del pts[best]
    return np.array(pts, dtype=np.float32)


@unittest.skipUnless(_HAVE_CV2, "needs numpy + OpenCV")
class FastPathTests(unittest.TestCase):

    def test_squared_distance_is_the_three_term_reduction(self):
        rng = np.random.default_rng(1)
        for n, k in ((5000, 8), (37, 3), (1, 1)):
            data = (rng.random((n, 3), dtype=np.float32) * np.float32(120) - np.float32(20)).astype(np.float32)
            centres = (rng.random((k, 3), dtype=np.float32) * np.float32(120) - np.float32(20)).astype(np.float32)
            reference = ((data[:, None, :] - centres[None, :, :]) ** 2).sum(axis=2)
            self.assertTrue(same_bits(vx._sq_dist(data, centres), reference), (n, k))

    def test_lab_lookup_is_the_formula(self):
        rng = np.random.default_rng(2)
        img = rng.integers(0, 256, (64, 96, 3)).astype(np.uint8)
        img[0, :, :] = np.arange(96)[:, None] * 2 + np.array([0, 1, 2])        # every channel value appears
        img[1, :64, :] = (np.arange(64)[:, None] * 4 + np.array([1, 2, 3])) % 256
        self.assertTrue(same_bits(vx.rgb_to_lab(img), vx.rgb_to_lab(img.astype(np.float32))))

    def test_batched_colours_are_the_per_mean_answers(self):
        rng = np.random.default_rng(3)
        means = (rng.random((3000, 3), dtype=np.float32) * np.float32(255)).astype(np.float32)
        means[:8] = vx.PALETTE_RGB8                                           # palette slots exactly
        means[8:16] = vx.PALETTE_RGB8 + np.float32(2.5)                       # ... and near them
        labs = vx.rgb_to_lab(means)
        grads = [None if i % 3 else (i % 8, i % 7 - 4) for i in range(len(means))]
        fills = vx.make_fills(means, grads, labs)
        for i, (slot_code, fill) in enumerate(zip(vx.choose_colours(means), fills)):
            self.assertTrue(same_bits(labs[i], vx.rgb_to_lab(means[i])), i)
            slot, code, _ = vx.choose_colour(means[i])
            self.assertEqual(slot_code, (slot, code), i)
            self.assertEqual(fill, vx.make_fill(means[i], grads[i]), i)

    def test_region_measurements_on_crops_are_the_whole_image_ones(self):
        # A noisy scene with many regions: each region's mean, centroid,
        # fill and lab, measured on its crop, against the whole-image formulas.
        rng = np.random.default_rng(4)
        rgb = np.zeros((64, 96, 3), np.uint8)
        for c in range(3):
            rgb[..., c] = blobs((64, 96), 10 + c, 3.0) * 120 + 40
        rgb = np.clip(rgb.astype(np.int16) + rng.integers(-12, 13, rgb.shape), 0, 255).astype(np.uint8)
        valid = np.ones((64, 96), bool)
        regions, region_map = vx.extract_regions(rgb, vx.rgb_to_lab(rgb), valid, None, vx.Segmenter(),
                                                 vx.Horizon(False))
        self.assertGreater(len(regions), 5)
        f = rgb.astype(np.float32)
        lum = vx.luma(f)
        yy, xx = np.mgrid[0:64, 0:96]
        xx, yy = xx.astype(np.float32), yy.astype(np.float32)
        for r in regions:
            comp = region_map == r.index
            self.assertEqual(int(comp.sum()), r.area)
            mean = f[comp].mean(axis=0)
            cx, cy = float(xx[comp].mean()), float(yy[comp].mean())
            self.assertTrue(same_bits(r.mean, mean.astype(np.float32)), r.index)
            self.assertEqual((r.centroid[0], r.centroid[1]), (cx, cy))
            a = np.stack([np.ones(r.area, np.float32), xx[comp] - cx, yy[comp] - cy], axis=1)
            coef, *_ = np.linalg.lstsq(a, lum[comp], rcond=None)
            self.assertEqual(r.fill, vx.make_fill(mean, vx.gradient_of(tuple(float(c) for c in coef), r.bbox)))
            self.assertTrue(same_bits(r.lab, vx.rgb_to_lab(mean)), r.index)

    def test_contours_on_crops_are_the_whole_image_contours(self):
        hood = np.zeros((64, 96), bool)
        hood[56:, 20:76] = True
        for seed in range(12):
            img = blobs((64, 96), seed, 1.5 + seed % 3)
            n, cc, stats, _ = cv2.connectedComponentsWithStats(img, connectivity=4)
            for i in range(1, n):
                piece, origin = vx._crop(cc, i, stats[i])
                for mask in (None, hood):
                    self.assertEqual(vx.polygon_of(piece, mask, origin), vx.polygon_of(cc == i, mask), (seed, i))
            edges = cv2.dilate(cv2.Canny((blobs((128, 192), 100 + seed, 2.0) * 255).astype(np.uint8), 50, 100),
                               np.ones((3, 3), np.uint8))
            n, cc, stats, _ = cv2.connectedComponentsWithStats((edges > 0).astype(np.uint8), connectivity=8)
            for i in range(1, n):
                piece, origin = vx._crop(cc, i, stats[i])
                whole = vx.polyline_of_component(cc == i)
                crop = vx.polyline_of_component(piece, origin, cc.shape)
                self.assertEqual(whole is None, crop is None, (seed, i))
                if whole is not None:
                    self.assertTrue(same_bits(whole, crop), (seed, i))

    def test_visvalingam_whyatt_matches_the_quadratic_loop(self):
        rng = np.random.default_rng(5)
        for t in range(300):
            n = int(rng.integers(11, 41))
            ring = rng.integers(0, 96, (n, 2)).astype(np.float32)
            if t % 4 == 0:
                ring[:, 0] = np.sort(ring[:, 0])                              # many equal areas: ties
                ring[::2, 1] = ring[0, 1]
            if t % 5 == 0:
                ring += np.float32(0.25)                                      # off the grid: the float32 path
            self.assertTrue(same_bits(vx._vw_simplify(ring, 10), vw_reference(ring, 10)), t)
            self.assertTrue(same_bits(vx._open_polyline_vw(ring, 5), open_vw_reference(ring, 5)), t)

    def test_painting_is_one_assignment_per_shape(self):
        rng = np.random.default_rng(6)
        base = (rng.random((64, 96, 3), dtype=np.float32) * np.float32(255)).astype(np.float32)
        gain = ev.VectorEncoder._gain_lin((14, 17, 20))
        shapes = []
        for k in range(30):
            raster = np.zeros((64, 96), bool)
            x0, y0 = int(rng.integers(0, 80)), int(rng.integers(0, 50))
            raster[y0:y0 + int(rng.integers(2, 20)), x0:x0 + int(rng.integers(2, 30))] = True
            fill = ev.vs.Fill(rgb444=int(rng.integers(0, 4096)))
            shapes.append(SimpleNamespace(id=k + 1, area=int(raster.sum()) if k % 7 else 40, raster=raster,
                                          shown=vx.fill_shown_rgb8(fill)))
        paint = ev._Painting(base, shapes, gain)
        order = sorted(shapes, key=lambda s: -s.area)

        def picture(skip=None):
            pic = base.copy()
            for s in order:
                if s.id != skip:
                    pic[s.raster] = np.clip(s.shown * gain, 0.0, 255.0)
            return pic

        self.assertTrue(same_bits(paint.picture, picture()))
        ys, xs = np.nonzero(np.ones((64, 96), bool))
        for s in shapes[::3]:
            self.assertTrue(same_bits(paint.under(s.id, ys, xs), picture(skip=s.id)[ys, xs]), s.id)
        top_only = ev._Painting(base, shapes, gain, with_second=False)
        self.assertTrue(same_bits(top_only.picture, paint.picture))

    def test_memos_return_the_codec_answers(self):
        cvs = ev.vs
        fill = cvs.Fill(rgb444=0x5A3, grad=(2, -1))
        records = [cvs.Poly(3, 0, ((10, 5), (14, 6), (16, 9), (15, 13), (11, 14)), fill),
                   cvs.Poly(3, 0, ((10, 5), (14, 6), (16, 9), (15, 13), (11, 14)), cvs.Fill(palette=2)),
                   cvs.Tree(33, 10, 12, 3, 4, fill), cvs.Upd(3, -2, 1), cvs.Ucol(3, fill), cvs.Del(9),
                   cvs.Edge(57, 1, ((3, 4), (9, 6), (14, 6))), cvs.Confirm(1, (0, None, 3)), cvs.Digest(4, 0x7E)]
        for _ in range(2):                                                    # cold, then from the memo
            for rec in records:
                self.assertEqual(ev._record_bits(rec), cvs.record_bits(rec), rec)
                if isinstance(rec, (cvs.Poly, cvs.Tree, cvs.Edge)):
                    self.assertEqual(ev._define_hash(rec), cvs.define_hash(rec), rec)
            for dx, dy, base in ((0, 0, 0x123), (-3, 2, 0xFFF), (7, -8, 0)):
                self.assertEqual(ev._state_hash(dx, dy, base), cvs.state_hash(dx, dy, base, [], []))
            shapes = ((1, 0x1234, 0x0F0F), (5, 0xBEEF, 0x0001), (33, 0x0042, 0xA5A5))
            self.assertEqual(ev._digest_crc(shapes, (16, 18, 15)),
                             cvs.digest_crc(list(shapes), [(0, 0)] * 4, 128, (16, 18, 15)))


if __name__ == "__main__":
    unittest.main()
