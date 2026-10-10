"""Run the new A8 tests against the OLD relabel rule (194fa5b8): they must fail.
Run from DC/base_station."""
import sys
import unittest

sys.path[:0] = ['.', 'tests', '../firmware/tractor_x8']
import numpy as np  # noqa: E402
import test_vector_sync as T  # noqa: E402
from x8_image_pipeline import encode_vector as ev  # noqa: E402
vx = ev.vx


def old(prev, prev_lab, cur, regions):
    np_, nc = int(prev.max()) + 2, int(cur.max()) + 2
    pair = np.bincount((prev.ravel() + 1) * nc + (cur.ravel() + 1), minlength=np_ * nc).reshape(np_, nc)
    area_c = pair.sum(axis=0)
    area_p = pair.sum(axis=1)
    total = float(area_c[1:].sum())
    if total <= 0 or np_ < 2:
        return 1.0 if total > 0 else 0.0
    rel = 0.0
    for r in regions:
        c = r.index + 1
        if c >= nc or area_c[c] == 0:
            continue
        p = int(np.argmax(pair[1:, c])) + 1
        inter = pair[p, c]
        union = area_c[c] + area_p[p] - inter
        if union <= 0 or inter / union < ev.MATCH_IOU or float(vx.delta_e76(r.lab, prev_lab[p - 1])) > ev.VERIFY_DE:
            rel += float(area_c[c])
    return rel / total


ev.VectorEncoder._relabelled_fraction = staticmethod(old)
ev.RELABEL_EPOCH_FRACTION = 0.40
names = ['test_camera_change_in_a_sky_heavy_view_starts_an_epoch_and_noise_does_not',
         'test_a_mass_that_splits_or_merges_keeps_the_epoch', 'test_noisy_pan_keeps_the_epoch',
         'test_relabelled_fraction_counts_pixels_not_regions',
         'test_static_noisy_scene_stays_in_step_and_never_churns', 'test_epoch_trigger_is_reported']
suite = unittest.TestSuite(T.SyncTests(n) for n in names)
r = unittest.TextTestRunner(verbosity=1).run(suite)
for t, tb in r.failures + r.errors:
    print('FAILED on the old rule:', t.id().split('.')[-1], '|', tb.strip().splitlines()[-1][:170])
