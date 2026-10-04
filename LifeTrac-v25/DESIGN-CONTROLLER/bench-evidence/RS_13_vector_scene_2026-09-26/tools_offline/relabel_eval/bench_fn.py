"""Cost of the relabel fraction, old (per region) vs new (per pixel), on a photo frame."""
import time

import numpy as np

from rl_common import view, ev

vx = ev.vx
new = ev.VectorEncoder._relabelled_fraction


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


cap = {}


class E(ev.VectorEncoder):
    def _epoch_trigger(self, es, hz, rm, regions, now):
        if self._prev_region_map is not None:
            cap["a"] = (self._prev_region_map, self._prev_region_lab, rm, regions)
        return super()._epoch_trigger(es, hz, rm, regions, now)


e = E(clock=lambda: 0.0)
for i in range(3):
    e.frame(view(29, 1920, 1300, 1800, 12, i), 203)
a = cap["a"]
for nm, f in (("old", old), ("new", new)):
    t = time.perf_counter()
    for _ in range(2000):
        v = f(*a)
    print(nm, "%.3f ms" % ((time.perf_counter() - t) / 2000 * 1000), "value %.2f" % v, "regions", len(a[3]))
