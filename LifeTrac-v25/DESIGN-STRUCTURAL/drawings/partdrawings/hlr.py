"""Hidden-line removal and feature detection for orthographic part views.

A view is defined by a unit vector ``d`` pointing from the part toward the
viewer and a screen basis (u right, v up).  Views follow third-angle
projection (ASME Y14.3): the top view sits above the front view and the right
side view sits to its right.

The line drawing for a view is built from
  * sharp edges  - dihedral angle above a threshold (plate edges, hole rims)
  * silhouettes  - edges between a face turned toward the viewer and one
                   turned away (the outline of round bars, bolt shanks ...)
Each candidate edge is sampled; a sample is hidden when some other triangle
covers it in 2D and lies closer to the viewer.
"""

import math
from dataclasses import dataclass, field

import numpy as np

from .mesh import edge_loops

SQ2, SQ3, SQ6 = math.sqrt(2), math.sqrt(3), math.sqrt(6)

VIEWS = {
    #         toward viewer        screen right            screen up
    "front": ((0, 0, 1), (1, 0, 0), (0, 1, 0)),
    "top": ((0, 1, 0), (1, 0, 0), (0, 0, -1)),
    "right": ((1, 0, 0), (0, 0, -1), (0, 1, 0)),
    "iso": (
        (1 / SQ3, 1 / SQ3, 1 / SQ3),
        (1 / SQ2, 0, -1 / SQ2),
        (-1 / SQ6, 2 / SQ6, -1 / SQ6),
    ),
}


@dataclass
class ViewDrawing:
    name: str
    segments: np.ndarray  # (M, 2, 2) line segments in view coordinates (mm)
    visible: np.ndarray  # (M,) bool
    lo: np.ndarray  # (2,) min corner of the part outline in this view
    hi: np.ndarray  # (2,) max corner
    circles: list = field(default_factory=list)  # detected hole/boss circles

    @property
    def size(self):
        return self.hi - self.lo


def view_basis(name):
    d, u, v = (np.asarray(x, dtype=float) for x in VIEWS[name])
    return d, u, v


def _candidate_edges(mesh, d, sharp):
    n0, n1 = mesh.edge_n
    known = (np.abs(n0).sum(axis=1) > 0) & (np.abs(n1).sum(axis=1) > 0)
    eps = 1e-6
    front0, front1 = (n0 @ d) > eps, (n1 @ d) > eps
    silhouette = known & (mesh.edge_faces[:, 1] >= 0) & (front0 != front1)
    return sharp | silhouette


def merge_segments(segs, vis, tol=1e-4):
    """Merge collinear, overlapping segments into maximal lines.

    Makes the drawing independent of how the surface happened to be
    triangulated, keeps dash patterns continuous, and drops hidden (dashed)
    lines that lie under visible ones.
    """
    if len(segs) == 0:
        return segs, vis
    p0, p1 = segs[:, 0], segs[:, 1]
    dvec = p1 - p0
    length = np.linalg.norm(dvec, axis=1)
    keep = length > tol
    p0, p1, dvec, length, vis = p0[keep], p1[keep], dvec[keep], length[keep], vis[keep]
    u = dvec / length[:, None]
    flip = (u[:, 0] < -1e-12) | ((np.abs(u[:, 0]) <= 1e-12) & (u[:, 1] < 0))
    u[flip] *= -1
    normal = np.stack([-u[:, 1], u[:, 0]], axis=1)
    offset = np.einsum("ij,ij->i", normal, p0)
    ang = np.arctan2(u[:, 1], u[:, 0])
    t0 = np.einsum("ij,ij->i", u, p0)
    t1 = np.einsum("ij,ij->i", u, p1)
    lo, hi = np.minimum(t0, t1), np.maximum(t0, t1)

    # Group segments lying on the same infinite line.
    order = np.lexsort((offset, np.round(ang / 1e-6)))
    groups, cur = [], [order[0]]
    for i in order[1:]:
        j = cur[-1]
        if abs(ang[i] - ang[j]) < 1e-6 and abs(offset[i] - offset[j]) < tol * 10:
            cur.append(i)
        else:
            groups.append(cur)
            cur = [i]
    groups.append(cur)

    def union(intervals):
        out = []
        for a, b in sorted(intervals):
            if out and a <= out[-1][1] + tol:
                out[-1][1] = max(out[-1][1], b)
            else:
                out.append([a, b])
        return out

    def subtract(intervals, cut):
        out = []
        for a, b in intervals:
            pieces = [[a, b]]
            for c0, c1 in cut:
                nxt = []
                for x0, x1 in pieces:
                    if c1 <= x0 + tol or c0 >= x1 - tol:
                        nxt.append([x0, x1])
                        continue
                    if c0 > x0 + tol:
                        nxt.append([x0, c0])
                    if c1 < x1 - tol:
                        nxt.append([c1, x1])
                pieces = nxt
            out += pieces
        return out

    out_segs, out_vis = [], []
    for g in groups:
        g = np.asarray(g)
        uu, nn, off = u[g[0]], normal[g[0]], float(np.mean(offset[g]))
        visible = union([(lo[i], hi[i]) for i in g if vis[i]])
        hidden = subtract(union([(lo[i], hi[i]) for i in g if not vis[i]]), visible)
        for flag, ivals in ((True, visible), (False, hidden)):
            for a, b in ivals:
                if b - a > tol:
                    out_segs.append((nn * off + uu * a, nn * off + uu * b))
                    out_vis.append(flag)
    segs = np.round(np.asarray(out_segs, dtype=float).reshape(-1, 2, 2), 6) + 0.0
    vis = np.asarray(out_vis, dtype=bool)
    order = np.lexsort((segs[:, 1, 1], segs[:, 1, 0], segs[:, 0, 1], segs[:, 0, 0], vis))
    return segs[order], vis[order]


def _occluded(tri2d, tdep, pts, pdep, excl, eps):
    """Return a bool array: is each sample point covered by a nearer triangle?"""
    P = len(pts)
    out = np.zeros(P, dtype=bool)
    if P == 0 or len(tri2d) == 0:
        return out

    a, b, c = tri2d[:, 0], tri2d[:, 1], tri2d[:, 2]
    den = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (c[:, 0] - a[:, 0]) * (b[:, 1] - a[:, 1])
    scale = np.abs(tri2d).max() + 1.0
    keep = np.abs(den) > 1e-12 * scale * scale  # skip triangles seen edge-on
    tid_all = np.flatnonzero(keep)
    if len(tid_all) == 0:
        return out

    lo = np.minimum(pts.min(axis=0), tri2d[keep].reshape(-1, 2).min(axis=0)) - 1e-6
    hi = np.maximum(pts.max(axis=0), tri2d[keep].reshape(-1, 2).max(axis=0)) + 1e-6
    G = int(np.clip(math.sqrt(len(tid_all)) / 1.5, 8, 160))
    cell = (hi - lo) / G

    tmin = tri2d[tid_all].min(axis=1)
    tmax = tri2d[tid_all].max(axis=1)
    ix0 = np.clip(((tmin[:, 0] - lo[0]) / cell[0]).astype(int), 0, G - 1)
    iy0 = np.clip(((tmin[:, 1] - lo[1]) / cell[1]).astype(int), 0, G - 1)
    ix1 = np.clip(((tmax[:, 0] - lo[0]) / cell[0]).astype(int), 0, G - 1)
    iy1 = np.clip(((tmax[:, 1] - lo[1]) / cell[1]).astype(int), 0, G - 1)
    w = ix1 - ix0 + 1
    h = iy1 - iy0 + 1
    n = w * h
    rep = np.repeat(np.arange(len(tid_all)), n)
    k = np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n)
    cx = ix0[rep] + k % w[rep]
    cy = iy0[rep] + k // w[rep]
    tcell = cy * G + cx
    order = np.argsort(tcell, kind="stable")
    tcell, ttri = tcell[order], tid_all[rep[order]]
    starts = np.searchsorted(tcell, np.arange(G * G))
    ends = np.searchsorted(tcell, np.arange(G * G), side="right")

    pcx = np.clip(((pts[:, 0] - lo[0]) / cell[0]).astype(int), 0, G - 1)
    pcy = np.clip(((pts[:, 1] - lo[1]) / cell[1]).astype(int), 0, G - 1)
    pcell = pcy * G + pcx
    porder = np.argsort(pcell, kind="stable")
    pcell_sorted = pcell[porder]
    uniq, pstart = np.unique(pcell_sorted, return_index=True)
    pend = np.append(pstart[1:], len(porder))

    tol = 1e-7
    for cid, s, e in zip(uniq, pstart, pend):
        tris = ttri[starts[cid]:ends[cid]]
        if len(tris) == 0:
            continue
        pidx = porder[s:e]
        # Chunk to bound memory on dense cells.
        step = max(1, 400000 // max(1, len(tris)))
        for j in range(0, len(pidx), step):
            pi = pidx[j:j + step]
            p = pts[pi][:, None, :]
            A, B, C = a[tris][None], b[tris][None], c[tris][None]
            dn = den[tris][None]
            l1 = ((p[..., 0] - A[..., 0]) * (C[..., 1] - A[..., 1]) - (C[..., 0] - A[..., 0]) * (p[..., 1] - A[..., 1])) / dn
            l2 = ((B[..., 0] - A[..., 0]) * (p[..., 1] - A[..., 1]) - (p[..., 0] - A[..., 0]) * (B[..., 1] - A[..., 1])) / dn
            l0 = 1.0 - l1 - l2
            inside = (l0 > tol) & (l1 > tol) & (l2 > tol)
            td = tdep[tris]
            depth = l0 * td[None, :, 0] + l1 * td[None, :, 1] + l2 * td[None, :, 2]
            nearer = depth > (pdep[pi][:, None] + eps)
            own = (tris[None, :] == excl[pi, 0][:, None]) | (tris[None, :] == excl[pi, 1][:, None])
            out[pi] = np.any(inside & nearer & ~own, axis=1)
    return out


def draw_view(mesh, name, sharp_angle=30.0, samples=400):
    """Compute the visible/hidden line drawing of ``mesh`` for view ``name``."""
    d, u, v = view_basis(name)
    V = mesh.V
    uv = np.stack([V @ u, V @ v], axis=1)
    dep = V @ d
    lo, hi = uv.min(axis=0), uv.max(axis=0)
    size = float(max(hi - lo)) or 1.0

    sharp = mesh.sharp_edges(sharp_angle)
    cand = np.flatnonzero(_candidate_edges(mesh, d, sharp))
    E = mesh.edges[cand]
    p0, p1 = uv[E[:, 0]], uv[E[:, 1]]
    L = np.linalg.norm(p1 - p0, axis=1)
    ok = L > size * 1e-6  # edges parallel to the view direction collapse to points
    cand, E, p0, p1, L = cand[ok], E[ok], p0[ok], p1[ok], L[ok]

    step = size / samples
    k = np.maximum(1, np.ceil(L / step).astype(int))
    eid = np.repeat(np.arange(len(E)), k)
    j = np.arange(k.sum()) - np.repeat(np.cumsum(k) - k, k)
    t0 = j / k[eid]
    t1 = (j + 1) / k[eid]
    tm = (t0 + t1) / 2
    d0, d1 = dep[E[:, 0]], dep[E[:, 1]]
    pts = p0[eid] + (p1[eid] - p0[eid]) * tm[:, None]
    pdep = d0[eid] + (d1[eid] - d0[eid]) * tm
    excl = mesh.edge_faces[cand][eid]

    tri2d = uv[mesh.F]
    tdep = dep[mesh.F]
    hidden = _occluded(tri2d, tdep, pts, pdep, excl, eps=size * 1e-5 + 1e-6)

    # Merge runs of equal visibility along each edge into single segments.
    segs, vis = [], []
    brk = np.ones(len(eid), dtype=bool)
    brk[1:] = (eid[1:] != eid[:-1]) | (hidden[1:] != hidden[:-1])
    starts = np.flatnonzero(brk)
    ends = np.append(starts[1:], len(eid))
    for s, e in zip(starts, ends):
        i = eid[s]
        a = p0[i] + (p1[i] - p0[i]) * t0[s]
        b = p0[i] + (p1[i] - p0[i]) * t1[e - 1]
        segs.append((a, b))
        vis.append(not hidden[s])
    segs = np.asarray(segs, dtype=float).reshape(-1, 2, 2)
    vis = np.asarray(vis, dtype=bool)
    segs, vis = merge_segments(segs, vis, tol=size * 1e-6 + 1e-6)

    circles = detect_circles(mesh, name, sharp) if name != "iso" else []
    return ViewDrawing(name, segs, vis, lo, hi, circles)


# ----------------------------------------------------------------------------
# Hole / boss detection
# ----------------------------------------------------------------------------

def detect_circles(mesh, name, sharp=None, min_vertices=8):
    """Find sharp-edge loops that project to circles in this view: holes and
    bosses whose axis points at the viewer.  The rim of a cross hole in a
    pin is not flat, but it still projects to a circle along the hole axis.

    Returns a list of dicts ``{u, v, r, depths, hole}`` - one per distinct
    circle, with every rim depth at which it occurs (entry and exit rims of a
    through-hole, both walls of a tube ...).
    """
    d, u, v = view_basis(name)
    if sharp is None:
        sharp = mesh.sharp_edges()
    faces = mesh.edge_faces[sharp]
    V = mesh.V
    found = []
    for vidx, eidx in edge_loops(mesh, sharp, min_vertices):
        P = np.stack([V[vidx] @ u, V[vidx] @ v], axis=1)
        dep = V[vidx] @ d
        # Algebraic (Kasa) circle fit.
        A = np.column_stack([2 * P, np.ones(len(P))])
        rhs = (P ** 2).sum(axis=1)
        (cx, cy, k), *_ = np.linalg.lstsq(A, rhs, rcond=None)
        r = math.sqrt(max(k + cx * cx + cy * cy, 0.0))
        if r <= 1e-6:
            continue
        resid = np.abs(np.hypot(P[:, 0] - cx, P[:, 1] - cy) - r)
        if resid.max() > 0.02 * r + 1e-4:
            continue  # an ellipse, a slot, or a loop seen from the side
        # Vertices must be spread evenly round the circle: a rounded rectangle
        # whose only vertices sit on its corner arcs also fits a circle.
        ang = np.sort(np.arctan2(P[:, 1] - cy, P[:, 0] - cx))
        gaps = np.diff(np.append(ang, ang[0] + 2 * np.pi))
        if gaps.max() > np.radians(50):
            continue
        # Hole or boss?  Look at the planar face on the rim.  Around a hole
        # that face lies outside the circle; on a boss, disc or washer OD it
        # lies inside (a cap triangulated only between rim vertices is inside too).
        is_hole = True
        for fa in faces[eidx].ravel():
            if fa < 0 or abs(mesh.N[fa] @ d) < 0.999:
                continue
            fv = V[mesh.F[fa]]
            dist = np.hypot(fv @ u - cx, fv @ v - cy)
            is_hole = not (dist < r * 0.999).any() and bool((dist > r * 1.001).any())
            break
        found.append({"u": cx, "v": cy, "r": r, "depth": float(dep.mean()), "hole": is_hole})

    size = float(max(np.ptp(np.stack([V @ u, V @ v], axis=1), axis=0).max(), 1e-9))
    # Merge rims that share a centre and radius.
    merged = []
    for f in sorted(found, key=lambda f: (round(f["u"], 2), round(f["v"], 2), f["depth"])):
        # Rims on a curved wall pick up extra vertices on the polygon's chords,
        # so their fitted centres wander by a few percent of the radius.
        tol = max(0.05 * f["r"], 1e-3 * size, 0.05)
        for m in merged:
            if abs(m["u"] - f["u"]) < tol and abs(m["v"] - f["v"]) < tol and abs(m["r"] - f["r"]) < tol:
                m["depths"].append(f["depth"])
                m["hole"] = m["hole"] or f["hole"]
                break
        else:
            merged.append({"u": f["u"], "v": f["v"], "r": f["r"], "depths": [f["depth"]], "hole": f["hole"]})
    for m in merged:
        m["depths"].sort()
    return merged
