"""Triangle-mesh helpers: STL loading, vertex welding, edge adjacency,
canonical part orientation and mass properties.

Everything here is plain numpy so the generator runs anywhere OpenSCAD does.
"""

import struct

import numpy as np


def load_stl(path):
    """Return an (N, 3, 3) float array of triangles from an ASCII or binary STL."""
    with open(path, "rb") as fh:
        data = fh.read()
    if data[:5].lower() == b"solid" and b"facet" in data[:1024]:
        return _load_ascii_stl(data.decode("ascii", errors="replace"))
    return _load_binary_stl(data)


def _load_ascii_stl(text):
    verts = []
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("vertex"):
            _, x, y, z = line.split()
            verts.append((float(x), float(y), float(z)))
    if len(verts) % 3:
        raise ValueError("ASCII STL vertex count is not a multiple of 3")
    return np.asarray(verts, dtype=float).reshape(-1, 3, 3)


def _load_binary_stl(data):
    (count,) = struct.unpack_from("<I", data, 80)
    rec = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    arr = np.frombuffer(data, dtype=rec, count=count, offset=84)
    return arr["v"].astype(float)


class Mesh:
    """Welded, indexed triangle mesh with per-edge face adjacency."""

    def __init__(self, tris, weld_tol=1e-4):
        tris = np.asarray(tris, dtype=float).reshape(-1, 3, 3)
        flat = tris.reshape(-1, 3)
        keys = np.round(flat / weld_tol).astype(np.int64)
        _, first, inverse = np.unique(keys, axis=0, return_index=True, return_inverse=True)
        self.V = flat[first]
        F = inverse.reshape(-1, 3)
        # Drop triangles that collapsed during welding.
        ok = (F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2])
        F = F[ok]
        # OpenSCAD lists the same triangles in a different order from run to
        # run; put them in a canonical order (winding preserved) so drawings
        # are byte-for-byte reproducible.
        roll = np.argmin(F, axis=1)
        idx = (np.arange(3)[None, :] + roll[:, None]) % 3
        F = np.take_along_axis(F, idx, axis=1)
        self.F = F[np.lexsort((F[:, 2], F[:, 1], F[:, 0]))]
        self._build()

    def _build(self):
        V, F = self.V, self.F
        a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
        cross = np.cross(b - a, c - a)
        area2 = np.linalg.norm(cross, axis=1)
        self.face_area = 0.5 * area2
        self.N = cross / np.where(area2 > 0, area2, 1.0)[:, None]

        # Edge -> adjacent faces.  Each face contributes three undirected edges.
        e = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
        fid = np.tile(np.arange(len(F)), 3)
        e.sort(axis=1)
        order = np.lexsort((e[:, 1], e[:, 0]))
        e, fid = e[order], fid[order]
        new = np.ones(len(e), dtype=bool)
        new[1:] = np.any(e[1:] != e[:-1], axis=1)
        starts = np.flatnonzero(new)
        counts = np.diff(np.append(starts, len(e)))
        self.edges = e[starts]
        self.edge_nfaces = counts
        # Faces for manifold edges (count == 2); -1 otherwise.
        f0 = fid[starts]
        f1 = np.where(counts >= 2, fid[np.minimum(starts + 1, len(fid) - 1)], -1)
        self.edge_faces = np.stack([f0, f1], axis=1)
        self._edge_normals()

    def _edge_normals(self):
        """Normals of the two faces on each edge, looking through zero-area
        sliver triangles.

        CGAL sometimes stitches a T-junction with a degenerate triangle whose
        three vertices are collinear.  Its normal is meaningless, and which
        slivers appear varies from run to run.  For an edge of a sliver we use
        the real face on the far side of the sliver instead, so sharp-edge and
        silhouette tests depend only on the real surface.
        """
        F, N = self.F, self.N
        f0, f1 = self.edge_faces[:, 0], self.edge_faces[:, 1]
        n0 = N[f0].copy()
        n1 = np.where((f1 >= 0)[:, None], N[np.maximum(f1, 0)], 0.0)
        area2 = 2 * self.face_area
        scale = float(np.abs(self.V).max()) + 1.0 if len(self.V) else 1.0
        degenerate = area2 <= 1e-10 * scale * scale
        if degenerate.any():
            index = {(int(a), int(b)): i for i, (a, b) in enumerate(self.edges)}

            def edge_of(a, b):
                return index[(min(a, b), max(a, b))]

            def across(e, face):
                a, b = self.edge_faces[e]
                other = b if a == face else a
                return other if other >= 0 and not degenerate[other] else -1

            for d in np.flatnonzero(degenerate):
                tri = [int(x) for x in F[d]]
                lens = [np.linalg.norm(self.V[tri[k]] - self.V[tri[(k + 1) % 3]]) for k in range(3)]
                k = int(np.argmax(lens))
                a, b, c = tri[k], tri[(k + 1) % 3], tri[(k + 2) % 3]
                e_ab, e_bc, e_ca = edge_of(a, b), edge_of(b, c), edge_of(c, a)
                r1 = across(e_ab, d)
                r23 = [r for r in (across(e_bc, d), across(e_ca, d)) if r >= 0]
                for e, sub in ((e_ab, r23[0] if r23 else -1), (e_bc, r1), (e_ca, r1)):
                    nrm = N[sub] if sub >= 0 else 0.0
                    if f0[e] == d:
                        n0[e] = nrm
                    elif f1[e] == d:
                        n1[e] = nrm
        self.edge_n = (n0, n1)

    # ------------------------------------------------------------------
    def transformed(self, R, t=None):
        """Return a copy with vertices mapped by v' = R @ v + t."""
        m = Mesh.__new__(Mesh)
        m.V = self.V @ np.asarray(R).T
        if t is not None:
            m.V = m.V + t
        m.F = self.F.copy()
        m._build()
        return m

    @property
    def bounds(self):
        return self.V.min(axis=0), self.V.max(axis=0)

    @property
    def extents(self):
        lo, hi = self.bounds
        return hi - lo

    def volume(self):
        """Signed volume via the divergence theorem (mm^3 for mm input)."""
        a, b, c = self.V[self.F[:, 0]], self.V[self.F[:, 1]], self.V[self.F[:, 2]]
        return float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0)

    def sharp_edges(self, angle_deg=30.0):
        """Boolean mask of edges whose dihedral angle exceeds angle_deg, plus
        boundary / non-manifold edges."""
        n0, n1 = self.edge_n
        manifold = (self.edge_nfaces == 2) & (self.edge_faces[:, 1] >= 0)
        known = (np.abs(n0).sum(axis=1) > 0) & (np.abs(n1).sum(axis=1) > 0)
        cosang = np.einsum("ij,ij->i", n0, n1)
        sharp = manifold & known & (cosang < np.cos(np.radians(angle_deg)))
        return sharp | ~manifold


def canonical_frame(mesh, axis_tol_deg=1.0):
    """Rotation that aligns a part's own planar faces with X/Y/Z.

    Returns R (3x3, rows are the new axes in old coordinates) such that the
    longest extent lies along X, the next along Y and the shortest (plate
    thickness, bolt axis ...) along Z.  The front view (looking down -Z) is then
    the most descriptive one: a plate's profile, a tube's broad face, etc.
    """
    N, A = mesh.N, mesh.face_area
    good = A > 1e-9
    N, A = N[good], A[good]
    # Fold n and -n together so both faces of a plate vote for one axis.
    flip = (N[:, 2] < -1e-9) | ((np.abs(N[:, 2]) <= 1e-9) & (N[:, 1] < -1e-9)) | (
        (np.abs(N[:, 2]) <= 1e-9) & (np.abs(N[:, 1]) <= 1e-9) & (N[:, 0] < 0)
    )
    N = np.where(flip[:, None], -N, N)
    cos_tol = np.cos(np.radians(axis_tol_deg))

    def dominant(normals, weights):
        """Area-weighted direction of the largest group of parallel faces."""
        if len(normals) == 0:
            return None
        keys = np.round(normals / 0.01).astype(np.int64)
        uniq, inv = np.unique(keys, axis=0, return_inverse=True)
        area = np.bincount(inv.ravel(), weights=weights)
        centre = uniq[np.argmax(area)] * 0.01
        centre /= np.linalg.norm(centre)
        member = np.abs(normals @ centre) >= cos_tol
        d = (normals[member] * weights[member, None]).sum(axis=0)
        return d / np.linalg.norm(d)

    def perpendicular_to(a, normals, weights):
        perp = np.abs(normals @ a) < np.sin(np.radians(axis_tol_deg))
        b = dominant(normals[perp], weights[perp]) if perp.any() else None
        if b is None:
            b = np.cross(a, np.eye(3)[np.argmin(np.abs(a))])
        b = b - a * (b @ a)
        return b / np.linalg.norm(b)

    a1 = dominant(N, A)
    a2 = perpendicular_to(a1, N, A)
    planar = A[np.abs(N @ a1) >= cos_tol].sum() + A[np.abs(N @ a2) >= cos_tol].sum()
    if planar >= 0.5 * A.sum():
        # Of the few biggest edge directions, square the part up to the one
        # that lines up the most edge area with the X and Y axes (a gusset
        # then shows its two legs, not its hypotenuse); ties go to the
        # smaller bounding box.
        perp = np.abs(N @ a1) < np.sin(np.radians(axis_tol_deg))
        keys = np.round(N[perp] / 0.01).astype(np.int64)
        if len(keys):
            uniq, inv = np.unique(keys, axis=0, return_inverse=True)
            area = np.bincount(inv.ravel(), weights=A[perp])
            best = None
            for k in np.argsort(-area, kind="stable")[:4]:
                if area[k] < 0.05 * area.max():
                    break
                centre = uniq[k] * 0.01
                centre /= np.linalg.norm(centre)
                member = perp & (np.abs(N @ centre) >= cos_tol)
                cand = (N[member] * A[member, None]).sum(axis=0)
                cand = cand - a1 * (cand @ a1)
                cand /= np.linalg.norm(cand)
                b = np.cross(a1, cand)
                aligned = A[perp & ((np.abs(N @ cand) >= cos_tol) | (np.abs(N @ b) >= cos_tol))].sum()
                box = np.ptp(mesh.V @ cand) * np.ptp(mesh.V @ b)
                score = (round(aligned / A[perp].sum(), 2), -box)
                if best is None or score > best[0]:
                    best = (score, cand)
            a2 = best[1]
    else:
        # Mostly curved (pin, shaft, round tube, bolt).  Its biggest round
        # edge gives the part's own axis; a cross hole, if any, decides how
        # it is turned about that axis, otherwise a flat (bolt head) does.
        axes = hole_axes(mesh)
        if axes:
            a1 = max(axes, key=lambda a: a[2])[0]
            cross = [ax for ax, _, _ in axes if abs(ax @ a1) < 0.05]
            if cross:
                a2 = cross[0] - a1 * (cross[0] @ a1)
                a2 /= np.linalg.norm(a2)
            else:
                a2 = perpendicular_to(a1, N, A)
    a3 = np.cross(a1, a2)
    basis = np.stack([a1, a2, a3])  # rows: candidate axes

    # Snap to the original axes when the part is already axis-aligned, so
    # small numerical noise never rotates a drawing by a fraction of a degree.
    snapped = np.where(np.abs(basis) > np.cos(np.radians(axis_tol_deg)), np.sign(basis), basis)
    if np.allclose(np.abs(snapped).sum(axis=1), 1.0):
        basis = snapped

    # Order the axes by the part's extent along each: longest -> X.
    proj = mesh.V @ basis.T
    ext = proj.max(axis=0) - proj.min(axis=0)
    order = np.argsort(-ext, kind="stable")
    R = basis[order]
    if np.linalg.det(R) < 0:
        R[2] = -R[2]
    return R


def components(n_vertices, edges):
    """Connected-component label per vertex (union-find over edge list)."""
    parent = np.arange(n_vertices)

    def find(x):
        root = x
        while parent[root] != root:
            root = parent[root]
        while parent[x] != root:
            parent[x], x = root, parent[x]
        return root

    for a, b in edges:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb
    return np.array([find(i) for i in range(n_vertices)])


def edge_loops(mesh, sharp=None, min_vertices=8):
    """Simple closed loops of sharp edges (hole rims, boss outlines), as
    arrays of vertex indices."""
    if sharp is None:
        sharp = mesh.sharp_edges()
    E = mesh.edges[sharp]
    if len(E) == 0:
        return []
    used = np.unique(E)
    remap = -np.ones(len(mesh.V), dtype=int)
    remap[used] = np.arange(len(used))
    le = remap[E]
    deg = np.bincount(le.ravel(), minlength=len(used))
    comp = components(len(used), le)
    ecomp = comp[le[:, 0]]
    loops = []
    for c in np.unique(comp):
        vidx = np.flatnonzero(comp == c)
        eidx = np.flatnonzero(ecomp == c)
        if len(vidx) >= min_vertices and len(eidx) == len(vidx) and np.all(deg[vidx] == 2):
            loops.append((used[vidx], eidx))
    return loops


def hole_axes(mesh):
    """(axis, loop count, largest radius) of round holes/bosses, most frequent
    first.  A rim lies on the hole's cylinder, so its points vary least along
    the hole axis."""
    found = []
    for vidx, _ in edge_loops(mesh):
        P = mesh.V[vidx] - mesh.V[vidx].mean(axis=0)
        w, vec = np.linalg.eigh(P.T @ P)
        if w[1] > 4 * w[0]:  # clearly flatter along one direction
            ax = vec[:, 0]
            ax = ax * np.sign(ax[np.argmax(np.abs(ax))])
            radius = float(np.sqrt(w[1:].mean() * 2 / len(P)))
            for f in found:
                if abs(f[0] @ ax) > 0.999:
                    f[1] += 1
                    f[2] = max(f[2], radius)
                    break
            else:
                found.append([ax, 1, radius])
    found.sort(key=lambda f: -f[1])
    return [tuple(f) for f in found]


def normalize(mesh, R=None):
    """Rotate into the canonical frame and move the bounding-box min to 0."""
    if R is None:
        R = canonical_frame(mesh)
    m = mesh.transformed(R)
    lo, _ = m.bounds
    return m.transformed(np.eye(3), -lo), R, -lo
