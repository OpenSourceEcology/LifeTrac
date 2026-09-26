"""Unit tests for the part-drawing generator (no OpenSCAD needed).

Run from DESIGN-STRUCTURAL/drawings:  python3 -m unittest discover -s tests -v
"""

import importlib.util
import io
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from partdrawings import sheet as S  # noqa: E402
from partdrawings.drawing import plan_part, render_part  # noqa: E402
from partdrawings.hlr import detect_circles, draw_view  # noqa: E402
from partdrawings.mesh import Mesh, normalize  # noqa: E402
import generate_part_drawings as G  # noqa: E402

_seq_path = Path(__file__).resolve().parents[3] / "BUILD-STRUCTURE" / "assembly_sequence.py"
_spec = importlib.util.spec_from_file_location("assembly_sequence", _seq_path)
SEQ = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(SEQ)


def box(lo, hi):
    """Closed, outward-facing triangle soup for an axis-aligned box."""
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    v = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]], float)
    quads = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (2, 3, 7, 6), (1, 2, 6, 5), (0, 4, 7, 3)]
    tris = []
    for a, b, c, d in quads:
        tris += [(v[a], v[b], v[c]), (v[a], v[c], v[d])]
    return np.array(tris)


def washer(R, r, t, n=32):
    """Annular prism (a flat washer / a plate with one hole)."""
    ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
    out = [np.stack([R * np.cos(ang), R * np.sin(ang), np.full(n, z)], 1) for z in (0, t)]
    inn = [np.stack([r * np.cos(ang), r * np.sin(ang), np.full(n, z)], 1) for z in (0, t)]
    tris = []
    for i in range(n):
        j = (i + 1) % n
        tris += [(out[1][i], out[1][j], inn[1][j]), (out[1][i], inn[1][j], inn[1][i])]  # top
        tris += [(out[0][i], inn[0][j], out[0][j]), (out[0][i], inn[0][i], inn[0][j])]  # bottom
        tris += [(out[0][i], out[0][j], out[1][j]), (out[0][i], out[1][j], out[1][i])]  # outer wall
        tris += [(inn[0][i], inn[1][j], inn[0][j]), (inn[0][i], inn[1][i], inn[1][j])]  # inner wall
    return np.array(tris)


class MeshTests(unittest.TestCase):
    def test_box_topology_and_volume(self):
        m = Mesh(box((0, 0, 0), (10, 20, 30)))
        self.assertEqual(len(m.V), 8)
        self.assertEqual(len(m.F), 12)
        self.assertEqual(len(m.edges), 18)
        self.assertTrue(np.all(m.edge_nfaces == 2))
        self.assertAlmostEqual(m.volume(), 6000.0, places=6)
        self.assertEqual(int(m.sharp_edges().sum()), 12)  # face diagonals are not drawn

    def test_canonical_frame_orders_extents(self):
        m, R, _ = normalize(Mesh(box((0, 0, 0), (10, 30, 5))))
        np.testing.assert_allclose(m.extents, [30, 10, 5], atol=1e-9)
        np.testing.assert_allclose(m.bounds[0], [0, 0, 0], atol=1e-9)
        self.assertAlmostEqual(np.linalg.det(R), 1.0)

    def test_canonical_frame_undoes_rotation(self):
        tris = box((0, 0, 0), (40, 12, 3))
        a = math.radians(33)
        Rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
        m, _, _ = normalize(Mesh(tris @ Rz.T))
        np.testing.assert_allclose(m.extents, [40, 12, 3], atol=1e-6)

    def test_face_order_does_not_change_geometry_id(self):
        tris = washer(20, 6, 3)
        a = Mesh(tris)
        b = Mesh(tris[::-1].copy())
        np.testing.assert_array_equal(a.F, b.F)
        self.assertEqual(G.geometry_id(a), G.geometry_id(b))


class HiddenLineTests(unittest.TestCase):
    def test_box_front_outline(self):
        vd = draw_view(Mesh(box((0, 0, 0), (10, 20, 30))), "front")
        np.testing.assert_allclose(vd.size, [10, 20])
        self.assertTrue(vd.visible.all())

    def test_part_behind_is_hidden(self):
        near = box((0, 0, 5), (10, 10, 10))
        far = box((3, 3, 0), (6, 6, 2))
        vd = draw_view(Mesh(np.concatenate([near, far])), "front")
        mid = vd.segments.mean(axis=1)
        inner = np.all((mid > 2.9) & (mid < 6.1), axis=1)
        self.assertTrue(inner.any())
        self.assertFalse(vd.visible[inner].any(), "edges of the rear box should be hidden")
        self.assertTrue(vd.visible[~inner].all())

    def test_detect_hole_and_boss(self):
        m = Mesh(washer(R=20, r=6, t=3))
        circles = detect_circles(m, "front")
        self.assertEqual(len(circles), 2)
        hole = [c for c in circles if c["hole"]]
        boss = [c for c in circles if not c["hole"]]
        self.assertEqual(len(hole), 1)
        self.assertEqual(len(boss), 1)
        self.assertAlmostEqual(hole[0]["r"], 6, places=6)
        self.assertAlmostEqual(boss[0]["r"], 20, places=6)
        self.assertEqual(len(hole[0]["depths"]), 2)  # entry and exit rims -> THRU
        self.assertEqual(detect_circles(m, "top"), [])  # seen edge-on: no circles

    def test_rounded_rectangle_is_not_a_circle(self):
        # Only corner vertices, all equidistant from the centre.
        pts = []
        for cx, cy, a0 in ((10, 10, 0), (-10, 10, 90), (-10, -10, 180), (10, -10, 270)):
            for k in range(3):
                a = math.radians(a0 + 45 * k)
                pts.append((cx + 2 * math.cos(a), cy + 2 * math.sin(a)))
        n = len(pts)
        tris = []
        for i in range(n):
            j = (i + 1) % n
            a0, a1 = pts[i], pts[j]
            tris += [((0, 0, 1), (a0[0], a0[1], 1), (a1[0], a1[1], 1)),
                     ((0, 0, 0), (a1[0], a1[1], 0), (a0[0], a0[1], 0)),
                     ((a0[0], a0[1], 0), (a1[0], a1[1], 0), (a1[0], a1[1], 1)),
                     ((a0[0], a0[1], 0), (a1[0], a1[1], 1), (a0[0], a0[1], 1))]
        self.assertEqual(detect_circles(Mesh(np.array(tris, float)), "front"), [])


def round_pin(R=10.0, L=100.0, n=32):
    """Faceted round pin along Z with flat ends, like OpenSCAD's cylinder()."""
    ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = [np.stack([R * np.cos(ang), R * np.sin(ang), np.full(n, z)], 1) for z in (0, L)]
    tris = []
    for i in range(n):
        j = (i + 1) % n
        tris += [(ring[0][i], ring[0][j], ring[1][j]), (ring[0][i], ring[1][j], ring[1][i])]
        tris += [((0, 0, 0), ring[0][j], ring[0][i]), ((0, 0, L), ring[1][i], ring[1][j])]
    return np.array(tris, float)


class OrientationTests(unittest.TestCase):
    def test_round_part_axis_along_x(self):
        m, _, _ = normalize(Mesh(round_pin()))
        self.assertAlmostEqual(m.extents[0], 100.0, places=6)

    def test_degenerate_sliver_does_not_add_lines(self):
        # A flat square face split with a T-junction stitched by a zero-area
        # sliver (as CGAL sometimes emits): no line may appear across the face.
        tris = list(box((0, 0, 0), (10, 10, 2)))
        rest = [t for t in tris if not np.allclose(np.array(t)[:, 2], 2)]
        a, b, c, d = (0, 0, 2), (10, 0, 2), (10, 10, 2), (0, 10, 2)
        mid = (5, 5, 2)  # on the diagonal a-c
        new_top = [(a, b, mid), (b, c, mid), (c, d, a), (a, mid, c)]  # last one is the sliver
        m = Mesh(np.array(rest + new_top, float))
        vd = draw_view(m, "front")
        mids = vd.segments.mean(axis=1)
        inside = np.all((mids > 0.5) & (mids < 9.5), axis=1)
        self.assertFalse(inside.any(), "a line was drawn across a flat face")


class MergeTests(unittest.TestCase):
    def test_collinear_pieces_merge_and_hidden_under_visible_is_dropped(self):
        from partdrawings.hlr import merge_segments

        segs = np.array([[[0, 0], [4, 0]], [[4, 0], [10, 0]],   # visible, split in two
                         [[2, 0], [6, 0]],                        # hidden, under the visible line
                         [[0, 5], [3, 5]], [[3, 5], [8, 5]]])     # hidden, split in two
        vis = np.array([True, True, False, False, False])
        out, ov = merge_segments(segs.astype(float), vis)
        self.assertEqual(len(out), 2)
        np.testing.assert_allclose(sorted(map(tuple, out[ov].reshape(-1, 4))), [(0, 0, 10, 0)])
        np.testing.assert_allclose(sorted(map(tuple, out[~ov].reshape(-1, 4))), [(0, 5, 8, 5)])


class FormattingTests(unittest.TestCase):
    def test_numbers(self):
        self.assertEqual(S.fmt_in(12.7), ".500")
        self.assertEqual(S.fmt_dual(146.05), "146.1 [5.750]")
        self.assertEqual(S.fmt_frac_in(146.05), '5-3/4"')
        self.assertEqual(S.fmt_frac_in(12.7), '1/2"')
        self.assertEqual(S.fmt_frac_in(101), '~4"')  # 3.976 in: more than 1/64 off
        self.assertEqual(S.scale_label(0.25), "1:4")

    def test_revision_letters_skip_ambiguous(self):
        self.assertEqual(G.next_rev(""), "A")
        self.assertEqual(G.next_rev("H"), "J")  # no I
        self.assertEqual(G.next_rev("N"), "P")  # no O
        self.assertEqual(G.next_rev("Y"), "AA")
        self.assertEqual(G.next_rev("AY"), "BA")


class SheetTests(unittest.TestCase):
    def test_render_plate_sheet(self):
        from reportlab.pdfgen import canvas

        mesh, _, _ = normalize(Mesh(washer(R=50, r=10, t=12.7)))
        info = dict(project="TEST", id="P99", name="Test washer plate", stock="PL 1/2", category="plate",
                    qty="4", qty_note="", process="CNC", finish="-", size_label="SIZE", size="100 x 100",
                    mass="0.7 kg", geom_id="deadbeef", source="x.scad : y()", used_in="-", notes=["NOTE"],
                    rev="A", date="2026-01-01")
        buf = io.BytesIO()
        c = canvas.Canvas(buf, pagesize=S.PAPERS["letter"], invariant=1)
        res = render_part(c, mesh, info, {"hidden_lines": False, "thickness": True, "drill_below": 12.7})
        c.save()
        self.assertEqual(res["sheets"], 1)
        self.assertEqual(len(res["holes"]), 1)
        self.assertAlmostEqual(res["holes"][0]["d"], 20, places=3)
        self.assertFalse(res["holes"][0]["drill"])  # Ø20 > 12.7 thick: plasma is fine
        self.assertTrue(buf.getvalue().startswith(b"%PDF"))


class RevisionAndBookTests(unittest.TestCase):
    def test_book_page_numbers(self):
        # Two index pages, then parts of 1, 2 and 1 sheets: pages 3, 4-5, 6.
        starts, total = G.book_page_starts(2, [1, 2, 1])
        self.assertEqual(starts, [3, 4, 6])
        self.assertEqual(total, 6)

    def test_fingerprint_tracks_what_is_printed(self):
        mesh, _, _ = normalize(Mesh(washer(R=50, r=10, t=12.7)))
        opts = {"hidden_lines": False, "thickness": True}
        base = plan_part(mesh, opts)["fingerprint"]
        self.assertEqual(base, plan_part(mesh, dict(opts))["fingerprint"])
        self.assertNotEqual(base, plan_part(mesh, opts, paper="a3")["fingerprint"])
        self.assertNotEqual(base, plan_part(mesh, dict(opts, views=["front", "iso"]))["fingerprint"])
        self.assertNotEqual(base, plan_part(mesh, dict(opts, hole_table=False))["fingerprint"])
        moved, _, _ = normalize(Mesh(washer(R=50, r=12, t=12.7)))
        self.assertNotEqual(base, plan_part(moved, opts)["fingerprint"])


class SequenceCheckTests(unittest.TestCase):
    PARTS = {
        "P1": {"name": "Plate", "category": "plate"},
        "F1": {"name": "Counted nut", "category": "fastener"},
        "F3": {"name": "Estimated bolt", "category": "fastener"},
        "J1": {"name": "Jig", "category": "printed"},
    }
    QTY = {"P1": 1, "F1": 4, "F3": 154, "J1": 2}
    EXACT = {"P1", "F1"}  # counted in the model; F3 is a hole-count estimate

    def run_check(self, steps, complete=False):
        seq = {"complete": complete, "phases": [{"id": "p", "steps": steps}]}
        return SEQ.check(seq, self.PARTS, self.QTY, self.EXACT)

    def test_counted_hardware_is_enforced_and_estimates_are_not(self):
        _, _, errors, warnings = self.run_check([
            {"id": "a", "add": [{"part": "P1", "qty": 1}, {"part": "F1", "qty": 5}, {"part": "F3", "qty": 200}]}])
        self.assertTrue(any("F1 placed 5" in e for e in errors), errors)
        self.assertFalse(any("F3" in e for e in errors), errors)
        self.assertTrue(any("F3 placed 200" in w for w in warnings), warnings)

    def test_unplaced_counted_part_is_reported(self):
        _, _, errors, warnings = self.run_check([{"id": "a", "add": [{"part": "P1", "qty": 1}]}])
        self.assertTrue(any("F1" in w for w in warnings), warnings)          # counted: must be placed
        self.assertFalse(any("F3" in w for w in warnings), warnings)         # estimate: not required
        _, _, errors, _ = self.run_check([{"id": "a", "add": [{"part": "P1", "qty": 1}]}], complete=True)
        self.assertTrue(any("F1" in e for e in errors), errors)

    def test_fabricated_tally_leaves_out_purchased_hardware(self):
        steps, placed, errors, warnings = self.run_check([
            {"id": "a", "add": [{"part": "P1", "qty": 1}, {"part": "F1", "qty": 4}]}])
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "seq.md"
            SEQ.write_markdown(out, {"phases": []}, self.PARTS, self.QTY, self.EXACT, {}, steps, placed,
                               errors, warnings, "seq.yaml")
            text = out.read_text()
        self.assertIn("1 of 1 fabricated pieces placed", text)   # P1 only; F1 is bought, not made

    def test_structure_errors(self):
        _, _, errors, _ = self.run_check([
            {"id": "a", "after": ["b"], "add": [{"part": "J1", "qty": 1}, {"part": "X9", "qty": 1}]},
            {"id": "a", "uses": ["nope"]}])
        text = " | ".join(errors)
        for needle in ("after: b", "J1 is a jig", "unknown part id X9", "also used by step 1", "uses: nope"):
            self.assertIn(needle, text)


if __name__ == "__main__":
    unittest.main()
