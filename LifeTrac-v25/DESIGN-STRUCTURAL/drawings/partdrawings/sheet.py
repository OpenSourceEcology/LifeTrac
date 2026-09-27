"""Draw part-drawing sheets with reportlab.

Sheet layout (landscape):

    +------------------------------------------+------------------+
    |  TOP                                     |  ISOMETRIC       |
    |                                          |                  |
    |  FRONT                 RIGHT             |  HOLE TABLE      |
    |                                          |                  |
    +---------------------------+--------------+------------------+
    |  NOTES                    |  TITLE BLOCK                    |
    +---------------------------+---------------------------------+

Views use third-angle projection (ASME Y14.3): top above front, right side
to the right.  All orthographic views share one standard scale.  Dimensions
are millimetres with inches in brackets.
"""

import math

import numpy as np
from reportlab.lib.colors import Color, black, white
from reportlab.lib.pagesizes import A3, A4, landscape, letter, TABLOID

from .hlr import draw_view

PT_PER_MM = 72.0 / 25.4
PAPERS = {
    "letter": landscape(letter),
    "a4": landscape(A4),
    "tabloid": landscape(TABLOID),
    "a3": landscape(A3),
}
SHEET_SIZE_NAME = {"letter": "A (LETTER)", "a4": "A4", "tabloid": "B (11x17)", "a3": "A3"}
STANDARD_SCALES = [5, 4, 2, 1, 1 / 2, 1 / 2.5, 1 / 4, 1 / 5, 1 / 8, 1 / 10, 1 / 15, 1 / 20, 1 / 25, 1 / 40, 1 / 50]

LINE_VISIBLE = 1.0
LINE_HIDDEN = 0.5
LINE_THIN = 0.35
GREY = Color(0.35, 0.35, 0.35)
LIGHT = Color(0.93, 0.93, 0.93)
ACCENT = Color(0.75, 0.1, 0.1)


# ----------------------------------------------------------------------------
# Number formatting
# ----------------------------------------------------------------------------

def fmt_mm(x):
    x = float(x)
    if abs(x - round(x)) < 0.05:
        return "%d" % round(x)
    return "%.1f" % x


def fmt_in(x_mm):
    s = "%.3f" % (float(x_mm) / 25.4)
    return s[1:] if s.startswith("0.") else s  # ASME: no leading zero for inches


def fmt_dual(x_mm):
    return "%s [%s]" % (fmt_mm(x_mm), fmt_in(x_mm))


def fmt_frac_in(x_mm, denom=16):
    """Nearest 1/16" as a tape-measure fraction, e.g. 146.05 -> 5-3/4"."""
    inches = float(x_mm) / 25.4
    n = round(inches * denom)
    whole, rem = divmod(n, denom)
    g = math.gcd(rem, denom)
    if rem == 0:
        s = '%d"' % whole
    elif whole == 0:
        s = '%d/%d"' % (rem // g, denom // g)
    else:
        s = '%d-%d/%d"' % (whole, rem // g, denom // g)
    if abs(inches - n / denom) > 1 / 64:
        s = "~" + s
    return s


def scale_label(r):
    if r >= 1:
        return "%g:1" % r
    return "1:%g" % (1 / r)


# ----------------------------------------------------------------------------
# Primitive drawing helpers
# ----------------------------------------------------------------------------

def _arrow(c, x, y, ang, size=4.5):
    p = c.beginPath()
    p.moveTo(x, y)
    p.lineTo(x - size * math.cos(ang - 0.28), y - size * math.sin(ang - 0.28))
    p.lineTo(x - size * math.cos(ang + 0.28), y - size * math.sin(ang + 0.28))
    p.close()
    c.drawPath(p, fill=1, stroke=0)


def _text_box(c, x, y, text, size, rot=0, anchor="middle"):
    """Draw text with a white knock-out box so it stays legible over lines."""
    w = c.stringWidth(text, "Helvetica", size)
    c.saveState()
    c.translate(x, y)
    c.rotate(rot)
    dx = {"middle": -w / 2, "start": 0, "end": -w}[anchor]
    c.setFillColor(white)
    c.rect(dx - 1.2, -1.5, w + 2.4, size + 1.5, stroke=0, fill=1)
    c.setFillColor(black)
    c.setFont("Helvetica", size)
    c.drawString(dx, 0, text)
    c.restoreState()


def dim_h(c, x1, x2, y_feature, y_dim, text, size=6.5):
    """Horizontal dimension between x1 and x2, drawn at y_dim."""
    c.saveState()
    c.setLineWidth(LINE_THIN)
    sgn = 1 if y_dim > y_feature else -1
    for x in (x1, x2):
        c.line(x, y_feature + sgn * 2, x, y_dim + sgn * 3)
    c.line(x1, y_dim, x2, y_dim)
    _arrow(c, x1, y_dim, math.pi)
    _arrow(c, x2, y_dim, 0)
    _text_box(c, (x1 + x2) / 2, y_dim + 1.5, text, size)
    c.restoreState()


def dim_v(c, y1, y2, x_feature, x_dim, text, size=6.5):
    """Vertical dimension between y1 and y2, drawn at x_dim (text reads upward)."""
    c.saveState()
    c.setLineWidth(LINE_THIN)
    sgn = 1 if x_dim > x_feature else -1
    for y in (y1, y2):
        c.line(x_feature + sgn * 2, y, x_dim + sgn * 3, y)
    c.line(x_dim, y1, x_dim, y2)
    _arrow(c, x_dim, y1, -math.pi / 2)
    _arrow(c, x_dim, y2, math.pi / 2)
    _text_box(c, x_dim - 1.5, (y1 + y2) / 2, text, size, rot=90)
    c.restoreState()


def projection_symbol(c, x, y, h=9.0):
    """Third-angle projection symbol (ASME Y14.3 / ISO 5456-2): truncated cone
    side view on the left, its small end pointing at the end view on the right."""
    c.saveState()
    c.setLineWidth(0.6)
    big, small, length = h, h * 0.5, h * 1.05
    p = c.beginPath()
    p.moveTo(x, y)
    p.lineTo(x, y + big)
    p.lineTo(x + length, y + big / 2 + small / 2)
    p.lineTo(x + length, y + big / 2 - small / 2)
    p.close()
    c.drawPath(p, fill=0, stroke=1)
    cx, cy = x + length + big * 0.95, y + big / 2
    c.circle(cx, cy, big / 2, fill=0, stroke=1)
    c.circle(cx, cy, small / 2, fill=0, stroke=1)
    c.setLineWidth(0.3)
    c.setDash(3, 1.2)
    c.line(x - 2, cy, cx + big / 2 + 2, cy)
    c.line(cx, y - 2, cx, y + big + 2)
    c.restoreState()


# ----------------------------------------------------------------------------
# Views
# ----------------------------------------------------------------------------

class PlacedView:
    def __init__(self, vd, ox, oy, s):
        self.vd, self.ox, self.oy, self.s = vd, ox, oy, s

    def xy(self, u, v):
        return self.ox + (u - self.vd.lo[0]) * self.s, self.oy + (v - self.vd.lo[1]) * self.s

    @property
    def box(self):
        w, h = self.vd.size * self.s
        return self.ox, self.oy, self.ox + w, self.oy + h


def draw_lines(c, pv, hidden=True):
    segs, vis = pv.vd.segments, pv.vd.visible
    ox, oy, s = pv.ox, pv.oy, pv.s
    lo = pv.vd.lo
    P = (segs - lo) * s + np.array([ox, oy])
    if hidden and (~vis).any():
        c.saveState()
        c.setLineWidth(LINE_HIDDEN)
        c.setStrokeColor(GREY)
        c.setDash(2.4, 1.4)
        c.lines([tuple(p.ravel()) for p in P[~vis]])
        c.restoreState()
    c.saveState()
    c.setLineWidth(LINE_VISIBLE)
    c.setLineCap(1)
    c.lines([tuple(p.ravel()) for p in P[vis]])
    c.restoreState()


def choose_scale(extents, area_w, area_h, views, pad_x, pad_y, gap):
    X, Y, Z = extents
    cols = X + (Z if "right" in views else 0)
    rows = Y + (Z if "top" in views else 0)
    ncol_gap = gap if "right" in views else 0
    nrow_gap = gap if "top" in views else 0
    s_max = min((area_w - pad_x - ncol_gap) / max(cols, 1e-6), (area_h - pad_y - nrow_gap) / max(rows, 1e-6))
    ratio = s_max / PT_PER_MM
    for r in STANDARD_SCALES:
        if r <= ratio * 1.0001:
            return r
    return ratio  # smaller than any standard scale: use the exact fit


# ----------------------------------------------------------------------------
# Hole table
# ----------------------------------------------------------------------------

def collect_holes(placed, thickness_hint=None, drill_below=None):
    """Tag every detected hole: letter per diameter, number per position."""
    rows = []
    for name in ("front", "top", "right"):
        pv = placed.get(name)
        if pv is None:
            continue
        for circ in pv.vd.circles:
            if not circ["hole"]:
                continue
            x = circ["u"] - pv.vd.lo[0]
            y = circ["v"] - pv.vd.lo[1]
            walls = max(1, len(circ["depths"]) // 2)
            rows.append({"view": name, "x": x, "y": y, "d": 2 * circ["r"], "walls": walls, "circ": circ, "pv": pv})
    sizes = sorted({round(r["d"], 1) for r in rows})
    letters = "ABCDEFGHJKLMNPRSTUVWXYZ"
    for r in rows:
        r["size_key"] = round(r["d"], 1)
    rows.sort(key=lambda r: (sizes.index(r["size_key"]), ("front", "top", "right").index(r["view"]), round(r["x"], 1), round(r["y"], 1)))
    counters = {}
    for r in rows:
        L = letters[sizes.index(r["size_key"]) % len(letters)]
        counters[L] = counters.get(L, 0) + 1
        r["tag"] = "%s%d" % (L, counters[L])
        r["drill"] = drill_below is not None and r["d"] < drill_below - 0.05
    return rows


def hole_size_text(r):
    s = "Ø%s [%s]" % (fmt_mm(r["d"]) if r["d"] >= 10 else "%.1f" % r["d"], fmt_in(r["d"]))
    s += " THRU" if r["walls"] == 1 else " THRU %dX WALLS" % r["walls"]
    if r["drill"]:
        s += " DRILL*"
    return s


def draw_hole_tags(c, rows):
    """Tag each hole, trying eight spots round it so tags don't collide
    with each other or with neighbouring holes."""
    font, fs = "Helvetica-Bold", 5.2
    obstacles = []
    for r in rows:
        x, y = r["pv"].xy(r["circ"]["u"], r["circ"]["v"])
        rad = r["circ"]["r"] * r["pv"].s
        obstacles.append((x - rad, y - rad, x + rad, y + rad))
    placed = []

    def clear(box, own):
        for k, o in enumerate(obstacles + placed):
            if k == own:
                continue
            if box[0] < o[2] and box[2] > o[0] and box[1] < o[3] and box[3] > o[1]:
                return False
        return True

    c.saveState()
    for n, r in enumerate(rows):
        pv, circ = r["pv"], r["circ"]
        x, y = pv.xy(circ["u"], circ["v"])
        rad = circ["r"] * pv.s
        w = c.stringWidth(r["tag"], font, fs)
        off = rad * 0.72 + 1.0
        spots = [(x + off, y + off), (x - off - w, y + off), (x + off, y - off - fs), (x - off - w, y - off - fs),
                 (x + rad + 1.2, y - fs / 2.5), (x - rad - 1.2 - w, y - fs / 2.5),
                 (x - w / 2, y + rad + 1.2), (x - w / 2, y - rad - 1.2 - fs)]
        choice = spots[0]
        for tx, ty in spots:
            box = (tx - 0.3, ty - 0.8, tx + w + 0.3, ty + fs * 0.85)
            if clear(box, n):
                choice = (tx, ty)
                break
        tx, ty = choice
        placed.append((tx - 0.3, ty - 0.8, tx + w + 0.3, ty + fs * 0.85))
        c.setFillColor(ACCENT)
        c.setFont(font, fs)
        c.drawString(tx, ty, r["tag"])
        if rad < 1.6:  # hole too small to see at this scale: mark its centre
            c.setStrokeColor(ACCENT)
            c.setLineWidth(0.3)
            c.line(x - 2, y, x + 2, y)
            c.line(x, y - 2, x, y + 2)
    c.restoreState()


HOLE_ROW_H = 8.4


def hole_table_capacity(y_top, y_min, rh=HOLE_ROW_H):
    """Rows draw_hole_table() fits between y_top and y_min, below its title
    (12) and header row.  plan_part() counts sheets with the same rule."""
    return max(0, int(math.floor((y_top - 12 - rh - y_min) / rh + 1e-9)))


def draw_hole_table(c, rows, x, y_top, w, y_min, title="HOLE TABLE"):
    """Draw as many rows as fit; return the rows that did not fit."""
    rh, fs = HOLE_ROW_H, 6.0
    cols = [("TAG", 0.10), ("VIEW", 0.12), ("X", 0.21), ("Y", 0.21), ("SIZE", 0.36)]
    c.saveState()
    c.setFont("Helvetica-Bold", 7)
    c.drawString(x, y_top - 8, title)
    c.setFont("Helvetica", 5.2)
    c.drawRightString(x + w, y_top - 8, "X, Y FROM LOWER-LEFT CORNER OF EACH VIEW")
    y = y_top - 12
    c.setFillColor(LIGHT)
    c.rect(x, y - rh, w, rh, stroke=0, fill=1)
    c.setFillColor(black)
    c.setFont("Helvetica-Bold", fs)
    cx = x
    for name, frac in cols:
        c.drawString(cx + 2, y - rh + 2.3, name)
        cx += frac * w
    y -= rh
    c.setFont("Helvetica", fs)
    c.setLineWidth(0.25)
    shown = 0
    for r in rows[:hole_table_capacity(y_top, y_min, rh)]:
        vals = [r["tag"], r["view"].upper(), fmt_dual(r["x"]), fmt_dual(r["y"]), hole_size_text(r)]
        cx = x
        for (name, frac), val in zip(cols, vals):
            if name == "SIZE":
                while c.stringWidth(val, "Helvetica", fs) > frac * w - 3 and fs > 4.2:
                    fs -= 0.2
                    c.setFont("Helvetica", fs)
            c.drawString(cx + 2, y - rh + 2.3, val)
            cx += frac * w
        fs = 6.0
        c.setFont("Helvetica", fs)
        c.line(x, y - rh, x + w, y - rh)
        y -= rh
        shown += 1
    c.rect(x, y, w, (y_top - 12) - y, stroke=1, fill=0)
    c.restoreState()
    return rows[shown:]


# ----------------------------------------------------------------------------
# Sheet frame, notes and title block
# ----------------------------------------------------------------------------

class SheetGeometry:
    def __init__(self, paper):
        self.W, self.H = PAPERS[paper]
        self.paper = paper
        m = 0.3 * 72
        self.frame = (m, m, self.W - m, self.H - m)
        x0, y0, x1, y1 = self.frame
        self.band_h = 1.5 * 72
        self.title_w = min(5.4 * 72, (x1 - x0) * 0.55)
        self.right_w = max(3.1 * 72, (x1 - x0) * 0.30)
        self.title_box = (x1 - self.title_w, y0, x1, y0 + self.band_h)
        self.notes_box = (x0, y0, x1 - self.title_w, y0 + self.band_h)
        self.views_box = (x0, y0 + self.band_h, x1 - self.right_w, y1)
        self.right_box = (x1 - self.right_w, y0 + self.band_h, x1, y1)


def draw_frame(c, g):
    x0, y0, x1, y1 = g.frame
    c.saveState()
    c.setLineWidth(1.2)
    c.rect(x0, y0, x1 - x0, y1 - y0)
    c.setLineWidth(0.6)
    c.line(x0, y0 + g.band_h, x1, y0 + g.band_h)
    c.line(g.title_box[0], y0, g.title_box[0], y0 + g.band_h)
    c.line(g.right_box[0], g.right_box[1], g.right_box[0], g.right_box[3])
    c.restoreState()


def draw_notes(c, g, notes):
    x0, y0, x1, y1 = g.notes_box
    c.saveState()
    c.setFont("Helvetica-Bold", 7)
    c.drawString(x0 + 5, y1 - 10, "NOTES - UNLESS OTHERWISE SPECIFIED")
    size = 5.8
    lines = []
    width = x1 - x0 - 16
    for i, n in enumerate(notes, 1):
        words = ("%d. %s" % (i, n)).split()
        cur = ""
        for w_ in words:
            trial = (cur + " " + w_).strip()
            if c.stringWidth(trial, "Helvetica", size) > width and cur:
                lines.append(cur)
                cur = "   " + w_
            else:
                cur = trial
        lines.append(cur)
    avail = int((y1 - y0 - 16) // (size + 1.6))
    if len(lines) > avail:
        size = max(4.4, size * avail / len(lines))
    c.setFont("Helvetica", size)
    y = y1 - 19
    for ln in lines:
        c.drawString(x0 + 6, y, ln)
        y -= size + 1.6
    c.restoreState()


def _cell(c, x, y, w, h, label, value, vsize=8, bold=False, color=black):
    c.rect(x, y, w, h, stroke=1, fill=0)
    c.setFont("Helvetica", 4.8)
    c.setFillColor(GREY)
    c.drawString(x + 2, y + h - 6, label)
    c.setFillColor(color)
    font = "Helvetica-Bold" if bold else "Helvetica"
    size = vsize
    while c.stringWidth(value, font, size) > w - 4 and size > 4:
        size -= 0.25
    c.setFont(font, size)
    c.drawString(x + 2.5, y + 2.8, value)
    c.setFillColor(black)


def draw_title_block(c, g, info, sheet_no, sheet_total):
    x0, y0, x1, y1 = g.title_box
    w = x1 - x0
    rh = (y1 - y0) / 6.0
    c.saveState()
    c.setLineWidth(0.45)
    # Row 6 (top): owner + part number
    y = y1 - rh
    _cell(c, x0, y, w * 0.62, rh, "PROJECT", info["project"], 7.5, bold=True)
    _cell(c, x0 + w * 0.62, y, w * 0.38, rh, "PART NO.", info["id"], 10, bold=True, color=ACCENT)
    # Row 5: title
    y -= rh
    _cell(c, x0, y, w, rh, "TITLE", info["name"].upper(), 10, bold=True)
    # Row 4: material + qty
    y -= rh
    _cell(c, x0, y, w * 0.62, rh, "MATERIAL / STOCK", info["stock"], 6.8)
    _cell(c, x0 + w * 0.62, y, w * 0.38, rh, "QTY PER MACHINE" + info.get("qty_note", ""), info["qty"], 10, bold=True, color=ACCENT)
    # Row 3: process / finish / size / mass
    y -= rh
    _cell(c, x0, y, w * 0.30, rh, "PROCESS", info["process"], 6.2)
    _cell(c, x0 + w * 0.30, y, w * 0.32, rh, "FINISH", info["finish"], 6.2)
    _cell(c, x0 + w * 0.62, y, w * 0.22, rh, info.get("size_label", "SIZE"), info["size"], 6.2)
    _cell(c, x0 + w * 0.84, y, w * 0.16, rh, "MASS (CALC)", info["mass"], 6.2)
    # Row 2: scale / units / projection / sheet / date / geometry id
    y -= rh
    _cell(c, x0, y, w * 0.10, rh, "SCALE", info["scale"], 7)
    _cell(c, x0 + w * 0.10, y, w * 0.11, rh, "UNITS", "mm [in]", 7)
    c.rect(x0 + w * 0.21, y, w * 0.11, rh)
    c.setFont("Helvetica", 4.8)
    c.setFillColor(GREY)
    c.drawString(x0 + w * 0.21 + 2, y + rh - 6, "3RD ANGLE")
    c.setFillColor(black)
    projection_symbol(c, x0 + w * 0.21 + 6, y + 2.2, h=rh * 0.5)
    _cell(c, x0 + w * 0.32, y, w * 0.11, rh, "SHEET", "%d OF %d" % (sheet_no, sheet_total), 7)
    _cell(c, x0 + w * 0.43, y, w * 0.07, rh, "REV", info.get("rev", "-"), 8, bold=True)
    _cell(c, x0 + w * 0.50, y, w * 0.17, rh, "REV DATE", info["date"], 7)
    _cell(c, x0 + w * 0.67, y, w * 0.33, rh, "GEOMETRY ID  |  SHEET SIZE", "%s  |  %s" % (info["geom_id"], SHEET_SIZE_NAME[g.paper]), 6.5)
    # Row 1: source
    y -= rh
    _cell(c, x0, y, w * 0.72, rh, "SOURCE (OPENSCAD)", info["source"], 5.8)
    _cell(c, x0 + w * 0.72, y, w * 0.28, rh, "USED IN", info["used_in"], 5.8)
    c.restoreState()


# ----------------------------------------------------------------------------
# Iso panel
# ----------------------------------------------------------------------------

def draw_iso(c, mesh, box, label="ISOMETRIC VIEW (REF, NOT TO SCALE)", shade=True):
    x0, y0, x1, y1 = box
    vd = draw_view(mesh, "iso")
    pad = 10
    w, h = vd.size
    s = min((x1 - x0 - 2 * pad) / max(w, 1e-6), (y1 - y0 - 2 * pad - 10) / max(h, 1e-6))
    ox = x0 + (x1 - x0 - w * s) / 2
    oy = y0 + (y1 - y0 - 10 - h * s) / 2
    pv = PlacedView(vd, ox, oy, s)
    if shade:
        _shade_faces(c, mesh, pv)
    draw_lines(c, pv, hidden=False)
    c.saveState()
    c.setFont("Helvetica-Bold", 6.5)
    c.drawCentredString((x0 + x1) / 2, y1 - 10, label)
    c.restoreState()
    return pv


def _shade_faces(c, mesh, pv):
    """Flat-shade front-facing triangles, painter's order (far to near)."""
    from .hlr import view_basis

    d, u, v = view_basis("iso")
    front = (mesh.N @ d) > 1e-6
    F = mesh.F[front]
    if len(F) == 0:
        return
    N = mesh.N[front]
    light = np.array([0.2, 0.9, 0.5])
    light /= np.linalg.norm(light)
    lum = 0.62 + 0.34 * np.clip(N @ light, 0, 1)
    V = mesh.V
    depth = (V[F] @ d).mean(axis=1)
    order = np.argsort(depth)
    uv = np.stack([V @ u, V @ v], axis=1)
    c.saveState()
    c.setLineWidth(0.15)
    for i in order:
        tri = uv[F[i]]
        pts = [pv.xy(p[0], p[1]) for p in tri]
        g_ = float(lum[i])
        col = Color(g_, g_, g_ * 1.02 if g_ < 0.98 else g_)
        c.setFillColor(col)
        c.setStrokeColor(col)
        p = c.beginPath()
        p.moveTo(*pts[0])
        p.lineTo(*pts[1])
        p.lineTo(*pts[2])
        p.close()
        c.drawPath(p, fill=1, stroke=1)
    c.restoreState()
