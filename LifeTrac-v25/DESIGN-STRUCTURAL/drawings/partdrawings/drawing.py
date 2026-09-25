"""Compose a complete part drawing (one or more sheets) on a reportlab canvas."""

import numpy as np

from . import sheet as S
from .hlr import draw_view

ORTHO = ("front", "top", "right")
VIEW_LABEL = {"front": "FRONT", "top": "TOP", "right": "RIGHT SIDE"}


def _ordinates(pv, rows, max_positions=12):
    """Unique hole X positions in a view, if few enough for running dims."""
    xs = {}
    for r in rows:
        if r["pv"] is pv:
            xs.setdefault(round(r["x"], 1), r["x"])  # dedupe, keep the exact value
    xs = [xs[k] for k in sorted(xs)]
    return xs if 0 < len(xs) <= max_positions else []


def _draw_ordinates(c, pv, xs, below=True):
    """Running (ordinate) dimensions from the left end of the view, as used
    for marking hole positions on angle and tube with a tape measure."""
    x0, y0, x1, y1 = pv.box
    c.saveState()
    c.setLineWidth(S.LINE_THIN)
    c.setFont("Helvetica", 5.2)
    base = y0 - 4 if below else y1 + 4
    tip = y0 - 14 if below else y1 + 14
    for val in [0.0] + list(xs):
        x = x0 + val * pv.s
        c.line(x, base, x, tip)
        text = S.fmt_dual(val) if val else "0"
        c.saveState()
        if below:
            c.translate(x + 1.9, tip - 1.5)
            c.rotate(90)
            c.drawRightString(0, 0, text)
        else:
            c.translate(x + 1.9, tip + 1.5)
            c.rotate(90)
            c.drawString(0, 0, text)
        c.restoreState()
    c.restoreState()


ORD_SPACE = 58  # points reserved for a row of rotated ordinate labels


def render_part(c, mesh, info, opts, paper="letter"):
    """Draw all sheets for one part.  ``mesh`` must already be normalised
    (canonical orientation, bounding-box min at the origin)."""
    g = S.SheetGeometry(paper)
    views = [v for v in opts.get("views", ["front", "top", "right", "iso"]) if v in ORTHO + ("iso",)]
    ortho = [v for v in ORTHO if v in views]
    hidden = opts.get("hidden_lines", True)
    running = opts.get("running_dims", False)

    vds = {n: draw_view(mesh, n) for n in ortho}
    X, Y, Z = mesh.extents

    # Provisional hole list (to know whether running dimensions need room).
    pre_rows = S.collect_holes({n: _as_pv(vd) for n, vd in vds.items()}) if opts.get("hole_table", True) else []
    ord_front = running and "front" in vds and 0 < len({round(r["x"], 1) for r in pre_rows if r["view"] == "front"}) <= 12
    ord_top = running and "top" in vds and 0 < len({round(r["x"], 1) for r in pre_rows if r["view"] == "top"}) <= 12

    ax0, ay0, ax1, ay1 = g.views_box
    pad_l, pad_r = 44, 14
    pad_b = 44 + (ORD_SPACE if ord_front else 0)
    pad_t = 16 + (ORD_SPACE if ord_top else 0)
    gap = 44 + (ORD_SPACE if ord_top else 0)
    scale = S.choose_scale((X, Y, Z), ax1 - ax0, ay1 - ay0, ortho, pad_l + pad_r, pad_b + pad_t, gap)
    s = scale * S.PT_PER_MM

    tot_w = X * s + ((gap + Z * s) if "right" in vds else 0)
    tot_h = Y * s + ((gap + Z * s) if "top" in vds else 0)
    ox = ax0 + pad_l + max(0, (ax1 - ax0 - pad_l - pad_r - tot_w) / 2)
    oy = ay0 + pad_b + max(0, (ay1 - ay0 - pad_b - pad_t - tot_h) / 2)

    placed = {}
    if "front" in vds:
        placed["front"] = S.PlacedView(vds["front"], ox, oy, s)
    if "top" in vds:
        placed["top"] = S.PlacedView(vds["top"], ox, oy + Y * s + gap, s)
    if "right" in vds:
        placed["right"] = S.PlacedView(vds["right"], ox + X * s + gap, oy, s)

    thick = opts.get("thickness")
    rows = S.collect_holes(placed, drill_below=opts.get("drill_below")) if opts.get("hole_table", True) else []

    # ------------------------------------------------------------------ pages
    x0r, y0r, x1r, y1r = g.right_box
    iso_h = (y1r - y0r) * (0.42 if rows else 0.62) if "iso" in views else 0
    table_top = y1r - iso_h - 4
    rh = 8.4
    first_fit = max(0, int((table_top - 12 - rh - (y0r + 6)) // rh))
    rest = max(0, len(rows) - first_fit)
    cont_cols = 3
    cont_fit = cont_cols * max(1, int(((ay1 - ay0) - 30) // rh))
    n_sheets = 1 + (-(-rest // cont_fit) if rest else 0)

    # ------------------------------------------------------------------ sheet 1
    S.draw_frame(c, g)
    S.draw_notes(c, g, info["notes"])
    S.draw_title_block(c, g, dict(info, scale=S.scale_label(scale)), 1, n_sheets)

    for name, pv in placed.items():
        S.draw_lines(c, pv, hidden=hidden or name == "right" and info.get("category") in ("angle", "tube"))

    # Round outlines are polygons in the model, so a Ø38.1 pin measures 38.0
    # across the flats.  Report such sizes as the true diameter.
    diameters = sorted({round(2 * c["r"], 4) for vd in vds.values() for c in vd.circles if not c["hole"]})

    def size_text(value, suffix=""):
        for dia in diameters:
            if abs(value - dia) <= 0.005 * dia:
                return "Ø" + S.fmt_dual(dia) + suffix
        return S.fmt_dual(value) + suffix

    fx0, fy0, fx1, fy1 = placed["front"].box if "front" in placed else (ox, oy, ox, oy)
    below = fy0 - 16 - (ORD_SPACE if ord_front else 0)
    S.dim_h(c, fx0, fx1, fy0, below, size_text(X))
    S.dim_v(c, fy0, fy1, fx0, fx0 - 16, size_text(Y))
    label_y = {"front": below - 13}
    if "top" in placed:
        tx0, ty0, tx1, ty1 = placed["top"].box
        S.dim_v(c, ty0, ty1, tx0, tx0 - 16, size_text(Z, " THK" if thick else ""))
        label_y["top"] = ty0 - 11
    if "right" in placed:
        rx0, ry0, rx1, ry1 = placed["right"].box
        if "top" not in placed:
            S.dim_h(c, rx0, rx1, ry0, ry0 - 16, size_text(Z, " THK" if thick else ""))
            label_y["right"] = ry0 - 29
        else:
            label_y["right"] = ry0 - 11
    if ord_front:
        _draw_ordinates(c, placed["front"], _ordinates(placed["front"], rows), below=True)
    if ord_top:
        _draw_ordinates(c, placed["top"], _ordinates(placed["top"], rows), below=False)

    c.saveState()
    c.setFont("Helvetica-Bold", 6.5)
    for name, pv in placed.items():
        bx0, by0, bx1, by1 = pv.box
        c.drawCentredString((bx0 + bx1) / 2, label_y.get(name, by0 - 11), VIEW_LABEL[name])
    c.setFont("Helvetica", 6)
    c.drawString(ax0 + 5, ay1 - 10, "SCALE %s  -  VIEWS IN THIRD-ANGLE PROJECTION" % S.scale_label(scale))
    c.restoreState()

    S.draw_hole_tags(c, rows)

    if "iso" in views:
        S.draw_iso(c, mesh, (x0r, y1r - iso_h, x1r, y1r), shade=opts.get("shade_iso", True))
        c.saveState()
        c.setLineWidth(0.4)
        c.line(x0r, y1r - iso_h, x1r, y1r - iso_h)
        c.restoreState()

    remaining = rows
    if rows:
        remaining = S.draw_hole_table(c, rows, x0r + 6, table_top, x1r - x0r - 12, y0r + 6)
    elif "iso" in views:
        _draw_summary(c, info, (x0r, y0r, x1r, y1r - iso_h))
    c.showPage()

    # --------------------------------------------------------- continuation
    sheet_no = 1
    while remaining:
        sheet_no += 1
        S.draw_frame(c, g)
        S.draw_notes(c, g, info["notes"])
        S.draw_title_block(c, g, dict(info, scale="-"), sheet_no, n_sheets)
        x0, y0, x1, y1 = ax0, ay0, x1r, ay1
        colw = (x1 - x0 - 12) / cont_cols
        for k in range(cont_cols):
            if not remaining:
                break
            remaining = S.draw_hole_table(
                c, remaining, x0 + 6 + k * colw, y1 - 4, colw - 8, y0 + 6,
                title="HOLE TABLE (CONT.)")
        c.showPage()
    return {"scale": scale, "holes": rows, "sheets": n_sheets}


def _as_pv(vd):
    return S.PlacedView(vd, 0.0, 0.0, 1.0)


def _draw_summary(c, info, box):
    """Key facts in the right column when a part has no holes to tabulate."""
    x0, y0, x1, y1 = box
    c.saveState()
    y = y1 - 16
    c.setFont("Helvetica-Bold", 7)
    c.drawString(x0 + 8, y, "PART SUMMARY")
    c.setFont("Helvetica", 6.5)
    width = x1 - x0 - 88
    for label, val in info.get("summary", []):
        y -= 11
        c.setFillColor(S.GREY)
        c.drawString(x0 + 8, y, label)
        c.setFillColor(S.black)
        line = ""
        for word in str(val).split():  # wrap long values (e.g. a full fastener callout)
            trial = (line + " " + word).strip()
            if line and c.stringWidth(trial, "Helvetica", 6.5) > width:
                c.drawString(x0 + 80, y, line)
                y -= 8.5
                line = word
            else:
                line = trial
        c.drawString(x0 + 80, y, line)
    c.restoreState()
