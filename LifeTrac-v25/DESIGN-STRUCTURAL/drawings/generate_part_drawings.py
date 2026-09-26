#!/usr/bin/env python3
"""Generate one PDF engineering drawing per LifeTrac v25 part.

For every part listed in parts_manifest.yaml this script

  1. writes a tiny wrapper .scad that calls the part's module,
  2. renders it to STL with OpenSCAD,
  3. orients it (longest extent along X, thinnest along Z), removes hidden
     lines and finds holes, and
  4. draws a third-angle sheet (front / top / right side + isometric) with a
     title block, notes, overall and running dimensions and a hole table.

Quantities come from the model itself: part modules call
``echo(BOM_PART = "<key>")`` and this script counts those echoes when the
full assembly is evaluated, so the "QTY PER MACHINE" box follows the design.

Outputs (committed by CI on main - see .github/workflows/generate-part-drawings.yml):
  generated/pdf/<ID>_<name>.pdf   one drawing per part
  generated/dxf/<ID>_<name>.dxf   1:1 flat pattern of every plate part (CNC input)
  generated/INDEX.md, bom.csv     bill of materials with links
  generated/CHECKS.md             quantity cross-checks and warnings
  generated/revisions.json        revision letter + date per part
and, not committed: build/LifeTrac_v25_Part_Drawings.pdf (all sheets in one book).

Usage:
  python3 generate_part_drawings.py              # everything
  python3 generate_part_drawings.py --only A4 T1 # a few parts, quick check
"""

import argparse
import csv
import datetime as dt
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
import yaml
from reportlab.pdfgen import canvas

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from partdrawings import sheet as S  # noqa: E402
from partdrawings.drawing import draw_part, plan_part  # noqa: E402
from partdrawings.hlr import detect_circles  # noqa: E402
from partdrawings.mesh import Mesh, load_stl, normalize  # noqa: E402

DESIGN = HERE.parent  # LifeTrac-v25/DESIGN-STRUCTURAL
REV_LETTERS = "ABCDEFGHJKLMNPRTUVWY"  # ASME Y14.35: skip I, O, Q, S, X, Z
BOM_RE = re.compile(r'BOM_PART\s*=\s*"([^"]+)"')


# ----------------------------------------------------------------------------
# Small utilities
# ----------------------------------------------------------------------------

def slug(text):
    return re.sub(r"[^a-z0-9]+", "-", text.lower()).strip("-")


def scad_path(p):
    return str(p).replace("\\", "/")


def run(cmd, cwd=None, timeout=900):
    return subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, timeout=timeout)


def git(*args):
    try:
        r = run(["git", *args], cwd=HERE)
        return r.stdout.strip() if r.returncode == 0 else ""
    except (OSError, subprocess.SubprocessError):
        return ""


def next_rev(rev):
    if not rev:
        return REV_LETTERS[0]
    head, last = rev[:-1], rev[-1]
    i = REV_LETTERS.index(last)
    if i + 1 < len(REV_LETTERS):
        return head + REV_LETTERS[i + 1]
    return (next_rev(head) if head else REV_LETTERS[0]) + REV_LETTERS[0]


def rotation(axis, deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return {
        "x": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]),
        "y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
        "z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]),
    }[axis]


def geometry_id(mesh):
    """Short hash of the oriented part's shape.

    Built from the vertex set and the volume, which do not depend on how the
    faces happen to be triangulated, so another OpenSCAD/CGAL build gives the
    same ID for the same part."""
    verts = np.unique(np.round(mesh.V, 2) + 0.0, axis=0)  # + 0.0 turns -0.0 into 0.0
    h = hashlib.sha1(np.ascontiguousarray(verts).tobytes())
    h.update(("%.0f" % abs(mesh.volume())).encode())
    return h.hexdigest()[:8]


# ----------------------------------------------------------------------------
# Manifest
# ----------------------------------------------------------------------------

class Part:
    def __init__(self, d, cat, defaults):
        self.raw = d
        self.id = str(d["id"])
        self.name = d["name"]
        self.category = d["category"]
        self.cat = cat
        self.stock = d.get("stock", cat.get("stock", ""))
        self.process = d.get("process", cat.get("process", ""))
        self.finish = d.get("finish", cat.get("finish", ""))
        self.density = float(d.get("density", cat.get("density", 7.85)))
        self.source = d["source"]
        self.call = d.get("call")
        self.clip = d.get("clip_box")
        self.rotate = d.get("rotate", [])
        self.count = d.get("count")
        self.count_per = int(d.get("count_per", 1))
        self.qty_from_holes = d.get("qty_from_holes")
        self.qty_manual = d.get("qty")
        self.used_in = d.get("used_in", "")
        self.issues = list(d.get("issues", []))
        self.notes = (list(defaults.get("notes", [])) + list(cat.get("notes", [])) + list(d.get("notes", []))
                      + ["CHECK BEFORE MAKING: " + i for i in self.issues])
        self.options = dict(cat.get("options", {}))
        self.options.update(d.get("options", {}))
        self.aliases = d.get("aliases", [])
        self.filename = "%s_%s" % (self.id, slug(self.name))

    @property
    def count_keys(self):
        if self.count is None:
            return []
        return self.count if isinstance(self.count, list) else [self.count]


def load_manifest(path):
    with open(path) as fh:
        m = yaml.safe_load(fh)
    cats = m.get("categories", {})
    parts = []
    seen = set()
    for d in m["parts"]:
        if d["id"] in seen:
            raise SystemExit("duplicate part id in manifest: %s" % d["id"])
        seen.add(d["id"])
        if d["category"] not in cats:
            raise SystemExit("part %s: unknown category %r" % (d["id"], d["category"]))
        parts.append(Part(d, cats[d["category"]], m.get("defaults", {})))
    return m, parts


# ----------------------------------------------------------------------------
# OpenSCAD
# ----------------------------------------------------------------------------

def wrapper_source(manifest, part, extra_transform=None):
    src = (DESIGN / part.source).resolve()
    params = (DESIGN / manifest["params"]).resolve()
    lines = [
        "// Generated by drawings/generate_part_drawings.py - do not edit",
        "include <%s>" % scad_path(params),
    ]
    if part.call:
        lines.append("use <%s>" % scad_path(src))
        body = part.call.strip()
    else:
        body = "include <%s>" % scad_path(src)
    if part.clip:
        (x0, y0, z0), (x1, y1, z1) = part.clip
        body = "intersection() {\n  %s\n  translate([%s, %s, %s]) cube([(%s)-(%s), (%s)-(%s), (%s)-(%s)]);\n}" % (
            body, x0, y0, z0, x1, x0, y1, y0, z1, z0)
    if extra_transform:
        body = "%s {\n%s\n}" % (extra_transform, body)
    lines.append(body)
    return "\n".join(lines) + "\n"


def render_stl(openscad, manifest, part, build):
    scad = build / "scad" / (part.id + ".scad")
    stl = build / "stl" / (part.id + ".stl")
    deps = build / "stl" / (part.id + ".deps")
    scad.write_text(wrapper_source(manifest, part))
    if stl.exists():
        stl.unlink()
    r = run([openscad, "-o", str(stl), "-d", str(deps), str(scad)], cwd=scad.parent)
    ok = r.returncode == 0 and stl.exists() and stl.stat().st_size > 200
    errors = [ln for ln in r.stderr.splitlines() if "ERROR" in ln]
    return ok, stl, "\n".join(errors[:5]) or r.stderr[-800:]


def render_dxf(openscad, manifest, part, M, thickness, out_path, build):
    m = ",".join("[%s]" % ",".join("%.9g" % x for x in row) for row in M)
    transform = "projection(cut=true) translate([0, 0, %.6f]) multmatrix([%s])" % (-thickness / 2, m)
    scad = build / "scad" / (part.id + "_flat.scad")
    scad.write_text(wrapper_source(manifest, part, transform))
    r = run([openscad, "-o", str(out_path), str(scad)], cwd=scad.parent)
    if r.returncode != 0 or not out_path.exists():
        return False
    add_dxf_units(out_path)
    return True


# OpenSCAD writes DXF with no HEADER, so CAM software has to guess the units
# and some default to inches.  Declare millimetres ($INSUNITS 4 = mm,
# $MEASUREMENT 1 = metric).
DXF_MM_HEADER = ("  0\nSECTION\n  2\nHEADER\n"
                 "  9\n$ACADVER\n  1\nAC1009\n"
                 "  9\n$INSUNITS\n 70\n4\n"
                 "  9\n$MEASUREMENT\n 70\n1\n"
                 "  0\nENDSEC\n")


def add_dxf_units(path):
    text = path.read_text()
    if "$INSUNITS" not in text:
        path.write_text(DXF_MM_HEADER + text)


def select_parts(parts, only):
    """Parts to render for ``--only``: the requested ones, plus the parts a
    requested hardware item counts its holes in (``qty_from_holes``).  Those
    are rendered only for their hole counts; nothing is written for them.
    Returns (to_render, wanted ids)."""
    wanted = set(only)
    unknown = wanted - {p.id for p in parts}
    if unknown:
        raise SystemExit("unknown part id(s): %s" % ", ".join(sorted(unknown)))
    needed = set(wanted)
    for p in parts:
        if p.id in wanted and p.qty_from_holes:
            needed.update(str(x) for x in p.qty_from_holes["parts"])
    return [p for p in parts if p.id in needed], wanted


def count_markers(openscad, manifest, build):
    assembly = (DESIGN / manifest["assembly"]).resolve()
    out = build / "assembly.echo"
    r = run([openscad, "-o", str(out), str(assembly)], cwd=assembly.parent)
    if r.returncode != 0 or not out.exists():
        raise SystemExit("could not evaluate %s:\n%s" % (assembly, r.stderr[-2000:]))
    counts = {}
    for line in out.read_text().splitlines():
        for key in BOM_RE.findall(line):
            counts[key] = counts.get(key, 0) + 1
    return counts


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--manifest", default=str(HERE / "parts_manifest.yaml"))
    ap.add_argument("--out", default=str(HERE / "generated"), help="committed outputs")
    ap.add_argument("--build", default=str(HERE / "build"), help="scratch + book (not committed)")
    ap.add_argument("--only", nargs="*", help="part ids to generate (skips index/book/cleanup)")
    ap.add_argument("--openscad", default=os.environ.get("OPENSCAD", "openscad"))
    ap.add_argument("--paper", default="letter", choices=sorted(S.PAPERS))
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 2)
    ap.add_argument("--strict", action="store_true",
                    help="exit 2 if there are any warnings (unregistered markers, quantity mismatches, DXF failures)")
    args = ap.parse_args()

    if not shutil.which(args.openscad):
        raise SystemExit("OpenSCAD not found (install it or pass --openscad PATH)")

    manifest, parts = load_manifest(args.manifest)
    wanted = None
    if args.only:
        parts, wanted = select_parts(parts, args.only)

    out, build = Path(args.out), Path(args.build)
    for d in (out / "pdf", out / "dxf", build / "scad", build / "stl"):
        d.mkdir(parents=True, exist_ok=True)

    print("Counting BOM_PART markers in %s ..." % manifest["assembly"])
    counts = count_markers(args.openscad, manifest, build)

    print("Rendering %d parts with OpenSCAD (%d jobs) ..." % (len(parts), args.jobs))
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        stl_results = list(pool.map(lambda p: render_stl(args.openscad, manifest, p, build), parts))

    rev_path = out / "revisions.json"
    revisions = json.loads(rev_path.read_text()) if rev_path.exists() else {}
    head_date = git("log", "-1", "--format=%cs") or dt.date.today().isoformat()
    head_sha = git("rev-parse", "--short", "HEAD")

    results, failures, warnings = [], [], []
    dxf_jobs = []
    for part, (ok, stl, err) in zip(parts, stl_results):
        if not ok:
            failures.append((part, err))
            print("  FAIL %-6s %s" % (part.id, err.splitlines()[0] if err else ""))
            continue
        mesh0 = Mesh(load_stl(stl))
        mesh, R, t = normalize(mesh0)
        for axis, deg in part.rotate:
            Rx = rotation(axis, deg)
            mesh, R2, t2 = normalize(mesh, Rx)
            R, t = R2 @ R, R2 @ t + t2
        X, Y, Z = mesh.extents

        # ---- quantity
        counted = sum(counts.get(k, 0) for k in part.count_keys) * part.count_per
        if part.count_keys and counted:
            qty, qty_src = counted, "model"
            if part.qty_manual is not None and int(part.qty_manual) != counted:
                warnings.append("%s: model count %d differs from manifest qty %s" % (part.id, counted, part.qty_manual))
        elif part.qty_manual is not None:
            qty, qty_src = int(part.qty_manual), "manual"
            if part.count_keys:
                warnings.append("%s: marker %s not found in the assembly - using manifest qty %s"
                                % (part.id, "/".join(part.count_keys), part.qty_manual))
        else:
            qty, qty_src = 0, "unknown"
            if not part.qty_from_holes:  # filled in from hole counts below
                warnings.append("%s: no quantity (add a BOM_PART marker or a qty in the manifest)" % part.id)

        mass = abs(mesh.volume()) * part.density * 1e-6
        cat = part.category
        if cat == "plate":
            size_label, size = "BLANK SIZE", "%s x %s" % (S.fmt_mm(X), S.fmt_mm(Y))
        elif cat in ("angle", "tube", "bar"):
            size_label, size = "CUT LENGTH", "%s [%s]" % (S.fmt_mm(X), S.fmt_frac_in(X))
        else:
            size_label, size = "SIZE", "%s x %s x %s" % (S.fmt_mm(X), S.fmt_mm(Y), S.fmt_mm(Z))
        if part.raw.get("size"):
            size_label, size = part.raw.get("size_label", "SIZE"), str(part.raw["size"])

        opts = dict(part.options)
        notes = [n.replace("{dxf}", part.filename + ".dxf") for n in part.notes]
        if cat == "plate":
            opts.setdefault("drill_below", Z)  # plasma holes smaller than thickness get drilled
        gid = geometry_id(mesh)
        info = {
            "project": manifest.get("project", "LIFETRAC v25"),
            "id": part.id, "name": part.name, "stock": part.stock, "category": cat,
            "qty": str(qty) if qty else "?",
            "qty_note": {"model": " (COUNTED IN MODEL)", "manual": " (FROM MANIFEST)"}.get(qty_src, ""),
            "process": part.process, "finish": part.finish,
            "size_label": size_label, "size": size, "mass": "%.2f kg" % mass,
            "geom_id": gid,
            "source": "%s : %s" % (part.source, (part.call or "(file)").strip().rstrip(";")),
            "used_in": part.used_in, "notes": notes,
            "summary": [("STOCK", part.stock), (size_label, size), ("MASS EACH", "%.2f kg" % mass),
                        ("MASS PER MACHINE", "%.2f kg" % (mass * qty)), ("QTY PER MACHINE", str(qty))],
        }
        if cat == "plate" and opts.get("dxf", True):
            M = np.eye(4)
            M[:3, :3], M[:3, 3] = R, t
            dxf_jobs.append((part, M, Z))

        # Holes seen face-on in the front/top views (end views show bores).
        bolt_holes = [2 * c["r"] for v in ("front", "top") for c in detect_circles(mesh, v) if c["hole"]]
        results.append({"part": part, "mesh": mesh, "info": info, "opts": opts, "qty": qty,
                        "qty_src": qty_src, "mass": mass, "extents": (X, Y, Z), "bolt_holes": bolt_holes})

    # ---- quantities estimated from hole counts (hardware not yet in the model)
    by_id = {r["part"].id: r for r in results}
    for res in results:
        rule = res["part"].qty_from_holes
        if not rule:
            continue
        diameters = rule["diameters"] if isinstance(rule["diameters"], list) else [rule["diameters"]]
        n = 0
        for pid in rule["parts"]:
            src = by_id.get(pid)
            if src is None:
                warnings.append("%s: qty_from_holes names unknown part %s" % (res["part"].id, pid))
                continue
            n += src["qty"] * sum(1 for h in src["bolt_holes"] for d in diameters if abs(h - d) < 0.3)
        qty = int(n * rule.get("per_hole", 1))
        res.update(qty=qty, qty_src="holes")
        res["info"].update(qty=str(qty), qty_note=" (EST. FROM HOLE COUNT)")
        res["info"]["summary"] = [(k, str(qty) if k == "QTY PER MACHINE" else (
            "%.2f kg" % (res["mass"] * qty) if k == "MASS PER MACHINE" else v)) for k, v in res["info"]["summary"]]

    if wanted is not None:  # --only: the extra parts were rendered just for their hole counts
        results = [r for r in results if r["part"].id in wanted]
        dxf_jobs = [j for j in dxf_jobs if j[0].id in wanted]

    # ---- plan every sheet (views, scale, holes ...) before deciding revisions
    for res in results:
        res["plan"] = plan_part(res["mesh"], res["opts"], args.paper, res["part"].category)

    # ---- revision bookkeeping: the letter goes up when anything printed on
    # the sheet changes - title block and notes (everything in `info` except
    # the revision and date themselves) or the planned views, dimensions,
    # holes, scale and paper.  Pure styling changes to the generator do not.
    for res in results:
        info = res["info"]
        printed = {k: v for k, v in info.items() if k not in ("rev", "date")}
        content = hashlib.sha1(json.dumps({"info": printed, "sheet": res["plan"]["fingerprint"]},
                                          sort_keys=True).encode()).hexdigest()[:12]
        prev = revisions.get(info["id"])
        if prev and prev.get("content") == content:
            rev, date = prev["rev"], prev["date"]
        else:
            rev = next_rev(prev["rev"]) if prev else REV_LETTERS[0]
            date = head_date
        revisions[info["id"]] = {"rev": rev, "date": date, "content": content, "geometry": info["geom_id"]}
        info.update(rev=rev, date=date)

    # ---- DXF flat patterns (plates)
    if dxf_jobs:
        print("Exporting %d plate flat patterns (DXF) ..." % len(dxf_jobs))
        with ThreadPoolExecutor(max_workers=args.jobs) as pool:
            dxf_ok = list(pool.map(lambda j: render_dxf(args.openscad, manifest, j[0], j[1], j[2],
                                                        out / "dxf" / (j[0].filename + ".dxf"), build), dxf_jobs))
        for (part, _, _), ok in zip(dxf_jobs, dxf_ok):
            if not ok:
                warnings.append("%s: DXF export failed" % part.id)

    # ---- per-part PDFs
    print("Drawing sheets ...")
    for res in results:
        part = res["part"]
        pdf = out / "pdf" / (part.filename + ".pdf")
        c = canvas.Canvas(str(pdf), pagesize=S.PAPERS[args.paper], invariant=1)
        c.setTitle("%s %s" % (part.id, part.name))
        c.setAuthor(manifest.get("project", "LifeTrac"))
        c.setSubject("Part drawing, rev %s, geometry %s" % (res["info"]["rev"], res["info"]["geom_id"]))
        r = draw_part(c, res["mesh"], res["plan"], res["info"])
        c.save()
        res.update(sheets=r["sheets"], holes=r["holes"], scale=r["scale"])
        print("  %-6s %-42s qty %-4s scale %-6s holes %3d  sheets %d" % (
            part.id, part.name[:42], res["info"]["qty"], S.scale_label(r["scale"]), len(r["holes"]), r["sheets"]))

    if not args.only:
        # Remove drawings of parts that are no longer in the manifest / renamed.
        keep = {res["part"].filename for res in results} | {p.filename for p, _ in failures}
        for sub, ext in (("pdf", ".pdf"), ("dxf", ".dxf")):
            for f in (out / sub).glob("*" + ext):
                if f.stem not in keep:
                    f.unlink()
        ids = {p.id for p in parts}
        revisions = {k: v for k, v in revisions.items() if k in ids}
        rev_path.write_text(json.dumps(revisions, indent=1, sort_keys=True) + "\n")

        registered = {k for p in parts for k in p.count_keys}
        ignored = manifest.get("ignore_markers", {}) or {}
        for key in sorted(set(counts) - registered - set(ignored)):
            warnings.append("marker %r appears %d time(s) in the assembly but no manifest part uses it"
                            % (key, counts[key]))
        write_index(out, manifest, results, failures)
        write_checks(out, results, failures, warnings, counts, ignored)
        write_book(build / "LifeTrac_v25_Part_Drawings.pdf", manifest, results, args.paper, head_sha, head_date)

    for w in warnings:
        print("WARNING:", w)
    # Exit codes: 0 clean; 2 finished with warnings (--strict only); 3 finished
    # but some parts failed to render.  In every case all outputs were written
    # (failed parts keep their previous drawing); a crash exits 1.
    if failures:
        print("%d part(s) failed to render" % len(failures))
        return 3
    if args.strict and warnings:
        return 2
    return 0


# ----------------------------------------------------------------------------
# Index, checks and book
# ----------------------------------------------------------------------------

CATEGORY_ORDER = ["plate", "angle", "tube", "bar", "lug", "ring", "printed", "fastener", "purchased"]


def _sorted(results):
    def key(r):
        cat = r["part"].category
        num = [int(x) if x.isdigit() else x for x in re.split(r"(\d+)", r["part"].id)]
        return (CATEGORY_ORDER.index(cat) if cat in CATEGORY_ORDER else 99, num)
    return sorted(results, key=key)


def write_index(out, manifest, results, failures):
    rows = _sorted(results)
    with open(out / "bom.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["id", "name", "category", "stock", "qty_per_machine", "qty_source", "size",
                    "mass_each_kg", "mass_total_kg", "holes", "rev", "geometry_id", "drawing", "flat_pattern"])
        for r in rows:
            p, i = r["part"], r["info"]
            dxf = ("dxf/%s.dxf" % p.filename) if (out / "dxf" / (p.filename + ".dxf")).exists() else ""
            w.writerow([p.id, p.name, p.category, p.stock, r["qty"], r["qty_src"], i["size"],
                        "%.2f" % r["mass"], "%.2f" % (r["mass"] * r["qty"]), len(r["holes"]),
                        i["rev"], i["geom_id"], "pdf/%s.pdf" % p.filename, dxf])

    total_mass = sum(r["mass"] * r["qty"] for r in rows)
    total_pieces = sum(r["qty"] for r in rows)
    lines = [
        "# LifeTrac v25 - Part Drawings Index",
        "",
        "> **Generated file - do not edit.** Produced by "
        "[`../generate_part_drawings.py`](../generate_part_drawings.py) from "
        "[`../parts_manifest.yaml`](../parts_manifest.yaml) and the OpenSCAD model. "
        "See [`../README.md`](../README.md).",
        "",
        "Plain quantities are counted from `BOM_PART` markers in the assembly. Quantities marked "
        "*(manifest)* are typed in by hand; *(est. from holes)* are estimated from hole counts "
        "(`qty_from_holes`). Check both before ordering.",
        "",
        "**%d unique parts, %d pieces, %.0f kg of fabricated/purchased parts (calculated).**" % (
            len(rows), total_pieces, total_mass),
        "",
    ]
    current = None
    for r in rows:
        p, i = r["part"], r["info"]
        if p.category != current:
            current = p.category
            label = manifest["categories"][current].get("label", current)
            lines += ["", "## %s" % label, "",
                      "| Part | Name | Stock | Qty | Size | Mass each | Rev | Drawing | CNC |",
                      "|---|---|---|---:|---|---:|:-:|---|---|"]
        dxf = (out / "dxf" / (p.filename + ".dxf")).exists()
        lines.append("| **%s** | %s | %s | %s%s | %s | %.2f kg | %s | [PDF](pdf/%s.pdf) | %s |" % (
            p.id, p.name, p.stock, r["qty"], {"model": "", "holes": " *(est. from holes)*"}.get(r["qty_src"], " *(manifest)*"),
            i["size"], r["mass"], i["rev"], p.filename,
            "[DXF](dxf/%s.dxf)" % p.filename if dxf else ""))
    if failures:
        lines += ["", "## Not drawn (render failed)", ""]
        lines += ["- **%s** %s" % (p.id, p.name) for p, _ in failures]
    (out / "INDEX.md").write_text("\n".join(lines) + "\n")


def write_checks(out, results, failures, warnings, counts, ignored):
    lines = ["# Part drawing checks", "",
             "> Generated file - do not edit. Regenerated with the drawings on every design change.", ""]
    lines += ["## Warnings", ""]
    lines += ["- %s" % w for w in warnings] or ["- none"]
    lines += ["", "## Known model issues (from `issues:` in parts_manifest.yaml)", "",
              "These are printed on the affected drawings as CHECK BEFORE MAKING notes. "
              "Fix the model, then delete the issue from the manifest.", ""]
    issues = [(r["part"].id, i) for r in _sorted(results) for i in r["part"].issues]
    lines += ["- **%s**: %s" % (pid, i) for pid, i in issues] or ["- none"]
    lines += ["", "## Modelled but deliberately not drawn (`ignore_markers`)", ""]
    lines += ["- `%s` x%d: %s" % (k, counts.get(k, 0), ignored[k]) for k in sorted(ignored)] or ["- none"]
    lines += ["", "## Render failures", ""]
    lines += ["- **%s**: `%s`" % (p.id, (e.splitlines() or [""])[0]) for p, e in failures] or ["- none"]

    drill = []
    for r in _sorted(results):
        small = {}
        for h in r["holes"]:
            if h.get("drill"):
                small.setdefault(round(h["d"], 1), []).append(h["tag"])
        for d, tags in sorted(small.items()):
            drill.append("- **%s**: %d x Ø%.1f mm in %.1f mm plate (%s)" % (
                r["part"].id, len(tags), d, r["extents"][2], ", ".join(tags)))
    lines += ["", "## Plate holes smaller than the plate thickness (drill after cutting)", ""]
    lines += drill or ["- none"]

    # Fastener estimate: every bolted joint in the frame passes through an
    # angle-iron or tube hole, so their hole counts bound the bolt count.
    by_d = {}
    for r in results:
        if r["part"].category not in ("angle", "tube"):
            continue
        for d in r["bolt_holes"]:
            key = round(d, 1)
            by_d.setdefault(key, {"holes": 0, "parts": set()})
            by_d[key]["holes"] += r["qty"]
            by_d[key]["parts"].add(r["part"].id)
    lines += ["", "## Hole count by diameter in angle and tube parts (x quantity)", "",
              "Rough fastener estimate: one bolt, one nut and two washers per angle-iron hole; tube "
              "holes usually share a bolt with an angle hole. Bolt length depends on the joint grip.", "",
              "| Hole Ø (mm) | Hole Ø (in) | Holes per machine | Parts |", "|---:|---:|---:|---|"]
    for d in sorted(by_d):
        lines.append("| %.1f | %.3f | %d | %s |" % (d, d / 25.4, by_d[d]["holes"], ", ".join(sorted(by_d[d]["parts"]))))

    lines += ["", "## BOM_PART markers counted in the assembly", "", "| Marker | Count |", "|---|---:|"]
    lines += ["| `%s` | %d |" % (k, counts[k]) for k in sorted(counts)] or ["| (none) | |"]
    (out / "CHECKS.md").write_text("\n".join(lines) + "\n")


def book_page_starts(index_pages, sheets):
    """First page number of each part in the book, and the total page count.
    Pages 1..index_pages are the index; the first drawing is the next page."""
    starts, last = [], index_pages
    for n in sheets:
        starts.append(last + 1)
        last += n
    return starts, last


def write_book(path, manifest, results, paper, head_sha, head_date):
    rows = _sorted(results)
    W, H = S.PAPERS[paper]
    c = canvas.Canvas(str(path), pagesize=(W, H), invariant=1)
    c.setTitle("LifeTrac v25 part drawings")
    per_page = 34
    index_pages = max(1, -(-len(rows) // per_page))
    starts, page = book_page_starts(index_pages, [r["sheets"] for r in rows])
    for k in range(index_pages):
        c.setFont("Helvetica-Bold", 20)
        c.drawString(50, H - 60, "LifeTrac v25 - Part Drawings")
        c.setFont("Helvetica", 9)
        c.drawString(50, H - 78, "%s  |  model commit %s (%s)  |  %d unique parts, %d pieces" % (
            manifest.get("project", ""), head_sha or "-", head_date, len(rows), sum(r["qty"] for r in rows)))
        y = H - 104
        c.setFont("Helvetica-Bold", 8)
        for x, t in ((50, "PART"), (100, "NAME"), (330, "STOCK"), (560, "QTY"), (600, "REV"), (640, "PAGE")):
            c.drawString(x, y, t)
        c.setFont("Helvetica", 8)
        for r, start in list(zip(rows, starts))[k * per_page:(k + 1) * per_page]:
            y -= 13
            p = r["part"]
            c.drawString(50, y, p.id)
            c.drawString(100, y, p.name[:48])
            c.drawString(330, y, p.stock[:50])
            c.drawRightString(580, y, str(r["qty"]))
            c.drawString(604, y, r["info"]["rev"])
            c.drawRightString(660, y, str(start))
        c.setFont("Helvetica", 7)
        c.drawString(50, 30, "Generated from the OpenSCAD model by drawings/generate_part_drawings.py. Page %d." % (k + 1))
        c.showPage()
    for r in rows:
        draw_part(c, r["mesh"], r["plan"], r["info"])
    c.save()
    print("Book: %s (%d pages)" % (path, page))


if __name__ == "__main__":
    sys.exit(main())
