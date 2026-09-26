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
A marker that no manifest entry claims yet is drawn anyway when it names a
part number and one no-argument module draws just that part (see
discover_new_parts); CHECKS.md then suggests its manifest entry.

Outputs (committed by CI on main - see .github/workflows/generate-part-drawings.yml):
  generated/pdf/<ID>_<name>.pdf   one drawing per part
  generated/dxf/<ID>_<name>.dxf   1:1 flat pattern of every plate part (CNC input)
  generated/INDEX.md, bom.csv     bill of materials with links; INDEX.md starts with
                                  totals per category and per stock size
  generated/CHECKS.md             quantity cross-checks and warnings
  generated/revisions.json        revision letter + date per part
and, not committed: build/LifeTrac_v25_Part_Drawings.pdf (all sheets in one book).

Usage:
  python3 generate_part_drawings.py              # everything
  python3 generate_part_drawings.py --only A4 T1 # a few parts, quick check
"""

import argparse
import collections
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

import ezdxf
import numpy as np
import yaml
from ezdxf.addons import Importer
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
        self.auto = bool(d.get("auto"))  # found in the model, not in the manifest yet
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
    # A failed export must not leave the previous flat pattern next to a
    # drawing that may have changed, so remove it first and on any failure.
    out_path.unlink(missing_ok=True)
    m = ",".join("[%s]" % ",".join("%.9g" % x for x in row) for row in M)
    transform = "projection(cut=true) translate([0, 0, %.6f]) multmatrix([%s])" % (-thickness / 2, m)
    scad = build / "scad" / (part.id + "_flat.scad")
    raw = build / "scad" / (part.id + "_flat.dxf")
    raw.unlink(missing_ok=True)
    scad.write_text(wrapper_source(manifest, part, transform))
    r = run([openscad, "-o", str(raw), str(scad)], cwd=scad.parent)
    if r.returncode != 0 or not raw.exists():
        return False
    try:
        write_mm_dxf(raw, out_path)
    except Exception:
        out_path.unlink(missing_ok=True)
        return False
    return True


# No timestamps or GUIDs in the files ezdxf writes, so a DXF only changes
# when its geometry does.
ezdxf.options.write_fixed_meta_data_for_testing = True


def write_mm_dxf(src, dst):
    """Rewrite OpenSCAD's unitless R12 DXF as an R2000 DXF in millimetres.

    CAM software that has to guess the units may assume inches.  The units
    header ($INSUNITS 4 = mm, $MEASUREMENT 1 = metric) only exists from R2000
    on, and an R2000 file needs handles, tables and objects that OpenSCAD does
    not write, so ezdxf builds a complete one around the same entities."""
    doc = ezdxf.new("R2000", units=ezdxf.units.MM)
    importer = Importer(ezdxf.readfile(str(src)), doc)
    importer.import_modelspace()
    importer.finalize()
    doc.saveas(str(dst))


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


def echo_markers(openscad, scad, out):
    """Evaluate a .scad file without rendering any geometry and count the
    BOM_PART markers it echoes.  Returns (counts, stderr); counts is None if
    OpenSCAD fails."""
    out.unlink(missing_ok=True)
    r = run([openscad, "-o", str(out), str(scad)], cwd=scad.parent)
    if r.returncode != 0 or not out.exists():
        return None, r.stderr
    counts = {}
    for line in out.read_text().splitlines():
        for key in BOM_RE.findall(line):
            counts[key] = counts.get(key, 0) + 1
    return counts, r.stderr


def count_markers(openscad, manifest, build):
    assembly = (DESIGN / manifest["assembly"]).resolve()
    counts, err = echo_markers(openscad, assembly, build / "assembly.echo")
    if counts is None:
        raise SystemExit("could not evaluate %s:\n%s" % (assembly, err[-2000:]))
    return counts


# ----------------------------------------------------------------------------
# New parts: draw markers that no manifest entry claims yet
# ----------------------------------------------------------------------------

PART_ID_RE = re.compile(r"^([A-Z]+)\d+(?:-\d+)?$")  # P20, A11, A6-4
MODULE_RE = re.compile(r"\bmodule\s+([A-Za-z_]\w*)\s*\(")
ANY_MARKER_RE = re.compile(r"\bBOM_PART\s*=")
NEW_PART_ISSUE = ("NEW PART, DRAWN AUTOMATICALLY BY CALLING %s WITH ITS DEFAULT ARGUMENTS. NAME, STOCK AND "
                  "USED IN ARE PLACEHOLDERS UNTIL THE PART HAS AN ENTRY IN parts_manifest.yaml "
                  "(generated/CHECKS.md HAS ONE READY TO PASTE).")
ScadModule = collections.namedtuple("ScadModule", "name params_start params_end start end")


def scad_code_mask(src):
    """For each character of an OpenSCAD source: True if it is code, False
    if it is inside a comment or a string."""
    mask = [True] * len(src)
    i, n = 0, len(src)
    while i < n:
        if src.startswith("//", i):
            j = src.find("\n", i)
            j = n if j < 0 else j
        elif src.startswith("/*", i):
            j = src.find("*/", i + 2)
            j = n if j < 0 else j + 2
        elif src[i] == '"':
            j = i + 1
            while j < n and src[j] != '"':
                j += 2 if src[j] == "\\" else 1
            j = min(j + 1, n)
        else:
            i += 1
            continue
        mask[i:j] = [False] * (j - i)
        i = j
    return mask


def _close(src, mask, i):
    """Index just past the bracket that closes the one at src[i]."""
    pairs = {"(": ")", "[": "]", "{": "}"}
    stack = []
    for j in range(i, len(src)):
        if mask[j] and src[j] in pairs:
            stack.append(pairs[src[j]])
        elif mask[j] and stack and src[j] == stack[-1]:
            stack.pop()
            if not stack:
                return j + 1
    return len(src)


def scad_modules(src, mask):
    """Every module definition in an OpenSCAD source.  start..end spans the
    whole definition: its { block } or single statement."""
    mods = []
    for m in MODULE_RE.finditer(src):
        if not mask[m.start()]:
            continue
        params_end = _close(src, mask, m.end() - 1)
        j = params_end
        while j < len(src):
            if mask[j] and src[j] == "{":
                j = _close(src, mask, j)
                break
            if mask[j] and src[j] == ";":
                j += 1
                break
            j += 1
        mods.append(ScadModule(m.group(1), m.end(), params_end - 1, m.start(), j))
    return mods


def params_have_defaults(src, mask, start, end):
    """True if every parameter in src[start:end] has a default value, so the
    module can be called with no arguments."""
    depth, text, default = 0, False, False
    for j in range(start, end):
        c = src[j]
        if not mask[j] or c.isspace():
            continue
        if c in "([{":
            depth += 1
        elif c in ")]}":
            depth -= 1
        elif depth == 0 and c == ",":
            if text and not default:
                return False
            text = default = False
            continue
        elif depth == 0 and c == "=":
            default = True
        text = True
    return default or not text


def part_name(module, part_id):
    """Readable name from a module name: part_a11_seat_bracket -> Seat bracket."""
    words = re.sub(r"^part_", "", module)
    words = re.sub(r"^%s_" % re.escape(part_id.lower().replace("-", "_")), "", words)
    words = words.replace("_", " ").strip()
    return words[:1].upper() + words[1:] if words else part_id


def discover_new_parts(manifest, parts, counts, openscad=None, build=None, design=DESIGN):
    """Draw BOM_PART markers that no manifest part claims yet.

    A marker becomes a new, provisional part when its key is a part number
    whose letter prefix has a category in the manifest's `auto_categories`
    (A11 -> angle), and exactly one module under openscad/ echoes it, as its
    only marker, and can be called with no arguments.  With `openscad`, that
    call is also evaluated on its own and must echo the marker exactly once
    and no other marker, so it draws one part and nothing else.  Returns
    (new parts, {marker: reason}) where the reasons explain the markers left
    undrawn."""
    cats = manifest.get("categories", {})
    prefixes = manifest.get("auto_categories") or {}
    ignored = manifest.get("ignore_markers") or {}
    claimed = {k for p in parts for k in p.count_keys}
    ids = {p.id for p in parts}
    todo = sorted(k for k in counts if k not in claimed and k not in ignored)
    new, why = [], {}
    if not todo:
        return new, why
    files = []
    for f in sorted((Path(design) / "openscad").rglob("*.scad")):
        src = f.read_text(errors="replace")
        mask = scad_code_mask(src)
        files.append((f, src, mask, scad_modules(src, mask)))
    for key in todo:
        m = PART_ID_RE.match(key)
        cat = prefixes.get(m.group(1)) if m else None
        if not m:
            why[key] = "it is not a plain part number, so it needs a manifest entry with a `call`"
            continue
        if key in ids:
            why[key] = "the manifest already has a part %s that counts a different marker" % key
            continue
        if cat not in cats:
            why[key] = "there is no category for the prefix %s (`auto_categories` in the manifest)" % m.group(1)
            continue
        pattern = re.compile(r'echo\s*\(\s*BOM_PART\s*=\s*"%s"\s*\)' % re.escape(key))
        hits = []
        for f, src, mask, mods in files:
            for h in pattern.finditer(src):
                if mask[h.start()]:
                    inside = [md for md in mods if md.start <= h.start() < md.end]
                    hits.append((f, src, mask, max(inside, key=lambda md: md.start) if inside else None))
        if len(hits) != 1:
            why[key] = ('no `echo(BOM_PART = "%s");` statement was found under openscad/' % key if not hits
                        else "it is echoed in %d places" % len(hits))
            continue
        f, src, mask, mod = hits[0]
        if mod is None:
            why[key] = "it is echoed outside a module"
            continue
        markers = [h for h in ANY_MARKER_RE.finditer(src, mod.start, mod.end) if mask[h.start()]]
        if len(markers) > 1:
            why[key] = "module %s() echoes more than one marker, so it draws more than one part" % mod.name
            continue
        if not params_have_defaults(src, mask, mod.params_start, mod.params_end):
            why[key] = "module %s() needs arguments" % mod.name
            continue
        d = {"id": key, "name": part_name(mod.name, key), "category": cat, "stock": "NOT SPECIFIED YET",
             "source": os.path.relpath(f, design).replace(os.sep, "/"), "call": "%s();" % mod.name,
             "count": key, "used_in": "NOT SPECIFIED YET", "issues": [NEW_PART_ISSUE % (mod.name + "()")],
             "auto": True}
        part = Part(d, cats[cat], manifest.get("defaults", {}))
        if openscad:
            scad = Path(build) / "scad" / (key + "_markers.scad")
            scad.write_text(wrapper_source(manifest, part))
            echoed, _ = echo_markers(openscad, scad, scad.with_suffix(".echo"))
            if echoed is None:
                why[key] = "%s() fails when it is called on its own" % mod.name
                continue
            if echoed != {key: 1}:
                why[key] = "calling %s() on its own echoes %s, so it does not draw exactly one part" % (
                    mod.name, ", ".join("%s x%d" % kv for kv in sorted(echoed.items())) or "no marker")
                continue
        new.append(part)
    return new, why


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
    out, build = Path(args.out), Path(args.build)
    for d in (out / "pdf", out / "dxf", build / "scad", build / "stl"):
        d.mkdir(parents=True, exist_ok=True)

    print("Counting BOM_PART markers in %s ..." % manifest["assembly"])
    counts = count_markers(args.openscad, manifest, build)
    new_parts, not_drawn = discover_new_parts(manifest, parts, counts, args.openscad, build)
    ci = os.environ.get("GITHUB_ACTIONS") == "true"
    for p in new_parts:
        print("%s%s (%s) is not in the manifest yet: drawn automatically with %s from %s. generated/CHECKS.md "
              "has a manifest entry to paste." % ("::notice title=New part::" if ci else "  NEW    ", p.id,
                                                  p.name, p.call.rstrip(";"), p.source))
    parts += new_parts

    wanted = None
    if args.only:
        parts, wanted = select_parts(parts, args.only)

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
            warnings.append("marker %r appears %d time(s) in the assembly but no manifest part uses it%s" % (
                key, counts[key], ", and it could not be drawn automatically: " + not_drawn[key]
                if key in not_drawn else ""))
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
LINEAR = ("angle", "tube", "bar")   # bought by length; the cut length is the part's X extent
FLAT = ("plate", "ring")            # cut from sheet; the blank is X by Y
BOUGHT = ("fastener", "purchased")  # bought ready-made, not cut from stock


def _sorted(results):
    def key(r):
        cat = r["part"].category
        num = [int(x) if x.isdigit() else x for x in re.split(r"(\d+)", r["part"].id)]
        return (CATEGORY_ORDER.index(cat) if cat in CATEGORY_ORDER else 99, num)
    return sorted(results, key=key)


def totals_lines(manifest, rows):
    """INDEX.md totals: per category, then per stock for the parts cut from
    stock.  Everything is added up from the per-part quantities, so it
    follows the model like they do."""
    cats = manifest.get("categories", {})
    lines = ["## Totals", "",
             "| Category | Unique parts | Pieces per machine | Mass per machine |", "|---|---:|---:|---:|"]
    rough = False
    for cat in dict.fromkeys(r["part"].category for r in rows):
        rs = [r for r in rows if r["part"].category == cat]
        est = any(r["qty_src"] != "model" for r in rs)
        rough = rough or est
        lines.append("| %s | %d | %d%s | %.1f kg |" % (
            cats.get(cat, {}).get("label", cat), len(rs), sum(r["qty"] for r in rs), " †" if est else "",
            sum(r["mass"] * r["qty"] for r in rs)))
    lines.append("| **All parts** | **%d** | **%d**%s | **%.0f kg** |" % (
        len(rows), sum(r["qty"] for r in rows), " †" if rough else "", sum(r["mass"] * r["qty"] for r in rows)))
    if rough:
        lines += ["", "† Includes quantities typed into the manifest or estimated from hole counts; the "
                  "tables below mark which."]

    groups = {}
    for r in rows:
        p = r["part"]
        if p.category in BOUGHT:
            continue
        g = groups.setdefault((p.category, p.stock), {"ids": [], "qty": 0, "length": 0.0, "area": 0.0, "mass": 0.0})
        X, Y, _ = r["extents"]
        g["ids"].append(p.id)
        g["qty"] += r["qty"]
        g["length"] += X * r["qty"] if p.category in LINEAR else 0.0
        g["area"] += X * Y * r["qty"] if p.category in FLAT else 0.0
        g["mass"] += r["mass"] * r["qty"]
    lines += ["", "### Stock", "",
              "Material for the parts cut from stock, added up per stock size. Lengths and blank areas are net: "
              "allow for saw kerf, offcuts and plate nesting when ordering.", "",
              "| Stock | Parts | Pieces | Total cut length | Total blank area | Mass |",
              "|---|---|---:|---:|---:|---:|"]
    for (cat, stock), g in groups.items():
        lines.append("| %s | %s | %d | %s | %s | %.1f kg |" % (
            stock, ", ".join(g["ids"]), g["qty"],
            "%.2f m [%.1f ft]" % (g["length"] / 1000, g["length"] / 304.8) if cat in LINEAR else "",
            "%.2f m² [%.1f ft²]" % (g["area"] / 1e6, g["area"] / 92903.04) if cat in FLAT else "",
            g["mass"]))
    return lines


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
        "(`qty_from_holes`). Check both before ordering. Masses are calculated from the model.",
        "",
    ] + totals_lines(manifest, rows)
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
        name = p.name + (" *(new - not in the manifest yet)*" if p.auto else "")
        qty_note = {"model": "", "holes": " *(est. from holes)*"}.get(r["qty_src"], " *(manifest)*")
        lines.append("| **%s** | %s | %s | %s%s | %s | %.2f kg | %s | [PDF](pdf/%s.pdf) | %s |" % (
            p.id, name, p.stock, r["qty"], qty_note,
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
    lines += ["", "## New parts drawn automatically", "",
              "These parts have a `BOM_PART` marker in the model but no entry in parts_manifest.yaml yet. "
              "They are drawn and counted like any other part, but the name comes from the module name, "
              "and the stock and USED IN are placeholders, so each sheet says CHECK BEFORE MAKING. Paste "
              "the entries below under `parts:` in the manifest, fill in the `<...>` fields, and the next "
              "run gives the drawings a proper title block.", ""]
    new = [r for r in _sorted(results) if r["part"].auto]
    lines += ["- **%s** %s: `%s` in `%s`, %d per machine" % (r["part"].id, r["part"].name, r["part"].call,
                                                           r["part"].source, r["qty"]) for r in new] or ["- none"]
    if new:
        lines += ["", "```yaml"]
        for r in new:
            p = r["part"]
            lines += ["  - id: %s" % p.id, "    name: %s" % p.name, "    category: %s" % p.category,
                      "    stock: <material and size>", "    source: %s" % p.source, "    call: '%s'" % p.call,
                      '    count: "%s"' % p.id, "    used_in: <where it goes>"]
        lines.append("```")
    lines += ["", "## Known model issues (from `issues:` in parts_manifest.yaml)", "",
              "These are printed on the affected drawings as CHECK BEFORE MAKING notes. "
              "Fix the model, then delete the issue from the manifest.", ""]
    issues = [(r["part"].id, i) for r in _sorted(results) if not r["part"].auto for i in r["part"].issues]
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
