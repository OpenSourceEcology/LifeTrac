#!/usr/bin/env python3
"""Check the structural assembly sequence and write the numbered step list.

    python3 assembly_sequence.py           # check, then write ASSEMBLY_SEQUENCE.md
    python3 assembly_sequence.py --check   # the same, but exit 1 if there are errors (CI)

The sequence lives in assembly_sequence.yaml. Part ids and names come from
DESIGN-STRUCTURAL/drawings/parts_manifest.yaml. Quantities per machine come
from the generated BOM (drawings/generated/bom.csv), which counts them from
the OpenSCAD model, so the checks follow the design.

A quantity is *exact* when the BOM says it was counted in the model
(qty_source = model). Hand-entered and hole-count estimates (most bolts, nuts
and washers today) are not exact.

Errors (fail CI):  unknown part id, a part with an exact quantity used more
                   times than the machine has, duplicate step id, broken
                   `after` / `uses` links, a sub-assembly installed twice (its
                   parts are counted when it is built, so the quantity check
                   cannot see this), a jig listed as a machine part.
Warnings:          parts with an exact quantity not placed by any step
                   (errors once `complete: true`), sub-assemblies built but
                   never installed, over-use of an estimated quantity.
"""

import argparse
import csv
import os
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
TOOLS_ONLY = {"printed"}     # 3D-printed jigs are tools, never installed
PURCHASED = {"fastener"}     # bought, not fabricated: checked, but left out of the fabricated tally


def load_quantities(base, seq, parts):
    """Per-machine quantity, whether it is exact (counted in the model), and
    drawing path for every part id."""
    qty, exact, drawing = {}, set(), {}
    if seq.get("bom"):
        bom = base / seq["bom"]
        if not bom.exists():
            # Without it no quantity counts as exact and over-use goes unchecked.
            raise SystemExit("BOM %s not found - run DESIGN-STRUCTURAL/drawings/generate_part_drawings.py "
                             "first, or remove `bom:` from the sequence file" % bom)
        with open(bom, newline="") as fh:
            for row in csv.DictReader(fh):
                qty[row["id"]] = int(row["qty_per_machine"] or 0)
                if row.get("qty_source") == "model":
                    exact.add(row["id"])
                drawing[row["id"]] = bom.parent / row["drawing"]
    for pid, p in parts.items():
        if pid not in qty and p.get("qty") is not None:
            qty[pid] = int(p["qty"])
    return qty, exact, drawing


def check(seq, parts, qty, exact):
    """Walk the steps in order. Return (steps, placed, errors, warnings)."""
    errors, warnings = [], []
    steps, index, placed = [], {}, {}
    subassemblies, used_subs = {}, {}
    for phase in seq.get("phases", []):
        for step in phase.get("steps", []):
            n = len(steps) + 1
            sid = step.get("id")
            where = "step %d (%s)" % (n, sid or "no id")
            if not sid:
                errors.append("%s has no id" % where)
            elif sid in index:
                errors.append("%s: id is also used by step %d" % (where, index[sid]))
            else:
                index[sid] = n
            if step.get("subassembly"):
                subassemblies[sid] = n
            for ref in step.get("after", []) or []:
                if ref not in index:
                    errors.append("%s: `after: %s` must name an earlier step" % (where, ref))
            for ref in step.get("uses", []) or []:
                if ref not in subassemblies:
                    errors.append("%s: `uses: %s` must name an earlier sub-assembly step" % (where, ref))
                elif ref in used_subs:
                    errors.append("%s: sub-assembly %s is already installed in step %d"
                                  % (where, ref, used_subs[ref]))
                else:
                    used_subs[ref] = n
            rows = []
            for item in step.get("add", []) or []:
                pid, q = str(item.get("part")), item.get("qty", 1)
                if pid not in parts:
                    errors.append("%s: unknown part id %s" % (where, pid))
                    continue
                cat = parts[pid]["category"]
                if cat in TOOLS_ONLY:
                    errors.append("%s: %s is a jig - list it under tools, not add" % (where, pid))
                    continue
                if not isinstance(q, int) or q < 1:
                    errors.append("%s: qty for %s must be a whole number >= 1" % (where, pid))
                    continue
                placed[pid] = placed.get(pid, 0) + q
                total = qty.get(pid)
                if total is not None and placed[pid] > total:
                    msg = "%s: %s placed %d times so far but the machine has %d" % (where, pid, placed[pid], total)
                    (errors if pid in exact else warnings).append(msg)
                rows.append((pid, q, placed[pid]))
            steps.append({"n": n, "phase": phase, "step": step, "rows": rows})

    for sid, n in subassemblies.items():
        if sid not in used_subs:
            warnings.append("sub-assembly %s (step %d) is built but never installed (no later `uses`)" % (sid, n))

    for pid, p in parts.items():
        if p["category"] in TOOLS_ONLY or pid not in exact:
            continue
        total, done = qty.get(pid), placed.get(pid, 0)
        if total is not None and done < total:
            msg = "%s (%s): %d of %d placed" % (pid, p["name"], done, total)
            (errors if seq.get("complete") else warnings).append("not placed by any step: " + msg)
    return steps, placed, errors, warnings


def write_markdown(path, seq, parts, qty, exact, drawing, steps, placed, errors, warnings, source_name):
    def link(pid):
        d = drawing.get(pid)
        if d is None:
            return "**%s**" % pid
        return "[**%s**](%s)" % (pid, os.path.relpath(d, path.parent).replace(os.sep, "/"))

    status = {}
    for s in steps:
        st = s["step"].get("status", "draft")
        status[st] = status.get(st, 0) + 1
    machine = [pid for pid, p in parts.items() if p["category"] not in TOOLS_ONLY and pid in exact]
    fabricated = [pid for pid in machine if parts[pid]["category"] not in PURCHASED]
    n_total = sum(qty.get(pid, 0) for pid in fabricated)
    n_placed = sum(min(placed.get(pid, 0), qty.get(pid, 0)) for pid in fabricated)

    out = [
        "# LifeTrac v25 - Structural Assembly Sequence",
        "",
        "> **Generated from [`%s`](%s). Edit that file, not this one.** Step numbers come from "
        "the order of the steps, so inserting or moving a step renumbers everything after it. Refer "
        "to steps by their `id`, which never changes. See [`ASSEMBLY_MANUAL_AUTOGEN.md`](ASSEMBLY_MANUAL_AUTOGEN.md) "
        "for how this list will drive a picture manual." % (source_name, source_name),
        "",
        "**%s** - %d steps (%s) - %d of %d fabricated pieces placed." % (
            "COMPLETE" if seq.get("complete") else "DRAFT", len(steps),
            ", ".join("%d %s" % (v, k) for k, v in sorted(status.items())), n_placed, n_total),
        "",
    ]
    current = None
    for s in steps:
        step, phase = s["step"], s["phase"]
        if phase is not current:
            current = phase
            out += ["## %s" % phase.get("title", phase.get("id", "")), ""]
        badge = []
        if step.get("subassembly"):
            badge.append("sub-assembly: *%s*%s" % (step["subassembly"],
                                                    " x%d" % step["make"] if step.get("make", 1) > 1 else ""))
        badge.append(step.get("status", "draft"))
        out += ["### Step %d - %s" % (s["n"], step.get("title", "")), "",
                "`%s` - %s" % (step.get("id", "?"), " - ".join(badge)), ""]
        if s["rows"]:
            out += ["| Add | Part | Qty | Placed so far | Per machine |", "|---|---|---:|---:|---:|"]
            for pid, q, running in s["rows"]:
                out.append("| %s | %s | %d | %d | %s |" % (link(pid), parts[pid]["name"], q, running,
                                                           qty.get(pid, "?")))
            out.append("")
        extra = []
        if step.get("uses"):
            extra.append("**Installs:** " + ", ".join("`%s`" % u for u in step["uses"]))
        if step.get("after"):
            extra.append("**After:** " + ", ".join("`%s`" % a for a in step["after"]))
        if step.get("tools"):
            extra.append("**Tools:** " + ", ".join(link(t) if t in parts else str(t) for t in step["tools"]))
        if extra:
            out += ["  \n".join(extra), ""]
        if step.get("notes"):
            out += ["> " + " ".join(str(step["notes"]).split()), ""]

    remaining = [(pid, parts[pid]["name"], qty.get(pid, 0), placed.get(pid, 0)) for pid in machine
                 if placed.get(pid, 0) < qty.get(pid, 0)]
    out += ["## Parts not yet placed", ""]
    if remaining:
        out += ["| Part | Name | Per machine | Placed | Remaining |", "|---|---|---:|---:|---:|"]
        out += ["| %s | %s | %d | %d | %d |" % (link(pid), name, t, d, t - d) for pid, name, t, d in remaining]
    else:
        out.append("Every fabricated part is placed exactly as many times as the model uses it.")
    estimated = [pid for pid, p in parts.items()
                 if p["category"] not in TOOLS_ONLY and pid not in exact and not placed.get(pid)]
    if estimated:
        out += ["", "Not yet assigned to steps (quantities are estimates, not counted in the model): "
                + ", ".join(estimated) + "."]
    out += ["", "## Checks", ""]
    out += ["- ERROR: %s" % e for e in errors] + ["- warning: %s" % w for w in warnings] or ["- all good"]
    path.write_text("\n".join(out) + "\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sequence", default=str(HERE / "assembly_sequence.yaml"))
    ap.add_argument("--out", default=str(HERE / "ASSEMBLY_SEQUENCE.md"))
    ap.add_argument("--check", action="store_true", help="exit 1 if there are errors")
    args = ap.parse_args()

    seq_path = Path(args.sequence).resolve()
    seq = yaml.safe_load(seq_path.read_text())
    base = seq_path.parent
    manifest = yaml.safe_load((base / seq["manifest"]).read_text())
    parts = {str(p["id"]): p for p in manifest["parts"]}
    qty, exact, drawing = load_quantities(base, seq, parts)

    steps, placed, errors, warnings = check(seq, parts, qty, exact)
    write_markdown(Path(args.out).resolve(), seq, parts, qty, exact, drawing, steps, placed, errors, warnings,
                   seq_path.name)

    ci = os.environ.get("GITHUB_ACTIONS") == "true"
    for w in warnings:
        print(("::warning title=Assembly sequence::" if ci else "warning: ") + w)
    for e in errors:
        print(("::error title=Assembly sequence::" if ci else "ERROR: ") + e)
    print("%d steps, %d errors, %d warnings -> %s" % (len(steps), len(errors), len(warnings), args.out))
    return 1 if (args.check and errors) else 0


if __name__ == "__main__":
    sys.exit(main())
