# LifeTrac v25 — Part Drawings

One printable engineering drawing (PDF) for every unique part of the v25 tractor:
CNC-cut plate, angle iron and tube, pins, rings and lugs, 3D-printed jigs, and
reference sheets for the bolts, nuts and washers. The drawings are **generated
from the OpenSCAD model**, so they change when the design changes. Nobody edits them by
hand.

**Browse the drawings:** [`generated/INDEX.md`](generated/INDEX.md) (bill of
materials with a PDF link per part) · **Warnings and quantity checks:**
[`generated/CHECKS.md`](generated/CHECKS.md) · **Plate cutting files:**
[`generated/dxf/`](generated/dxf/)

## What a sheet looks like

Each sheet is a Letter-landscape, third-angle drawing:

* **Front, top and right side views** at one standard scale (1:1, 1:2, 1:2.5,
  1:4, 1:5, 1:8, 1:10 …). Visible lines are solid and hidden lines dashed.
* **An isometric view**, shaded, for orientation.
* **Overall dimensions** in `mm [inch]`, with a `THK` callout on plates.
* **Running dimensions** from the left end on angle iron and tube, for marking holes
  with a tape measure.
* **A hole table** where each tag (A1, B3…) is keyed to the drawing, with X/Y from
  the lower-left corner of the view, and Ø and `THRU` / `THRU 2X WALLS`.
  Plate holes smaller than the plate thickness are flagged `DRILL*`.
* **A title block** with:
  * part number, title and material/stock
  * **quantity per machine** (counted from the model)
  * process, finish and cut length / blank size, plus the calculated mass
  * scale, the third-angle symbol, **revision letter and date**
  * the geometry ID and the source `.scad` module
* **Notes** for the process: tolerances, "cut from DXF", deburr, marking.

See [`DRAWING_STANDARDS.md`](DRAWING_STANDARDS.md) for the research behind these
conventions (ASME Y14.x, AISC detailing, plasma-cutting practice).

## How it stays up to date

The GitHub Action
[`.github/workflows/generate-part-drawings.yml`](../../../.github/workflows/generate-part-drawings.yml)
runs whenever something under `openscad/` or `drawings/` changes, or
`BUILD-STRUCTURE/assembly_sequence.yaml` changes:

1. It runs the unit tests (`tests/`).
2. It renders every part in [`parts_manifest.yaml`](parts_manifest.yaml) and redraws
   the sheets.
3. It checks the assembly sequence against the parts list
   ([`../../BUILD-STRUCTURE/assembly_sequence.py`](../../BUILD-STRUCTURE/assembly_sequence.py)).
4. It uploads everything as the **`part-drawings` artifact**. That includes the
   one-file book `LifeTrac_v25_Part_Drawings.pdf`.
5. **On `main` only**, it commits the updated `generated/` folder back with
   `[skip ci]`.

The output is byte-for-byte reproducible, so the commit contains only the
drawings that really changed. A drawing gets the next **revision letter**
(A, B, C … skipping I, O, Q, S, X, Z) whenever anything printed on it changes:

* the geometry, views, dimensions, hole table, scale or paper size;
* the title block (material, quantity, mass, notes …).

The date recorded is the date of the commit that changed it. A change to the
generator's styling alone does not bump revisions. The letter and date are
kept in [`generated/revisions.json`](generated/revisions.json).

CI runs the generator with `--strict`. The job fails, after still uploading the
artifact, when any of these happens:

* the model has a `BOM_PART` marker no manifest part uses (a new part without a
  drawing);
* a manifest part's marker is never reached;
* a quantity disagrees with the model;
* a DXF export fails;
* a part fails to render.

Pull requests only get the artifact. Please **don't commit files in
`generated/` by hand**; CI rewrites them after merge.

## Quantities come from the model

"QTY PER MACHINE" is counted, not typed. Every part module announces itself when it
is placed in the assembly:

```openscad
module part_a4_frame_tube_mount(show_holes=true) {
    echo(BOM_PART = "A4");  // counted by drawings/generate_part_drawings.py
    _part_a4_frame_tube_mount(show_holes);
}
```

The generator evaluates `openscad/lifetrac_v25.scad` (about half a second, with no
geometry rendered) and counts the `BOM_PART` echoes. When the design adds a
fifth motor-plate angle, every A4 drawing and the BOM show the new number.

`generated/CHECKS.md` lists:

* markers in the model that no manifest part uses: a new part nobody has drawn yet;
* manifest parts whose marker is never reached: a part the assembly no longer uses;
* differences between a model count and a hand-entered `qty`.

Parts without a marker use one of two other sources:

* `qty` typed into the manifest (jigs, 1/4-20 lug bolts). The sheet says
  `(FROM MANIFEST)`.
* `qty_from_holes`: bolts, nuts and washers are not in the model yet, so their
  counts are estimated from the model's own holes. For example, one 3/8-16 bolt
  per Ø9.5 hole in the listed angle parts, times each part's counted quantity.
  The sheet says `(EST. FROM HOLE COUNT)`. The estimate is an upper bound, because
  a bolt through a tube between two angles is counted twice.

Modelled items that deliberately have no drawing, such as the hydraulic
cylinders' own pin nuts, are listed under `ignore_markers` with a reason.

### Known model problems

The manifest can attach `issues:` to a part. Each one is printed on the sheet as a
`CHECK BEFORE MAKING` note and collected in `generated/CHECKS.md`. The drawings
always show the model as it is. When the drawings reveal a model bug, record it
there, fix the model, then delete the issue.

## Adding or changing a part

1. Make sure one OpenSCAD statement draws exactly **one** copy of the part.
   * Standalone part files (`openscad/parts/*.scad`) and the structural wrappers
     already do this.
   * For modules in `lifetrac_v25.scad` that draw a part together with its
     neighbours, use `clip_box` in the manifest to keep just the part.
2. Add `echo(BOM_PART = "<ID>");` inside the module that the assembly calls once
   per part.
3. Add an entry to [`parts_manifest.yaml`](parts_manifest.yaml):

   ```yaml
   - id: P07                       # part number (unique; used in assembly steps)
     name: Wheel mount plate
     category: plate               # plate | angle | tube | bar | lug | ring | printed | fastener
     stock: PL 1/2 [12.7] ASTM A36
     source: openscad/parts/wheel_mount.scad    # relative to DESIGN-STRUCTURAL/
     call: wheel_mount();          # the file is `use`d; params are `include`d
     count: P07                    # BOM_PART marker(s) -> quantity
     used_in: Wheel assemblies
     # optional:
     # qty: 4                      # hand quantity (cross-checked if count is set)
     # clip_box: [[x0,y0,z0],[x1,y1,z1]]   # keep only this region of the call's output
     # rotate: [[z, 90]]           # turn the part after auto-orientation
     # notes: ["WELD NUT ON FAR SIDE"]
     # issues: ["HOLE X IS MODELLED BLIND - CUT THROUGH"]   # known model bug, printed as CHECK BEFORE MAKING
     # qty_from_holes: {diameters: [9.525], parts: [A1, A2], per_hole: 1}  # hardware estimate
     # options: {views: [front, right, iso], hole_table: false, running_dims: true}
   ```
4. Check it locally, then push. CI does the rest.

   ```bash
   cd LifeTrac-v25/DESIGN-STRUCTURAL/drawings
   python3 generate_part_drawings.py --only P07
   ```

## Running it locally

```bash
sudo apt-get install openscad          # 2021.01, the same as CI
pip install -r requirements.txt
python3 -m unittest discover -s tests  # fast, no OpenSCAD needed
python3 generate_part_drawings.py      # all parts, ~1 min
```

Scratch files and the combined book go to `build/`, which is not committed.
Use `--paper a4` (or `tabloid`, `a3`) for other sheet sizes.

## Files

| Path | What |
|---|---|
| `parts_manifest.yaml` | The list of parts to draw: part numbers, names, stock, notes |
| `generate_part_drawings.py` | The generator (CLI) |
| `partdrawings/mesh.py` | STL loading, vertex welding, auto-orientation, volume |
| `partdrawings/hlr.py` | Hidden-line removal, hole detection |
| `partdrawings/sheet.py`, `drawing.py` | Sheet layout: views, dimensions, hole table, title block |
| `hardware.scad` | Reference models of bolts, nuts, washers and pins (ASME B18.2.1/.2) |
| `tests/` | Unit tests |
| `generated/` | **Generated.** PDFs, DXFs, index, checks, revision history |

## Limits (for now)

* Holes are detected when they are round and seen along their axis. Slots, arcs
  and chamfers are drawn but not dimensioned. The DXF is the authority for plate
  profiles.
* Fasteners are not in the model yet, so hardware counts are estimated from holes
  and lengths are "per joint". Placing a `BOM_PART` marker at each bolted joint
  would make both exact (see
  [`../../BUILD-STRUCTURE/ASSEMBLY_MANUAL_AUTOGEN.md`](../../BUILD-STRUCTURE/ASSEMBLY_MANUAL_AUTOGEN.md)).
* Hole-to-hole matching between mating parts is not checked yet. `CHECKS.md`
  lists the model problems found so far.
* Very long, thin parts are drawn at a small scale on Letter paper. Use
  `--paper tabloid` for shop prints of those.
