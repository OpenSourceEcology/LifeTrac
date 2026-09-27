# Part Drawing Conventions (research notes)

This page records how professional shops draw each kind of part on the tractor,
and which of those conventions the automatic drawing generator
([`generate_part_drawings.py`](generate_part_drawings.py)) follows. It was written from
published sources (listed at the end). Some values could only be checked against
secondary sources; they are marked *(verify)*. Check those before relying on them.

## 1. Standards in play

| Topic | US practice | International practice | What LifeTrac drawings use |
|---|---|---|---|
| Views / projection | ASME Y14.3, **third-angle** | ISO 128 / ISO 5456-2, first-angle | **Third-angle**, with the projection symbol in the title block |
| Dimensioning | ASME Y14.5 | ISO 129 / ISO 14405 | Millimetres first, inches in brackets: `146.1 [5.750]` |
| Sheet size, title block | ASME Y14.1: A = 8.5×11, B = 11×17 | ISO 5457 (A4, A3), ISO 7200 title block | ANSI A (Letter) landscape by default, with `--paper a4/tabloid/a3` |
| Drawing type | ASME Y14.24 "monodetail" (one part per drawing) | — | One PDF per part |
| Parts list | ASME Y14.34 | — | `generated/INDEX.md` and `bom.csv` |
| Revisions | ASME Y14.35: letters, skipping I, O, Q, S, X, Z | — | Automatic: the letter goes up when a part's sheet content changes |
| General tolerances | Title-block tolerance note | ISO 2768-1 (`-m` medium, `-c` coarse) | A tolerance note per process (see below) |

**Line weights.** Visible lines are thick and hidden lines thin and dashed, at
roughly a 2:1 ratio (ASME Y14.2 uses about 0.6 / 0.3 mm). The generator uses
1.0 / 0.5 pt, which suits parts drawn at reduced scale on Letter paper.

**Inch formats (ASME Y14.5).** Inches have no leading zero (`.500`); millimetres
do (`0.5`).

## 2. View layout

With third-angle projection, the **front** view is the main view. The **top**
view sits directly above it and the **right side** view directly to its right,
all lined up and all at one scale. The isometric goes in a spare corner, top
right, marked as reference.

Use only the views that fully define the part:

| Part type | Typical views | Generator default |
|---|---|---|
| Flat plate | One profile view plus a thickness callout (`PL 1/2 [12.7]`) | Profile plus thin edge views, no hidden lines, `THK` dimension |
| Angle, tube | Elevation of the drilled face plus an end view showing which leg the holes are in | Front, top and right side with hidden lines, plus iso |
| Pin, round bar | One side view with Ø, length and chamfers | Front and right, plus iso |
| Purchased hardware | Usually not drawn; called out by standard designation | Reference sheet labelled *PURCHASED – DO NOT FABRICATE* |

The generator turns every part so its longest dimension runs left to right
and its thinnest points at the viewer. The front view then shows a plate's
profile or a tube's broad face.

## 3. CNC plasma / laser / waterjet plate

* **The DXF is what the machine cuts; the drawing is for inspection.** Each plate
  sheet says `CUT THE PROFILE FROM generated/dxf/<part>.dxf AT 1:1`. The generator
  exports that DXF from the same model, turned the same way as the drawing.
* **Dimensioning:** overall extents, and every hole in a **hole table**
  (`TAG | X | Y | SIZE`) measured from the lower-left corner of the view (X0, Y0).
  Tags are one letter per hole size (A = smallest) plus a number.
* **Material callout:** `PL 1/2 [12.7] ASTM A36`.
* **Small holes:** plasma holes smaller than about 1× the plate thickness come out
  tapered and undersized. High-definition "True Hole" plasma can do about 1:1 for
  holes up to 1 in; below that, shops plasma-mark the centre and drill. The
  generator tags such holes `DRILL*`, and `generated/CHECKS.md` lists them.
* **Typical cut accuracy:**
  * Conventional plasma: about ±0.03 in (0.8 mm) with a 3–5° edge bevel.
  * High-definition plasma: ±0.010–0.020 in.
  * Laser and waterjet: about ±0.005 in.

  Plate sheets use `±0.8 [±.03]` for the profile and hole positions.
* **Standard notes:**
  * Remove dross, break sharp edges.
  * Edges square unless noted.
  * Mark the part number on each piece.
* **Edge quality.** ISO 9013 grades thermal-cut edges by squareness and
  roughness; name a class there if it matters *(the exact designation string is
  not verified)*.

## 4. Angle, tube and channel (cut and drill)

These follow structural steel detailing (AISC) "single-part drawings" with piece
marks:

* **Every distinct piece gets a mark.** Here that is the part number (A4, T1…),
  painted on the steel.
* **Shape designations:**
  * `L2x2x1/4` for angle.
  * `HSS6x2x1/4` for rectangular tube. "TS" is the obsolete name, replaced by HSS
    in 1998.
  * `C6x8.2` for channel.
  * Usual grades: ASTM A36 for angle, channel and plate; ASTM A500 Gr B/C for HSS.
* **Hole positions are running (ordinate) dimensions from one reference end,**
  which is how a fabricator lays them out with a tape. The generator draws them
  under the front view (and above the top view) from `0` at the left end, and
  also lists every hole in the hole table.
* **End cuts:** `SQUARE CUT BOTH ENDS` unless a miter or cope is shown.
* **Tolerances:** cut length ±1/16 in (±1.5 mm); hole location ±1/32 in (±0.8 mm).
* **Angle gages** (hole distance from the heel) come from the AISC workable-gage
  table, e.g. 1-1/8 in for a 2 in leg *(verify against AISC Manual Table 1-7A)*.
  Standard bolt holes are the bolt Ø + 1/16 in *(verify, AISC J3.3)*.
* **Holes through tube.** The hole table says `THRU 2X WALLS` when a hole goes
  through both walls on one axis, so it can be drilled in one pass on a drill
  press.

## 5. Purchased hardware (bolts, nuts, washers, pins)

Shops normally don't draw standard hardware. They call it out by designation in the
parts list, or use a vendor/source-control drawing when one supplier is required.
Common callout formats:

* `HEX BOLT 1/2-13 UNC-2A x 2-1/2, SAE J429 GR 8, YELLOW ZINC, ASME B18.2.1`
* `HEX NUT 1/2-13 UNC-2B, GR 8, ASME B18.2.2`
* Metric: `ISO 4017 - M12 x 80 - 8.8` (screw), `ISO 4032 - M12 - 8` (nut)
* Hydraulics: dash sizes in sixteenths of an inch (`-8` = 1/2 in),
  e.g. `-8 ORB (SAE J1926) x -8 JIC 37° (SAE J514)`.

The LifeTrac hardware sheets are reference sheets, so builders can recognise and
order the right part. They use nominal ASME B18.2.1 / B18.2.2 / SAE washer
dimensions from [`hardware.scad`](hardware.scad), are marked
*PURCHASED – DO NOT FABRICATE*, and show threads the simplified ASME Y14.6 way.

## 6. 3D-printed jigs

ASME Y14.46 covers additive manufacturing. For simple jigs the useful notes are:

* The process and material: FFF, PLA or PETG.
* Layer height, walls, infill and supports.
* The build orientation.
* That the STL is the manufacturing source.

The jig sheets use the print settings from
[`../openscad/3d_printed_welding_jigs/README.md`](../openscad/3d_printed_welding_jigs/README.md).

## 7. Title block

The fields follow ISO 7200 and common US practice:

* Project and owner.
* **Part number.**
* **Title.**
* **Material/stock.**
* **Quantity per machine**, counted from the model when possible.
* Process, finish, size (blank size or cut length) and calculated mass.
* Scale, units, third-angle symbol, sheet `n OF m`.
* **Revision letter and date.**
* Geometry ID: a hash of the part's shape, so a printed sheet can be matched to the model version.
* Source `.scad` file and module.
* "Used in".
* The notes block, including `DO NOT SCALE DRAWING`.

## 8. Tooling notes (why the generator works the way it does)

* **OpenSCAD 2021.01** exports STL, DXF, SVG, PDF and echo text. It has no STEP
  export and no hidden-line drawing, and `projection()` gives only the outer
  silhouette. So the generator renders each part to STL and does its own
  hidden-line removal in numpy: sharp edges plus silhouettes, each sampled and
  depth-tested.
* **Holes** are found by fitting circles to closed sharp-edge loops. OpenSCAD puts
  polygon vertices exactly on the circle, so the fitted Ø is exact. A hole is told
  from a boss by which side of the rim the flat face lies on.
* **Other ways to do this:**
  * FreeCAD TechDraw can export view SVGs headless, but headless PDF export is
    still an open issue.
  * CadQuery and build123d do true B-rep hidden-line removal and have drawing
    templates, but they need solid models, which OpenSCAD cannot export.
  * If the model ever moves to a B-rep CAD tool, those become the better base.
* **reportlab** draws the sheet. Output is made byte-identical run to run: fixed
  PDF metadata, triangles in a canonical order, and pinned library versions. As a
  result, CI commits only the drawings whose part actually changed.

## Sources

The standards bodies' own sites could not be fetched during research. These
sources were used:

* ASME Y14.1 overview — https://en.wikipedia.org/wiki/ANSI/ASME_Y14.1
* First- vs third-angle projection — https://www.gdandtbasics.com/first-vs-third-angle-orthographic-views/
* ISO 7200 — https://www.iso.org/standard/35446.html
* ISO 2768 tables — https://www.rivcut.com/resources/iso-2768-tolerance-chart
* Y14.35 revision letters — https://www.fcsuper.com/swblog/?p=102
* ASME Y14.24 drawing types — https://ndia.dtic.mil/wp-content/uploads/2008/technical/GastonEngineeringDrawingsY14_24a.pdf
* Plasma hole quality — https://www.hypertherm.com/resources/system-support/maintenance-and-use/cut-quality/hole-quality/
* True Hole — https://www.hypertherm.com/solutions/about-our-products/surecut-technology/true-hole/
* Laser minimum geometry — https://sendcutsend.com/faq/what-are-best-practices-for-minimum-geometry/
* ISO 9013 — https://www.laserspechub.com/guides/iso-9013-practical-guide
* AISC workable gages discussion — https://www.eng-tips.com/threads/aisc-325-17aw-table-1-7a-workable-gages-in-angle-legs-an-aerospace-engineer-asks.499329/
* TS vs HSS — https://www.atlastube.com/atlas-observer/whats-the-difference-between-a-ts-and-an-hss/
* ASTM A500 — https://steeltubeinstitute.org/resources/astm-a500/
* ASME B18.2.1 — https://www.portlandbolt.com/technical/faqs/ansi-b18-2-1/
* ISO 4017 — https://andrewsfasteners.uk/standards/iso-4017-hexagon-head-screws-basic-dimensions/
* Purchased parts on drawings — https://www.eng-tips.com/threads/asme-14-24-engineering-drawing-purchased-parts.502881/
* ASME Y14.46 (additive) — https://www.asme.org/codes-standards/find-codes-standards/y14-46-product-definition-additive-manufacturing-(1)
* FreeCAD headless PDF issue — https://github.com/FreeCAD/FreeCAD/issues/5710
* build123d technical drawings — https://github.com/gumyr/build123d/blob/dev/docs/tech_drawing_tutorial.rst
* CadQuery SVG export (HLR) — https://github.com/CadQuery/cadquery/blob/master/cadquery/occ_impl/exporters/svg.py
* OpenSCAD dimensioned-drawing request — https://github.com/openscad/openscad/issues/3433
