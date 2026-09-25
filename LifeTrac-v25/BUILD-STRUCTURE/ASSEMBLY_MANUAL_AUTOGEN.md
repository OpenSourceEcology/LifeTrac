# Auto-generating a LEGO / IKEA-style assembly manual for LifeTrac v25

**Status:** design proposal. Phase 0 (the editable step list) is implemented;
the rest is a plan.

**Related:**
* [`assembly_sequence.yaml`](assembly_sequence.yaml): the editable step list.
* [`ASSEMBLY_SEQUENCE.md`](ASSEMBLY_SEQUENCE.md): the generated numbered list.
* [`../DESIGN-STRUCTURAL/drawings/`](../DESIGN-STRUCTURAL/drawings/): part
  drawings, part numbers and model-counted quantities.

## 1. What we want

A picture-first build book that anyone with a welder and a drill press can
follow. It should need almost no words and never go out of date with the model.
Every step page shows:

```
+--------------------------------------------------------------+
|  12                                   [x2]   [2-person lift] |
|  +-------------------+                                       |
|  | [T1] x1  [A4] x8  |       machine so far: light grey      |
|  | [F3] x16 (1:1)    |       new parts: bold, coloured       |
|  +-------------------+       arrow + dashed path showing     |
|                              where the new parts go          |
|  [tools: 3/4" wrench, J1 jig]                  (rotate icon) |
+--------------------------------------------------------------+
```

* **A big step number.** It is derived from the order of the steps; nobody
  types it.
* **A parts callout box** (LEGO calls it the PLI). It shows a thumbnail of each
  part added in the step, with its part number and count. Bolts, nuts and washers
  are drawn **at 1:1 scale** so they can be matched by laying them on the page,
  as IKEA does.
* **The assembly so far** in light grey. The **new parts** are bold and coloured,
  with an arrow and a dashed guide line from where they start to where they go.
* **Sub-assemblies** built off the machine, drawn in a framed callout. A `x2`
  badge shows when two are needed (left and right arms, four wheel hubs).
* **Icons:** tools and jigs; a two-person/hoist icon when a part is heavy (the
  outer side panel is 82 kg, calculated on its drawing); a "turn the model" icon
  when the viewpoint swings round.

## 2. Why LEGO and IKEA instructions work, and what we borrow

* **Research basis.** Agrawala et al., *Designing Effective Step-By-Step
  Assembly Instructions* (SIGGRAPH 2003), drew their design principles from
  cognitive-psychology studies of how people understand and communicate assembly.
  They then built a system that generates instructions automatically from a model
  and an assembly sequence, drawing on earlier robotics work on assembly
  planning. The principles we adopt:
  * Add parts in small, meaningful groups, and add symmetric or identical parts in
    the same step.
  * Keep every new part and its attachment point visible.
  * Keep the viewpoint stable and only change it when the work demands it.
  * Build hierarchically, using sub-assemblies.
  * Show both what the result looks like (a structural diagram) and how to get
    there (an action diagram, with arrows and guide lines).
* **LEGO / LDraw precedent.** The open-source
  [LPub3D](https://trevorsandy.github.io/lpub3d/) generates LEGO-style
  instructions from an LDraw model plus step markers. It produces per-step parts
  lists, automatic sub-model callouts and a bill of materials. That is the same
  shape as this proposal: *model + ordered step list → rendered pages*.
* **IKEA.** Its manuals are wordless and draw hardware at 1:1. They use tool
  icons, "two people" icons and do/don't panels. For a steel tractor, the 1:1
  hardware and two-person/hoist icons matter most.

## 3. What already exists

| Piece | Where | Used by the manual for |
|---|---|---|
| Part numbers, names, stock | `DESIGN-STRUCTURAL/drawings/parts_manifest.yaml` | Callout labels, links to drawings |
| Quantities counted from the model | `echo(BOM_PART = …)` markers → `generated/bom.csv` | Checking every part is placed exactly once |
| Per-part isometric renders | The part-drawing generator (`partdrawings/`) | Callout-box thumbnails |
| Hidden-line line-art engine | `partdrawings/hlr.py` | IKEA-style vector step drawings |
| Part mass | Title blocks / `bom.csv` | Two-person/hoist icons |
| **Editable ordered step list** | `assembly_sequence.yaml` | Everything below |
| Step list checker + numbered list | `assembly_sequence.py` → `ASSEMBLY_SEQUENCE.md` | Kept green by CI |

### The numbered list: which part goes on next

[`assembly_sequence.yaml`](assembly_sequence.yaml) is the single source of truth:

```yaml
phases:
  - id: frame
    title: Base frame
    steps:
      - id: cross-tubes                       # stable id - never renumbered or reused
        title: Slide the front and rear cross tubes through the panel windows and bolt on the tube angles
        after: [inner-panels, motor-plates-install]
        add:
          - {part: T1, qty: 1}
          - {part: T2, qty: 1}
          - {part: A4, qty: 16}
        tools: [J1]
        status: draft                          # draft | checked | verified
```

* **Numbering is derived, never stored.** Step numbers come from the order of
  the list. Inserting, moving or deleting a step renumbers everything after it.
* **Every step has a permanent `id`.** Issues, photos, comments and the future
  manual refer to that. When a builder finds that step 14 must come before step
  12, someone moves one block of YAML in a pull request. The ids stay the same,
  so nothing that points at them breaks.
* **The checker runs in CI.** It uses the same workflow as the part drawings.
  * It **fails** on an unknown part number, on a part used more times than the
    model has it, on a duplicate id, or on an `after`/`uses` that points forward.
  * It **warns** about parts no step places. That becomes an error once the file
    is marked `complete: true`.
  * It regenerates [`ASSEMBLY_SEQUENCE.md`](ASSEMBLY_SEQUENCE.md). That file is
    the human-readable numbered list, with a running "placed so far / per
    machine" count and a link to each part's drawing.
* **The model and the list stay in step.** When the design adds a part, its new
  `BOM_PART` marker appears in the count. The checker then reports "not placed by
  any step" until someone adds it to the sequence. When a quantity changes, the
  running totals catch it.
* **`status` tracks trust:**
  * `draft`: written from the model.
  * `checked`: desk-checked against the model.
  * `verified`: someone built it this way. Record who and when in `notes`.

## 4. The missing link: where each part goes

A manual needs to show *this particular* A4 angle being bolted to *this*
tube. Today the model only says "there are 26 A4s". The proposal is to give
every placed copy a **name**, using one small wrapper module:

```openscad
// openscad/bom.scad  (proposed)
STEP_SHOW = [];   // instance names to draw; [] = draw everything (normal renders)
STEP_NEW  = [];   // instance names to highlight as "added in this step"

module bom_part(id, instance) {
    echo(BOM_PART = id, INSTANCE = instance);   // replaces today's bare markers
    shown = STEP_SHOW == [] || len(search([instance], STEP_SHOW)[0]) > 0;
    if (shown)
        color(len(search([instance], STEP_NEW)[0]) > 0 ? "DarkOrange" : "LightGray")
        children();
}
```

Each call site in `lifetrac_v25.scad` changes mechanically:

```openscad
// before
translate([left_inner_panel_face, front_tube_rear_y, angle_z]) frame_tube_angle_iron();
// after
bom_part("A4", "frame/tube-front/left-inner/rear")
    translate([left_inner_panel_face, front_tube_rear_y, angle_z]) frame_tube_angle_iron();
```

* **Instance names** follow `subsystem/sub-assembly/side/position` and are stable,
  like step ids.
* **The instance list from the model** is one echo run (about half a second, the
  same as today's count). It is the full list of things that must be installed.
* **Steps can name instances with globs,** for example
  `add: [{part: A4, instances: ["frame/tube-*/*"]}]`. The checker then proves that
  every instance is installed **exactly once** and that each step names real
  instances. Counts per step are no longer typed by hand.
* **The step state reaches OpenSCAD on the command line:**
  `openscad -D 'STEP_SHOW=[...]' -D 'STEP_NEW=[...]'`. Normal renders are
  unaffected because both lists default to empty.

## 5. Rendering the pages

Two renderers are planned. They are built in order, and both are driven by the
same step list.

1. **Quick shaded version (OpenSCAD PNG).** For each step, render the machine
   with `STEP_SHOW` = everything installed so far and `STEP_NEW` = this step's
   instances. OpenSCAD's PNG export runs headless in CI under `xvfb`, the same
   way the existing `assembly.png` and animation workflows work. It is cheap and
   in colour, but raster only, and the camera must be given explicitly.
2. **IKEA-style line art (vector).**
   * Export each instance once, as an STL in world coordinates. Cache it by the
     part's geometry ID plus the instance's placement.
   * Run the hidden-line engine from the part drawings on the union of all
     installed instances. It draws:
     * old parts as thin grey lines,
     * new parts as bold black lines with a light fill,
     * correct occlusion between parts.
   * Output is a crisp vector PDF with no rendering farm.

Common to both:

* **Camera.** Each phase has a default view, and each step can override it with
  `view: {azimuth: 35, elevation: 25, zoom: new}`. `zoom: new` frames the new
  parts with some surrounding context. A "turn the model" icon is added when the
  view swings by more than 90°.
* **Motion arrows (exploded "action diagrams").** Each step can declare
  `motion: {parts: [T1, T2], direction: [1, 0, 0], distance: 400}`. The new parts
  are drawn pulled back along `direction`, with dashed guide lines and an arrow.
  Later, `direction` can default to the contact-face normal between the new part
  and parts already installed.
* **Callout boxes.**
  * Thumbnails reuse the isometric view from each part drawing, labelled with the
    part number and `xN`.
  * Hardware is drawn at 1:1 next to a length ruler.
  * Sub-assembly steps (`subassembly:` / `make:`) are drawn framed, with the `x2`
    badge, on the page of the step that `uses` them. Large ones get their own
    pages, as LEGO does.
* **Automatic icons.**
  * Two-person/hoist when a new part's calculated mass is over a set limit
    (e.g. 25 kg).
  * Welding when the step builds a weldment.
  * The jigs from `tools:`.
* **Output.** A PDF manual as a CI artifact, next to the drawing book. An HTML
  version with one step per screen would suit phones in the shop.

## 6. Checks the sequence makes possible later

* **Interference per step.** Each step's new parts must not overlap anything
  already installed. The repo already has
  [`tools/collision-check`](../tools/collision-check/) for pair checks.
* **Insertion path.** Sweep the new parts along `motion.direction` and check that
  nothing blocks them. This is what the robotics assembly-planning literature
  automates.
* **Tool access.** Is there wrench clearance around every bolt? This needs the
  bolts modelled (Phase 4).
* **Hole alignment.** Holes in mating parts should line up. The part drawings
  already find every hole in every part; with instance placements (section 4),
  mating holes can be matched in world coordinates. That would have caught the
  A6 segment mismatch now flagged on the drawings.

## 7. Roadmap

| Phase | Deliverable | Effort | Status |
|---|---|---|---|
| 0 | Editable step list + checker + numbered list + CI | small | **done (this PR)** |
| 1 | `bom.scad` `bom_part(id, instance)`; convert call sites; instance-level checks | medium (mechanical edits to `lifetrac_v25.scad`) | proposed |
| 2 | First manual: OpenSCAD PNG per step + callout boxes + PDF artifact | medium | proposed |
| 3 | IKEA line-art renderer, motion arrows, 1:1 hardware | medium-large | proposed |
| 4 | Bolts modelled at every joint (length from grip), exact hardware counts, torque values per step | medium | proposed |
| 5 | Automated interference / insertion / hole-alignment checks per step | large | proposed |

## 8. Questions for the team

1. Should the manual show **inches first or mm first**? The drawings currently
   show `mm [in]`.
2. Should it use **Letter or A4**? Should it have **1 step or up to 4 steps per page**?
3. Should it be **wordless** (IKEA), or keep a one-line title per step (current
   draft)?
4. Should steps have **photos of real builds** (`photos:` per step) once someone
   builds a machine this way? A photo marks the step as `verified`.
5. Which sub-assemblies do builders actually make on a bench (arms? bucket?), and
   which are built on the machine? This decides the callout boxes.

## Sources

* M. Agrawala, D. Phan, J. Heiser, J. Haymaker, J. Klingner, P. Hanrahan,
  B. Tversky. *Designing Effective Step-By-Step Assembly Instructions.* ACM
  Transactions on Graphics 22(3):828-837 (SIGGRAPH 2003).
  [ACM DL](https://dl.acm.org/doi/10.1145/1201775.882352) ·
  [paper PDF](https://courses.ischool.berkeley.edu/i247/f05/readings/Agrawala_Assembly_SIGGRAPH03.pdf)
* LPub3D, an LDraw editor for LEGO-style digital building instructions:
  <https://trevorsandy.github.io/lpub3d/> ·
  [callouts](https://sites.google.com/view/workingwithlpub3d/learning-the-basics/callouts)
