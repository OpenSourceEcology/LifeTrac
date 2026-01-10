# 2026-01-10 Mechanical TODO (v25)

This file is the working backlog + design notebook for the current mechanical CAD iteration.
It is intentionally verbose so we can pause for weeks/months and resume without losing context.

## Scope

This TODO list covers the specific items requested in chat:

- Pick a wheel shaft + motor mounting design (prefer older OSE/LifeTrac patterns)
- Fix mounting of bucket
- Adopt a quick attach design (now 3 options)
- Add bottom plate tying side wall plates together
- Place crossbeam so hydraulic arms bottom out near collision (controlled bottom-out)
- Decide engine mounting (explicitly serviceable)
- Fix hole mismatch for middle vertical cross plate
- Fix bucket front: removable blade piece / off-the-shelf option
- Fix mounting of the main arm hydraulic cylinders to the underside of the arms
- Add options for wheeled vs tracked drive unit (OSE UTU/UWU dimensions to be pulled later)

## Design principles (explicit assumptions)

- Serviceability is a hard requirement, especially for the engine.
- Interfaces first: define attachment datums + hole patterns before refining geometry.
- Keep OpenSCAD parametric: make options controlled by a small number of top-level parameters.
- When unsure, prefer patterns already present in v25 CAD and older OSE conventions.

## Measurements to capture (checklist)

This section is intentionally “kitchen sink”. The idea is to capture the minimum set of numbers that prevents rework.
When you take measurements, record:

- Source: (published standard URL / manufacturer drawing / calipers on part)
- Units: mm (preferred)
- Datum definition: where is (0,0,0)? what face/edge is referenced?
- Tolerances: at least “tight/normal/loose” if exact tolerances aren’t known
- Photo/scan: include a picture of the measurement setup if possible

### A) Wheeled vs tracked drive unit interface

Capture (per option):

- Chassis mount plane definition (which face on side panels / bracket)
- Mount hole pattern (XY locations, hole diameters, edge distances)
- Axle/sprocket center location relative to mount plane
- Required clearance envelope (tire/track + motor + hoses)
- Motor removal path: what needs to come off first?

Wheel-specific:

- Wheel hub/bolt pattern (PCD, bolt count, pilot bore)
- Motor mount bolt circle (PCD, bolt size, pilot bore) if standardized
- Shaft diameter, key size, key length, hub engagement length
- Bearing choice: inner diameter, outer housing size, bolt pattern

Tracked-specific (defer until UTU dims pulled):

- Sprocket pitch + tooth count, track width
- Idler diameter + spacing
- Tensioner travel range and mounting envelope

### B) Front attachment options (SSQA true / DIY SSQA / lugs)

#### B1) True SSQA (standards-driven)

Minimum geometry to capture (from the best published drawing/spec we can get):

- Overall plate width and height
- Upper hook geometry:
	- hook bar height from bottom
	- bar/edge thickness the hook engages
	- hook angle and any radii/chamfers that matter for engagement
- Lower latch interface:
	- latch pin diameter (if pins) or slot width/height (if slots)
	- latch center-to-center spacing
	- latch offset from bottom edge
- Any “keepout zones” and clearance requirements
- Tolerance/fit intent (clearances at hooks and latches)

Also capture (practical sanity check):

- Measure at least one real SSQA implement plate (width/height/critical latch geometry) and note variance.

#### B2) DIY SSQA (fabrication-friendly)

Capture:

- Adjustment range needed to accommodate plate-to-plate variance (e.g., ±X mm at latch)
- Chosen latch mechanism dims (pin dia, wedge travel, lever arm length)
- Plate thicknesses chosen (main plate, hook bars, latch brackets)
- Weld prep requirements (bevel angles, minimum fillet sizes)

#### B3) Simple lug bucket attachment

Capture:

- Pin diameter(s) and target pin material (and whether bushings are used)
- Lug spacing (center-to-center) and lug wall thickness
- Lug length (engagement) and minimum edge distance to holes
- Allowed angular misalignment (how much “slop” is acceptable?)

### C) Bucket mounting (for any attachment mode)

Capture:

- Location of the bucket pivot axis relative to the chosen attach datum
- Cylinder lug locations relative to pivot axis
- Bucket width/depth/height (if changing from current)
- Bolt patterns that tie bucket plates together:
	- hole diameters, spacing, and edge distances
- Stiffness measures:
	- rib locations, rib thickness, gusset sizes

### D) Removable cutting edge (off-the-shelf wear edge)

Capture (once a product is chosen):

- Edge length, thickness, and profile
- Hole pattern:
	- hole diameter
	- hole spacing
	- distance from cutting edge to hole line
- Hardware spec:
	- bolt grade, nut type, washer type
- Backing bar/doubler dimensions and material

### E) Bottom plate tying side walls

Capture:

- Desired belly pan coverage (start/end Y, width X)
- Thickness and any reinforcement ribs
- Bolt pattern into side panels (hole sizes, spacing, edge distances)
- Required service cutouts:
	- hose pass-throughs
	- drain holes
	- access ports for wrenching

### F) Crossbeam placement for controlled bottom-out

Capture:

- Desired “near-collision clearance” target at the bottom position (e.g., 10–25mm)
- Exact surfaces for that clearance check (bucket plate vs arm front face, etc.)
- Cylinder minimum safe length margin (how close to full retraction is acceptable?)
- Mechanical stop concept:
	- where the stop contacts
	- contact pad size/material

### G) Engine mounting (serviceable) + rear stiffener hatch

Engine mount capture:

- Engine footprint bolt pattern (X/Y hole positions)
- Crankshaft output position relative to engine base
- Engine service access zones (spark plugs, oil drain, filter, starter)
- Removal envelope when rear stiffener plate is removed:
	- max engine width/height that can pass through
	- lift points / required clearance above
- Vibration isolator specs if used (stud size, height, durometer)

Rear stiffener hatch capture:

- Hatch opening width/height and location relative to plate edges
- Required access targets through hatch (exact components)
- Cover overlap/flange width
- Bolt pattern:
	- bolt size
	- bolt count
	- edge distance to prevent tear-out
- Reinforcement frame/ring dimensions

### H) Mid stiffener hole mismatch (debug measurements)

Capture:

- Which part(s) mismatch: mid plate vs inner panel vs outer panel
- Measured offset vector (ΔX, ΔZ) and which direction
- Whether holes are mirrored incorrectly (left/right) or referenced to wrong origin
- Actual drill/plasma kerf effect and hole clearance assumptions

### I) Lift cylinder underside mounting

Capture:

- Current lug locations in CAD (baseline numbers)
- Desired lug center relative to arm tube bottom
- Clearance to crossbeams/panels throughout full arm motion
- Hose routing bend radii and clamp locations

---

## Parametric design considerations (dependencies + interfaces)

This section answers:

- What interfaces exist between subassemblies?
- Which parameters are “source of truth” vs derived?
- What breaks (downstream) if a dimension changes?

General rule of thumb:

- Prefer **one source of truth** for any hole pattern or datum.
- Prefer **interfaces defined in one place** and *used* in many places.
- Avoid duplicating the same geometry in both part files and assembly (it drifts).

### Core global drivers (change ripple)

These parameters cause wide ripples across the model:

- Frame envelope: `MACHINE_WIDTH`, `MACHINE_LENGTH`, `MACHINE_HEIGHT` (assembly file)
- Side-panel spacing: `TRACK_WIDTH`, `SANDWICH_SPACING`, `PANEL_THICKNESS`
- Wheel packaging: `WHEEL_DIAMETER`, `WHEEL_WIDTH`, `GROUND_CLEARANCE`, `WHEEL_X_OFFSET`
- Arm kinematics: `ARM_PIVOT_Y`, `ARM_PIVOT_Z`, `ARM_MIN_ANGLE`, `ARM_MAX_ANGLE`, `ARM_SPACING`, `ARM_TIP_X`, `ARM_TIP_Z`
- Crossbeam clearance: `CROSS_BEAM_1_POS`, `CROSS_BEAM_2_POS`, `CROSS_BEAM_SIZE`, `CROSS_BEAM_CLEARANCE`
- Bucket geometry: `BUCKET_WIDTH`, `BUCKET_DEPTH`, `BUCKET_HEIGHT`, `BUCKET_PIVOT_HEIGHT_FROM_BOTTOM`

Where these are defined:

- Many are in [../mechanical_design/openscad/lifetrac_v25.scad](../mechanical_design/openscad/lifetrac_v25.scad)
- Many “part-safe” constants are in [../mechanical_design/openscad/lifetrac_v25_params.scad](../mechanical_design/openscad/lifetrac_v25_params.scad)
- Side panel part geometry (including top slope derived from arm angle) is in [../mechanical_design/openscad/parts/side_panel.scad](../mechanical_design/openscad/parts/side_panel.scad)

### Interface map (what touches what)

Below, “Upstream → Downstream” indicates dependency direction.

#### 1) Drive Unit Interface (wheel vs track)

Interfaces:

- Chassis ↔ Drive unit mount: side panel hole pattern + reinforcement strategy
- Drive unit ↔ Ground: axle/sprocket center height vs `GROUND_CLEARANCE` and `WHEEL_RADIUS`

Current implementation fragments:

- Wheel plate part: [../mechanical_design/openscad/parts/wheel_mount.scad](../mechanical_design/openscad/parts/wheel_mount.scad)
- Conceptual assemblies: [../mechanical_design/openscad/modules/wheels.scad](../mechanical_design/openscad/modules/wheels.scad)
- Side panel wheel holes (currently simplistic “wheel axle holes”): in both the assembly’s `side_panel_with_holes()` and the part’s `side_panel()`.

Parametric dependency notes:

- `WHEEL_DIAMETER` affects: side panel wheel-hole Z, front/rear wheel clearance, arm/wheel tangency calculations in params.
- `TRACK_WIDTH` and `SANDWICH_SPACING` affect: wheel X offset and therefore hose clearance and overall width.

Design recommendation:

- Define a single `drive_unit_mount_pattern()` helper (returns hole centers in a local mount plane) and use it:
	- in side panel part holes
	- in wheel mount plate alignment checks
	- later in track-unit bracket

If we later add tracks:

- Keep the mount pattern constant; only the envelope + axle/sprocket center move if needed (but that should be avoided).

#### 2) Side panels ↔ stiffeners ↔ rear structure

Interfaces:

- Side panels (inner/outer) ↔ mid stiffener plate ↔ angle irons
- Side panels ↔ rear stiffener plate (`back_stiffener_plate()`) ↔ angle irons
- Side panels ↔ rear crossmember plate (part file) ↔ platform pivots

Where this lives:

- Stiffener plate hole patterns + cutters are in the assembly file:
	- `get_stiffener_holes_a(h)` / `get_stiffener_holes_b(h)`
	- `stiffener_side_panel_cutters(is_inner, is_left)`
	- `mid_stiffener_plate(...)`
	- `back_stiffener_plate()`
	in [../mechanical_design/openscad/lifetrac_v25.scad](../mechanical_design/openscad/lifetrac_v25.scad)
- Side panel part geometry and holes live in [../mechanical_design/openscad/parts/side_panel.scad](../mechanical_design/openscad/parts/side_panel.scad)
- Rear crossmember CNC part lives in [../mechanical_design/openscad/parts/rear_crossmember.scad](../mechanical_design/openscad/parts/rear_crossmember.scad)

Parametric dependency notes:

- `ARM_MIN_ANGLE` influences the top slope of the side panel profile (part file) via `wall_height(y)`.
- `CROSS_BEAM_1_POS/CROSS_BEAM_2_POS` influence inner panel arc slots.
- `FRAME_Z_OFFSET` shifts many Z references; some parts subtract it (panel coords) while assembly uses world Z.

Mismatch risk (known issue):

- Mid stiffener hole mismatch is likely caused by diverging “coordinate transforms” and/or A/B hole patterns used inconsistently.
- There is also duplicated “side panel profile” logic between the assembly file and the part file (two different `side_panel_profile()` implementations exist), which increases drift risk.

Design recommendation:

- Choose one place for the side panel profile math (ideally the part file) and have the assembly *use* the part module, not re-implement it.
- Centralize stiffener hole patterns into one function per station, e.g. `stiffener_holes(station="mid"|"back", face="A"|"B")`.

#### 3) Loader arms ↔ crossbeam ↔ bucket cylinders

Interfaces:

- Arm geometry ↔ crossbeam position: crossbeam must connect arms and carry bucket cylinder base lugs.
- Crossbeam ↔ side panels: inner panel arc slots must clear crossbeam through motion.

Where it lives:

- Crossbeam is currently created inside the `loader_arms()` logic in [../mechanical_design/openscad/lifetrac_v25.scad](../mechanical_design/openscad/lifetrac_v25.scad) and is positioned using `CROSS_BEAM_1_POS` (with a manual shift `-76.2`).
- Bucket cylinder kinematics use `CROSS_BEAM_1_POS` and `CROSS_BEAM_MOUNT_Z_OFFSET` in `bucket_cylinders()`.

Parametric dependency notes:

- `CROSS_BEAM_1_POS` upstream affects:
	- bucket cylinder base attachment point
	- inner panel arc slot requirements
	- physical crossbeam mount-hole location
- `BUCKET_CYL_MOUNT_Z_OFFSET` and `BUCKET_PIVOT_HEIGHT_FROM_BOTTOM` upstream affect:
	- bucket lug Z placement in `bucket_attachment()`
	- bucket cylinder rod-end placement

Bottom-out control coupling:

- Changing crossbeam position changes cylinder leverage and may change min/max cylinder lengths.
- If we add a mechanical stop, it must be defined in arm space and checked against bucket/plate clearances.

Design recommendation:

- Treat crossbeam position as a first-class parameter with a declared intent:
	- “chosen to satisfy clearance + cylinder geometry”
- Avoid permanent “magic offsets” like `-76.2` without a named parameter, because they break when arm geometry changes.

#### 4) Bucket ↔ bucket attachment interface ↔ bucket cylinders

Interfaces:

- Bucket body ↔ attachment mount: bolt pattern or weldment that ties bucket to the interface plate/lugs.
- Attachment interface ↔ cylinder lugs: cylinder lugs must be in a stable, repeatable place relative to bucket pivot.

Where it lives:

- Bucket body in assembly: `bucket()` is defined in [../mechanical_design/openscad/lifetrac_v25.scad](../mechanical_design/openscad/lifetrac_v25.scad) and includes a non-replaceable modeled cutting edge.
- Bucket attachment (lugs) is currently in `bucket_attachment()` in the same file.
- Flat bucket parts exist separately:
	- [../mechanical_design/openscad/parts/bucket_bottom.scad](../mechanical_design/openscad/parts/bucket_bottom.scad)
	- [../mechanical_design/openscad/parts/bucket_side.scad](../mechanical_design/openscad/parts/bucket_side.scad)

Parametric dependency notes:

- `BUCKET_HEIGHT`, `BUCKET_PIVOT_HEIGHT_FROM_BOTTOM` upstream affect:
	- where the pivot axis intersects the bucket body
	- the translation used to place `bucket()` under `bucket_attachment()`
- `ARM_SPACING` upstream affects:
	- lug spacing (current lugs are placed at ±`ARM_SPACING/2`)
- `BUCKET_LUG_OFFSET` (if changed) will move bucket relative to lugs and must be kept consistent with cylinder geometry.

Three attachment modes impact:

- SSQA_TRUE and SSQA_DIY will introduce a new plate geometry whose dimensions affect:
	- bucket pivot location (if pivot is located on/near plate)
	- cylinder mount location if referenced to SSQA plate top
	- bucket collision envelope at full dump/curl
- LUGS mode keeps the existing dependence on `ARM_SPACING` and pin diameter.

Design recommendation:

- Define a single “Attachment Datum” coordinate frame at the end of the arms.
- Each attachment mode should implement:
	- `attachment_pivot_axis()` (position + direction)
	- `attachment_cyl_lug_points()` (two points per side: base/rod if applicable)
	- `attachment_envelope()` (for collision checks)

#### 5) Engine ↔ rear stiffener plate ↔ service access

Interfaces:

- Engine mount ↔ frame (bolt pattern + isolators)
- Engine service access ↔ rear stiffener plate (removal path + hatch opening)

Where it lives:

- Engine placeholder + TODO is in [../mechanical_design/openscad/lifetrac_v25.scad](../mechanical_design/openscad/lifetrac_v25.scad)
- Rear stiffener plate is `back_stiffener_plate()` in the same file.
- Rear crossmember is a separate CNC part in [../mechanical_design/openscad/parts/rear_crossmember.scad](../mechanical_design/openscad/parts/rear_crossmember.scad)

Parametric dependency notes:

- If we add a hatch cutout:
	- it depends on stiffener hole locations (must not overlap)
	- it changes plate stiffness; reinforcement design must be linked to opening size
- If the engine footprint changes:
	- mount plate hole pattern changes
	- removal envelope through the rear opening may no longer work

Design recommendation:

- Treat “engine removal envelope” as a design constraint and encode it as:
	- a simple bounding box in CAD
	- plus a keepout zone for hatch bolts/angles.

#### 6) Lift cylinders underside mounting ↔ side panels ↔ arm geometry

Interfaces:

- Lift cylinder base mount ↔ side panels (holes + lug bracket)
- Lift cylinder rod end ↔ arm (underside lug location)

Where it lives:

- Side panel part includes the lift cylinder base mount hole (in part coords) in [../mechanical_design/openscad/parts/side_panel.scad](../mechanical_design/openscad/parts/side_panel.scad)
- Assembly file has a different `side_panel_with_holes()` which currently puts the base hole only on inner panels (note: this differs from the part file).
- `lift_cylinders()` in [../mechanical_design/openscad/lifetrac_v25.scad](../mechanical_design/openscad/lifetrac_v25.scad) places the rod-end attach point as `arm_local_attach = [0, LIFT_CYL_ARM_OFFSET, -ARM_TUBE_SIZE[0]/2 - 30]` (already “below” the arm).

Parametric dependency notes:

- `LIFT_CYL_BASE_Y`, `LIFT_CYL_BASE_Z` drive base mount holes and lug placement.
- `LIFT_CYL_ARM_OFFSET` drives leverage and therefore lift capacity.
- Arm tube size affects “underside” Z.

Design recommendation:

- Eliminate duplicated base-mount hole logic (assembly vs part file) and make the part file authoritative.
- Promote underside-mount offset (`-ARM_TUBE_SIZE[0]/2 - 30`) to a named parameter (easier to tune).

## Where things live (quick map)

- Main assembly + many inlined modules: [../mechanical_design/openscad/lifetrac_v25.scad](../mechanical_design/openscad/lifetrac_v25.scad)
- Shared constants for part files: [../mechanical_design/openscad/lifetrac_v25_params.scad](../mechanical_design/openscad/lifetrac_v25_params.scad)
- Wheel concepts: [../mechanical_design/openscad/modules/wheels.scad](../mechanical_design/openscad/modules/wheels.scad)
- Wheel motor mount plate part: [../mechanical_design/openscad/parts/wheel_mount.scad](../mechanical_design/openscad/parts/wheel_mount.scad)
- Bucket plates (flat CNC parts):
	- [../mechanical_design/openscad/parts/bucket_bottom.scad](../mechanical_design/openscad/parts/bucket_bottom.scad)
	- [../mechanical_design/openscad/parts/bucket_side.scad](../mechanical_design/openscad/parts/bucket_side.scad)
- Manufacturing status / hole-feature expectations: [../mechanical_design/CNCLAYOUT_STATUS.md](../mechanical_design/CNCLAYOUT_STATUS.md)
- AI progress tracker: [../mechanical_design/AI_DESIGN_PROGRESS.md](../mechanical_design/AI_DESIGN_PROGRESS.md)
- Historical notes: [conversation_log.md](conversation_log.md)
- Code reviews:
	- [CODE REVIEWS/2025-01-09_Review_ClaudeOpus4_5.md](CODE%20REVIEWS/2025-01-09_Review_ClaudeOpus4_5.md)
	- [CODE REVIEWS/2026-01-09_Review_Gemini3Pro.md](CODE%20REVIEWS/2026-01-09_Review_Gemini3Pro.md)
	- [CODE REVIEWS/2026-01-09_Review_GPT5_2.md](CODE%20REVIEWS/2026-01-09_Review_GPT5_2.md)

---

## 0) Master decision list (fill these in as we go)

This is the small set of decisions that will unblock many tasks.

### 0.1 Drive unit option: wheeled vs tracked

Decision: should the chassis hardpoints support both wheel units and track units with no re-cut of side panels?

Notes:

- Current v25 assumes wheels, including `show_wheels`, `all_wheel_drive`, wheel geometry, and `WHEEL_X_OFFSET` in [../mechanical_design/openscad/lifetrac_v25.scad](../mechanical_design/openscad/lifetrac_v25.scad).
- We will retrieve OSE UTU/WUW dimensions later:
	- https://wiki.opensourceecology.org/wiki/Universal_Track_Unit
	- https://wiki.opensourceecology.org/wiki/Universal_Wheel_Unit

Proposed approach (interface-first):

1. Define a Drive Unit Interface with a coordinate system:
	 - `DU_MOUNT_PLANE`: the unit’s chassis attachment plane
	 - `DU_DATUM`: a repeatable point on the side panel (e.g., axle center projection)
	 - `DU_CENTERLINE_Z`: ground reference is Z=0 in current model
2. Define a small set of required interface fields:
	 - `du_mount_hole_pattern`: hole locations in chassis coordinates (or mount plane local 2D)
	 - `du_mount_plate_thickness`
	 - `du_axle_or_sprocket_center`: location relative to mount plane
	 - `du_envelope_bbox`: clearance bounding box for tire/track + motor
	 - `du_service_access_zone`: volumes that must remain unobstructed
3. Implement wheel version using existing `wheel_mount()` plate + wheel modules.
4. Implement track placeholder module later when UTU dims are known.

Acceptance checks:

- Wheel and track variants both bolt to the same chassis hardpoints.
- No collision with side panels through full motion range.
- Service access (motor removal, coupler, tensioner) is possible with common tools.

Unknowns to capture later:

- UTU/WUW top pages do not include bolt patterns/shaft sizes; expect to pull from linked CAD/BOM pages later.

### 0.2 Front attachment option: 3 modes

Decision: support 3 selectable front attachment interface types:

1) True SSQA standard (standards-driven; must match published spec)
2) DIY SSQA (fabrication-friendly, compatible with common SSQA plates via adjustability)
3) Simple bucket lugs (current U-channel lug/pin approach)

We should define a single selector parameter (example):

- `ATTACHMENT_MODE = "SSQA_TRUE" | "SSQA_DIY" | "LUGS"`

#### True SSQA (standards-driven)

- Published standard reference: SAE J2513 (public listing, paywalled dimensions)
	- https://www.sae.org/standards/content/j2513_201804/
- Interim public guidance sources (use for sanity checks, not tolerance-grade):
	- https://farmhack.org/tools/skidsteer-quick-attach-standards-sae-j2513
	- https://www.everythingattachments.com/category-s/10525.htm
	- Diagram image: https://www.everythingattachments.com/v/vspfiles/assets/images/SSQA%20CAD.jpg

- Off-the-shelf Bob-Tach-style assembly option (example product, dimensions not listed on page):
	- https://www.fridayparts.com/bobtach-mounting-system-assembly-7276373-for-bobcat-skid-steer-loader-s510-s530-s550-s570-s590-s595-t550-t590-s595
	- Notes to capture when we revisit: does it include the wedges/levers/latch pins, and what are the exact critical dims (hook bar, latch geometry)?

We already have “approximate Bobcat QA” parameters in the assembly file:

- `BOBCAT_QA_WIDTH`, `BOBCAT_QA_HEIGHT`, `BOBCAT_QA_HOOK_HEIGHT`, `BOBCAT_QA_WEDGE_HEIGHT`, `BOBCAT_QA_PIN_DIA` in [../mechanical_design/openscad/lifetrac_v25.scad](../mechanical_design/openscad/lifetrac_v25.scad)

Required geometry fields to fill in from a published drawing/spec:

- Datum definition: attachment plane vs chassis plane
- Upper hook bar location, bar diameter/thickness, hook angles
- Lower latch pin/slot geometry (center spacing, diameters/slot widths)
- Overall envelope (width/height) and edge distances
- Tolerance strategy (if SAE doc not accessible, we must choose conservative clearances)

Acceptance checks:

- A purchased SSQA implement plate should physically latch on without grinding.
- Top hooks fully engage; latch reaches fully locked position.

#### DIY SSQA (fabrication-friendly)

Goal: keep SSQA engagement geometry but simplify fabrication and explicitly include adjustability.

Ideas:

- Simplified latch (manual pins + retainers) or a simple lever wedge.
- Oversized slots + shim packs.
- Adjustment window so we can fit real-world plates.

Acceptance checks:

- Can be built with flat plate + a few welded bars.
- Test-fit checklist is written and repeatable.

#### Simple bucket lugs (current)

Current approach uses U-channel lugs + pins; related modules exist in the assembly file:

- `u_channel_lug(...)`
- `u_channel_lug_with_pin(...)`
- `bucket_attachment()`

all in [../mechanical_design/openscad/lifetrac_v25.scad](../mechanical_design/openscad/lifetrac_v25.scad)

Acceptance checks:

- Pin insertion/removal is possible without prying.
- Lug spacing is standardized so future tools (forks/grapple) can reuse it.

### 0.3 Engine: serviceability-first

Assumptions (explicit):

- Engine install/removal: remove rear stiffener plate (rear back plate) then lift/slide engine in/out.
- A bolt-on hatch in the rear stiffener plate is for inspection/service, not full engine removal.

---

## 1) Wheel shaft + motor mounting design

Goal: choose a shaft + bearing + motor mount strategy that is manufacturable, reliable, and serviceable.

Current state (in repo):

- Wheel motor mounting plate exists as a CNC part: `wheel_mount()` in [../mechanical_design/openscad/parts/wheel_mount.scad](../mechanical_design/openscad/parts/wheel_mount.scad)
	- 80mm center hole, 8-bolt circle, 4 corner holes.
- Conceptual wheel/axle/bearing modules exist in [../mechanical_design/openscad/modules/wheels.scad](../mechanical_design/openscad/modules/wheels.scad)
	- `axle()` has a keyway option.
	- `bearing_housing()` exists but is generic.
	- `modular_wheel_unit()` is a simplified mount concept.

Decisions needed (write down final answers):

- Shaft diameter standard (2", 2.5", 3"?)
- Bearing type (pillow block vs flange vs custom housing)
- Motor coupling (keyed shaft + hub vs splined coupler vs clamp coupler)
- Motor mount bolt circle standard and bolt size

Serviceability constraints:

- Prefer motor removal without removing the entire wheel.
- Prefer bearing replacement without disassembling side panels.
- Protect motor/hoses from debris.

Next actions:

1. Define the Drive Unit Interface in writing (datum, mount plane, axle center).
2. Decide whether `wheel_mount.scad` becomes a “hard standard” plate (preferred) or a parameterized plate.
3. Add notes about how the wheel plate bolts to side panels (edge distance, reinforcement), cross-reference [../mechanical_design/CNCLAYOUT_STATUS.md](../mechanical_design/CNCLAYOUT_STATUS.md).

Acceptance criteria:

- We can name a specific shaft + bearing concept (even if vendor TBD), and encode constraints in CAD.
- No motor mount interference with wheel rim/hub.

Open questions:

- Should the wheel mount plate carry all loads, or should there be a weldment/bracket behind it?

---

## 2) Fix mounting of bucket

Goal: make bucket attachment stiff, repeatable, and compatible with the chosen attachment option.

Current state:

- Bucket flat parts:
	- [../mechanical_design/openscad/parts/bucket_bottom.scad](../mechanical_design/openscad/parts/bucket_bottom.scad) includes “Back edge mounting holes for quick attach”.
	- [../mechanical_design/openscad/parts/bucket_side.scad](../mechanical_design/openscad/parts/bucket_side.scad)
- In the full assembly, `bucket_attachment()` and bucket-cylinder lug placement are in [../mechanical_design/openscad/lifetrac_v25.scad](../mechanical_design/openscad/lifetrac_v25.scad).

Main risk:

- Bucket mounting logic is split between part plate hole patterns and assembly lug alignment logic.

Decisions needed:

- For each attachment mode (SSQA/DIY/lugs), what is the bucket’s primary load path?
	- direct welded bucket-to-plate?
	- bolt-on backplate?
	- bolt pattern tied into the bottom plate holes?

Next actions:

1. Write a bucket mounting interface contract: fastener sizes, plate thickness, mount datum.
2. Decide whether “back edge mounting holes” correspond to SSQA backplate bolting, lug bracket bolting, or are legacy.
3. Plan stiffening ribs/gussets so the bucket doesn’t rack.

Acceptance criteria:

- Bucket mount points are explicit and consistent between parts and assembly.
- Bolt patterns are documented and manufacturable.

---

## 3) Adopt quick attach design (3 options)

Goal: implement a clean selectable interface rather than one-off geometry.

Plan:

- Add selector parameter `ATTACHMENT_MODE`.
- Implement each mode in its own module with a shared datum.

Per-mode TODO:

- True SSQA: fill in SAE J2513-driven dimensions once we obtain a published drawing/spec.
- DIY SSQA: define adjustability windows and a test-fit procedure.
- Lugs: standardize lug spacing and pin sizes.

Acceptance criteria:

- CAD can render all three modes without breaking downstream geometry.

---

## 4) Add a bottom plate connecting the side wall plates

Goal: add a plate at the bottom of the machine tying left/right side panel structure together, improving torsional stiffness.

Notes:

- There is a rear crossmember plate part: [../mechanical_design/openscad/parts/rear_crossmember.scad](../mechanical_design/openscad/parts/rear_crossmember.scad)
- The chassis base is also built from tubing in the assembly.

Decisions needed:

- Is the bottom plate a full belly pan (also protects hoses) or just a structural tie plate (smaller with cutouts)?
- Does it need drain holes and access ports?

Next actions:

1. Identify where it bolts: inner panels, outer panels, or both.
2. Choose thickness and whether it includes welded nut plates.
3. Add service cutouts: engine/pump lines, inspection access.

Acceptance criteria:

- Plate exists in 2D CNC outputs and has defined mount holes.

---

## 5) Place crossbeam for controlled bottom-out

Goal: place crossbeam so the hydraulic arms bottom out when the bucket/quick attach plate almost collides with the front of the arms.

Current state:

- Crossbeam geometry and arc-slot cutout calculations are in [../mechanical_design/openscad/lifetrac_v25.scad](../mechanical_design/openscad/lifetrac_v25.scad).
- `CROSS_BEAM_2_POS` is defined; `CROSS_BEAM_1_POS` appears to be commented/reworked.

Interpretation (define what “bottom-out” means):

1) Cylinder reaches minimum length (bad as a hard stop)
2) Arm hits mechanical stop (preferred)
3) Bucket/plate collides with arm (avoid; use only as “near-limit”)

Next actions:

1. Decide the intended hard stop (mechanical stop preferred).
2. Define a numeric clearance target (e.g., 10–25mm) between bucket/plate and arm at minimum position.
3. Adjust crossbeam position logic so the stop happens at that clearance.
4. Verify arc slots still clear the crossbeam through the motion range.

Acceptance criteria:

- At lowest position, clearance is near target, with no collision.
- Cylinders are not used as the hard stop.

---

## 6) Decide engine mounting (serviceable) + bolt-on rear stiffener hatch

Goal: engine mounting must be easy to install/remove/maintain.

Current state:

- Engine is represented and explicitly notes missing mount plate: `// TODO: Design proper engine mounting plate and bolt pattern` in [../mechanical_design/openscad/lifetrac_v25.scad](../mechanical_design/openscad/lifetrac_v25.scad).
- Rear stiffener plate module: `back_stiffener_plate()` is in the same file.

Service procedure assumptions:

- Engine install/removal: remove rear stiffener plate.
- Hatch is for inspection/service only.

Engine mount design decisions:

- Rubber isolators (recommended) vs solid mount.
- Slotted holes for belt/chain alignment.
- One plate with stiffeners vs two rails.

Serviceability checklist:

- Access to spark plugs, oil drain, filter, starter.
- Room for hands/tools to disconnect hoses/wires.
- No trapped fasteners behind inaccessible plates.

Rear stiffener hatch (bolt-on) design notes:

- Must not conflict with stiffener-to-side-panel bolts or angle irons.
- Must restore stiffness using a bolt-on reinforcement frame/ring.

Parameters to define later:

- `HATCH_OPENING_W`, `HATCH_OPENING_H`, `HATCH_CORNER_R`
- `HATCH_BOLT_COUNT` or spacing
- `HATCH_FLANGE_OVERLAP`
- `HATCH_FRAME_THICKNESS`

Acceptance criteria:

- Hatch can be removed with common tools.
- Hatch provides meaningful access for maintenance.

---

## 7) Fix hole mismatch for middle vertical cross plate

Goal: resolve mismatch between the mid stiffener plate holes and side panel holes.

Where to look:

- `mid_stiffener_plate(...)` in [../mechanical_design/openscad/lifetrac_v25.scad](../mechanical_design/openscad/lifetrac_v25.scad)
- Side panel hole cutters: `stiffener_side_panel_cutters(is_inner, is_left)`
- Hole pattern generators: `get_stiffener_holes_a(h)` and `get_stiffener_holes_b(h)`

Hypothesis:

- Different hole pattern functions (A/B) or reference heights are being used in different places.

Next actions:

1. Document the mismatch (which holes, what offset, which exported parts).
2. Choose a single source of truth (panel holes or plate holes).
3. Centralize hole pattern generation to prevent drift.

Acceptance criteria:

- Exported mid stiffener plate bolt holes align with side panel holes without forcing.

---

## 8) Fix bucket front: removable cutting edge (off-the-shelf)

Goal: make the cutting edge a replaceable wear part.

Current notes:

- [../mechanical_design/CNCLAYOUT_STATUS.md](../mechanical_design/CNCLAYOUT_STATUS.md) calls out “Wear plate attachment holes (if applicable)”.

Decisions needed:

- Choose a target off-the-shelf edge (vendor/model) and match its hole pattern.
- Decide bolt size and backing reinforcement strategy.

Next actions:

1. Pick an actual wear edge product.
2. Add holes to the bucket front/bottom to match.
3. Add a backing bar/doubler so bolts don’t tear out.

Acceptance criteria:

- Cutting edge can be replaced without cutting/welding.

---

## 9) Fix mounting of main arm lift cylinders to underside of arms

Goal: mount the lift cylinders to the underside/bottom side of the arms for clearance and protection.

Current state:

- Lift cylinder mounting calculations are in [../mechanical_design/openscad/lifetrac_v25_params.scad](../mechanical_design/openscad/lifetrac_v25_params.scad).

Decisions needed:

- Define “underside” in arm coordinates and desired offsets.
- Confirm pinch points and hose routing constraints.

Next actions:

1. Document current cylinder lug placements.
2. Set a target lug Z (relative to arm bottom).
3. Verify no collisions through full range.

Acceptance criteria:

- Cylinders do not collide with crossbeams/panels/bucket linkage through full range.

---

## Suggested execution order

1) Lock attachment interface modes (SSQA/DIY/lugs) as an abstraction
2) Fix bucket mounting around the chosen interface(s)
3) Fix mid stiffener hole mismatch (unblocks physical build)
4) Add bottom plate (and confirm it doesn’t block service)
5) Re-tune crossbeam position for controlled bottom-out
6) Design engine mount + rear stiffener bolt-on hatch (serviceability-first)
7) Decide wheel shaft/motor mount strategy
8) Add tracked option placeholder and defer UTU dims capture

## Notes log (append-only)

- 2026-01-10: Added 3 front attachment modes; SSQA standard is SAE J2513 (dims paywalled).
- 2026-01-10: Rear stiffener hatch will be bolt-on; assume engine install/removal by removing rear stiffener plate.