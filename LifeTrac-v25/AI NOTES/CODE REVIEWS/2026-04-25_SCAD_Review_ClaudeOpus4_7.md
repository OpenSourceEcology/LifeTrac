# SCAD Code Review: LifeTrac v25 Mechanical Design
**Reviewer:** Claude Opus 4.7
**Date:** 2026-04-25

## Scope
Review of the OpenSCAD model under [LifeTrac-v25/mechanical_design/openscad/](../../mechanical_design/openscad/). Files audited:

- Top-level assemblies: [lifetrac_v25.scad](../../mechanical_design/openscad/lifetrac_v25.scad) (~225 KB), [lifetrac_v25_UTU.scad](../../mechanical_design/openscad/lifetrac_v25_UTU.scad) (~61 KB), [lifetrac_v25_params.scad](../../mechanical_design/openscad/lifetrac_v25_params.scad) (~68 KB)
- Modules: [modules/loader_arm_v2.scad](../../mechanical_design/openscad/modules/loader_arm_v2.scad), [modules/hydraulics.scad](../../mechanical_design/openscad/modules/hydraulics.scad), [modules/structural_steel.scad](../../mechanical_design/openscad/modules/structural_steel.scad), [modules/plate_steel.scad](../../mechanical_design/openscad/modules/plate_steel.scad), [modules/wheels.scad](../../mechanical_design/openscad/modules/wheels.scad), [modules/fasteners.scad](../../mechanical_design/openscad/modules/fasteners.scad)
- Parts: [parts/arm_plate.scad](../../mechanical_design/openscad/parts/arm_plate.scad), [parts/pivot_mount_assembly.scad](../../mechanical_design/openscad/parts/pivot_mount_assembly.scad), [parts/structural/](../../mechanical_design/openscad/parts/structural/) sub-tree, plus the various `export_*.scad` thin wrappers
- CNC layout: [cnclayout.scad](../../mechanical_design/openscad/cnclayout.scad), [cnclayout_simple_outlines_backup.scad](../../mechanical_design/openscad/cnclayout_simple_outlines_backup.scad), [export_for_cnc.scad](../../mechanical_design/openscad/export_for_cnc.scad)
- 3D-printed jigs under [3d_printed_bolt_hole_cutting_jigs/](../../mechanical_design/openscad/3d_printed_bolt_hole_cutting_jigs/) and [3d_printed_welding_jigs/](../../mechanical_design/openscad/3d_printed_welding_jigs/)
- Test/development files: [test_cylinder_sizing.scad](../../mechanical_design/openscad/test_cylinder_sizing.scad), [test_check.csg](../../mechanical_design/openscad/test_check.csg), [test_output.csg](../../mechanical_design/openscad/test_output.csg)

This review focuses on **OpenSCAD code quality**: correctness, maintainability, fragility, and the geometry/parameter-modelling patterns. Mechanical design correctness is covered by other reviews and the existing structural analysis; this pass calls out *code-side* problems that put that geometry at risk.

---

## Critical Findings

### 1. Material/dimension constants duplicated between main assembly and params
[lifetrac_v25.scad](../../mechanical_design/openscad/lifetrac_v25.scad#L1) does `include <lifetrac_v25_params.scad>` and then **re-declares** the same constants:

- `PLATE_1_4_INCH`, `TUBE_3X3_1_4`, `TUBE_4X4_1_4`, `TUBE_2X2_1_4`, `TUBE_2X4_1_4` — re-declared at [lifetrac_v25.scad](../../mechanical_design/openscad/lifetrac_v25.scad#L62-L68).
- `MACHINE_WIDTH`, `MACHINE_LENGTH`, `MACHINE_HEIGHT`, `WHEEL_BASE`, `TRACK_WIDTH`, `SANDWICH_SPACING`, `WHEEL_DIAMETER`, `WHEEL_WIDTH`, `WHEEL_CLEARANCE`, `GROUND_CLEARANCE`, `WHEEL_X_OFFSET` — re-declared at [lifetrac_v25.scad](../../mechanical_design/openscad/lifetrac_v25.scad#L83-L102).
- `ENGINE_HP`, `ENGINE_WEIGHT_KG`, `ENGINE_POS_Y`, `ENGINE_POS_Z` — re-declared at [lifetrac_v25.scad](../../mechanical_design/openscad/lifetrac_v25.scad#L112-L120) *and* re-declared again in [lifetrac_v25_UTU.scad](../../mechanical_design/openscad/lifetrac_v25_UTU.scad#L77-L80).
- `PIVOT_PIN_DIA`, `BOLT_DIA_*` — re-declared.

OpenSCAD treats these as variable assignments scoped to the file; the included value will be used unless the local re-declaration shadows it. The current values happen to match, so the model still renders, but:

- Any future change in [lifetrac_v25_params.scad](../../mechanical_design/openscad/lifetrac_v25_params.scad) that does not also propagate to the duplicate block will silently diverge between the assembly and the parts (parts files `include` params, the assembly `include`s params and then overrides).
- OpenSCAD does emit `WARNING: ... unknown variable` and assignment-shadowing warnings depending on order; this drowns real warnings.

**Fix:** make `lifetrac_v25_params.scad` the single source of truth and delete the duplicate constant blocks from `lifetrac_v25.scad` and the manually-mirrored block in `lifetrac_v25_UTU.scad` (see finding 2 for the underlying cause of the UTU duplication).

### 2. UTU variant manually re-derives ~60 top-level constants from the main assembly
[lifetrac_v25_UTU.scad](../../mechanical_design/openscad/lifetrac_v25_UTU.scad#L43-L172) explains the problem itself:

> Variables computed at top-level scope in lifetrac_v25.scad are NOT imported by `use`. We re-derive all variables needed by the modules we call (side panels, stiffeners, loader arms, etc.).

Re-derived constants include `BOLT_DIA_3_8`, `BOTTOM_PLATE_*`, `FRAME_TUBE_*`, `_FRONT_WHEEL_AXIS_Y`, `_REAR_WHEEL_AXIS_Y`, `FRONT_FRAME_TUBE_Y`, `REAR_FRAME_TUBE_Y`, `ANGLE_SEGMENT_*`, `MOTOR_PLATE_*`, `MIN_BOLT_SPACING`, `UWU_SHAFT_Z`, `INNER_WALL_PANEL_X`, `BUCKET_CYL_*`, `HYDRAULIC_PRESSURE_*`, `BUCKET_MIN_TILT`, etc.

This is a known OpenSCAD limitation, but the current workaround means **two files have to be kept in lockstep by hand**. A change in [lifetrac_v25.scad](../../mechanical_design/openscad/lifetrac_v25.scad) (e.g. moving `_REAR_WHEEL_AXIS_Y` formula) will silently produce a UTU model that disagrees with the UWU model.

**Fix:** factor the derived-geometry block out of [lifetrac_v25.scad](../../mechanical_design/openscad/lifetrac_v25.scad) into a new `lifetrac_v25_derived.scad` and have *both* the UWU and UTU top-levels `include` it. Then nothing needs to be re-derived by hand.

### 3. Pervasive `is_undef(x) ? default : x` guards hide cross-file breakage
There are dozens of patterns like:

```scad
_arm_max_for_animation = is_undef(ARM_MAX_ANGLE_LIMITED) ? ARM_MAX_ANGLE : ARM_MAX_ANGLE_LIMITED;
_pivot_y_offset_safe   = is_undef(BUCKET_PIVOT_Y_OFFSET) ? -64.6 : BUCKET_PIVOT_Y_OFFSET;
_lug_z_safe            = is_undef(_lug_z_local) ? 242.8 : _lug_z_local;
_arm_tip_x_safe        = is_undef(ARM_TIP_X) ? 1500 : ARM_TIP_X;
_PIVOT_MOUNT_BOLT_ANGLES = is_undef(PIVOT_MOUNT_BOLT_ANGLES) ?
    [295.5, 372.6, 449.8, 526.9, 604.1] : PIVOT_MOUNT_BOLT_ANGLES;
```

These appear in [lifetrac_v25_params.scad](../../mechanical_design/openscad/lifetrac_v25_params.scad), [parts/arm_plate.scad](../../mechanical_design/openscad/parts/arm_plate.scad#L14-L21), [modules/loader_arm_v2.scad](../../mechanical_design/openscad/modules/loader_arm_v2.scad), and elsewhere. The defaults are **stale snapshots** of values that the params file actively recomputes.

Concrete problems:
- If a typo or a future refactor breaks the upstream definition of `BUCKET_PIVOT_Y_OFFSET`, the part silently falls back to `-64.6` mm and the assembly still renders — but with the wrong geometry. Silent magic-number drift is much worse than a render-time error.
- The "default" arrays (e.g. `[295.5, 372.6, 449.8, 526.9, 604.1]`) duplicate logic that the params file actually computes from `PIVOT_MOUNT_SLOT_ANGLE`, `PIVOT_MOUNT_BOLT_COUNT`, etc. Change the slot angle and the parts file *won't follow* unless the upstream variable also exists.

**Fix:** remove the `is_undef(...) ? default : ...` fallbacks. If a part needs a constant, it should `include <../lifetrac_v25_params.scad>`; if the constant is missing, that should fail loudly with `assert(!is_undef(BUCKET_PIVOT_Y_OFFSET), "BUCKET_PIVOT_Y_OFFSET must be defined by params")`.

### 4. Arm geometry solver returns `undef` on infeasible configurations and propagates silently
In [lifetrac_v25_params.scad](../../mechanical_design/openscad/lifetrac_v25_params.scad#L249-L253), `solve_L2_func` returns `undef` when the discriminant goes negative (no triangle solution). Downstream:

```scad
function solve_E_func(l1, P, T, C, ang_deg) =
    let( ...
        l2 = solve_L2_func(l1, d, ang_deg),
        cos_alpha = (l2 == undef) ? 0 : ...,
        ...
    )
    (l2 == undef) ? [0,0] : (P + ...);
```

A `[0,0]` elbow position fed into `eval_balance` (which then drives the binary search `find_optimal_L1`) will return a numerically valid but **geometrically meaningless** result — the search will happily converge on whichever direction `dist_line_func(C, [0,0], [0,0])` happens to push it (and that function returns `9999` when `base==0`, so the search will be biased into one half-space).

Today the search range `[1200, 1600]` happens to be entirely feasible for the documented machine size. If anyone changes `ARM_PIVOT_Z`, `ARM_PIVOT_Y`, `WHEEL_BASE`, or `WHEEL_RADIUS` such that part of the search interval is infeasible, the solver will silently return a wrong arm length and the model will still render.

**Fix:** make `find_optimal_L1` `assert(!is_undef(l2), "Arm solver: infeasible L1 in search interval")` before recursing. Better, do a bracketing pre-check.

### 5. `hydraulic_cylinder()` silently clamps `actual_extension` to `[0, stroke]`
At [modules/hydraulics.scad](../../mechanical_design/openscad/modules/hydraulics.scad#L57):

```scad
actual_extension = max(0, min(stroke, target_extension));
```

When `pin_to_pin_length` is supplied but does not fall in the `[closed, extended]` range of the cylinder, the model **clamps** to the limit and renders a perfectly happy cylinder at full retract or full extend. The render then visually agrees with the rest of the assembly — even though the geometry actually requires the cylinder to be longer or shorter than it physically can be.

This is the "silent failure" pattern the params file's lift-cylinder solver explicitly tries to prevent (it echoes "WARNING: Cylinder too short at max extension!" at [lifetrac_v25.scad](../../mechanical_design/openscad/lifetrac_v25.scad#L621-L625)). The hydraulics module should do the same.

**Fix:** when clamping, `echo` a clear warning *and* render the cylinder with a contrasting color (e.g. red) so it is obvious in the viewport. Optionally `assert` in non-preview mode.

---

## High-Severity Findings

### 6. Echo storm — every render dumps hundreds of `echo()` lines
`echo()` is used as a development log throughout. Each F5/F6 in OpenSCAD now produces (rough count) 200+ lines:

- `=== ARM GEOMETRY CALCULATION ===`
- `=== PIVOT MOUNT BOLT PATTERN ===`
- `=== LIFT CYLINDER PARAMETRIC STROKE CALCULATION ===` (~25 lines on its own)
- `=== BUCKET CYLINDER LUG DEBUG ===`, `STEP 1`..`STEP 8` (~30 lines)
- `=== UTU SPROCKET PHASES ===`, `=== UTU AXIS POSITIONS ===`, `=== UTU FRAME TUBE EXTENSION ===`
- `=== HYDRAULIC FORCE CALCULATION ===`, `=== LIFTING CAPACITY ANALYSIS ===`, `=== STRUCTURAL ANALYSIS ===`, ...

Real `WARNING:` lines from OpenSCAD itself get buried. The duplicated constants in finding 1 will produce variable-shadowing warnings that nobody will see.

**Fix:** gate all debug echoes behind a global `DEBUG = false;` flag and `if (DEBUG) echo(...)`. Keep only `echo` calls that report computed *outputs* a fabricator needs (e.g. final cylinder strokes, arm length, BOM-relevant dimensions).

### 7. `$fn` set inconsistently across files
- [lifetrac_v25.scad](../../mechanical_design/openscad/lifetrac_v25.scad#L36) sets `$fn = 32` at top-level.
- [lifetrac_v25_UTU.scad](../../mechanical_design/openscad/lifetrac_v25_UTU.scad#L184) also sets `$fn = 32`.
- [modules/structural_steel.scad](../../mechanical_design/openscad/modules/structural_steel.scad#L6) and [modules/hydraulics.scad](../../mechanical_design/openscad/modules/hydraulics.scad#L6) set `$fn = 32` at the top of *module* files.
- Many primitives override locally with `$fn=16, 24, 32, 48, 64`.

`$fn` set at the top of a module file consumed via `use <>` does **not** propagate (top-level statements are ignored). Set via `include <>` it does. Half the files end up at the file-set value, half end up at the per-call override. This makes resolution unpredictable, especially for the bearing-flange / sprocket renders.

**Fix:** define `$fa`/`$fs` (preferred) or `$fn` once at the top of each top-level assembly file (`lifetrac_v25.scad`, `lifetrac_v25_UTU.scad`, the export wrappers) and remove `$fn = ...` from the *module* and *part* files. Use special-case overrides inline only where needed (e.g., `$fn=64` for visible round plates).

### 8. `lifetrac_v25_UTU.scad` overrides display toggles that the main file already declared
The UTU file `use`s [lifetrac_v25.scad](../../mechanical_design/openscad/lifetrac_v25.scad), then *re-declares* `show_wheels`, `show_uwu`, `all_wheel_drive`, `exploded_view`, `explode_distance`, `show_frame`, `show_hydraulics`, `show_loader_arms`, `show_bucket`, `show_folding_platform`, `platform_fold_angle` (lines 65-69 and 188-194).

This works only because `use <>` does not execute top-level statements, so the main file's defaults never run. It is a **leaky abstraction**: someone reading `lifetrac_v25.scad` would reasonably assume the toggles defined there control all consumers, but the UTU variant ignores them.

**Fix:** factor toggles into a `lifetrac_v25_toggles.scad` that both top-level files `include`. Or — better — keep the toggles in the params file and remove the duplicates in both top-level files.

### 9. `arm_plate.scad` calls `arm_plate();` at top level
[parts/arm_plate.scad](../../mechanical_design/openscad/parts/arm_plate.scad#L23) renders the part when the file is opened directly, which is intentional and useful. But because [modules/loader_arm_v2.scad](../../mechanical_design/openscad/modules/loader_arm_v2.scad#L2) does `use <../parts/arm_plate.scad>`, this top-level call is suppressed in the assembly — fine.

The risk is that other parts files do `include <../parts/arm_plate.scad>` (search the structural parts and exports), which would duplicate the part at world origin in the assembly. I did not find such an `include`, but the pattern is fragile. Standard OpenSCAD convention is to put the demo render behind `if ($preview) arm_plate();` so any accidental include does not dump ghost geometry.

**Fix:** wrap top-level demo renders in `if ($preview)` or move them into the dedicated `parts/export_*.scad` wrappers (which already exist for several parts).

### 10. `cnclayout_simple_outlines_backup.scad` — old backup file in the source tree
[cnclayout_simple_outlines_backup.scad](../../mechanical_design/openscad/cnclayout_simple_outlines_backup.scad) is a 6 KB historical snapshot. Backups belong in version control history, not in the repository. Same goes for `test_check.csg` / `test_output.csg` (CSG dumps that look like one-off debug artifacts).

**Fix:** delete the `_backup.scad`, `test_check.csg`, `test_output.csg` files and let git carry the history. If a snapshot is truly needed, tag a commit instead.

### 11. Magic numbers throughout, not parameterized
Examples:

- `_crossmember_top_z = FRAME_Z_OFFSET + MACHINE_HEIGHT * 0.7;` ([lifetrac_v25_params.scad](../../mechanical_design/openscad/lifetrac_v25_params.scad#L1163)) — `0.7` is undocumented.
- Loader arm bolt patterns: `bolt_spacing_x = 100; bolt_spacing_y = 80;` ([modules/loader_arm_v2.scad](../../mechanical_design/openscad/modules/loader_arm_v2.scad#L13-L14)) — these are hard-coded inside the module rather than named in the params file.
- `cross_beam_w = 152.4` and `cross_beam_h = 50.8` ([modules/loader_arm_v2.scad](../../mechanical_design/openscad/modules/loader_arm_v2.scad#L21-L22)) — duplicates `TUBE_2X6_1_4` data.
- `extension_len = 350; spacer_len = 150;` ([modules/loader_arm_v2.scad](../../mechanical_design/openscad/modules/loader_arm_v2.scad#L25, L113)).
- `_FRONT_WHEEL_AXIS_Y - 152.4 - UTU_FRAME_TUBE_WIDTH` in [lifetrac_v25_UTU.scad](../../mechanical_design/openscad/lifetrac_v25_UTU.scad) — `152.4` is `FRAME_TUBE_OFFSET_FROM_WHEEL` already named in the same file, but the literal is repeated.
- `BUCKET_CYL_MOUNT_Z_OFFSET = -117.2` ([lifetrac_v25_params.scad](../../mechanical_design/openscad/lifetrac_v25_params.scad#L867)) — comment says "Calculated for 45 deg dump angle" but the value is hand-tuned, not symbolically derived.

**Fix:** promote each to a named constant in the params file; reference the existing `TUBE_2X6_1_4`, `FRAME_TUBE_OFFSET_FROM_WHEEL`, etc. instead of repeating the millimeter literal.

### 12. `PIVOT_MOUNT_BOLT_ANGLES` exceeds 360°
The computed array is `[295.45, 372.7, 449.9, 527.2, 604.4]` — angles past one full revolution. `cos`/`sin` handle these correctly, so geometry is right, but it is confusing on inspection and makes any `assert(angle < 360)` style check impossible.

**Fix:** apply `% 360` when building the array. The values become `[295.5, 12.7, 89.9, 167.2, 244.4]` which match the helpful comment in the file.

### 13. `_utu_solve_rear_y` recursion depends on near-1:1 loop-length sensitivity
[lifetrac_v25_UTU.scad](../../mechanical_design/openscad/lifetrac_v25_UTU.scad#L335-L342):

```scad
function _utu_solve_rear_y(rear_y, iter) =
    iter <= 0 ? rear_y
    : _utu_solve_rear_y(
          rear_y - (_utu_target_loop - _utu_loop_length_for_rear_y(rear_y)),
          iter - 1);
```

This is a fixed-point iteration that assumes `d(loop_length)/d(rear_y) ≈ -1`. The comment says it "iterates a few times to converge precisely." That is true for the current geometry, but if the drive axis position changes such that the diagonal sensitivities differ from 1:1, the iteration can oscillate or converge slowly with no diagnostic.

**Fix:** add a `_utu_loop_length_for_rear_y(result) ≈ _utu_target_loop` assertion (within a small epsilon), or use a bisection that is provably convergent.

### 14. `solve_E_func`'s clamp masks out-of-range geometry
At [lifetrac_v25_params.scad](../../mechanical_design/openscad/lifetrac_v25_params.scad#L267):

```scad
cos_alpha_c = max(-1, min(1, cos_alpha)),
angle_EPT = acos(cos_alpha_c),
```

Clamping `cos_alpha` is necessary for floating-point safety, but a `cos_alpha` outside `[-1.05, 1.05]` indicates the geometry is genuinely impossible (Law of Cosines violated) — silently clamping and returning a position is dangerous. Add an `assert(cos_alpha >= -1.1 && cos_alpha <= 1.1, ...)` guard.

### 15. `validateAndClampJoystickValue` style validation missing on `assert` messages
File-wide: `assert(condition, "message")` calls don't include the actual values. For example:

```scad
assert(PLATFORM_ARM_LENGTH >= PLATFORM_DEPTH/2,
    "PLATFORM_ARM_LENGTH must be at least half of PLATFORM_DEPTH");
```

When this fails, the user sees the message but not the values that caused failure. Use:

```scad
assert(PLATFORM_ARM_LENGTH >= PLATFORM_DEPTH/2,
    str("PLATFORM_ARM_LENGTH (", PLATFORM_ARM_LENGTH,
        ") must be >= PLATFORM_DEPTH/2 (", PLATFORM_DEPTH/2, ")"));
```

This is small but pays off the next time anyone changes `PLATFORM_DEPTH`.

---

## Medium-Severity Findings

### 16. Animation `animation_phase = animation_time < 0.5 ? (animation_time * 2) : (2 - animation_time * 2)` duplicated in both top files
Same expression appears at [lifetrac_v25.scad](../../mechanical_design/openscad/lifetrac_v25.scad#L156) and [lifetrac_v25_UTU.scad](../../mechanical_design/openscad/lifetrac_v25_UTU.scad#L84). Extract to a function in the params file, e.g. `function lifetrac_anim_phase(t) = t < 0.5 ? t*2 : 2 - t*2;`.

### 17. `controlValve()`-style duplication in arm-plate semicircle and pivot-mount geometry
The semicircular arm-plate end ([parts/arm_plate.scad](../../mechanical_design/openscad/parts/arm_plate.scad#L93-L100)) duplicates the circular pivot-plate geometry that already lives in [modules/loader_arm_v2.scad](../../mechanical_design/openscad/modules/loader_arm_v2.scad). Both files compute "6" plate diameter + 5 bolt holes at the same angles" and both fall back to the same `[295.5, 372.6, 449.8, 526.9, 604.1]` literal. A single `module pivot_mount_plate()` would eliminate the drift risk.

### 18. `hydraulic_motor()` BCD coupled to body diameter
At [modules/hydraulics.scad](../../mechanical_design/openscad/modules/hydraulics.scad#L173-L180):

```scad
body_diameter = sqrt(displacement * 4);
...
for (angle = [0:90:270]) {
    rotate([0, 0, angle])
    translate([body_diameter*0.6, 0, -1])
        cylinder(d=13, h=12, $fn=32);
}
```

The bolt circle is `body_diameter * 0.6` — a fictitious motor flange. Real hydraulic motors have standard SAE / ISO mount BCDs (SAE A is 82.55 mm, SAE B is 101.6 mm). For visualization-only purposes this is fine, but the file presents itself as a parametric library and the comment says "Mounting holes". Either drop the holes or accept a `mount_bcd` parameter.

### 19. `cylinder_base_clevis` and `cylinder_rod_clevis` use awkward unrooted constants
[modules/hydraulics.scad](../../mechanical_design/openscad/modules/hydraulics.scad#L99-L139) sprinkles `length*0.6`, `bore*1.2`, `rod_d*0.4`, `rod_d*1.6` literals throughout. Visually correct, but the magic ratios aren't documented and any tweak (e.g., to match a Prince welded clevis) requires reverse-engineering. A small `clevis_dims()` helper or a comment block explaining the proportions would help.

### 20. UTU file performs significant geometry computation at top level (not in a module)
Sprocket-phase computations fill [lifetrac_v25_UTU.scad](../../mechanical_design/openscad/lifetrac_v25_UTU.scad#L348-L417) with `_utu_*` private constants at module/file scope. They need to be at top level so the assembled chain can be placed, but the result is a long sequence of name-spaced underscored variables that read like a mini-program. Consider hoisting into a named function `function utu_compute_phases(...) = [phaseA, phaseB, phaseC];` so the file reads more like a pipeline.

### 21. `eval_balance` returns `(d_main - d_drop - ARM_BALANCE_BIAS)` without protection against `solve_L2_func` returning undef
Even though `solve_E_func` returns `[0,0]` on undef, `dist_line_func(C, P, [0,0])` returns a finite (huge) number, which the binary search treats as a normal value and steers toward. See finding 4. Fix together.

### 22. Nested `if (toward_center) { ... } else { ... }` blocks duplicate angle-iron placement
[lifetrac_v25_UTU.scad](../../mechanical_design/openscad/lifetrac_v25_UTU.scad#L774-L805): each branch repeats the same translate/mirror pattern with sign flips. Replace with a `mirror([!toward_center, 0, 0])` wrapper or a 2-arg helper.

### 23. `for (xb = [spacer_len/3, spacer_len*2/3]) for (zb = [tube_h*0.3, tube_h*0.7])` — bolt placement that "looks about right"
Comment: "Rough placement within taper". For a structural part this is below the standard the rest of the model holds itself to. Either compute the placement from the taper geometry or assert that the bolts land inside the spacer envelope (otherwise they will silently disappear when the taper changes). See [modules/loader_arm_v2.scad](../../mechanical_design/openscad/modules/loader_arm_v2.scad#L161-L164).

### 24. `inch = 25.4` and `PI = 3.14159265359` redefined at top of UTU file
[lifetrac_v25_UTU.scad](../../mechanical_design/openscad/lifetrac_v25_UTU.scad#L195-L196) redefines `inch` and `PI`. OpenSCAD already provides `PI`. The `inch` helper is fine but should live in a shared utilities file.

### 25. No unit-test or compile-check CI for the SCAD files
The repo has [ARDUINO_CI.md](../../ARDUINO_CI.md) for the firmware. There is no equivalent that verifies each top-level SCAD compiles cleanly. A simple GitHub Action that runs `openscad --hardwarnings -o /dev/null lifetrac_v25.scad`, the UTU variant, and each `parts/export_*.scad` would catch the variable-shadowing and `is_undef` regressions before merge.

---

## Low-Severity / Quality Findings

### 26. Commented-out blocks left in source
Examples:
- "Old manual adjustment" block at [lifetrac_v25_params.scad](../../mechanical_design/openscad/lifetrac_v25_params.scad#L876-L880).
- The earlier "Moved logic further down" commentary at [lifetrac_v25_params.scad](../../mechanical_design/openscad/lifetrac_v25_params.scad#L82-L86).
- `// LIFT_CYL_BASE_Z = _min_base_z - 25; // Experimental lower` at [lifetrac_v25_params.scad](../../mechanical_design/openscad/lifetrac_v25_params.scad#L688).
- The // ARM_SHAPE_ANGLE = 130; // The fixed L-angle - CALCULATED BELOW dance at [lifetrac_v25_params.scad](../../mechanical_design/openscad/lifetrac_v25_params.scad#L211).

Delete; git carries history.

### 27. `loader_arm_v2.scad`'s `cx`/`cz` reverse-engineering comments
Long inline comments like "We need to know which side is 'Bottom' in world space..." in [modules/loader_arm_v2.scad](../../mechanical_design/openscad/modules/loader_arm_v2.scad#L132-L155) and [parts/arm_plate.scad](../../mechanical_design/openscad/parts/arm_plate.scad#L62-L72) read as the author thinking out loud. They suggest the coordinate-frame conventions are not documented anywhere central. Add a short COORDS.md (or section in [DESIGN_RULES.md](../../mechanical_design/DESIGN_RULES.md)) defining: world frame, panel-local frame, arm-local frame, bucket-local frame, and the rotation conventions used to map between them.

### 28. `parent_modules()` shim
[lifetrac_v25_params.scad](../../mechanical_design/openscad/lifetrac_v25_params.scad#L7-L11) provides a stub for BOSL2's `parent_modules()`. If BOSL2 isn't actually used anywhere in the tree, drop the shim. If it is used, document which parts depend on it.

### 29. `test_cylinder_sizing.scad` is undocumented
Standalone test/sanity script with no README mention. Either fold it into a proper test harness (see finding 25) or move under a `tests/` subdirectory.

### 30. Parts that do not assert on input ranges
For example, [parts/arm_plate.scad](../../mechanical_design/openscad/parts/arm_plate.scad) accepts `is_inner_plate` and uses it to flip cutouts, but does not assert that `ARM_MAIN_LEN > 0`, `ARM_DROP_LEN > 0`, `ARM_PLATE_THICKNESS > 0`. With the upstream solver that produces these values, a degenerate case could yield negative lengths and OpenSCAD would silently render zero-thickness or self-intersecting geometry.

### 31. `export_*.scad` wrappers are inconsistent
[parts/export_bucket_bottom.scad](../../mechanical_design/openscad/parts/export_bucket_bottom.scad) (218 bytes) vs [parts/export_platform_pivot_bracket.scad](../../mechanical_design/openscad/parts/export_platform_pivot_bracket.scad) (315 bytes) — sizes vary and not all parts have wrappers. Either generate the wrappers from a single template, or document why some parts have them and others don't.

### 32. `cnclayout.scad` and `export_for_cnc.scad` overlap
Both files appear to drive CNC export. The relationship is not documented in [README_EXPORTS.md](../../mechanical_design/openscad/parts/README_EXPORTS.md). Add a one-line comment at the top of each pointing to the other and explaining when to use which.

### 33. Render colors hard-coded throughout
`color("DarkBlue")`, `color("Goldenrod")`, `color("DarkGray")`, `color("DarkSlateBlue")`, `color("Silver")`, etc. Centralize in a small color palette file so the assembly's appearance can be tuned in one place — currently changing the visual style of every "steel plate" requires editing every module.

### 34. No `make` / shell-side build system
[export_all_cnc_parts.sh](../../mechanical_design/export_all_cnc_parts.sh) and [export_individual_svgs.sh](../../mechanical_design/export_individual_svgs.sh) exist, but there is no top-level `Makefile` or `build.sh` that says "regenerate `assembly.png`, `lifetrac_v25_animation.gif`, and `cnclayout.svg` from the SCAD sources". Anyone landing in the repo for the first time has to discover the toolchain.

---

## Overall Assessment

The mechanical model is *substantively* well-designed: the parametric arm solver, the lift-cylinder geometry-first algorithm, the UTU chain pitch snapping, and the bucket-cylinder parallelism limiter are genuinely impressive pieces of OpenSCAD work. The structural-analysis section in `lifetrac_v25.scad` is unusual and valuable.

The *code-side* issues fall into three repeating patterns:

1. **Brittle cross-file plumbing** (findings 1, 2, 3, 8, 16, 17, 22). The model has outgrown the OpenSCAD module system; shadow declarations, manual re-derivation, and `is_undef` fallbacks are all symptoms of one missing abstraction layer (a `derived.scad` / `toggles.scad` split).
2. **Silent failure modes in solvers** (findings 4, 5, 13, 14, 21, 30). Each solver gracefully returns *something* on infeasible input, which is dangerous when the something is then handed to a CNC plasma cutter.
3. **Debug noise hiding real warnings** (findings 6, 7, 26, 27). 200+ lines of `echo` per render plus inconsistent `$fn` make it hard to notice the duplicate-variable warnings finding 1 produces.

None of these block the model from rendering today. Together they make the code progressively harder to change without breaking it.

## Recommended Fix Order

1. **Stop silent solver failures.** Findings 4, 5, 14, 13, 21. Fail loudly on infeasible geometry and on out-of-range hydraulic cylinders.
2. **Eliminate cross-file constant duplication.** Findings 1, 2, 3, 8 — extract a `lifetrac_v25_derived.scad` and `lifetrac_v25_toggles.scad`, and remove `is_undef(...) ? default : ...` fallbacks.
3. **Quiet the build.** Findings 6, 7, 26 — gate echoes behind a `DEBUG` flag, normalize `$fn`, and remove dead/commented code.
4. **Add a SCAD compile-check CI job.** Finding 25 — an `openscad --hardwarnings` smoke test will catch many of the issues in 1–3 automatically.
5. **Hygiene.** Findings 9–12, 15, 18–20, 22–24, 27–34.
