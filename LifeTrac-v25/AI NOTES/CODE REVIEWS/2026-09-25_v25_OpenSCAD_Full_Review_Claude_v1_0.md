# LifeTrac v25 OpenSCAD: Full Code Review

**Reviewer:** Claude (Claude Code)
**Date:** 2026-09-25
**Scope:** every `.scad` file under [DESIGN-STRUCTURAL/](../../DESIGN-STRUCTURAL/) (57 files, about 13,700 lines), plus the CI workflows and helper scripts that drive them
**Previous SCAD review:** [2026-04-25_SCAD_Review_ClaudeOpus4_7.md](2026-04-25_SCAD_Review_ClaudeOpus4_7.md). Its links point at the old `mechanical_design/` path; the tree now lives at `DESIGN-STRUCTURAL/openscad/`.

### Status: fixed alongside this review

The body of this review describes the tree as it was on 2026-09-25. A closer look at the structural analysis on 2026-09-26 added the method errors in B4, the load check in P3, the new finding P14 and Appendix C; those passages are marked *(added 2026-09-26)*. The pull request that adds this document also fixes the following mechanical defects. Each fix was verified locally with OpenSCAD 2021.01; the details are in the commit messages.

| Finding | Fix |
|---|---|
| B1a: clearance check reported double the distance | `dist_point_line_verify()` now returns `twice_area / base`. The solver prints −125.4 / −160.4 mm, and the difference is exactly the 35 mm `ARM_BALANCE_BIAS`. No other echo line or geometry changed |
| P1: four export wrappers produced edge slices | The `rotate()` is removed from `export_side_panel_outer/inner`, `export_rear_crossmember` and `export_bucket_side`. They now export 1,392.6 × 1,000, 1,045.4 × 550 and 449.2 × 600 mm outlines |
| P3: pivot hole 4 mm from the edge | `parts/side_panel.scad` (used by the assembly, the exports and `cnclayout.svg`) now adds a boss of radius `SIDE_PANEL_PIVOT_BOSS_R = 2 × PIVOT_PIN_DIA` (76.2 mm) around the pivot hole. Measured on the exported outline, there is now 46–56 mm of steel ahead of the hole in the directions the pin pushes (it was 5.6 mm). The nearest edge is now the top edge, 30 mm away. `assert`s guard both distances. The profile's inside corners now have 10 mm fillets. In the regenerated `cnclayout.svg`, only the four side-panel outlines changed |
| P4: round holes exported as rectangles | The hole cylinders in `bucket_side.scad` and `cylinder_lug.scad` now go through the plate thickness. `cnclayout.svg` is regenerated; only its six lug pivot holes changed |
| Jig includes (section 5.4) | `angle_iron_drill_jig.scad` and `tube_drill_jig.scad` include `../lifetrac_v25_params.scad` and compile with 0 warnings. Both are still placeholders |
| `generate-part-svgs.yml` (section 5.5) | Watches and exports `openscad/parts/`, fails if an SVG is missing, and uploads the SVGs as an artifact (`output/` is git-ignored). Its steps that wrote README links to never-committed files, and tried to commit them, are removed; the README now lists the SVGs by file name and says where to get them. The same path error is fixed in `README_EXPORTS.md` and `INDIVIDUAL_PARTS_GUIDE.md`, whose commands now also create the output folder first. The guide now describes what the workflow actually does |
| `openscad-structural-analysis.yml` (section 5.5) | Writes the ECHO output to `structural_analysis.log` instead of `/dev/null`, fails if the summary is missing, and extracts the whole summary table |
| B4: stale capacity now published | The analysis log and its PR comment carry a warning that the checks model an earlier design and are not ratings. The log's "Current Rated Capacity" line is relabelled as the hydraulic lift capacity used as the design load |
| Section 8, step 0 | `DESIGN-STRUCTURAL/README.md` now carries a "not yet ready for fabrication" warning that points here |

**Not changed on purpose:**
- **`pivot_welding_jig.scad`'s include.** Any change under `3d_printed_welding_jigs/` triggers `generate-jig-previews.yml`, whose push-event commit step has failed on both of its runs on main. The cause is that it `git add`s `renders/*.png` and `*.jpg`, which the root `.gitignore` ignores. Whether jig renders should be committed is a maintainer decision.
- **The camera maths in the PNG/GIF workflows.** Fixing it would visibly change `assembly.png` and the animation.
- **The bucket pivot joint (P14).** It can't be fixed at the arm tip alone: the bucket's lug sits just beyond the tip and is no stronger. The fix changes the bucket lugs and depends on which bucket cylinder is chosen.
- **Everything else that needs a design decision.** That covers wheel choice, DOM length, hole patterns, T5 and deletions. The analysis was rebuilt in a follow-up pull request (below).

### Status: fixed in follow-up pull requests

| Finding | Fix |
|---|---|
| B4, B5: structural and stability analysis | [#136](https://github.com/OpenSourceEcology/LifeTrac/pull/136) replaces both with `openscad/analysis/structural_analysis.scad`, which checks the current parts. `DESIGN-STRUCTURAL/STRUCTURAL_ANALYSIS.md` describes the method. It also removes the stale-analysis warning described above. With the estimated masses, the rated operating capacity is 92 kg, because the tipping load is only 184 kg at full reach. Seven checks still fail, and each is listed as a known issue with its finding (B4, P14, M8) |

---

## 0. How this review was done

This review didn't rely on reading alone. Every file was run through OpenSCAD 2021.01, the same version CI installs with `apt-get install openscad`:

- **Evaluated every file** with `openscad -o file.echo` and collected all `WARNING`/`ERROR` lines and the full `echo()` output (the solver results).
- **Rendered geometry where a claim depended on it:** the UWU wheel units (to measure overall width), the inner side panel against the T3 cross beam at four arm angles (a suspected collision; it turned out not to be one), and several single parts.
- **Tested the OpenSCAD scoping rules** that the UTU variant depends on, with small test files (see B11).
- **Checked the dead code mechanically** by scanning every module, function and top-level variable for references across the whole tree.
- **Re-checked all 34 findings from the April review** against the current code (section 2).
- **Reviewed `parts/`, the jigs, the CNC layout and the CI workflows in two parallel detailed passes** (section 5). Their headline claims were re-verified independently before inclusion:
  - P1: exports re-rendered and measured
  - P3: ligament computed
  - P10: scoping rule tested
  - P11: hole lists compared
  - P12: profiles compared
  - the structural-analysis `/dev/null` capture: re-run locally
  - the camera values: read from the 2026-09-14 CI job log

### 0.1 Compile/evaluate results

| File group | Result |
|---|---|
| `lifetrac_v25.scad` | Evaluates with 0 OpenSCAD warnings, but prints **455 echo lines**, including its own `WARNING: Wheels will NOT touch ground`, `SAFE? NO`, and 4 structural `FAIL`s |
| `lifetrac_v25_UTU.scad` | Evaluates with 0 warnings and 228 echo lines |
| `lifetrac_v25_params.scad` | Evaluates with 0 warnings; its 128 echo lines are printed again by every part file that includes it |
| `modules/*.scad` (6 files) | Evaluate with 0 warnings |
| `parts/**/*.scad` (38 files) | Evaluate with 0 warnings (see section 5 for content issues) |
| `3d_printed_bolt_hole_cutting_jigs/angle_iron_drill_jig.scad` | **18 warnings**: broken include path, so the jig renders with undefined sizes |
| `3d_printed_bolt_hole_cutting_jigs/tube_drill_jig.scad` | **26 warnings**: broken include path and an unknown variable |
| `3d_printed_welding_jigs/pivot_welding_jig.scad` | **9 warnings**: broken include path |
| `test_cylinder_sizing.scad` | **49 warnings**: uses variables that no longer exist |
| `test_params_v3.scad` | 1 warning |

CI either never compiles these files or swallows their warnings (`generate-jig-previews.yml` renders `pivot_welding_jig.scad` to an almost blank image and passes), so they have rotted without anyone noticing.

---

## 1. Executive summary

The model is ambitious and, in places, excellent: the parametric arm solver, the bucket-cylinder parallelism limiter, the UTU chain-pitch snapping (which closes to 0.00 mm pitch error) and the new collision-check tooling (issue #119) are all strong work. But the code has three structural problems that now produce wrong numbers, not just messy code:

1. **The analysis layer has drifted away from the geometry.** The solvers and the structural/stability checks still use the parameters of an earlier machine: a straight 3"×3" arm, a 500 mm wheel on a 1,400 mm wheelbase, 3/4" pivot rings, and a 2"×2" cross beam. The rendered machine has an L-shaped 2"×6" arm, a 617 mm tyre on a 750 mm wheelbase, a bolted pivot mount, and a 2"×6" cross beam. So the printed "RATED LIFT CAPACITY 3,305 kg" and the four structural `FAIL`s describe a machine that isn't the one being drawn (B3, B4, B5). The analysis also has method errors of its own: it applies the safety factor twice, treats the pinned arm as a cantilever, and leaves the cylinder force out of the pivot-pin load (B4).
2. **The same quantity is defined in several places, and some copies now disagree.** Examples: `CROSS_BEAM_2_POS` is 1,457 mm in the assembly but 1,674 mm in the CNC side-panel part (B2); the lift cylinder's closed length is 803.5 mm in params but 831.5 mm in the cylinder module (B6); pivot DOM length is 120 mm vs 50.8 mm (B8); four different clevis-pin diameters are used for the same joints (B7); the UTU angle irons don't match the holes drilled for them (B9).
3. **Self-checks print failures but nothing fails.** The model's own output says `SAFE? NO` (bucket-cylinder parallelism, B1b), `WARNING: Wheels will NOT touch ground`, and `REVIEW REQUIRED ✗`, and CI stays green. One verification function (`dist_point_line_verify`) reports double the true distance, so the "arm clears the wheel by +75 mm / +5 mm" output is really −125 mm / −160 mm in its own 2D model (B1a).

**The fabrication outputs aren't usable yet**:

- **The four 1/2" side panels export as 1,373 × 13 mm strips** (P1; the export is fixed in this PR). Even a fixed export would be missing every hole the assembly adds (P2).
- **The arm-pivot hole in those panels has about 4 mm of edge distance** (P3; fixed in this PR). At relief pressure the pin load on that edge was about equal to its tear-out strength.
- **The bucket pivot joint is weaker than the bucket cylinders that load it** (P14). At relief pressure the 3" bucket cylinder can tear the 1" pin out of the arm tip, and the bucket's own lug and its four 1/4" bolts are no stronger.
- **The CNC layout has 7 overlapping part pairs and leaves out all the stiffener, motor and pivot-mount plates** (N1, N2).
- **The angle-iron cut-list parts drill holes that don't line up with the plates they bolt to** (P10, P11).
- **The arm-leg spacer T5 is an empty solid** (P12).
- **Three of five jigs fail to load params** (section 5.4; two are fixed in this PR).

**Several CI jobs report green while doing nothing.** The structural-analysis job pipes its output to `/dev/null`. The part-SVG job points at a folder that doesn't exist. (Both are fixed in this PR.) The assembly PNG is rendered with a mis-computed camera (section 5.5).

**Of the 34 findings in the April review, none is fully fixed, 2 are partly fixed (#10, #25) and 32 are still open** (section 2). Several of the open ones have grown. The most useful next step is the same as it was then: make `lifetrac_v25_params.scad` (plus a new `derived.scad`) the only source of truth, and make CI fail on warnings, asserts and self-check failures. Also make **the assembly and every export call the same part modules** (C1.6).

---

## 2. Status of the 2026-04-25 findings

| # | April finding | Status | Evidence (current code) |
|---|---|---|---|
| 1 | Constants re-declared in main after `include` of params | **Open, and now diverging** | [lifetrac_v25.scad:64-120](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L64-L120), [:173-177](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L173-L177), [:213-214](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L213-L214), [:231-239](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L231-L239), [:1110-1120](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L1110-L1120). `CROSS_BEAM_2_POS` now differs (B2) |
| 2 | UTU re-derives ~60 constants by hand | Open (and the reason given for it is wrong, see B11) | [lifetrac_v25_UTU.scad:40-161](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_UTU.scad#L40-L161) |
| 3 | `is_undef(x) ? literal : x` fallbacks | **Open, and grown to 125** across 19 files | Worst: params 12, `arm_plate.scad` 11, `a7` 11, `a6` 10, `a8` 10, `side_panel.scad` 10, main 10 |
| 4 | `solve_L2_func` returns `undef`, and the search runs on silently | Open | [lifetrac_v25_params.scad:262-318](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L262-L318) |
| 5 | `hydraulic_cylinder()` silently clamps extension | Open. It is clamping *today* (B6) | [modules/hydraulics.scad:55](../../DESIGN-STRUCTURAL/openscad/modules/hydraulics.scad#L55) |
| 6 | Echo storm | **Open, and worse**: 611 `echo()` call sites; 455 lines per main render | Params prints 128 lines on *every* part render; `hydraulic_cylinder` and `arm_plate` echo once per instance |
| 7 | `$fn` set in library files | Open. Verified: a top-level `$fn` in a `use`d file **does** apply to that library's own modules, so `hydraulics.scad` (32), `wheels.scad` (64) and `fasteners.scad` (**6**) override whatever the top-level file sets | [fasteners.scad:6](../../DESIGN-STRUCTURAL/openscad/modules/fasteners.scad#L6) |
| 8 | UTU re-declares display toggles | Open. Most of them have no effect on the modules they are meant to control (B11) | [lifetrac_v25_UTU.scad:47-52](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_UTU.scad#L47-L52), [:169-177](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_UTU.scad#L169-L177) |
| 9 | `arm_plate();` called at top level | Open | [parts/arm_plate.scad:23](../../DESIGN-STRUCTURAL/openscad/parts/arm_plate.scad#L23) |
| 10 | Backup/debug files in tree | Partly fixed: the `.csg` dumps are gone; `cnclayout_simple_outlines_backup.scad`, `test_params_v3.scad`, `temp.ipynb` and `cylinder_output.txt` remain | |
| 11 | Magic numbers | Open. Examples: `MACHINE_HEIGHT * 0.65` (panel slope), `* 0.7` (crossmember), `ARM_LENGTH * 0.50`, `ARM_TIP_X * 0.65 / 0.95`, `-117.2`, `152.4` repeated in UTU | |
| 12 | `PIVOT_MOUNT_BOLT_ANGLES` > 360° | Open. Now `[243.7, 301.8, 360, 418.2, 476.3]`. The slot moved to 180° but comments still describe a 270° slot and "~50.9°" gap (actual 127.3°) | [params:147-186](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L147-L186) |
| 13 | UTU fixed-point iteration unchecked | Open (it converges today: pitch error 0.00 mm) | [UTU:358-363](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_UTU.scad#L358-L363) |
| 14 | `solve_E_func` clamps `cos` silently | Open | [params:279](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L279) |
| 15 | `assert` messages lack values | Open | [params:1142-1169](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L1142-L1169) |
| 16 | Animation-phase expression duplicated | Open | main:151, UTU:65 |
| 17 | Pivot-plate geometry duplicated | Open, **3 copies** | `loader_arm_v2.scad`, `arm_plate.scad`, `pivot_mount_assembly.scad` (and the copies disagree, B8) |
| 18 | `hydraulic_motor()` fictitious bolt circle | Open (only used by the non-UWU wheel path) | |
| 19 | Clevis proportions are unexplained ratios | Open, and now causes B6/B7 | |
| 20 | UTU top-level computation reads like a program | Open | |
| 21 | `eval_balance` with `undef` | Open | |
| 22 | `toward_center` if/else duplication | Open | [UTU:805-829](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_UTU.scad#L805-L829) |
| 23 | Spacer bolts "rough placement" | Open | [loader_arm_v2.scad:165-167](../../DESIGN-STRUCTURAL/openscad/modules/loader_arm_v2.scad#L165-L167) |
| 24 | `inch`/`PI` redefined in UTU | Open (`PI` shadows the builtin; `eps` also added) | [UTU:183-185](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_UTU.scad#L183-L185) |
| 25 | No compile-check CI | **Partly fixed.** `openscad-structural-analysis.yml` evaluates main plus modules, and `openscad-collision-check.yml` was added. But nothing uses `--hardwarnings`, no job fails on the model's own `FAIL`/`NO`/`WARNING` echoes, and jigs, parts and UTU aren't compiled | |
| 26 | Commented-out blocks | Open, and much larger (see C4) | |
| 27 | No coordinate-frame doc | Open. The code still re-derives frames inline in long comments | |
| 28 | `parent_modules()` shim | Open (BOSL2 isn't used anywhere) | [params:11](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L11) |
| 29 | `test_cylinder_sizing.scad` undocumented | Open, and now **broken** (49 warnings; root cause in section 5.5) | |
| 30 | Parts don't assert input ranges | Open | |
| 31 | `export_*.scad` wrappers inconsistent | Open, and **4 of 10 produce the wrong geometry** (P1) | |
| 32 | `cnclayout.scad` vs `export_for_cnc.scad` overlap | Open. `export_for_cnc.scad` is stale and wrong (N4) | |
| 33 | Colours hard-coded | Open: 139 `color("…")` calls | |
| 34 | No Makefile/build entry point | Open | |

The biggest improvement since April is the collision-check workflow (issue #119). It checks real interference across the pose envelope, which goes further than #25 asked for, even though the compile-check part of #25 is still missing.

---

## 3. Correctness bugs and inconsistencies (new in this review)

Severity key: **Critical** means a wrong number that a fabricator or a buyer would act on. **High** means the model disagrees with itself in a way that will produce a wrong part. **Medium** means wrong but currently harmless, or confined to dead or analysis code.

### B1. The solver's self-checks report wrong or failing results, and nothing fails — **Critical**

**B1a. The clearance verification reports double the true distance.** *(Fixed in this PR; see Status.)* `dist_point_line_verify()` computes the shoelace term, which is already twice the triangle area, then multiplies by 2 again. The true distance is `area/base`:

```scad
// lifetrac_v25_params.scad:369-373
function dist_point_line_verify(pt, v1, v2) =
    let( area = abs(v1[0]*(v2[1]-pt[1]) + v2[0]*(pt[1]-v1[1]) + pt[0]*(v1[1]-v2[1])),
         base = norm(v1 - v2) )
    (base == 0) ? norm(pt - v1) : (area / base * 2);   // should be: area / base
```

I reproduced the solver numbers in Python (P = [200, 1100], E = [1519.5, 568.3], T = [1675.4, 90], C = [1400, 400]):

| Quantity | Echoed by model | True value |
|---|---|---|
| Main-arm clearance to wheel | +75.35 mm | **−125.4 mm** |
| Drop-arm clearance to wheel | +5.35 mm | **−160.4 mm** |
| "Difference" | 70.0 | 35.0 (exactly `ARM_BALANCE_BIAS`, which confirms the ×2) |

The line just above it already prints `Clearance at Elbow: -145.182`, which contradicts the "+75 mm". Also, the optimiser (`find_optimal_L1`) only *balances* the two clearances; nothing enforces the "strictly enforce 1 inch clearance" goal in the comment at [params:231-233](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L231-L233).

Mitigating factor: the arms run in the sandwich gap (X ≈ ±450 mm) and the tyres sit outboard (X ≈ 580–834 mm), so this 2D overlap isn't a physical collision today. But see B3: the "wheel" this solver avoids isn't the wheel that is rendered anyway.

**Fix:** return `area / base`. Add `assert(_clr_main_geometric >= WHEEL_CLEARANCE_TARGET && _clr_drop_geometric >= WHEEL_CLEARANCE_TARGET, str(...))`, or delete the check and let the 3D collision tool own clearance.

**B1b. The bucket-cylinder parallelism check fails, and its limit is computed for a different cross-beam position.** The params file prints:

```
Angle from parallel at limit: 5.20451 degrees
SAFE? NO                    (limit is 10°)
```

The cause: `ARM_MAX_ANGLE_LIMITED` is found by bisection with a *reference* cross-beam position `_cb_ref_pos = ARM_TIP_X * 0.65` (1,145 mm, [params:862](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L862)). Step 7 then places the real cross beam at `CROSS_BEAM_1_POS` = 736 mm ([params:993](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L993)). The limit and the beam position depend on each other, and the code does one pass of each.

**Fix:** iterate the two until they agree (a fixed-point loop of 3–5 passes, or bisect on the arm angle using the beam position computed for that angle), then `assert(_final_parallelism >= _parallelism_limit)`.

**B1c. Nothing fails.** `SAFE? NO`, `WARNING: Wheels will NOT touch ground…`, `✗ TOO LONG/SHORT`, `REVIEW REQUIRED ✗` and the collision tool's `COLLISION_*` lines are all plain `echo()`. The structural-analysis workflow greps for `FAIL` only to write a markdown log, and it commits that log back to the branch while the check stays green.

**Fix:** convert every design-rule check into `assert()`, with a `STRICT = true` flag if you want to keep a permissive exploration mode. Then run CI with `--hardwarnings` so a failed check turns the build red.

### B2. `CROSS_BEAM_2_POS` has two different values: the assembly and the CNC part disagree — **High**

- [params:1029](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L1029): `CROSS_BEAM_2_POS = ARM_TIP_X * 0.95;` gives **1,673.9 mm**
- [lifetrac_v25.scad:173](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L173): `CROSS_BEAM_2_POS = ARM_LENGTH - 200;` gives **1,457 mm**

In OpenSCAD the last assignment in a scope wins, so the assembly uses 1,457. Files that only include params (`parts/side_panel.scad`, which feeds the CNC export) use 1,673.9. Verified by evaluating both files. The consequences:

- The guard `if (CROSS_BEAM_2_POS < WHEEL_BASE + 200)` in [side_panel.scad:201](../../DESIGN-STRUCTURAL/openscad/parts/side_panel.scad#L201) is **true in the assembly** (1,457 < 1,600) and **false in the CNC part** (1,674 > 1,600). The rendered inner panels get a second arc slot that the plasma-cut panels won't have.
- **There is no second cross beam.** `CROSS_BEAM_2_POS` is used only for this slot, and `loader_arms()` draws one beam (T3). The slot is for a part that doesn't exist.

**Fix:** delete `CROSS_BEAM_2_POS` and the second-slot code, or model the second beam. Either way, remove every re-declaration in the main file (April #1).

### B3. There are three incompatible wheel geometries, and the arm solver designs around one that isn't drawn — **Critical**

| Where | Wheel diameter | Front-axle Y | Wheelbase | Wheel-centre Z |
|---|---|---|---|---|
| Arm solver, `_C` in [params:226](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L226) | `WHEEL_DIAMETER` = **500** | `WHEEL_BASE` = **1,400** | **1,400** (implied) | `FRAME_Z_OFFSET + WHEEL_RADIUS` = **400** |
| Rendered UWU wheel ([main:3599](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L3599), [:3713](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L3713), [:3944-3960](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L3944-L3960)) | tyre OD **forced to `2 × UWU_SHAFT_Z` = 617.5** | `WHEEL_BASE − WHEEL_RADIUS` = **1,150** | **750** | **308.75** |
| Declared real wheel ([params:1206-1226](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L1206-L1226)) | Bobcat 12-16.5, `BOBCAT_TIRE_OD` = **851** | n/a | n/a | would sink **116 mm below ground** at Z = 308.75 |

The rendered tyre is a 617.5 mm "tyre" on a 16.5" rim, which isn't a real product. Its OD is simply whatever value makes it touch the ground. A real 12-16.5 is about 851 mm. Using it means either raising the frame by about 116 mm (`GROUND_CLEARANCE` 150 → about 266) or choosing a different wheel. Either choice changes the arm solver target, the bucket height, stability and lift-cylinder geometry.

Knock-on effects:

- **Measured overall width is 1,667 mm** (rendered `uwu_assemblies()` to STL: X from −833.5 to +833.5). `MACHINE_WIDTH` is 1,200. `WHEEL_X_OFFSET`, `WHEEL_WIDTH` = 200 and `WHEEL_CLEARANCE` = 50 describe the legacy wheel. The UWU shaft length uses `WHEEL_WIDTH/2` of that legacy wheel ([main:3770](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L3770)), then a hand-tuned `+25.4 + 50.8 − 127.0` shift ([main:3807](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L3807)). The resulting tyre-to-wall gap is 56.8 mm, not the 3" the comment claims.
- **The rear-axle position is derived from the lift-cylinder base.** `_REAR_WHEEL_AXIS_Y = min(LIFT_CYL_BASE_Y + WHEEL_RADIUS + 100, …)` ([main:1854-1857](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L1854-L1857)). Moving a hydraulic pin moves the axle.
- **This wheel-position logic is copied five times:** [main:1851-1857](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L1851-L1857), [:3320-3323](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L3320-L3323), [:3426-3430](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L3426-L3430), [:3540-3556](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L3540-L3556), [:3692-3698](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L3692-L3698), plus [UTU:104-108](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_UTU.scad#L104-L108).
- **`WHEEL_BASE` really means "frame length".** It sets the side-panel length, the bottom plate, the bucket target, and the solver's "front wheel", while the real front axle sits 250 mm behind it.
- **The Bobcat centre bore is defined three ways:** `BOBCAT_CENTER_BORE = 76.2` ([params:1220](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L1220), unused), `_bc_center_bore = 152.4` ([wheels.scad:16](../../DESIGN-STRUCTURAL/openscad/modules/wheels.scad#L16)), and the literal `152.4` in [main:3933](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L3933).

**Fix:** decide on the production wheel. Put `FRONT_AXLE_Y`, `REAR_AXLE_Y`, `WHEEL_OD`, `WHEEL_CENTER_Z` and `TIRE_WIDTH` in params as the *only* wheel definition, derived from the UWU shaft height and the chosen tyre. Then point the arm solver, stability, UWU placement, UTU and side-panel length at those values. Add `assert(abs(WHEEL_CENTER_Z − WHEEL_OD/2) < 1, …)`.

### B4. The structural analysis checks components that no longer exist, and misses the ones that do — **Critical**

*(Rebuilt in a follow-up pull request; see Status.)*

The summary block ([main:784-1104](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L784-L1104)) reports 4 FAILs with ratios of 11.3, 18.4, 10.7 and 14.2. Almost every input describes the v1/v2-era machine:

| Check | What the code analyses | What the model actually has |
|---|---|---|
| Arm bending/deflection | `ARM_TUBE_SIZE = TUBE_3X3_1_4` (3"×3", S ≈ 38,200 mm³) as a straight cantilever of `ARM_LENGTH` = 1,657 mm | L-arm made of **2"×6"×1/4" tube** (S ≈ 83,000 mm³ about the strong axis, before counting the two 1/4" side plates) with the load at `ARM_TIP_X/Z` |
| Pivot "ring welds" | "Two 3/4" plate rings welded to each side of arm tube" | A **bolted pivot mount**: two 6" × 1/4" plates welded to a DOM tube, held by **5 × 1/2" bolts per plate**. Neither the bolt group nor the DOM welds are checked |
| Cross beam | `TUBE_2X2_1_4` (S ≈ 14,900 mm³) | T3 is **2"×6"×1/4"** ([tube_t3_arm_crossbeam.scad:2](../../DESIGN-STRUCTURAL/openscad/parts/structural/tube_t3_arm_crossbeam.scad#L2)) |
| Load moment arm | `ARM_LENGTH * cos(angle)` (straight arm, [main:730](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L730)) | Tip is at `ARM_TIP_X/Z` in the arm frame, and the bucket load centre is further forward still |
| Capacity at max | `lift_capacity_kg(ARM_MAX_ANGLE)` = 73.9° ([main:755](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L755)) | The operating limit is `ARM_MAX_ANGLE_LIMITED` = 49.4° |

The "RATED LIFT CAPACITY 3,305 kg (7,289 lb)" is the **hydraulic** lift force at the bucket. The same render's stability block prints a **tipping load of 661 kg and a rated operating capacity of 330 kg**. The README workflow publishes a separate hard-coded "~1,200 kg" estimate. So three different "capacities" circulate.

As a rough, static, order-of-magnitude check (not a substitute for a proper analysis): swapping in the real 2×6 section and designing to the tipping-limited load brings the arm bending ratio from 11.3 to about 0.5. So the FAILs are mostly an artefact of the stale inputs. **But** a loader also sees hydraulic-limited loads when the bucket is caught or prying (breakout), so both load cases need checking.

**Method errors** *(added 2026-09-26)*. Even with the right parts, the block would give wrong answers:

1. **The safety factor is applied twice.** The load is doubled (`DESIGN_LOAD_N = … × SAFETY_FACTOR_STATIC`, [main:808](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L808)) and the allowable stresses are halved ([main:841](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L841), [:887](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L887), [:911](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L911)), so the arm, pin and bearing checks run at a factor of 4. The weld allowable of 145 MPa is already the AISC ASD value (0.3 × F_EXX, which includes Ω = 2), and it is halved again ([main:1036](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L1036)), so the pivot-weld check runs at 8.
2. **The arm is treated as a cantilever built into the pivot** (`M = P·L`, [main:835](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L835); `δ = PL³/3EI`, [main:860](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L860)). The arm hangs on a pin, which carries no moment, and the lift cylinder props it 828.5 mm out. The peak moment is at the lift bracket and is 40–55 % of `P·L` over the working range. The cantilever deflection is about 3.5× too large, and the deflection check also uses the doubled load.
3. **The pivot-pin load is the tip load** ([main:881](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L881)). The pin also takes the lift-cylinder force. At relief pressure it carries 59–69 kN per arm, 3–7 times the 9–24 kN tip load.
4. **The "pivot ring weld" check passes the cantilever moment through the pivot** ([main:1033](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L1033)). A pin joint has no moment to pass, and the rings don't exist.
5. **The cross beam is loaded by one bucket cylinder at mid-span** (`PL/4`, [main:987](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L987)). There are two cylinders, each 135 mm in from an arm, so the moment is `P × 135 mm`. Their lugs hang 63.5 mm below the beam, which also twists it; that isn't checked. The "cross beam welds" check covers welds that don't exist: T3 is bolted to the arms through A5 angle clips.
6. **The "rated" capacity is the lower of two hydraulic values** (arms down and arms level, [main:778](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L778)), with the load on a straight 1,657 mm arm. At the L-arm's actual bucket pin, the hydraulic capacity falls from about 4,800 kg with the arms down to about 1,800 kg at the 49.4° working limit. That is before the arm and bucket weights and the load's position inside the bucket are counted. The machine tips long before either figure (the stability check gives 330 kg).

Appendix C gives the hand check behind these numbers. At the 3,000 psi relief pressure, with the real sections, it gives:

- **Arm at the lift bracket:** about 100 MPa if the 2×6 tube and the two 1/4" side plates share the moment. It rises to about 280 MPa, above the 250 MPa yield, if the plates carry it alone. The tube is bolted to the plates only near each end ([loader_arm_v2.scad:65-74](../../DESIGN-STRUCTURAL/openscad/modules/loader_arm_v2.scad#L65-L74)), so either that needs checking or the plates should be welded to the tube.
- **T3:** about 150 MPa in bending plus about 70 MPa of torsional shear near the lugs.
- **Comfortable:** the pivot pin (about 30 MPa in shear), the 1" clevis pins and the pivot-mount bolts.
- **Not checked, and weak:** the side-panel pivot hole (P3) and the bucket pivot joint (P14).

**Fix:** rebuild the analysis block against the real parts, preferably in its own `analysis/` file that reads `derived.scad`:

- (a) ROC load case: `min(hydraulic, 0.5 × tipping)` times a dynamic factor of about 2 to 2.5.
- (b) Relief-pressure breakout case: full cylinder force, checked against yield with a lower safety factor.
- Add checks for the 5-bolt pivot-mount bolt group, the DOM-to-plate welds, the T3 cross beam as 2×6, the 1/4" arm side plates, the 1/2" side panels at the pivot and cylinder pins, and the platform pivot.
- *(Added 2026-09-26.)* Model the arm as pinned at the pivot and propped by the lift cylinder, and take pin loads from the cylinder force as well as the tip load. Apply one safety factor. Check T3 in torsion, the tear-out of every pin hole, and the bucket pivot joint with every cylinder lug and its bolts (P14).
- Report *one* capacity figure, and have the README workflow read it instead of hard-coding 1,200 kg.

### B5. The stability and lift-cylinder maths use legacy geometry — **High**

- `calculate_cog()` ([main:3402-3527](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L3402-L3527)) places the bucket at `ARM_PIVOT + ARM_LENGTH·(cos, sin)` (straight arm) and the frame centroid at `MACHINE_LENGTH/2` = 900. The frame actually spans 0–1,400, with the bucket hanging forward of it.
- The operator is included in the tipping moment but not in `total_weight_empty`.
- The weights (`w_frame = 350`, `w_arm = 180`, …) are guesses; they aren't computed from part volumes × 7.85 g/cm³, which the model could do.
- The stability result is computed only at the current animation pose. With the default `$t = 0` that is arms-down, which is not the worst case.
- `LIFT_CYL_ARM_OFFSET = ARM_LENGTH * 0.50` ([params:552](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L552)) is 50 % of the *legacy straight-arm length* (1,657 mm → 828.5 mm), not of `ARM_MAIN_LEN`. The main file's comment still says `ARM_LENGTH * 0.35` ([main:375](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L375)).
- `_theta_max_lift_estimate = 50` while the comment says 60 ([params:556-560](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L556-L560)).
- *(Added 2026-09-26.)* The rebuilt analysis uses the L-arm geometry, puts the load inside the bucket and covers the whole arm range. With the same estimated masses, it puts the tipping load at about 184 kg at full reach, with the arms near level. That makes the rated operating capacity about 92 kg. The short 750 mm wheelbase and the long reach need counterweight or a longer wheelbase.

**Fix:** compute the stability envelope over the whole arm/bucket range (the collision tool already sweeps poses, and it could emit CoG too). Derive masses from geometry where possible, and pick the lift-cylinder attachment relative to `ARM_MAIN_LEN`.

### B6. The lift cylinder has two different closed lengths, and the rendered cylinder is clamped at rest — **High**

- **Params** uses `closed = stroke + bore + 90` = **803.5 mm** ([params:615](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L615), [:643](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L643)).
- **`hydraulic_cylinder()`** computes its own retracted length, `0.8B + (stroke + 0.8B + 5) + 0.2B + 5 + 1.5R` = **831.45 mm** ([hydraulics.scad:27-41](../../DESIGN-STRUCTURAL/openscad/modules/hydraulics.scad#L27-L41)).
- **The bucket cylinder** uses a third formula, `1.8B + 1.5R + 10` ([params:977](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L977)). That one happens to equal the module's result for the bucket bore and rod.

At `ARM_MIN_ANGLE` the pin-to-pin distance is 830.45 mm, so the module asks for −1.0 mm extension and silently clamps it to 0 (echo: `target_extension (before clamp): -1.00031`). The drawn cylinder is 1 mm longer than the space it sits in.

The stroke selection is also questionable:

- It picks **650 mm (25.6")**. That isn't a catalogue stroke; the bucket path picks from inch strokes, and the lift path from a 50 mm ladder.
- It applies a **30 % margin "to account for any angle limiting done elsewhere"** ([params:623-626](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L623-L626)). But angle limiting *reduces* the stroke needed. The result is 143 mm of stroke the geometry can never use, and a longer, costlier, more buckling-prone cylinder.
- `_max_stroke_from_closed` is computed and never used.
- `select_standard_stroke()`, `_LIFT_STROKE_RECOMMENDED`, `CYLINDER_STROKE_MARGIN` (1.15) and `CYLINDER_CLOSED_MARGIN` in the main file are a third, unused sizing path ([main:339-368](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L339-L368), [:446-513](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L446-L513)).

**Fix:** use one `cylinder_dims(bore, rod, stroke)` function in params that returns `[closed, extended, …]`, used by both sizing and rendering. Pick strokes from one catalogue list (inch strokes, like the bucket). Drop the 30 % factor, or justify it. Pass `closed_length` into `hydraulic_cylinder()` instead of recomputing it, and make an out-of-range length an `assert` or render the cylinder red (April #5).

### B7. The same joints are drawn with four different pin sizes — **High**

| Joint | Hole or pin as drawn |
|---|---|
| Bucket-cylinder base lug on the cross beam | lug hole `BOLT_DIA_3_4 + 2` = **21.05** ([main:3999](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L3999)); pin drawn by `oriented_cylinder` `rod × 0.6` = **22.86**, which is larger than the hole ([main:1165](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L1165)); cylinder clevis hole `bore × 0.4` = **30.5** ([hydraulics.scad:107](../../DESIGN-STRUCTURAL/openscad/modules/hydraulics.scad#L107)) |
| Bucket-cylinder rod end on the bucket lug | lug hole **21.05** ([main:4176](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L4176)); pin `0.8 × 22.86` = **18.3** ([main:1193](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L1193)); rod-clevis hole `rod × 0.5` = **19.05** |
| Lift-cylinder base | `clevis_pin(BOLT_DIA_1)` = **25.4** ([main:1720](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L1720)) *and* a coaxial 22.86 pin from `oriented_cylinder`; panel hole `BOLT_DIA_1 + 2` |
| Lift-cylinder rod end on the arm bracket | bracket bolt `HYD_BRACKET_BOLT_DIA` = **25.4** ([params:682](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L682)); pin drawn **18.3** |
| Structural check | assumes **1" clevis pins everywhere** (`CLEVIS_PIN_DIA`, [main:926](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L926)) |

**Fix:** add `LIFT_CYL_BASE_PIN_DIA`, `LIFT_CYL_ROD_PIN_DIA`, `BUCKET_CYL_BASE_PIN_DIA`, `BUCKET_CYL_ROD_PIN_DIA` and `BUCKET_PIVOT_PIN_DIA` to params, taken from the chosen catalogue cylinders. Size every hole as `pin + PIN_CLEARANCE` and pass the pins into `oriented_cylinder()`/`hydraulic_cylinder()`.

### B8. The arm-pivot DOM is 120 mm long in the assembly but 50.8 mm in the fabrication part — **High**

- [loader_arm_v2.scad:178-180](../../DESIGN-STRUCTURAL/openscad/modules/loader_arm_v2.scad#L178-L180) draws the DOM with `h = SANDWICH_SPACING` (120 mm).
- [parts/pivot_mount_assembly.scad:43](../../DESIGN-STRUCTURAL/openscad/parts/pivot_mount_assembly.scad#L43) and `PIVOT_MOUNT_DOM_LENGTH` in [params:189](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L189) use 50.8 mm. (`PIVOT_MOUNT_DOM_LENGTH` is also flagged as unused.)
- The arm is 63.5 mm wide (2" tube plus two 1/4" plates) inside a 120 mm sandwich gap. So either the DOM doubles as the lateral spacer (120 mm) or separate spacers/thrust washers are needed. **Neither is in the part files.**

**Fix:** decide which, make `pivot_mount_assembly()` the single source of truth, and have `loader_arm_v2()` instantiate it (April #17).

### B9. UTU angle irons don't match the holes drilled for them — **High**

`utu_frame_tube_angle_iron()` ([UTU:736-768](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_UTU.scad#L736-L768)) is a local copy of part A4, but with **3/8" holes at 20/40/60/80 % and 10/30/50/70 % of the length**. The real A4 ([angle_iron_a4_frame_tube_mount.scad:6-37](../../DESIGN-STRUCTURAL/openscad/parts/structural/angle_iron_a4_frame_tube_mount.scad#L6-L37)) has **1/2" holes at ±2" (wall leg) and ±1" (tube leg)**.

The UTU variant places the copy on all four wall panels ([UTU:832-835](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_UTU.scad#L832-L835)), whose holes are 1/2" at ±50.8 mm ([main:2260-2261](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L2260-L2261)). It also places it on the new UTU plates, which *also* cut 1/2" at ±50.8 mm ([UTU:642-664](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_UTU.scad#L642-L664)). Nothing lines up. The design notes claim "Same angle iron parts (A4)" ([UTU:1441](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_UTU.scad#L1441)).

**Fix:** call `part_a4_frame_tube_mount()` and delete the copy.

### B10. UTU design notes promise mounting holes that don't exist — **High**

The notes ([UTU:1418-1433](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_UTU.scad#L1418-L1433)) say the UTU motor mounts to the inner wall "using the same UWU bolt pattern already cut into those panels", and that the outer wall is the bearing mount. But:

- The wall panels only cut the **bearing** pattern (`UWU_BOLT_CIRCLE_DIAM` 157.2 BCD), and only at `_FRONT/_REAR_WHEEL_AXIS_Y` at the UWU shaft height ([main:1426-1445](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L1426-L1445)). The **motor** pattern (`UWU_MOTOR_BCD` 209.6) is only on the motor plates, and the UTU removes those.
- None of the three UTU axes is at a UWU hole: drive axis Y = 577.8, Z = 552.2; idlers Y = 1,300 / 345.2, Z = 185.3; UWU holes are at Y = 1,150 / 400, Z = 308.75.

So the drive shaft passes through two wall panels, and the drive motor bolts to one, with **no holes in either**. The UTU needs its own side-panel variant (or a parametrised hole list) and a matching CNC export.

### B11. The UTU file misunderstands `use <>` scoping; its overrides mostly do nothing — **Medium**

I tested this with OpenSCAD 2021.01. A module imported with `use` reads **its own file's** top-level variables. It can't see the caller's variables (a variable defined only in the caller is `undef` inside the module), while `-D` command-line overrides reach both. Consequences:

- The comment "We re-derive all variables needed by the modules we call" ([UTU:43-45](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_UTU.scad#L43-L45)) is wrong. The ~120 lines of re-derived constants are needed only by UTU's *own* modules. The modules imported from `lifetrac_v25.scad` (side panels, stiffeners, `loader_arms()`, `lift_cylinders()`, `folding_platform_assembly()` …) keep using `lifetrac_v25.scad`'s values.
- The UTU's `show_hydraulics`, `show_loader_arms`, `show_bucket`, `show_folding_platform` and `platform_fold_angle` only work because the UTU assembly wraps each call in its own `if` ([UTU:1370-1398](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_UTU.scad#L1370-L1398)). The same flags inside the imported modules still read the main file's values.
- The good news: `-D ARM_LIFT_ANGLE=…` does pose the UTU variant correctly (verified), so the collision tool can be pointed at it.

**Fix:** as in April #2: move derived geometry into `lifetrac_v25_derived.scad`, `include` it from both top-level files, and delete the re-derivation block.

### B12. Smaller correctness issues — **Medium**

- **`exploded_view` does nothing.** `explode_distance` is computed ([main:49-50](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L49-L50)) and never read.
- **`structural_steel.scad` `center=` is wrong.** `square_tubing()` always centres its cube, so `center=false` gives a centred tube and `center=true` offsets it by −L/2 ([structural_steel.scad:30-34](../../DESIGN-STRUCTURAL/openscad/modules/structural_steel.scad#L30-L34)). `angle_iron()` extrudes along −Y but applies the centering offset in Z ([:68-70](../../DESIGN-STRUCTURAL/openscad/modules/structural_steel.scad#L68-L70)). Nothing calls these today (0 external calls), so this is latent.
- **`side_panel_with_holes()` is dead and wrong.** It is never called ([main:1512-1551](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L1512-L1551)). It places the lift-cylinder base hole at panel X = `WHEEL_BASE − LIFT_CYL_BASE_Y` (1,350) and Y = `LIFT_CYL_BASE_Z` (absolute Z used as a panel-local coordinate). The real pin is at Y = 50, Z = 600. Delete it before someone revives it.
- **Duplicate module names with different signatures.** `bolt_hole()` is defined in both [main:1215](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L1215) (centred, `d = diameter`) and [fasteners.scad:117](../../DESIGN-STRUCTURAL/openscad/modules/fasteners.scad#L117) (not centred, `d = diameter × 1.1`). `hex_nut()` is also defined in both, with different parameters. Which one runs depends on the calling file.
- **Stale side-panel clearance comment.** `side_panel_profile()` assumes a 3" arm tube ("half is 38.1 mm", [side_panel.scad:20-22](../../DESIGN-STRUCTURAL/openscad/parts/side_panel.scad#L20-L22)). With the 6" tube, the arm's underside line at `ARM_MIN_ANGLE` is about 6 mm below the inner-panel top edge. This isn't a collision (the arm runs in the sandwich gap), but the 80 mm is no longer a real clearance and should be derived from `TUBE_2X6_1_4`.
- **The cross-beam arc slots are leftovers.** They are sized from the obsolete `CROSS_BEAM_SIZE = TUBE_2X2` (80.8 mm), while T3 is 152.4 mm deep along the arm. I rendered the inner panel against T3 at −27.7°, 0°, 30° and 49°: the intersection is empty in every pose, because the panel top is already cut down along the arm's lowest line. The slots only notch the panel edge in the 5° over-travel. Remove them, or size them from T3.

---

## 4. Clean-up and improvement opportunities

### C1. Architecture: one source of truth

1. **Split params into layers.** Suggested files:
   - `lifetrac_v25_params.scad`: inputs only.
   - `lifetrac_v25_derived.scad`: solvers, wheel positions, `FRAME_TUBE_*`, `BOTTOM_PLATE_*`, `MOTOR_PLATE_*`, `ANGLE_SEGMENT_*`, `UWU_SHAFT_Z`.
   - `lifetrac_v25_toggles.scad`: display flags.
   - `lifetrac_v25_analysis.scad`: structural and stability checks.

   Today `BOTTOM_PLATE_*`, `FRAME_TUBE_*`, `MOTOR_PLATE_*` and `ANGLE_SEGMENT_*` live in the main file ([main:1824-1916](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L1824-L1916), [:2848-2855](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L2848-L2855)), so the UTU and the parts have to copy them.
2. **Delete every re-declaration** in `lifetrac_v25.scad` (material, machine, engine, bucket, deck and angle constants; `PIVOT_PANEL_X/Y`; `CROSS_BEAM_SIZE/CLEARANCE`). Also delete the fourth and fifth copies of the tube/plate constants in [structural_steel.scad:9-17](../../DESIGN-STRUCTURAL/openscad/modules/structural_steel.scad#L9-L17) and [plate_steel.scad:9-11](../../DESIGN-STRUCTURAL/openscad/modules/plate_steel.scad#L9-L11).
   **Note:** the README spec extractor in `openscad-render.yml` greps `MACHINE_WIDTH`, `WHEEL_DIAMETER` etc. *from `lifetrac_v25.scad`*. It will silently stop updating once those lines move. Point it at params, or better, have OpenSCAD `echo()` the specs and parse that.
3. **Remove all 125 `is_undef()` fallbacks** (April #3). Files that `include` params never need them, and their literal defaults are already stale (e.g. `side_panel.scad` falls back to `HYD_BRACKET_ARM_POS = 300`; the real value is 828.5).
4. **Stop using `ARM_LENGTH` / `ARM_GROUND_ANGLE` / `MIN_HORIZONTAL_REACH`.** These describe the old straight arm but still feed the lift-cylinder offset, capacity, CoG, the cross-beam z-range, `CROSS_BEAM_2_POS` and the CI camera maths.
5. **Replace the four near-identical side-panel modules** (`side_panel_left_outer/left_inner/right_inner/right_outer`, [main:1553-1703](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L1553-L1703)) with `side_panel_placed(side, is_inner)`. Do the same for `stiffener_side_panel_cutters()`, which has 8 hand-written branches ([main:2124-2327](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L2124-L2327)), and replace the 20 hand-placed `frame_tube_angle_iron()` calls ([main:3169-3283](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L3169-L3283)) with a table of panel faces and a loop. Also:
   - Fold the 24 near-identical `_plate_segment_holes()` calls ([main:2754-2801](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L2754-L2801)) into two loops.
   - Collapse the two copies of the motor-plate hole block ([main:2930-3026](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L2930-L3026)) into one module with a `mirror`.
6. **Make the assembly call the part modules.** The main file re-draws parts that have their own part files: bucket plates ([main:4020-4145](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L4020-L4145)) and `parts/bucket_*.scad`, the pivot mount (`loader_arm_v2` and `parts/pivot_mount_assembly.scad`), the UWU wheel (inline in `uwu_assembly_positioned` and `wheels.scad` `bobcat_wheel()`), and the frame tubes (UTU `utu_extended_frame_tube` and T1/T2). A part drawn twice will drift; B8 and B9 are examples. The rule should be: the assembly only places parts.
7. **Move helper geometry out of the main file.** `u_channel_lug`, the pivot rings, the hex bolt/nut/clevis pin, and `vec_*`/`rot_x` all belong in `modules/`. The main file should contain placement only; today it is 4,883 lines.

### C2. Dead code (verified by reference count across all 57 files)

- **Unreferenced modules/functions in `lifetrac_v25.scad` (16):** `cross_beam_z_range`, `select_standard_stroke`, `select_standard_bore`, `pivot_ring_large`, `pivot_ring_small`, `pivot_boss`, `hex_bolt_assembly`, `side_panel_with_holes`, `get_front_stiffener_holes`, `horizontal_angle_iron_smart`, `split_horizontal_angle_iron`, `split_horizontal_angle_iron_rotated`, `mid_stiffener_plate`, `cross_tube_cutout_profile`, `cross_tube`, `engine`.
- **Unreferenced in params:** `bucket_cyl_to_back_angle()` (the "legacy" wrapper).
- **Unused top-level variables:**
  - **31 in main.** Examples: `STANDARD_BORES_*`, `STANDARD_STROKES_MM`, `CLOSED_LENGTH_RATIO`, `BUCKET_MAX_CURL/DUMP`, `BUCKET_CYL_ARM_Z`, `BUCKET_CYL_BUCKET_Y/Z`, `DECK_*`, `MIN_BOLT_SPACING`, `PIVOT_RING_OUTER`, `TOTAL_WELD_AREA_PIVOT`, `SAFETY_FACTOR_FATIGUE`, `explode_distance`, `_LIFT_CLOSED_LENGTH`.
  - **27 in params.** Examples: `ARM_PIVOT_EXT`, `ARM_TANGENT_ANGLE`, `PIVOT_HOLE_EDGE_OFFSET`, `PIVOT_MOUNT_DOM_LENGTH`, `PIVOT_MOUNT_TUBE_START`, `BOBCAT_CENTER_BORE`, `BOBCAT_LUG_SPACING`, `BUCKET_CYL_MOUNT_Y_OFFSET`, `PLATFORM_CLEARANCE`, `PLATFORM_BRACKET_CORNER_RADIUS`, `deck_bolt_inner_dist/outer_dist`, `_attach_*_prelim`, `_bend_deflection`, `_E`.
- **Dead computation.** The mid-plate position block inside `base_frame()` runs even though the plate is commented out ([main:3318-3341](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L3318-L3341)). `slope_x_at_flat_top` is computed and never used ([main:1583-1592](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L1583-L1592)). `shift_y = 0` and `deck_shift_y = 0` in the platform are unused ([main:4553-4557](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L4553-L4557)). The L2 computation is repeated three times ([params:262-343](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L262-L343)).
- **Unused library modules.** `structural_steel.scad` (all 6 modules unused), `wheels.scad` (`bobcat_wheel`, `powered_wheel`, `modular_wheel_unit`, `axle`, `bearing_housing`, `complete_wheel_assembly` are not used by the assembly), `hydraulics.scad` (`hydraulic_pump`, `hydraulic_valve`, `hydraulic_hose` are only used in its own demo). Either use them (see section 6) or move them to a `library/` folder that is clearly not part of the machine.

### C3. Output noise

- Put every diagnostic `echo()` behind `DEBUG = false` (April #6). Keep one machine-readable summary block, e.g. `echo("SPEC", key, value)` lines, which CI and the README workflow can parse instead of grepping source.
- Remove per-instance echoes inside modules (`hydraulic_cylinder`, `oriented_cylinder`, `arm_plate`, `lift_cylinders`, `bucket_cylinders`). The same "ARM PLATE HYD BRACKET DEBUG" block prints 4 times per render.
- Remove `$fn` from `hydraulics.scad`, `wheels.scad`, `structural_steel.scad` and `plate_steel.scad`, and **especially `$fn = 6` from `fasteners.scad`**. Set `$fa`/`$fs` once in each top-level file and use explicit `$fn` only where a round part must look round.

### C4. Comments that should be deleted or rewritten

- **[main:4651-4780](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L4651-L4780): about 130 lines of pasted AI working notes** in the middle of `folding_platform_assembly()`. Examples: "`platform_transverse_angle` is not defined in the snippet", "Wait, if rot=0, then Y is Depth?", "Let's write the code." They start on the same line as real code (`platform_transverse_angle(transverse_len);        // And …`), which makes them easy to miss.
- [main:4403-4487](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L4403-L4487): about 80 more lines of the same in `platform_angle_iron()` ("Wait. Bracket X is distance from Pivot…", "This is confusing.").
- [params:481-495](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L481-L495) ("But wait… we hope") and [params:691-720](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L691-L720) (30 lines deriving `BUCKET_LUG_OFFSET`).
- [loader_arm_v2.scad:121-127](../../DESIGN-STRUCTURAL/openscad/modules/loader_arm_v2.scad#L121-L127) ("Re-calc logic for local context? … wait").
- **Commented-out code:**
  - [params:98-103](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L98-L103), [:123](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L123), [:205-207](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L205-L207), [:214](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L214), [:511-515](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L511-L515), [:684-686](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L684-L686), [:764-768](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L764-L768)
  - [main:172](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L172), [:181-182](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L181-L182), [:3341](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L3341), [:4859](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L4859), [:4880-4883](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L4880-L4883)
- **Empty section headers:**
  - "BOBCAT QUICK ATTACH INTERFACE" ([main:4006-4011](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L4006-L4011))
  - a duplicated "BUCKET" header ([main:4012-4018](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L4012-L4018))
  - "LOADER ARMS" followed by blank lines ([main:3965-3970](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L3965-L3970))
  - "JIG PARAMETERS & NEW ARM PARAMETERS" followed by 12 blank lines ([params:1289-1303](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L1289-L1303))
- **Misleading names:** `_theta_rad` and `_gamma_rad` hold degrees ([params:326](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L326), [:495-496](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L495-L496)). `ARM_ANGLE`, `ARM_SHAPE_ANGLE` and `ARM_TANGENT_ANGLE` are three names for related things, one of them unused. The lowercase params (`deck_bolt_offset`, `pivot_bolt_offset`, `deck_bolt_inner_dist`) break the `UPPER_CASE` convention.
- **Stale file header.** The params header says "This file contains only the constants needed for part design"; it now contains three solvers and 128 echoes.

### C5. Small simplifications

- **Replace the stroke ladders with one expression.** [params:629-640](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L629-L640) and [main:345-355](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L345-L355), [:503-513](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L503-L513) are equivalent to `max(150, ceil(x/50)*50)`. Better still, pick from a catalogue list (see B6).
- **Drop the `0 * cos(...)` terms** in the lug-position maths ([params:847-854](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L847-L854), [:921-924](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L921-L924)). Better, use a `rot2(v, a)` helper.
- **Drop the `_anim_abs_angle = ($t == 0) ? … : …` special case** ([main:253-254](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L253-L254)). The general branch already gives the same answer at `$t = 0`.
- **Remove `safe_min`/`safe_max`** ([main:571-581](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L571-L581)). `min(list)` over values that can't be `undef` is enough, and the six-way min/max only feeds a debug echo.
- **Normalise `PIVOT_MOUNT_BOLT_ANGLES` with `% 360`** and fix the 270° comments (April #12).
- **Use `FRAME_TUBE_OFFSET_FROM_WHEEL` / `FRONT_FRAME_TUBE_Y` / `REAR_FRAME_TUBE_Y`** instead of the literal `152.4` in the UTU ([UTU:563-568](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_UTU.scad#L563-L568), [:591-592](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_UTU.scad#L591-L592), [:794-795](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_UTU.scad#L794-L795)).
- **Fix the hub docstrings.** `wheels.scad` says "rim width (default 9.75")" and "tire width (default ~12.5")" while the values are 10.0" and 10.0" ([wheels.scad:24-27](../../DESIGN-STRUCTURAL/openscad/modules/wheels.scad#L24-L27)).

### C6. CI and tooling

- **Add one "compile everything" job.** It should run `openscad --hardwarnings -o x.echo` on every `.scad` file except intentionally broken scratch files, then fail on any `WARNING`, any `assert`, or any `FAIL|SAFE\?, "NO"|✗` echo. This would have caught the four broken jig/test files, and B1b.
- **Fix the workflows that currently mislead** (details in section 5.5):
  - The structural-analysis job sends its output to `/dev/null`.
  - `generate-part-svgs.yml` points at a folder that doesn't exist.
  - The PNG and GIF camera maths breaks on a trailing comment.
  - `openscad-render.yml` greps variables that moved to params, and hard-codes `ARM_LENGTH_APPROX=1600` and a 1,200 kg capacity.
  - Several jobs pipe through `grep -v WARNING || true`.

  Replace every `grep`-the-source extraction with values that OpenSCAD itself `echo`s.
- **Add a `Makefile` or `build.sh`** that regenerates `assembly.png`, the animation GIFs, `cnclayout.svg`, the per-part DXF/SVGs and a BOM from the SCAD (April #34).

---

## 5. Parts, jigs, CNC layout and CI (detail)

All 38 files under `parts/` evaluate without OpenSCAD warnings. I rendered all 10 `export_*.scad` wrappers to SVG and measured the output. The most serious findings (P1, P2, P3) were re-checked by hand.

### 5.1 Plate parts and CNC exports

**P1. Four of the ten export wrappers output a thin strip, not the part — Critical.** *(Fixed in this PR; see Status.)* Each one rotates a plate that already lies flat onto its edge, then takes `projection(cut=true)` at z = 0:

```scad
// parts/export_side_panel_outer.scad (same pattern in _inner, _rear_crossmember, _bucket_side)
projection(cut=true)
rotate([90, 0, 0])  // "Rotate to lay flat": but side_panel() is already flat
side_panel(is_inner=false);
```

| Wrapper | Rendered SVG | Should be |
|---|---|---|
| `export_side_panel_outer.scad` | **1,373 × 13 mm** | about 1,400 × 1,000 mm outline |
| `export_side_panel_inner.scad` | **1,373 × 13 mm** | ditto |
| `export_rear_crossmember.scad` | **1,046 × 13 mm** | plate outline |
| `export_bucket_side.scad` | **7 × 588 mm** | 450 × 600 mm side profile |
| `export_bucket_bottom.scad`, `export_platform_deck.scad` | 1,100 × 600, 736 × 400 | correct |

These four are the 1/2" side panels, the most important plates on the machine. `export_platform_deck.scad` and `export_standing_deck.scad` produce byte-identical SVGs.

**P2. Even a corrected side-panel export would not match the assembly — High.** `parts/side_panel.scad` has the outline, pivot hole, arc slots and hydraulic-bracket notch. But the assembly adds, *on top of it*:

- cross-tube cutouts
- UWU bearing holes
- all stiffener and angle-iron bolt holes
- the inner-panel front and bottom trims

([main:1553-1703](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L1553-L1703), [:2124-2327](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L2124-L2327)). The CNC side panel is therefore missing dozens of holes. `side_panel.scad` also still has:

- legacy "wheel axle" holes that no longer match the axles ([side_panel.scad:221-226](../../DESIGN-STRUCTURAL/openscad/parts/side_panel.scad#L221-L226)): the rear one is a half-notch in the edge and the front one falls outside the outline, while the real axles are at Y 400 and 1,150.
- a stowed-platform lock hole whose maths puts it at x ≈ −52, so it is never cut ([:252-260](../../DESIGN-STRUCTURAL/openscad/parts/side_panel.scad#L252-L260)).
- a lift-cylinder hole ([:218](../../DESIGN-STRUCTURAL/openscad/parts/side_panel.scad#L218)) that passes through the A1/A2 angle legs, which have no matching hole.

**Fix:** move *all* panel features into `side_panel(is_inner, side)` so that the assembly and the export call exactly the same module. Export with `projection()` of the flat part (no rotate), and add a CI check that each exported SVG's bounding box matches the expected plate size.

**P3. The arm-pivot hole in the side panels leaves about 4 mm of steel to the edge — Critical.** *(Fixed in this PR; see Status.)* The Ø40.1 mm pivot hole at panel (200, 950) ([side_panel.scad:213](../../DESIGN-STRUCTURAL/openscad/parts/side_panel.scad#L213)) sits 24.0 mm from the steep edge running from (200, 1,000) to (300, 817.5) ([:31-49](../../DESIGN-STRUCTURAL/openscad/parts/side_panel.scad#L31-L49)). That leaves a **4.0 mm ligament** (computed, and measured on the rendered outline as 4.2 mm after corner rounding). This pin carries the whole loader arm reaction. Edge distance for a loaded pin hole should be roughly 1.5–2 × the hole diameter, i.e. 60–80 mm.

**Load check** *(added 2026-09-26; Appendix C)*. With both lift cylinders at the 3,000 psi relief pressure, the pin pushes on each pair of panels with 59–69 kN. With the arms level or raised, that force points forward and up, toward this thin edge. The steel in that direction is only 5.5–5.7 mm. The nominal AISC tear-out strength of the two 1/2" panels (1.2 × l_c × t × F_u, before any safety factor) is 67–69 kN. So the hole has essentially no margin at relief pressure. At the 2,000–2,500 psi working pressure the load is still above what AISC ASD allows (Ω = 2).

**Fix:** keep a flat "pivot boss" region in the profile around `ARM_PIVOT_Y` with radius ≥ `2 × PIVOT_PIN_DIA`, and `assert` the edge distance.

**P4. Round holes come out as rectangles.** *(Fixed in this PR.)* In `cylinder_lug.scad:25` and `bucket_side.scad:26,33` the hole cylinders are rotated into the plane of the plate, so the projection shows 16.7 × 24.1 and about 10 × 13 mm rectangles.

**P5. Hole-to-edge ligaments of 2.65 mm** on `bucket_bottom.scad` and `bucket_side.scad`: holes 10 mm from the edge ([bucket_bottom.scad:14,23](../../DESIGN-STRUCTURAL/openscad/parts/bucket_bottom.scad#L14-L23)). Note that the assembly and cnclayout don't use these bolt-hole bucket plates at all; they use the tab-and-slot plates drawn inline in main.

**P6. `arm_plate.scad`:**
- Top-level `arm_plate();` (April #9).
- Stale fallback bolt angles for the old bottom slot ([arm_plate.scad:14-21](../../DESIGN-STRUCTURAL/openscad/parts/arm_plate.scad#L14-L21)).
- **Zero-clearance** holes: the bucket-pivot hole equals the pin (`:172`); the 1/2" holes are exactly 12.7 mm; the cross-beam cutout is exactly 152.4 × 50.8, the tube's outside size (`:238-240`).
- "Alignment" holes (`:214-219`) with no mating holes in the arm tube.
- No holes for the leg-spacer bolts.
- Per-call debug echoes (`:85-89`).
- No export wrapper, although cnclayout places four of these plates.

**P7. Orphaned or obsolete plate parts:**

| File | Status |
|---|---|
| `pivot_mount_assembly.scad` | Only mentioned in a comment. It describes a bottom slot of DOM width, while the arm plate cuts a rear 63.5 mm slot. DOM length conflicts with the assembly (B8). Its exploded view moves along the wrong axis. It sets a global `$fn`, and the bolt loop is written out three times |
| `rear_crossmember.scad` | `use`d by main but never called; superseded by `back_stiffener_plate()`. Its bolt holes face the wrong way, its platform-pivot holes run along Y while the pin runs along X, and it uses `PLATFORM_ARM_LENGTH` as the lock radius |
| `standing_deck.scad` | Nothing references it. Main defines a *different* `standing_deck()` ([main:4848](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L4848)) |
| `cylinder_lug.scad` | Obsolete (the assembly uses a through-bolt and U-channel lugs), yet cnclayout still places 6 |
| `wheel_mount.scad` | Not used by the UWU assembly, but cnclayout still places 4 |
| `bucket_bottom.scad`, `bucket_side.scad` | Used only by exports and CI, not by the assembly |
| `lifetrac_v25.scad:19-26` | Imports six part files it never calls |

**P8. The pre-render guard in 10 part files is a no-op.** The guard is `if ($preview || len(search("bucket_bottom.scad", parent_modules())) == 0)`. The `parent_modules()` stub in params always returns `[]`, so the condition is always true. Any file that `include`s a part gets a stray copy at the origin; only `use` protects it. Affected: `bucket_bottom:30`, `bucket_side:40`, `cylinder_lug:39`, `platform_angle_arm:97`, `platform_deck:142`, `platform_pivot_bracket:105`, `rear_crossmember:86`, `side_panel:266`, `standing_deck:78`, `wheel_mount:39`. Use the `parts/export_*.scad` wrappers for rendering, and delete both the guards and the shim (April #28).

**P9. `platform_angle_arm.scad` is broken, and the BOM points fabricators at it as the drilling template** ([BILL_OF_MATERIALS.md:25-28](../../DESIGN-STRUCTURAL/documentation/BILL_OF_MATERIALS.md)):
- The L profile has a 44.45 mm leg instead of 50.8, pointing +Z where the header says −Z (`:52-57`).
- The pivot-end holes miss the part (`:68-70`).
- The deck holes are 2 mm blind pockets (`:81-82`).
- The hole spacing doesn't match `platform_pivot_bracket` or the deck.

The assembly uses A7/A8 geometry drawn inline instead.

### 5.2 Structural cut-list parts (`parts/structural/`)

**P10. A6 (bottom angle iron): wrong segment lengths in both the assembly and the cut list — High.**
- `angle_iron_a6_bottom_horizontal.scad:114-147` reads `ANGLE_SEGMENT_*` and `ANGLE_IRON_GAP`. Those are defined only in `lifetrac_v25.scad` ([main:1883-1905](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L1883-L1905)).
- A6 is reached through `use` (main → `structural_parts.scad` → a6), so it can't see them (Appendix B) and always falls back to **200 / 300 / 250 mm**.
- The real segment lengths are **146 / 546.8 / 97.6 mm**.
- The 30 A6 pieces in the assembly (the header says 24) therefore have their wheel gaps in the wrong place, and the standalone cut-list file prints the wrong lengths.
- A6 also drills a fixed 150 mm hole pitch, while the bottom plate and panels use the percentage functions ([main:1812-1820](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L1812-L1820)), so the holes don't line up.

This is a direct consequence of the missing `derived.scad` layer (C1).

**P11. A1/A2 (back vertical angles): holes don't match the back plate or the panels — High.** A1/A2 drill at **50 / 200 / 350 / 500** mm and **100 / 250 / 400 / 550** mm (fixed 150 mm pitch, `a1:28-48`). For the 650 mm height, the back plate and panels drill at **130 / 260 / 390 / 520** and **65 / 195 / 325 / 455** ([main:1792-1793](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L1792-L1793)). None of the 8 holes per angle line up. A1 and A2 are also geometrically identical.

**P12. T5 (arm leg spacer) is an empty solid — High.** The "hollow" is subtracted with a profile identical to the outside: an `offset(r=12.7)` rounded 152.4 × 50.8 rectangle minus the `hull()` of four r = 12.7 circles at (±63.5, ±12.7), which is the same rectangle. The result is a sliver about 0.07 mm thick. The same code is in [loader_arm_v2.scad:142-150](../../DESIGN-STRUCTURAL/openscad/modules/loader_arm_v2.scad#L142-L150), so **the assembly shows no leg spacer at all**. Also:
- The taper is never reached (the 150 mm stock is shorter than the 248 mm taper start).
- The header says a 150° bend; it is 130°.
- The spacer bolt pattern is "rough placement" (U7).

**P13. Other structural-part notes:**
- **A7:** deck holes 25.4 mm from the ends vs 34 mm (`deck_bolt_offset`) in the deck.
- **A9, A10:** fall back to literals because `FRAME_TUBE_HEIGHT` and `BOLT_DIA_3_8` are main-only.
- **T1/T2:** add 8 motor-plate holes per tube with no mating angle. T2 is a verbatim copy of T1, and the ends are modelled closed.
- **T4:** duplicates `loader_arm_v2` and is unused.
- **Parts never instantiated:** A3, A7, A8, A9, T4, T5 (A7/A8/T4/T5 are drawn inline instead).
- **Duplicate part numbers:** A1 ≡ A2, A3 ≡ A10, A4 ≈ A5.
- **Piece count:** `structural_parts.scad:122-123` claims 71 pieces, but its own subtotals add to 77.

**P14. The bucket pivot joint is weaker than the bucket cylinders that load it — Critical.** *(Added 2026-09-26; see Appendix C.)* Each 3" bucket cylinder pushes with 94.3 kN at the 3,000 psi relief pressure. When the dump stroke is resisted, for example when prying or pushing down with the bucket, that push goes through the bucket pivot pin at each arm tip. Three parts of that joint have no margin at relief pressure. All three are also below what AISC ASD allows (Ω = 2) at the 2,000–2,500 psi working pressure:

| Part | Geometry | Nominal strength, before any safety factor |
|---|---|---|
| Arm tip | The 1" pin hole is centred 25.4 mm from the end of the two 1/4" arm plates (`PIVOT_HOLE_X_FROM_FRONT = BUCKET_PIVOT_PIN_DIA`, [params:409](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L409); tip radius, [arm_plate.scad:160](../../DESIGN-STRUCTURAL/openscad/parts/arm_plate.scad#L160)). That leaves 12.7 mm of steel. The drop-leg tube stops short of the tip, so the plates carry the pin alone | Tear-out: **77 kN** |
| Bucket pivot lug | A 3×3×1/4 U-channel ([main:1226-1291](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L1226-L1291)) with the hole in the middle. The channel's open side faces the arm and ends 28.6 mm from the hole centre ([main:1256](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L1256)), leaving 14.9 mm of steel in each 1/4" wall. The pin pushes toward that side | Tear-out: **91 kN** |
| Lug-to-bucket bolts | 4 × 1/4" bolts ([main:1277-1288](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L1277-L1288), M8), which the pin pulls in tension | **68 kN** (Grade 5) or **85 kN** (Grade 8), before prying |

The tip can't simply be made bigger. The lug's base sits 31.75 mm from the pin, only 6.35 mm beyond the current tip, and it swings around the pin as the bucket tilts. The same U-lug with four 1/4" bolts also holds both ends of each bucket cylinder. On T3 those bolts are in shear and hold about 47 kN (Grade 5, threads in the shear plane), against the cylinder's 94 kN push.

**Fix:** redesign the joint and the cylinder lugs together, sized for the bucket cylinder that is actually chosen. For example: welded 1/2" clevis plates on a reinforced bucket back, an arm-tip boss with a radius of at least 2 × the pin diameter, and larger pins with bushings (M9). The model's 3" bucket cylinders push more than twice as hard as the 2" ones in the BOM ([BILL_OF_MATERIALS.md:71](../../DESIGN-STRUCTURAL/documentation/BILL_OF_MATERIALS.md); 41.9 kN at 3,000 psi). So choosing the cylinder (section 8, step 2) sets these loads.

### 5.3 CNC layout (`cnclayout.scad`, `export_for_cnc.scad`, backup)

`cnclayout.scad` renders 25 parts in about 28 s with 0 warnings. Its output is byte-identical to the committed `cnclayout.svg`. But:

**N1. Seven pairs of parts overlap on the sheet — High.** Each part was rendered to DXF separately and intersected pairwise in 2D. The overlapping pairs:

- bucket bottom × each bucket side, and the two bucket sides with each other
- platform deck × pivot bracket 1, and bracket 1 × bracket 2
- bracket 2 × wheel mount 1
- wheel mount 4 × cylinder lug 1

The causes:
- `layout_part()` ([cnclayout.scad:30-35](../../DESIGN-STRUCTURAL/openscad/cnclayout.scad#L30-L35)) assumes every part's origin is its lower-left corner. Several parts are centred on the origin, and 90° rotations push parts into negative X.
- Several assumed widths are wrong: bucket side 450 (actually 600), pivot bracket 350 (actually 400), arm plate 1,600 (actually 1,792).
- `x_pos_8` is computed and never used.

`CNCLAYOUT_CHANGES.md` claims "No Overlapping Parts".

**N2. The layout doesn't match the machine.**
- *Missing:* the back, bottom and front stiffener plates, the two motor plates, and the four circular pivot-mount plates (`pivot_mount_plate_flat()` already exists).
- *Extra:* 4 wheel mounts and 6 cylinder lugs that the assembly no longer uses.
- *Wrong outline:* the side panels are the raw `side_panel()` outline (P2). Left and right panels actually differ (`stiffener_side_panel_cutters(is_left)`), but the layout uses identical copies.

**N3. There is no manufacturing logic.**
- No kerf offset anywhere. The only mention is a "future" item in `CNCLAYOUT_CHANGES.md:123`.
- No sheet-size limit. It is a single strip 18.4 m × 1.84 m.
- 1/2" and 1/4" parts are mixed on one "sheet".
- No automated overlap check.

**N4. `export_for_cnc.scad` and `cnclayout_simple_outlines_backup.scad` are stale duplicates.**
- `export_for_cnc.scad` hard-codes wrong sizes: wheel mount 300 (actually 250), deck 1,000 × 400 (actually 735.5 × 400), bucket back and side heights 400 (actually 450).
- It exports parts that no longer exist.
- It **unions its label text into the cut geometry**: a 100 mm lug exports 123.8 mm wide.
- An unknown part name exits 0 and outputs a 100 × 100 sample plate.
- `export_all_cnc_parts.sh` drives it; under `set -e` its error branch is dead code.
- The backup file uses rectangles and 100 mm sandwich spacing (params: 120). Nothing references it. **Delete both** and generate CNC outputs from the part modules.

### 5.4 3D-printed jigs

| Jig | State |
|---|---|
| `3d_printed_bolt_hole_cutting_jigs/angle_iron_drill_jig.scad` | **Broken include** (`../../lifetrac_v25_params.scad`; should be `../`; fixed in this PR). Even fixed, it is a placeholder: a 100 mm L-sleeve with one 5 mm pilot hole per leg that matches no real hole pattern (A4 needs 2 holes per leg at 4" and 2") |
| `3d_printed_bolt_hole_cutting_jigs/tube_drill_jig.scad` | **Broken include** (fixed in this PR). Its hole cylinders run *along* the tube axis inside the cavity, so no holes are cut (the intersection with the shell is empty). The inner loop variable is unused, so each hole is drawn twice. Hard-coded 100/80 spacing matches nothing. Superseded by `3d_printed_welding_jigs/tube_drilling_jig.scad`; delete it |
| `3d_printed_welding_jigs/pivot_welding_jig.scad` | **Broken include.** The DOM cutout lies inside the lightening cutout, so it cuts nothing. The plate gap is 50.8 mm but should be 38.1 mm (50.8 − 2 × 6.35). Contains "Add alignment tabs?". Placeholder |
| `3d_printed_welding_jigs/pivot_mount_welding_jig.scad` | Compiles cleanly but can't be used as drawn: the 6" plates pass through the jig base (DOM centre at z = 48.4 mm, plate bottom at −27.8); the rings sit at the base height, not the DOM height; both rings get the same rotation, so the left one intersects its plate; it uses **6 bolts at 60°** where the part uses **5 bolts at non-uniform angles**; several unused parameters (`JIG_FINGER_*`, `DOM_CRADLE_ANGLE`) |
| `3d_printed_welding_jigs/tube_drilling_jig.scad` | Hole pattern correct (matches the A4 tube leg). The body is off-centre (walls 10.2 / 14.2 mm) because the `minkowski` cube starts at 0. The engraved marks and text are buried or float over the slot, so none are visible. It redefines `JIG_CLEARANCE` (overriding params) and duplicates `JIG_WALL_THICKNESS` |

The jig README says all jigs "fall back to defaults" (the three broken ones don't), and its image links point at an empty `renders/` folder.

**Fix:** correct the three include paths (verified: all three then compile with 0 warnings). Either finish the placeholders against the real part patterns or delete them. Add the jigs to the compile-everything CI job.

### 5.5 CI workflows and helper scripts

| Workflow or script | Problem | Evidence |
|---|---|---|
| `openscad-structural-analysis.yml` | **Captures nothing** (fixed in this PR). `openscad -o /dev/null --export-format echo` writes the ECHO lines into `/dev/null` (locally: 0 lines captured). So every run records "No structural analysis summary found". The committed [STRUCTURAL_ANALYSIS_LOG.md](../../DESIGN-STRUCTURAL/STRUCTURAL_ANALYSIS_LOG.md) (commit `c409fa0`, 2026-09-14) says exactly that. The PR comment says ✅ unconditionally, and the job never fails on the model's `FAIL` results. **Fix:** `openscad -o structural_analysis.echo …` and parse that file | local run and committed log |
| `generate-assembly-png.yml`, `generate-animation-gif.yml` | **Wrong camera.** The value extraction has no `head -1`, so `GROUND_CLEARANCE` comes out as `"150\n150"` (the trailing comment "150mm" also matches). The `$((…))` maths then errors, and the step carries on with empty values. The 2026-09-14 main-branch run rendered with `CAMERA_DISTANCE 4540`, `CENTER_Z 917`, instead of about 7,262 / 1,467. `ARM_MAX_ANGLE` isn't in the main file and silently defaults to 60 | job log of run 34865875850 |
| `generate-part-svgs.yml` | **Wrong paths** (fixed in this PR, along with the missing-output check and an artifact upload; the README-link and commit steps are removed). It watches and exports `DESIGN-STRUCTURAL/parts/…`, which doesn't exist (should be `openscad/parts/`). Failures are swallowed (`grep -v WARNING \|\| true`), so the job passes with no output. The outputs are git-ignored, so the README links point at files that are never committed. Even with fixed paths, 4 of the 8 outputs are the strips from P1 | |
| `openscad-render.yml` | Greps `ARM_MAX_ANGLE` and `LIFT_CYLINDER_BORE` from `lifetrac_v25.scad`, where they don't exist (they are in params). So the README spec table is **never updated** (the script exits 0 on the missing value). The module-render step looks in `DESIGN-STRUCTURAL/modules` (doesn't exist) yet reports "5 modules". The quoted heredoc prints a literal `$(date)`. `ARM_LENGTH_APPROX=1600` and a 1,200 kg capacity are hard-coded | |
| `generate-cnclayout-svg.yml` | `grep -v WARNING \|\| true`. The `-f cnclayout.svg` check always passes because the file is committed. The path-count check only warns. The triggers omit `lifetrac_v25.scad`, which supplies the bucket plates | |
| `generate-jig-previews.yml` | Only covers the welding-jig folder. `\|\| echo "Warning…"` swallows failures (the broken `pivot_welding_jig` renders an almost blank image and exits 0). **Its push-event "Commit and push renders" step has failed on both runs on main** (2026-02-03 and 2026-04-26): it `git add`s `renders/*.png` and `renders/*.jpg`, which the root `.gitignore` ignores (`*.png`, `*.jpg`), so `git add` exits non-zero. As a result the jig README's preview images are never committed | run 24945972952 job steps |
| `openscad-collision-check.yml` | **Working.** The `COLLISION_*` echo interface and the `-D` pose overrides are present and behave as documented | |
| `export_individual_svgs.sh` | Masks failures. Claims 4 outer and 4 inner side panels (the assembly has 2 of each). Runs the four broken wrappers | |
| `verify_design.py` | Hard-coded Windows path; fails on Linux | |
| `calc_kinematics.py`, `test_solve_arm.py` | Stale copies of the solver with old inputs (120° arm angle, 1,224/550 arm lengths, arm offset 0.25). They give different answers from the SCAD. **Delete** | |
| `scripts/calc_cut_coords.py` | Analyses a `triangle_leg` polygon that no longer exists in any `.scad` | |
| `test_cylinder_sizing.scad` | 49 warnings. Root cause: it re-assigns `LIFT_CYL_BASE_Y` (already set in params). OpenSCAD evaluates a re-assigned variable at its *first* position, inside params, where the local `calc_base` doesn't exist yet, so it becomes `undef` and the error cascades. It also silently overrides `FRAME_Z_OFFSET` and `WHEEL_DIAMETER`. Renaming the locals fixes the warnings, but its model (arm offset 0.25, unlimited 73.9° max) disagrees with params. **Delete** | |

### 5.6 Stray files to delete

- **In `openscad/`:** `cnclayout_simple_outlines_backup.scad`, `export_for_cnc.scad`, `temp.ipynb` (one empty scratch cell), `cylinder_output.txt` (a UTF-16 PowerShell error message), `calc_kinematics.py`, `test_solve_arm.py`, `test_cylinder_sizing.scad`.
- **In `DESIGN-STRUCTURAL/`:** `cnclayout_old_backup.svg`, `test_params_v3.scad` (echoes a variable that doesn't exist), `scripts/calc_cut_coords.py`, `export_all_cnc_parts.sh`, and `3d_printed_bolt_hole_cutting_jigs/tube_drill_jig.scad`.
- **Docs to rewrite from the model:** `CNCLAYOUT_PARTS_LIST.md` (describes the old 23-part layout, with parts A2/C1/C2/G1 that don't exist), `CNCLAYOUT_CHANGES.md`, `CUSTOM_PARTS_LIST.md` (lists pivot rings that aren't drawn and omits the pivot-mount plates), `README_EXPORTS.md` and `INDIVIDUAL_PARTS_GUIDE.md` (wrong `parts/` paths), and the CNC/part-SVG sections of `DESIGN-STRUCTURAL/README.md`.

---

## 6. Missing elements and modules

These are things a buildable v25 needs that have **no SCAD geometry at all**, or have geometry that is not placed in the assembly.

**Power unit and hydraulics (none of it is in the assembly)**

| # | Missing element | Notes |
|---|---|---|
| M1 | **Engine / power unit** | `engine()` exists but its call is commented out "for simplification" ([main:4859](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L4859)), and it has a TODO: "Design proper engine mounting plate and bolt pattern. Currently floating in space" ([main:3364-3365](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L3364-L3365)). Its 450 mm width against the 475 mm gap between motor plates has not been checked. The stability maths still counts its 60 kg |
| M2 | **Hydraulic pump, reservoir/tank, filter, oil cooler** | `hydraulic_pump()` exists in `hydraulics.scad` but is never placed. The BOM calls for a 25–30 gal reservoir, which is a large volume to fit, and 40 kg of fluids is assumed in the CoG |
| M3 | **Control valves / valve bank** | `hydraulic_valve()` exists, never placed. There is no valve mounting plate |
| M4 | **Hoses and routing** | `hydraulic_hose()` exists, never placed. No hose routing along the arms or across the pivots, and no bulkhead fittings |
| M5 | **Fuel tank, battery, muffler/exhaust, air intake** | No geometry |
| M6 | **Controller enclosure** | The v25 controller (Portenta/Opta, LoRa radio, E-stop) has no enclosure, mounting plate, antenna mast, E-stop button or cable-gland geometry. The BOM lists a "Control housing base 300×200" (G1) with no SCAD |

**Loader and bucket**

| # | Missing element | Notes |
|---|---|---|
| M7 | **Bobcat-style quick-attach plate** | Section header only ([main:4006-4011](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L4006-L4011)). `BOBCAT_QA_*` parameters exist but are unused. The bucket pins straight to the arm tips, so the "universal skid-steer attachment" goal isn't met |
| M8 | **Bucket structure** | No top back rail, no side-edge reinforcement, no wear strips, no gussets behind the lugs. The pivot and cylinder lugs are "bolted to bucket back plate" with **4 × 1/4" bolts** ([main:1277-1288](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L1277-L1288)), which is far too small for the bucket-cylinder force of about 94 kN per cylinder that the model itself computes (see P14) |
| M9 | **Pivot spacers / thrust washers / bushings** | For the 63.5 mm-wide arm in the 120 mm sandwich gap (see B8). Also for the bucket pivot and the cylinder clevises. No greasing provision (zerks, grease grooves) anywhere |
| M10 | **Pin retention** | Pins are drawn with nuts, but there are no retaining plates, roll pins or cotter pins on the main pivots, and no lock-collar design for the 1.5" arm pivot pin |
| M11 | **Second cross beam** | Referenced by `CROSS_BEAM_2_POS` and the second arc slot, but never drawn (B2). Decide whether it exists |
| M12 | **Hard stops** | No mechanical stops for arm-down, arm-up or bucket curl/dump. The collision rules note the bucket curl stop is a "plate-on-plate hard stop by definition" |

**Drive**

| # | Missing element | Notes |
|---|---|---|
| M13 | **UTU-specific side panels / CNC outputs** | Drive-shaft and motor holes at the UTU axes (B10). A cut list and CNC layout for the UTU bearing plates, extended tubes and track links |
| M14 | **UWU bearing/motor as real parts** | `uwu_bearing()` and `uwu_hydraulic_motor()` are generic shapes. There is no catalogue part number, and the UWU shaft coupling is a plain 2"×2" cylinder |

**Frame and operator**

| # | Missing element | Notes |
|---|---|---|
| M15 | **Operator protection, guards and handling points** | No ROPS/FOPS (probably not needed for a remote-controlled machine, but the standing platform implies an on-board operator), no guard over the drive/sprockets on the UTU, no lifting eyes, no tie-down points, no rear hitch |
| M16 | **Covers / belly pan / hood** | None |
| M17 | **Real mass properties** | CoG and weights are hand-entered (B5). A `mass_kg()` per part (volume × density) would make stability automatic |

**Tooling and outputs**

| # | Missing element | Notes |
|---|---|---|
| M18 | **BOM / cut-list export** | [documentation/BILL_OF_MATERIALS.md](../../DESIGN-STRUCTURAL/documentation/BILL_OF_MATERIALS.md) contradicts the model almost everywhere. It lists 4×4 tube frame members (the model uses 1/2" sandwich plates), 3×3 arms (the model uses 2×6), lift cylinders of 2.5"×1.25"×16" (the model uses 2.5"×1.5"×25.6"), bucket cylinders of 2"×1"×12" (the model uses 3"×1.5"×20"), 500 mm wheels (the model uses Bobcat 16.5" rims), and 2" axles (the model uses 1.25" shafts). The model should echo a machine-readable BOM (`echo("BOM", part_id, material, qty, length)`), and a script should build the markdown from it |
| M19 | **Coordinate-frame reference** (April #27) | The world frame, panel-local, arm-local, bucket-local and platform-local frames are re-derived in comments over and over. A short `COORDINATES.md` would remove most of the "thinking-out-loud" comments |
| M20 | **Colour/material palette file** (April #33) | 139 hard-coded `color()` calls |

**Part files, exports and jigs**

| # | Missing element | Notes |
|---|---|---|
| M21 | **Part files and export wrappers for plates that only exist inline in the main file** | Bucket back plate ([main:4020](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L4020)), back/bottom/front stiffener plates ([:2403](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L2403), [:2728](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L2728), [:2510](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L2510)), motor plates ([:2903](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L2903)). No export wrapper exists for `arm_plate` or `pivot_mount_plate_flat`. None of these can be cut from the repo today |
| M22 | **Mating holes** | `arm_plate` has no holes for the leg-spacer bolts; A1/A2 have no hole where the lift-cylinder through-bolt passes; T1/T2 have motor-plate holes with no mating angle iron |
| M23 | **Nesting and kerf** | No kerf compensation, sheet size, thickness grouping or overlap check (N3) |
| M24 | **Jigs that match the real parts** | No drilling jig for the 5-bolt pivot-mount pattern on the arm plates, and no A4 drilling jig with the real 4"/2" pattern. Three existing jigs are placeholders (section 5.4) |
| M25 | **Documented parts with no SCAD** | `CNCLAYOUT_PARTS_LIST.md` lists C1 arm reinforcement, C2 bucket attach and G1 control housing base. `CUSTOM_PARTS_LIST.md` lists Type B/C pivot rings (modules exist but are never called) and a QA flange plate |

---

## 7. Parts that exist but are not finished

| # | Item | What's unfinished | Where |
|---|---|---|---|
| U1 | **Arm / wheel geometry basis** | The solver avoids a 500 mm wheel at Y = 1,400 that doesn't exist (B3). It must be re-run against the real tyre and axle positions, and the clearance target enforced (B1a) | params:211-449 |
| U2 | **Bucket-cylinder geometry** | Fails its own 10° parallelism rule (5.2°, B1b). `BUCKET_CYL_MOUNT_Z_OFFSET = -117.2` is hand-tuned ("Calculated for 45 deg dump angle", but not calculated in code), and the 20" stroke is picked with "no margin" | [params:742](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L742), [:958-967](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L958-L967) |
| U3 | **Lift cylinder** | Non-catalogue 650 mm stroke with a 30 % margin whose rationale is backwards. Closed-length model disagrees with the renderer (B6). Needs a real catalogue cylinder (closed length, pin sizes, port positions) | params:540-675 |
| U4 | **Structural/stability analysis** | Rebuild against current parts (B4, B5) | main:784-1104, 3402-3527 |
| U5 | **UWU wheel end** | The tyre OD is fitted to the shaft height rather than a real tyre. A hand-tuned hub shift ("Shifted 2" outboard then 4" inboard", [main:3806-3807](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L3806-L3807)). "SIMPLIFIED RIM" ([main:3918](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L3918)). Two hub styles (`DIY`/`QD`) with no selection rationale or strength check of the 1/2" cross-bolt torque path | main:3749-3963 |
| U6 | **UTU variant** | Front-idler position is hand-tuned ("Verify visually and adjust UTU_FRONT_IDLER_FORWARD_SHIFT", [UTU:297-303](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_UTU.scad#L297-L303)). Track chain is a "simplified visualization" ([UTU:1307](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_UTU.scad#L1307)). There is no track tensioner or idler adjustment slot, even though a 41-link chain needs one. There are no holes for the drive axis (B10), the angle irons are mismatched (B9), and there is no track-link part file or cut list | lifetrac_v25_UTU.scad |
| U7 | **Loader arm spacer (T5)** | Bolt pattern is "rough placement within taper" ([loader_arm_v2.scad:166](../../DESIGN-STRUCTURAL/openscad/modules/loader_arm_v2.scad#L166)). Magic `bolt_spacing_x = 100`, `bolt_spacing_y = 80`, `spacer_len = 150`, and `extension_len = 350` unused | modules/loader_arm_v2.scad |
| U8 | **Pivot mount** | DOM length conflict (B8). Bolt angles > 360° with stale comments. Three copies of the plate geometry | params:133-203 and the three files |
| U9 | **Clevis/pin hardware** | Pins sized by ratio of rod diameter, four sizes per joint (B7). No clevis-bracket (lug) design for the lift-cylinder base: it is a "through-bolt between the wall plates" whose lateral location in the 120 mm gap, and whose spacers, aren't modelled ([main:1709-1735](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L1709-L1735)) | |
| U10 | **Bucket** | Tab-and-slot plates only. The side-plate profile uses `PLATE_1_2_INCH` as a *coordinate* ([main:4092](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad#L4092)). No QA plate (M7) and no reinforcement (M8). Lug bolting is undersized (M8) | main:4016-4188 |
| U11 | **Folding platform** | Works, but its code carries about 210 lines of unresolved working notes (C4). `PLATFORM_*` parameters include unused ones. `_crossmember_top_z = … MACHINE_HEIGHT * 0.7` in the stowed-lock assert refers to a "rear crossmember" whose real height isn't a parameter ([params:1166-1169](../../DESIGN-STRUCTURAL/openscad/lifetrac_v25_params.scad#L1166-L1169)) | |
| U12 | **Front stiffener / motor plates** | Hard-coded heights (`254.0 + PLATE_1_4_INCH`, `127.0`, `57.15`) and a "10 inch" motor plate whose height fixes the UWU shaft Z. That is the real reason the wheels don't touch the ground (B3) | main:2510-2709, 2848-3100 |
| U13 | **Mid stiffener plate** | "Removed for simplification". Decide whether the frame needs it (torsional stiffness of a 1,400 mm open sandwich frame with only two 2×6 cross tubes) and either delete the module or restore it | main:2329-2401, 3318-3341 |
| U14 | **Engine mount** | See M1 | |

| U15 | **Side panel (the main 1/2" plate)** | The CNC part lacks every hole the assembly adds (P2). The export is broken (P1). The pivot-hole edge distance is about 4 mm (P3). Legacy axle holes and a lock hole that never gets cut. Arc slots sized for a 2×2 beam (B12) | parts/side_panel.scad |
| U16 | **Structural cut-list parts** | A6 segment lengths fall back to 200/300/250 (P10). A1/A2 and A6 hole patterns don't match their plates (P10, P11). T5 spacer is empty (P12). Duplicate part numbers (A1 ≡ A2, A3 ≡ A10, A4 ≈ A5). Piece count 71 vs 77 | parts/structural/ |
| U17 | **CNC layout** | Overlaps, missing and extra parts, no kerf or sheets (N1–N3) | cnclayout.scad |
| U18 | **Jigs** | Three with broken includes, two that cut nothing, one that intersects itself (section 5.4) | 3d_printed_*_jigs/ |
| U19 | **Platform angle arm** | Wrong leg size and holes that miss the part, yet the BOM cites it as the drilling template (P9) | parts/platform_angle_arm.scad |
| U20 | **Bolt-hole bucket plates** | 2.65 mm edge ligaments. Round holes export as rectangles. Superseded by the tab-and-slot plates in the assembly, so decide which bucket is real (P4, P5) | parts/bucket_*.scad |

---

## 8. Recommended order of work

0. **Before anyone cuts steel from the repo's outputs:** mark the side-panel, rear-crossmember and bucket-side exports, `cnclayout.svg`, and the A1/A2/A6/T5 cut-list files as *not for fabrication* (e.g. a note in `DESIGN-STRUCTURAL/README.md`) until P1–P3 and P10–P12 are fixed.
1. **Make failures fail (about 1 day).**
   - Fix `dist_point_line_verify` (B1a).
   - Convert the `SAFE?`, wheel-ground, cylinder-fit and structural PASS/FAIL echoes into `assert`s behind a `STRICT` flag.
   - Add the compile-everything CI job with `--hardwarnings` (C6).
   - Fix the structural-analysis `/dev/null` capture and the `parts/` path in `generate-part-svgs.yml` (section 5.5).
   - Fix the three jig include paths, and delete the stray files (section 5.6).
   - Add a CI check that each exported SVG's bounding box matches the expected plate size.
2. **Choose the real wheel and the real cylinders (a decision, not code).** Tyre and rim, frame height, and catalogue lift/bucket cylinders with their pin sizes. Everything in B3, B6 and B7 flows from this, and the bucket-cylinder bore sets the loads in P14.
3. **Single source of truth (2–3 days).**
   - Layer the files: `params` → `derived` → `toggles`.
   - Delete the re-declarations and `is_undef` fallbacks.
   - Have the assembly, the UTU *and the structural part files* `include` `derived.scad`. That fixes the A6 fallback (P10) and similar cases.
   - Make the assembly instantiate part modules instead of re-drawing them, and give every panel feature a home in its part module, so that exports equal the assembly (C1, P2).

   This removes B2, B8, B9, B11, P2 and P10 by construction. Then fix P3 (pivot boss), P11 (A1/A2 holes) and P12 (T5 hollow), and regenerate `cnclayout.scad` from the part list with real extents and kerf (N1–N3).
4. **Re-run the solvers against the real geometry.** Arm (U1), bucket-cylinder fixed point (B1b/U2), lift cylinder (U3). Then rebuild the structural and stability analysis (B4, B5) and publish one capacity figure.
5. **Clean-up pass (1 day).** Delete the pasted working notes and commented-out code (C4), the dead modules and variables (C2), the debug echoes and library `$fn` (C3).
6. **Fill the gaps** in section 6, in this order: QA plate (M7), bucket reinforcement and lug attachment together with the bucket pivot joint (M8, P14), pivot spacers and greasing (M9–M10), power unit and hydraulic layout (M1–M4), controller enclosure (M6), UTU panel variant (M13), BOM export (M18).

---

## Appendix A: Key solver outputs at the default pose ($t = 0)

```
Arm solver:  L1 = 1422.6, L2 = 503.1, main angle -21.9 deg, ARM_MAIN_LEN 1356.5, ARM_DROP_LEN 348.0
             ARM_TIP = [1762.0, -249.3], ARM_MIN_ANGLE -27.71, ARM_MAX_ANGLE 73.9 -> LIMITED 49.45
Lift cyl:    2.5" x 1.5", stroke 650 (auto), closed 803.5 (params) / 831.45 (renderer), extended 1453.5
             pin-to-pin 830.45 (min) .. 1311.2 (max at 50 deg)
Bucket cyl:  3" x 1.5", stroke 508 (20"), closed 712.3, extended 1220.3, cross beam at 736.5 mm
             parallelism at limit 5.2 deg (rule: >= 10) -> "SAFE? NO"
Wheels:      front axle Y 1150, rear axle Y 400 (wheelbase 750), shaft Z 308.75
             rendered tyre OD 617.5, declared Bobcat tyre OD 851, solver wheel 500 @ Y 1400
Overall:     width 1667 mm (rendered UWU), MACHINE_WIDTH param 1200
Stability:   empty mass 910 kg, tipping load 661 kg, ROC (50%) 330 kg
Structural:  "RATED LIFT CAPACITY" 3305.7 kg (hydraulic); 4 FAILs (ratios 11.3 / 18.4 / 10.7 / 14.2)
UTU:         idlers Y 345.2 / 1300 at Z 185.3, drive Y 577.8 Z 552.2, 41 links x 76.2 mm, fit error 0.00 mm
```

## Appendix B: OpenSCAD scoping facts verified for this review (2021.01)

These were each checked with a minimal test file; the UTU architecture depends on them.

1. A module imported with `use <lib.scad>` reads **`lib.scad`'s own** top-level variables, including `$fn`. With `lib: X = 1` and `main: X = 2`, the module sees `1`.
2. A variable defined only in the caller is `undef` inside a used module.
3. `-D X=3` on the command line overrides both the top-level file and the used library.
4. When a variable is assigned twice in the same scope (including via `include`), the **last** value wins everywhere in that scope. That is why the main file's `CROSS_BEAM_2_POS` silently replaces the params value inside the assembly but not inside parts that only include params (B2).

## Appendix C: Hand check of the loader statics *(added 2026-09-26)*

This is a rough static check, not a substitute for the rebuilt analysis. It supports the numbers in B4, P3 and P14. The rebuilt analysis puts the load at the bucket's load centre rather than on the pin, which raises the arm moment by about 20–30 %. Its figures therefore differ a little from these.

**Inputs:**
- **Geometry:** the model's own echo output: pivot (200, 1,100), lift-cylinder base (50, 600), bracket at arm-local (828.5, −114.3), and bucket pin at arm-local (1,697.4, −249.3).
- **Hydraulics:** 3,000 psi, the relief setting in [HYDRAULIC_BOM.md](../../DESIGN-HYDRAULIC/HYDRAULIC_BOM.md). That gives 65.5 kN per 2.5" lift cylinder and 94.3 kN per 3" bucket cylinder, pushing.
- **Steel:** A36 with F_y = 250 MPa and F_u = 400 MPa (the model's constants).
- **Sections:** sharp corners, as in the model.
- **Loads:** no dynamic factor. Tear-out is the AISC nominal value, 1.2 × l_c × t × F_u.

**Lift cylinders at relief, load on the bucket pin:**

| Arm angle | Cylinder moment arm | Load moment arm: analysis / real pin | Hydraulic capacity, both cylinders: analysis / real pin | Pivot-pin force per arm | Arm moment at the lift bracket, vs the analysis's `P·L` for the same load |
|---|---|---|---|---|---|
| −27.7° (down) | 497.5 mm | 1,467 / 1,387 mm | 4,529 / 4,791 kg | 69.2 kN, pointing −19° | 15.4 vs 38.9 kN·m |
| 0° (level) | 410.2 mm | 1,657 / 1,697 mm | 3,306 / 3,227 kg | 61.5 kN, pointing +8° | 13.8 vs 26.2 kN·m |
| 45° | 198.9 mm | 1,172 / 1,377 mm | 2,267 / 1,930 kg | 58.5 kN, pointing +45° | 7.5 vs 15.7 kN·m |
| 49.4° (working limit) | 175.2 mm | 1,077 / 1,293 mm | 2,172 / 1,810 kg | 58.6 kN, pointing +49° | 6.7 vs 14.7 kN·m |

Where the model prints these values (arms down, level and 45°), the "analysis" columns reproduce them exactly. The pin force is the force the arm puts on the pin and the side panels; its direction is measured from horizontal, with forward and up positive.

**Section moduli about the bending axis:**

| Section | Modulus |
|---|---|
| 3×3×1/4 tube (what the analysis uses) | 38,180 mm³ |
| 2×6×1/4 tube | 83,045 mm³ |
| Two 1/4" × 6" side plates | 49,161 mm³ |
| Tube and plates together | 132,206 mm³ |

**Results at relief:**
- **Arm:** the bracket gusset deepens the plates for 76 mm on either side of the bracket, so the critical section is at its outboard end. With the arms down the moment there is 13.8 kN·m. That gives 104 MPa if the tube and plates share it, and 280 MPa if the plates carry it alone.
- **T3:** the two cylinders sit 134.6 mm in from the arms, so the moment is 94.3 kN × 134.6 mm = 12.7 kN·m, or 153 MPa about the 6" axis. Each lug pin is 63.5 mm below the beam axis, which adds 6.0 kN·m of torque per lug. That is about 73 MPa of shear between each lug and its arm (closed-section formula).
- **Pivot pin:** 69.2 kN over 2 × 1,140 mm² gives 30 MPa of shear.
- **Side-panel pivot hole and bucket pivot joint:** see P3 and P14.
