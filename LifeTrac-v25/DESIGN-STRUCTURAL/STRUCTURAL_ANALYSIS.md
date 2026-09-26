# LifeTrac v25 Structural Analysis

The loader's static structural and stability checks live in [`openscad/analysis/structural_analysis.scad`](openscad/analysis/structural_analysis.scad). `lifetrac_v25.scad` includes that file at its end, so the checks always use the geometry the model draws.

The **OpenSCAD Structural Analysis** workflow runs them on every change to a `.scad` file. It writes the results to [STRUCTURAL_ANALYSIS_LOG.md](STRUCTURAL_ANALYSIS_LOG.md) and to a comment on the pull request.

This analysis replaced an earlier one on 2026-09-26. The earlier one checked a straight 3"×3" arm and a 2"×2" cross beam that are no longer in the design. It also applied its safety factor twice and treated the pinned arm as a cantilever. See finding B4 of the [2026-09-25 OpenSCAD review](../AI%20NOTES/CODE%20REVIEWS/2026-09-25_v25_OpenSCAD_Full_Review_Claude_v1_0.md).

## Load cases

Both cases are static and use the hydraulic relief pressure, `HYDRAULIC_PRESSURE_PSI` (3,000 psi, the relief setting in [HYDRAULIC_BOM.md](../DESIGN-HYDRAULIC/HYDRAULIC_BOM.md)). A loader reaches relief pressure in normal use, whenever a cylinder stalls against the ground or a load. So these forces are treated as working loads, with the AISC allowable-stress factors below and no extra dynamic factor.

| Case | What happens | What it loads |
|---|---|---|
| **Lift** | Both lift cylinders push at relief. The load sits at the bucket's load centre: half the bucket depth ahead of its back plate and 40 % of its height above the floor, with the bucket level. The case is repeated at 9 arm angles from arms down to the bucket-cylinder limit. | The arm, which hangs on its pivot pin and is propped by the lift cylinder. The pivot pin, which takes the cylinder's push as well as the load. The pivot mount, the side-panel pivot holes and the lift-cylinder pins and holes. |
| **Bucket** | Both bucket cylinders push or pull at relief, stalled by an obstacle that holds the bucket's cutting edge: a dump pressed against something, or a breakout when curling. The case is repeated at 145 bucket tilts, from full dump with the arms raised to full curl with the arms down; the report's table shows 9 of them. | The bucket pivot pins, the arm tips and the bucket's pivot lugs, which carry the cylinder's force plus the obstacle's force on the edge. The arm ahead of T3, which carries the same force. The cross beam T3, which carries both cylinders on lugs hung below it. The cylinders' pins and lug bolts. |

**The obstacle's force** is the smallest force at the cutting edge that stops the bucket turning. So it acts at right angles to the line from the bucket pin to the edge. The bucket pin then carries more than the cylinder's force: the two add up.

A force with a part along that line, such as the machine driving the edge into a pile, adds to the pin force and isn't checked. The machine's traction limits it. When the edge is simply pressed down onto the ground, the front of the machine lifts long before the cylinders reach relief. So the dump case is an upper bound, and the breakout when curling is the usual design case for a loader's bucket joint and arms. The report gives both.

## Criteria

The criteria follow AISC 360 allowable-stress design (ASD):

| Failure mode | Allowable |
|---|---|
| Bending | 0.6 F_y |
| Pin shear | 0.4 F_y of the pin steel |
| Fillet welds | 0.3 F_EXX (AISC's ASD value) |
| Bolts | R_n / 2 |
| Bolts in tension and shear together | (T / T_a)² + (V / V_a)² ≤ 1, the elliptical interaction in the AISC commentary to J3.7 |
| Pin holes | R_n / 2, where R_n is the lower of tear-out (1.2 l_c t F_u) and bearing (2.4 d t F_u), with l_c measured to the nearest edge whatever the load direction |

**Materials:**
- Plate, tube and angle: A36 (F_y 250 MPa, F_u 400 MPa).
- Pins: AISI 1045 cold drawn (F_y 530 MPa). This is an assumption until the pins are specified.
- Bolts: SAE Grade 5.
- Welds: E70 electrode.

## Checks

| Check | Covers |
|---|---|
| `PIVOT_PIN_SHEAR` | 1.5" arm pivot pin in double shear |
| `PIVOT_HOLE_SIDE_PANELS` | Arm pivot hole in the two 1/2" side panels |
| `PIVOT_MOUNT_BOLTS`, `PIVOT_MOUNT_WELDS` | The bolted pivot mount: 1/2" bolts, and DOM-to-plate fillet welds |
| `ARM_BENDING` | Arm just outboard of the lift-bracket gusset, with the 2×6 tube and both side plates |
| `ARM_BENDING_PLATES_ONLY` | The same section with the side plates alone, because the tube is bolted to them only near its ends |
| `ARM_BENDING_BUCKET` | The same section, tube and plates together, in the bucket case. T3 is behind the section, so the section carries the bucket pin's whole force. The moment keeps rising toward T3, but the gusset deepens the arm there |
| `LIFT_CYL_PINS`, `LIFT_BRACKET_HOLE`, `LIFT_BASE_HOLE` | Lift-cylinder pins, and their holes in the arm plates and the side panels |
| `T3_COMBINED` | Cross beam T3 in bending and twist (von Mises) |
| `BUCKET_CYL_PINS`, `BUCKET_CYL_LUG_BOLTS`, `BUCKET_CYL_LUG_BOLTS_BUCKET` | Bucket-cylinder pins, and the 1/4" bolts that hold their lugs to T3 (shear) and to the bucket (tension and shear) |
| `BUCKET_PIN_SHEAR`, `BUCKET_PIN_ARM_TIP`, `BUCKET_PIVOT_LUG`, `BUCKET_LUG_BOLTS` | The bucket pivot joint: the 1" pin, the arm tip, the bucket's U-lug and its bolts (tension and shear) |

**Lug bolts on the bucket.** Each U-lug is bolted to the bucket's back plate through its base. The part of the force on the lug that pulls it off the plate puts the bolts in tension; the part along the plate puts them in shear. The force's direction changes as the bucket tilts, so the check takes the tilt and stroke with the highest interaction ratio. For these two checks the report's capacity is the bolt group's strength in the direction of that force.

## Rated operating capacity

The rated operating capacity is 50 % of the tipping load, the usual rating for skid-steer loaders (ISO 14397-1). It is capped at the hydraulic lift capacity.

**Tipping load:**
- It is the load at the load centre that lifts the rear wheels, taking moments about the front axle.
- It is computed at each of the 9 arm angles, and the lowest value counts.
- The operator isn't counted, because the machine is normally remote-controlled. The report also gives the value with an operator on the rear platform.

**Hydraulic capacity:**
- It is the load both lift cylinders hold at relief, after carrying the arms and the bucket.

**Masses** are estimates carried over from the earlier stability check:

| Item | Mass |
|---|---|
| Frame | 350 kg |
| Wheel units | 4 × 40 kg |
| Arms, cross beam and cylinders | 180 kg |
| Bucket | 120 kg |
| Fluids | 40 kg |
| Engine | `ENGINE_WEIGHT_KG` |

Computing each part's mass from its geometry (review finding M17) would make this figure much more reliable.

## Known issues and CI

Some checks fail on the current design and need a design decision, such as the bucket pivot joint (review finding P14). These are listed in `STRUCT_KNOWN_ISSUES`, each with the review finding that covers it, and the report marks them "known fail".

The workflow fails if any other check fails. It checks before it commits the log, so a failing result is never committed as a log update. It warns when a listed check starts to pass, so the list stays current. When a design change fixes a known issue, remove it from the list in the same change.

**To add a check,** append a row to `AN_CHECKS`: `[id, description, demand, capacity, unit]`. Forces are in N and reported in kN; stresses are in MPa.

## Limitations

- **Static checks only.** There is no fatigue, impact or dynamic factor beyond treating relief pressure as a working load.
- **Symmetric loads only.** A load on one side of the bucket, which twists the arms and T3, isn't checked.
- **The obstacle's force in the bucket case** is the smallest one that stalls the cylinder (see Load cases). The bucket's own weight is left out; it is small beside the cylinders' force.
- **Simplified connections.** T3's bolted angle-clip end connections aren't modelled. The tear-out checks use the nearest edge, which is conservative when the load points elsewhere. Prying of the lugs' 1/4" bases, which adds bolt tension, isn't included.
- **Estimated masses.** The rated operating capacity depends directly on them.
- **Uncovered parts.** The frame, the side panels as plates, the bucket's own plates, the UWU wheel units and the platform aren't checked.

## References

- AISC 360-16, *Specification for Structural Steel Buildings*: chapters F (bending), J2 (welds), J3 (bolts, bearing and tear-out)
- ISO 14397-1, *Earth-moving machinery — Loaders and backhoe loaders — Part 1: Calculation of rated operating capacity and test method for verifying calculated tipping load*
- [2026-09-25 OpenSCAD review](../AI%20NOTES/CODE%20REVIEWS/2026-09-25_v25_OpenSCAD_Full_Review_Claude_v1_0.md): findings B4, B5, P3, P14 and Appendix C
