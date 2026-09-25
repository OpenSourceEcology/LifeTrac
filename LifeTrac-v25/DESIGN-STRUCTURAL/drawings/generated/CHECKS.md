# Part drawing checks

> Generated file - do not edit. Regenerated with the drawings on every design change.

## Warnings

- none

## Known model issues (from `issues:` in parts_manifest.yaml)

These are printed on the affected drawings as CHECK BEFORE MAKING notes. Fix the model, then delete the issue from the manifest.

- **P1**: THE UWU BEARING BOLT HOLES ARE MODELLED BLIND (8.35 DEEP IN 12.7 PLATE, uwu_bearing_holes_3d USES center=true). THEY ARE BOLT HOLES: CUT THEM THROUGH.
- **P2**: THE UWU BEARING BOLT HOLES ARE MODELLED BLIND (8.35 DEEP IN 12.7 PLATE, uwu_bearing_holes_3d USES center=true). THEY ARE BOLT HOLES: CUT THEM THROUGH.
- **P3**: THE UWU BEARING BOLT HOLES ARE MODELLED BLIND (8.35 DEEP IN 12.7 PLATE, uwu_bearing_holes_3d USES center=true). THEY ARE BOLT HOLES: CUT THEM THROUGH.
- **P4**: IN THE MODEL THIS PLATE PASSES THROUGH BOTH INNER SIDE PANELS (NO SLOT OR NOTCH). CONFIRM THE FIT BEFORE CUTTING.
- **P7**: THE A6 ANGLE SEGMENTS THAT BOLT TO THIS PLATE ARE DRAWN 200 / 300 / 250 LONG (THEIR PART FILE CANNOT SEE ANGLE_SEGMENT_* IN lifetrac_v25.scad) BUT THIS PLATE'S HOLES ASSUME 146 / 546.8 / 97.6. MATCH-DRILL OR FIX THE MODEL FIRST.
- **P8**: THE 4 UWU MOTOR BOLT HOLES (12.7 ON 209.55 BCD) ARE NOT CUT IN THE MODEL: uwu_motor_mount_holes ROTATES THEM OFF THE PLATE. ONLY THE 50.8 CENTRE HOLE IS SHOWN.
- **P14**: bucket_side_plate() IS OFFSET 6.35 FROM ITS INTENDED POSITION, SO THE TAB SLOTS ARE ONLY MODELLED 1 MM DEEP. SLOTS FOR THE BACK/BOTTOM TABS MUST BE CUT THROUGH.
- **A4**: SIX A4 PLACEMENTS IN lifetrac_v25.scad USE mirror([1,1,0]), WHICH SWAPS THE WALL-LEG AND TUBE-LEG HOLE PATTERNS. CHECK THOSE JOINTS (LIKELY MEANT rotate([0,0,180])).
- **A6-1**: SEGMENT LENGTH AND HOLES COME FROM angle_iron_a6_bottom_horizontal.scad DEFAULTS (200 / 300 / 250), WHICH DO NOT MATCH THE PLATE HOLES IN lifetrac_v25.scad (146 / 546.8 / 97.6). 5 OF THE 10 RUNS ARE ALSO MIRRORED (OPPOSITE HAND). CONFIRM BEFORE CUTTING.
- **A6-2**: SEGMENT LENGTH AND HOLES COME FROM angle_iron_a6_bottom_horizontal.scad DEFAULTS (200 / 300 / 250), WHICH DO NOT MATCH THE PLATE HOLES IN lifetrac_v25.scad (146 / 546.8 / 97.6). 5 OF THE 10 RUNS ARE ALSO MIRRORED (OPPOSITE HAND). CONFIRM BEFORE CUTTING.
- **A6-3**: SEGMENT LENGTH AND HOLES COME FROM angle_iron_a6_bottom_horizontal.scad DEFAULTS (200 / 300 / 250), WHICH DO NOT MATCH THE PLATE HOLES IN lifetrac_v25.scad (146 / 546.8 / 97.6). 5 OF THE 10 RUNS ARE ALSO MIRRORED (OPPOSITE HAND). CONFIRM BEFORE CUTTING.
- **U1**: u_channel_lug() ROTATES ITS SIDE PROFILE WITH rotate([0,90,0]), SO THE LUG IS CUT TO THE TUBE SIZE (76.2) INSTEAD OF ITS LENGTH ARGUMENT (80 / 100). U2 AND U3 LOSE THEIR 4 BASE BOLT HOLES AS A RESULT. CONFIRM THE LENGTH BEFORE CUTTING.
- **U2**: u_channel_lug() ROTATES ITS SIDE PROFILE WITH rotate([0,90,0]), SO THE LUG IS CUT TO THE TUBE SIZE (76.2) INSTEAD OF ITS LENGTH ARGUMENT (80 / 100). U2 AND U3 LOSE THEIR 4 BASE BOLT HOLES AS A RESULT. CONFIRM THE LENGTH BEFORE CUTTING.
- **U3**: u_channel_lug() ROTATES ITS SIDE PROFILE WITH rotate([0,90,0]), SO THE LUG IS CUT TO THE TUBE SIZE (76.2) INSTEAD OF ITS LENGTH ARGUMENT (80 / 100). U2 AND U3 LOSE THEIR 4 BASE BOLT HOLES AS A RESULT. CONFIRM THE LENGTH BEFORE CUTTING.

## Modelled but deliberately not drawn (`ignore_markers`)

- `NUT 18.288` x8: Hydraulic cylinder rod-pin nuts - hardware supplied with the cylinders (modelled size is not a standard nut)
- `NUT 22.86` x8: Hydraulic cylinder base-pin nuts - hardware supplied with the cylinders (modelled size is not a standard nut)

## Render failures

- none

## Plate holes smaller than the plate thickness (drill after cutting)

- **P1**: 12 x Ø9.5 mm in 12.7 mm plate (A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12)
- **P2**: 12 x Ø9.5 mm in 12.7 mm plate (A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12)
- **P3**: 16 x Ø9.5 mm in 12.7 mm plate (A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16)

## Hole count by diameter in angle and tube parts (x quantity)

Rough fastener estimate: one bolt, one nut and two washers per angle-iron hole; tube holes usually share a bolt with an angle hole. Bolt length depends on the joint grip.

| Hole Ø (mm) | Hole Ø (in) | Holes per machine | Parts |
|---:|---:|---:|---|
| 9.5 | 0.374 | 170 | A1, A10, A2, A6-1, A6-2, A6-3 |
| 12.7 | 0.500 | 168 | A4, A5, T1, T2, T3, T4, T7 |
| 14.7 | 0.579 | 14 | A7, A8 |

## BOM_PART markers counted in the assembly

| Marker | Count |
|---|---:|
| `A1` | 2 |
| `A10` | 8 |
| `A2` | 4 |
| `A4` | 26 |
| `A5` | 4 |
| `A6-1` | 10 |
| `A6-2` | 10 |
| `A6-3` | 10 |
| `A7 L247.6` | 2 |
| `A8 L729.2` | 2 |
| `NUT 18.288` | 8 |
| `NUT 22.86` | 8 |
| `NUT 38.1` | 4 |
| `P1` | 1 |
| `P10` | 2 |
| `P11` | 4 |
| `P12` | 1 |
| `P13` | 1 |
| `P14` | 2 |
| `P15` | 1 |
| `P16` | 2 |
| `P17` | 4 |
| `P18` | 16 |
| `P19` | 1 |
| `P2` | 1 |
| `P3` | 2 |
| `P4` | 1 |
| `P5` | 1 |
| `P6` | 2 |
| `P7` | 1 |
| `P8` | 2 |
| `P9` | 2 |
| `PIN 15.875x32.7` | 2 |
| `PIN 19.05x106.2` | 4 |
| `PIN 25.4x106.2` | 2 |
| `PIN 25.4x160` | 2 |
| `PIN 25.4x55.4` | 2 |
| `R2` | 2 |
| `SHAFT 31.75x474` | 4 |
| `T1` | 1 |
| `T2` | 1 |
| `T3` | 1 |
| `T4` | 2 |
| `T6` | 2 |
| `T7` | 4 |
| `U-LUG 76.2x100 D21.05` | 2 |
| `U-LUG 76.2x100 D27.4` | 2 |
| `U-LUG 76.2x80 D21.05` | 2 |
