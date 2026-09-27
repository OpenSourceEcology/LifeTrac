# LifeTrac v25 Structural Steel Parts Catalog

This document catalogs all structural steel parts (angle iron and tubing) used in the LifeTrac v25 assembly.

**Quantities are not listed here.** Typed counts went stale whenever the assembly changed. The drawing
generator counts every part in the assembly model instead: see [`drawings/generated/INDEX.md`](../../../drawings/generated/INDEX.md)
for the quantity per machine of each part, the totals, and the total cut length per stock size.

## Summary

| Category | Part IDs | Material |
|----------|----------|----------|
| Back Stiffener Plate Angles | A1, A2 | 2"×2"×1/4" Angle Iron |
| Front Stiffener Plate Angles | A3, A9, A10 | 2"×2"×1/4" Angle Iron |
| Frame Tube Mount Angles | A4 | 2"×2"×1/4" Angle Iron |
| Arm Crossbeam Mount Angles | A5 | 2"×2"×1/4" Angle Iron |
| Bottom Stiffener Plate Angles | A6 | 2"×2"×1/4" Angle Iron |
| Standing Platform Angles | A7, A8 | 2"×2"×1/4" Angle Iron |
| Cross Frame Tubes | T1, T2 | 2"×6"×1/4" Rect. Tube |
| Arm Crossbeam Tube | T3 | 2"×6"×1/4" Rect. Tube |
| Main Arm Tubes | T4 | 2"×6"×1/4" Rect. Tube |
| Arm Leg Spacer Tubes | T5 | 2"×6"×1/4" Rect. Tube |

---

## Angle Iron Parts (A1-A10)

All angle iron uses 2"×2"×1/4" (50.8mm × 6.35mm) standard stock.

### A1: Back Stiffener - Outer Wall Vertical
- **File:** `angle_iron_a1_back_outer_vertical.scad`
- **Location:** Back stiffener plate, left and right outer walls
- **Height:** Parametric (~600mm, calculated from `FRAME_Z_OFFSET + 350` to `FRAME_Z_OFFSET + MACHINE_HEIGHT`)
- **Holes:** 3/8" diameter bolts
  - Leg A (against wall): Multiple holes at 150mm spacing
  - Leg B (against plate): Multiple holes at 150mm spacing, offset
- **Assembly Integration:** ✅ `part_a1_back_outer_vertical()` in `back_stiffener_plate()`

### A2: Back Stiffener - Inner Wall Vertical
- **File:** `angle_iron_a2_back_inner_vertical.scad`
- **Location:** Back stiffener plate, both sides of each inner wall panel
- **Height:** Same as A1 (~600mm)
- **Holes:** Same pattern as A1
- **Assembly Integration:** ✅ `part_a2_back_inner_vertical()` in `back_stiffener_plate()`

### A3: Front Stiffener - Outer Section Vertical
- **File:** `angle_iron_a3_front_outer_vertical.scad`
- **Location:** Front stiffener plate outer sections (5" tall sections)
- **Height:** ~120.65mm (4.75") - `127mm - 2×3.175mm trim`
- **Holes:** 3/8" diameter, 2 per leg at 3" spacing
- **Assembly Integration:** Available but front_stiffener_plate uses A10 for these locations

### A4: Frame Tube Mount Angles
- **File:** `angle_iron_a4_frame_tube_mount.scad`
- **Location:** At each 2×6 frame tube, mounting to wall panels
- **Height:** ~146.05mm (5.75") - `152.4mm - 2×3.175mm trim`
- **Holes:** 1/2" diameter
  - Leg A (against wall): 2 holes at 4" (101.6mm) spacing
  - Leg B (against tube): 2 holes at 2" (50.8mm) spacing
- **Assembly Integration:** ✅ `part_a4_frame_tube_mount()` in `frame_tube_angle_iron()`, `front_stiffener_plate()`

### A5: Arm Crossbeam Mount Angles
- **File:** `angle_iron_a5_arm_crossbeam_mount.scad`
- **Location:** Connecting arm side plates to cross beam
- **Height:** ~146.05mm (5.75")
- **Holes:** Same pattern as A4
- **Assembly Integration:** ✅ `part_a5_arm_crossbeam_mount()` in `loader_arm_v2.scad`

### A6: Bottom Stiffener - Horizontal (Split into 3 Segments)
- **File:** `angle_iron_a6_bottom_horizontal.scad`
- **Location:** Bottom stiffener plate, all wall faces + motor plate faces
- **Orientation:** Horizontal along Y axis, X-leg flat on plate, Z-leg vertical against wall
- **Segments:** Split into 3 segments with 8" gaps at wheel axes
  - Segment 1 (rear): `ANGLE_SEGMENT_1_LENGTH` 
  - Segment 2 (middle): `ANGLE_SEGMENT_2_LENGTH`
  - Segment 3 (front): `ANGLE_SEGMENT_3_LENGTH`
- **Holes:** 3/8" diameter at ~150mm spacing
- **Assembly Integration:** ✅ `part_a6_split_horizontal_angle_iron()` in `bottom_stiffener_plate()`, motor plate angles

### A7: Platform Side Angle Iron
- **File:** `angle_iron_a7_platform_side.scad`
- **Location:** Folding standing platform side arms, left and right (mirrored)
- **Length:** Calculated from platform geometry (~400mm)
- **Holes:** 4 total (2 pivot bracket, 2 deck attachment)
- **Assembly Integration:** Part available, assembly uses `platform_angle_iron()` due to complex positioning

### A8: Platform Transverse Angle Iron
- **File:** `angle_iron_a8_platform_transverse.scad`
- **Location:** Platform transverse bracing (left-right across deck), front and rear
- **Length:** Platform width minus clearances
- **Holes:** 3 per piece for deck mounting
- **Assembly Integration:** Part available, assembly uses `platform_transverse_angle()` due to complex positioning

### A9: Front Stiffener - Center Section Vertical
- **File:** `angle_iron_a9_front_center.scad`
- **Location:** Front stiffener plate center section (10" tall)
- **Height:** ~146.05mm (5.75") - same as A4
- **Holes:** 1/2" diameter, 2 per leg at 4" spacing
- **Note:** Same dimensions as A4, could use A4 in assembly
- **Assembly Integration:** A4 is used in assembly at these locations

### A10: Front Stiffener - Outer Section (Motor Plate Sides)
- **File:** `angle_iron_a10_front_outer.scad`
- **Location:** Front stiffener plate outer sections (5" sections)
- **Height:** ~120.65mm (4.75")
- **Holes:** 3/8" diameter, 2 per leg at 3" spacing
- **Assembly Integration:** ✅ `part_a10_front_outer_angle()` in `front_stiffener_plate()`

---

## Tubing Parts (T1-T5)

All tubing uses 2"×6"×1/4" (50.8mm × 152.4mm × 6.35mm wall) rectangular tube.

### T1: Front Cross Frame Tube
- **File:** `tube_t1_front_frame.scad`
- **Location:** Front frame cross tube, behind front wheels
- **Length:** `FRAME_TUBE_LENGTH` (~1133mm with 1/2" extensions past outer panels)
- **Features:** Rounded corners per actual tube profile
- **Assembly Integration:** ✅ `part_t1_front_frame_tube()` in `cross_frame_tubes()`

### T2: Rear Cross Frame Tube
- **File:** `tube_t2_rear_frame.scad`
- **Location:** Rear frame cross tube, in front of rear wheels
- **Length:** Same as T1
- **Assembly Integration:** ✅ `part_t2_rear_frame_tube()` in `cross_frame_tubes()`

### T3: Arm Crossbeam Tube
- **File:** `tube_t3_arm_crossbeam.scad`
- **Location:** Connects left and right loader arms at crossbeam position
- **Length:** `ARM_SPACING` (~900mm)
- **Orientation:** 6" wide, 2" tall
- **Holes:** 4 mounting holes at each end for angle iron attachment
- **Assembly Integration:** ✅ `part_t3_arm_crossbeam()` in `loader_arms()`

### T4: Main Arm Tube
- **File:** `tube_t4_arm_main.scad`
- **Location:** Loader arm main section (pivot to elbow), one per arm
- **Length:** Calculated from arm geometry
- **Holes:** 16 total (8 rear for pivot mount, 8 front for elbow)
- **Assembly Integration:** Part available, not yet called in assembly

### T5: Arm Leg Spacer Tube
- **File:** `tube_t5_arm_leg_spacer.scad`
- **Location:** Spacer tube at elbow of each loader arm
- **Length:** ~150mm stock, plasma cut to tapered profile
- **Features:** 
  - `part_t5_arm_leg_spacer_raw()` - raw stock for ordering
  - `part_t5_arm_leg_spacer_cut()` - final tapered shape
- **Assembly Integration:** Part available, not yet called in assembly

---

## Material Requirements Summary

Generated, not typed: the **Totals** and **Stock** tables at the top of
[`drawings/generated/INDEX.md`](../../../drawings/generated/INDEX.md) give the pieces per machine and the total cut
length of 2"×2"×1/4" angle iron and 2"×6"×1/4" rectangular tube, counted from the assembly model on
every design change.

---

## Usage

### In OpenSCAD Assembly Files

```openscad
// Include the master structural parts file
use <parts/structural/structural_parts.scad>

// Then call any part module:
part_a1_back_outer_vertical();        // Back stiffener outer angle
part_a4_frame_tube_mount();           // Frame tube mount angle
part_t1_front_frame_tube();           // Front frame cross tube
part_a6_split_horizontal_angle_iron(); // All 3 bottom plate segments
```

### Individual Part Preview

Open any individual part file directly in OpenSCAD to see that specific part rendered with dimensions echoed to the console.

---

## Notes

1. All dimensions are parametric and calculated from `lifetrac_v25_params.scad`
2. Hole positions are generated by helper functions for consistent spacing
3. Bottom plate angles (A6) use split pattern with 8" gaps at wheel positions
4. Frame tube angles (A4) have matching bolt patterns for tube-to-wall mounting
5. Platform angles (A7, A8) have complex positioning - individual files available but assembly uses local modules
6. Parts T4 and T5 are defined but not yet integrated into assembly

## File Naming Convention

- Angle Iron: `angle_iron_a[N]_[description].scad`
- Tubing: `tube_t[N]_[description].scad`
- Master Include: `structural_parts.scad`

Where [N] is the part number and [description] is a brief descriptive name.
