# LifeTrac v25 Frame Tubes and Stiffener Plate Updates
**Date:** February 3, 2026

## Summary
This session focused on fixing frame tube placement, adding bolt holes for angle iron connections, widening the front stiffener plate, and reshaping the inner wall plates.

---

## Changes Made

### 1. Frame Tube Rotation Fix
**Files Modified:** `lifetrac_v25.scad`

**Problem:** Frame tubes (T1 and T2) were misoriented - the 6" and 2" dimensions were swapped.

**Solution:** Added `rotate([90, 0, 0])` to both frame tube placements in `cross_frame_tubes()` module. Also adjusted Y translation to compensate for the rotation shifting the tube position.

```scad
// Before:
translate([0, FRONT_FRAME_TUBE_Y, FRAME_TUBE_Z])
part_t1_front_frame_tube();

// After:
translate([0, FRONT_FRAME_TUBE_Y + FRAME_TUBE_WIDTH, FRAME_TUBE_Z])
rotate([90, 0, 0])
part_t1_front_frame_tube();
```

---

### 2. Arm Crossbeam (T3) Length Adjustment
**Files Modified:** 
- `tube_t3_arm_crossbeam.scad`
- `lifetrac_v25.scad`

**Initial Change:** Shortened crossbeam to be flush with outer faces of inner arm CNC plates:
```scad
PART_T3_LENGTH = ARM_SPACING - TUBE_2X6_1_4[0] - 2*ARM_PLATE_THICKNESS;  // ~837mm
```

**Final Change:** Extended crossbeam through the arm CNC plates (user requested it reach through):
```scad
PART_T3_LENGTH = ARM_SPACING - TUBE_2X6_1_4[0];  // ~849mm
```

Also updated `CROSS_BEAM_SPAN` constant in main assembly to match.

---

### 3. Frame Tube Bolt Holes (T1 and T2)
**Files Modified:**
- `tube_t1_front_frame.scad`
- `tube_t2_rear_frame.scad`
- `structural_parts.scad`

**Added bolt holes for angle iron mounts at 6 X positions:**
1. Left outer panel
2. Left inner panel
3. Left motor plate
4. Right motor plate
5. Right inner panel
6. Right outer panel

**Hole Configuration:**
- 24 holes per tube (6 positions × 2 faces × 2 holes per face)
- 1/2" diameter bolts
- Holes drilled through top and bottom Z faces (which become front/rear Y faces after rotation)
- Y positions at ~50.8mm and ~101.6mm (matching angle iron hole spacing)

**Key Fix:** Initial holes were too close to tube edges. Corrected by:
- Drilling through Z faces instead of Y faces
- Positioning holes along the 6" Y dimension (not the 2" Z dimension)

**Motor Plate Alignment Fix:** Added `plate_thickness/2` offset to motor plate hole positions to match actual angle iron placement:
```scad
_left_motor_x = -(_track_width/2 - _sandwich_spacing/2) + _motor_plate_inboard + _motor_plate_thick/2;
_right_motor_x = (_track_width/2 - _sandwich_spacing/2) - _motor_plate_inboard - _motor_plate_thick/2;
```

---

### 4. Front Stiffener Plate Widening
**Files Modified:** `lifetrac_v25.scad`

**Change:** Extended the center (10") section from outer edge of left motor plate to outer edge of right motor plate.

```scad
// Before:
translate([-MOTOR_PLATE_X, y_pos, z_start])
cube([2*MOTOR_PLATE_X, plate_thickness, center_height]);

// After:
center_half_width = MOTOR_PLATE_X + MOTOR_PLATE_THICKNESS/2;
translate([-center_half_width, y_pos, z_start])
cube([2*center_half_width, plate_thickness, center_height]);
```

The outer 5" sections were also adjusted to start at the new center section edges.

---

### 5. Inner Wall Plate Reshaping
**Files Modified:** `lifetrac_v25.scad`

**Changes to `side_panel_left_inner()` and `side_panel_right_inner()`:**

1. **Trimmed front section** beyond front stiffener plate position (`BOTTOM_PLATE_Y_END`)

2. **Created flat 5" vertical section** at front, flush with front stiffener plate:
   - Starts at `BOTTOM_PLATE_INNER_TRIM` (31.75mm)
   - Height: 127mm (5")
   - Matches outer section height of front stiffener plate

3. **Angled continuation** from top of 5" section to existing slope line:
   - Removes triangular section between flat top and original slope
   - Creates smooth transition to upper panel profile

```scad
// Key parameters:
flat_section_height = 127.0;  // 5" flat section at front
flat_section_top = BOTTOM_PLATE_INNER_TRIM + flat_section_height;
```

---

## Technical Notes

### Coordinate System Reminder
For frame tubes after `rotate([90, 0, 0])` in assembly:
- Part Y (0 to 152.4mm) → World Z (height)
- Part Z (0 to 50.8mm) → World -Y (depth)

### Part Module Pattern
- Individual part files use underscore-prefixed modules: `_part_xxx()`
- `structural_parts.scad` provides wrapper modules: `part_xxx()`
- Wrappers pass through parameters like `show_holes`

### Hole Count Summary
| Part | Holes | Description |
|------|-------|-------------|
| T1 Front Frame Tube | 24 | 6 positions × 2 faces × 2 holes |
| T2 Rear Frame Tube | 24 | 6 positions × 2 faces × 2 holes |
| T3 Arm Crossbeam | Existing | Via angle iron mounts |

---

## Files Modified
- `lifetrac_v25.scad` - Main assembly
- `parts/structural/tube_t1_front_frame.scad` - Front frame tube part
- `parts/structural/tube_t2_rear_frame.scad` - Rear frame tube part
- `parts/structural/tube_t3_arm_crossbeam.scad` - Arm crossbeam part
- `parts/structural/structural_parts.scad` - Wrapper modules

---

## Part File Details

### T1: Front Cross Frame Tube
**File:** `parts/structural/tube_t1_front_frame.scad`

**Material:** 2"×6"×1/4" Rectangular Tubing (50.8mm × 152.4mm × 6.35mm wall)

**Quantity:** 1 piece

**Dimensions:**
- Length: `TRACK_WIDTH + SANDWICH_SPACING + 2*PANEL_THICKNESS + 2*12.7mm` (extends 1/2" past outer panels)
- Width: 152.4mm (6")
- Height: 50.8mm (2")

**Bolt Holes Added:**
- 24 total holes (6 X positions × 2 faces × 2 holes per position)
- 1/2" diameter
- Drilled through top and bottom faces (Z direction in part coords)
- Y positions: ~50.8mm and ~101.6mm from edge

**X Position Calculations:**
```scad
// Panel positions
_left_outer_x = -(_track_width/2 + _sandwich_spacing/2);
_left_inner_x = -(_track_width/2 - _sandwich_spacing/2) + _panel_thick;
_right_inner_x = (_track_width/2 - _sandwich_spacing/2) - _panel_thick;
_right_outer_x = (_track_width/2 + _sandwich_spacing/2);

// Motor plate positions (with plate_thickness/2 offset)
_left_motor_x = -(_track_width/2 - _sandwich_spacing/2) + _motor_plate_inboard + _motor_plate_thick/2;
_right_motor_x = (_track_width/2 - _sandwich_spacing/2) - _motor_plate_inboard - _motor_plate_thick/2;

// Hole X positions (1" inset from face toward center)
PART_T1_HOLE_X = [
    _left_outer_x + _angle_thick + _hole_inset,
    _left_inner_x + _hole_inset,
    _left_motor_x + _hole_inset,
    _right_motor_x - _hole_inset,
    _right_inner_x - _hole_inset,
    _right_outer_x - _angle_thick - _hole_inset
];
```

**Module Signature:**
```scad
module _part_t1_front_frame_tube(show_cutaway=false, show_holes=true)
```

---

### T2: Rear Cross Frame Tube
**File:** `parts/structural/tube_t2_rear_frame.scad`

**Material:** 2"×6"×1/4" Rectangular Tubing (identical to T1)

**Quantity:** 1 piece

**Dimensions:** Same as T1

**Bolt Holes:** Same pattern as T1 (24 holes)

**Note:** Identical geometry to T1, positioned at rear wheel location in assembly.

**Module Signature:**
```scad
module _part_t2_rear_frame_tube(show_cutaway=false, show_holes=true)
```

---

### T3: Arm Crossbeam Tube
**File:** `parts/structural/tube_t3_arm_crossbeam.scad`

**Material:** 2"×6"×1/4" Rectangular Tubing

**Quantity:** 1 piece

**Length Calculation (Updated):**
```scad
// Crossbeam extends through inner arm CNC plates
_arm_spacing = is_undef(ARM_SPACING) ? 900 : ARM_SPACING;
_tube_w = is_undef(TUBE_2X6_1_4) ? 50.8 : TUBE_2X6_1_4[0];  // Arm tube width (2")

// Ends flush with inner face of inner arm CNC plates
PART_T3_LENGTH = _arm_spacing - _tube_w;  // ~849.2mm
```

**Previous Length:** `ARM_SPACING` (900mm) - was too long

**Intermediate Length:** `ARM_SPACING - TUBE_2X6_1_4[0] - 2*ARM_PLATE_THICKNESS` (~836.5mm) - was too short

**Final Length:** `ARM_SPACING - TUBE_2X6_1_4[0]` (~849.2mm) - extends through plates

**Module Signature:**
```scad
module _part_t3_arm_crossbeam(show_holes=true)
```

---

### Structural Parts Wrapper Updates
**File:** `parts/structural/structural_parts.scad`

**Updated wrapper modules to pass new parameters:**

```scad
// T1: Front Cross Frame Tube (1 pc)
use <tube_t1_front_frame.scad>
module part_t1_front_frame_tube(show_cutaway=false, show_holes=true) {
    _part_t1_front_frame_tube(show_cutaway, show_holes);
}

// T2: Rear Cross Frame Tube (1 pc)
use <tube_t2_rear_frame.scad>
module part_t2_rear_frame_tube(show_cutaway=false, show_holes=true) {
    _part_t2_rear_frame_tube(show_cutaway, show_holes);
}
```

---

## Assembly Integration

### cross_frame_tubes() Module
**Location:** `lifetrac_v25.scad`, lines ~3086-3105

```scad
module cross_frame_tubes() {
    // Front frame tube (T1)
    // Part module: length in X, origin at X=0 center, Y=0, Z=0 (bottom)
    // Assembly needs: tube at Y=FRONT_FRAME_TUBE_Y, Z=FRAME_TUBE_Z
    // Rotate to get 6" width in Y, 2" height in Z
    translate([0, FRONT_FRAME_TUBE_Y + FRAME_TUBE_WIDTH, FRAME_TUBE_Z])
    rotate([90, 0, 0])
    part_t1_front_frame_tube();
    
    // Rear frame tube (T2)
    translate([0, REAR_FRAME_TUBE_Y + FRAME_TUBE_WIDTH, FRAME_TUBE_Z])
    rotate([90, 0, 0])
    part_t2_rear_frame_tube();
    
    // Angle iron mounts (16 total: 2 per tube face × 4 wall panels × 2 tubes)
    frame_tube_angle_irons();
}
```

### CROSS_BEAM_SPAN Constant Update
```scad
// Cross beam span (extends through inner arm CNC plates)
// Reduced by arm tube width (2") only - ends flush with inner face of plates
CROSS_BEAM_SPAN = ARM_SPACING - TUBE_2X6_1_4[0];  // ~849mm
```
