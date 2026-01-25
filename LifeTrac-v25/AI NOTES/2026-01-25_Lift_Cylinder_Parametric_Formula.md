# Lift Cylinder Parametric Formula Development

**Date:** January 24-25, 2026  
**Session Focus:** Create parametric formula to auto-select appropriate lift cylinder stroke length

---

## Problem Statement

The user requested a parametric formula that would automatically choose an appropriate stroke length for the lift cylinder when machine dimensions change. Previous work had fixed undefined variable errors, but the cylinder geometry was hardcoded and didn't adapt to changes in arm length, pivot position, or other parameters.

---

## Key Challenges Encountered

### 1. OpenSCAD Variable Evaluation
OpenSCAD evaluates all variables simultaneously, not sequentially. This caused issues when trying to reference variables defined later in the file (e.g., `ARM_MAX_ANGLE_LIMITED`).

### 2. Geometry Constraints
The fundamental constraint is that a hydraulic cylinder must:
- **Fit when closed:** `closed_length ≤ pin-to-pin distance at MIN angle`
- **Reach when extended:** `extended_length ≥ pin-to-pin distance at MAX angle`

Initial attempts produced cylinders that were either too long when closed or couldn't reach full extension.

### 3. Coordinate System Confusion
The Y-axis convention (Y=0 at rear, Y increases toward front) initially caused the cylinder base to be positioned at the wrong end of the machine (forward instead of rear).

---

## Solution Approach: Geometry-First Algorithm

The final algorithm works **backwards from available space**:

1. **Set arm attachment position** (50% of arm length for adequate leverage)
2. **Set base mount position** (rear of machine, Y=50mm)
3. **Calculate pin-to-pin distances** at MIN and MAX arm angles
4. **Calculate required stroke** = (pin-to-pin MAX - pin-to-pin MIN) × 1.30 margin
5. **Round UP** to standard stroke sizes
6. **Verify fit** at both extremes

---

## Final Configuration

| Parameter | Value | Description |
|-----------|-------|-------------|
| `LIFT_CYL_BASE_Y` | 50mm | Near rear of machine |
| `LIFT_CYL_BASE_Z` | 600mm | Raised position on frame walls |
| `LIFT_CYL_ARM_OFFSET` | 828.5mm (50% of arm) | Attachment point on arm |
| `LIFT_CYLINDER_STROKE` | 650mm (25.6") | Auto-selected |
| `LIFT_CYLINDER_BORE` | 63.5mm (2.5") | Fixed |
| `LIFT_CYLINDER_ROD` | 38.1mm (1.5") | Fixed |

### Fit Verification
- **Margin at closed (MIN angle):** +27mm ✓
- **Margin at extended (MAX angle):** +144mm ✓

---

## Key Code Changes

### lifetrac_v25_params.scad (Lines ~451-600)

The parametric calculation is located in the "LIFT CYLINDER GEOMETRY CALCULATION" section:

```openscad
// Key parameters that drive the calculation:
LIFT_CYL_ARM_OFFSET = ARM_LENGTH * 0.50;  // Attachment at 50% of arm
LIFT_CYL_BASE_Y = max(50, ARM_PIVOT_Y - 150);  // Near rear
LIFT_CYL_BASE_Z = FRAME_Z_OFFSET + WHEEL_DIAMETER/2 + 200;  // Raised

// Stroke selection with 30% margin:
_stroke_margin_factor = 1.30;
_stroke_to_use = _required_stroke_from_geometry * _stroke_margin_factor;

// Cylinder overhead (welded cylinder):
_cylinder_overhead = _LIFT_CYL_BORE + 90;  // ~153.5mm for 2.5" bore
```

### lifetrac_v25.scad (Lines ~460-475)

Fixed to use `ARM_MAX_ANGLE_LIMITED` instead of `ARM_MAX_ANGLE` for verification calculations:

```openscad
_LIFT_ARM_MAX_FOR_CALC = is_undef(ARM_MAX_ANGLE_LIMITED) ? ARM_MAX_ANGLE : ARM_MAX_ANGLE_LIMITED;
_LIFT_PIN_TO_PIN_AT_MAX = lift_cyl_length(_LIFT_ARM_MAX_FOR_CALC, ...);
```

---

## Design Trade-offs

### Arm Attachment Position (50% of arm length)
- **Higher %** = More mechanical advantage, but requires longer cylinder
- **Lower %** = Shorter cylinder, but less leverage (higher hydraulic pressure needed)
- **50% chosen** for balance between leverage and fitting geometry with rear-mounted base

### Base Position (Y=50mm, Z=600mm)
- **Y position:** Near rear to be between side walls, behind arm pivot
- **Z position:** Raised to 600mm to increase pin-to-pin distance at MIN angle

### Stroke Margin (30%)
- Accounts for manufacturing tolerances
- Provides cushion for `ARM_MAX_ANGLE_LIMITED` being lower than `ARM_MAX_ANGLE`
- Ensures cylinder reaches full extension even with slight geometry variations

---

## Cylinder Physical Length Calculation

For a welded hydraulic cylinder:
```
closed_length = stroke + overhead
overhead = bore + 90mm ≈ 153.5mm for 2.5" bore
extended_length = closed_length + stroke
```

With 650mm stroke:
- Closed length: 650 + 153.5 = 803.5mm
- Extended length: 803.5 + 650 = 1453.5mm

---

## Verification Output

```
=== LIFT CYLINDER PARAMETRIC STROKE CALCULATION ===
Pin-to-pin at MIN: 830.45 mm
Pin-to-pin at MAX: 1311.19 mm
Available geometric stroke: 480.74 mm
Stroke with 30% margin: 625 mm
AUTO-SELECTED STROKE: 650 mm (25.6 inches)
Margin at min (closed fit): +26.95 mm ✓ OK
Margin at max (extended fit): +142.32 mm ✓ OK
```

---

## Files Modified

1. **lifetrac_v25_params.scad**
   - Complete rewrite of lift cylinder geometry calculation (lines 451-600)
   - Added geometry-first approach with automatic stroke selection
   - Fixed coordinate system usage (rear = low Y)

2. **lifetrac_v25.scad**
   - Updated verification calculations to use `ARM_MAX_ANGLE_LIMITED`
   - Fixed warning messages to reflect correct margins

---

## Future Considerations

1. **ARM_MAX_ANGLE_LIMITED Dependency:** The lift cylinder calculation uses a conservative estimate (50°) since `ARM_MAX_ANGLE_LIMITED` is calculated later in the params file. A more elegant solution would be to reorganize the file to calculate the parallelism limit first.

2. **Cylinder Overhead:** The 90mm overhead constant is based on typical welded cylinder dimensions. This could be made configurable for different cylinder types.

3. **Rear Wheel Placement Warning:** The current configuration still shows a warning about rear wheel placement. This is a separate frame layout issue unrelated to the lift cylinder.

---

## Summary

The parametric lift cylinder formula now automatically selects an appropriate stroke length based on:
- Arm length and attachment position
- Arm pivot location
- Base mount position (constrained to rear of machine)
- Arm angle range (MIN to MAX LIMITED)

When machine dimensions change, the formula recalculates and selects the nearest standard stroke size that ensures the cylinder fits when closed and reaches when extended.
