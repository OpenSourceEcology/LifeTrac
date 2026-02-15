# 2026-02-15 — UTU v25 Track Chain Implementation

## Summary

Created and iteratively refined the UTU (Universal Track Unit) v25 OpenSCAD model at `UTU/UTU-v25/utu_v25.scad`. The UTU is a triangular 3-axis track system designed to be interchangeable with the UWU (Universal Wheel Unit), sharing the same plate/bearing/motor mounting interfaces.

## Work Completed

### 1. UTU v25 File Creation
- Created `UTU/UTU-v25/utu_v25.scad` modeled after `UWU/UWU-v25/uwu_v25.scad` style.
- 3-axis layout: 2 bottom idlers (4 ft apart), 1 drive axis (2 ft above, centered).
- Downloaded `chrisspen/gears` library to `UTU/UTU-v25/gears.scad` (later replaced with custom sprocket).

### 2. Track Link Design
- Chainsaw-chain stagger pattern with alternating inner/outer connector plates.
- Track bars: 2.5" × 0.5" × 10" steel bars.
- Connecting rods: Ø1" steel rods.
- Connector plates with two rod holes each, circular pads at rod hole locations.
- Inner plates at ±1.125" from bar center (2" face-to-face gap).
- Outer plates at ±1.5" (1/8" gap from inner plate outer face).
- Plate stagger offsets derived parametrically from `inner_face_gap` and `stagger_gap`.

### 3. Custom Chain Sprocket
- Replaced involute gear (`spur_gear`) with custom chain sprocket.
- Pitch radius: `P / (2 × sin(180/N))` with 12 teeth.
- Rod seat cutouts at each tooth, fillets at roots using `offset(r=-r) offset(r=+r)`.
- Root circle sized to sit on track bar surface.
- Per-sprocket phase rotation computed from chain path geometry so teeth mesh correctly.

### 4. Full Parametric Chain Path
- Computes total chain loop around all 3 sprockets (straight spans + arc wraps).
- Automatically determines integer link count (`num_chain_links = round(len / pitch)`).
- Front (left) idler shifts along X (`left_idler_x`) so chain length = exact integer × pitch.
- `chain_point(s)` function returns `[x, z, angle]` for any cumulative distance along loop.
- Wrap angles, tangent contact angles, and segment boundaries all derived from axis positions.

### 5. Chord-Based Link Positioning
- Links positioned at chord midpoint between their two bounding rod positions on the pitch circle.
- Fixes rod-hole alignment on curved (sprocket) sections — rod holes land exactly on pitch circle.
- Connecting rods placed at computed pitch-circle positions.

### 6. Sprocket Phase Alignment
- Each sprocket gets a unique phase rotation based on where chain rods land on its arc.
- `sprocket_phase()` function computes rotation from chain arrival angle and cumulative distance.
- Modules parameterized: `sprocket(phase)`, `idler_axis_assembly(phase)`, `drive_axis_assembly(phase)`.

### 7. Full Parametric Cleanup
- All hardcoded magic numbers replaced with named parameters.
- New parameters: `eps`, `bearing_flange_h`, `bearing_flange_lobe_diam`, `bearing_housing_len`, `motor_shaft_diam`, `motor_flange_size`, `motor_flange_t`, `collar_gap`, `rod_hole_clearance`, `inner_face_gap`, `stagger_gap`.
- Derived values: `bearing_total_h`, `inner_plate_offset`, `outer_plate_offset` computed from base params.
- All module bodies use only named parameters — every dimension changeable from config section.

### 8. Removed Tensioner Mechanism
- Removed `plate_slotted()`, `bearing_4bolt_slotted()`, `tensioner_idler_axis_assembly()`.
- Both idlers now use identical `idler_axis_assembly()`.
- Tensioner travel absorbed by the front idler X-position adjustment in chain path computation.

### 9. Removed Track Path Outline
- Removed transparent red rectangle overlay (`track_path_outline()` module).
- Full chain visualization makes it redundant.

### 10. Documentation Updates
- Updated `UTU/README.md` with full v25 specifications.
- Updated `UWU/README.md` with UTU compatibility note.

## Key Parametric Dependencies

Changing any of these config values will automatically recompute the entire chain:

```
idler_spacing → axis positions → chain path → link count → front idler adjust → sprocket phases
drive_height_above_idlers → same cascade
chain_pitch (from track_bar_width + track_gap) → sprocket radius → all geometry
num_teeth → sprocket radius → chain path → link count
```

## Files Modified
- `UTU/UTU-v25/utu_v25.scad` — Created and refined (~690 lines)
- `UTU/UTU-v25/gears.scad` — Downloaded gear library (referenced but not actively used)
- `UTU/README.md` — Updated with full v25 specs
- `UWU/README.md` — Added UTU compatibility note
