# 2026-02-02 UWU Integration Summary

## Overview
This session integrated the Universal Wheel Unit (UWU) as an optional modular wheel system for LifeTrac v25. The UWU can be swapped with the UTU (Universal Track Unit) for different terrain requirements.

## Components Added

### UWU Assembly Components
- **Hydraulic Motor**: 6" diameter, 5" length motor body, mounts to motor mount plate
- **Bearing Housings**: Two flange-mounted bearings with 4-bolt pattern (BCD 157.23mm / 6.19")
- **Main Shaft**: 1.25" (31.75mm) diameter shaft connecting motor to wheel
- **Shaft Coupling**: Between motor shaft and main shaft
- **Wheel**: Parametric wheel with hub, tire (torus), and disc

### Mounting Configuration
- **Motor**: Mounts to existing motor mount plate (6" inboard from inner wall)
- **Inner Bearing**: Mounts to inner face of inner side wall panel, housing extends toward motor
- **Outer Bearing**: Mounts to outer face of outer side wall panel, housing extends outward
- **Shaft Axis Height**: Centered at 5" (127mm) up from bottom of motor plate (halfway up 10" plate)

## Key Parameters (lifetrac_v25_params.scad)
```
UWU_SHAFT_DIAM = 31.75;              // 1.25" shaft diameter
UWU_BOLT_CIRCLE_DIAM = 157.23;       // 6.19" bearing BCD
UWU_MOTOR_BOLT_CIRCLE_DIAM = 209.55; // 8.25" motor BCD  
UWU_BOLT_HOLE_DIAM = 12.7;           // 1/2" bolt holes
UWU_SHAFT_AXIS_HEIGHT_ON_PLATE = 127; // 5" up from plate bottom
```

## Geometry Calculations (lifetrac_v25.scad)
```
UWU_SHAFT_Z = FRAME_Z_OFFSET + BOTTOM_PLATE_INNER_TRIM + UWU_SHAFT_AXIS_HEIGHT_ON_PLATE
INNER_WALL_PANEL_X = TRACK_WIDTH/2 - SANDWICH_SPACING/2
UWU_MOTOR_TO_WALL_DIST = INNER_WALL_PANEL_X - MOTOR_PLATE_X

motor_to_outer_wall_face = motor_to_inner_wall + PANEL_THICKNESS + SANDWICH_SPACING + PANEL_THICKNESS
wheel_to_wall_clearance = 50.8  // 2" from outer wall to wheel inside edge
shaft_total_length = motor_to_outer_wall_face + wheel_to_wall_clearance + WHEEL_WIDTH/2
```

## Side Panel Modifications
- Added `uwu_bearing_holes_2d()` and `uwu_bearing_holes_3d()` for bearing mounting holes
- Added bearing hole cutouts to all four side wall panels (left/right inner/outer)
- Added motor mount hole cutouts to motor mount plates via `uwu_motor_mount_holes()`

## Display Toggle
```
show_uwu = true;  // Toggle UWU wheel assemblies vs traditional wheels
```

## Issues Resolved During Session
1. **Wheel collision with side wall**: Adjusted shaft length calculation to move wheels outward
2. **Missing outer bearing**: Added second bearing on outer side wall panel
3. **Bearing not flush**: Fixed positioning to eliminate 1/2" gap from panel faces
4. **Inner bearing orientation**: Flipped inner bearing 180° so housing extends toward motor (keeps sandwich gap clear)
5. **Wheel clearance**: Set final wheel position to 2" from outer wall panel

## Remaining TODO
- [ ] Add top deck plate above motor mount plate
- [ ] Finish wheel and tire design with actual dimensions
- [ ] Consider adding UTU (track) module as alternative to UWU

## Files Modified
- `lifetrac_v25_params.scad`: Added UWU parameters
- `lifetrac_v25.scad`: Added UWU modules, bearing holes, motor mount holes, display toggle
