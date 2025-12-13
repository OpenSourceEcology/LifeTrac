# CNC Layout Changes

## Overview
The `cnclayout.scad` file has been refactored to arrange parts in a left-to-right layout optimized for direct CNC machine use.

## Changes Made

### 1. Layout Arrangement: Left-to-Right
**Previous**: Parts were arranged in multiple rows (top to bottom)
**New**: All parts are arranged in a single horizontal line from left to right

This makes it easier for CNC operators to:
- Follow the cutting sequence
- Understand the material flow
- Optimize sheet placement

### 2. Text Labels Removed
**Previous**: Each part had text labels for identification
**New**: All text elements have been removed

Benefits:
- Output is ready for direct import into CNC software
- No manual cleanup needed
- Prevents text from interfering with cutting paths
- Cleaner DXF/SVG exports

### 3. Parametric Spacing
**Previous**: Some spacing was hard-coded
**New**: All spacing is calculated parametrically based on part dimensions

The layout now:
- Defines dimensions for each part type
- Calculates X positions based on actual part widths
- Uses the `SPACING` parameter (20mm) consistently between all parts
- Automatically adjusts if part dimensions change in the future

### 4. No Overlapping Parts
Each part's position is calculated by adding:
- Previous part's X position
- Previous part's width
- Spacing constant (20mm)

This ensures no parts overlap, even if dimensions are changed in the parameter file.

## Part Order (Left to Right)

1. Side Panels (4x) - Largest parts, rotated 90°
2. Rear Crossmember
3. Bucket Bottom
4. Standing Deck
5. Bucket Sides (2x) - Rotated 90°
6. Wheel Mounts (4x)
7. Cylinder Lugs (6x)

Total: 19 parts in a single horizontal line

## Layout Dimensions

- **Total Width**: ~9.9 meters
- **Total Height**: ~1.4 meters
- **Spacing**: 20mm between all parts
- **Start Position**: (10, 10)

## Usage

### Generate SVG for CNC
```bash
cd LifeTrac-v25/mechanical_design/openscad
openscad --export-format=svg -o output/cnclayout.svg cnclayout.scad
```

### Generate DXF for CNC
```bash
openscad --export-format=dxf -o output/cnclayout.dxf cnclayout.scad
```

### Adjust Spacing
To change the spacing between parts, edit the `SPACING` parameter in `cnclayout.scad`:
```openscad
SPACING = 20; // mm between parts - change this value as needed
```

## Technical Details

### Parametric Dimensions
All part dimensions are derived from the main parameter file (`lifetrac_v25_params.scad`):
- `MACHINE_HEIGHT` - Used for side panel dimensions
- `WHEEL_BASE` - Used for side panel dimensions
- `BUCKET_WIDTH`, `BUCKET_DEPTH` - Used for bucket parts
- `DECK_WIDTH`, `DECK_DEPTH` - Used for standing deck
- etc.

### Position Calculation
Positions are calculated sequentially:
```openscad
x_pos_0 = START_X;
x_pos_1 = x_pos_0 + side_panel_width_rotated + SPACING;
x_pos_2 = x_pos_1 + side_panel_width_rotated + SPACING;
// ... and so on
```

This approach ensures:
- No variable reassignment (OpenSCAD compatible)
- Clear dependency chain
- Easy to understand and modify
- Automatic adjustment when dimensions change

## Compatibility

The layout is compatible with:
- OpenSCAD 2019.05 and later
- CNC plasma cutters
- Laser cutters
- Waterjet cutters
- Any CNC software that accepts SVG or DXF input

## Future Enhancements

Possible future improvements:
- Add optional material optimization (nesting)
- Support for multiple sheet layouts
- Automatic part labeling layer (toggle-able)
- Kerf compensation parameters
