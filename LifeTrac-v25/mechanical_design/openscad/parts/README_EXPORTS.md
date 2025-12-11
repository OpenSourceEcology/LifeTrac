# Individual Part SVG Exports

This directory contains export scripts for generating individual 2D SVG cutouts of each plate part.

## Export Files

Each part has a corresponding `export_*.scad` file that generates a 2D projection for CNC cutting:

### Half-Inch (1/2") Plate Parts

1. **export_side_panel_outer.scad** → `side_panel_outer.svg`
   - Outer side panel with pivot holes, wheel axle holes
   - Dimensions: ~1400mm × 1000mm
   - Quantity needed: 2 (left and right)

2. **export_side_panel_inner.scad** → `side_panel_inner.svg`
   - Inner side panel with arc slots for cross beams
   - Includes pivot holes, cylinder mounts, wheel axle holes
   - Dimensions: ~1400mm × 1000mm
   - Quantity needed: 2 (left and right)

3. **export_wheel_mount.scad** → `wheel_mount.svg`
   - Wheel mounting plate with motor bolt pattern
   - Includes center hub hole and corner mounting holes
   - Dimensions: 250mm × 250mm
   - Quantity needed: 4 (FL, FR, RL, RR)

4. **export_cylinder_lug.scad** → `cylinder_lug.svg`
   - Hydraulic cylinder mounting lug
   - Includes pivot hole and base mounting holes
   - Dimensions: 100mm × 150mm
   - Quantity needed: 6 (2× lift cylinder, 2× bucket cylinder, 2× bucket attach)

5. **export_rear_crossmember.scad** → `rear_crossmember.svg`
   - Rear structural crossmember
   - Includes mounting holes and lightening holes
   - Dimensions: Variable height × full width
   - Quantity needed: 1

### Quarter-Inch (1/4") Plate Parts

6. **export_standing_deck.scad** → `standing_deck.svg`
   - Operator standing platform
   - Includes anti-slip hole pattern and corner mounts
   - Dimensions: 700mm × 400mm
   - Quantity needed: 1

7. **export_bucket_bottom.scad** → `bucket_bottom.svg`
   - Bucket bottom plate
   - Includes edge mounting holes
   - Dimensions: 1100mm × 600mm
   - Quantity needed: 1

8. **export_bucket_side.scad** → `bucket_side.svg`
   - Bucket side plate (trapezoidal)
   - Includes mounting holes
   - Dimensions: ~450mm × 600mm
   - Quantity needed: 2 (left and right, mirror for opposite side)

## Generating SVG Files

### Manual Export (Individual Part)

To export a single part:

```bash
cd mechanical_design
openscad -o output/svg/parts/part_name.svg --export-format=svg parts/export_part_name.scad
```

### Batch Export (All Parts)

To export all parts at once:

```bash
cd mechanical_design
./export_individual_svgs.sh
```

This will create SVG files in `output/svg/parts/` directory.

## Using the SVG Files

### For CNC Plasma Cutting

1. Open the SVG file in your CAM software
2. Verify dimensions (should be in millimeters)
3. Set appropriate kerf compensation for plasma cutting
4. Generate G-code with proper lead-ins/lead-outs

### For Documentation

The SVG files can be viewed in web browsers or vector graphics software for:
- Manufacturing documentation
- Assembly instructions
- Part verification
- Quality control

## Part Features

All exported SVGs include:
- **Mounting holes** with proper clearances
- **Pivot holes** for arm and cylinder connections
- **Bolt patterns** for motor and structural mounting
- **Lightening holes** where applicable
- **Arc slots** for moving parts clearance (inner panels)
- **Rounded corners** per design specifications

## Notes

- All dimensions are in millimeters
- Hole sizes include clearance for bolts
- Parts are oriented flat for CNC cutting
- Some parts need to be mirrored for left/right pairs
- Material thickness is embedded in part design but not in SVG (note separately)
