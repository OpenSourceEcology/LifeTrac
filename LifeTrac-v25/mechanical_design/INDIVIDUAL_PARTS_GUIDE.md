# Individual Part SVG Export Guide

## Overview

Each flat plate steel part in the LifeTrac v25 design has been extracted into its own .scad file with complete manufacturing details. Individual 2D SVG cutouts can be generated for each part for CNC plasma cutting.

## Quick Start

### Generate All Individual SVGs

```bash
cd LifeTrac-v25/mechanical_design
./export_individual_svgs.sh
```

SVG files will be created in `output/svg/parts/`

### Generate Single Part SVG

```bash
cd LifeTrac-v25/mechanical_design
openscad -o output/svg/parts/wheel_mount.svg --export-format=svg parts/export_wheel_mount.scad
```

## Part List

### Half-Inch (1/2" = 12.7mm) Plate Parts

| Part Name | Export File | Output SVG | Qty | Dimensions |
|-----------|-------------|------------|-----|------------|
| Side Panel Outer | `export_side_panel_outer.scad` | `side_panel_outer.svg` | 2 | ~1400×1000mm |
| Side Panel Inner | `export_side_panel_inner.scad` | `side_panel_inner.svg` | 2 | ~1400×1000mm |
| Wheel Mount | `export_wheel_mount.scad` | `wheel_mount.svg` | 4 | 250×250mm |
| Cylinder Lug | `export_cylinder_lug.scad` | `cylinder_lug.svg` | 6 | 100×150mm |
| Rear Crossmember | `export_rear_crossmember.scad` | `rear_crossmember.svg` | 1 | Variable |

### Quarter-Inch (1/4" = 6.35mm) Plate Parts

| Part Name | Export File | Output SVG | Qty | Dimensions |
|-----------|-------------|------------|-----|------------|
| Standing Deck | `export_standing_deck.scad` | `standing_deck.svg` | 1 | 700×400mm |
| Bucket Bottom | `export_bucket_bottom.scad` | `bucket_bottom.svg` | 1 | 1100×600mm |
| Bucket Side | `export_bucket_side.scad` | `bucket_side.svg` | 2 | ~450×600mm |

**Total: 23 parts from 8 unique designs**

## Part Features Included

All individual SVG exports include:

### Structural Features
- ✅ **Rounded corners** (6.35-10mm radius per design)
- ✅ **Proper dimensions** in millimeters
- ✅ **Lightening holes** where applicable (weight reduction)

### Mounting Features
- ✅ **Bolt holes** with clearances
  - 1/2" bolt holes: 14.7mm (12.7mm + 2mm clearance)
  - 3/4" bolt holes: 21.05mm (19.05mm + 2mm clearance)
  - 1" bolt holes: 27.4mm (25.4mm + 2mm clearance)
- ✅ **Corner mounting holes** for panel attachment
- ✅ **Edge mounting holes** for assembly

### Operational Features
- ✅ **Pivot holes** (38.1mm for 1.5" pins)
- ✅ **Arc slots** for cross beam clearance (inner panels only)
- ✅ **Motor mounting bolt patterns** (wheel mounts)
- ✅ **Anti-slip hole patterns** (standing deck)
- ✅ **Hydraulic cylinder pivot holes** (lugs)

## Usage Instructions

### For CNC Plasma Cutting

1. **Load SVG** into your CAM software (e.g., SheetCAM, Torchmate CAD)
2. **Verify dimensions** - all measurements in millimeters
3. **Set kerf compensation** - typically 1-2mm for plasma cutting
4. **Add lead-ins/lead-outs** - 5-10mm approach paths
5. **Set pierce delays** appropriate for material thickness
6. **Generate G-code**

### Material Specifications

#### 1/2" Plate Parts (ASTM A36 or equivalent)
- Thickness: 12.7mm
- Parts: Side panels, wheel mounts, cylinder lugs, rear crossmember
- Total area: ~11.6 m² (calculate from SVG files)

#### 1/4" Plate Parts (ASTM A36 or equivalent)
- Thickness: 6.35mm  
- Parts: Standing deck, bucket bottom, bucket sides
- Total area: ~2.5 m² (calculate from SVG files)

### Assembly Notes

#### Side Panels
- **Outer panels**: 2× identical (left/right are same, just flip)
- **Inner panels**: 2× identical with arc slots for cross beams
- Install with sandwich spacing of 120mm between inner/outer

#### Wheel Mounts
- 4× identical plates
- Positions: Front Left, Front Right, Rear Left, Rear Right
- Install with motor shaft centered through hub hole

#### Cylinder Lugs
- 6× identical lugs used for:
  - 2× Lift cylinder base mounts
  - 2× Bucket cylinder mounts
  - 2× Bucket attachment points

#### Bucket Sides
- 2× required (use same SVG, mirror for opposite side)
- Trapezoidal shape tapers from front to back

## File Structure

```
mechanical_design/
├── parts/
│   ├── side_panel.scad                    # Part definition
│   ├── export_side_panel_outer.scad       # 2D export wrapper
│   ├── export_side_panel_inner.scad       # 2D export wrapper
│   ├── wheel_mount.scad                   # Part definition
│   ├── export_wheel_mount.scad            # 2D export wrapper
│   └── ... (other parts)
├── output/
│   └── svg/
│       └── parts/
│           ├── side_panel_outer.svg       # Generated SVG
│           ├── side_panel_inner.svg       # Generated SVG
│           ├── wheel_mount.svg            # Generated SVG
│           └── ... (other SVGs)
└── export_individual_svgs.sh              # Batch export script
```

## Automated Generation

The GitHub Actions workflow `.github/workflows/generate-part-svgs.yml` automatically:
- Regenerates all individual part SVGs when part files are modified
- Commits the updated SVG files back to the repository
- Updates README with links to SVG files

## Troubleshooting

### SVG file is empty or missing features
- Ensure OpenSCAD is installed: `openscad --version`
- Check that parameters file is accessible: `openscad/lifetrac_v25_params.scad`
- Verify projection is set to `cut=true` in export file

### Dimensions are wrong
- All dimensions are in millimeters by default in OpenSCAD
- Verify your CAM software is set to metric units
- Check viewBox attribute in SVG for scale

### Holes are too small/large
- Hole sizes include 2mm clearance for bolts
- Adjust clearance in part .scad files if needed
- Remember to account for kerf in CAM software

## Contributing

When adding new plate parts:

1. Create the part file: `parts/new_part.scad`
2. Create export wrapper: `parts/export_new_part.scad`
3. Add to export script: `export_individual_svgs.sh`
4. Update this guide with part specifications
5. Test SVG generation before committing

## See Also

- `parts/README_EXPORTS.md` - Detailed export file documentation
- `cnclayout.scad` - Combined layout of all parts
- `openscad/lifetrac_v25.scad` - Full assembly model
- `openscad/lifetrac_v25_params.scad` - Shared parameters
