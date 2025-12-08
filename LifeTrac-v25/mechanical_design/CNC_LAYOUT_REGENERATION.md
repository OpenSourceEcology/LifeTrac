# CNC Layout SVG Regeneration

## Overview

The `cnclayout.svg` file is automatically regenerated whenever the source files change. This ensures the SVG always reflects the latest part designs with all manufacturing details.

## Automatic Regeneration

### GitHub Actions Workflow

The workflow `.github/workflows/generate-cnclayout-svg.yml` automatically:

1. **Triggers on changes to:**
   - `cnclayout.scad` (main layout file)
   - `parts/*.scad` (individual part files)
   - `openscad/lifetrac_v25_params.scad` (shared parameters)

2. **Regenerates:**
   - `cnclayout.svg` - Combined layout with all 23 parts
   - Updates README with current information

3. **Commits back:**
   - The updated SVG file
   - Any README changes

### What Gets Included

The regenerated SVG includes all manufacturing details from the individual part files:

#### Half-Inch (1/2") Plate Parts
- **Side panels**: Pivot holes (38.1mm), cylinder mounts, wheel axle holes, arc slots
- **Wheel mounts**: Motor shaft hole (80mm), 8-bolt pattern, corner mounts
- **Cylinder lugs**: Pivot holes (25.4mm), 4 base mounting holes
- **Rear crossmember**: Panel mounting holes, lightening holes

#### Quarter-Inch (1/4") Plate Parts
- **Standing deck**: Anti-slip hole pattern (25mm holes on 80mm grid), corner mounts
- **Bucket bottom**: Edge mounting holes, quick-attach connection holes
- **Bucket sides**: Assembly holes for panel connections

## Manual Regeneration

### Prerequisites

- OpenSCAD installed: `sudo apt-get install openscad`
- For headless systems: `sudo apt-get install xvfb`

### Command

```bash
cd LifeTrac-v25/mechanical_design

# With display
openscad --render -o cnclayout.svg cnclayout.scad

# Headless (using xvfb)
xvfb-run -a openscad --render \
  --imgsize=4096,8192 \
  --colorscheme=Tomorrow \
  --projection=ortho \
  --camera=0,0,0,0,0,0,3000 \
  -o cnclayout.svg \
  cnclayout.scad
```

### Render Options Explained

- `--render`: Full render (vs preview mode)
- `--imgsize=4096,8192`: Large image for detailed parts
- `--colorscheme=Tomorrow`: Color scheme for better visibility
- `--projection=ortho`: Orthographic projection (no perspective)
- `--camera=0,0,0,0,0,0,3000`: Top-down view from 3000mm above

## Troubleshooting

### SVG is empty or incomplete

**Problem**: SVG file exists but shows no parts or only some parts

**Solutions**:
1. Check that all part files in `parts/` directory are valid OpenSCAD files
2. Verify `openscad/lifetrac_v25_params.scad` has all required parameters
3. Look for OpenSCAD errors in the workflow logs
4. Try rendering manually to see error messages

### SVG is too large

**Problem**: SVG file is multiple megabytes and slow to load

**Solutions**:
1. This is expected - the SVG contains detailed geometry for all parts
2. For smaller files, use individual part SVGs from `output/svg/parts/`
3. Reduce `--imgsize` parameter (but may lose detail)

### Parts missing details

**Problem**: Parts show as simple outlines without holes

**Solutions**:
1. Verify individual part files contain the features (holes, slots, etc.)
2. Ensure `projection(cut=true)` is used in `cnclayout.scad`
3. Check that parts are 3D models, not 2D shapes (projection needs 3D input)

### Workflow fails in GitHub Actions

**Problem**: Workflow runs but fails to generate SVG

**Solutions**:
1. Check workflow logs for OpenSCAD errors
2. Verify all file paths are correct
3. Ensure OpenSCAD syntax is valid (test locally first)
4. Check that xvfb is properly installed in the workflow

## Architecture Notes

### Why Automatic Regeneration?

The SVG is a **rendered output** from the source SCAD files, similar to how:
- Assembly images are rendered from 3D models
- PDFs are compiled from LaTeX source
- Binaries are compiled from source code

Automatic regeneration ensures:
1. **Consistency**: SVG always matches the source
2. **Accuracy**: No manual export errors
3. **Traceability**: Git history shows what changed
4. **Convenience**: No manual steps required

### File Relationships

```
Source Files (hand-edited):
├── parts/side_panel.scad
├── parts/wheel_mount.scad
├── parts/cylinder_lug.scad
├── ... (other parts)
└── cnclayout.scad (imports and layouts parts)
    │
    ├── Uses projection(cut=true) to create 2D
    │
    └── OpenSCAD renders to:
        │
        └── cnclayout.svg (auto-generated output)
```

### Workflow Integration

```
Developer workflow:
1. Edit part file: parts/wheel_mount.scad
2. Commit and push changes
3. GitHub Actions detects change
4. Workflow runs:
   - Installs OpenSCAD
   - Renders cnclayout.svg
   - Commits updated SVG
5. SVG is now current with latest changes
```

## See Also

- `INDIVIDUAL_PARTS_GUIDE.md` - Guide for individual part SVG exports
- `parts/README_EXPORTS.md` - Export documentation
- `.github/workflows/generate-cnclayout-svg.yml` - Workflow file
- `.github/workflows/generate-part-svgs.yml` - Individual parts workflow
