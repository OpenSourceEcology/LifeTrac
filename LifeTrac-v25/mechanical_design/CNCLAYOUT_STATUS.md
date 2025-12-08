# CNC Layout Status and Requirements

## Current State

The current cnclayout.svg generates **simple outlines only** - no holes, cutouts, or mounting features.

### What's Missing

Each part needs detailed features:

1. **Triangular Side Panels (A1-x)**
   - Arm pivot hole (38.1mm / 1.5" dia)
   - Lift cylinder mounting holes
   - Wheel axle mounting holes
   - Cross beam attachment holes
   - Lightening holes (weight reduction)

2. **Wheel Mount Plates (A4-x)**
   - Center hub hole for hydraulic motor
   - Bolt pattern for motor mounting
   - Holes for attaching to side panels

3. **Hydraulic Cylinder Lugs (A5-x, Bucket Cyl Lugs)**
   - Center pivot hole for cylinder pin
   - Mounting holes for bolting to frame/arm

4. **Bucket Attachment Plates (C2-x)**
   - Quick-attach pin holes (25.4mm / 1" dia)
   - Bucket cylinder mounting holes
   - Attachment holes for connecting to loader arm

5. **Rear Crossmember (A2)**
   - Holes for bolting to side panels
   - Lift cylinder base mounting holes
   - Possible lightening holes

6. **Arm Reinforcements (C1-x)**
   - Holes along length for bolting to square tube
   - Lightening holes

7. **Bucket Panels (E1-x)**
   - Bolt holes for assembly
   - Wear plate attachment holes (if applicable)
   - Corner holes for connecting panels

8. **Standing Deck (F1)**
   - Anti-slip hole pattern (already specified in code)
   - Edge mounting holes

9. **Housing Base (G1)**
   - Mounting holes for control housing

## Technical Approach

To generate proper CNC layouts (following CEB-Press pattern):

### 1. Create Individual Part Modules

For each part, create a dedicated .scad file (e.g., `parts/a1_side_panel.scad`) with:
```openscad
module a1_side_panel() {
    difference() {
        // Base triangular shape with rounded corners
        offset(r=6.35)
        offset(r=-6.35)
        linear_extrude(height=PLATE_1_2)
        polygon([...]);
        
        // Arm pivot hole at calculated position
        translate([ARM_PIVOT_Y, ARM_PIVOT_Z_HEIGHT, -1])
        cylinder(d=PIVOT_PIN_DIA, h=PLATE_1_2+2);
        
        // Lift cylinder mounting holes
        translate([LIFT_CYL_BASE_Y, LIFT_CYL_BASE_Z, -1])
        cylinder(d=BOLT_DIA_3_4, h=PLATE_1_2+2);
        
        // Additional holes...
    }
}
```

### 2. Update cnclayout.scad

Use projection to create 2D cutting layouts:
```openscad
// Import all part modules
use <parts/a1_side_panel.scad>
use <parts/a4_wheel_mount.scad>
// etc...

// Layout with projection
projection(cut=true)
translate([x, y, 0])
rotate([0, 0, 90])  // Orient for cutting
a1_side_panel();
```

### 3. Required Specifications

To implement properly, need:
- Exact hole positions relative to part edges
- Bolt patterns (number of holes, spacing, circle diameter)
- Hole sizes for different bolt grades
- Clearance hole specifications vs tapped holes
- Weld tab positions and sizes

## Dimensional Information Available

From lifetrac_v25.scad, we have:
- Pivot pin diameter: 38.1mm (1.5")
- Bolt sizes: 12.7mm (1/2"), 19.05mm (3/4"), 25.4mm (1")
- Arm pivot position: Y=200mm, Z=950mm
- Lift cylinder base: Y=700mm, Z=550mm
- Cylinder mounting positions (calculated)
- Overall part dimensions

## Missing Specifications

Need to determine/specify:
- Wheel mount bolt patterns
- Cross beam attachment hole spacing
- Number and position of lightening holes
- Weld tab dimensions
- Edge distances for holes (minimum from edge)
- Hole patterns for panel-to-panel connections

## Next Steps

1. Prioritize which parts need details first (suggest wheel mounts and cylinder lugs as highest priority)
2. Create detailed specifications for hole patterns
3. Implement part modules with full details
4. Update cnclayout.scad to use projection of detailed parts
5. Regenerate SVG with all features

## References

- CEB-Press example: https://github.com/OpenSourceEcology/CEB-Press/blob/master/v2509P/
- Current dimensions: `openscad/lifetrac_v25.scad`
- Module library: `modules/plate_steel.scad`
