# Pivot Mount Assembly Design Session
**Date:** January 25, 2025  
**Session Focus:** Replaceable Pivot Mount Assembly for Loader Arm

---

## Overview

Designed and implemented a replaceable pivot mount assembly that allows the main arm pivot to be serviced without welding the arm plates. The assembly slides into the arm from below through a slot and bolts to both arm plates.

---

## Components Created/Modified

### New Files
- `parts/pivot_mount_assembly.scad` - Complete welded assembly definition
- `3d_printed_welding_jigs/pivot_mount_welding_jig.scad` - Welding jig for fabrication

### Modified Files
- `lifetrac_v25_params.scad` - Added pivot mount parameters
- `parts/arm_plate.scad` - Added DOM slot cutout, semicircular end, and bolt holes
- `modules/loader_arm_v2.scad` - Added assembly visualization with weld indicators

---

## Design Specifications

### Pivot Mount Assembly
| Parameter | Value | Description |
|-----------|-------|-------------|
| Plate Diameter | 152.4mm (6") | Circular mounting plates |
| Plate Thickness | 6.35mm (1/4") | Standard plate stock |
| DOM Tube OD | 50.8mm (2") | Drawn Over Mandrel tubing |
| DOM Tube ID | 38.1mm (1.5") | Fits pivot pin |
| Bolt Count | 5 | Per plate |
| Bolt Circle Diameter | 114.3mm (4.5") | Bolt pattern |
| Bolt Diameter | 12.7mm (1/2") | Mounting bolts |

### Non-Uniform Bolt Spacing
The bolt pattern uses **non-uniform spacing** to provide clearance around the DOM insertion slot at the bottom of the arm (270°):

| Parameter | Value | Description |
|-----------|-------|-------------|
| Slot Angle | 270° | Bottom of arm plate |
| Bolt Slot Clearance | 127mm (5") | Total arc clearance across slot |
| Slot Gap Angle | ~127.3° | Angular gap around slot |
| Regular Gap | ~58.2° | Spacing between other bolts |

**Bolt Positions (5 bolts):**
- Bolt 0: ~333.7° (lower-right, flanking slot)
- Bolt 1: ~391.8° (31.8°, right side)
- Bolt 2: ~450° (90°, top)
- Bolt 3: ~508.2° (148.2°, left side)  
- Bolt 4: ~566.3° (206.3°, lower-left, flanking slot)

### Weld Specifications
| Parameter | Value | Description |
|-----------|-------|-------------|
| Weld Bead Diameter | 6.35mm (1/4") | Torus cross-section |
| Weld OD | ~57.15mm | DOM OD + weld diameter |
| Weld Count | 4 per plate | Inside and outside faces |
| Total Welds | 8 | 4 welds × 2 plates |

### Slot Cutout
| Parameter | Value | Description |
|-----------|-------|-------------|
| Slot Diameter | ~63.5mm | Weld OD + 1/4" clearance |
| Slot Clearance | 6.35mm (1/4") | Around weld bead |

---

## Key Parameters Added to `lifetrac_v25_params.scad`

```openscad
// Pivot Mount Assembly Parameters
PIVOT_MOUNT_PLATE_DIA = 152.4;          // 6" diameter plates
PIVOT_MOUNT_BOLT_COUNT = 5;             // Bolts per plate
PIVOT_MOUNT_BOLT_CIRCLE_DIA = 114.3;    // 4.5" bolt circle
PIVOT_MOUNT_SLOT_ANGLE = 270;           // Slot at bottom
PIVOT_MOUNT_BOLT_SLOT_CLEARANCE = 127;  // 5" arc clearance for bolts

// Weld Parameters
PIVOT_MOUNT_WELD_DIA = 6.35;            // 0.25" weld bead diameter
PIVOT_MOUNT_WELD_OD = DOM_PIPE_OD + PIVOT_MOUNT_WELD_DIA;
PIVOT_MOUNT_WELD_SLOT_CLEARANCE = 6.35; // 0.25" clearance around weld
PIVOT_MOUNT_SLOT_DIA = PIVOT_MOUNT_WELD_OD + PIVOT_MOUNT_WELD_SLOT_CLEARANCE;

// Bolt Angles Array (non-uniform spacing)
PIVOT_MOUNT_BOLT_ANGLES = [calculated array based on slot clearance]
```

---

## Design Evolution

### Iteration 1: Initial 6-Bolt Pattern
- Started with 6 evenly spaced bolts
- Issue: Bolts could interfere with DOM slot at bottom

### Iteration 2: 5-Bolt Pattern with Start Angle
- Changed to 5 bolts with 18° start angle
- Issue: Still uniform spacing, bolts too close to slot

### Iteration 3: Non-Uniform Spacing
- Implemented larger gap around slot (initially 2")
- Remaining bolts closer together to compensate

### Iteration 4: Increased Slot Clearance
- 2" → 4" → 5" total arc clearance
- Final: 2.5" clearance on each side of slot

### Iteration 5: Weld Visualization
- Added torus (donut) shapes for weld beads
- Initially quarter-circle fillets, changed to full torus
- Added welds on both inside AND outside faces of plates

### Iteration 6: Slot Cutout Sizing
- Slot sized to accommodate weld OD + 1/4" clearance
- Allows assembly to slide in without interference

---

## Weld Visualization

The welds are shown as red torus shapes using OpenSCAD's `rotate_extrude`:

```openscad
rotate_extrude(angle=360, $fn=64)
    translate([DOM_PIPE_OD/2, 0])
        circle(d=_weld_dia, $fn=24);
```

**4 weld locations per assembly:**
1. Front plate - inside face
2. Front plate - outside face
3. Rear plate - inside face
4. Rear plate - outside face

---

## Assembly Installation

1. Weld DOM tube between two 6" circular plates (use welding jig)
2. Slide assembly into arm from below through DOM slot
3. Align 5 bolt holes with arm plate holes
4. Secure with 1/2" bolts (10 total, 5 per side)

---

## Benefits of This Design

1. **Replaceable**: Pivot can be serviced without cutting/welding arm
2. **Parametric**: All dimensions driven by parameters
3. **Non-uniform bolt pattern**: Maximum clearance from slot while maintaining strength
4. **Proper weld clearance**: Slot sized for weld bead accommodation
5. **Visual verification**: Red weld indicators show fabrication requirements

---

## Files Reference

| File | Purpose |
|------|---------|
| [lifetrac_v25_params.scad](../mechanical_design/openscad/lifetrac_v25_params.scad) | Central parameters |
| [pivot_mount_assembly.scad](../mechanical_design/openscad/parts/pivot_mount_assembly.scad) | Assembly definition |
| [arm_plate.scad](../mechanical_design/openscad/parts/arm_plate.scad) | Arm plate with slot/holes |
| [loader_arm_v2.scad](../mechanical_design/openscad/modules/loader_arm_v2.scad) | Full arm visualization |
| [pivot_mount_welding_jig.scad](../mechanical_design/openscad/3d_printed_welding_jigs/pivot_mount_welding_jig.scad) | Fabrication jig |

---

## Console Output Verification

When rendering, the console displays:
```
=== PIVOT MOUNT BOLT PATTERN ===
Slot at: 270° (bottom of arm)
Slot gap: 127.3° (127mm arc)
Regular gap: 58.2°
Bolt flanking slot (CCW): 206.3° (lower-left)
Bolt flanking slot (CW):  333.7° (lower-right)
All bolt angles (raw): [333.662, 391.831, 450, 508.169, 566.338]
```

---

*Report generated by GitHub Copilot - January 25, 2025*
