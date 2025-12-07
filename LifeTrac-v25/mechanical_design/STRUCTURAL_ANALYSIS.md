# LifeTrac v25 Structural Analysis

## Overview

This document summarizes the structural engineering calculations implemented in `lifetrac_v25.scad` to verify that the loader design can safely support its rated lifting capacity.

## Lifting Capacity Calculation

### Hydraulic Force

The lifting force is calculated from the hydraulic cylinder parameters:

| Parameter | Value |
|-----------|-------|
| Operating Pressure | 3000 PSI (20.7 MPa) |
| Lift Cylinder Bore | 63.5 mm (2.5") |
| Number of Lift Cylinders | 2 |

**Force per cylinder:**
$$F = P \times A = 3000 \text{ psi} \times \frac{\pi (2.5/2)^2}{1} = 14,726 \text{ lbf}$$

**Total lift cylinder force:** 29,452 lbf (131 kN)

### Lever Arm Geometry

The actual lifting capacity depends on the mechanical advantage at different arm positions:

```
                    ┌─ Bucket Load Point
                    │
    ════════════════╧═══════
              ARM            ↑ Cylinder
    ═════════════════════════│ Force
         ↑                   │
         │ Pivot            ═╧═ Cylinder Base
```

**Mechanical Advantage = Cylinder Moment Arm / Load Moment Arm**

The capacity varies with arm angle:
- At ground level: Lower mechanical advantage (minimum capacity)
- Arms raised: Higher mechanical advantage

### Rated Capacity

The rated lift capacity is the **minimum** capacity across the full range of arm motion, providing a conservative rating.

## Structural Analysis - Mechanics of Deformable Bodies

### Material Properties

**A36 Structural Steel (HSS Tubing):**
| Property | Value |
|----------|-------|
| Yield Strength | 250 MPa (36 ksi) |
| Ultimate Tensile Strength | 400 MPa (58 ksi) |
| Elastic Modulus | 200,000 MPa (29,000 ksi) |
| Shear Modulus | 77,200 MPa (11,200 ksi) |

**Grade 8 Bolts:**
| Property | Value |
|----------|-------|
| Yield Strength | 896 MPa (130 ksi) |
| Tensile Strength | 1034 MPa (150 ksi) |
| Shear Strength | 620 MPa (90 ksi) |

### Safety Factors

| Application | Safety Factor |
|-------------|---------------|
| Static Loads | 2.0 |
| Fatigue Loading | 3.0 |
| Bolted Connections | 2.5 |

### 1. Arm Bending Stress Analysis

Each arm is modeled as a cantilever beam:

**Geometry:**
- Tube: 3" × 3" × 1/4" wall (76.2 × 76.2 × 6.35 mm)
- Length: Calculated parametrically based on reach requirements

**Section Properties:**
$$I = \frac{b_{outer}^4 - b_{inner}^4}{12}$$

$$S = \frac{I}{c} = \frac{I}{b_{outer}/2}$$

**Bending Stress:**
$$\sigma = \frac{M}{S} = \frac{F \times L}{S}$$

**Allowable Stress:**
$$\sigma_{allow} = \frac{\sigma_{yield}}{SF} = \frac{250}{2.0} = 125 \text{ MPa}$$

### 2. Arm Deflection Analysis

Maximum tip deflection for a cantilever with end load:

$$\delta = \frac{PL^3}{3EI}$$

**Allowable Deflection:** L/180 (typical structural limit)

### 3. Pivot Pin Shear Analysis

The 1.5" (38.1 mm) pivot pins are in **double shear**:

$$\tau = \frac{F}{2A} = \frac{F}{2 \times \frac{\pi d^2}{4}}$$

**Allowable Shear Stress:**
$$\tau_{allow} = \frac{0.6 \times \sigma_{yield}}{SF}$$

### 4. Bearing Stress Analysis

Bearing stress on pivot holes in arm tubing:

$$\sigma_{bearing} = \frac{F}{d \times t \times 2}$$

Where:
- d = pin diameter
- t = tube wall thickness
- 2 = two walls in contact

**Allowable Bearing Stress:** 1.5 × σ_yield / SF

### 5. Clevis Pin Analysis

Cylinder clevis pins (1" diameter) in double shear:

Uses Grade 8 bolt material properties for allowable stress.

### 6. Cross Beam Stress Analysis

Cross beams (2" × 2" × 1/4" tube) span between arms:

**Loading:** Bucket cylinder forces create bending and shear

$$M_{max} = \frac{F \times L}{4}$$ (point load at center)

### 7. Weld Stress Analysis

#### Pivot Ring Welds
- 1/4" fillet welds around arm tube perimeter
- Two rings per arm
- Weld throat: 0.707 × leg size

**Weld Stress:**
$$\sigma_{weld} = \frac{M}{S_{weld}}$$

#### Cross Beam Welds
- 1/4" fillet welds at arm connections
- Shear loading from bucket cylinders

**Allowable Weld Stress (E70XX):**
$$\sigma_{allow} = \frac{0.3 \times 70 \text{ ksi}}{SF} = \frac{145 \text{ MPa}}{2.0} = 72.5 \text{ MPa}$$

## Results Summary

When the OpenSCAD file is rendered, it outputs a complete structural analysis:

```
========================================
    COMPLETE STRUCTURAL ANALYSIS SUMMARY    
========================================
RATED LIFT CAPACITY: XXX kg (XXX lbs)

Component              | Stress Ratio | Status
---------------------- | ------------ | ------
Arm Bending            | X.XX         | PASS/FAIL
Arm Deflection         | X.XX         | PASS/FAIL
Pivot Pin Shear        | X.XX         | PASS/FAIL
Pivot Bearing          | X.XX         | PASS/FAIL
Clevis Pin Shear       | X.XX         | PASS/FAIL
Cross Beam Bending     | X.XX         | PASS/FAIL
Pivot Ring Welds       | X.XX         | PASS/FAIL
Cross Beam Welds       | X.XX         | PASS/FAIL

*** OVERALL STRUCTURAL ASSESSMENT: ALL CHECKS PASS ✓✓✓
========================================
```

## Design Modifications for Increased Capacity

If any check fails, consider:

1. **Arm Bending:** Upgrade to larger tube (4" × 4" × 1/4")
2. **Pivot Pins:** Increase diameter (2" pins available)
3. **Bearing Stress:** Add bearing sleeves or pivot rings
4. **Welds:** Increase weld size to 5/16" or 3/8"
5. **Cross Beam:** Use 3" × 3" tube or add gussets

## Limitations

This analysis provides **first-order estimates** using classical mechanics. For production use, consider:

1. **FEA Analysis:** Finite Element Analysis for stress concentrations
2. **Fatigue Analysis:** Cyclic loading effects over service life
3. **Dynamic Loading:** Impact factors for rough operation
4. **Buckling:** Compression member stability
5. **Connection Details:** Local effects at joints

## References

- AISC Steel Construction Manual
- AWS D1.1 Structural Welding Code
- SAE Grade 8 Bolt Specifications
- ASTM A500 HSS Tubing Properties
