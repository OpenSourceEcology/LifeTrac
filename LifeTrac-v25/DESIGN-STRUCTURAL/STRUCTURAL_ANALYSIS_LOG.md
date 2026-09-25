# Structural Analysis Test Log

This file tracks the history of structural analysis test results.

---

## Latest Analysis

**Date:** 2026-09-25 15:42:42 UTC
**Commit:** eec4063c1ef247ffa415de2b8b232755db8f6909
**Branch:** claude/dreamy-mccarthy-lpn9af
**Workflow Run:** https://github.com/OpenSourceEcology/LifeTrac/actions/runs/36155925400

### Results Summary

```
"========================================"
"RATED LIFT CAPACITY:", 3305.74, "kg (", 7289.16, "lbs)"
""
"Component              | Stress Ratio | Status"
"---------------------- | ------------ | ------"
"Arm Bending            |", 11.2595, "|", "FAIL"
"Arm Deflection         |", 18.3632, "|", "FAIL"
"Pivot Pin Shear        |", 0.18963, "|", "PASS"
"Pivot Bearing          |", 0.357444, "|", "PASS"
"Clevis Pin Shear       |", 0.260416, "|", "PASS"
"Cross Beam Bending     |", 10.7261, "|", "FAIL"
"Pivot Ring Welds       |", 14.2164, "|", "FAIL"
"Cross Beam Welds       |", 0.713108, "|", "PASS"
""
"*** OVERALL STRUCTURAL ASSESSMENT:", "REVIEW REQUIRED ✗"
```

### ⚠️ Failed Components

**Current Rated Capacity:** 3305.74, "kg (", 7289.16, "lbs)"

#### Detailed Failure Analysis

**1. Arm Bending Stress**
- "=== ARM BENDING STRESS ANALYSIS ==="
- "Arm tube:", 76.2, "x", 76.2, "x", 6.35, "mm"
- "Moment of inertia:", 1.45464e+6, "mm⁴"
- "Section modulus:", 38179.6, "mm³"
- "Load per arm:", 32429.3, "N"
- "Max bending moment at pivot:", 5.37354e+7, "N·mm"

**2. Arm Deflection**
- "=== ARM DEFLECTION ANALYSIS ==="
- "Calculated tip deflection:", 169.043, "mm"
- "Allowable deflection (L/180):", 9.20556, "mm"
- "ARM DEFLECTION CHECK:", "FAIL ✗"
- "=== PIVOT PIN SHEAR ANALYSIS ==="
- "Pivot pin diameter:", 38.1, "mm (", 1.5, "inches)"

**3. Cross Beam Bending**
- "=== CROSS BEAM STRESS ANALYSIS ==="
- "Cross beam:", 50.8, "x", 50.8, "x", 6.35, "mm"
- "Span between arms:", 849.2, "mm"
- "Section modulus:", 14936.1, "mm³"
- "Bucket cylinder force:", 94327.8, "N per cylinder"
- "Max bending moment:", 2.00258e+7, "N·mm"

**4. Pivot Ring Welds**
- "=== PIVOT RING WELD ANALYSIS ==="
- "Weld size (fillet leg):", 6.35, "mm"
- "Weld throat thickness:", 4.48945, "mm"
- "Total weld length per arm:", 609.6, "mm"
- "Weld section modulus:", 52135.4, "mm³"
- "Weld bending stress:", 1030.69, "MPa"

### Recommended Actions

See [STRUCTURAL_ANALYSIS.md](STRUCTURAL_ANALYSIS.md) for design modification recommendations.

---

_This file is automatically updated by the OpenSCAD Structural Analysis workflow._
