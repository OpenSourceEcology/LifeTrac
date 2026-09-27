# Structural Analysis Test Log

This file tracks the history of structural analysis test results.

---

## Latest Analysis

**Date:** 2026-09-27 09:18:39 UTC
**Commit:** f5987e32615abfb9f51f9ab35193a54b1e27c4b7
**Branch:** main
**Workflow Run:** https://github.com/OpenSourceEcology/LifeTrac/actions/runs/36308920341

### Summary

Static checks at the 3000 psi relief pressure: each lift cylinder pushes 65.5 kN, and each bucket cylinder pushes 94.3 kN and pulls 70.7 kN. Arm angles -27.7 to 49.4 deg; bucket tilts -94.4 to 50 deg from the arm. Method and assumptions: `openscad/analysis/structural_analysis.scad`.

**Rated operating capacity: 92 kg (202 lb)**, half the lowest tipping load of 184 kg (294 kg with an operator on the platform). The lowest hydraulic lift capacity is 1258 kg. The masses are estimates (910 kg empty).

**Bucket case:** with a bucket cylinder stalled against the cutting edge, each bucket pivot pin carries up to 116.2 kN when dumping and 87.1 kN when curling. The arm at the lift bracket carries up to 57 kN m, against 17.5 kN m in the lift case.

| Check | Demand | Capacity | Ratio | Status |
|---|---|---|---|---|
| Arm pivot pin, double shear (`PIVOT_PIN_SHEAR`) | 30 MPa | 212 MPa | 0.14 | PASS |
| Arm pivot hole in the two side panels (`PIVOT_HOLE_SIDE_PANELS`) | 67.9 kN | 182.6 kN | 0.37 | PASS |
| Pivot-mount bolts, 1/2in, per bolt (`PIVOT_MOUNT_BOLTS`) | 6.8 kN | 19.4 kN | 0.35 | PASS |
| Pivot-mount DOM-to-plate fillet welds (`PIVOT_MOUNT_WELDS`) | 47 MPa | 145 MPa | 0.33 | PASS |
| Arm at the lift bracket, tube and side plates together (`ARM_BENDING`) | 132 MPa | 150 MPa | 0.88 | PASS |
| Arm at the lift bracket, side plates alone (`ARM_BENDING_PLATES_ONLY`) | 355 MPa | 150 MPa | 2.37 | known fail (B4: the tube is bolted to the side plates only near its ends) |
| Arm at the lift bracket, bucket cylinder stalled, tube and side plates together (`ARM_BENDING_BUCKET`) | 431 MPa | 150 MPa | 2.87 | known fail (P14: the bucket cylinders bend the front of the arm) |
| Lift-cylinder pins, double shear (`LIFT_CYL_PINS`) | 65 MPa | 212 MPa | 0.30 | PASS |
| Lift-cylinder hole in the two arm plates (`LIFT_BRACKET_HOLE`) | 65.5 kN | 74.4 kN | 0.88 | PASS |
| Lift-cylinder base hole in the two side panels (`LIFT_BASE_HOLE`) | 65.5 kN | 221.3 kN | 0.30 | PASS |
| Cross beam T3, bending about both axes and twist (von Mises) (`T3_COMBINED`) | 238 MPa | 150 MPa | 1.59 | known fail (B4: the bucket-cylinder lugs hang below T3 and twist it) |
| Bucket-cylinder pins, 3/4in, double shear (`BUCKET_CYL_PINS`) | 165 MPa | 212 MPa | 0.78 | PASS |
| Bucket-cylinder lug bolts on T3, 4 x 1/4in, shear (`BUCKET_CYL_LUG_BOLTS`) | 94.3 kN | 23.6 kN | 4.00 | known fail (P14, M8) |
| Bucket-cylinder lug bolts on the bucket, 4 x 1/4in, tension and shear (`BUCKET_CYL_LUG_BOLTS_BUCKET`) | 94.3 kN | 23.7 kN | 3.99 | known fail (P14, M8) |
| Bucket pivot pin, 1in, double shear (`BUCKET_PIN_SHEAR`) | 115 MPa | 212 MPa | 0.54 | PASS |
| Bucket pivot hole in the two arm-tip plates (`BUCKET_PIN_ARM_TIP`) | 116.2 kN | 38.7 kN | 3.00 | known fail (P14) |
| Bucket pivot hole in the U-lug's two walls (`BUCKET_PIVOT_LUG`) | 116.2 kN | 45.3 kN | 2.56 | known fail (P14) |
| Bucket pivot lug bolts, 4 x 1/4in, tension and shear (`BUCKET_LUG_BOLTS`) | 112.1 kN | 25.1 kN | 4.46 | known fail (P14, M8) |

OVERALL: no new failures; 8 known issues

### Detail

Lift case, per arm, with both lift cylinders at relief and the load at the bucket's load centre:

| Arm angle | Cylinder lever | Load lever | Load held | Pivot-pin force | Arm moment at bracket | Tipping load | Hydraulic capacity |
|---|---|---|---|---|---|---|---|
| -27.7 deg | 497 mm | 1725 mm | 18.9 kN | 67.9 kN | 17.5 kN m | 342 kg | 3674 kg |
| -18.1 deg | 472 mm | 1875 mm | 16.5 kN | 65.2 kN | 16.7 kN m | 256 kg | 3187 kg |
| -8.4 deg | 441 mm | 1981 mm | 14.6 kN | 63.2 kN | 15.8 kN m | 210 kg | 2796 kg |
| 1.2 deg | 405 mm | 2040 mm | 13 kN | 61.7 kN | 14.8 kN m | 188 kg | 2473 kg |
| 10.9 deg | 365 mm | 2052 mm | 11.7 kN | 60.7 kN | 13.6 kN m | 184 kg | 2197 kg |
| 20.5 deg | 322 mm | 2015 mm | 10.5 kN | 60.1 kN | 12.2 kN m | 196 kg | 1951 kg |
| 30.2 deg | 275 mm | 1931 mm | 9.3 kN | 59.7 kN | 10.7 kN m | 230 kg | 1723 kg |
| 39.8 deg | 226 mm | 1802 mm | 8.2 kN | 59.7 kN | 9.1 kN m | 294 kg | 1497 kg |
| 49.4 deg | 175 mm | 1631 mm | 7 kN | 60 kN | 7.3 kN m | 416 kg | 1258 kg |

Bucket case, per side, with one bucket cylinder at relief stalled against the cutting edge. The tilt is the bucket's angle to the arm, negative toward dump:

| Bucket tilt | Cylinder length | Edge force, dump / curl | Bucket-pin force, dump / curl | Arm moment at bracket, dump / curl |
|---|---|---|---|---|
| -94.4 deg | 1220 mm | 3.2 / 2.4 kN | 97.5 / 73.2 kN | 10.9 / 8.2 kN m |
| -76.4 deg | 1204 mm | 12 / 9 kN | 106.1 / 79.6 kN | 19.3 / 14.5 kN m |
| -58.3 deg | 1169 mm | 20.1 / 15.1 kN | 112.7 / 84.5 kN | 30.8 / 23.1 kN m |
| -40.3 deg | 1118 mm | 27.1 / 20.3 kN | 116 / 87 kN | 42.8 / 32.1 kN m |
| -22.2 deg | 1054 mm | 32.4 / 24.3 kN | 115 / 86.2 kN | 52.5 / 39.4 kN m |
| -4.2 deg | 980 mm | 35.2 / 26.4 kN | 109.2 / 81.9 kN | 56.9 / 42.7 kN m |
| 13.9 deg | 904 mm | 34.9 / 26.2 kN | 99 / 74.3 kN | 54.3 / 40.7 kN m |
| 31.9 deg | 833 mm | 30.6 / 23 kN | 87.3 / 65.5 kN | 44.4 / 33.3 kN m |
| 50 deg | 776 mm | 21.8 / 16.3 kN | 80.7 / 60.5 kN | 29.8 / 22.4 kN m |

---

_This file is automatically updated by the OpenSCAD Structural Analysis workflow._
