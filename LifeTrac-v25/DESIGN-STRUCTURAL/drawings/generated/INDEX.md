# LifeTrac v25 - Part Drawings Index

> **Generated file - do not edit.** Produced by [`../generate_part_drawings.py`](../generate_part_drawings.py) from [`../parts_manifest.yaml`](../parts_manifest.yaml) and the OpenSCAD model. See [`../README.md`](../README.md).

Plain quantities are counted from `BOM_PART` markers in the assembly. Quantities marked *(manifest)* are typed in by hand; *(est. from holes)* are estimated from hole counts (`qty_from_holes`). Check both before ordering. Masses are calculated from the model.

## Totals

| Category | Unique parts | Pieces per machine | Mass per machine |
|---|---:|---:|---:|
| CNC-cut plate | 19 | 47 | 628.7 kg |
| Angle iron (cut and drill) | 10 | 78 | 87.3 kg |
| Rectangular / square tube (cut and drill) | 6 | 11 | 112.9 kg |
| Round bar and pins (cut and drill) | 6 | 16 | 19.2 kg |
| Lugs and brackets cut from tube | 3 | 6 | 4.0 kg |
| 3D-printed jigs | 6 | 13 † | 4.4 kg |
| Purchased hardware (reference) | 10 | 1350 † | 24.2 kg |
| **All parts** | **60** | **1521** † | **881 kg** |

† Includes quantities typed into the manifest or estimated from hole counts; the tables below mark which.

### Stock

Material for the parts cut from stock, added up per stock size. Lengths and blank areas are net: allow for saw kerf, offcuts and plate nesting when ordering.

| Stock | Parts | Pieces | Total cut length | Total blank area | Mass |
|---|---|---:|---:|---:|---:|
| PL 1/2 [12.7] ASTM A36 | P1, P2, P3, P17 | 8 |  | 5.61 m² [60.4 ft²] | 330.2 kg |
| PL 1/4 [6.35] ASTM A36 | P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15, P16, P18 | 38 |  | 7.59 m² [81.7 ft²] | 285.3 kg |
| PL 3/4 [19.05] ASTM A36 | P19 | 1 |  | 0.09 m² [0.9 ft²] | 13.2 kg |
| L2x2x1/4 ANGLE (2" x 2" x 1/4"), ASTM A36 | A1, A2, A4, A5, A6-1, A6-2, A6-3, A7, A8, A10 | 78 | 18.70 m [61.4 ft] |  | 87.3 kg |
| HSS 6x2x1/4 RECT TUBE (2" x 6" x 1/4"), ASTM A500 | T1, T2, T3, T4 | 5 | 5.70 m [18.7 ft] |  | 106.0 kg |
| DOM TUBE 2.000 OD x .250 WALL [50.8 x 6.35] | T6 | 2 | 0.24 m [0.8 ft] |  | 1.7 kg |
| DOM TUBE 2.000 OD [50.8], BORED TO THE ID SHOWN | T7 | 4 | 0.56 m [1.8 ft] |  | 5.2 kg |
| Ø1 [25.4] ROUND BAR, 4140 OR EQUIVALENT | R1, R4, R5 | 6 | 0.64 m [2.1 ft] |  | 2.5 kg |
| Ø1-1/2 [38.1] ROUND BAR, 4140 OR EQUIVALENT | R2 | 2 | 0.45 m [1.5 ft] |  | 4.0 kg |
| Ø3/4 [19.05] ROUND BAR, 4140 OR EQUIVALENT | R3 | 4 | 0.42 m [1.4 ft] |  | 0.9 kg |
| Ø1-1/4 [31.75] ROUND SHAFTING | R6 | 4 | 1.90 m [6.2 ft] |  | 11.7 kg |
| HSS 3x3x1/4 SQUARE TUBE, ASTM A500 | U1, U2, U3 | 6 |  |  | 4.0 kg |
| PLA OR PETG FILAMENT | J1, J2, J3, J4, J5, J6 | 13 |  |  | 4.4 kg |

## CNC-cut plate

| Part | Name | Stock | Qty | Size | Mass each | Rev | Drawing | CNC |
|---|---|---|---:|---|---:|:-:|---|---|
| **P1** | Side panel, outer, left | PL 1/2 [12.7] ASTM A36 | 1 | 1392.6 x 1000 | 82.41 kg | A | [PDF](pdf/P1_side-panel-outer-left.pdf) | [DXF](dxf/P1_side-panel-outer-left.dxf) |
| **P2** | Side panel, outer, right | PL 1/2 [12.7] ASTM A36 | 1 | 1392.6 x 1000 | 82.41 kg | A | [PDF](pdf/P2_side-panel-outer-right.pdf) | [DXF](dxf/P2_side-panel-outer-right.dxf) |
| **P3** | Side panel, inner | PL 1/2 [12.7] ASTM A36 | 2 | 1349.2 x 968.2 | 75.28 kg | A | [PDF](pdf/P3_side-panel-inner.pdf) | [DXF](dxf/P3_side-panel-inner.dxf) |
| **P4** | Back stiffener plate | PL 1/4 [6.35] ASTM A36 | 1 | 1020 x 650 | 32.97 kg | A | [PDF](pdf/P4_back-stiffener-plate.pdf) | [DXF](dxf/P4_back-stiffener-plate.dxf) |
| **P5** | Front stiffener plate, centre | PL 1/4 [6.35] ASTM A36 | 1 | 481.6 x 260.4 | 6.22 kg | A | [PDF](pdf/P5_front-stiffener-plate-centre.pdf) | [DXF](dxf/P5_front-stiffener-plate-centre.dxf) |
| **P6** | Front stiffener plate, outer | PL 1/4 [6.35] ASTM A36 | 2 | 269.2 x 127 | 1.68 kg | A | [PDF](pdf/P6_front-stiffener-plate-outer.pdf) | [DXF](dxf/P6_front-stiffener-plate-outer.dxf) |
| **P7** | Bottom stiffener plate | PL 1/4 [6.35] ASTM A36 | 1 | 1196.8 x 1020 | 60.57 kg | A | [PDF](pdf/P7_bottom-stiffener-plate.pdf) | [DXF](dxf/P7_bottom-stiffener-plate.dxf) |
| **P8** | Motor mounting plate | PL 1/4 [6.35] ASTM A36 | 2 | 1196.8 x 254 | 14.13 kg | A | [PDF](pdf/P8_motor-mounting-plate.pdf) | [DXF](dxf/P8_motor-mounting-plate.dxf) |
| **P9** | Arm side plate, inner | PL 1/4 [6.35] ASTM A36 | 2 | 1792 x 350.1 | 13.21 kg | A | [PDF](pdf/P9_arm-side-plate-inner.pdf) | [DXF](dxf/P9_arm-side-plate-inner.dxf) |
| **P10** | Arm side plate, outer | PL 1/4 [6.35] ASTM A36 | 2 | 1792 x 350.1 | 13.62 kg | A | [PDF](pdf/P10_arm-side-plate-outer.pdf) | [DXF](dxf/P10_arm-side-plate-outer.dxf) |
| **P11** | Arm pivot mount plate | PL 1/4 [6.35] ASTM A36 | 4 | 152.3 x 152.3 | 0.78 kg | A | [PDF](pdf/P11_arm-pivot-mount-plate.pdf) | [DXF](dxf/P11_arm-pivot-mount-plate.dxf) |
| **P12** | Bucket back plate | PL 1/4 [6.35] ASTM A36 | 1 | 1112.7 x 450 | 24.66 kg | A | [PDF](pdf/P12_bucket-back-plate.pdf) | [DXF](dxf/P12_bucket-back-plate.dxf) |
| **P13** | Bucket bottom plate | PL 1/4 [6.35] ASTM A36 | 1 | 1112.7 x 600 | 32.79 kg | A | [PDF](pdf/P13_bucket-bottom-plate.pdf) | [DXF](dxf/P13_bucket-bottom-plate.dxf) |
| **P14** | Bucket side plate | PL 1/4 [6.35] ASTM A36 | 2 | 600 x 450 | 8.83 kg | A | [PDF](pdf/P14_bucket-side-plate.pdf) | [DXF](dxf/P14_bucket-side-plate.dxf) |
| **P15** | Platform deck | PL 1/4 [6.35] ASTM A36 | 1 | 735.5 x 400 | 14.30 kg | A | [PDF](pdf/P15_platform-deck.pdf) | [DXF](dxf/P15_platform-deck.dxf) |
| **P16** | Platform pivot bracket | PL 1/4 [6.35] ASTM A36 | 2 | 400 x 230 | 2.45 kg | A | [PDF](pdf/P16_platform-pivot-bracket.pdf) | [DXF](dxf/P16_platform-pivot-bracket.dxf) |
| **P17** | Wheel hub lug plate | PL 1/2 [12.7] ASTM A36 | 4 | 228.6 x 228.6 | 3.71 kg | A | [PDF](pdf/P17_wheel-hub-lug-plate.pdf) | [DXF](dxf/P17_wheel-hub-lug-plate.dxf) |
| **P18** | Wheel hub gusset | PL 1/4 [6.35] ASTM A36 | 16 | 86.4 x 83.9 | 0.18 kg | A | [PDF](pdf/P18_wheel-hub-gusset.pdf) | [DXF](dxf/P18_wheel-hub-gusset.dxf) |
| **P19** | Bucket cutting edge | PL 3/4 [19.05] ASTM A36 | 1 | 1100 x 80 | 13.16 kg | A | [PDF](pdf/P19_bucket-cutting-edge.pdf) | [DXF](dxf/P19_bucket-cutting-edge.dxf) |

## Angle iron (cut and drill)

| Part | Name | Stock | Qty | Size | Mass each | Rev | Drawing | CNC |
|---|---|---|---:|---|---:|:-:|---|---|
| **A1** | Back stiffener outer vertical angle | L2x2x1/4 ANGLE (2" x 2" x 1/4"), ASTM A36 | 2 | 650 [~25-9/16"] | 3.06 kg | A | [PDF](pdf/A1_back-stiffener-outer-vertical-angle.pdf) |  |
| **A2** | Back stiffener inner vertical angle | L2x2x1/4 ANGLE (2" x 2" x 1/4"), ASTM A36 | 4 | 650 [~25-9/16"] | 3.06 kg | A | [PDF](pdf/A2_back-stiffener-inner-vertical-angle.pdf) |  |
| **A4** | Frame tube mount angle | L2x2x1/4 ANGLE (2" x 2" x 1/4"), ASTM A36 | 26 | 146.1 [5-3/4"] | 0.67 kg | A | [PDF](pdf/A4_frame-tube-mount-angle.pdf) |  |
| **A5** | Arm crossbeam mount angle | L2x2x1/4 ANGLE (2" x 2" x 1/4"), ASTM A36 | 4 | 146.1 [5-3/4"] | 0.67 kg | A | [PDF](pdf/A5_arm-crossbeam-mount-angle.pdf) |  |
| **A6-1** | Bottom stiffener angle, rear segment | L2x2x1/4 ANGLE (2" x 2" x 1/4"), ASTM A36 | 10 | 200 [7-7/8"] | 0.94 kg | A | [PDF](pdf/A6-1_bottom-stiffener-angle-rear-segment.pdf) |  |
| **A6-2** | Bottom stiffener angle, middle segment | L2x2x1/4 ANGLE (2" x 2" x 1/4"), ASTM A36 | 10 | 300 [11-13/16"] | 1.41 kg | A | [PDF](pdf/A6-2_bottom-stiffener-angle-middle-segment.pdf) |  |
| **A6-3** | Bottom stiffener angle, front segment | L2x2x1/4 ANGLE (2" x 2" x 1/4"), ASTM A36 | 10 | 250 [~9-13/16"] | 1.18 kg | A | [PDF](pdf/A6-3_bottom-stiffener-angle-front-segment.pdf) |  |
| **A7** | Platform side angle arm | L2x2x1/4 ANGLE (2" x 2" x 1/4"), ASTM A36 | 2 | 247.6 [9-3/4"] | 1.14 kg | A | [PDF](pdf/A7_platform-side-angle-arm.pdf) |  |
| **A8** | Platform transverse angle | L2x2x1/4 ANGLE (2" x 2" x 1/4"), ASTM A36 | 2 | 729.2 [~28-11/16"] | 3.44 kg | A | [PDF](pdf/A8_platform-transverse-angle.pdf) |  |
| **A10** | Front stiffener outer angle | L2x2x1/4 ANGLE (2" x 2" x 1/4"), ASTM A36 | 8 | 120.7 [4-3/4"] | 0.56 kg | A | [PDF](pdf/A10_front-stiffener-outer-angle.pdf) |  |

## Rectangular / square tube (cut and drill)

| Part | Name | Stock | Qty | Size | Mass each | Rev | Drawing | CNC |
|---|---|---|---:|---|---:|:-:|---|---|
| **T1** | Front frame cross tube | HSS 6x2x1/4 RECT TUBE (2" x 6" x 1/4"), ASTM A500 | 1 | 1070.8 [~42-3/16"] | 20.18 kg | A | [PDF](pdf/T1_front-frame-cross-tube.pdf) |  |
| **T2** | Rear frame cross tube | HSS 6x2x1/4 RECT TUBE (2" x 6" x 1/4"), ASTM A500 | 1 | 1070.8 [~42-3/16"] | 20.18 kg | A | [PDF](pdf/T2_rear-frame-cross-tube.pdf) |  |
| **T3** | Arm crossbeam tube | HSS 6x2x1/4 RECT TUBE (2" x 6" x 1/4"), ASTM A500 | 1 | 849.2 [33-7/16"] | 16.58 kg | A | [PDF](pdf/T3_arm-crossbeam-tube.pdf) |  |
| **T4** | Arm main tube | HSS 6x2x1/4 RECT TUBE (2" x 6" x 1/4"), ASTM A500 | 2 | 1356.5 [~53-7/16"] | 24.54 kg | A | [PDF](pdf/T4_arm-main-tube.pdf) |  |
| **T6** | Arm pivot DOM tube | DOM TUBE 2.000 OD x .250 WALL [50.8 x 6.35] | 2 | 120 [~4-3/4"] | 0.83 kg | A | [PDF](pdf/T6_arm-pivot-dom-tube.pdf) |  |
| **T7** | Wheel hub DOM tube | DOM TUBE 2.000 OD [50.8], BORED TO THE ID SHOWN | 4 | 139.7 [5-1/2"] | 1.29 kg | A | [PDF](pdf/T7_wheel-hub-dom-tube.pdf) |  |

## Round bar and pins (cut and drill)

| Part | Name | Stock | Qty | Size | Mass each | Rev | Drawing | CNC |
|---|---|---|---:|---|---:|:-:|---|---|
| **R1** | Lift cylinder base pin | Ø1 [25.4] ROUND BAR, 4140 OR EQUIVALENT | 2 | 160 [6-5/16"] | 0.63 kg | A | [PDF](pdf/R1_lift-cylinder-base-pin.pdf) |  |
| **R2** | Arm pivot pin | Ø1-1/2 [38.1] ROUND BAR, 4140 OR EQUIVALENT | 2 | 226.4 [~8-15/16"] | 2.02 kg | A | [PDF](pdf/R2_arm-pivot-pin.pdf) |  |
| **R3** | Clevis pin 3/4 | Ø3/4 [19.05] ROUND BAR, 4140 OR EQUIVALENT | 4 | 106.2 [4-3/16"] | 0.23 kg | A | [PDF](pdf/R3_clevis-pin-3-4.pdf) |  |
| **R4** | Bucket pivot pin | Ø1 [25.4] ROUND BAR, 4140 OR EQUIVALENT | 2 | 106.2 [4-3/16"] | 0.42 kg | A | [PDF](pdf/R4_bucket-pivot-pin.pdf) |  |
| **R5** | Platform pivot pin | Ø1 [25.4] ROUND BAR, 4140 OR EQUIVALENT | 2 | 55.4 [2-3/16"] | 0.22 kg | A | [PDF](pdf/R5_platform-pivot-pin.pdf) |  |
| **R6** | Wheel shaft | Ø1-1/4 [31.75] ROUND SHAFTING | 4 | 474 [~18-11/16"] | 2.93 kg | A | [PDF](pdf/R6_wheel-shaft.pdf) |  |

## Lugs and brackets cut from tube

| Part | Name | Stock | Qty | Size | Mass each | Rev | Drawing | CNC |
|---|---|---|---:|---|---:|:-:|---|---|
| **U1** | Arm crossbeam cylinder lug | HSS 3x3x1/4 SQUARE TUBE, ASTM A500 | 2 | 76.2 x 76.2 x 66.7 | 0.66 kg | A | [PDF](pdf/U1_arm-crossbeam-cylinder-lug.pdf) |  |
| **U2** | Bucket pivot lug | HSS 3x3x1/4 SQUARE TUBE, ASTM A500 | 2 | 76.2 x 76.2 x 66.7 | 0.66 kg | A | [PDF](pdf/U2_bucket-pivot-lug.pdf) |  |
| **U3** | Bucket cylinder lug | HSS 3x3x1/4 SQUARE TUBE, ASTM A500 | 2 | 76.2 x 76.2 x 66.7 | 0.68 kg | A | [PDF](pdf/U3_bucket-cylinder-lug.pdf) |  |

## 3D-printed jigs

| Part | Name | Stock | Qty | Size | Mass each | Rev | Drawing | CNC |
|---|---|---|---:|---|---:|:-:|---|---|
| **J1** | Tube drilling jig | PLA OR PETG FILAMENT | 4 *(manifest)* | 177.7 x 101.5 x 76.1 | 0.64 kg | A | [PDF](pdf/J1_tube-drilling-jig.pdf) |  |
| **J2** | Pivot mount welding jig base | PLA OR PETG FILAMENT | 1 *(manifest)* | 243.2 x 192.4 x 34.3 | 0.47 kg | A | [PDF](pdf/J2_pivot-mount-welding-jig-base.pdf) |  |
| **J3** | Pivot mount welding jig ring | PLA OR PETG FILAMENT | 2 *(manifest)* | 171.4 x 171.4 x 36.4 | 0.30 kg | A | [PDF](pdf/J3_pivot-mount-welding-jig-ring.pdf) |  |
| **J4** | Pivot welding spacer jig | PLA OR PETG FILAMENT | 2 *(manifest)* | 100 x 100 x 50.8 | 0.23 kg | A | [PDF](pdf/J4_pivot-welding-spacer-jig.pdf) |  |
| **J5** | Angle iron drill jig | PLA OR PETG FILAMENT | 2 *(manifest)* | 100 x 54.8 x 54.8 | 0.05 kg | A | [PDF](pdf/J5_angle-iron-drill-jig.pdf) |  |
| **J6** | Tube drill jig | PLA OR PETG FILAMENT | 2 *(manifest)* | 160.4 x 58.8 x 50 | 0.10 kg | A | [PDF](pdf/J6_tube-drill-jig.pdf) |  |

## Purchased hardware (reference)

| Part | Name | Stock | Qty | Size | Mass each | Rev | Drawing | CNC |
|---|---|---|---:|---|---:|:-:|---|---|
| **F1** | Hex nut 1-1/2-6 | HEX NUT 1-1/2-6 UNC-2B, GRADE 8, ASME B18.2.2 | 4 | 1-1/2-6 UNC | 0.42 kg | A | [PDF](pdf/F1_hex-nut-1-1-2-6.pdf) |  |
| **F2** | Platform lock pin 5/8 | HITCH / CLEVIS PIN Ø5/8 [15.875] x 1-5/16 USABLE, ZINC | 2 | Ø5/8 x 1-5/16 | 0.05 kg | A | [PDF](pdf/F2_platform-lock-pin-5-8.pdf) |  |
| **F3** | Hex bolt 1/2-13 | HEX BOLT 1/2-13 UNC-2A x L, SAE J429 GR 8, YELLOW ZINC, ASME B18.2.1 | 154 *(est. from holes)* | 1/2-13 x L (2 TO 6 IN) | 0.07 kg | A | [PDF](pdf/F3_hex-bolt-1-2-13.pdf) |  |
| **F4** | Hex nut 1/2-13 | HEX NUT 1/2-13 UNC-2B, GRADE 8, YELLOW ZINC, ASME B18.2.2 | 154 *(est. from holes)* | 1/2-13 UNC | 0.02 kg | A | [PDF](pdf/F4_hex-nut-1-2-13.pdf) |  |
| **F5** | Flat washer 1/2 SAE | FLAT WASHER 1/2 SAE (17/32 ID x 1-1/16 OD), HARDENED, ZINC | 308 *(est. from holes)* | 1/2 SAE | 0.01 kg | A | [PDF](pdf/F5_flat-washer-1-2-sae.pdf) |  |
| **F6** | Hex bolt 3/8-16 | HEX BOLT 3/8-16 UNC-2A x L, SAE J429 GR 8, YELLOW ZINC, ASME B18.2.1 | 170 *(est. from holes)* | 3/8-16 x L | 0.03 kg | A | [PDF](pdf/F6_hex-bolt-3-8-16.pdf) |  |
| **F7** | Hex nut 3/8-16 | HEX NUT 3/8-16 UNC-2B, GRADE 8, YELLOW ZINC, ASME B18.2.2 | 170 *(est. from holes)* | 3/8-16 UNC | 0.01 kg | A | [PDF](pdf/F7_hex-nut-3-8-16.pdf) |  |
| **F8** | Flat washer 3/8 SAE | FLAT WASHER 3/8 SAE (13/32 ID x 13/16 OD), HARDENED, ZINC | 340 *(est. from holes)* | 3/8 SAE | 0.00 kg | A | [PDF](pdf/F8_flat-washer-3-8-sae.pdf) |  |
| **F9** | Hex bolt 1/4-20 | HEX BOLT 1/4-20 UNC-2A x 1, SAE J429 GR 5, ZINC, ASME B18.2.1 | 24 *(manifest)* | 1/4-20 x 1 | 0.01 kg | A | [PDF](pdf/F9_hex-bolt-1-4-20.pdf) |  |
| **F10** | Hex nut 1/4-20 | HEX NUT 1/4-20 UNC-2B, GRADE 5, ZINC, ASME B18.2.2 | 24 *(manifest)* | 1/4-20 UNC | 0.00 kg | A | [PDF](pdf/F10_hex-nut-1-4-20.pdf) |  |
