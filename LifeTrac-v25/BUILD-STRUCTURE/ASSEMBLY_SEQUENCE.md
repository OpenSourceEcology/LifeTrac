# LifeTrac v25 - Structural Assembly Sequence

> **Generated from [`assembly_sequence.yaml`](assembly_sequence.yaml). Edit that file, not this one.** Step numbers come from the order of the steps, so inserting or moving a step renumbers everything after it. Refer to steps by their `id`, which never changes. See [`ASSEMBLY_MANUAL_AUTOGEN.md`](ASSEMBLY_MANUAL_AUTOGEN.md) for how this list will drive a picture manual.

**DRAFT** - 24 steps (24 draft) - 158 of 158 fabricated pieces placed.

## Prepare the parts

### Step 1 - Cut, drill and deburr every part from its drawing and paint-pen its part number on it

`prep-cut-drill-mark` - draft

**Tools:** [**J1**](../DESIGN-STRUCTURAL/drawings/generated/pdf/J1_tube-drilling-jig.pdf), [**J5**](../DESIGN-STRUCTURAL/drawings/generated/pdf/J5_angle-iron-drill-jig.pdf), [**J6**](../DESIGN-STRUCTURAL/drawings/generated/pdf/J6_tube-drill-jig.pdf), drill press, chop saw or bandsaw, CNC plasma

> Drawings: DESIGN-STRUCTURAL/drawings/generated/INDEX.md. Read each sheet's CHECK BEFORE MAKING notes first; they list known model problems. Lay the parts out by phase.

## Base frame

### Step 2 - Bolt the bottom angle runs to the floor plate

`floor-angles` - sub-assembly: *Floor* - draft

| Add | Part | Qty | Placed so far | Per machine |
|---|---|---:|---:|---:|
| [**P7**](../DESIGN-STRUCTURAL/drawings/generated/pdf/P7_bottom-stiffener-plate.pdf) | Bottom stiffener plate | 1 | 1 | 1 |
| [**A6-1**](../DESIGN-STRUCTURAL/drawings/generated/pdf/A6-1_bottom-stiffener-angle-rear-segment.pdf) | Bottom stiffener angle, rear segment | 6 | 6 | 10 |
| [**A6-2**](../DESIGN-STRUCTURAL/drawings/generated/pdf/A6-2_bottom-stiffener-angle-middle-segment.pdf) | Bottom stiffener angle, middle segment | 6 | 6 | 10 |
| [**A6-3**](../DESIGN-STRUCTURAL/drawings/generated/pdf/A6-3_bottom-stiffener-angle-front-segment.pdf) | Bottom stiffener angle, front segment | 6 | 6 | 10 |

> Six runs of three segments, vertical leg up, one run along each side panel line. Check the A6 lengths against the plate holes first: the drawings flag a mismatch.

### Step 3 - Bolt the vertical angles to the back stiffener plate

`rear-wall` - sub-assembly: *Rear wall* - draft

| Add | Part | Qty | Placed so far | Per machine |
|---|---|---:|---:|---:|
| [**P4**](../DESIGN-STRUCTURAL/drawings/generated/pdf/P4_back-stiffener-plate.pdf) | Back stiffener plate | 1 | 1 | 1 |
| [**A1**](../DESIGN-STRUCTURAL/drawings/generated/pdf/A1_back-stiffener-outer-vertical-angle.pdf) | Back stiffener outer vertical angle | 2 | 2 | 2 |
| [**A2**](../DESIGN-STRUCTURAL/drawings/generated/pdf/A2_back-stiffener-inner-vertical-angle.pdf) | Back stiffener inner vertical angle | 4 | 4 | 4 |

### Step 4 - Bolt the angles to the three front stiffener plates

`front-wall` - sub-assembly: *Front wall* - draft

| Add | Part | Qty | Placed so far | Per machine |
|---|---|---:|---:|---:|
| [**P5**](../DESIGN-STRUCTURAL/drawings/generated/pdf/P5_front-stiffener-plate-centre.pdf) | Front stiffener plate, centre | 1 | 1 | 1 |
| [**P6**](../DESIGN-STRUCTURAL/drawings/generated/pdf/P6_front-stiffener-plate-outer.pdf) | Front stiffener plate, outer | 2 | 2 | 2 |
| [**A10**](../DESIGN-STRUCTURAL/drawings/generated/pdf/A10_front-stiffener-outer-angle.pdf) | Front stiffener outer angle | 8 | 8 | 8 |
| [**A4**](../DESIGN-STRUCTURAL/drawings/generated/pdf/A4_frame-tube-mount-angle.pdf) | Frame tube mount angle | 2 | 2 | 26 |

### Step 5 - Bolt the bottom angle runs and tube angles to each motor plate

`motor-plates` - sub-assembly: *Motor plate* x2 - draft

| Add | Part | Qty | Placed so far | Per machine |
|---|---|---:|---:|---:|
| [**P8**](../DESIGN-STRUCTURAL/drawings/generated/pdf/P8_motor-mounting-plate.pdf) | Motor mounting plate | 2 | 2 | 2 |
| [**A6-1**](../DESIGN-STRUCTURAL/drawings/generated/pdf/A6-1_bottom-stiffener-angle-rear-segment.pdf) | Bottom stiffener angle, rear segment | 4 | 10 | 10 |
| [**A6-2**](../DESIGN-STRUCTURAL/drawings/generated/pdf/A6-2_bottom-stiffener-angle-middle-segment.pdf) | Bottom stiffener angle, middle segment | 4 | 10 | 10 |
| [**A6-3**](../DESIGN-STRUCTURAL/drawings/generated/pdf/A6-3_bottom-stiffener-angle-front-segment.pdf) | Bottom stiffener angle, front segment | 4 | 10 | 10 |
| [**A4**](../DESIGN-STRUCTURAL/drawings/generated/pdf/A4_frame-tube-mount-angle.pdf) | Frame tube mount angle | 8 | 10 | 26 |

### Step 6 - Stand both inner side panels on the floor and bolt them to its angles

`inner-panels` - draft

| Add | Part | Qty | Placed so far | Per machine |
|---|---|---:|---:|---:|
| [**P3**](../DESIGN-STRUCTURAL/drawings/generated/pdf/P3_side-panel-inner.pdf) | Side panel, inner | 2 | 2 | 2 |

**Installs:** `floor-angles`

> Brace the panels square and plumb before tightening.

### Step 7 - Bolt the motor plates to the floor between the inner panels

`motor-plates-install` - draft

**Installs:** `motor-plates`  
**After:** `inner-panels`

### Step 8 - Slide the front and rear cross tubes through the panel windows and bolt on the tube angles

`cross-tubes` - draft

| Add | Part | Qty | Placed so far | Per machine |
|---|---|---:|---:|---:|
| [**T1**](../DESIGN-STRUCTURAL/drawings/generated/pdf/T1_front-frame-cross-tube.pdf) | Front frame cross tube | 1 | 1 | 1 |
| [**T2**](../DESIGN-STRUCTURAL/drawings/generated/pdf/T2_rear-frame-cross-tube.pdf) | Rear frame cross tube | 1 | 1 | 1 |
| [**A4**](../DESIGN-STRUCTURAL/drawings/generated/pdf/A4_frame-tube-mount-angle.pdf) | Frame tube mount angle | 16 | 26 | 26 |

**After:** `inner-panels`, `motor-plates-install`  
**Tools:** [**J1**](../DESIGN-STRUCTURAL/drawings/generated/pdf/J1_tube-drilling-jig.pdf)

### Step 9 - Fit the left and right outer side panels

`outer-panels` - draft

| Add | Part | Qty | Placed so far | Per machine |
|---|---|---:|---:|---:|
| [**P1**](../DESIGN-STRUCTURAL/drawings/generated/pdf/P1_side-panel-outer-left.pdf) | Side panel, outer, left | 1 | 1 | 1 |
| [**P2**](../DESIGN-STRUCTURAL/drawings/generated/pdf/P2_side-panel-outer-right.pdf) | Side panel, outer, right | 1 | 1 | 1 |

**After:** `cross-tubes`

> P1 goes on the left and P2 on the right. They are not interchangeable.

### Step 10 - Bolt the rear wall between the side panels

`rear-wall-install` - draft

**Installs:** `rear-wall`  
**After:** `outer-panels`

> The drawing flags an interference with the inner panels. Check the fit first.

### Step 11 - Bolt the front wall to the panels and motor plates

`front-wall-install` - draft

**Installs:** `front-wall`  
**After:** `outer-panels`, `motor-plates-install`

## Wheel units (UWU)

### Step 12 - Weld each wheel hub (DOM tube, lug plate and four gussets)

`hub-weldment` - sub-assembly: *Wheel hub* x4 - draft

| Add | Part | Qty | Placed so far | Per machine |
|---|---|---:|---:|---:|
| [**T7**](../DESIGN-STRUCTURAL/drawings/generated/pdf/T7_wheel-hub-dom-tube.pdf) | Wheel hub DOM tube | 4 | 4 | 4 |
| [**P17**](../DESIGN-STRUCTURAL/drawings/generated/pdf/P17_wheel-hub-lug-plate.pdf) | Wheel hub lug plate | 4 | 4 | 4 |
| [**P18**](../DESIGN-STRUCTURAL/drawings/generated/pdf/P18_wheel-hub-gusset.pdf) | Wheel hub gusset | 16 | 16 | 16 |

### Step 13 - Fit the shafts, bearings, motors and hubs on the motor plates

`wheel-drives` - draft

| Add | Part | Qty | Placed so far | Per machine |
|---|---|---:|---:|---:|
| [**R6**](../DESIGN-STRUCTURAL/drawings/generated/pdf/R6_wheel-shaft.pdf) | Wheel shaft | 4 | 4 | 4 |

**Installs:** `hub-weldment`  
**After:** `motor-plates-install`, `outer-panels`

> Bearings, couplings, hydraulic motors, rims and tyres are purchased and not in the parts manifest yet.

## Loader arms

### Step 14 - Weld the pivot plates to each pivot DOM tube

`pivot-mounts` - sub-assembly: *Arm pivot mount* x2 - draft

| Add | Part | Qty | Placed so far | Per machine |
|---|---|---:|---:|---:|
| [**T6**](../DESIGN-STRUCTURAL/drawings/generated/pdf/T6_arm-pivot-dom-tube.pdf) | Arm pivot DOM tube | 2 | 2 | 2 |
| [**P11**](../DESIGN-STRUCTURAL/drawings/generated/pdf/P11_arm-pivot-mount-plate.pdf) | Arm pivot mount plate | 4 | 4 | 4 |

**Tools:** [**J2**](../DESIGN-STRUCTURAL/drawings/generated/pdf/J2_pivot-mount-welding-jig-base.pdf), [**J3**](../DESIGN-STRUCTURAL/drawings/generated/pdf/J3_pivot-mount-welding-jig-ring.pdf), [**J4**](../DESIGN-STRUCTURAL/drawings/generated/pdf/J4_pivot-welding-spacer-jig.pdf)

### Step 15 - Bolt each arm together (main tube between inner and outer plates, pivot mount, crossbeam angles)

`arms` - sub-assembly: *Loader arm* x2 - draft

| Add | Part | Qty | Placed so far | Per machine |
|---|---|---:|---:|---:|
| [**T4**](../DESIGN-STRUCTURAL/drawings/generated/pdf/T4_arm-main-tube.pdf) | Arm main tube | 2 | 2 | 2 |
| [**P9**](../DESIGN-STRUCTURAL/drawings/generated/pdf/P9_arm-side-plate-inner.pdf) | Arm side plate, inner | 2 | 2 | 2 |
| [**P10**](../DESIGN-STRUCTURAL/drawings/generated/pdf/P10_arm-side-plate-outer.pdf) | Arm side plate, outer | 2 | 2 | 2 |
| [**A5**](../DESIGN-STRUCTURAL/drawings/generated/pdf/A5_arm-crossbeam-mount-angle.pdf) | Arm crossbeam mount angle | 4 | 4 | 4 |

**Installs:** `pivot-mounts`

### Step 16 - Join the arms with the crossbeam and pin on the bucket-cylinder base lugs

`crossbeam` - draft

| Add | Part | Qty | Placed so far | Per machine |
|---|---|---:|---:|---:|
| [**T3**](../DESIGN-STRUCTURAL/drawings/generated/pdf/T3_arm-crossbeam-tube.pdf) | Arm crossbeam tube | 1 | 1 | 1 |
| [**U1**](../DESIGN-STRUCTURAL/drawings/generated/pdf/U1_arm-crossbeam-cylinder-lug.pdf) | Arm crossbeam cylinder lug | 2 | 2 | 2 |
| [**R3**](../DESIGN-STRUCTURAL/drawings/generated/pdf/R3_clevis-pin-3-4.pdf) | Clevis pin 3/4 | 2 | 2 | 4 |

**Installs:** `arms`

### Step 17 - Hang the arm frame on the machine with the pivot pins

`hang-arms` - draft

| Add | Part | Qty | Placed so far | Per machine |
|---|---|---:|---:|---:|
| [**R2**](../DESIGN-STRUCTURAL/drawings/generated/pdf/R2_arm-pivot-pin.pdf) | Arm pivot pin | 2 | 2 | 2 |
| [**F1**](../DESIGN-STRUCTURAL/drawings/generated/pdf/F1_hex-nut-1-1-2-6.pdf) | Hex nut 1-1/2-6 | 4 | 4 | 4 |

**After:** `crossbeam`, `outer-panels`

> One 1-1/2 nut on each end of each pin.

## Bucket

### Step 18 - Tack and weld the bucket (back, bottom, sides, cutting edge)

`bucket-weldment` - sub-assembly: *Bucket* - draft

| Add | Part | Qty | Placed so far | Per machine |
|---|---|---:|---:|---:|
| [**P12**](../DESIGN-STRUCTURAL/drawings/generated/pdf/P12_bucket-back-plate.pdf) | Bucket back plate | 1 | 1 | 1 |
| [**P13**](../DESIGN-STRUCTURAL/drawings/generated/pdf/P13_bucket-bottom-plate.pdf) | Bucket bottom plate | 1 | 1 | 1 |
| [**P14**](../DESIGN-STRUCTURAL/drawings/generated/pdf/P14_bucket-side-plate.pdf) | Bucket side plate | 2 | 2 | 2 |
| [**P19**](../DESIGN-STRUCTURAL/drawings/generated/pdf/P19_bucket-cutting-edge.pdf) | Bucket cutting edge | 1 | 1 | 1 |

> The drawing flags that the side plates' tab slots are modelled only 1 mm deep. Cut them through.

### Step 19 - Weld the pivot and cylinder lugs to the bucket back

`bucket-lugs` - draft

| Add | Part | Qty | Placed so far | Per machine |
|---|---|---:|---:|---:|
| [**U2**](../DESIGN-STRUCTURAL/drawings/generated/pdf/U2_bucket-pivot-lug.pdf) | Bucket pivot lug | 2 | 2 | 2 |
| [**U3**](../DESIGN-STRUCTURAL/drawings/generated/pdf/U3_bucket-cylinder-lug.pdf) | Bucket cylinder lug | 2 | 2 | 2 |

**Installs:** `bucket-weldment`

### Step 20 - Pin the bucket to the arm tips

`bucket-mount` - draft

| Add | Part | Qty | Placed so far | Per machine |
|---|---|---:|---:|---:|
| [**R4**](../DESIGN-STRUCTURAL/drawings/generated/pdf/R4_bucket-pivot-pin.pdf) | Bucket pivot pin | 2 | 2 | 2 |

**After:** `hang-arms`, `bucket-lugs`

## Cylinders

### Step 21 - Pin the lift cylinders between the inner panels and the arms

`lift-cylinders` - draft

| Add | Part | Qty | Placed so far | Per machine |
|---|---|---:|---:|---:|
| [**R1**](../DESIGN-STRUCTURAL/drawings/generated/pdf/R1_lift-cylinder-base-pin.pdf) | Lift cylinder base pin | 2 | 2 | 2 |

**After:** `hang-arms`

> Hydraulic cylinders are purchased and not in the parts manifest yet.

### Step 22 - Pin the bucket cylinders between the crossbeam lugs and the bucket lugs

`bucket-cylinders` - draft

| Add | Part | Qty | Placed so far | Per machine |
|---|---|---:|---:|---:|
| [**R3**](../DESIGN-STRUCTURAL/drawings/generated/pdf/R3_clevis-pin-3-4.pdf) | Clevis pin 3/4 | 2 | 4 | 4 |

**After:** `bucket-mount`, `crossbeam`

## Folding operator platform

### Step 23 - Bolt the side and transverse angles to the deck

`platform-weldment` - sub-assembly: *Platform* - draft

| Add | Part | Qty | Placed so far | Per machine |
|---|---|---:|---:|---:|
| [**P15**](../DESIGN-STRUCTURAL/drawings/generated/pdf/P15_platform-deck.pdf) | Platform deck | 1 | 1 | 1 |
| [**A7**](../DESIGN-STRUCTURAL/drawings/generated/pdf/A7_platform-side-angle-arm.pdf) | Platform side angle arm | 2 | 2 | 2 |
| [**A8**](../DESIGN-STRUCTURAL/drawings/generated/pdf/A8_platform-transverse-angle.pdf) | Platform transverse angle | 2 | 2 | 2 |

### Step 24 - Hinge the platform to the inner panels and fit the lock pins

`platform-install` - draft

| Add | Part | Qty | Placed so far | Per machine |
|---|---|---:|---:|---:|
| [**P16**](../DESIGN-STRUCTURAL/drawings/generated/pdf/P16_platform-pivot-bracket.pdf) | Platform pivot bracket | 2 | 2 | 2 |
| [**R5**](../DESIGN-STRUCTURAL/drawings/generated/pdf/R5_platform-pivot-pin.pdf) | Platform pivot pin | 2 | 2 | 2 |
| [**F2**](../DESIGN-STRUCTURAL/drawings/generated/pdf/F2_platform-lock-pin-5-8.pdf) | Platform lock pin 5/8 | 2 | 2 | 2 |

**Installs:** `platform-weldment`  
**After:** `outer-panels`

## Parts not yet placed

Every fabricated part is placed exactly as many times as the model uses it.

Not yet assigned to steps (quantities are estimates, not counted in the model): F3, F4, F5, F6, F7, F8, F9, F10.

## Checks

- all good
