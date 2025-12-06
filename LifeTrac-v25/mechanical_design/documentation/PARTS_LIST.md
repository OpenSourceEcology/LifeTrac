# LifeTrac v25 - Mechanical Parts List

## Part Numbering System
- **A** = Frame and Structural Components
- **B** = Wheel Assemblies
- **C** = Loader Arm System
- **D** = Hydraulic Cylinders
- **E** = Bucket Assembly
- **F** = Optional Components
- **G** = Electronics Housing

---

## A - Frame and Structural Components

### A1 - Base Frame Assembly
**Description:** Main structural frame constructed from 4"x4" square tubing  
**Material:** 4"x4"x1/4" wall square tubing (101.6mm x 6.35mm wall)  
**Quantity:**
- 2x longitudinal members @ 1800mm length
- 3x cross members @ 1200mm length  
- 2x vertical uprights @ 1000mm length
- 1x top cross member @ 1200mm length

**Connects to:**
- A2 (Wheel Mounting Plates)
- C1 (Loader Arms)
- F1 (Standing Deck)
- G1 (Control Housing)

**Fasteners:** 1" hex bolts for frame connections

---

### A2 - Wheel Mounting Plates (Set of 4)
**Description:** Heavy-duty mounting plates for wheel assemblies  
**Material:** 1/2" (12.7mm) plate steel, 300mm x 300mm with rounded corners  
**Quantity:** 4 (one per wheel position)  
**Features:**
- 4x mounting holes per plate (13mm diameter for 1/2" bolts)
- Holes at 40mm from edges
- 6.35mm corner radius

**Connects to:**
- A1 (Base Frame)
- B1, B2, B3, B4 (Wheel Assemblies)

**Fasteners:** 1/2" hex bolts (16 total - 4 per plate)

---

## B - Wheel Assemblies

### B1 - Front Left Wheel Assembly
**Description:** Complete wheel unit with hydraulic motor drive  
**Components:**
- 500mm diameter wheel with tire
- 150mm hub assembly
- Hydraulic motor (100cc displacement)
- Motor mounting bracket
- Axle (50.8mm diameter)
- Bearings and housing

**Connects to:** A2 (Wheel Mounting Plate FL)

**Fasteners:** 1/2" hex bolts

---

### B2 - Front Right Wheel Assembly
**Description:** Complete wheel unit with hydraulic motor drive  
**Components:** Same as B1  
**Connects to:** A2 (Wheel Mounting Plate FR)

---

### B3 - Rear Left Wheel Assembly
**Description:** Complete wheel unit with hydraulic motor drive  
**Components:** Same as B1  
**Connects to:** A2 (Wheel Mounting Plate RL)

---

### B4 - Rear Right Wheel Assembly
**Description:** Complete wheel unit with hydraulic motor drive  
**Components:** Same as B1  
**Connects to:** A2 (Wheel Mounting Plate RR)

---

## C - Loader Arm System

### C1 - Loader Arm Structure (Pair)
**Description:** Main lifting arms for loader function  
**Material:**
- 3"x3"x1/4" wall square tubing (76.2mm x 6.35mm) @ 1200mm length
- 1/4" (6.35mm) plate steel reinforcement @ 150mm x 1200mm

**Quantity:** 2 (left and right arms)  
**Features:**
- Pivot mounting points at base
- Cylinder mounting lugs
- Bucket attachment interface at end
- Cross brace connection points

**Connects to:**
- A1 (Base Frame - pivot points)
- C2 (Bucket Attachment)
- D1, D2 (Lift Cylinders)

**Fasteners:**
- 1" hex bolts for pivot pins
- 1/2" hex bolts for reinforcement plates

---

### C2 - Bucket Attachment Interface
**Description:** Quick-attach mounting system for bucket  
**Material:**
- 1/2" (12.7mm) plate steel mounting plates
- 4"x4" square tubing cross member

**Connects to:**
- C1 (Loader Arms)
- E1 (Bucket)
- D3, D4 (Bucket Cylinders)

**Fasteners:** 1" hex bolts for quick-attach pins

---

## D - Hydraulic Cylinders

### D1 - Left Lift Cylinder
**Description:** Hydraulic cylinder for raising left loader arm  
**Specifications:**
- Bore: 63.5mm (2.5")
- Rod: 31.75mm (1.25")
- Stroke: 400mm
- Mounting: Clevis ends

**Connects to:**
- A1 (Base Frame)
- C1 (Left Loader Arm)

---

### D2 - Right Lift Cylinder
**Description:** Hydraulic cylinder for raising right loader arm  
**Specifications:** Same as D1  
**Connects to:**
- A1 (Base Frame)
- C1 (Right Loader Arm)

---

### D3 - Left Bucket Cylinder
**Description:** Hydraulic cylinder for bucket tilt control  
**Specifications:**
- Bore: 50mm (2")
- Rod: 25mm (1")
- Stroke: 300mm
- Mounting: Clevis ends

**Connects to:**
- C1 (Left Loader Arm)
- C2 (Bucket Attachment)

---

### D4 - Right Bucket Cylinder
**Description:** Hydraulic cylinder for bucket tilt control  
**Specifications:** Same as D3  
**Connects to:**
- C1 (Right Loader Arm)
- C2 (Bucket Attachment)

---

## E - Bucket Assembly

### E1 - Bucket Assembly
**Description:** Material handling bucket with rounded edges  
**Material:** 1/4" (6.35mm) plate steel with 6.35mm corner radius  
**Dimensions:**
- Width: 1100mm
- Depth: 600mm
- Height: 400mm

**Components:**
- Bottom plate: 1100mm x 600mm
- Back plate: 1100mm x 400mm
- Left side plate: 600mm x 400mm
- Right side plate: 600mm x 400mm
- Reinforcement ribs (as needed)

**Connects to:** C2 (Bucket Attachment)

**Fasteners:** 1/2" hex bolts for assembly

---

## F - Optional Components

### F1 - Standing Deck
**Description:** Operator standing platform at rear of machine  
**Material:** 1/4" (6.35mm) plate steel with anti-slip pattern  
**Dimensions:**
- Width: 1000mm
- Depth: 400mm
- Anti-slip holes: 20mm diameter, 100mm spacing

**Support Structure:**
- 2x 2"x2" square tubing legs @ 100mm height

**Connects to:** A1 (Base Frame - rear)

**Fasteners:** 1/2" hex bolts

---

## G - Electronics Housing

### G1 - Control System Housing
**Description:** Weatherproof enclosure for electronics  
**Material:**
- Sheet metal housing (painted/powder coated)
- 1/4" mounting plate base

**Dimensions:**
- Width: 300mm
- Height: 400mm
- Depth: 200mm

**Houses:**
- Arduino Opta WiFi controller
- Arduino Opta extensions
- Hydraulic valve controllers
- Wiring and connections

**Connects to:** A1 (Base Frame - center)

**Fasteners:** 1/2" hex bolts

---

## Summary Statistics

### Plate Steel Requirements
| Thickness | Approximate Total Area | Typical Sheet Sizes |
|-----------|----------------------|---------------------|
| 1/4" (6.35mm) | 8-10 m² | Cut from 4'x8' sheets |
| 1/2" (12.7mm) | 1-2 m² | Cut from 4'x8' sheets |

### Square Tubing Requirements
| Size | Total Length | Uses |
|------|-------------|------|
| 4"x4"x1/4" | ~7000mm | Main frame |
| 3"x3"x1/4" | ~3000mm | Loader arms |
| 2"x2"x1/4" | ~500mm | Deck supports |

### Fasteners Summary
| Size | Approximate Quantity | Primary Uses |
|------|---------------------|--------------|
| 1" hex bolts | 40-50 | Frame joints, pivot pins |
| 1/2" hex bolts | 80-100 | Plate mounting, general assembly |

### Hydraulic Components
- 4x Hydraulic cylinders (2 large, 2 medium)
- 4x Hydraulic motors (100cc)
- Hydraulic hoses and fittings (as per hydraulic diagram)

---

## Manufacturing Notes

### CNC Plasma Cutting
All plate steel components should be nested for efficient cutting:
- Group by thickness
- Maintain 3mm minimum spacing between parts
- Include alignment holes for assembly
- Mark part numbers on each piece

### Welding Requirements
- All square tubing joints: Full penetration welds
- Plate to tubing: Fillet welds, both sides where accessible
- Stress points: Reinforcement gussets recommended

### Finishing
- Grind all sharp edges
- Verify all corners are properly rounded
- Prime and paint all steel components
- Apply anti-slip coating to standing deck

### Quality Control
- Verify all hole alignments before welding
- Check frame squareness (diagonal measurements)
- Test fit all bolt connections
- Ensure smooth operation of pivot points

---

**Document Version:** 1.0  
**Last Updated:** 2025-12-06  
**Design Reference:** lifetrac_v25.scad
