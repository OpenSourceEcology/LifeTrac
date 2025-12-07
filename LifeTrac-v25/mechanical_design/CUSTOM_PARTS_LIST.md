# LifeTrac v25 Custom Parts List

This document identifies parts that are NOT standard CNC-cut plate steel or standard square/round tubing. These parts require custom fabrication using standardized methods.

**Excluded from this list:** Hydraulic components (cylinders, motors, valves), wheels/tires, fasteners (bolts, pins), and purchased components.

---

## STANDARDIZED PART TYPES (Implemented in OpenSCAD)

### TYPE A: U-Channel Lug (from Square Tubing)
**Fabrication:** Torch-cut top and bottom of square tube to create U-shape with pivot hole through sides.

| Location | Source Tube | Length | Hole Dia | Qty |
|----------|-------------|--------|----------|-----|
| Frame lift cyl base mount | 4"x4"x1/4" | 100mm | 27mm (1"+2) | 2 |
| Arm lift cyl attachment | 3"x3"x1/4" | 80mm | 27mm (1"+2) | 2 |
| Cross beam bucket cyl mount | 3"x3"x1/4" | 80mm | 21mm (3/4"+2) | 2 |
| QA plate bucket cyl mount | 3"x3"x1/4" | 80mm | 21mm (3/4"+2) | 2 |

**Total Type A Brackets: 8**

---

### TYPE B: Large Pivot Ring (CNC Plasma Cut)
**Material:** 3/4" plate steel
**Dimensions:** 106mm OD, 40mm ID

| Location | Qty per Arm | Total |
|----------|-------------|-------|
| Arm pivot reinforcement (2 rings per arm, inner and outer) | 2 | 4 |

**Total Type B Rings: 4**

---

### TYPE C: Small Pivot Ring (CNC Plasma Cut)  
**Material:** 1" plate steel
**Dimensions:** 70mm OD, 40mm ID

| Location | Qty |
|----------|-----|
| QA plate arm connection flanges | 2 |

**Total Type C Rings: 2**

---

### ROUND BAR STOCK (1.5" / 38mm diameter)
**Material:** 4140 steel or equivalent

| Part | Length | Qty |
|------|--------|-----|
| Arm pivot pin | ~160mm | 2 |
| Bucket pivot pin | ~1000mm | 1 |
| QA hook bar | ~1018mm | 1 |

**Total Round Bar Pieces: 4**

---

### CNC PLATE PARTS

| Part | Material | Dimensions | Qty |
|------|----------|------------|-----|
| QA arm flange plate | 1/2" plate | 80x100mm | 2 |
| Standing deck plate | 1/4" plate | 700x400mm | 1 |

---

### SQUARE TUBE SECTIONS (Cut to Length)

| Part | Source Tube | Length | Qty |
|------|-------------|--------|-----|
| Deck connection bracket | 3"x3"x1/4" | 100mm | 2 |
| Deck vertical support | 2"x2"x1/4" | ~244mm | 2 |
| Deck cross brace | 2"x2"x1/4" | 600mm | 1 |

---

## FABRICATION SUMMARY

### Type A U-Channel Fabrication Steps:
1. Cut square tube to specified length
2. Mark cut lines for U-channel (remove top and bottom faces)
3. Torch cut or plasma cut the marked areas
4. Grind cut edges smooth
5. Drill pivot hole through both remaining walls
6. Deburr hole edges

### Type B/C Ring Fabrication:
1. Nest all rings on plate steel for efficient cutting
2. CNC plasma or laser cut
3. Deburr inner and outer edges

---

## PARTS COUNT SUMMARY

| Part Type | Description | Quantity |
|-----------|-------------|----------|
| Type A (4"x4") | U-Channel Lug, 27mm hole | 2 |
| Type A (3"x3") | U-Channel Lug, 27mm hole | 2 |
| Type A (3"x3") | U-Channel Lug, 21mm hole | 4 |
| Type B Ring | Large Pivot Ring (3/4" plate) | 4 |
| Type C Ring | Small Pivot Ring (1" plate) | 2 |
| Round Bar | 1.5" dia, various lengths | 4 |
| CNC Plate | Flange plates | 2 |
| CNC Plate | Deck plate | 1 |
| Tube Section | 3"x3" bracket | 2 |
| Tube Section | 2"x2" deck frame | 3 |

**Total Custom Fabricated Parts: 26**
**Unique Fabrication Types: 3** (U-Channel, CNC Ring, Cut Tube)
