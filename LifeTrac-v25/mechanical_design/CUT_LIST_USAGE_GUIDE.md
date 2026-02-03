# Structural Steel Cut List Usage Guide

## Overview

The structural steel cut list provides detailed manufacturing instructions for all angle iron and square tubing parts needed to build the LifeTrac v25 machine.

## What's Included

The PDF cut list (`structural_steel_cut_list.pdf`) contains **5 unique parts** requiring **38 total pieces** to fabricate:

| Part Code | Description | Material | Quantity |
|-----------|-------------|----------|----------|
| A1 | Platform Angle Arm | 2" × 2" × 1/4" Angle Iron | 2 |
| A2 | Frame Tube Mounting Angle | 2" × 2" × 1/4" Angle Iron | 16 |
| A3 | Loader Arm Mounting Angle | 2" × 2" × 1/4" Angle Iron | 4 |
| A4 | Side Panel Vertical Angle | 2" × 2" × 1/4" Angle Iron | 8 |
| A5 | Bottom Plate Horizontal Angle | 2" × 2" × 1/4" Angle Iron | 8 |

**Total:** 38 pieces

## How to Use the Cut List

### Step 1: Print the PDF

Download and print `structural_steel_cut_list.pdf`. Each page represents one unique part type.

### Step 2: Purchase Materials

All parts use standard **2" × 2" × 1/4" angle iron** stock. Calculate total length needed:

- A1: 2 × 425mm = 850mm (33.5")
- A2: 16 × 146mm = 2,336mm (92")
- A3: 4 × 146mm = 584mm (23")
- A4: 8 × 550mm = 4,400mm (173")
- A5: 8 × 400mm = 3,200mm (126")

**Total angle iron needed:** ~11.4 meters (37.5 feet)

💡 **Tip:** Purchase 20-foot lengths (6 meters) - you'll need 2 sticks with some extra for waste.

### Step 3: Cut to Length

For each part type:

1. Locate the page in the PDF (by part code)
2. Note the total length dimension (shown in both inches and millimeters)
3. Cut the specified quantity of pieces to this length
4. Use a metal cutoff saw or angle grinder with cutoff wheel
5. Check off the "Cut" operation on your printed sheet

### Step 4: Mark Hole Positions

Each part page includes an engineering drawing showing:
- Total length with dimension arrows
- Hole positions measured from one end
- Hole diameters (typically 1/2" or 3/8")

Use the dimensions to mark hole centers:
1. Measure from the end as shown on the drawing
2. Use a center punch to mark each hole position
3. Double-check measurements before drilling

### Step 5: Drill Holes

For each hole:
1. Reference the operations checklist on the part page
2. Use the specified drill bit size (e.g., "1/2" hole")
3. Drill through the indicated leg of the angle iron
4. Check off each drilling operation as you complete it

⚠️ **Important Notes:**
- Part A1 (Platform Angle Arm) has holes in BOTH legs
  - 2 holes through the vertical leg (pivot end)
  - 2 holes through the horizontal leg (deck end)
- All other parts have holes through one leg only
- Use cutting oil and drill slowly to prevent work hardening

### Step 6: Verify and Label

After completing each part type:
1. Verify all operations are checked off
2. Count pieces to ensure correct quantity
3. Mark each piece with the part code (use paint marker or label)
4. Store pieces organized by part code

## Assembly Reference

When assembling the machine:
- Each part is referenced by its code (A1, A2, etc.)
- The quantity tells you how many of each part should be used
- Part notes provide orientation and installation hints

## Advanced: Jigs and Templates

For high precision or multiple machines:

1. **Drilling jig:** Create a drilling template for each part
   - Use a piece of scrap angle iron as a template
   - Drill holes in perfect positions
   - Clamp template to workpiece for drilling

2. **Cut stop:** Set up a stop block on your saw for consistent lengths
   - Measure once, cut multiple pieces
   - Ensures all pieces are exactly the same length

## Automatic Updates

This cut list is automatically regenerated when:
- Design files change in the OpenSCAD source
- The GitHub Action workflow runs
- Always use the latest PDF from the repository

## Questions?

If you find errors or need clarification:
- Open an issue on the GitHub repository
- Reference the specific part code in your question
- Include photos if asking about fabrication techniques

---

**Part of:** LifeTrac v25 Open Source Utility Loader  
**License:** GPL v3  
**Organization:** Open Source Ecology
