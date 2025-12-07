# LifeTrac v25 CNC Layout - Parts List

## Overview
The cnclayout.svg file contains all sheet metal parts for plasma cutting in the LifeTrac v25 design. This document lists all parts included in the layout.

## Parts Included (23 total parts)

### Frame Components (Half-inch plate, 1/2" = 12.7mm)

#### Triangular Side Panels (4 parts - Sandwich Design)
- **A1-L-Outer** - Left Outer Side Panel (1000mm × 1400mm)
- **A1-L-Inner** - Left Inner Side Panel (1000mm × 1400mm)
- **A1-R-Inner** - Right Inner Side Panel (1000mm × 1400mm)
- **A1-R-Outer** - Right Outer Side Panel (1000mm × 1400mm)

Note: These four panels form a sandwich structure with 100mm spacing. The loader arm pivots between the inner pair.

#### Wheel Mounts (4 parts)
- **A4-1** - Wheel Mount Front Left (250mm × 250mm)
- **A4-2** - Wheel Mount Front Right (250mm × 250mm)
- **A4-3** - Wheel Mount Rear Left (250mm × 250mm)
- **A4-4** - Wheel Mount Rear Right (250mm × 250mm)

#### Hydraulic Cylinder Mounts (4 parts)
- **A5-L** - Lift Cylinder Mount Left (100mm × 150mm)
- **A5-R** - Lift Cylinder Mount Right (100mm × 150mm)
- **Bucket Cyl Lug 1** - Bucket Cylinder Lug (100mm × 150mm)
- **Bucket Cyl Lug 2** - Bucket Cylinder Lug (100mm × 150mm)

#### Bucket Attachment Points (2 parts)
- **C2-1** - Bucket Attach Left (200mm × 200mm)
- **C2-2** - Bucket Attach Right (200mm × 200mm)

### Structural Components (Quarter-inch plate, 1/4" = 6.35mm)

#### Frame Crossmember (1 part)
- **A2** - Rear Crossmember (1100mm × 600mm)

#### Loader Arm Components (2 parts)
- **C1-1** - Arm Reinforcement Left (150mm × 1200mm)
- **C1-2** - Arm Reinforcement Right (150mm × 1200mm)

#### Bucket Components (4 parts)
- **E1-1** - Bucket Bottom (1100mm × 600mm)
- **E1-2** - Bucket Back (1100mm × 400mm)
- **E1-3** - Bucket Side Left (600mm × 400mm)
- **E1-4** - Bucket Side Right (600mm × 400mm)

#### Operator Platform (1 part)
- **F1** - Standing Deck (1000mm × 400mm)

#### Control Housing (1 part)
- **G1** - Housing Base (300mm × 200mm)

## Material Specifications

### Half-inch Plate (12.7mm thick) - Red in layout
- 4× Triangular side panels (main structural frame)
- 4× Wheel mount plates
- 4× Hydraulic cylinder mounts
- 2× Bucket attachment plates
- **Total: 14 parts**

### Quarter-inch Plate (6.35mm thick) - Blue in layout
- 1× Rear crossmember
- 2× Arm reinforcements
- 4× Bucket panels
- 1× Standing deck
- 1× Housing base
- **Total: 9 parts**

## Cutting Specifications
- All corners have 6.35mm (1/4") radius
- Maintain 3mm spacing between cuts
- Layout optimized for minimal material waste
- Total layout size: 2019mm × 6867mm

## Design Notes
- The four triangular panels create a unique "sandwich" design where:
  - Outer panels provide main structural support
  - Inner panels are sandwiched 100mm from outer panels
  - Loader arm pivots between the inner pair of panels
- All parts designed for CNC plasma cutting
- Rounded corners reduce stress concentrations

## Fixed Issues
The original cnclayout.svg was only rendering text labels and missing all part geometries. This was fixed by:
1. Converting 3D projection() calls to direct 2D geometry
2. Ensuring all shapes are rendered at the same 2D level as text
3. Regenerating the SVG file with OpenSCAD

All 23 parts are now correctly rendered in the SVG file.
