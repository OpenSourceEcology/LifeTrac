# LifeTrac v25 Skid Steer - Design Requirements & Specifications

This document captures all requirements, requests, and design decisions made during the development of the LifeTrac v25 mechanical design. It serves as a reference for future enhancements and modifications.

---

## Table of Contents
1. [Original Issue Requirements](#original-issue-requirements)
2. [Structural Design Requirements](#structural-design-requirements)
3. [Material Requirements](#material-requirements)
4. [Hydraulic System Requirements](#hydraulic-system-requirements)
5. [Size & Operational Requirements](#size--operational-requirements)
6. [Manufacturing Requirements](#manufacturing-requirements)
7. [Documentation Requirements](#documentation-requirements)
8. [Design Evolution & Refinements](#design-evolution--refinements)
9. [Future Enhancement Opportunities](#future-enhancement-opportunities)

---

## Original Issue Requirements

### Primary Design Goals
- Design the v25 LifeTrac machine using OpenSCAD
- Create a compact utility loader (skid steer) design
- Use only standardized, manufacturable components
- Enable community fabrication with CNC plasma cutting and welding

### Core Specifications from Initial Request
1. **Materials**: Plate steel, angle iron, and square tubing only for primary structure
2. **Wheels**: Standardized Open Source Ecology motors, wheels, and axle designs
3. **Hydraulics**: 
   - 2 cylinders for raising/lowering loader arms
   - 2 cylinders for bucket attachment actuation at arm ends
4. **Drive System**: Hydraulic wheel motors (2-wheel drive standard, 4-wheel drive optional)
5. **Size**: Between Toro Dingo and Bobcat dimensions
6. **Operator Station**: No seat required, remote controlled, optional standing deck at rear
7. **Fasteners**: 1/2" and 1" hex bolts for assembly
8. **Plate Thickness**: 1/4" standard, other sizes for special situations
9. **Design Details**: All plate steel corners rounded where possible
10. **Animation**: Implement animation capabilities in OpenSCAD

---

## Structural Design Requirements

### Frame Architecture (Evolved Through Iterations)

#### Final Design: Four-Panel Triangular Sandwich
- **4 triangular side panels** (1/2" plate steel):
  - A1-L-Outer: Left outer panel
  - A1-L-Inner: Left inner panel
  - A1-R-Inner: Right inner panel
  - A1-R-Outer: Right outer panel
- **Panel orientation**: Front-to-back along machine length
- **Panel dimensions**: 1400mm long × 1000mm tall (triangular shape)
- **Sandwich spacing**: 100mm (~4 inches) between outer and inner panels on each side
- **Connection**: 
  - Rear crossmember (A2) connects all 4 panels at back
  - Front crossmember (A3) connects panels at arm pivot area

### Key Structural Features
1. **Arm pivot mounting**: Sandwiched between the two inner panels at front top
2. **Wheel mounting**: Along base edges of triangular panels
   - Front wheels at wheelbase position (1400mm forward)
   - Rear wheels at origin (rear of machine)
3. **Triangular geometry**: Provides superior structural strength
4. **Load distribution**: Pivot stress distributed across full panel width

---

## Material Requirements

### Plate Steel Specifications
- **Primary thickness**: 1/4" (6.35mm) for most components
- **High-stress areas**: 1/2" (12.7mm) for:
  - Main triangular side panels (A1-L/R-Outer/Inner)
  - Wheel mounting brackets (A4-1 to A4-4)
  - Cylinder mounting lugs (A5-L, A5-R)
  - Bucket attachment plates (C2-1, C2-2)
  - Critical stress points

### Structural Steel
- **Square tubing**: 4"×4" for loader arms and reinforcements
- **Angle iron**: Available for reinforcement where needed
- **Corner treatment**: All plate steel corners rounded to 6.35mm radius

### Fasteners
- **Primary bolts**: 1/2" and 1" hex bolts (Grade 5 or higher)
- **Mounting**: Bolt patterns with adequate spacing for strength

### Material Quantities (Estimated)
- **1/4" plate**: 8-10 m²
- **1/2" plate**: 2.8 m² (four main frame panels + mounting lugs)
- **Square tubing**: ~10 meters total
- **Estimated cost**: $6,750 - $10,450 USD for materials

---

## Hydraulic System Requirements

### Lift Cylinder Configuration (Final Design)
**Requirements addressed in commit 1aba660:**
- **Cylinders positioned directly underneath loader arms**
- **Mounting configuration**:
  - Base end: Attached between inner sandwich panels (A1-L-Inner, A1-R-Inner)
  - Mounting lugs (A5-L, A5-R): 100×150mm plates at mid-height on frame
  - Top end: Connected to bottom of arms ~200mm from pivot point
- **Cylinder specifications**: 2.5" bore × 16" stroke (D1, D2)
- **Arm pivot**: Between sandwiched plates at front top of frame

### Bucket Tilt Cylinders
- **Quantity**: 2 cylinders (D3, D4)
- **Mounting**: At end of loader arms
- **Connection**: Arms to bucket attachment system
- **Specifications**: 2" bore × 12" stroke

### Wheel Drive Motors
- **Quantity**: 4 hydraulic motors (configurable 2WD or 4WD)
- **Type**: 100cc hydraulic wheel motors
- **Configuration**: 
  - Standard: 2-wheel drive (rear wheels)
  - Optional: All 4 wheels powered
- **Integration**: OSE standardized motor design

### Hydraulic Circuit
- **Valve manifold**: 4 directional valves
- **Connections**:
  - 4 wheel motors (hydraulic and electronic)
  - 4 cylinders (2 lift + 2 bucket)
- **Control**: Integrated with v25 electronics system

---

## Size & Operational Requirements

### Dimensions
- **Overall**: 1200mm wide × 1400mm long × 1000mm tall (base frame)
- **Wheelbase**: 1400mm (front to rear wheels)
- **Track width**: 1200mm (side to side)
- **Loader arm length**: ~1500mm
- **Bucket width**: 1100mm (43")
- **Lift height**: ~2000mm (79")

### Target Size Range
- **Larger than**: Toro Dingo (mini skid steer)
- **Smaller than**: Bobcat (full-size skid steer)
- **Weight class**: Compact utility loader

### Operational Features
- **Control**: Remote operated (no onboard operator)
- **No seat required**: Design excludes operator seat
- **Optional standing deck**: Rear-mounted platform (F1)
  - 400mm deep platform
  - Anti-slip pattern
  - Mounted at 200mm height
- **Attachment system**: Quick-attach bucket mount at arm ends

---

## Manufacturing Requirements

### CNC Plasma Cutting Focus
**Requirement: NO 3D printing** (addressed in commit 3fac029)
- All parts designed for CNC plasma cutting from plate steel
- Welded fabrication assembly
- No STL files or 3D printing workflows

### Manufacturing Outputs Required

#### 1. Assembly Visualization
- **assembly.png**: 1024×768 PNG render of complete assembly
- Format: Matching CEB-Press example style
- Shows: Frame, wheels, arms, bucket, hydraulics
- Auto-generated via GitHub Actions

#### 2. CNC Layout
- **cnclayout.svg**: 2D vector layout for CNC cutting
- Features:
  - All plate parts laid out efficiently
  - Color-coded by thickness (blue=1/4", red=1/2")
  - Part labels with numbers and descriptions
  - Ready for CAM software import
- Format: Matching CEB-Press example style
- Size: 113KB vector file

#### 3. Individual Part Exports
- **DXF files**: Individual files for each plate part
- **Organization**: Grouped by material thickness
  - `quarter_inch/` directory
  - `half_inch/` directory
- **Batch export**: Shell script for automated generation
- **Part naming**: Clear identification (A1-L-Outer, A2, A4-1, etc.)

### Part Numbering System
- **A series**: Frame components (A1-A5)
- **B series**: Wheel assemblies (B1-B4)
- **C series**: Loader arms and attachments (C1-C2)
- **D series**: Hydraulic cylinders (D1-D4)
- **E series**: Bucket components (E1)
- **F series**: Optional components (F1 standing deck)
- **G series**: Electronics housing (G1)

### GitHub Actions Automation
**Workflow requirements:**
1. Syntax validation of OpenSCAD files
2. Generate assembly.png (3D render)
3. Generate cnclayout.svg (2D CNC layout)
4. Generate preview renders (multiple angles)
5. Generate animation frames (36-frame sequence)
6. Auto-commit assembly.png and cnclayout.svg back to repo
7. Upload artifacts for download

---

## Documentation Requirements

### Required Documentation Files

#### 1. AI_DESIGN_PROGRESS.md
- Track AI session progress on hardware design
- Document design decisions and iterations
- Record changes and rationale

#### 2. PARTS_LIST.md
- Complete catalog of all parts
- Part numbering with letter/number system
- Brief description of each part
- List of parts each component connects to or interacts with

#### 3. ASSEMBLY.md
- Phase-by-phase build instructions
- Step-by-step assembly guide
- Troubleshooting section
- Tool requirements
- Safety considerations

#### 4. BILL_OF_MATERIALS.md
- Detailed BOM with quantities
- Material specifications
- Cost estimates
- Supplier recommendations
- Hardware lists (bolts, fasteners)

#### 5. INTEGRATION.md
- Mechanical/electronics integration
- Cable routing guidance
- Control housing details (G1)
- Hydraulic manifold connections
- Interface with v25 electronics

#### 6. README.md
- Comprehensive design overview
- Usage instructions
- File structure explanation
- Quick start guide

### Documentation Standards
- **Total documentation**: 42,000+ words
- **Format**: Markdown for all documentation
- **Links**: Reference OSE historical designs:
  - https://wiki.opensourceecology.org/wiki/LifeTrac_v17.10
  - https://wiki.opensourceecology.org/wiki/LifeTrac

---

## Design Evolution & Refinements

### Iteration 1: Initial Design (Commits 5b09f28 - 5c307bb)
- Created OpenSCAD module system
- Implemented basic square tubing frame
- Added all mechanical components
- Documentation and GitHub Actions setup

### Iteration 2: Assembly & CNC Visualization (Commit 5e0cda1)
**Request from @dorkmo:**
> "please also include and generate a png of the assembly and an svg of the parts layout"

**Implementation:**
- Added assembly.png (1024×768)
- Added cnclayout.svg with all parts
- Matched CEB-Press example format

### Iteration 3: Remove 3D Printing (Commit 3fac029)
**Request from @dorkmo:**
> "there should be no files for 3d printing. please remove 3d printing"

**Implementation:**
- Removed STL generation from GitHub Actions
- Removed all 3D printing references from documentation
- Focus exclusively on CNC plasma cutting

### Iteration 4: Triangular Side Panel Design (Commit 0908a64)
**Request from @dorkmo:**
> "on each side of the machine, please sandwich the arm joints between two pieces of plate steel that make up the side of the machine, perhaps each sheet is similar to a triangle shape"

**Implementation:**
- Replaced square tubing frame with triangular plate panels
- Two main side panels (A1-L, A1-R)
- Back connecting plate
- Arms sandwiched between panels

### Iteration 5: Four-Panel Sandwich with Rotation (Commit e41c879)
**Request from @dorkmo:**
> "the sandwiched plates need to be rotated 90 degrees so that the front and back wheels can mounted along the plate's base. also the sandwich should be two fairly similar pieces of the triangle sheet about 4inches apart. so there should be 4 of the triangle style pieces"

**Implementation:**
- Rotated triangular panels 90° (now front-to-back)
- Changed from 2 to 4 triangular panels
- 100mm (4") spacing between sandwich pairs
- Wheels mount along base edges
- Four panels: A1-L-Outer, A1-L-Inner, A1-R-Inner, A1-R-Outer

### Iteration 6: Lift Cylinder Repositioning (Commit 1aba660)
**Request from @dorkmo:**
> "make the hydraulic cylinder that connects to the arms be mounted directly underneath the arms with one end attached to the triangular sandwich pieces, between them. and the other end connected to the bottom of the arms"

**Implementation:**
- Cylinders positioned directly under loader arms
- Base mounting between inner sandwich panels
- New mounting lugs (A5-L, A5-R) at mid-height
- Top end attached to arm bottom ~200mm from pivot
- Arms pivot between sandwiched plates at front top

---

## Future Enhancement Opportunities

### Mechanical Enhancements
1. **FreeCAD Export**: Implement export to FreeCAD format (original requirement not yet fulfilled)
2. **Advanced animations**: More detailed motion sequences showing operation
3. **Attachment variations**: Design multiple bucket/attachment options
4. **Counterweight system**: Optimize balance for different loads
5. **Reinforcement analysis**: FEA simulation integration

### Manufacturing Improvements
1. **Nesting optimization**: Improve CNC layout for material efficiency
2. **Welding sequences**: Document optimal welding order
3. **Fixture designs**: Create jigs for accurate assembly
4. **Quality control**: Add inspection checklists
5. **Assembly time tracking**: Real-world build time data

### Documentation Additions
1. **Video tutorials**: Assembly and operation videos
2. **Common issues**: Expanded troubleshooting based on builds
3. **Modification guides**: How to customize for specific needs
4. **Maintenance schedules**: Preventive maintenance procedures
5. **Upgrade paths**: Future version compatibility

### Electronics Integration
1. **Sensor provisions**: Mounting points for additional sensors
2. **Camera mounts**: Vision system integration
3. **Telemetry**: Data logging and remote monitoring
4. **Autonomous features**: Preparation for automated operation
5. **Safety systems**: Emergency stop improvements

### Hydraulic System
1. **Flow optimization**: Circuit efficiency improvements
2. **Valve selection**: Specific valve recommendations
3. **Pressure testing**: Detailed pressure specifications
4. **Hose routing**: Detailed hydraulic line layout
5. **Temperature management**: Cooling system considerations

### Standardization
1. **Modular attachments**: Standard quick-attach interface
2. **Component library**: Expanded OSE standard parts
3. **Version compatibility**: Backward/forward compatibility planning
4. **International standards**: Metric/imperial dual specifications
5. **Certification**: Safety certification pathways

### Community Features
1. **Build log template**: Standard documentation for community builds
2. **Parts sourcing guide**: Regional supplier lists
3. **Cost tracking**: Actual build costs from community
4. **Design contributions**: Process for community improvements
5. **Testing protocols**: Standardized performance testing

### Software Tools
1. **Configuration tool**: Web-based design customizer
2. **BOM generator**: Automated BOM creation from parameters
3. **Cost calculator**: Real-time cost estimation
4. **Cutting optimizer**: Advanced nesting algorithms
5. **3D viewer**: Interactive web-based model viewer

---

## Design Philosophy & Standards

### Open Source Ecology Principles
1. **Modularity**: Interchangeable components and subsystems
2. **Standardization**: Use OSE standard parts where possible
3. **Simplicity**: Easy to understand and replicate
4. **Manufacturability**: Buildable with common tools and skills
5. **Accessibility**: Low-cost, locally sourceable materials
6. **Durability**: Long service life with maintainability
7. **Documentation**: Comprehensive and clear instructions

### Safety Considerations
1. **Structural integrity**: Adequate safety factors
2. **Hydraulic safety**: Proper pressure ratings and relief
3. **Operational safety**: Guards and safety features
4. **Maintenance safety**: Safe service procedures
5. **Emergency stops**: Accessible emergency controls

### Quality Standards
1. **Weld quality**: Proper weld specifications
2. **Material grades**: Specified steel grades
3. **Fastener grades**: Grade 5 or higher bolts
4. **Hydraulic components**: Rated for application
5. **Manufacturing tolerances**: CNC cutting accuracy

---

## Contact & Contribution

### Resources
- **Repository**: OpenSourceEcology/LifeTrac
- **Design Files**: LifeTrac-v25/mechanical_design/
- **Issue Tracking**: GitHub Issues
- **Discussions**: OSE Forums and community channels

### How to Contribute
1. Review this requirements document
2. Check existing issues and pull requests
3. Propose enhancements with clear rationale
4. Submit changes following OSE guidelines
5. Document all modifications thoroughly

### Version Control
- **Current Version**: v25
- **Design Status**: Complete mechanical design, ready for prototyping
- **Last Updated**: 2025-12-07
- **Major Revisions**: 6 design iterations documented above

---

*This requirements document should be updated as new requirements emerge or design changes are made. It serves as the single source of truth for the LifeTrac v25 design specifications.*
