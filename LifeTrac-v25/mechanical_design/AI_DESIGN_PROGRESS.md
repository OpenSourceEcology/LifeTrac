# LifeTrac v25 Mechanical Design - AI Progress Tracker

## Project Overview
This document tracks the AI-assisted hardware design process for the LifeTrac v25 machine using OpenSCAD.

## Design Goals
- Create a compact utility loader between Toro Dingo and Bobcat size
- Remote operated (no seat required) with optional standing deck
- Use standardized plate steel, angle iron, and square tubing
- Utilize Open Source Ecology standard wheels, axles, and hydraulic motors
- Enable automated manufacturing with CNC-ready outputs

## Design Specifications

### Structural Components
- **Primary Material**: 1/4" plate steel (standard)
- **Alternative Materials**: 1/2" plate steel (high-stress areas), angle iron, square tubing
- **Fasteners**: 1/2" and 1" hex bolts
- **Design Feature**: Rounded corners on all plate steel where possible

### Hydraulic System
- **Arm Lift**: 2 hydraulic cylinders
- **Bucket Actuation**: 2 hydraulic cylinders  
- **Wheel Drive**: Hydraulic motors (2 standard, 4 optional)
- **Mounting**: Between plate steel, angle iron, or square tubing

### Size Reference
- Between Toro Dingo (compact) and Bobcat (mid-size)
- No onboard operator seat
- Optional standing deck at rear

## Progress Log

### Session 1 - Initial Setup (2025-12-06)

#### ✅ Completed
- Created mechanical_design directory structure
  - `/openscad` - Main design files
  - `/modules` - Reusable component modules
  - `/output` - Generated STL, DXF, and rendering files
  - `/documentation` - Part lists, BOM, assembly instructions
- Created AI_DESIGN_PROGRESS.md tracking document
- Reviewed existing LifeTrac versions for design inspiration
- Analyzed v25 electronic control system architecture
- Created OpenSCAD component modules:
  - `plate_steel.scad` - Plate steel with rounded corners (5 module types)
  - `structural_steel.scad` - Square tubing, angle iron, flat bar (6 module types)
  - `fasteners.scad` - Hex bolts, nuts, washers with patterns (7 module types)
  - `hydraulics.scad` - Cylinders, motors, pumps, valves, hoses (5 module types)
  - `wheels.scad` - Wheels, axles, bearings, modular units (6 module types)
- Created main design file:
  - `lifetrac_v25.scad` - Complete parametric machine design
  - Base frame with 4x4" square tubing
  - Four wheel assemblies with hydraulic motors
  - Loader arm system with lift cylinders
  - Bucket assembly with tilt cylinders
  - Optional standing deck
  - Electronics housing
  - Animation support for moving parts
- Created documentation:
  - `PARTS_LIST.md` - Complete part catalog with descriptions and connections
  - `ASSEMBLY.md` - Step-by-step assembly instructions
  - `README.md` - Comprehensive design documentation
- Set up GitHub Actions workflow:
  - `openscad-render.yml` - Automated rendering and validation
  - Validates OpenSCAD syntax on every commit
  - Generates STL files for 3D models
  - Creates preview renders (main, front, side, top views)
  - Produces 36-frame animation
  - Renders module examples
- Updated .gitignore for generated files

#### 🔄 In Progress
- Testing and validation of design files

#### 📋 Next Steps
1. Create standardized component modules:
   - Plate steel with rounded corners
   - Square tubing sections
   - Angle iron sections
   - Bolt and fastener library
   - Hydraulic cylinder placeholder
   - Hydraulic motor placeholder
   - Wheel and axle assembly

2. Design main frame:
   - Base frame using square tubing
   - Wheel mounting points (4 positions)
   - Hydraulic motor mounts
   - Standing platform location

3. Design loader arms:
   - Arm structure with cylinder mounts
   - Pivot points and bearings
   - Bucket attachment interface
   - Range of motion calculation

4. Create animation system:
   - Arm lift sequence
   - Bucket tilt sequence
   - Wheel rotation

5. Set up automated workflows:
   - GitHub Actions for rendering
   - STL generation for 3D printing
   - DXF generation for CNC cutting
   - Part grouping by size

## Design Decisions

### Material Choices
- **1/4" Plate Steel**: Default for most structural plates
  - Adequate strength for compact loader
  - Easier to cut and form than thicker material
  - Reduces overall weight

- **1/2" Plate Steel**: Used for high-stress areas
  - Cylinder mounting points
  - Pivot points
  - Wheel mounting plates

- **Square Tubing**: Main frame construction
  - Efficient structural element
  - Easy to weld and bolt
  - Follows OSE construction set philosophy

### Design Philosophy
Following Open Source Ecology principles:
- Modularity - Easy to replace and repair components
- Standardization - Use common sizes and fasteners
- Simplicity - Minimize custom parts
- Manufacturability - Design for available tools (CNC plasma, drill press, welder)

## References
- [LifeTrac v17.10 Wiki](https://wiki.opensourceecology.org/wiki/LifeTrac_v17.10)
- [LifeTrac Main Wiki](https://wiki.opensourceecology.org/wiki/LifeTrac)
- [OSE CEB Press GitHub](https://github.com/OpenSourceEcology/CEB-Press)
- [FabricRack OpenSCAD](https://github.com/dorkmo/FabricRack-OpenSCAD)

## Technical Notes

### OpenSCAD Best Practices for This Project
- Use parameters for all dimensions (easy scaling)
- Create modules for repeated components
- Include part numbers and descriptions as comments
- Use color coding for different materials
- Generate exploded views for assembly

### CNC Manufacturing Considerations
- Maintain minimum 1/8" spacing between cut lines
- Include alignment holes for assembly
- Group parts by material thickness
- Optimize nesting for material efficiency
- Export with part numbers as layer names

## Questions & Issues
*Document any design challenges, decisions requiring input, or technical issues here*

---
Last Updated: 2025-12-06
