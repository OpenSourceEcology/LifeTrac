# Universal Wheel Unit (UWU)

The Universal Wheel Unit (UWU) is a standardized system of bearings and motors mounted to three parallel plates.

## References
- [Universal Wheel Unit Wiki](https://wiki.opensourceecology.org/wiki/Universal_Wheel_Unit)
- [Quick Connect Wheels Wiki](https://wiki.opensourceecology.org/wiki/Quick_Connect_Wheels)

## Integration
We want to have the option to use either the UWU or UTU (Universal Track Unit) for the LifeTrac v25.

The design goal is to be able to give a coordinate on any future machine and define the three plates and the UWU size, so that the UWU or UTU can be adopted for use in place.

## UWU v25 Specifications

The v25 iteration of the UWU is designed with the following parameters (Source: `UWU-v25/uwu_v25.scad`):

### Structure
- **Plates:** Three 8" x 8" x 0.25" Steel Plates.
- **Spacing:**
  - Motor Mount Plate to First Bearing Plate: 3.0"
  - First Bearing Plate to Second Bearing Plate: 4.0"

### Motor Mount (Plate 1)
- **Motor Body:** Ø6.0"
- **Motor Flange:** 7.5" x 7.5" Square.
- **Mounting Pattern:** 4-bolt, Ø8.25" Bolt Circle Diameter (4.125" radius), rotated 45° to fit corners.
- **Center Pilot:** Ø2.00"

### Bearing Mounts (Plate 2 & 3)
- **Bearing Type:** 4-bolt Flange Bearing.
- **Housing:** Ø2.5"
- **Mounting Pattern:** 4-bolt, Ø6.19" Bolt Circle Diameter (Derived from QC Wheel fabrication diagrams).
- **Center Clearance:** Ø2.00"

### Drivetrain
- **Shaft:** Ø1.25" x 10.0" Length.
- **Coupling:** Ø2.0" x 2.0" Length connecting Motor Shaft to Main Shaft.

### CAD Files
- **OpenSCAD:** `UWU-v25/uwu_v25.scad` (Parametric source).
- **FreeCAD Script:** `UWU-v25/generate_uwu_freecad.py` (Generator script).
