# 2026-01-29 Mechanical TODO (v25)

Supersedes: [2026-01-10_Mechanical_TODO.md](2026-01-10_Mechanical_TODO.md)

## Status
- **Loader Arms**: General arm code is about finished.
  - *Pending:* Need to input actual hydraulic ram geometry.

## Active Backlog

### Priority Bugs (Fixes)
- [ ] **Crossbeam Collision**: Crossbeam is too long, colliding with arm's square tubing.
- [ ] **Crossbeam Angle Iron Holes**: Add missing holes for cross beam angle iron on the outside plates of the arms.
- [ ] **Side Wall Safety**: Need to lower side walls a few inches to avoid big pinch point.
- [ ] **Lift Cylinder Mounting**: Move the arm's main hydraulic cylinder attach point to the walls so that it doesn't collide with machine's back plate.
- [ ] **Frame Tube Angle Iron Holes**: Fix the orientation of bolt holes on `frame_tube_angle_iron()` module to align properly with mounting surfaces (check `get_stiffener_holes_a/b` functions).

### Design Requirements
- [ ] **Quick Attach**: Adopt/design quick attach plate from previous OSE work.
- [ ] **Drive Train**: Select wheel motor and mounting design.
- [ ] **Chassis Reinforcement**: Add a few more structural plates on bottom of machine and perhaps top area below arms.
- [ ] **Power Plant**: Add engine and hydraulic pump mounting system.
- [ ] **Hydraulics**: Update model with actual hydraulic ram geometry.
- [ ] **Top Deck Plate**: Add a top deck plate above the motor mount plate.
- [ ] **UWU Wheel/Tire Design**: Finish the wheel and tire design for the Universal Wheel Unit (UWU) integration.

## Design Principles (Carried over)
- **Serviceability** is a hard requirement, especially for the engine.
- **Interfaces first**: define attachment datums + hole patterns before refining geometry.
- **Keep OpenSCAD parametric**: make options controlled by a small number of top-level parameters.
- When unsure, prefer patterns already present in v25 CAD and older OSE conventions.
