import FreeCAD
import Part
import math

# Units in mm
inch = 25.4

# --- Parameters ---
plate_w = 8 * inch
plate_h = 8 * inch
plate_t = 0.25 * inch

plate_spacing_1_2 = 3 * inch
plate_spacing_2_3 = 4 * inch

shaft_diam = 1.25 * inch

# --- Updated from QC Wheel Diagram ---
center_hole_diam = 2.00 * inch
bolt_circle_diam = 6.19 * inch
bolt_hole_diam = 0.5 * inch

doc = FreeCAD.newDocument("UWU_v25")

def create_plate(name, z_pos, is_motor=False):
    # Create base cube
    pl = Part.makeBox(plate_w, plate_h, plate_t)
    # Center it in X/Y
    pl.translate(FreeCAD.Vector(-plate_w/2, -plate_h/2, z_pos - plate_t/2))
    
    # Cut holes
    # Center Hole (Used for both Shaft clearance and Motor Pilot in this standard)
    hole = Part.makeCylinder(center_hole_diam/2, plate_t * 3)
    hole.translate(FreeCAD.Vector(0,0, z_pos - plate_t))
    pl = pl.cut(hole)
    
    # Bolt holes - Circular Pattern (Ø6.19)
    # Drawing typically shows 4 holes. Assuming 45 deg offset standard for 4-bolt.
    radius = bolt_circle_diam / 2
    for i in range(4):
        angle_deg = 45 + (i * 90)
        angle_rad = math.radians(angle_deg)
        x = radius * math.cos(angle_rad)
        y = radius * math.sin(angle_rad)
        
        b_hole = Part.makeCylinder(bolt_hole_diam/2, plate_t * 3)
        b_hole.translate(FreeCAD.Vector(x, y, z_pos - plate_t))
        pl = pl.cut(b_hole)
        
    # Apply to doc
    obj = doc.addObject("Part::Feature", name)
    obj.Shape = pl
    return obj

# 1. Motor Plate
create_plate("MotorPlate", 0, is_motor=True)

# 2. Bearing Plate 1
create_plate("BearingPlate1", plate_spacing_1_2)

# 3. Bearing Plate 2
create_plate("BearingPlate2", plate_spacing_1_2 + plate_spacing_2_3)

# 4. Simple Cylinder Representations for Motor and Bearings
# Motor
motor = Part.makeCylinder(3.5*inch/2, 5*inch)
motor.translate(FreeCAD.Vector(0,0, -5*inch - plate_t/2))
obj_motor = doc.addObject("Part::Feature", "HydraulicMotor")
obj_motor.Shape = motor
obj_motor.ViewObject.ShapeColor = (1.0, 0.5, 0.0)

# Shaft
shaft = Part.makeCylinder(shaft_diam/2, 10*inch)
shaft.translate(FreeCAD.Vector(0,0, plate_t/2))
obj_shaft = doc.addObject("Part::Feature", "MainShaft")
obj_shaft.Shape = shaft

doc.recompute()
print("UWU v25 Assembly generated.")
