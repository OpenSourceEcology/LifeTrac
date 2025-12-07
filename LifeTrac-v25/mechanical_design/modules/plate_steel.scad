// plate_steel.scad
// Standardized plate steel module with rounded corners
// Part of LifeTrac v25 OpenSCAD design

// Global parameters
$fn = 32; // Circle resolution

// Standard plate thicknesses (inches)
PLATE_1_4_INCH = 6.35;  // 1/4" = 6.35mm
PLATE_1_2_INCH = 12.7;  // 1/2" = 12.7mm
PLATE_1_INCH = 25.4;    // 1" = 25.4mm

// Standard corner radius
DEFAULT_CORNER_RADIUS = 6.35; // 1/4" radius

/**
 * Creates a rectangular plate with rounded corners
 * @param width Width in mm
 * @param length Length in mm
 * @param thickness Thickness in mm (use PLATE_* constants)
 * @param corner_radius Radius for rounded corners in mm
 * @param center Center the plate on origin if true
 */
module plate_steel(width, length, thickness=PLATE_1_4_INCH, corner_radius=DEFAULT_CORNER_RADIUS, center=false) {
    translate(center ? [-width/2, -length/2, 0] : [0, 0, 0])
    linear_extrude(height=thickness)
    offset(r=corner_radius)
    offset(r=-corner_radius)
    square([width, length]);
}

/**
 * Creates a circular plate
 * @param diameter Diameter in mm
 * @param thickness Thickness in mm (use PLATE_* constants)
 * @param center Center the plate on origin if true
 */
module circular_plate(diameter, thickness=PLATE_1_4_INCH, center=false) {
    translate(center ? [0, 0, -thickness/2] : [0, 0, 0])
    cylinder(d=diameter, h=thickness, $fn=64);
}

/**
 * Creates a plate with mounting holes
 * @param width Width in mm
 * @param length Length in mm
 * @param thickness Thickness in mm
 * @param hole_diameter Bolt hole diameter in mm
 * @param hole_offset Distance from edge to hole centers in mm
 * @param corner_radius Radius for rounded corners in mm
 */
module plate_with_holes(width, length, thickness=PLATE_1_4_INCH, 
                       hole_diameter=13, hole_offset=25, 
                       corner_radius=DEFAULT_CORNER_RADIUS) {
    difference() {
        plate_steel(width, length, thickness, corner_radius);
        
        // Corner mounting holes
        for(x = [hole_offset, width-hole_offset])
            for(y = [hole_offset, length-hole_offset])
                translate([x, y, -1])
                    cylinder(d=hole_diameter, h=thickness+2);
    }
}

/**
 * Creates a gusset plate (triangular reinforcement)
 * @param width Base width in mm
 * @param height Height in mm
 * @param thickness Thickness in mm
 * @param corner_radius Radius for rounded corners in mm
 */
module gusset_plate(width, height, thickness=PLATE_1_4_INCH, corner_radius=DEFAULT_CORNER_RADIUS) {
    linear_extrude(height=thickness)
    offset(r=corner_radius)
    offset(r=-corner_radius)
    polygon([[0,0], [width,0], [0,height]]);
}

/**
 * Creates a plate with a centered circular hole
 * @param width Width in mm
 * @param length Length in mm
 * @param thickness Thickness in mm
 * @param hole_diameter Hole diameter in mm
 * @param corner_radius Radius for rounded corners in mm
 */
module plate_with_center_hole(width, length, thickness=PLATE_1_4_INCH,
                              hole_diameter=50,
                              corner_radius=DEFAULT_CORNER_RADIUS) {
    difference() {
        plate_steel(width, length, thickness, corner_radius);
        translate([width/2, length/2, -1])
            cylinder(d=hole_diameter, h=thickness+2);
    }
}

// Example usage and visualization
if ($preview) {
    // Basic plate
    color("DarkSlateGray")
    plate_steel(200, 150, PLATE_1_4_INCH);
    
    // Plate with mounting holes
    translate([0, 200, 0])
    color("DarkSlateGray")
    plate_with_holes(200, 150, PLATE_1_4_INCH);
    
    // Circular plate
    translate([250, 0, 0])
    color("DarkSlateGray")
    circular_plate(150, PLATE_1_4_INCH);
    
    // Gusset plate
    translate([250, 200, 0])
    color("DarkSlateGray")
    gusset_plate(100, 100, PLATE_1_4_INCH);
    
    // Plate with center hole
    translate([0, 400, 0])
    color("DarkSlateGray")
    plate_with_center_hole(200, 150, PLATE_1_4_INCH, 80);
}
