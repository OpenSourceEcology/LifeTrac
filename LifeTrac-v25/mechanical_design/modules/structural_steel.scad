// structural_steel.scad
// Square tubing and angle iron modules
// Part of LifeTrac v25 OpenSCAD design

// Global parameters
$fn = 32;

// Standard square tubing sizes (outer dimension x wall thickness, in mm)
TUBE_2X2_1_4 = [50.8, 6.35];   // 2"x2" x 1/4" wall
TUBE_3X3_1_4 = [76.2, 6.35];   // 3"x3" x 1/4" wall
TUBE_4X4_1_4 = [101.6, 6.35];  // 4"x4" x 1/4" wall (OSE standard)
TUBE_4X4_3_8 = [101.6, 9.525]; // 4"x4" x 3/8" wall

// Standard angle iron sizes (leg size x thickness, in mm)
ANGLE_2X2_1_4 = [50.8, 6.35];   // 2"x2" x 1/4"
ANGLE_3X3_1_4 = [76.2, 6.35];   // 3"x3" x 1/4"
ANGLE_4X4_1_4 = [101.6, 6.35];  // 4"x4" x 1/4"

/**
 * Creates square tubing
 * @param length Length in mm
 * @param size [outer_dimension, wall_thickness]
 * @param center Center on origin if true
 */
module square_tubing(length, size=TUBE_4X4_1_4, center=false) {
    outer = size[0];
    wall = size[1];
    inner = outer - 2*wall;
    
    translate(center ? [0, 0, -length/2] : [0, 0, 0])
    difference() {
        cube([outer, outer, length], center=true);
        cube([inner, inner, length+2], center=true);
    }
}

/**
 * Creates rectangular tubing
 * @param length Length in mm
 * @param width Width in mm
 * @param height Height in mm
 * @param wall_thickness Wall thickness in mm
 * @param center Center on origin if true
 */
module rectangular_tubing(length, width, height, wall_thickness=6.35, center=false) {
    inner_width = width - 2*wall_thickness;
    inner_height = height - 2*wall_thickness;
    
    translate(center ? [-width/2, -height/2, -length/2] : [0, 0, 0])
    rotate([90, 0, 0])
    difference() {
        cube([width, length, height], center=false);
        translate([wall_thickness, -1, wall_thickness])
            cube([inner_width, length+2, inner_height], center=false);
    }
}

/**
 * Creates angle iron (L-shaped profile)
 * @param length Length in mm
 * @param size [leg_size, thickness]
 * @param center Center on origin if true
 */
module angle_iron(length, size=ANGLE_3X3_1_4, center=false) {
    leg = size[0];
    thick = size[1];
    
    translate(center ? [0, 0, -length/2] : [0, 0, 0])
    rotate([90, 0, 0])
    linear_extrude(height=length)
    polygon([
        [0, 0],
        [leg, 0],
        [leg, thick],
        [thick, thick],
        [thick, leg],
        [0, leg]
    ]);
}

/**
 * Creates a mitered square tubing joint (for corner connections)
 * @param length1 Length of first tube
 * @param length2 Length of second tube
 * @param size [outer_dimension, wall_thickness]
 * @param angle Angle between tubes (typically 90)
 */
module mitered_tube_joint(length1, length2, size=TUBE_4X4_1_4, angle=90) {
    // First tube (horizontal)
    square_tubing(length1, size);
    
    // Second tube (at angle)
    translate([0, 0, 0])
    rotate([0, angle, 0])
    square_tubing(length2, size);
}

/**
 * Creates a flat bar (solid rectangular bar stock)
 * @param length Length in mm
 * @param width Width in mm
 * @param thickness Thickness in mm
 * @param center Center on origin if true
 */
module flat_bar(length, width=50.8, thickness=6.35, center=false) {
    translate(center ? [-width/2, -thickness/2, -length/2] : [0, 0, 0])
    cube([width, thickness, length]);
}

/**
 * Creates a round tube
 * @param length Length in mm
 * @param outer_diameter Outer diameter in mm
 * @param wall_thickness Wall thickness in mm
 * @param center Center on origin if true
 */
module round_tube(length, outer_diameter, wall_thickness, center=false) {
    inner_diameter = outer_diameter - 2*wall_thickness;
    
    translate(center ? [0, 0, -length/2] : [0, 0, 0])
    difference() {
        cylinder(d=outer_diameter, h=length);
        translate([0, 0, -1])
            cylinder(d=inner_diameter, h=length+2);
    }
}

// Example usage and visualization
if ($preview) {
    // Square tubing examples
    color("Silver")
    square_tubing(300, TUBE_4X4_1_4);
    
    translate([150, 0, 0])
    color("Silver")
    square_tubing(300, TUBE_3X3_1_4);
    
    translate([300, 0, 0])
    color("Silver")
    square_tubing(300, TUBE_2X2_1_4);
    
    // Angle iron example
    translate([0, 150, 0])
    color("Gray")
    angle_iron(300, ANGLE_3X3_1_4);
    
    // Flat bar example
    translate([150, 150, 0])
    color("DimGray")
    flat_bar(300, 50.8, 6.35);
    
    // Round tube example
    translate([300, 150, 0])
    color("Silver")
    round_tube(300, 50.8, 6.35);
    
    // Rectangular tubing example
    translate([0, 300, 0])
    color("Silver")
    rectangular_tubing(300, 101.6, 50.8);
}
