// platform_pivot_bracket.scad
// Pivot bracket plate for LifeTrac v25 folding platform
// CNC cut from 1/4" plate
// Part of the 5-component folding platform system
//
// This bracket connects the angle iron arm to the machine frame:
//   - One end has bolt holes for angle iron attachment
//   - Other end has a 1" pivot hole and lock pin hole
//
// Two brackets required (left and right side)

include <../lifetrac_v25_params.scad>

/**
 * Creates the pivot bracket plate with pivot hole, lock hole, and bolt holes
 * All dimensions are parametric from lifetrac_v25_params.scad
 * 
 * Coordinate system when rendered:
 *   - Centered at origin in X and Y
 *   - Thickness in Z direction
 *   - Pivot hole at Origin
 *   - Bolt holes at +X
 *   - Designed to be oriented vertically when installed
 */
module platform_pivot_bracket() {
    // Local variables
    thickness = PLATFORM_THICKNESS;
    width = PLATFORM_BRACKET_WIDTH;
    
    // Dimensions
    pivot_to_corner_dist = 60; // Distance from pivot to corner (Vertical offset)
    corner_to_bolt_dist = 60;  // Distance from corner to first bolt (Horizontal offset)
    bolt_spacing = 50;
    
    // Derived positions
    // Top Section (Vertical)
    // Pivot at [0,0]
    // Lock pin position (Above pivot)
    lock_pin_y = 60; 
    
    // Bottom Section (Horizontal)
    // Corner position (Down from pivot)
    corner_y = -pivot_to_corner_dist;
    
    // Bolt positions (relative to corner along X)
    // Updated to match new side angle iron hole pattern (30mm and 80mm from start)
    // Pivot bracket starts at pivot. Angle iron starts at pivot bracket corner?
    // Angle iron pivot holes are at z=30 and z=80.
    // Pivot bracket corner is at corner_y = -60.
    // So angle iron starts at corner_y.
    // Bolt 1: 30mm from corner.
    // Bolt 2: 80mm from corner.
    bolt_1_x = corner_to_bolt_dist - 30; // Wait, corner_to_bolt_dist was 60.
    // Let's redefine based on angle iron holes.
    // Angle iron holes are at z=30, 80 from its start.
    // Its start aligns with the corner of the bracket?
    // Yes, bracket extends from pivot to corner, then along arm.
    // So bolts should be at 30 and 80 from corner.
    bolt_1_x = 30;
    bolt_2_x = 80;
    
    // Hole diameters
    pivot_hole_dia = PLATFORM_PIVOT_PIN_DIA + PLATFORM_BOLT_CLEARANCE;
    bolt_hole_dia = PLATFORM_BOLT_DIA + PLATFORM_BOLT_CLEARANCE;
    lock_hole_dia = PLATFORM_LOCK_PIN_DIA + PLATFORM_BOLT_CLEARANCE;
    
    difference() {
        union() {
            // Top Section (Vertical Leg)
            hull() {
                translate([0, lock_pin_y, 0])
                cylinder(d=width, h=thickness, center=true, $fn=48);
                
                translate([0, corner_y, 0])
                cylinder(d=width, h=thickness, center=true, $fn=48);
            }
            
            // Bottom Section (Horizontal Leg)
            hull() {
                translate([0, corner_y, 0])
                cylinder(d=width, h=thickness, center=true, $fn=48);
                
                // Extended to reach back of platform deck (PLATFORM_ARM_LENGTH)
                translate([PLATFORM_ARM_LENGTH - width/2, corner_y, 0]) 
                cylinder(d=width, h=thickness, center=true, $fn=48);
            }
        }
        
        // Pivot hole at Origin
        cylinder(d=pivot_hole_dia, h=thickness + 4, center=true, $fn=48);
        
        // Lock pin hole
        translate([0, lock_pin_y, 0])
        cylinder(d=lock_hole_dia, h=thickness + 4, center=true, $fn=32);
        
        // Bolt holes
        translate([bolt_1_x, corner_y, 0])
        cylinder(d=bolt_hole_dia, h=thickness + 4, center=true, $fn=32);
        
        translate([bolt_2_x, corner_y, 0])
        cylinder(d=bolt_hole_dia, h=thickness + 4, center=true, $fn=32);
    }
}

// Render the part for preview/export
if ($preview || len(search("platform_pivot_bracket.scad", parent_modules())) == 0) {
    platform_pivot_bracket();
}
