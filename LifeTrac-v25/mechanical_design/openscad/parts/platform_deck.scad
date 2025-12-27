// platform_deck.scad
// Folding platform deck plate for LifeTrac v25 operator
// CNC cut from 1/4" plate with anti-slip hole pattern
// Part of the 5-component folding platform system
//
// This plate attaches to two angle iron arms via bolt holes along the rear edge.
// Width is parametric, derived from inner side panel spacing.
// Bolt holes are positioned to align with the angle iron arms at PLATFORM_PIVOT_X

include <../lifetrac_v25_params.scad>

/**
 * Creates the platform deck plate with anti-slip holes and mounting holes
 * All dimensions are parametric from lifetrac_v25_params.scad
 * 
 * Coordinate system when rendered:
 *   - Centered at origin in X and Y
 *   - Thickness in Z direction
 *   - Rear edge (angle iron attachment) at +Y
 */
module platform_deck() {
    // Local variables for clarity
    width = PLATFORM_WIDTH;
    depth = PLATFORM_DEPTH;
    thickness = PLATFORM_THICKNESS;
    
    // Bolt hole parameters
    bolt_hole_dia = PLATFORM_BOLT_DIA + PLATFORM_BOLT_CLEARANCE;
    
    // Angle iron attachment positions (X coordinates)
    deck_half_width = width / 2;
    arm_margin = 35;
    arm_x_pos = min(PLATFORM_PIVOT_X, deck_half_width - arm_margin);
    arm_x_left = -arm_x_pos;
    arm_x_right = arm_x_pos;
    
    difference() {
        // Base plate
        cube([width, depth, thickness], center=true);
        
        // Anti-slip hole pattern
        for (x = [-width/2 + PLATFORM_ANTISLIP_EDGE_MARGIN : 
                   PLATFORM_ANTISLIP_SPACING : 
                   width/2 - PLATFORM_ANTISLIP_EDGE_MARGIN]) {
            for (y = [-depth/2 + PLATFORM_ANTISLIP_EDGE_MARGIN : 
                       PLATFORM_ANTISLIP_SPACING : 
                       depth/2 - PLATFORM_ANTISLIP_EDGE_MARGIN]) {
                // Skip holes that would interfere with bolt hole areas
                // Expanded exclusion zones for new transverse angles
                if (!(abs(x - arm_x_left) < 50) && 
                    !(abs(x - arm_x_right) < 50) &&
                    !(abs(y) > depth/2 - 60)) {
                    translate([x, y, 0])
                    cylinder(d=PLATFORM_ANTISLIP_HOLE_DIA, 
                             h=thickness + 4, 
                             center=true, 
                             $fn=24);
                }
            }
        }
        
        // Left angle iron mounting holes (Side Angle)
        // Y positions: -125, 125 (relative to center) - Adjusted for new arm length
        for (y_pos = [-125, 125]) {
            translate([arm_x_left, y_pos, 0])
            cylinder(d=bolt_hole_dia, h=thickness + 4, center=true, $fn=32);
        }
        
        // Right angle iron mounting holes (Side Angle)
        for (y_pos = [-125, 125]) {
            translate([arm_x_right, y_pos, 0])
            cylinder(d=bolt_hole_dia, h=thickness + 4, center=true, $fn=32);
        }
        
        // Transverse Angle Holes (Front and Rear)
        // Y positions: -187.3 (Front), 187.3 (Rear) - 1/2 inch from edges
        // X positions: 0, -100, 100
        for (y_pos = [-187.3, 187.3]) {
            for (x_pos = [-100, 0, 100]) {
                translate([x_pos, y_pos, 0])
                cylinder(d=bolt_hole_dia, h=thickness + 4, center=true, $fn=32);
            }
        }
        
        // Corner mounting holes (optional, for additional support straps)
        for (x = [-width/2 + 40, width/2 - 40]) {
            for (y = [-depth/2 + 40]) {  // Front corners only
                translate([x, y, 0])
                cylinder(d=bolt_hole_dia, h=thickness + 4, center=true, $fn=32);
            }
        }
    }
}

// Render the part for preview/export
if ($preview || len(search("platform_deck.scad", parent_modules())) == 0) {
    platform_deck();
}
