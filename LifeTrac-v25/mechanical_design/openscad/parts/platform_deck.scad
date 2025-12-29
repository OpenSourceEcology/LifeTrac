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
        // 2 bolts upward connecting to the platform itself
        for (y_pos = [-PLATFORM_SIDE_DECK_BOLT_SPACING/2, PLATFORM_SIDE_DECK_BOLT_SPACING/2]) {
            translate([arm_x_left, y_pos, 0])
            cylinder(d=bolt_hole_dia, h=thickness + 4, center=true, $fn=32);
        }
        
        // Right angle iron mounting holes (Side Angle)
        for (y_pos = [-PLATFORM_SIDE_DECK_BOLT_SPACING/2, PLATFORM_SIDE_DECK_BOLT_SPACING/2]) {
            translate([arm_x_right, y_pos, 0])
            cylinder(d=bolt_hole_dia, h=thickness + 4, center=true, $fn=32);
        }
        
        // Transverse Angle Holes (Front and Rear)
        // 2 bolts per angle: Near ends (aligned with side rails)
        // Angle irons are positioned with outside edge at PLATFORM_EDGE_MARGIN from deck edge.
        // Bolt holes should be centered on the angle iron leg.
        transverse_bolt_y_margin = PLATFORM_EDGE_MARGIN + PLATFORM_ANGLE_LEG / 2;
        transverse_bolt_y = depth/2 - transverse_bolt_y_margin;
        // Transverse angles fit between side angles (at arm_x_pos)
        // So holes must be inside arm_x_pos
        // Align with holes in platform_transverse_angle (which are at length/2 - OFFSET)
        // length/2 = arm_x_pos - GAP
        // So Hole X = arm_x_pos - GAP - OFFSET
        transverse_bolt_x = arm_x_pos - (PLATFORM_TRANSVERSE_GAP + PLATFORM_TRANSVERSE_BOLT_END_OFFSET);
        
        for (y_pos = [-transverse_bolt_y, transverse_bolt_y]) {
            for (x_pos = [-transverse_bolt_x, transverse_bolt_x]) {
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
