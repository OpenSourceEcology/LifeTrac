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
    bolt_edge_margin = 40;  // Distance from rear edge to bolt holes
    
    // Angle iron attachment positions (X coordinates)
    // Position at PLATFORM_PIVOT_X (where the angle irons are)
    // If pivot X is outside deck width, clamp to deck edge minus margin
    deck_half_width = width / 2;
    arm_margin = 35;  // Minimum distance from deck edge to bolt hole center
    
    // Clamp arm X position to be within the deck
    arm_x_pos = min(PLATFORM_PIVOT_X, deck_half_width - arm_margin);
    arm_x_left = -arm_x_pos;
    arm_x_right = arm_x_pos;
    
    bolt_y = depth/2 - bolt_edge_margin;  // Near rear edge
    
    // Bolt spacing along angle iron (centered, with one above and one below center)
    bolt_spacing = PLATFORM_ANGLE_BOLT_OFFSET;  // ~30mm
    
    difference() {
        // Base plate
        cube([width, depth, thickness], center=true);
        
        // Anti-slip hole pattern
        // Grid of holes for traction and drainage
        for (x = [-width/2 + PLATFORM_ANTISLIP_EDGE_MARGIN : 
                   PLATFORM_ANTISLIP_SPACING : 
                   width/2 - PLATFORM_ANTISLIP_EDGE_MARGIN]) {
            for (y = [-depth/2 + PLATFORM_ANTISLIP_EDGE_MARGIN : 
                       PLATFORM_ANTISLIP_SPACING : 
                       depth/2 - PLATFORM_ANTISLIP_EDGE_MARGIN]) {
                // Skip holes that would interfere with bolt hole areas
                if (!(abs(x - arm_x_left) < 50 && y > bolt_y - 50) &&
                    !(abs(x - arm_x_right) < 50 && y > bolt_y - 50)) {
                    translate([x, y, 0])
                    cylinder(d=PLATFORM_ANTISLIP_HOLE_DIA, 
                             h=thickness + 4, 
                             center=true, 
                             $fn=24);
                }
            }
        }
        
        // Left angle iron mounting holes (3 holes in Y direction along rear edge)
        for (y_off = [-bolt_spacing, 0, bolt_spacing]) {
            translate([arm_x_left, bolt_y + y_off, 0])
            cylinder(d=bolt_hole_dia, h=thickness + 4, center=true, $fn=32);
        }
        
        // Right angle iron mounting holes (3 holes in Y direction along rear edge)
        for (y_off = [-bolt_spacing, 0, bolt_spacing]) {
            translate([arm_x_right, bolt_y + y_off, 0])
            cylinder(d=bolt_hole_dia, h=thickness + 4, center=true, $fn=32);
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
