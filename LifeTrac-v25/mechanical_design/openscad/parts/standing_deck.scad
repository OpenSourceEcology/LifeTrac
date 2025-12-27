// standing_deck.scad
// DEPRECATED - Use platform_deck.scad instead
// This file is kept for backward compatibility
// The new folding platform uses platform_deck.scad for the deck plate
//
// For the complete folding platform assembly, see:
//   - platform_deck.scad (CNC-cut deck plate)
//   - platform_pivot_bracket.scad (CNC-cut pivot brackets)
//   - platform_angle_arm.scad (purchased angle iron, drilling template)

include <../lifetrac_v25_params.scad>

// Redirect to new platform deck module
module standing_deck() {
    // Import and call the new platform_deck module
    // Note: This only renders the deck plate, not the full folding assembly
    
    // Replicate the deck plate geometry for backward compatibility
    width = PLATFORM_WIDTH;
    depth = PLATFORM_DEPTH;
    thickness = PLATFORM_THICKNESS;
    
    // Bolt hole parameters
    bolt_hole_dia = PLATFORM_BOLT_DIA + PLATFORM_BOLT_CLEARANCE;
    bolt_edge_margin = 40;
    
    // Angle iron attachment positions
    arm_x_left = -PLATFORM_PIVOT_X;
    arm_x_right = PLATFORM_PIVOT_X;
    bolt_y = depth/2 - bolt_edge_margin;
    bolt_spacing_z = PLATFORM_ANGLE_BOLT_OFFSET;
    
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
        
        // Left angle iron mounting holes
        for (x_off = [-bolt_spacing_z, 0, bolt_spacing_z]) {
            translate([arm_x_left + x_off, bolt_y, 0])
            cylinder(d=bolt_hole_dia, h=thickness + 4, center=true, $fn=32);
        }
        
        // Right angle iron mounting holes
        for (x_off = [-bolt_spacing_z, 0, bolt_spacing_z]) {
            translate([arm_x_right + x_off, bolt_y, 0])
            cylinder(d=bolt_hole_dia, h=thickness + 4, center=true, $fn=32);
        }
        
        // Corner mounting holes
        for (x = [-width/2 + 40, width/2 - 40]) {
            for (y = [-depth/2 + 40]) {
                translate([x, y, 0])
                cylinder(d=bolt_hole_dia, h=thickness + 4, center=true, $fn=32);
            }
        }
    }
}

// Render the part for preview/export
if ($preview || len(search("standing_deck.scad", parent_modules())) == 0) {
    standing_deck();
}
