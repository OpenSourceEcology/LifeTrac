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
    // Aligned with the center of the angle iron leg (Leg 2)
    // Angle Iron Center X = PLATFORM_PIVOT_X - PLATFORM_SIDE_GAP (Gap/2)
    // Hole Offset from Center = PLATFORM_ANGLE_LEG/2 (Inward)
    // So Hole X = (PLATFORM_PIVOT_X - PLATFORM_SIDE_GAP) - PLATFORM_ANGLE_LEG/2
    
    angle_iron_center_x = PLATFORM_PIVOT_X - PLATFORM_SIDE_GAP;
    arm_x_pos = angle_iron_center_x - PLATFORM_ANGLE_LEG/2;
    arm_x_left = -arm_x_pos;
    arm_x_right = arm_x_pos;
    
    difference() {
        // Base plate
        cube([width, depth, thickness], center=true);
        
        // Anti-slip hole pattern
        // Symmetrical grid calculation
        // Calculate number of holes that fit within margins
        avail_x = width - 2 * PLATFORM_ANTISLIP_EDGE_MARGIN;
        num_x = floor(avail_x / PLATFORM_ANTISLIP_SPACING);
        
        avail_y = depth - 2 * PLATFORM_ANTISLIP_EDGE_MARGIN;
        num_y = floor(avail_y / PLATFORM_ANTISLIP_SPACING);
        
        // Center the grid
        start_x = - (num_x * PLATFORM_ANTISLIP_SPACING) / 2;
        start_y = - (num_y * PLATFORM_ANTISLIP_SPACING) / 2;
        
        for (i = [0 : num_x]) {
            x = start_x + i * PLATFORM_ANTISLIP_SPACING;
            for (j = [0 : num_y]) {
                y = start_y + j * PLATFORM_ANTISLIP_SPACING;
                
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
        // Symmetrical from ends of the side angle iron.
        
        // Replicate assembly logic to find World Y positions
        _deck_pivot_y = PLATFORM_BRACKET_WIDTH/2;
        _deck_far_y = -(PLATFORM_DEPTH - PLATFORM_BRACKET_WIDTH/2);
        _front_angle_corner_y = (_deck_pivot_y - PLATFORM_EDGE_MARGIN) - PLATFORM_ANGLE_LEG;
        _rear_angle_corner_y = (_deck_far_y + PLATFORM_EDGE_MARGIN) + PLATFORM_ANGLE_LEG;
        _gap = 12.7;
        _side_angle_start_y = _front_angle_corner_y - _gap;
        _side_angle_end_y = _rear_angle_corner_y + _gap;
        
        // Bolt Offset from Ends = deck_bolt_offset (matching angle iron definition)
        _bolt_offset = deck_bolt_offset;
        _bolt_1_world_y = _side_angle_start_y - _bolt_offset;
        _bolt_2_world_y = _side_angle_end_y + _bolt_offset;
        
        // Convert World Y to Deck Y
        // Deck Y = -World Y - Shift
        // Shift = PLATFORM_DEPTH/2 - PLATFORM_BRACKET_WIDTH/2
        _shift = PLATFORM_DEPTH/2 - PLATFORM_BRACKET_WIDTH/2;
        
        _bolt_1_deck_y = -_bolt_1_world_y - _shift;
        _bolt_2_deck_y = -_bolt_2_world_y - _shift;
        
        for (y_pos = [_bolt_1_deck_y, _bolt_2_deck_y]) {
            translate([arm_x_left, y_pos, 0])
            cylinder(d=bolt_hole_dia, h=thickness + 4, center=true, $fn=32);
            
            translate([arm_x_right, y_pos, 0])
            cylinder(d=bolt_hole_dia, h=thickness + 4, center=true, $fn=32);
        }
        
        // Transverse Angle Holes (Front and Rear)
        // 3 bolts per angle: Near ends and Center
        // Angle irons are positioned with outside edge at PLATFORM_EDGE_MARGIN from deck edge.
        // Bolt holes should be centered on the angle iron leg.
        transverse_bolt_y_margin = PLATFORM_EDGE_MARGIN + PLATFORM_ANGLE_LEG / 2;
        transverse_bolt_y = depth/2 - transverse_bolt_y_margin;
        // Transverse angles fit between side angles (at arm_x_pos)
        // So holes must be inside arm_x_pos
        // Align with holes in platform_transverse_angle (which are at length/2 - OFFSET)
        // length/2 = angle_iron_center_x - GAP
        // So Hole X = angle_iron_center_x - GAP - OFFSET
        transverse_bolt_x = angle_iron_center_x - (PLATFORM_TRANSVERSE_GAP + PLATFORM_TRANSVERSE_BOLT_END_OFFSET);
        
        for (y_pos = [-transverse_bolt_y, transverse_bolt_y]) {
            for (x_pos = [-transverse_bolt_x, 0, transverse_bolt_x]) {
                translate([x_pos, y_pos, 0])
                cylinder(d=bolt_hole_dia, h=thickness + 4, center=true, $fn=32);
            }
        }
        
        // Corner mounting holes (optional, for additional support straps) - REMOVED
        // for (x = [-width/2 + 40, width/2 - 40]) {
        //     for (y = [-depth/2 + 40]) {  // Front corners only
        //         translate([x, y, 0])
        //         cylinder(d=bolt_hole_dia, h=thickness + 4, center=true, $fn=32);
        //     }
        // }
    }
}

// Render the part for preview/export
if ($preview || len(search("platform_deck.scad", parent_modules())) == 0) {
    platform_deck();
}
