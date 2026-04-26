// rear_crossmember.scad
// Rear crossmember plate for LifeTrac v25
// Connects left and right side panels at the rear
// Includes pivot holes and lock pin holes for folding platform

include <../lifetrac_v25_params.scad>

module rear_crossmember() {
    back_height = MACHINE_HEIGHT * 0.55;
    crossmember_width = TRACK_WIDTH + SANDWICH_SPACING + PANEL_THICKNESS * 2;
    plate_thickness = PANEL_THICKNESS;

    // Folding platform pivot mounting calculations
    // Y coordinate in crossmember local coords (centered square, so offset from center)
    // Pivot Z in world = PLATFORM_PIVOT_HEIGHT, crossmember bottom Z = FRAME_Z_OFFSET
    // Crossmember is centered, so local Y = world_Z - (FRAME_Z_OFFSET + back_height/2)
    pivot_local_y = PLATFORM_PIVOT_HEIGHT - FRAME_Z_OFFSET - back_height/2;
    
    // Deployed lock hole position (below pivot)
    deployed_lock_y = pivot_local_y - PLATFORM_LOCK_OFFSET;
    
    // Stowed lock hole position (above pivot, at arm length distance)
    stowed_lock_y = pivot_local_y + PLATFORM_ARM_LENGTH;

    // Lay the plate flat in XY so projection captures all features
    difference() {
        // Main plate
        linear_extrude(height=plate_thickness)
        square([crossmember_width, back_height], center=true);

        // Bolt holes for mounting to side panels (4 per side)
        for (side = [-1, 1]) {
            x_pos = side * (crossmember_width/2 - 50);
            for (z_pos = [back_height/4, back_height * 3/4]) {
                y_pos = z_pos - back_height/2; // shift because square is centered
                translate([x_pos, y_pos, plate_thickness/2])
                cylinder(d=BOLT_DIA_1_2 + 2, h=plate_thickness+4, center=true, $fn=32);
            }
        }

        // Lightening holes to reduce weight
        for (x = [-200, 0, 200]) {
            y_pos = (back_height/2 - 100) - back_height/2; // place 100mm below top edge
            translate([x, y_pos, plate_thickness/2])
            cylinder(d=60, h=plate_thickness+4, center=true, $fn=32);
        }
        
        // =================================================================
        // FOLDING PLATFORM MOUNTING HOLES
        // =================================================================
        
        // Pivot pin holes (1" diameter + clearance) - one on each side
        for (side = [-1, 1]) {
            pivot_x = side * PLATFORM_PIVOT_X;
            
            // Main pivot hole
            translate([pivot_x, pivot_local_y, plate_thickness/2])
            cylinder(d=PLATFORM_PIVOT_PIN_DIA + PLATFORM_BOLT_CLEARANCE, 
                     h=plate_thickness + 4, 
                     center=true, 
                     $fn=48);
            
            // Deployed position lock pin hole (3/8" + clearance)
            // Located below pivot when platform is horizontal
            translate([pivot_x, deployed_lock_y, plate_thickness/2])
            cylinder(d=PLATFORM_LOCK_PIN_DIA + PLATFORM_BOLT_CLEARANCE, 
                     h=plate_thickness + 4, 
                     center=true, 
                     $fn=32);
            
            // Stowed position lock pin hole (3/8" + clearance)
            // Located above pivot when platform is vertical against crossmember
            // Only add if within crossmember bounds
            if (stowed_lock_y < back_height/2 - 20) {
                translate([pivot_x, stowed_lock_y, plate_thickness/2])
                cylinder(d=PLATFORM_LOCK_PIN_DIA + PLATFORM_BOLT_CLEARANCE, 
                         h=plate_thickness + 4, 
                         center=true, 
                         $fn=32);
            }
        }
    }
}

// Render the part for preview/export
if ($preview || len(search("rear_crossmember.scad", parent_modules())) == 0) {
    rear_crossmember();
}
