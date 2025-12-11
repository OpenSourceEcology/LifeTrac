// rear_crossmember.scad
// Rear crossmember plate for LifeTrac v25
// Connects left and right side panels at the rear

include <../lifetrac_v25_params.scad>

module rear_crossmember() {
    back_height = MACHINE_HEIGHT * 0.55;
    crossmember_width = TRACK_WIDTH + SANDWICH_SPACING + PANEL_THICKNESS * 2;
    plate_thickness = PANEL_THICKNESS;

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
    }
}

// Render the part for preview/export
if ($preview || len(search("rear_crossmember.scad", parent_modules())) == 0) {
    rear_crossmember();
}
