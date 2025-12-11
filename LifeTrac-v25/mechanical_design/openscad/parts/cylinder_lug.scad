// cylinder_lug.scad
// Hydraulic cylinder mounting lug for LifeTrac v25
// CNC cut from 1/2" plate steel with center pivot hole

include <../lifetrac_v25_params.scad>

module cylinder_lug() {
    lug_width = 100;
    lug_height = 150;
    
    difference() {
        // Main lug plate with rounded top
        linear_extrude(height=PLATE_1_2_INCH)
        offset(r=8) offset(r=-8)  // Rounded corners
        polygon([
            [-lug_width/2, 0],
            [lug_width/2, 0],
            [lug_width/2, lug_height - 30],
            [0, lug_height],
            [-lug_width/2, lug_height - 30]
        ]);
        
        // Center pivot hole for cylinder pin
        translate([0, lug_height - 40, PLATE_1_2_INCH/2])
        rotate([0, 90, 0])
        cylinder(d=BOLT_DIA_1 + 2, h=PLATE_1_2_INCH+4, center=true, $fn=32);
        
        // Base mounting holes (4 holes)
        for (x = [-lug_width/2 + 20, lug_width/2 - 20]) {
            for (y = [20, 50]) {
                translate([x, y, PLATE_1_2_INCH/2])
                cylinder(d=BOLT_DIA_1_2 + 2, h=PLATE_1_2_INCH+4, center=true, $fn=32);
            }
        }
    }
}

// Render the part for preview/export
if ($preview || len(search("cylinder_lug.scad", parent_modules())) == 0) {
    cylinder_lug();
}
