// wheel_mount.scad
// Wheel mounting plate for LifeTrac v25
// CNC cut from 1/2" plate steel
// Mounts hydraulic wheel motor

include <../lifetrac_v25_params.scad>

module wheel_mount() {
    plate_size = 250;
    
    difference() {
        // Main square plate with rounded corners
        linear_extrude(height=PLATE_1_2_INCH)
        offset(r=10) offset(r=-10)
        square([plate_size, plate_size], center=true);
        
        // Center hole for hydraulic motor shaft
        translate([0, 0, PLATE_1_2_INCH/2])
        cylinder(d=80, h=PLATE_1_2_INCH+4, center=true, $fn=48);
        
        // Bolt circle for motor mounting (8 bolts)
        for (angle = [0 : 45 : 315]) {
            rotate([0, 0, angle])
            translate([90, 0, PLATE_1_2_INCH/2])
            cylinder(d=BOLT_DIA_3_4 + 2, h=PLATE_1_2_INCH+4, center=true, $fn=32);
        }
        
        // Corner mounting holes for attaching to side panel (4 corners)
        for (x = [-1, 1]) {
            for (y = [-1, 1]) {
                translate([x * (plate_size/2 - 30), y * (plate_size/2 - 30), PLATE_1_2_INCH/2])
                cylinder(d=BOLT_DIA_1_2 + 2, h=PLATE_1_2_INCH+4, center=true, $fn=32);
            }
        }
    }
}

// Render the part for preview/export
if ($preview || len(search("wheel_mount.scad", parent_modules())) == 0) {
    wheel_mount();
}
