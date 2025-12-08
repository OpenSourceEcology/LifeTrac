// bucket_bottom.scad
// Bucket bottom plate for LifeTrac v25
// CNC cut from 1/4" plate steel

include <../openscad/lifetrac_v25_params.scad>

module bucket_bottom() {
    difference() {
        // Main plate
        cube([BUCKET_WIDTH, BUCKET_DEPTH, PLATE_1_4_INCH], center=true);
        
        // Edge mounting holes for sides (4 per side)
        for (side = [-1, 1]) {
            x_pos = side * (BUCKET_WIDTH/2 - 10);
            for (y = [-BUCKET_DEPTH/2+80, -BUCKET_DEPTH/2+BUCKET_DEPTH*0.4, -BUCKET_DEPTH/2+BUCKET_DEPTH*0.7, BUCKET_DEPTH/2-80]) {
                translate([x_pos, y, 0])
                cylinder(d=BOLT_DIA_1_2 + 2, h=PLATE_1_4_INCH+4, center=true, $fn=32);
            }
        }
        
        // Back edge mounting holes for quick attach
        for (x = [-BUCKET_WIDTH/2+100 : 150 : BUCKET_WIDTH/2-100]) {
            translate([x, BUCKET_DEPTH/2-10, 0])
            cylinder(d=BOLT_DIA_1_2 + 2, h=PLATE_1_4_INCH+4, center=true, $fn=32);
        }
    }
}

// Render the part for preview/export
if ($preview || len(search("bucket_bottom.scad", parent_modules())) == 0) {
    bucket_bottom();
}
