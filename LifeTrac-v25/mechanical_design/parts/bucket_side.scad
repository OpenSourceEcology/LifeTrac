// bucket_side.scad
// Bucket side plate for LifeTrac v25
// CNC cut from 1/4" plate steel
// Trapezoidal shape

include <../openscad/lifetrac_v25_params.scad>

module bucket_side() {
    plate_thickness = PLATE_1_4_INCH;
    
    difference() {
        // Trapezoidal side profile
        linear_extrude(height=plate_thickness)
        offset(r=6.35) offset(r=-6.35)  // Rounded corners
        polygon([
            [0, 0],                           // Bottom front
            [BUCKET_HEIGHT, 0],               // Top front
            [BUCKET_HEIGHT, plate_thickness], // Top front edge
            [BUCKET_HEIGHT * 0.3, BUCKET_DEPTH], // Bottom back
            [0, BUCKET_DEPTH]                 // Bottom back front
        ]);
        
        // Bottom edge mounting holes
        for (y = [80, BUCKET_DEPTH*0.4, BUCKET_DEPTH*0.7, BUCKET_DEPTH-80]) {
            translate([10, y, plate_thickness/2])
            rotate([0, 90, 0])
            cylinder(d=BOLT_DIA_1_2 + 2, h=plate_thickness+4, center=true, $fn=32);
        }
        
        // Top/front edge mounting holes
        for (z = [BUCKET_HEIGHT*0.25, BUCKET_HEIGHT*0.5, BUCKET_HEIGHT*0.75]) {
            translate([z, 10, plate_thickness/2])
            rotate([90, 0, 0])
            cylinder(d=BOLT_DIA_1_2 + 2, h=plate_thickness+4, center=true, $fn=32);
        }
    }
}

// Render the part for preview/export
if ($preview || len(search("bucket_side.scad", parent_modules())) == 0) {
    bucket_side();
}
