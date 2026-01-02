include <../../lifetrac_v25_params.scad>

module pivot_welding_jig() {
    // Holds two plates apart at TUBE_2X6_1_4[0] (2 inches)
    // Aligns DOM pipe
    
    gap = TUBE_2X6_1_4[0];
    pipe_od = DOM_PIPE_OD;
    
    difference() {
        // Main block
        cube([100, gap, 100], center=true);
        
        // Cutout for DOM pipe
        rotate([90, 0, 0]) cylinder(d=pipe_od + JIG_CLEARANCE, h=gap+2, center=true, $fn=64);
        
        // Cutouts to save material
        cube([80, gap+2, 80], center=true);
    }
    
    // Add alignment tabs for plates?
    // ...
}

pivot_welding_jig();
