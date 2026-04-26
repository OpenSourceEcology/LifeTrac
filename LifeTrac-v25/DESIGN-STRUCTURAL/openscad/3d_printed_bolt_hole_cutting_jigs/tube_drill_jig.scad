include <../../lifetrac_v25_params.scad>

module tube_drill_jig() {
    // Fits over 2x6 tube
    tube_w = TUBE_2X6_1_4[0];
    tube_h = TUBE_2X6_1_4[1];
    wall = JIG_WALL_THICKNESS;
    
    bolt_spacing_x = 100;
    bolt_spacing_y = 80;
    
    difference() {
        // Outer shell
        cube([tube_h + 2*wall, tube_w + 2*wall, 50], center=true);
        
        // Inner cutout for tube
        cube([tube_h + JIG_CLEARANCE, tube_w + JIG_CLEARANCE, 50+2], center=true);
        
        // Drill holes
        for (x = [-bolt_spacing_y/2, bolt_spacing_y/2]) // Note: x/y swapped relative to tube orientation in arm file
        for (y = [-bolt_spacing_x/2, bolt_spacing_x/2]) // This depends on how jig is oriented
            translate([x, 0, 0]) cylinder(d=5, h=tube_w + 2*wall + 2, center=true, $fn=16); // Pilot holes
    }
}

tube_drill_jig();
