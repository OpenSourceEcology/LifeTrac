include <../../lifetrac_v25_params.scad>

module angle_iron_drill_jig() {
    leg = ANGLE_2X2_1_4[0];
    wall = JIG_WALL_THICKNESS;
    length = 100;
    
    difference() {
        cube([leg + wall, leg + wall, length]);
        translate([wall, wall, -1]) cube([leg + 1, leg + 1, length + 2]);
        
        // Drill holes
        translate([wall + leg/2, 0, length/2]) rotate([90,0,0]) cylinder(d=5, h=wall*3, center=true, $fn=16);
        translate([0, wall + leg/2, length/2]) rotate([0,90,0]) cylinder(d=5, h=wall*3, center=true, $fn=16);
    }
}
angle_iron_drill_jig();
