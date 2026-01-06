include <../lifetrac_v25_params.scad>
use <../parts/arm_plate.scad>

module loader_arm_v2(angle=0, side="left") {
    main_tube_len = ARM_MAIN_LEN; 
    drop_len = ARM_DROP_LEN;
    overlap = ARM_OVERLAP;
    plate_thick = ARM_PLATE_THICKNESS;
    tube_w = TUBE_2X6_1_4[0]; 
    tube_h = TUBE_2X6_1_4[1];
    bolt_spacing_x = 100;
    bolt_spacing_y = 80;
    elbow_angle = ARM_ANGLE; 
    drop_ext = ARM_DROP_EXT;
    corner_rad = 25.4; // 1 inch radius for corners
    wall_thick = 6.35;
    tube_rad = 12.7;
    cross_beam_rad = 12.7; // 1/2 inch radius for crossbeam
    
    // Pivot Point Configuration (Unused here, defined in arm_plate.scad)
    // pivot_dia = BUCKET_PIVOT_PIN_DIA; 
    // hole_x_offset = drop_ext - 30; 

    // Elbow assembly parameters
    extension_len = 350;
    cross_beam_w = 152.4; // 6 inches wide
    cross_beam_h = 50.8;  // 2 inches tall
    
    // Determine which plate is the "Inner" plate (facing machine center)
    is_left = (side == "left");

    module angle_iron_mount() {
        len = 152.4;
        leg = 50.8;
        thick = 6.35;
        
        // Bolt spacing from center
        plate_bolt_offset = 63.5; // 2.5 inches from center (5 inch spacing) - Wider
        beam_bolt_offset = 25.4;  // 1.0 inch from center (2 inch spacing) - Narrower
        
        difference() {
            union() {
                cube([len, thick, leg]); // Vertical leg (against plate)
                cube([len, leg, thick]); // Horizontal leg (against beam)
            }
            // Holes
            // Vertical leg (Plate bolts)
            translate([len/2 - plate_bolt_offset, thick + 1, 25.4]) rotate([90,0,0]) cylinder(d=BOLT_DIA_1_2, h=thick+2, $fn=32);
            translate([len/2 + plate_bolt_offset, thick + 1, 25.4]) rotate([90,0,0]) cylinder(d=BOLT_DIA_1_2, h=thick+2, $fn=32);
            // Horizontal leg (Beam bolts)
            translate([len/2 - beam_bolt_offset, 25.4, -1]) cylinder(d=BOLT_DIA_1_2, h=thick+2, $fn=32);
            translate([len/2 + beam_bolt_offset, 25.4, -1]) cylinder(d=BOLT_DIA_1_2, h=thick+2, $fn=32);
        }
    }

    rotate([0, -angle, 0]) {
        // 1. Main Tube
        translate([overlap, 0, 0])
        difference() {
            // Hollow Rounded Tube
            translate([0, 0, tube_h])
            rotate([0, 90, 0])
            linear_extrude(height = main_tube_len)
            difference() {
                hull() {
                    translate([tube_rad, tube_rad]) circle(r=tube_rad);
                    translate([tube_h-tube_rad, tube_rad]) circle(r=tube_rad);
                    translate([tube_h-tube_rad, tube_w-tube_rad]) circle(r=tube_rad);
                    translate([tube_rad, tube_w-tube_rad]) circle(r=tube_rad);
                }
                offset(delta = -wall_thick)
                hull() {
                    translate([tube_rad, tube_rad]) circle(r=tube_rad);
                    translate([tube_h-tube_rad, tube_rad]) circle(r=tube_rad);
                    translate([tube_h-tube_rad, tube_w-tube_rad]) circle(r=tube_rad);
                    translate([tube_rad, tube_w-tube_rad]) circle(r=tube_rad);
                }
            }
            // Rear Holes
             for (x = [overlap/2 - bolt_spacing_x/2, overlap/2 + bolt_spacing_x/2])
             for (z = [tube_h/2 - bolt_spacing_y/2, tube_h/2 + bolt_spacing_y/2])
                 translate([x, tube_w/2, z]) rotate([90,0,0]) cylinder(d=BOLT_DIA_1_2, h=tube_w+10, center=true, $fn=32);
            // Front Holes
            translate([main_tube_len - overlap, 0, 0])
             for (x = [overlap/2 - bolt_spacing_x/2, overlap/2 + bolt_spacing_x/2])
             for (z = [tube_h/2 - bolt_spacing_y/2, tube_h/2 + bolt_spacing_y/2])
                 translate([x, tube_w/2, z]) rotate([90,0,0]) cylinder(d=BOLT_DIA_1_2, h=tube_w+10, center=true, $fn=32);
        }

        // 2. Continuous Side Plates
        
        // Instantiate Plates
        
        // Plate 1: Y = -plate_thick
        translate([0, -plate_thick, 0]) {
            arm_plate(is_left); // is_left means it's inner for left arm
            
            if (is_left) { // Add angles for Left Arm Inner
                hole_x = CROSS_BEAM_1_POS;
                // Top Angle
                translate([hole_x - 152.4/2, 0, tube_h/2 + cross_beam_h/2])
                    scale([1, -1, 1]) angle_iron_mount();
                // Bottom Angle
                translate([hole_x - 152.4/2, 0, tube_h/2 - cross_beam_h/2])
                    scale([1, -1, -1]) angle_iron_mount();
            }
        }
        
        // Plate 2: Y = tube_w
        translate([0, tube_w, 0]) {
            arm_plate(!is_left); // !is_left means it's inner for right arm
            
            if (!is_left) { // Add angles for Right Arm Inner
                hole_x = CROSS_BEAM_1_POS;
                // Top Angle
                translate([hole_x - 152.4/2, plate_thick, tube_h/2 + cross_beam_h/2])
                    scale([1, 1, 1]) angle_iron_mount();
                // Bottom Angle
                translate([hole_x - 152.4/2, plate_thick, tube_h/2 - cross_beam_h/2])
                    scale([1, 1, -1]) angle_iron_mount();
            }
        }
        
        // DOM Pipe (Through everything)
        translate([0, tube_w/2, tube_h/2]) rotate([90,0,0]) 
            difference() {
                cylinder(d=DOM_PIPE_OD, h=SANDWICH_SPACING, center=true, $fn=64);
                cylinder(d=DOM_PIPE_ID, h=SANDWICH_SPACING + 2, center=true, $fn=64);
            }
    }
}
