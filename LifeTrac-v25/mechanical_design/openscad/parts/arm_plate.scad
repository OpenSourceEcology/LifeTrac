// arm_plate.scad
// CNC cut plate for Loader Arm v2
// Part of LifeTrac v25 OpenSCAD design

include <../lifetrac_v25_params.scad>

module arm_plate(is_inner_plate=false) {
    // Local parameters derived from globals
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
    pivot_circle_r = BUCKET_PIVOT_PIN_DIA; // 2x pivot pin radius
    triangle_leg = tube_h / 2; // Triangle leg length (half the arm tip height)
    triangle_corner_x = triangle_leg; // Shift toward tip by one leg length
    triangle_corner_z = 0;            // Bottom corner of tip (local to extension)
    
    // Pivot Point Configuration
    pivot_dia = BUCKET_PIVOT_PIN_DIA; // 1 inch
    hole_x_offset = drop_ext - PIVOT_HOLE_X_FROM_FRONT; // 1.5" from front edge

    // Elbow assembly parameters
    cross_beam_w = 152.4; // 6 inches wide
    cross_beam_h = 50.8;  // 2 inches tall
    
    difference() {
        union() {
            // Main Horizontal Strip (Pivot to Elbow)
            // Starts at -tube_h/2 (to cover pivot)
            // Ends at overlap + main_tube_len (end of tube)
            translate([-tube_h/2, 0, 0])
            cube([overlap + main_tube_len + tube_h/2, plate_thick, tube_h]);
            
            // Drop Leg (Elbow)
            translate([overlap + main_tube_len, 0, tube_h]) 
            rotate([0, 180-elbow_angle, 0]) 
            translate([0, 0, -tube_h]) {
                cube([drop_len, plate_thick, tube_h]); 
                // Extension for pivot to clear bucket
                translate([drop_len, 0, 0]) {
                    difference() {
                        union() {
                            // Square extension (no rounding on the front/top tip corner)
                            cube([drop_ext, plate_thick, tube_h]);
                            // Add pivot circle shifted one radius out and one diameter backward
                            translate([drop_ext - pivot_circle_r, 0, tube_h])
                                rotate([-90, 0, 0]) cylinder(r=pivot_circle_r, h=plate_thick);
                        }
                        // Triangular cut on bottom inside corner (back corner taper)
                        // Expanded by 2mm to ensure clean corner cut
                        translate([drop_ext, -1, 0]) // Position at Tip
                        rotate([-90, 0, 0]) // Rotate to cut profile (X-Z plane)
                        linear_extrude(height=plate_thick+2)
                        polygon([[2, 2], [-triangle_leg, 2], [2, -triangle_leg]]);
                        // Pivot hole at circle center
                        translate([drop_ext - pivot_circle_r, -1, tube_h])
                            rotate([-90, 0, 0]) cylinder(d=pivot_dia, h=plate_thick+2, $fn=32);
                    }
                }
            }
        }
        
        // --- HOLES ---
        
        // 1. Pivot DOM Pipe Hole
        translate([0, plate_thick/2, tube_h/2]) rotate([90,0,0]) cylinder(d=DOM_PIPE_OD, h=plate_thick+2, center=true, $fn=64);
        
        // 2. Cylinder Mount Hole
        translate([LIFT_CYL_ARM_OFFSET, plate_thick/2, tube_h/2]) rotate([90,0,0]) cylinder(d=BOLT_DIA_1 + 2, h=plate_thick+2, center=true, $fn=32);
        
        // 3. Extra Cylinder Bolt Holes (2x)
        for (i = [1, 2]) {
            translate([LIFT_CYL_ARM_OFFSET + i * 50.8, plate_thick/2, tube_h/2]) 
            rotate([90,0,0]) 
            cylinder(d=BOLT_DIA_1_2, h=plate_thick+2, center=true, $fn=32);
        }
        
        // 4. Tube Bolts (Rear Set - near pivot)
        translate([overlap + overlap/2, plate_thick/2, tube_h/2]) // Position relative to overlap start
            for (x = [-bolt_spacing_x/2, bolt_spacing_x/2])
            for (z = [-bolt_spacing_y/2, bolt_spacing_y/2])
                translate([x, 0, z]) rotate([90,0,0]) cylinder(d=BOLT_DIA_1_2, h=plate_thick+2, center=true, $fn=32);

        // 5. Tube Bolts (Front Set - near elbow)
        translate([overlap + main_tube_len - overlap, plate_thick/2, tube_h/2])
            for (x = [-bolt_spacing_x/2, bolt_spacing_x/2])
            for (z = [-bolt_spacing_y/2, bolt_spacing_y/2])
                translate([x, 0, z]) rotate([90,0,0]) cylinder(d=BOLT_DIA_1_2, h=plate_thick+2, center=true, $fn=32);
                
        // 6. Cross Beam Hole (Only if Inner Plate)
        if (is_inner_plate) {
            translate([CROSS_BEAM_1_POS, plate_thick/2, tube_h/2])
                rotate([90, 0, 0])
                cube([cross_beam_w, cross_beam_h, plate_thick+2], center=true);
                
            // Angle Iron Mounting Bolts
            // Note: These must match the holes in angle_iron_mount()
            // Vertical leg bolts are at +/- 63.5mm from center
            // Vertical position is 25.4mm from bottom of angle iron
            // Angle iron bottom is at tube_h/2 +/- cross_beam_h/2
            
            plate_bolt_offset = 63.5;
            
            // Top Angle Bolts
            translate([CROSS_BEAM_1_POS - plate_bolt_offset, plate_thick/2, tube_h/2 + cross_beam_h/2 + 25.4])
                rotate([90,0,0]) cylinder(d=BOLT_DIA_1_2, h=plate_thick+20, center=true, $fn=32);
            translate([CROSS_BEAM_1_POS + plate_bolt_offset, plate_thick/2, tube_h/2 + cross_beam_h/2 + 25.4])
                rotate([90,0,0]) cylinder(d=BOLT_DIA_1_2, h=plate_thick+20, center=true, $fn=32);
                
            // Bottom Angle Bolts
            translate([CROSS_BEAM_1_POS - plate_bolt_offset, plate_thick/2, tube_h/2 - cross_beam_h/2 - 25.4])
                rotate([90,0,0]) cylinder(d=BOLT_DIA_1_2, h=plate_thick+20, center=true, $fn=32);
            translate([CROSS_BEAM_1_POS + plate_bolt_offset, plate_thick/2, tube_h/2 - cross_beam_h/2 - 25.4])
                rotate([90,0,0]) cylinder(d=BOLT_DIA_1_2, h=plate_thick+20, center=true, $fn=32);
        }
        
        // 7. Bucket Pivot Hole
        translate([overlap + main_tube_len, 0, tube_h]) 
        rotate([0, 180-elbow_angle, 0]) 
        translate([drop_len + hole_x_offset, plate_thick/2, PIVOT_HOLE_Z_FROM_BOTTOM])
            rotate([90,0,0]) cylinder(d=pivot_dia, h=plate_thick+2, center=true, $fn=32);
    }
}
