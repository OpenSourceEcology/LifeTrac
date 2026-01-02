include <../lifetrac_v25_params.scad>

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
    hole_x_offset = drop_ext - 25.4; // 1 inch from front edge

    rotate([0, -angle, 0]) {
        // 1. Main Tube
        translate([overlap, 0, 0])
        difference() {
            cube([main_tube_len, tube_w, tube_h]);
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

        // 2. Pivot Assembly
        translate([0, -plate_thick, 0]) {
            difference() {
                union() {
                    cube([overlap + 50, plate_thick, tube_h]); 
                    translate([0, 0, tube_h/2]) cylinder(d=tube_h, h=plate_thick, $fn=64); 
                    // Cylinder Mount Extension
                    translate([overlap + 50, 0, 0]) cube([LIFT_CYL_ARM_OFFSET - (overlap + 50) + 50, plate_thick, tube_h]);
                }
                translate([0, 0, tube_h/2]) cylinder(d=DOM_PIPE_OD, h=plate_thick+2, center=true, $fn=64);
                translate([overlap/2 + 50, 0, 0]) 
                 for (x = [-bolt_spacing_x/2, bolt_spacing_x/2])
                 for (z = [tube_h/2 - bolt_spacing_y/2, tube_h/2 + bolt_spacing_y/2])
                     translate([x + overlap/2, plate_thick/2, z]) rotate([90,0,0]) cylinder(d=BOLT_DIA_1_2, h=plate_thick+2, center=true, $fn=32);
                
                // Cylinder Mount Hole
                translate([LIFT_CYL_ARM_OFFSET, plate_thick/2, tube_h/2]) rotate([90,0,0]) cylinder(d=BOLT_DIA_1 + 2, h=plate_thick+2, center=true, $fn=32);
            }
        }
        translate([0, tube_w, 0]) { // Inner Plate
             difference() {
                union() {
                    cube([overlap + 50, plate_thick, tube_h]); 
                    translate([0, 0, tube_h/2]) cylinder(d=tube_h, h=plate_thick, $fn=64);
                    // Cylinder Mount Extension
                    translate([overlap + 50, 0, 0]) cube([LIFT_CYL_ARM_OFFSET - (overlap + 50) + 50, plate_thick, tube_h]);
                }
                translate([0, 0, tube_h/2]) cylinder(d=DOM_PIPE_OD, h=plate_thick+2, center=true, $fn=64);
                translate([overlap/2 + 50, 0, 0])
                 for (x = [-bolt_spacing_x/2, bolt_spacing_x/2])
                 for (z = [tube_h/2 - bolt_spacing_y/2, tube_h/2 + bolt_spacing_y/2])
                     translate([x + overlap/2, plate_thick/2, z]) rotate([90,0,0]) cylinder(d=BOLT_DIA_1_2, h=plate_thick+2, center=true, $fn=32);
                
                // Cylinder Mount Hole
                translate([LIFT_CYL_ARM_OFFSET, plate_thick/2, tube_h/2]) rotate([90,0,0]) cylinder(d=BOLT_DIA_1 + 2, h=plate_thick+2, center=true, $fn=32);
            }
        }
        translate([0, tube_w/2, tube_h/2]) rotate([90,0,0]) // DOM Pipe
            difference() {
                cylinder(d=DOM_PIPE_OD, h=tube_w + 2*plate_thick, center=true, $fn=64);
                cylinder(d=DOM_PIPE_ID, h=tube_w + 2*plate_thick + 2, center=true, $fn=64);
            }

        // 3. Elbow Assembly
        extension_len = 350;
        cross_beam_w = 152.4; // 6 inches wide
        cross_beam_h = 50.8;  // 2 inches tall
        
        // Determine which plate is the "Inner" plate (facing machine center)
        // Left Arm (side="left"): Inner is y < 0 (Plate at -plate_thick)
        // Right Arm (side="right"): Inner is y > 0 (Plate at tube_w)
        is_left = (side == "left");
        
        translate([overlap + main_tube_len - overlap - extension_len, 0, 0]) {
             // Plate at y = -plate_thick (Inner for Left, Outer for Right)
             translate([0, -plate_thick, 0])
             difference() {
                 union() {
                     cube([overlap + 50 + extension_len, plate_thick, tube_h]); // Horizontal Extended
                     translate([overlap + 50 + extension_len, 0, tube_h]) rotate([0, 180-elbow_angle, 0]) translate([0, 0, -tube_h]) {
                        cube([drop_len, plate_thick, tube_h]); // Drop
                        // Extension for pivot to clear bucket
                        translate([drop_len, 0, 0]) {
                            hull() {
                                // Base interface
                                cube([0.1, plate_thick, tube_h]);
                                
                                // Top Front Corner
                                translate([drop_ext - corner_rad, 0, tube_h - corner_rad])
                                    rotate([-90, 0, 0]) cylinder(r=corner_rad, h=plate_thick);
                                    
                                // Bottom Front Corner
                                translate([drop_ext - corner_rad, 0, corner_rad])
                                    rotate([-90, 0, 0]) cylinder(r=corner_rad, h=plate_thick);
                            }
                        }
                     }
                     
                     // Angle Irons (Only if this is the Inner plate -> Left Arm)
                     if (is_left) {
                         // Top Angle (Extending -Y)
                         translate([extension_len - 50 - 152.4/2, 0, tube_h/2 + cross_beam_h/2])
                            rotate([90, 0, 0]) // Rotate to face -Y
                            cube([152.4, 50.8, 6.35]); // L-shape simplified as plate for now, or actual angle?
                            // Using simple plate for angle iron face for now as per "face touches outer face"
                            // Actually, let's make it an L-shape
                            // But user said "angle iron... face touches the outer face of the cnc plate"
                            // So the flat part is against the plate.
                            
                         translate([extension_len - 50 - 152.4/2, -6.35, tube_h/2 + cross_beam_h/2])
                            cube([152.4, 6.35, 50.8]); // Vertical leg against plate
                         translate([extension_len - 50 - 152.4/2, -50.8, tube_h/2 + cross_beam_h/2])
                            cube([152.4, 50.8, 6.35]); // Horizontal leg
                            
                         // Bottom Angle
                         translate([extension_len - 50 - 152.4/2, -6.35, tube_h/2 - cross_beam_h/2 - 50.8])
                            cube([152.4, 6.35, 50.8]); // Vertical leg against plate
                         translate([extension_len - 50 - 152.4/2, -50.8, tube_h/2 - cross_beam_h/2 - 6.35])
                            cube([152.4, 50.8, 6.35]); // Horizontal leg
                     }
                 }
                 // Tube Bolts (Existing)
                 translate([extension_len + overlap/2, plate_thick/2, tube_h/2])
                  for (x = [-bolt_spacing_x/2, bolt_spacing_x/2])
                  for (z = [-bolt_spacing_y/2, bolt_spacing_y/2])
                      translate([x, 0, z]) rotate([90,0,0]) cylinder(d=BOLT_DIA_1_2, h=plate_thick+2, center=true, $fn=32);
                 
                 // New Bolts (Rear)
                 translate([100, plate_thick/2, tube_h/2])
                  for (z = [-bolt_spacing_y/2, bolt_spacing_y/2])
                      translate([0, 0, z]) rotate([90,0,0]) cylinder(d=BOLT_DIA_1_2, h=plate_thick+2, center=true, $fn=32);

                 // Cross Beam Hole (Only if Left Arm)
                 if (is_left) {
                     translate([extension_len - 50, plate_thick/2, tube_h/2])
                        cube([cross_beam_w, plate_thick+2, cross_beam_h], center=true);
                        
                     // Angle Iron Mounting Bolts
                     translate([extension_len - 50, plate_thick/2, tube_h/2 + cross_beam_h/2 + 25.4])
                        rotate([90,0,0]) cylinder(d=BOLT_DIA_1_2, h=plate_thick+20, center=true, $fn=32);
                     translate([extension_len - 50, plate_thick/2, tube_h/2 - cross_beam_h/2 - 25.4])
                        rotate([90,0,0]) cylinder(d=BOLT_DIA_1_2, h=plate_thick+20, center=true, $fn=32);
                 }

                 // Bucket Pivot (at end of extension)
                 translate([overlap + 50 + extension_len, 0, tube_h]) rotate([0, 180-elbow_angle, 0]) translate([drop_len + hole_x_offset, plate_thick/2, -tube_h/2])
                    rotate([90,0,0]) cylinder(d=PIVOT_PIN_DIA, h=plate_thick+2, center=true, $fn=32);
             }
             
             // Plate at y = tube_w (Outer for Left, Inner for Right)
             translate([0, tube_w, 0])
             difference() {
                 union() {
                     cube([overlap + 50 + extension_len, plate_thick, tube_h]); 
                     translate([overlap + 50 + extension_len, 0, tube_h]) rotate([0, 180-elbow_angle, 0]) translate([0, 0, -tube_h]) {
                        cube([drop_len, plate_thick, tube_h]); 
                        // Extension for pivot to clear bucket
                        translate([drop_len, 0, 0]) {
                            hull() {
                                // Base interface
                                cube([0.1, plate_thick, tube_h]);
                                
                                // Top Front Corner
                                translate([drop_ext - corner_rad, 0, tube_h - corner_rad])
                                    rotate([-90, 0, 0]) cylinder(r=corner_rad, h=plate_thick);
                                    
                                // Bottom Front Corner
                                translate([drop_ext - corner_rad, 0, corner_rad])
                                    rotate([-90, 0, 0]) cylinder(r=corner_rad, h=plate_thick);
                            }
                        }
                     }
                     
                     // Angle Irons (Only if this is the Inner plate -> Right Arm)
                     if (!is_left) {
                         // Top Angle (Extending +Y)
                         translate([extension_len - 50 - 152.4/2, plate_thick, tube_h/2 + cross_beam_h/2])
                            cube([152.4, 6.35, 50.8]); // Vertical leg
                         translate([extension_len - 50 - 152.4/2, plate_thick, tube_h/2 + cross_beam_h/2])
                            cube([152.4, 50.8, 6.35]); // Horizontal leg
                            
                         // Bottom Angle
                         translate([extension_len - 50 - 152.4/2, plate_thick, tube_h/2 - cross_beam_h/2 - 50.8])
                            cube([152.4, 6.35, 50.8]); // Vertical leg
                         translate([extension_len - 50 - 152.4/2, plate_thick, tube_h/2 - cross_beam_h/2 - 6.35])
                            cube([152.4, 50.8, 6.35]); // Horizontal leg
                     }
                 }
                 // Tube Bolts (Existing)
                 translate([extension_len + overlap/2, plate_thick/2, tube_h/2])
                  for (x = [-bolt_spacing_x/2, bolt_spacing_x/2])
                  for (z = [-bolt_spacing_y/2, bolt_spacing_y/2])
                      translate([x, 0, z]) rotate([90,0,0]) cylinder(d=BOLT_DIA_1_2, h=plate_thick+2, center=true, $fn=32);

                 // New Bolts (Rear)
                 translate([100, plate_thick/2, tube_h/2])
                  for (z = [-bolt_spacing_y/2, bolt_spacing_y/2])
                      translate([0, 0, z]) rotate([90,0,0]) cylinder(d=BOLT_DIA_1_2, h=plate_thick+2, center=true, $fn=32);
                 
                 // Cross Beam Hole (Only if Right Arm)
                 if (!is_left) {
                     translate([extension_len - 50, plate_thick/2, tube_h/2])
                        cube([cross_beam_w, plate_thick+2, cross_beam_h], center=true);
                        
                     // Angle Iron Mounting Bolts
                     translate([extension_len - 50, plate_thick/2, tube_h/2 + cross_beam_h/2 + 25.4])
                        rotate([90,0,0]) cylinder(d=BOLT_DIA_1_2, h=plate_thick+20, center=true, $fn=32);
                     translate([extension_len - 50, plate_thick/2, tube_h/2 - cross_beam_h/2 - 25.4])
                        rotate([90,0,0]) cylinder(d=BOLT_DIA_1_2, h=plate_thick+20, center=true, $fn=32);
                 }

                 // Bucket Pivot (at end of extension)
                 translate([overlap + 50 + extension_len, 0, tube_h]) rotate([0, 180-elbow_angle, 0]) translate([drop_len + hole_x_offset, plate_thick/2, -tube_h/2])
                    rotate([90,0,0]) cylinder(d=PIVOT_PIN_DIA, h=plate_thick+2, center=true, $fn=32);
             }
        }
    }
}
