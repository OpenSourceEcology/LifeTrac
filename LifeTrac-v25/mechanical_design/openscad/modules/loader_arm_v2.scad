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
        angle_trim = 3.175;  // 1/8" trimmed from each end
        len = 152.4 - 2*angle_trim;  // 6" minus 1/4" total = 5.75" (146.05mm)
        leg = 50.8;
        thick = 6.35;
        
        // Bolt spacing from center
        plate_bolt_offset = 50.8; // 2.0 inches from center (4 inch spacing) - Wider
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
        // Tube starts at 'overlap' from pivot center (1" clearance from 6" plate)
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
            // Rear Holes (near pivot mount - parametric inset from tube start)
            // TUBE_BOLT_INSET is distance from tube edge to the NEAREST bolt
            _tube_bolt_inset = is_undef(TUBE_BOLT_INSET) ? 50.8 : TUBE_BOLT_INSET; // Use param or default 2"
             for (x = [_tube_bolt_inset, _tube_bolt_inset + bolt_spacing_x])
             for (z = [tube_h/2 - bolt_spacing_y/2, tube_h/2 + bolt_spacing_y/2])
                 translate([x, tube_w/2, z]) rotate([90,0,0]) cylinder(d=BOLT_DIA_1_2, h=tube_w+10, center=true, $fn=32);
            // Front Holes (near elbow - parametric inset from tube end)
             for (x = [main_tube_len - _tube_bolt_inset - bolt_spacing_x, main_tube_len - _tube_bolt_inset])
             for (z = [tube_h/2 - bolt_spacing_y/2, tube_h/2 + bolt_spacing_y/2])
                 translate([x, tube_w/2, z]) rotate([90,0,0]) cylinder(d=BOLT_DIA_1_2, h=tube_w+10, center=true, $fn=32);
        }

        // 2. Continuous Side Plates
        
        // Instantiate Plates
        
        // Angle iron positioning - trimmed length centered on crossbeam
        angle_trim = 3.175;  // 1/8" trimmed from each end
        angle_len = 152.4 - 2*angle_trim;  // Trimmed length
        
        // Plate 1: Y = -plate_thick
        translate([0, -plate_thick, 0]) {
            arm_plate(is_left); // is_left means it's inner for left arm
            
            if (is_left) { // Add angles for Left Arm Inner
                hole_x = CROSS_BEAM_1_POS;
                // Top Angle
                translate([hole_x - angle_len/2, 0, tube_h/2 + cross_beam_h/2])
                    scale([1, -1, 1]) angle_iron_mount();
                // Bottom Angle
                translate([hole_x - angle_len/2, 0, tube_h/2 - cross_beam_h/2])
                    scale([1, -1, -1]) angle_iron_mount();
            }
        }
        
        // Plate 2: Y = tube_w
        translate([0, tube_w, 0]) {
            arm_plate(!is_left); // !is_left means it's inner for right arm
            
            if (!is_left) { // Add angles for Right Arm Inner
                hole_x = CROSS_BEAM_1_POS;
                // Top Angle
                translate([hole_x - angle_len/2, plate_thick, tube_h/2 + cross_beam_h/2])
                    scale([1, 1, 1]) angle_iron_mount();
                // Bottom Angle
                translate([hole_x - angle_len/2, plate_thick, tube_h/2 - cross_beam_h/2])
                    scale([1, 1, -1]) angle_iron_mount();
            }
        }
        
        // 3. Leg Spacer Tube (2x6 Section)
        spacer_len = 150;
        translate([overlap + main_tube_len, 0, tube_h]) 
        rotate([0, 180-elbow_angle, 0]) 
        translate([0, 0, -tube_h]) {
            // Calculated Taper Parameters from arm_plate geometry
            cx = drop_len - (ARM_DROP_EXT - PIVOT_HOLE_X_FROM_FRONT); // Re-calc logic for local context? 
            // Better: Just use local variables if available, or re-derive.
            // arm_plate logic: cx (tip center X local) = drop_len + drop_ext - PIVOT_HOLE_X... - wait
            // In arm_plate: cx = drop_ext - PIVOT_HOLE_X_FROM_FRONT
            // Boss X = drop_len + cx.
            // Boss Z = cz = PIVOT_HOLE_Z_FROM_BOTTOM.
            
            _plate_cx = ARM_DROP_EXT - PIVOT_HOLE_X_FROM_FRONT;
            _boss_x = drop_len + _plate_cx;
            _boss_z = PIVOT_HOLE_Z_FROM_BOTTOM;
            _boss_r = PIVOT_HOLE_X_FROM_FRONT;
            
            // Define the Tapered Envelope (Matches arm_plate.scad logic)
            cut_start_x = drop_len - 100;
            
            difference() {
                intersection() {
                    // The Raw 2x6 Tube Section
                    translate([0, 0, tube_h])
                    rotate([0, 90, 0])
                    linear_extrude(height=spacer_len)
                    difference() {
                        offset(r=tube_rad) square([tube_h-tube_rad*2, tube_w-tube_rad*2], center=true);
                         hull() { // Inner hollow
                             translate([-tube_h/2+tube_rad, -tube_w/2+tube_rad]) circle(r=tube_rad);
                             translate([tube_h/2-tube_rad, -tube_w/2+tube_rad]) circle(r=tube_rad);
                             translate([tube_h/2-tube_rad, tube_w/2-tube_rad]) circle(r=tube_rad);
                             translate([-tube_h/2+tube_rad, tube_w/2-tube_rad]) circle(r=tube_rad);
                         }
                    }
                    
                    // The Profile Envelope (Expanded Y to infinite)
                    union() {
                        // Full Width Part
                        cube([cut_start_x, 100+tube_w, tube_h]);
                        // Taper Part
                        hull() {
                            translate([cut_start_x - 1, -50, 0]) cube([1, 100 + tube_w, tube_h]);
                            translate([_boss_x, -50, _boss_z]) rotate([-90,0,0]) cylinder(r=_boss_r, h=100+tube_w);
                        }
                    }
                }
                
                // 4 Bolts for Spacer
                for (xb = [spacer_len/3, spacer_len*2/3])
                for (zb = [tube_h*0.3, tube_h*0.7]) // Rough placement within taper
                    translate([xb, tube_w/2, zb]) rotate([90,0,0]) cylinder(d=BOLT_DIA_1_2, h=tube_w+50, center=true, $fn=32);
            }
        }
        
        // PIVOT MOUNT ASSEMBLY
        // Replaceable assembly with DOM tube welded between two 6" circular plates
        // Slides into arm from below and bolts to arm plates
        // See parts/pivot_mount_assembly.scad for full assembly
        translate([0, tube_w/2, tube_h/2]) {
            // DOM Pipe (Through everything)
            rotate([90, 0, 0]) 
            difference() {
                cylinder(d=DOM_PIPE_OD, h=SANDWICH_SPACING, center=true, $fn=64);
                cylinder(d=DOM_PIPE_ID, h=SANDWICH_SPACING + 2, center=true, $fn=64);
            }
            
            // Circular mounting plates (6" diameter) with 5 bolt holes
            // These are welded to the DOM tube and bolt to the arm plates
            _plate_dia = is_undef(PIVOT_MOUNT_PLATE_DIA) ? 152.4 : PIVOT_MOUNT_PLATE_DIA;
            _plate_thick = PLATE_1_4_INCH;
            _bolt_count = is_undef(PIVOT_MOUNT_BOLT_COUNT) ? 5 : PIVOT_MOUNT_BOLT_COUNT;
            _bolt_circle_dia = is_undef(PIVOT_MOUNT_BOLT_CIRCLE_DIA) ? 114.3 : PIVOT_MOUNT_BOLT_CIRCLE_DIA;
            _bolt_dia = is_undef(PIVOT_MOUNT_BOLT_DIA) ? BOLT_DIA_1_2 : PIVOT_MOUNT_BOLT_DIA;
            // Bolt angles array for non-uniform spacing (larger gap around slot)
            _bolt_angles = is_undef(PIVOT_MOUNT_BOLT_ANGLES) ? 
                [295.5, 372.6, 449.8, 526.9, 604.1] : PIVOT_MOUNT_BOLT_ANGLES;
            
            color("DarkSlateGray")
            rotate([90, 0, 0]) {
                // Front plate with bolt holes
                translate([0, 0, tube_w/2 - _plate_thick])
                difference() {
                    cylinder(d=_plate_dia, h=_plate_thick, $fn=64);
                    translate([0, 0, -1])
                        cylinder(d=DOM_PIPE_OD, h=_plate_thick + 2, $fn=64);
                    // 5 bolt holes using pre-calculated angles (non-uniform spacing)
                    for (i = [0:_bolt_count-1]) {
                        angle = _bolt_angles[i];
                        bolt_x = (_bolt_circle_dia / 2) * cos(angle);
                        bolt_y = (_bolt_circle_dia / 2) * sin(angle);
                        translate([bolt_x, bolt_y, -1])
                            cylinder(d=_bolt_dia, h=_plate_thick + 2, $fn=24);
                    }
                }
                
                // Rear plate with bolt holes
                translate([0, 0, -tube_w/2])
                difference() {
                    cylinder(d=_plate_dia, h=_plate_thick, $fn=64);
                    translate([0, 0, -1])
                        cylinder(d=DOM_PIPE_OD, h=_plate_thick + 2, $fn=64);
                    // 5 bolt holes using pre-calculated angles (non-uniform spacing)
                    for (i = [0:_bolt_count-1]) {
                        angle = _bolt_angles[i];
                        bolt_x = (_bolt_circle_dia / 2) * cos(angle);
                        bolt_y = (_bolt_circle_dia / 2) * sin(angle);
                        translate([bolt_x, bolt_y, -1])
                            cylinder(d=_bolt_dia, h=_plate_thick + 2, $fn=24);
                    }
                }
            }
            
            // WELD VISUALIZATION - Red torus (donut) at DOM-to-plate joints
            // These show the welded area between DOM tube and circular plates
            // Torus is created by rotating a circle around the DOM centerline
            // 4 welds per plate: inside and outside face of each plate
            _weld_dia = is_undef(PIVOT_MOUNT_WELD_DIA) ? 6.35 : PIVOT_MOUNT_WELD_DIA;  // 0.25" weld bead diameter
            color("Red", 0.8)
            rotate([90, 0, 0]) {
                // FRONT PLATE WELDS
                // Inside face (toward center of arm)
                translate([0, 0, tube_w/2 - _plate_thick])
                    rotate_extrude(angle=360, $fn=64)
                    translate([DOM_PIPE_OD/2, 0])
                        circle(d=_weld_dia, $fn=24);
                // Outside face (toward outside of arm)
                translate([0, 0, tube_w/2])
                    rotate_extrude(angle=360, $fn=64)
                    translate([DOM_PIPE_OD/2, 0])
                        circle(d=_weld_dia, $fn=24);
                
                // REAR PLATE WELDS
                // Inside face (toward center of arm)
                translate([0, 0, -tube_w/2 + _plate_thick])
                    rotate_extrude(angle=360, $fn=64)
                    translate([DOM_PIPE_OD/2, 0])
                        circle(d=_weld_dia, $fn=24);
                // Outside face (toward outside of arm)
                translate([0, 0, -tube_w/2])
                    rotate_extrude(angle=360, $fn=64)
                    translate([DOM_PIPE_OD/2, 0])
                        circle(d=_weld_dia, $fn=24);
            }
        }
        
        // NOTE: Hydraulic mounting is now integrated into the arm_plate profile
        // The plates extend downward with the 3" circle mounting point
        // No separate bracket needed - see arm_plate.scad
    }
}
