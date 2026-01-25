// arm_plate.scad
// CNC cut plate for Loader Arm v2
// Part of LifeTrac v25 OpenSCAD design

include <../lifetrac_v25_params.scad>

arm_plate();

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
    
    // Cross Beam Parameters (Used for inner plate cutouts)
    cross_beam_w = 152.4; // 6 inches wide
    cross_beam_h = 50.8;  // 2 inches tall

    // Tip Geometry Configuration
    // Alignment with Geometric Solver (lifetrac_v25_params.scad)
    drop_ext = ARM_DROP_EXT; 
    pivot_dia = BUCKET_PIVOT_PIN_DIA; 
    boss_r = PIVOT_HOLE_X_FROM_FRONT; // Use parameter to determine tip radius

    // Position: Defined by Global Parameters to match Kinematic Solver
    // X: Distance from front edge of arm tube
    // Z: Distance from bottom edge of arm tube
    // Note: Local coord system has X=Length, Y=Thick, Z=Height (Tube Width)
    
    // In params: PIVOT_HOLE_X_FROM_FRONT is distance from "front tip" back to hole.
    // Here: The tube ends at `drop_len`. The extension `drop_ext` is added beyond that.
    // The visual tip is located at `drop_len + drop_ext`.
    // The hole X position relative to `drop_len` should be `drop_ext - PIVOT_HOLE_X_FROM_FRONT`.
    
    cx = drop_ext - PIVOT_HOLE_X_FROM_FRONT; 
    
    // In params: PIVOT_HOLE_Z_FROM_BOTTOM is distance from bottom edge up to hole.
    // In arm_plate, Z=0 is the "bottom" edge (or top edge? Let's check rotate angle).
    // The drop leg is rotated by `180-elbow_angle`.
    // At 130 deg elbow, rotation is 50 deg.
    // The cube is `cube([drop_len, plate_thick, tube_h])`.
    // Z=0 is one side, Z=tube_h is the other side.
    // We need to know which side is "Bottom" in world space.
    // The main arm is flat. The drop arm goes DOWN.
    // If we rotate [0, 50, 0] relative to main arm...
    // Main arm Z faces UP.
    // Rotated Z faces UP-LEFT.
    // The "Bottom" of the arm in world space corresponds to Z=0 edge of the tube profile?
    // Let's assume Z=0 is the "Inner/Bottom" edge of the L-shape corner.
    // cz should be PIVOT_HOLE_Z_FROM_BOTTOM.
    
    cz = PIVOT_HOLE_Z_FROM_BOTTOM;
    
    // Hydraulic mounting bracket parameters (integrated into plate)
    hyd_bracket_pos = is_undef(HYD_BRACKET_ARM_POS) ? LIFT_CYL_ARM_OFFSET : HYD_BRACKET_ARM_POS;
    hyd_circle_dia = is_undef(HYD_BRACKET_CIRCLE_DIA) ? 76.2 : HYD_BRACKET_CIRCLE_DIA;
    hyd_circle_r = hyd_circle_dia / 2;
    hyd_bolt_dia = is_undef(HYD_BRACKET_BOLT_DIA) ? BOLT_DIA_1 : HYD_BRACKET_BOLT_DIA;
    
    // Circle center is below the tube bottom (Z=0) by circle radius
    hyd_circle_center_z = -hyd_circle_r;

    echo("=== ARM PLATE HYD BRACKET DEBUG ===");
    echo("HYD_BRACKET_ARM_POS:", HYD_BRACKET_ARM_POS);
    echo("LIFT_CYL_ARM_OFFSET:", LIFT_CYL_ARM_OFFSET);
    echo("hyd_bracket_pos (used):", hyd_bracket_pos);
    echo("hyd_circle_center_z:", hyd_circle_center_z);

    difference() {
        union() {
             // Main Horizontal Strip (Pivot to Elbow)
            translate([-tube_h/2, 0, 0])
            cube([overlap + main_tube_len + tube_h/2, plate_thick, tube_h]);
            
            // =============================================================
            // HYDRAULIC MOUNTING EXTENSION (below main tube)
            // =============================================================
            // This extends the plate downward at hyd_bracket_pos to create
            // a 3" circle mounting point for the hydraulic cylinder.
            // Right triangular gussets connect the tube bottom to the circle.
            
            // Gusset attachment length along arm (3x tube width for strength)
            gusset_attach_len = tube_w * 3;  // ~6" attachment length
            
            // Triangle + Circle geometry extending below tube
            translate([hyd_bracket_pos, 0, 0]) {
                // Right triangle gussets tangent to circle
                // Hull from tube bottom corners to circle
                hull() {
                    // Top edge at tube bottom (Z=0), spanning 3x tube width along arm
                    translate([-gusset_attach_len/2, 0, 0])
                    cube([gusset_attach_len, plate_thick, 1]);
                    
                    // Bottom: 3" circle centered below
                    translate([0, 0, hyd_circle_center_z])
                    rotate([-90, 0, 0])
                    cylinder(r=hyd_circle_r, h=plate_thick, $fn=48);
                }
            }
            
            // Drop Leg (Elbow) - 6" Width Maintained until Cut
            translate([overlap + main_tube_len, 0, tube_h]) 
            rotate([0, 180-elbow_angle, 0]) 
            translate([0, 0, -tube_h]) {
                // Determine cut start point
                // Maintain full width until proximity to tip
                cut_start_x = drop_len - 100; // Start cut 100mm before length (plus extension)
                
                union() {
                    // 1. Full Width Section (Elbow to Cut Start)
                    cube([cut_start_x, plate_thick, tube_h]);
                    
                    // 2. Tapered Tip Section
                    hull() {
                         // Connection Face from Full Width
                         translate([cut_start_x - 1, 0, 0])
                            cube([1, plate_thick, tube_h]);
                         
                         // The Boss Circle at the Tip
                         translate([drop_len + cx, 0, cz])
                            rotate([-90,0,0])
                            cylinder(r=boss_r, h=plate_thick);
                    }
                }
            }
        }
        
        // --- HOLES ---
        
        // 7. Bucket Pivot Hole (Tangent to Edges)
        translate([overlap + main_tube_len, 0, tube_h]) 
        rotate([0, 180-elbow_angle, 0]) 
        translate([drop_len + cx, plate_thick/2, cz - tube_h]) // Corrected Z offset logic
            rotate([90,0,0]) cylinder(d=pivot_dia, h=plate_thick+2, center=true, $fn=32);
            
        // 1. Pivot DOM Pipe Hole
        translate([0, plate_thick/2, tube_h/2]) rotate([90,0,0]) cylinder(d=DOM_PIPE_OD, h=plate_thick+2, center=true, $fn=64);
        
        // 2. Hydraulic Cylinder Mount Hole (in the circle below tube)
        // This is the primary mounting point for the lift cylinder
        translate([hyd_bracket_pos, plate_thick/2, hyd_circle_center_z]) 
        rotate([90,0,0]) 
        cylinder(d=hyd_bolt_dia + 2, h=plate_thick+2, center=true, $fn=32);
        
        // 3. Extra Alignment Bolt Holes along tube (2x)
        for (i = [1, 2]) {
            translate([hyd_bracket_pos + i * 50.8, plate_thick/2, tube_h/2]) 
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
    }
}

// =============================================================================
// HYDRAULIC MOUNTING GEOMETRY NOTES
// =============================================================================
// The hydraulic cylinder mounting is now integrated into the arm_plate profile.
// The plate extends downward below the 2x6 tube with:
// - A 3" diameter circle at the bottom (mounting point for cylinder pin)
// - Right triangular sections connecting the tube bottom to the circle (via hull)
// - Position adjustable via HYD_BRACKET_ARM_POS parameter
//
// The side panels need matching arc cutouts to clear this extension during rotation.
// See hydraulic_bracket_arc_slot() in side_panel.scad
