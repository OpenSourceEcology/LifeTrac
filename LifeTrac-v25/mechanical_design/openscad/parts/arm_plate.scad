// arm_plate.scad
// CNC cut plate for Loader Arm v2
// Part of LifeTrac v25 OpenSCAD design

include <../lifetrac_v25_params.scad>

// =============================================================================
// PIVOT MOUNT ASSEMBLY PARAMETERS (from params file)
// =============================================================================
// These parameters define the replaceable pivot mount assembly
// that slides into the arm from the bottom
// Note: Main parameters are now in lifetrac_v25_params.scad
// Local copies for backward compatibility:
_PIVOT_MOUNT_PLATE_DIA = is_undef(PIVOT_MOUNT_PLATE_DIA) ? 152.4 : PIVOT_MOUNT_PLATE_DIA;
_PIVOT_MOUNT_BOLT_COUNT = is_undef(PIVOT_MOUNT_BOLT_COUNT) ? 5 : PIVOT_MOUNT_BOLT_COUNT;
_PIVOT_MOUNT_BOLT_DIA = is_undef(PIVOT_MOUNT_BOLT_DIA) ? BOLT_DIA_1_2 : PIVOT_MOUNT_BOLT_DIA;
_PIVOT_MOUNT_BOLT_CIRCLE_DIA = is_undef(PIVOT_MOUNT_BOLT_CIRCLE_DIA) ? 114.3 : PIVOT_MOUNT_BOLT_CIRCLE_DIA;
_PIVOT_MOUNT_CLEARANCE = is_undef(PIVOT_MOUNT_CLEARANCE) ? 25.4 : PIVOT_MOUNT_CLEARANCE;
// Bolt angles array for non-uniform spacing (larger gap around slot)
_PIVOT_MOUNT_BOLT_ANGLES = is_undef(PIVOT_MOUNT_BOLT_ANGLES) ? 
    [295.5, 372.6, 449.8, 526.9, 604.1] : PIVOT_MOUNT_BOLT_ANGLES;

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

    // Pivot mount plate radius for rounded end
    pivot_plate_r = _PIVOT_MOUNT_PLATE_DIA / 2;
    
    difference() {
        union() {
            // SEMICIRCULAR END matching 6" pivot mount plate
            // This creates a half-circle end at the pivot that matches the assembly
            // Only the rear half (X <= 0) is the semicircle
            intersection() {
                translate([0, 0, tube_h/2])
                rotate([-90, 0, 0])
                cylinder(r=pivot_plate_r, h=plate_thick, $fn=64);
                
                // Cut to only keep the rear half (X <= 0)
                translate([-pivot_plate_r, 0, tube_h/2 - pivot_plate_r])
                cube([pivot_plate_r, plate_thick, pivot_plate_r * 2]);
            }
            
            // Main Horizontal Strip (from pivot center to elbow)
            // Starts at X=0 (pivot center) since semicircle covers X<0
            cube([overlap + main_tube_len, plate_thick, tube_h]);
            
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
            
        // 1. PIVOT MOUNT SLOT AND MOUNTING HOLES
        // Slot cut from back of arm to center for pivot mount assembly insertion
        // The pivot mount assembly slides in from the rear and bolts to this plate
        // Slot is oversized to accommodate welds between DOM and circular plates
        
        // Slot diameter includes weld clearance
        _slot_dia = is_undef(PIVOT_MOUNT_SLOT_DIA) ? DOM_PIPE_OD + 25.4 : PIVOT_MOUNT_SLOT_DIA;
        
        // Slot: rectangular cut from back edge (X negative) to center + semicircle for DOM + weld clearance
        // Slot faces 180° (back/left, toward X negative)
        translate([0, plate_thick/2, tube_h/2]) 
        rotate([90,0,0]) 
        linear_extrude(height=plate_thick+2, center=true) {
            // Rectangular slot from back edge to tube center
            translate([-pivot_plate_r - 1, -_slot_dia/2])
                square([pivot_plate_r + 1, _slot_dia]);
            // Semicircle at end of slot for DOM + weld clearance
            circle(d=_slot_dia, $fn=64);
        }
        
        // 5 bolt holes for pivot mount assembly (on 4.5" bolt circle)
        // These match the holes in the pivot mount circular plates
        // NON-UNIFORM spacing: larger gap around slot, closer spacing elsewhere
        translate([0, plate_thick/2, tube_h/2])
        rotate([90,0,0])
        for (i = [0:_PIVOT_MOUNT_BOLT_COUNT-1]) {
            angle = _PIVOT_MOUNT_BOLT_ANGLES[i];
            bolt_x = (_PIVOT_MOUNT_BOLT_CIRCLE_DIA / 2) * cos(angle);
            bolt_z = (_PIVOT_MOUNT_BOLT_CIRCLE_DIA / 2) * sin(angle);
            
            translate([bolt_x, bolt_z, 0])
                cylinder(d=_PIVOT_MOUNT_BOLT_DIA, h=plate_thick+2, center=true, $fn=24);
        }
        
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
        
        // 4. Tube Bolts (Rear Set - near pivot mount assembly)
        // Tube starts at 'overlap' from pivot center (pivot_plate_r + clearance)
        // TUBE_BOLT_INSET is distance from tube edge to the NEAREST bolt
        _tube_bolt_inset = is_undef(TUBE_BOLT_INSET) ? 50.8 : TUBE_BOLT_INSET; // Distance from tube end to nearest bolt
        translate([overlap + _tube_bolt_inset, plate_thick/2, tube_h/2])
            for (x = [0, bolt_spacing_x])
            for (z = [-bolt_spacing_y/2, bolt_spacing_y/2])
                translate([x, 0, z]) rotate([90,0,0]) cylinder(d=BOLT_DIA_1_2, h=plate_thick+2, center=true, $fn=32);

        // 5. Tube Bolts (Front Set - near elbow)
        translate([overlap + main_tube_len - _tube_bolt_inset - bolt_spacing_x, plate_thick/2, tube_h/2])
            for (x = [0, bolt_spacing_x])
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
