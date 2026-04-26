// side_panel.scad
// Triangular side panel for LifeTrac v25
// This is the main structural plate that forms the sides of the machine
// Part of the "sandwich" design with 4 panels total (2 inner, 2 outer)

// Include necessary parameters from main assembly
include <../lifetrac_v25_params.scad>

// Triangular profile for side panels
// Tall end at rear (Y=0), shorter sloped end at front (Y=WHEEL_BASE)
module side_panel_profile() {
    // Calculate arm bottom slope line
    // Pivot point in panel coordinates
    pivot_y_panel = ARM_PIVOT_Y;
    pivot_z_panel = MACHINE_HEIGHT - 50;
    
    // Arm angle (ensure it's defined)
    arm_angle = is_undef(ARM_MIN_ANGLE) ? -45 : ARM_MIN_ANGLE;
    
    // Vertical offset to clear arm bottom (Half tube height + clearance)
    // Tube is 3" (76.2mm), half is 38.1mm. Add ~40mm clearance.
    arm_clearance_z = 80;
    
    // Function to calculate wall height at a given Y based on arm slope
    function wall_height(y) = 
        let(
            dy = y - pivot_y_panel,
            dz = dy * tan(arm_angle)
        )
        pivot_z_panel + dz - arm_clearance_z;

    // Define key Y coordinates
    y_rear = 0;
    y_pivot_start = pivot_y_panel - 100; // Start of pivot area
    y_pivot_end = pivot_y_panel + 100;   // End of pivot area (start of slope)
    y_front = WHEEL_BASE;                // Front of machine
    
    // Calculate Z heights
    z_pivot_end = wall_height(y_pivot_end);
    z_front_slope = wall_height(y_front - 200); // Point before final drop
    
    offset(r=10) offset(r=-10)  // Rounded corners
    polygon([
        [0, 0],                              // Rear bottom
        [WHEEL_BASE, 0],                     // Front bottom
        [WHEEL_BASE, 0],                     // Front top (drop to corner)
        [WHEEL_BASE - 200, z_front_slope],   // End of arm slope
        [y_pivot_end, z_pivot_end],          // Start of arm slope
        [pivot_y_panel, MACHINE_HEIGHT],     // Pivot high point
        [0, MACHINE_HEIGHT]                  // Rear top (full height)
    ]);
}

// Arc slot cutout for cross beam clearance
// Creates an arc-shaped slot that allows the cross beam to rotate through
module cross_beam_arc_slot(beam_distance, slot_width) {
    // Pivot point in panel local coordinates
    pivot_x = PIVOT_PANEL_X;
    pivot_y = PIVOT_PANEL_Y;
    
    // Arc radius = distance from pivot to cross beam center
    arc_radius = beam_distance;
    
    // Half-width of the slot (beam size + clearance)
    half_slot = slot_width / 2;
    
    // Generate arc by rotating around pivot
    // The arc sweeps from ARM_MIN_ANGLE to ARM_MAX_ANGLE
    // We extend slightly beyond to ensure full clearance
    // Safety check for undefined angles
    safe_min_angle = is_undef(ARM_MIN_ANGLE) ? -45 : ARM_MIN_ANGLE;
    safe_max_angle = is_undef(ARM_MAX_ANGLE) ? 60 : ARM_MAX_ANGLE;
    
    arc_start = safe_min_angle - 5;
    arc_end = safe_max_angle + 5;
    arc_steps = 60;
    
    translate([pivot_x, pivot_y])
    for (i = [0:arc_steps-1]) {
        angle1 = arc_start + i * (arc_end - arc_start) / arc_steps;
        angle2 = arc_start + (i + 1) * (arc_end - arc_start) / arc_steps;
        
        // Inner and outer points for this arc segment
        hull() {
            // Point at angle1, inner radius
            translate([(arc_radius - half_slot) * cos(angle1), (arc_radius - half_slot) * sin(angle1)])
            circle(d=5, $fn=8);
            
            // Point at angle1, outer radius
            translate([(arc_radius + half_slot) * cos(angle1), (arc_radius + half_slot) * sin(angle1)])
            circle(d=5, $fn=8);
            
            // Point at angle2, inner radius
            translate([(arc_radius - half_slot) * cos(angle2), (arc_radius - half_slot) * sin(angle2)])
            circle(d=5, $fn=8);
            
            // Point at angle2, outer radius
            translate([(arc_radius + half_slot) * cos(angle2), (arc_radius + half_slot) * sin(angle2)])
            circle(d=5, $fn=8);
        }
    }
}

// Arc slot cutout for hydraulic bracket clearance
// Creates an arc-shaped divot with gusset profile that allows the arm's 
// hydraulic mounting extension to rotate through without collision
module hydraulic_bracket_arc_slot(bracket_pos, bracket_dia, clearance) {
    // Pivot point in panel local coordinates
    pivot_x = PIVOT_PANEL_X;
    pivot_y = PIVOT_PANEL_Y;
    
    // Bracket circle parameters
    circle_r = bracket_dia / 2;
    tube_h = is_undef(TUBE_2X6_1_4) ? 152.4 : TUBE_2X6_1_4[1];  // 6" tube height
    tube_w = is_undef(TUBE_2X6_1_4) ? 50.8 : TUBE_2X6_1_4[0];   // 2" tube width
    
    // Cutout depth - only 3" deep (not full bracket depth)
    cutout_depth = 76.2;  // 3 inches
    
    // Gusset attachment length (matching arm plate: 3x tube width)
    gusset_attach_len = tube_w * 3;
    
    // The cutout profile mirrors the arm plate extension but shallower
    // Circle center offset from arm axis (at tube bottom - cutout_depth/2)
    cutout_circle_r = cutout_depth / 2;
    
    // Offset perpendicular to arm axis (below the arm)
    perp_offset = tube_h/2 + cutout_circle_r;
    
    // Total cutout radius (half of cutout depth + clearance)
    cutout_r = cutout_circle_r + clearance;
    
    // Half of gusset attachment (for the triangular profile)
    gusset_half = gusset_attach_len / 2 + clearance;
    
    // Safety check for undefined angles
    safe_min_angle = is_undef(ARM_MIN_ANGLE) ? -45 : ARM_MIN_ANGLE;
    safe_max_angle = is_undef(ARM_MAX_ANGLE) ? 60 : ARM_MAX_ANGLE;
    
    // Arc sweep with buffer
    arc_start = safe_min_angle - 5;
    arc_end = safe_max_angle + 5;
    arc_steps = 40;
    
    // Generate arc slot with gusset profile by sweeping the shape along the arc path
    // The gusset rotates WITH the arm, so wide edge stays perpendicular to arm direction
    translate([pivot_x, pivot_y])
    for (i = [0:arc_steps-1]) {
        angle1 = arc_start + i * (arc_end - arc_start) / arc_steps;
        angle2 = arc_start + (i + 1) * (arc_end - arc_start) / arc_steps;
        
        hull() {
            // Profile at angle1 - rotates with arm
            rotate([0, 0, angle1]) {
                // Wide edge at tube bottom (along arm direction)
                translate([bracket_pos - gusset_half, -tube_h/2 - clearance])
                square([gusset_attach_len + 2*clearance, 1]);
                
                // Circle at bottom of gusset (perpendicular to arm, below it)
                translate([bracket_pos, -perp_offset])
                circle(r=cutout_r, $fn=24);
            }
            
            // Profile at angle2 - rotates with arm
            rotate([0, 0, angle2]) {
                // Wide edge at tube bottom
                translate([bracket_pos - gusset_half, -tube_h/2 - clearance])
                square([gusset_attach_len + 2*clearance, 1]);
                
                // Circle at bottom of gusset
                translate([bracket_pos, -perp_offset])
                circle(r=cutout_r, $fn=24);
            }
        }
    }
}

// Main side panel module with all features (holes, slots)
// is_inner: true for inner panels (with arc slots), false for outer panels
module side_panel(is_inner = false) {
    slot_width = CROSS_BEAM_SIZE + 2 * CROSS_BEAM_CLEARANCE;  // Total slot width with clearance
    
    // Hydraulic bracket parameters (with fallback defaults)
    hyd_bracket_pos = is_undef(HYD_BRACKET_ARM_POS) ? 300 : HYD_BRACKET_ARM_POS;
    hyd_bracket_dia = is_undef(HYD_BRACKET_CIRCLE_DIA) ? 76.2 : HYD_BRACKET_CIRCLE_DIA;
    hyd_bracket_clearance = is_undef(HYD_BRACKET_CUTOUT_CLEARANCE) ? 15 : HYD_BRACKET_CUTOUT_CLEARANCE;
    
    difference() {
        linear_extrude(height=PANEL_THICKNESS)
        difference() {
            side_panel_profile();
            
            // Cross beam arc slots (only on inner panels)
            if (is_inner) {
                // First cross beam slot (near pivot)
                cross_beam_arc_slot(CROSS_BEAM_1_POS, slot_width);
                
                // Second cross beam slot (near bucket) - only if it intersects the panel
                // The second beam is far forward and may not need a cutout
                // Check if any part of the arc passes through the panel
                if (CROSS_BEAM_2_POS < WHEEL_BASE + 200) {
                    cross_beam_arc_slot(CROSS_BEAM_2_POS, slot_width);
                }
                
                // Hydraulic bracket arc slot (inner panels only)
                // This prevents the bracket mounting bolts from colliding with the wall
                hydraulic_bracket_arc_slot(hyd_bracket_pos, hyd_bracket_dia, hyd_bracket_clearance);
            }
        }
        
        // Arm pivot hole near rear top (where the tall section is)
        // Position matches ARM_PIVOT_Y in world coordinates
        translate([ARM_PIVOT_Y, MACHINE_HEIGHT - 50, PANEL_THICKNESS/2])
        cylinder(d=PIVOT_PIN_DIA + 2, h=PANEL_THICKNESS+4, center=true, $fn=48);
        
        // Lift cylinder base mount hole (on both inner and outer panels)
        // Position matches LIFT_CYL_BASE_Y in world coordinates (Y=0 is rear)
        translate([LIFT_CYL_BASE_Y, LIFT_CYL_BASE_Z - FRAME_Z_OFFSET, PANEL_THICKNESS/2])
        cylinder(d=BOLT_DIA_1 + 2, h=PANEL_THICKNESS+4, center=true, $fn=32);
        
        // Wheel axle holes
        translate([0, WHEEL_DIAMETER/2 - 50, PANEL_THICKNESS/2])
        cylinder(d=BOLT_DIA_3_4 + 2, h=PANEL_THICKNESS+4, center=true, $fn=32);
        
        translate([WHEEL_BASE, WHEEL_DIAMETER/2 - 50, PANEL_THICKNESS/2])
        cylinder(d=BOLT_DIA_3_4 + 2, h=PANEL_THICKNESS+4, center=true, $fn=32);
        
        // =================================================================
        // FOLDING PLATFORM PIVOT HOLES (inner panels only)
        // =================================================================
        // These holes allow the platform pivot pin to pass through
        // Located near rear of panel, but clear of stiffener cutout zone
        // Stiffener cutout extends to X=31.75 in panel coords, so pivot must be beyond that
        // X position in panel coords = Y position in world coords
        // Y position in panel coords = Z position in world coords - FRAME_Z_OFFSET
        if (is_inner) {
            // Pivot Y position in world coords (clear of stiffener zone)
            pivot_world_y = PLATFORM_MOUNT_Y;  // Parametric forward position
            pivot_panel_x = pivot_world_y;  // Panel X = world Y
            pivot_panel_y = PLATFORM_PIVOT_HEIGHT - FRAME_Z_OFFSET;  // Panel Y = world Z - offset
            
            // Main pivot pin hole (1" + clearance)
            translate([pivot_panel_x, pivot_panel_y, PANEL_THICKNESS/2])
            cylinder(d=PLATFORM_PIVOT_PIN_DIA + PLATFORM_BOLT_CLEARANCE, 
                     h=PANEL_THICKNESS+4, center=true, $fn=48);
            
            // Deployed position lock pin hole (below pivot in Z = below in panel Y)
            translate([pivot_panel_x, pivot_panel_y - PLATFORM_LOCK_OFFSET, PANEL_THICKNESS/2])
            cylinder(d=PLATFORM_LOCK_PIN_DIA + PLATFORM_BOLT_CLEARANCE, 
                     h=PANEL_THICKNESS+4, center=true, $fn=32);
            
            // Stowed position lock pin hole (toward rear in Y = toward X=0 in panel)
            // When stowed, arm extends upward, lock pin aligns with arm end
            // This hole is at same Z as pivot but closer to rear
            stowed_lock_panel_x = pivot_panel_x - PLATFORM_ARM_LENGTH * 0.3;  // Offset toward rear
            if (stowed_lock_panel_x > 35) {  // Only if clear of stiffener cutout
                translate([stowed_lock_panel_x, pivot_panel_y + PLATFORM_ARM_LENGTH, PANEL_THICKNESS/2])
                cylinder(d=PLATFORM_LOCK_PIN_DIA + PLATFORM_BOLT_CLEARANCE, 
                         h=PANEL_THICKNESS+4, center=true, $fn=32);
            }
        }
    }
}

// Render the part for preview/export
if ($preview || len(search("side_panel.scad", parent_modules())) == 0) {
    side_panel(is_inner = false);  // Default to outer panel
}
