// side_panel.scad
// Triangular side panel for LifeTrac v25
// This is the main structural plate that forms the sides of the machine
// Part of the "sandwich" design with 4 panels total (2 inner, 2 outer)

// Include necessary parameters from main assembly
include <../lifetrac_v25_params.scad>

// Triangular profile for side panels
// Tall end at rear (Y=0), shorter sloped end at front (Y=WHEEL_BASE)
module side_panel_profile() {
    offset(r=10) offset(r=-10)  // Rounded corners
    polygon([
        [0, 0],                              // Rear bottom
        [WHEEL_BASE, 0],                     // Front bottom
        [WHEEL_BASE, MACHINE_HEIGHT * 0.65], // Front top (sloped down)
        [200, MACHINE_HEIGHT],               // Near-rear top (arm pivot area)
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
    arc_start = ARM_MIN_ANGLE - 5;
    arc_end = ARM_MAX_ANGLE + 5;
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

// Main side panel module with all features (holes, slots)
// is_inner: true for inner panels (with arc slots), false for outer panels
module side_panel(is_inner = false) {
    slot_width = CROSS_BEAM_SIZE + 2 * CROSS_BEAM_CLEARANCE;  // Total slot width with clearance
    
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
            }
        }
        
        // Arm pivot hole near rear top (where the tall section is)
        // Position matches ARM_PIVOT_Y in world coordinates
        translate([ARM_PIVOT_Y, MACHINE_HEIGHT - 50, PANEL_THICKNESS/2])
        cylinder(d=PIVOT_PIN_DIA + 2, h=PANEL_THICKNESS+4, center=true, $fn=48);
        
        // Lift cylinder base mount hole (only on inner panels)
        if (is_inner) {
            translate([WHEEL_BASE - LIFT_CYL_BASE_Y, LIFT_CYL_BASE_Z, PANEL_THICKNESS/2])
            cylinder(d=BOLT_DIA_1 + 2, h=PANEL_THICKNESS+4, center=true, $fn=32);
        }
        
        // Wheel axle holes
        translate([0, WHEEL_DIAMETER/2 - 50, PANEL_THICKNESS/2])
        cylinder(d=BOLT_DIA_3_4 + 2, h=PANEL_THICKNESS+4, center=true, $fn=32);
        
        translate([WHEEL_BASE, WHEEL_DIAMETER/2 - 50, PANEL_THICKNESS/2])
        cylinder(d=BOLT_DIA_3_4 + 2, h=PANEL_THICKNESS+4, center=true, $fn=32);
    }
}

// Render the part for preview/export
if ($preview || len(search("side_panel.scad", parent_modules())) == 0) {
    side_panel(is_inner = false);  // Default to outer panel
}
