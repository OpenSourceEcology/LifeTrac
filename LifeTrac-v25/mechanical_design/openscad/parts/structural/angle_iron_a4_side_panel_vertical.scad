// angle_iron_a4_side_panel_vertical.scad
// Side Panel Vertical Angle - Part A4
// 2" × 2" × 1/4" Angle Iron
// Vertical stiffeners for side panels
// Quantity needed: 8

include <../../lifetrac_v25_params.scad>
use <../../modules/structural_steel.scad>

/**
 * Side panel vertical stiffener angle iron with holes
 * Length: 550mm (21.65")
 * Material: 2" × 2" × 1/4" angle iron
 */
module angle_iron_a4_side_panel_vertical(show_holes=true) {
    leg = 50.8;  // 2" angle iron leg
    thick = 6.35;  // 1/4" thickness
    length = 550.0;  // 550mm
    
    // Hole specifications
    hole_dia = 9.525;  // 3/8" (9.525mm)
    hole1_pos = 60.0;   // First hole
    hole2_pos = 490.0;  // Second hole
    
    difference() {
        // Main angle iron shape
        color("Gray")
        rotate([90, 0, 0])
        linear_extrude(height=length)
        polygon([
            [0, 0],
            [leg, 0],
            [leg, thick],
            [thick, thick],
            [thick, leg],
            [0, leg]
        ]);
        
        if (show_holes) {
            // Holes through horizontal leg (for panel mounting)
            translate([leg/2, -hole1_pos, thick/2])
            cylinder(d=hole_dia, h=thick+2, center=true, $fn=24);
            
            translate([leg/2, -hole2_pos, thick/2])
            cylinder(d=hole_dia, h=thick+2, center=true, $fn=24);
        }
    }
}

// Render the part
angle_iron_a4_side_panel_vertical(show_holes=true);
