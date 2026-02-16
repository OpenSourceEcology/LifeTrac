// angle_iron_a2_frame_mount.scad
// Frame Tube Mounting Angle - Part A2
// 2" × 2" × 1/4" Angle Iron
// Used to mount frame tubes to side panels
// Quantity needed: 16 (2 per tube × 2 tubes × 4 panels)

include <../../lifetrac_v25_params.scad>
use <../../modules/structural_steel.scad>

/**
 * Frame tube mounting angle iron with holes
 * Length: 146.05mm (5.75") - 6" stock minus 2×1/8" trim
 * Material: 2" × 2" × 1/4" angle iron
 */
module angle_iron_a2_frame_mount(show_holes=true) {
    leg = 50.8;  // 2" angle iron leg
    thick = 6.35;  // 1/4" thickness  
    length = 146.05;  // 5.75" (6" stock - 2×trim)
    
    // Hole specifications
    hole_dia = 12.7;  // 1/2" (12.7mm)
    hole1_pos = 22.86;  // First hole position from end
    hole2_pos = 123.19; // Second hole position from end
    
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
            // Holes through horizontal leg (for plate mounting)
            translate([leg/2, -hole1_pos, thick/2])
            cylinder(d=hole_dia, h=thick+2, center=true, $fn=24);
            
            translate([leg/2, -hole2_pos, thick/2])
            cylinder(d=hole_dia, h=thick+2, center=true, $fn=24);
            
            // Holes through vertical leg (for tube mounting)
            translate([thick/2, -hole1_pos, leg/2])
            rotate([0, 90, 0])
            cylinder(d=hole_dia, h=thick+2, center=true, $fn=24);
            
            translate([thick/2, -hole2_pos, leg/2])
            rotate([0, 90, 0])
            cylinder(d=hole_dia, h=thick+2, center=true, $fn=24);
        }
    }
}

// Render the part
angle_iron_a2_frame_mount(show_holes=true);
