// angle_iron_a3_loader_mount.scad
// Loader Arm Mounting Angle - Part A3
// 2" × 2" × 1/4" Angle Iron
// Connects loader arm to side panels
// Quantity needed: 4 (2 per arm × 2 arms)

include <../../lifetrac_v25_params.scad>
use <../../modules/structural_steel.scad>

/**
 * Loader arm mounting angle iron with holes
 * Length: 146.05mm (5.75") - same as frame mount
 * Material: 2" × 2" × 1/4" angle iron
 */
module angle_iron_a3_loader_mount(show_holes=true) {
    leg = 50.8;  // 2" angle iron leg
    thick = 6.35;  // 1/4" thickness
    length = 146.05;  // 5.75"
    
    // Hole specifications
    hole_dia = 12.7;  // 1/2" (12.7mm)
    // Holes through horizontal leg (for plate mounting)
    plate_hole1_pos = 22.86;
    plate_hole2_pos = 123.19;
    // Holes through vertical leg (for beam mounting - different positions)
    beam_hole1_pos = 48.26;
    beam_hole2_pos = 97.79;
    
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
            translate([leg/2, -plate_hole1_pos, thick/2])
            cylinder(d=hole_dia, h=thick+2, center=true, $fn=24);
            
            translate([leg/2, -plate_hole2_pos, thick/2])
            cylinder(d=hole_dia, h=thick+2, center=true, $fn=24);
            
            // Holes through vertical leg (for beam mounting)
            translate([thick/2, -beam_hole1_pos, leg/2])
            rotate([0, 90, 0])
            cylinder(d=hole_dia, h=thick+2, center=true, $fn=24);
            
            translate([thick/2, -beam_hole2_pos, leg/2])
            rotate([0, 90, 0])
            cylinder(d=hole_dia, h=thick+2, center=true, $fn=24);
        }
    }
}

// Render the part
angle_iron_a3_loader_mount(show_holes=true);
