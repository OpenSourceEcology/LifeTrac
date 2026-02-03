// frame_tube_t1_front.scad
// Front Frame Tube - Part T1
// 2" × 6" × 1/4" Rectangular Tubing
// Front cross frame tube passing through side panels
// Quantity needed: 1

include <../../lifetrac_v25_params.scad>
use <../../modules/structural_steel.scad>

/**
 * Front frame tube - 2" × 6" rectangular tubing
 * Length: Track width (distance between side panels)
 * Material: 2" × 6" × 1/4" rectangular tubing
 */
module frame_tube_t1_front(show_holes=false) {
    // Tube dimensions from params
    width = 50.8;    // 2" (Y direction)
    height = 152.4;  // 6" (Z direction)
    wall = 6.35;     // 1/4" wall thickness
    
    // Length spans the full track width between panels
    // Using approximate value - actual would be TRACK_WIDTH
    length = 1200.0;  // Approximate track width in mm
    
    // Inner dimensions
    inner_width = width - 2*wall;
    inner_height = height - 2*wall;
    
    color("DarkGray")
    translate([0, 0, 0])
    rotate([0, 90, 0])  // Tube runs along X axis
    difference() {
        // Outer tube
        cube([height, width, length], center=true);
        
        // Inner hollow
        cube([inner_height, inner_width, length+2], center=true);
    }
}

// Render the part
frame_tube_t1_front(show_holes=false);
