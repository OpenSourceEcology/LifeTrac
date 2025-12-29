// platform_pivot_bracket.scad
// Pivot bracket plate for LifeTrac v25 folding platform
// CNC cut from 1/4" plate
// Part of the 5-component folding platform system
//
// This bracket connects the angle iron arm to the machine frame:
//   - One end has bolt holes for angle iron attachment
//   - Other end has a 1" pivot hole and lock pin hole
//
// Two brackets required (left and right side)

include <../lifetrac_v25_params.scad>

/**
 * Creates the pivot bracket plate with pivot hole, lock hole, and bolt holes
 * All dimensions are parametric from lifetrac_v25_params.scad
 * 
 * Coordinate system when rendered:
 *   - Centered at origin in X and Y
 *   - Thickness in Z direction
 *   - Lock hole at Origin [0,0]
 *   - Pivot hole at [0, PLATFORM_LOCK_OFFSET]
 *   - Bolt holes at +X
 *   - Designed to be oriented vertically when installed
 */
module platform_pivot_bracket() {
    // Local variables
    thickness = PLATFORM_THICKNESS;
    width = PLATFORM_BRACKET_WIDTH;
    
    // Dimensions relative to Pivot Hole at (0,0)
    pivot_y = 0;
    lock_y = -PLATFORM_LOCK_OFFSET; // -60mm
    corner_y = lock_y + PLATFORM_SIDE_BOLT_Y_OFFSET; // -130mm
    
    // Corner radius - using width/2 to maintain constant width and avoid pinching
    corner_radius = width / 2; 
    
    // Bolt positions
    // Moved to Leg 2 (Horizontal in File X, Vertical in Assembly Z)
    // This places bolts on the "bottom" leg relative to the L-shape in 2D view
    
    // Y position is constant (centered on Leg 2)
    // Leg 2 is centered at corner_y. Width is width.
    // We want bolts in the "bottom half" of the leg width.
    // Bottom edge is corner_y - width/2.
    // Bolt Y = Bottom Edge + LEG/2.
    bolt_y = corner_y - width/2 + PLATFORM_ANGLE_LEG/2;
    
    // X positions (distributed along Leg 2)
    // Leg 2 extends from X=0 to X=PLATFORM_BRACKET_LENGTH - width/2
    // We place bolts at pivot_bolt_offset from each end of this span
    leg2_length = PLATFORM_BRACKET_LENGTH - width/2;
    bolt_x_1 = pivot_bolt_offset;
    bolt_x_2 = leg2_length - pivot_bolt_offset;
    
    // Hole diameters
    pivot_hole_dia = PLATFORM_PIVOT_PIN_DIA + PLATFORM_BOLT_CLEARANCE;
    bolt_hole_dia = PLATFORM_BOLT_DIA + PLATFORM_BOLT_CLEARANCE;
    lock_hole_dia = PLATFORM_LOCK_PIN_DIA + PLATFORM_BOLT_CLEARANCE;
    
    difference() {
        union() {
            // Vertical Leg (Pivot to Corner) - Along Y
            hull() {
                // Pivot position
                translate([0, pivot_y, 0])
                cylinder(d=width, h=thickness, center=true, $fn=48);
                
                // Corner position
                translate([0, corner_y, 0])
                cylinder(r=corner_radius, h=thickness, center=true, $fn=48);
            }
            
            // Horizontal Leg (Corner to End) - Along X
            hull() {
                // Corner position
                translate([0, corner_y, 0])
                cylinder(r=corner_radius, h=thickness, center=true, $fn=48);
                
                // Extended to reach back of platform deck (PLATFORM_BRACKET_LENGTH)
                translate([PLATFORM_BRACKET_LENGTH - width/2, corner_y, 0]) 
                cylinder(d=width, h=thickness, center=true, $fn=48);
            }
        }
        
        // Pivot hole at [0, 0]
        translate([0, pivot_y, 0])
        cylinder(d=pivot_hole_dia, h=thickness + 4, center=true, $fn=48);
        
        // Lock pin hole at [0, -60]
        translate([0, lock_y, 0])
        cylinder(d=lock_hole_dia, h=thickness + 4, center=true, $fn=32);
        
        // Bolt holes
        // Positioned along Leg 2 (X axis)
        for (bx = [bolt_x_1, bolt_x_2]) {
            translate([bx, bolt_y, 0])
            cylinder(d=bolt_hole_dia, h=thickness + 4, center=true, $fn=32);
        }
    }
}

// Render the part for preview/export
if ($preview || len(search("platform_pivot_bracket.scad", parent_modules())) == 0) {
    platform_pivot_bracket();
}
