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
 *   - Pivot hole end at +X, bolt holes at -X
 *   - Designed to be oriented vertically when installed
 */
module platform_pivot_bracket() {
    // Local variables for clarity
    length = PLATFORM_BRACKET_LENGTH;
    width = PLATFORM_BRACKET_WIDTH;
    thickness = PLATFORM_THICKNESS;
    
    // Pivot hole parameters
    pivot_hole_dia = PLATFORM_PIVOT_PIN_DIA + PLATFORM_BOLT_CLEARANCE;
    pivot_x = length/2 - PLATFORM_PIVOT_PIN_DIA/2 - 10;  // 10mm from edge
    
    // Lock pin hole parameters
    lock_hole_dia = PLATFORM_LOCK_PIN_DIA + PLATFORM_BOLT_CLEARANCE;
    lock_x = pivot_x - PLATFORM_LOCK_OFFSET;  // Offset toward bolt end
    
    // Angle iron bolt hole parameters
    bolt_hole_dia = PLATFORM_BOLT_DIA + PLATFORM_BOLT_CLEARANCE;
    bolt_x = -length/2 + PLATFORM_ANGLE_LEG/2 + 15;  // Position for angle iron attachment
    bolt_spacing = PLATFORM_ANGLE_BOLT_OFFSET;  // Vertical spacing
    
    difference() {
        // Base plate - rectangular with rounded corners for strength
        hull() {
            for (x = [-length/2 + 10, length/2 - 10]) {
                for (y = [-width/2 + 10, width/2 - 10]) {
                    translate([x, y, 0])
                    cylinder(r=10, h=thickness, center=true, $fn=24);
                }
            }
        }
        
        // Pivot pin hole (1" diameter + clearance)
        translate([pivot_x, 0, 0])
        cylinder(d=pivot_hole_dia, h=thickness + 4, center=true, $fn=48);
        
        // Lock pin hole (3/8" diameter + clearance)
        // Located between pivot and bolt holes, for deployed position lock
        translate([lock_x, 0, 0])
        cylinder(d=lock_hole_dia, h=thickness + 4, center=true, $fn=32);
        
        // Angle iron mounting bolt holes (3 holes in vertical line)
        // These align with holes in the angle iron
        for (y_off = [-bolt_spacing, 0, bolt_spacing]) {
            translate([bolt_x, y_off, 0])
            cylinder(d=bolt_hole_dia, h=thickness + 4, center=true, $fn=32);
        }
        
        // Weight reduction slot (optional, between bolt holes and pivot)
        slot_x = (bolt_x + lock_x) / 2;
        slot_length = abs(lock_x - bolt_x) - 30;
        if (slot_length > 20) {
            translate([slot_x, 0, 0])
            hull() {
                translate([-slot_length/4, 0, 0])
                cylinder(d=width * 0.4, h=thickness + 4, center=true, $fn=24);
                translate([slot_length/4, 0, 0])
                cylinder(d=width * 0.4, h=thickness + 4, center=true, $fn=24);
            }
        }
    }
}

// Render the part for preview/export
if ($preview || len(search("platform_pivot_bracket.scad", parent_modules())) == 0) {
    platform_pivot_bracket();
}
