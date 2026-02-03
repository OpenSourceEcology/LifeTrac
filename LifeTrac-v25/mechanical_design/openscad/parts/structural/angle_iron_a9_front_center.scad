// Part A9: Front Stiffener Center Section Angle Iron
// Material: 2"×2"×1/4" Angle Iron (50.8mm leg × 6.35mm thickness)
// Quantity: 2 pieces (left motor plate, right motor plate)
// Location: Front stiffener plate, 10" center section between motor plates
//
// These are the same height as frame tube angles (5.75" / 146.05mm)
// Used to connect the 10" tall center section to the motor mount plates
// Bolt pattern: 1/2" bolts, 4" spacing (same as frame tube angles)

include <../../lifetrac_v25_params.scad>

// =============================================================================
// PART DIMENSIONS
// =============================================================================

_leg = 50.8;    // 2" angle leg
_thick = 6.35;  // 1/4" thickness
_trim = 3.175;  // 1/8" trimmed from each end

// Height matches frame tube angle: 6" tube - 2×(1/8" trim) = 5.75"
_frame_tube_height = is_undef(FRAME_TUBE_HEIGHT) ? 152.4 : FRAME_TUBE_HEIGHT;

PART_A9_HEIGHT = _frame_tube_height - 2*_trim;  // 146.05mm (5.75")
PART_A9_LEG = _leg;
PART_A9_THICK = _thick;

// Bolt parameters (match frame tube angle pattern)
_bolt_dia = is_undef(BOLT_DIA_1_2) ? 12.7 : BOLT_DIA_1_2;  // 1/2" bolts
_plate_bolt_offset = 50.8;  // 2" from center (4" total spacing)
_hole_offset = _leg / 2;    // Center of angle leg

// =============================================================================
// MAIN MODULE
// =============================================================================

/**
 * Part A9: Front Stiffener Center Section Angle Iron
 * Vertical angle iron for 10" center section of front stiffener
 * Same dimensions as frame tube mounting angles
 * show_holes: if true, renders bolt holes
 */
module _part_a9_front_center_angle(show_holes=true) {
    height = PART_A9_HEIGHT;
    leg = PART_A9_LEG;
    thick = PART_A9_THICK;
    
    color("Silver")
    difference() {
        // L-profile
        linear_extrude(height=height)
        polygon([
            [0, 0],
            [leg, 0],
            [leg, thick],
            [thick, thick],
            [thick, leg],
            [0, leg]
        ]);
        
        if (show_holes) {
            // Vertical leg holes (through thickness, bolt to plate)
            // 2 holes at 4" spacing centered on height
            _center_z = height / 2;
            for (z_off = [-_plate_bolt_offset, _plate_bolt_offset]) {
                translate([_hole_offset, -1, _center_z + z_off])
                rotate([-90, 0, 0])
                cylinder(d=_bolt_dia, h=thick + 2, $fn=32);
            }
            
            // Horizontal leg holes (through thickness, bolt to stiffener plate)
            for (z_off = [-_plate_bolt_offset, _plate_bolt_offset]) {
                translate([-1, _hole_offset, _center_z + z_off])
                rotate([0, 90, 0])
                cylinder(d=_bolt_dia, h=thick + 2, $fn=32);
            }
        }
    }
}

// =============================================================================
// RENDER
// =============================================================================

_part_a9_front_center_angle(show_holes=true);

echo("=== PART A9: Front Stiffener Center Section Angle Iron ===");
echo("Height:", PART_A9_HEIGHT, "mm (", PART_A9_HEIGHT/25.4, "in)");
echo("Leg:", PART_A9_LEG, "mm (2 inches)");
echo("Thickness:", PART_A9_THICK, "mm (1/4 inch)");
echo("Bolt pattern: 2 holes per leg, 4\" spacing, 1/2\" dia");
echo("Quantity needed: 2 pieces (left and right motor plate faces)");
echo("Note: Identical dimensions to frame tube mount angles (A4)");
