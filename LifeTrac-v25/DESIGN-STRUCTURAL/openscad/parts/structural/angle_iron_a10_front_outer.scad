// Part A10: Front Stiffener Outer Section Angle Iron
// Material: 2"×2"×1/4" Angle Iron (50.8mm leg × 6.35mm thickness)
// Quantity: 8 pieces total
//   - 2 on outer walls (left and right)
//   - 4 on inner walls (2 per side, both faces)
//   - 2 on motor plates, outer side (left and right)
// Location: Front stiffener plate, 5" outer sections
//
// Shorter than center angles: ~4.75" (120.65mm)
// Used in the 5" tall outer sections between motor plates and outer walls
// Bolt pattern: 3/8" bolts, 3" spacing

include <../../lifetrac_v25_params.scad>

// =============================================================================
// PART DIMENSIONS
// =============================================================================

_leg = 50.8;    // 2" angle leg
_thick = 6.35;  // 1/4" thickness
_trim = 3.175;  // 1/8" trimmed from each end
_outer_height = 127.0;  // 5" outer section height

PART_A10_HEIGHT = _outer_height - 2*_trim;  // ~120.65mm (4.75")
PART_A10_LEG = _leg;
PART_A10_THICK = _thick;

// Bolt parameters (smaller pattern for outer sections)
_bolt_dia = is_undef(BOLT_DIA_3_8) ? 9.525 : BOLT_DIA_3_8;  // 3/8" bolts
_outer_bolt_offset = 38.1;  // 1.5" from center (3" total spacing)
_hole_offset = _leg / 2;    // Center of angle leg

// =============================================================================
// MAIN MODULE
// =============================================================================

/**
 * Part A10: Front Stiffener Outer Section Angle Iron
 * Vertical angle iron for 5" outer sections of front stiffener
 * show_holes: if true, renders bolt holes
 */
module _part_a10_front_outer_angle(show_holes=true) {
    height = PART_A10_HEIGHT;
    leg = PART_A10_LEG;
    thick = PART_A10_THICK;
    
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
            // 2 holes at 3" spacing centered on height
            _center_z = height / 2;
            for (z_off = [-_outer_bolt_offset, _outer_bolt_offset]) {
                translate([_hole_offset, -1, _center_z + z_off])
                rotate([-90, 0, 0])
                cylinder(d=_bolt_dia, h=thick + 2, $fn=32);
            }
            
            // Horizontal leg holes (through thickness, bolt to stiffener plate)
            for (z_off = [-_outer_bolt_offset, _outer_bolt_offset]) {
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

_part_a10_front_outer_angle(show_holes=true);

echo("=== PART A10: Front Stiffener Outer Section Angle Iron ===");
echo("Height:", PART_A10_HEIGHT, "mm (", PART_A10_HEIGHT/25.4, "in)");
echo("Leg:", PART_A10_LEG, "mm (2 inches)");
echo("Thickness:", PART_A10_THICK, "mm (1/4 inch)");
echo("Bolt pattern: 2 holes per leg, 3\" spacing, 3/8\" dia");
echo("Quantity needed: 8 pieces total");
echo("  - 2 at outer walls (left, right)");
echo("  - 4 at inner walls (2 per side)");
echo("  - 2 at motor plates outer face (left, right)");
