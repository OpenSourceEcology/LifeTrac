// Part A3: Front Stiffener - Outer Section Vertical Angle Iron
// Material: 2"×2"×1/4" Angle Iron (50.8mm × 6.35mm)
// Quantity: 6 pieces (outer walls + inner wall faces)
// Location: Front stiffener plate outer sections (5" tall sections)
//
// These are shorter angle irons used in the 5" tall outer sections of the front stiffener plate.

include <../../lifetrac_v25_params.scad>

// =============================================================================
// PART DIMENSIONS
// =============================================================================

// Front stiffener outer section is 5" (127mm) tall
_outer_height = 127.0;
_angle_trim = 3.175;  // 1/8" trimmed from each end

PART_A3_HEIGHT = _outer_height - 2 * _angle_trim;  // ~120.65mm (4.75")
PART_A3_LEG = 50.8;
PART_A3_THICK = 6.35;
PART_A3_BOLT_DIA = 9.525;  // 3/8" bolts for outer sections
PART_A3_HOLE_OFFSET = PART_A3_LEG / 2;  // Center of leg

// 2 holes spaced 3" (76.2mm) apart, centered on angle
_bolt_spacing = 38.1;  // 1.5" from center (3" total)
PART_A3_HOLES_LEG_A = [PART_A3_HEIGHT/2 - _bolt_spacing, PART_A3_HEIGHT/2 + _bolt_spacing];
PART_A3_HOLES_LEG_B = [PART_A3_HEIGHT/2 - _bolt_spacing, PART_A3_HEIGHT/2 + _bolt_spacing];

// =============================================================================
// MAIN MODULE
// =============================================================================

/**
 * Part A3: Front Stiffener Outer Section Vertical Angle Iron
 * Shorter angle for 5" tall outer sections of front stiffener plate
 */
module _part_a3_front_outer_vertical(show_holes=true) {
    height = PART_A3_HEIGHT;
    leg = PART_A3_LEG;
    thick = PART_A3_THICK;
    bolt_dia = PART_A3_BOLT_DIA;
    hole_offset = PART_A3_HOLE_OFFSET;
    
    color("DarkGray")
    difference() {
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
            // Holes on Leg A
            for (z = PART_A3_HOLES_LEG_A) {
                translate([hole_offset, 0, z])
                rotate([90, 0, 0])
                cylinder(d=bolt_dia, h=40, center=true, $fn=16);
            }
            
            // Holes on Leg B
            for (z = PART_A3_HOLES_LEG_B) {
                translate([0, hole_offset, z])
                rotate([0, 90, 0])
                cylinder(d=bolt_dia, h=40, center=true, $fn=16);
            }
        }
    }
}

// =============================================================================
// RENDER
// =============================================================================

_part_a3_front_outer_vertical(show_holes=true);

echo("=== PART A3: Front Stiffener Outer Section Vertical ===");
echo("Height:", PART_A3_HEIGHT, "mm (", PART_A3_HEIGHT/25.4, "inches)");
echo("Bolt diameter:", PART_A3_BOLT_DIA, "mm (3/8\")");
echo("Holes per leg: 2");
echo("Quantity needed: 6 pieces");
