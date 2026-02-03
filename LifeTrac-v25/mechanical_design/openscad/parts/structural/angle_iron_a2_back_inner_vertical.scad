// Part A2: Back Stiffener - Inner Wall Vertical Angle Iron
// Material: 2"×2"×1/4" Angle Iron (50.8mm × 6.35mm)
// Quantity: 4 pieces (2 per inner panel, both sides)
// Location: Back stiffener plate, adjacent to inner wall panels
//
// This file defines a single structural steel part used in the LifeTrac v25 assembly.

include <../../lifetrac_v25_params.scad>

// =============================================================================
// PART DIMENSIONS
// =============================================================================

_back_z_start = FRAME_Z_OFFSET + 350;
_back_z_end = FRAME_Z_OFFSET + MACHINE_HEIGHT;

PART_A2_HEIGHT = _back_z_end - _back_z_start;
PART_A2_LEG = 50.8;
PART_A2_THICK = 6.35;
PART_A2_BOLT_DIA = 9.525;
PART_A2_HOLE_OFFSET = PART_A2_LEG * 0.6;

// Hole patterns (same as A1)
function _get_holes_a(h) = 
    let(start=50, end=50, spacing=150, count=floor((h-start-end)/spacing)+1)
    [for (i=[0:count-1]) start + i*spacing];

function _get_holes_b(h) = 
    let(start=100, end=50, spacing=150, count=floor((h-start-end)/spacing)+1)
    [for (i=[0:count-1]) start + i*spacing];

PART_A2_HOLES_LEG_A = _get_holes_a(PART_A2_HEIGHT);
PART_A2_HOLES_LEG_B = _get_holes_b(PART_A2_HEIGHT);

// =============================================================================
// MAIN MODULE
// =============================================================================

/**
 * Part A2: Back Stiffener Inner Wall Vertical Angle Iron
 * Identical dimensions to A1, used in 4 locations on inner panels
 */
module _part_a2_back_inner_vertical(show_holes=true) {
    height = PART_A2_HEIGHT;
    leg = PART_A2_LEG;
    thick = PART_A2_THICK;
    bolt_dia = PART_A2_BOLT_DIA;
    hole_offset = PART_A2_HOLE_OFFSET;
    
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
            for (z = PART_A2_HOLES_LEG_A) {
                translate([hole_offset, 0, z])
                rotate([90, 0, 0])
                cylinder(d=bolt_dia, h=40, center=true, $fn=16);
            }
            
            for (z = PART_A2_HOLES_LEG_B) {
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

_part_a2_back_inner_vertical(show_holes=true);

echo("=== PART A2: Back Stiffener Inner Wall Vertical ===");
echo("Height:", PART_A2_HEIGHT, "mm");
echo("Quantity needed: 4 pieces");
