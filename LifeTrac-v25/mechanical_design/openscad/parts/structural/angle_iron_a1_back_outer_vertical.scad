// Part A1: Back Stiffener - Outer Wall Vertical Angle Iron
// Material: 2"×2"×1/4" Angle Iron (50.8mm × 6.35mm)
// Quantity: 2 pieces
// Location: Back stiffener plate, left and right outer walls
//
// This file defines a single structural steel part used in the LifeTrac v25 assembly.
// The angle iron runs vertically, connecting the back stiffener plate to the outer wall panels.

include <../../lifetrac_v25_params.scad>

// =============================================================================
// PART DIMENSIONS (calculated from assembly parameters)
// =============================================================================

// Back stiffener plate vertical extent
_back_y_pos = 25.4;  // 1 inch from rear
_back_z_start = FRAME_Z_OFFSET + 350;
_back_z_end = FRAME_Z_OFFSET + MACHINE_HEIGHT;

// Part dimensions
PART_A1_HEIGHT = _back_z_end - _back_z_start;
PART_A1_LEG = 50.8;      // 2" leg
PART_A1_THICK = 6.35;    // 1/4" thickness
PART_A1_BOLT_DIA = 9.525; // 3/8" bolts
PART_A1_HOLE_OFFSET = PART_A1_LEG * 0.6;  // Offset from corner for tool clearance

// Hole spacing pattern (parametric based on height)
function get_stiffener_holes_a_local(h) = 
    let(
        start_offset = 50,
        end_offset = 50,
        spacing = 150,
        count = floor((h - start_offset - end_offset) / spacing) + 1
    )
    [for (i = [0:count-1]) start_offset + i * spacing];

function get_stiffener_holes_b_local(h) = 
    let(
        start_offset = 100,
        end_offset = 50,
        spacing = 150,
        count = floor((h - start_offset - end_offset) / spacing) + 1
    )
    [for (i = [0:count-1]) start_offset + i * spacing];

// Generate hole position arrays
PART_A1_HOLES_LEG_A = get_stiffener_holes_a_local(PART_A1_HEIGHT);
PART_A1_HOLES_LEG_B = get_stiffener_holes_b_local(PART_A1_HEIGHT);

// =============================================================================
// MAIN MODULE
// =============================================================================

/**
 * Part A1: Back Stiffener Outer Wall Vertical Angle Iron
 * 
 * Orientation:
 *   - Extrudes along Z axis (vertical)
 *   - Corner at origin [0,0,0]
 *   - Leg A extends along +X (against wall panel)
 *   - Leg B extends along +Y (against stiffener plate)
 *
 * @param show_holes If true, drill bolt holes; if false, show solid angle
 */
module _part_a1_back_outer_vertical(show_holes=true) {
    height = PART_A1_HEIGHT;
    leg = PART_A1_LEG;
    thick = PART_A1_THICK;
    bolt_dia = PART_A1_BOLT_DIA;
    hole_offset = PART_A1_HOLE_OFFSET;
    
    color("DarkGray")
    difference() {
        // L-profile extruded along Z
        // Origin at corner, legs extend along +X and +Y
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
            // Holes on Leg A (X-leg) - bolts through in Y direction
            for (z = PART_A1_HOLES_LEG_A) {
                translate([hole_offset, 0, z])
                rotate([90, 0, 0])
                cylinder(d=bolt_dia, h=40, center=true, $fn=16);
            }
            
            // Holes on Leg B (Y-leg) - bolts through in X direction
            for (z = PART_A1_HOLES_LEG_B) {
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

// Render the part when this file is opened directly
_part_a1_back_outer_vertical(show_holes=true);

// Echo part specifications
echo("=== PART A1: Back Stiffener Outer Wall Vertical ===");
echo("Height:", PART_A1_HEIGHT, "mm");
echo("Leg size:", PART_A1_LEG, "mm");
echo("Thickness:", PART_A1_THICK, "mm");
echo("Bolt diameter:", PART_A1_BOLT_DIA, "mm");
echo("Holes Leg A:", len(PART_A1_HOLES_LEG_A));
echo("Holes Leg B:", len(PART_A1_HOLES_LEG_B));
