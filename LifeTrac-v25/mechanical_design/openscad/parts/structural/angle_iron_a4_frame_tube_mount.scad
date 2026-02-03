// Part A4: Frame Tube Mount Angle Iron
// Material: 2"×2"×1/4" Angle Iron (50.8mm × 6.35mm)
// Quantity: 16 pieces (2 tubes × 4 walls × 2 faces)
// Location: Mounting 2×6 cross frame tubes to wall panels
//
// This is the standard frame tube mounting angle with asymmetric hole spacing:
// - Wall side (Leg A): 4" spacing (2" from center) with 1/2" bolts
// - Tube side (Leg B): 2" spacing (1" from center) with 1/2" bolts

include <../../lifetrac_v25_params.scad>

// =============================================================================
// PART DIMENSIONS
// =============================================================================

// Frame tube height is 6" (152.4mm), angles are 1/8" shorter on each end
_frame_tube_height = 152.4;
_angle_trim = 3.175;  // 1/8" trim from each end

PART_A4_HEIGHT = _frame_tube_height - 2 * _angle_trim;  // 146.05mm (5.75")
PART_A4_LEG = 50.8;      // 2"
PART_A4_THICK = 6.35;    // 1/4"
PART_A4_BOLT_DIA = 12.7; // 1/2" bolts

// Hole positions (from bottom of angle)
_plate_bolt_offset = 50.8;  // 2" from center for wall side (4" total spacing)
_tube_bolt_offset = 25.4;   // 1" from center for tube side (2" total spacing)
_hole_inset = 25.4;         // 1" from corner for hole placement on tube leg

// Centered on height for both
_center_z = PART_A4_HEIGHT / 2;

// Leg A holes (against wall) - 4" spacing
PART_A4_HOLES_LEG_A = [_center_z - _plate_bolt_offset, _center_z + _plate_bolt_offset];

// Leg B holes (against tube) - 2" spacing, inset from corner
PART_A4_HOLES_LEG_B = [_center_z - _tube_bolt_offset, _center_z + _tube_bolt_offset];
PART_A4_TUBE_HOLE_INSET = _hole_inset;

// =============================================================================
// MAIN MODULE
// =============================================================================

/**
 * Part A4: Frame Tube Mount Angle Iron
 * 
 * Orientation when installed:
 *   - Vertical leg against wall panel (bolts in X direction)
 *   - Horizontal leg against frame tube (bolts in Y direction)
 *   - Extrusion runs vertically along Z
 *
 * Hole pattern:
 *   - Leg A (wall): 2 holes, 4" apart, 1/2" bolts
 *   - Leg B (tube): 2 holes, 2" apart, 1/2" bolts, 1" from corner
 */
module _part_a4_frame_tube_mount(show_holes=true) {
    height = PART_A4_HEIGHT;
    leg = PART_A4_LEG;
    thick = PART_A4_THICK;
    bolt_dia = PART_A4_BOLT_DIA;
    
    color("DarkGray")
    difference() {
        union() {
            // Vertical leg (against wall panel) - runs full height in Z, thin in X
            cube([thick, leg, height]);
            // Horizontal leg (toward tube) - extends in +X direction
            cube([leg, thick, height]);
        }
        
        if (show_holes) {
            // Holes in vertical leg (bolts through to wall plate in X direction)
            for (z = PART_A4_HOLES_LEG_A) {
                translate([-1, leg/2, z])
                rotate([0, 90, 0])
                cylinder(d=bolt_dia, h=thick+2, $fn=32);
            }
            
            // Holes in horizontal leg (bolts through in Y direction into tube side)
            for (z = PART_A4_HOLES_LEG_B) {
                translate([PART_A4_TUBE_HOLE_INSET, -1, z])
                rotate([-90, 0, 0])
                cylinder(d=bolt_dia, h=thick+2, $fn=32);
            }
        }
    }
}

// =============================================================================
// RENDER
// =============================================================================

_part_a4_frame_tube_mount(show_holes=true);

echo("=== PART A4: Frame Tube Mount Angle Iron ===");
echo("Height:", PART_A4_HEIGHT, "mm (", PART_A4_HEIGHT/25.4, "inches)");
echo("Bolt diameter:", PART_A4_BOLT_DIA, "mm (1/2\")");
echo("Wall side spacing: 4\" (101.6mm)");
echo("Tube side spacing: 2\" (50.8mm)");
echo("Quantity needed: 16 pieces");
