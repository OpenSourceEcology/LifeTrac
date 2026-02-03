// Part A5: Arm Crossbeam Mount Angle Iron
// Material: 2"×2"×1/4" Angle Iron (50.8mm × 6.35mm)
// Quantity: 4 pieces (2 per arm, top and bottom)
// Location: Connecting arm side plates to cross beam
//
// Mounts the 2×6 cross beam to the loader arm side plates.
// Same dimensions as frame tube mount (A4) but used on the arms.

include <../../lifetrac_v25_params.scad>

// =============================================================================
// PART DIMENSIONS
// =============================================================================

// Crossbeam is 6" (152.4mm) in one dimension
_angle_trim = 3.175;  // 1/8" trim from each end

PART_A5_HEIGHT = 152.4 - 2 * _angle_trim;  // 146.05mm (5.75")
PART_A5_LEG = 50.8;
PART_A5_THICK = 6.35;
PART_A5_BOLT_DIA = 12.7;  // 1/2" bolts

// Hole spacing
_plate_bolt_offset = 50.8;  // 2" from center for plate side
_beam_bolt_offset = 25.4;   // 1" from center for beam side
_center_z = PART_A5_HEIGHT / 2;

PART_A5_HOLES_LEG_A = [_center_z - _plate_bolt_offset, _center_z + _plate_bolt_offset];
PART_A5_HOLES_LEG_B = [_center_z - _beam_bolt_offset, _center_z + _beam_bolt_offset];

// =============================================================================
// MAIN MODULE
// =============================================================================

/**
 * Part A5: Arm Crossbeam Mount Angle Iron
 * 
 * Same hole pattern as frame tube mount but used on loader arms.
 * Installed with:
 *   - Vertical leg against arm side plate
 *   - Horizontal leg against cross beam
 */
module _part_a5_arm_crossbeam_mount(show_holes=true) {
    height = PART_A5_HEIGHT;
    leg = PART_A5_LEG;
    thick = PART_A5_THICK;
    bolt_dia = PART_A5_BOLT_DIA;
    
    color("DarkGray")
    difference() {
        union() {
            // Vertical leg (against plate)
            cube([height, thick, leg]);
            // Horizontal leg (against beam)
            cube([height, leg, thick]);
        }
        
        if (show_holes) {
            // Holes in vertical leg (plate bolts)
            for (x = PART_A5_HOLES_LEG_A) {
                translate([x, thick + 1, leg/2])
                rotate([90, 0, 0])
                cylinder(d=bolt_dia, h=thick+2, $fn=32);
            }
            
            // Holes in horizontal leg (beam bolts)
            for (x = PART_A5_HOLES_LEG_B) {
                translate([x, leg/2, -1])
                cylinder(d=bolt_dia, h=thick+2, $fn=32);
            }
        }
    }
}

// =============================================================================
// RENDER
// =============================================================================

_part_a5_arm_crossbeam_mount(show_holes=true);

echo("=== PART A5: Arm Crossbeam Mount Angle Iron ===");
echo("Height:", PART_A5_HEIGHT, "mm");
echo("Bolt diameter:", PART_A5_BOLT_DIA, "mm (1/2\")");
echo("Plate side spacing: 4\" (101.6mm)");
echo("Beam side spacing: 2\" (50.8mm)");
echo("Quantity needed: 4 pieces");
