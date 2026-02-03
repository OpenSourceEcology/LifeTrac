// Part T4: Main Arm Tube (Left and Right are identical)
// Material: 2"×6"×1/4" Rectangular Tubing (50.8mm × 152.4mm × 6.35mm wall)
// Quantity: 2 pieces (1 left, 1 right - mirror in assembly)
// Location: Main loader arm tube from pivot to elbow
//
// Length = ARM_MAIN_LEN (calculated from kinematic parameters)
// 2" face is vertical (Z), 6" face is horizontal (Y)

include <../../lifetrac_v25_params.scad>

// =============================================================================
// PART DIMENSIONS
// =============================================================================

_tube_w = is_undef(TUBE_2X6_1_4) ? 152.4 : TUBE_2X6_1_4[0];  // 6 inches (horizontal)
_tube_h = is_undef(TUBE_2X6_1_4) ? 50.8 : TUBE_2X6_1_4[1];   // 2 inches (vertical)
_wall_thick = 6.35;    // 1/4 inch wall
_tube_rad = 12.7;      // 1/2 inch radius for rounded corners

// Length calculated from kinematic parameters
_arm_main_len = is_undef(ARM_MAIN_LEN) ? 1500 : ARM_MAIN_LEN;

PART_T4_LENGTH = _arm_main_len;
PART_T4_WIDTH = _tube_w;   // 152.4mm (6") - horizontal
PART_T4_HEIGHT = _tube_h;  // 50.8mm (2") - vertical
PART_T4_WALL = _wall_thick;
PART_T4_RADIUS = _tube_rad;

// Bolt hole parameters
_bolt_spacing_x = 100;  // X spacing between bolt pairs
_bolt_spacing_y = 80;   // Y spacing between bolts (through-tube)
_tube_bolt_inset = is_undef(TUBE_BOLT_INSET) ? 50.8 : TUBE_BOLT_INSET;  // 2" from tube end
_bolt_dia = is_undef(BOLT_DIA_1_2) ? 12.7 : BOLT_DIA_1_2;

// =============================================================================
// MAIN MODULE
// =============================================================================

/**
 * Part T4: Main Arm Tube
 * Loader arm main section from pivot to elbow
 * Oriented with origin at rear end (pivot side), extends in +X
 * show_holes: if true, renders mounting holes for plate attachment
 */
module _part_t4_arm_main(show_holes=true) {
    length = PART_T4_LENGTH;
    width = PART_T4_WIDTH;
    height = PART_T4_HEIGHT;
    wall = PART_T4_WALL;
    rad = PART_T4_RADIUS;
    
    color("DarkGray")
    difference() {
        // Hollow Rounded Tube
        translate([0, 0, height])
        rotate([0, 90, 0])
        linear_extrude(height=length)
        difference() {
            // Outer profile
            hull() {
                translate([rad, rad]) circle(r=rad);
                translate([height - rad, rad]) circle(r=rad);
                translate([height - rad, width - rad]) circle(r=rad);
                translate([rad, width - rad]) circle(r=rad);
            }
            // Inner cutout
            offset(delta=-wall)
            hull() {
                translate([rad, rad]) circle(r=rad);
                translate([height - rad, rad]) circle(r=rad);
                translate([height - rad, width - rad]) circle(r=rad);
                translate([rad, width - rad]) circle(r=rad);
            }
        }
        
        if (show_holes) {
            // Rear Holes (near pivot mount)
            for (x = [_tube_bolt_inset, _tube_bolt_inset + _bolt_spacing_x])
                for (z = [height/2 - _bolt_spacing_y/2, height/2 + _bolt_spacing_y/2])
                    translate([x, width/2, z])
                    rotate([90, 0, 0])
                    cylinder(d=_bolt_dia, h=width + 10, center=true, $fn=32);
            
            // Front Holes (near elbow)
            for (x = [length - _tube_bolt_inset - _bolt_spacing_x, length - _tube_bolt_inset])
                for (z = [height/2 - _bolt_spacing_y/2, height/2 + _bolt_spacing_y/2])
                    translate([x, width/2, z])
                    rotate([90, 0, 0])
                    cylinder(d=_bolt_dia, h=width + 10, center=true, $fn=32);
        }
    }
}

// =============================================================================
// RENDER
// =============================================================================

_part_t4_arm_main(show_holes=true);

echo("=== PART T4: Main Arm Tube ===");
echo("Length:", PART_T4_LENGTH, "mm (", PART_T4_LENGTH/25.4, "in)");
echo("Width (6\" face):", PART_T4_WIDTH, "mm");
echo("Height (2\" face):", PART_T4_HEIGHT, "mm");
echo("Wall thickness:", PART_T4_WALL, "mm");
echo("Mounting holes: 16 total (8 rear, 8 front, 1/2\" dia)");
echo("Quantity needed: 2 pieces (left and right arms)");
