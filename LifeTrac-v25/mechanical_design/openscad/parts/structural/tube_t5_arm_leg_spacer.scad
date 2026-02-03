// Part T5: Arm Leg Spacer Tube
// Material: 2"×6"×1/4" Rectangular Tubing (50.8mm × 152.4mm × 6.35mm wall)
// Quantity: 2 pieces (1 left, 1 right)
// Location: Elbow section between main arm tube and bucket pivot
//
// This piece is tapered/cut to match the arm plate profile
// Full length before cutting: ~150mm
// Positioned at elbow angle (ARM_ANGLE, typically ~150 degrees)

include <../../lifetrac_v25_params.scad>

// =============================================================================
// PART DIMENSIONS (Pre-cut stock length)
// =============================================================================

_tube_w = is_undef(TUBE_2X6_1_4) ? 152.4 : TUBE_2X6_1_4[0];  // 6 inches
_tube_h = is_undef(TUBE_2X6_1_4) ? 50.8 : TUBE_2X6_1_4[1];   // 2 inches
_wall_thick = 6.35;
_tube_rad = 12.7;

// Spacer length before tapering
PART_T5_LENGTH_STOCK = 150;  // 150mm stock length
PART_T5_WIDTH = _tube_w;
PART_T5_HEIGHT = _tube_h;
PART_T5_WALL = _wall_thick;
PART_T5_RADIUS = _tube_rad;

// Taper parameters from arm geometry
_drop_len = is_undef(ARM_DROP_LEN) ? 300 : ARM_DROP_LEN;
_drop_ext = is_undef(ARM_DROP_EXT) ? 80 : ARM_DROP_EXT;
_pivot_hole_x = is_undef(PIVOT_HOLE_X_FROM_FRONT) ? 25.4 : PIVOT_HOLE_X_FROM_FRONT;
_pivot_hole_z = is_undef(PIVOT_HOLE_Z_FROM_BOTTOM) ? 127 : PIVOT_HOLE_Z_FROM_BOTTOM;

_plate_cx = _drop_ext - _pivot_hole_x;
_boss_x = _drop_len + _plate_cx;
_boss_z = _pivot_hole_z;
_boss_r = _pivot_hole_x;
_cut_start_x = _drop_len - 100;

_bolt_dia = is_undef(BOLT_DIA_1_2) ? 12.7 : BOLT_DIA_1_2;

// =============================================================================
// MAIN MODULE
// =============================================================================

/**
 * Part T5: Arm Leg Spacer Tube (Raw Stock)
 * This shows the uncut stock piece.
 * In fabrication, this will be plasma cut to match arm plate profile.
 * show_profile: if true, shows the cut profile envelope
 */
module _part_t5_arm_leg_spacer_raw() {
    spacer_len = PART_T5_LENGTH_STOCK;
    tube_w = PART_T5_WIDTH;
    tube_h = PART_T5_HEIGHT;
    wall = PART_T5_WALL;
    rad = PART_T5_RADIUS;
    
    color("DarkGray")
    translate([0, 0, tube_h])
    rotate([0, 90, 0])
    linear_extrude(height=spacer_len)
    difference() {
        // Outer profile
        hull() {
            translate([rad, rad]) circle(r=rad);
            translate([tube_h - rad, rad]) circle(r=rad);
            translate([tube_h - rad, tube_w - rad]) circle(r=rad);
            translate([rad, tube_w - rad]) circle(r=rad);
        }
        // Inner cutout
        offset(delta=-wall)
        hull() {
            translate([rad, rad]) circle(r=rad);
            translate([tube_h - rad, rad]) circle(r=rad);
            translate([tube_h - rad, tube_w - rad]) circle(r=rad);
            translate([rad, tube_w - rad]) circle(r=rad);
        }
    }
}

/**
 * Part T5: Arm Leg Spacer Tube (Cut to Profile)
 * Shows the final tapered shape matching arm plate
 * show_holes: if true, renders mounting holes
 */
module _part_t5_arm_leg_spacer_cut(show_holes=true) {
    spacer_len = PART_T5_LENGTH_STOCK;
    tube_w = PART_T5_WIDTH;
    tube_h = PART_T5_HEIGHT;
    wall = PART_T5_WALL;
    rad = PART_T5_RADIUS;
    
    color("DarkGray")
    difference() {
        intersection() {
            // Raw tube section
            translate([0, 0, tube_h])
            rotate([0, 90, 0])
            linear_extrude(height=spacer_len)
            difference() {
                offset(r=rad) square([tube_h - rad*2, tube_w - rad*2], center=true);
                hull() {
                    translate([-tube_h/2 + rad, -tube_w/2 + rad]) circle(r=rad);
                    translate([tube_h/2 - rad, -tube_w/2 + rad]) circle(r=rad);
                    translate([tube_h/2 - rad, tube_w/2 - rad]) circle(r=rad);
                    translate([-tube_h/2 + rad, tube_w/2 - rad]) circle(r=rad);
                }
            }
            
            // Profile envelope (tapers toward pivot)
            union() {
                // Full width portion
                cube([_cut_start_x, 100 + tube_w, tube_h]);
                // Taper portion
                hull() {
                    translate([_cut_start_x - 1, -50, 0])
                    cube([1, 100 + tube_w, tube_h]);
                    translate([_boss_x, -50, _boss_z])
                    rotate([-90, 0, 0])
                    cylinder(r=_boss_r, h=100 + tube_w);
                }
            }
        }
        
        // Mounting holes (4 bolts)
        if (show_holes) {
            for (xb = [spacer_len/3, spacer_len*2/3])
                for (zb = [tube_h*0.3, tube_h*0.7])
                    translate([xb, tube_w/2, zb])
                    rotate([90, 0, 0])
                    cylinder(d=_bolt_dia, h=tube_w + 50, center=true, $fn=32);
        }
    }
}

// =============================================================================
// RENDER
// =============================================================================

// Show the raw stock piece for ordering/cutting
_part_t5_arm_leg_spacer_raw();

// Uncomment to see the cut profile:
// translate([0, 200, 0]) _part_t5_arm_leg_spacer_cut(show_holes=true);

echo("=== PART T5: Arm Leg Spacer Tube ===");
echo("Stock Length:", PART_T5_LENGTH_STOCK, "mm (", PART_T5_LENGTH_STOCK/25.4, "in)");
echo("Width (6\" face):", PART_T5_WIDTH, "mm");
echo("Height (2\" face):", PART_T5_HEIGHT, "mm");
echo("Wall thickness:", PART_T5_WALL, "mm");
echo("Note: Cut to profile matching arm_plate.scad taper");
echo("Mounting holes: 8 total (1/2\" dia, through-tube)");
echo("Quantity needed: 2 pieces (left and right arms)");
