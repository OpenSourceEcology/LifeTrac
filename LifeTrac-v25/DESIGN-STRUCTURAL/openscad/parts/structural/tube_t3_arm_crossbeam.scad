// Part T3: Arm Crossbeam Tube
// Material: 2"×6"×1/4" Rectangular Tubing (50.8mm × 152.4mm × 6.35mm wall)
// Quantity: 1 piece
// Location: Connects left and right loader arms at elbow position
//
// Length = ARM_SPACING (same as TRACK_WIDTH)
// Mounting holes for angle irons at each end, top and bottom

include <../../lifetrac_v25_params.scad>

// =============================================================================
// PART DIMENSIONS
// =============================================================================

_tube_width = 152.4;  // 6 inches (along arm Y direction)
_tube_height = 50.8;  // 2 inches (Z direction)
_wall_thick = 6.35;   // 1/4 inch wall
_corner_radius = 12.7; // 1/2 inch radius

// Length calculated from params
// Crossbeam extends through inner arm CNC plates
_arm_spacing = is_undef(ARM_SPACING) ? 900 : ARM_SPACING;
_tube_w = is_undef(TUBE_2X6_1_4) ? 50.8 : TUBE_2X6_1_4[0];  // Arm tube width (2")

// Ends flush with inner face of inner arm CNC plates
PART_T3_LENGTH = _arm_spacing - _tube_w;
PART_T3_WIDTH = _tube_width;   // 152.4mm (6")
PART_T3_HEIGHT = _tube_height; // 50.8mm (2")
PART_T3_WALL = _wall_thick;
PART_T3_RADIUS = _corner_radius;

// Mounting hole parameters (for angle iron mounts)
_arm_plate_thick = is_undef(ARM_PLATE_THICKNESS) ? 6.35 : ARM_PLATE_THICKNESS;
_angle_hole_offset = 25.4;  // 1 inch from center of angle leg
_hole_x_offset = _arm_spacing/2 - (25.4 + _arm_plate_thick + _angle_hole_offset);
_hole_y_spacing = 25.4;  // 1 inch from beam center (2 inch total spacing)
_bolt_dia = is_undef(BOLT_DIA_1_2) ? 12.7 : BOLT_DIA_1_2;

// =============================================================================
// MAIN MODULE
// =============================================================================

/**
 * Part T3: Arm Crossbeam Tube
 * Cross tube connecting loader arms at elbow position
 * show_holes: if true, renders mounting holes for visualization
 */
module _part_t3_arm_crossbeam(show_holes=true) {
    length = PART_T3_LENGTH;
    width = PART_T3_WIDTH;
    height = PART_T3_HEIGHT;
    wall = PART_T3_WALL;
    rad = PART_T3_RADIUS;
    
    color("DarkGray")
    difference() {
        // Main tube - centered on X axis
        rotate([0, 90, 0])
        linear_extrude(height=length, center=true)
        hull() {
            translate([-height/2 + rad, -width/2 + rad]) circle(r=rad);
            translate([height/2 - rad, -width/2 + rad]) circle(r=rad);
            translate([height/2 - rad, width/2 - rad]) circle(r=rad);
            translate([-height/2 + rad, width/2 - rad]) circle(r=rad);
        }
        
        // Hollow interior
        rotate([0, 90, 0])
        linear_extrude(height=length - 2*wall, center=true)
        hull() {
            translate([-height/2 + rad + wall, -width/2 + rad + wall]) circle(r=rad);
            translate([height/2 - rad - wall, -width/2 + rad + wall]) circle(r=rad);
            translate([height/2 - rad - wall, width/2 - rad - wall]) circle(r=rad);
            translate([-height/2 + rad + wall, width/2 - rad - wall]) circle(r=rad);
        }
        
        // Mounting Holes for Angle Irons
        // Located at ends of beam, top and bottom
        if (show_holes) {
            for (x_pos = [-_hole_x_offset, _hole_x_offset]) {
                for (y_pos = [-_hole_y_spacing, _hole_y_spacing]) {
                    translate([x_pos, y_pos, 0])
                    cylinder(d=_bolt_dia, h=100, center=true, $fn=32);
                }
            }
        }
    }
}

// =============================================================================
// RENDER
// =============================================================================

_part_t3_arm_crossbeam(show_holes=true);

echo("=== PART T3: Arm Crossbeam Tube ===");
echo("Length:", PART_T3_LENGTH, "mm (", PART_T3_LENGTH/25.4, "in)");
echo("Width (6\" face):", PART_T3_WIDTH, "mm");
echo("Height (2\" face):", PART_T3_HEIGHT, "mm");
echo("Wall thickness:", PART_T3_WALL, "mm");
echo("Mounting holes: 8 total (4 per end, 1/2\" dia)");
echo("Quantity needed: 1 piece");
