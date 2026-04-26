// Part T1: Front Cross Frame Tube
// Material: 2"×6"×1/4" Rectangular Tubing (50.8mm × 152.4mm × 6.35mm wall)
// Quantity: 1 piece
// Location: Front frame cross tube, behind front wheels
//
// This tube passes through all four wall panels, extending 1/2" past each outer panel.
// CNC-friendly corner radius on cutout profile.
// Bolt holes for angle iron mounts on front and rear faces.

include <../../lifetrac_v25_params.scad>

// =============================================================================
// PART DIMENSIONS
// =============================================================================

// Frame tube dimensions
_tube_width = 152.4;   // 6" (in Y direction when installed)
_tube_height = 50.8;   // 2" (in Z direction when installed)  
_wall_thick = 6.35;    // 1/4" wall thickness
_corner_radius = 12.7; // 1/2" corner radius on tube profile

// Length calculation (matches FRAME_TUBE_LENGTH in main assembly)
_extension = 12.7;     // 1/2" extension past outer panels
_track_width = is_undef(TRACK_WIDTH) ? 900 : TRACK_WIDTH;
_sandwich_spacing = is_undef(SANDWICH_SPACING) ? 120 : SANDWICH_SPACING;
_panel_thick = is_undef(PANEL_THICKNESS) ? 12.7 : PANEL_THICKNESS;

PART_T1_LENGTH = _track_width + _sandwich_spacing + 2*_panel_thick + 2*_extension;
PART_T1_WIDTH = _tube_width;
PART_T1_HEIGHT = _tube_height;
PART_T1_WALL = _wall_thick;
PART_T1_RADIUS = _corner_radius;

// =============================================================================
// BOLT HOLE PARAMETERS
// =============================================================================

_bolt_dia = is_undef(BOLT_DIA_1_2) ? 12.7 : BOLT_DIA_1_2;  // 1/2" bolts
_angle_leg = 50.8;        // 2" angle leg
_angle_thick = 6.35;      // 1/4" angle thickness
_hole_inset = 25.4;       // 1" from panel face (where angle hole is)

// Angle iron hole positions (from angle bottom)
// Angle is 5.75" tall, holes are 2" apart centered = at 1.875" and 3.875" from bottom
_angle_height = 146.05;   // 5.75"
_angle_hole_spacing = 50.8;  // 2" spacing on tube leg
_angle_trim = 3.175;      // 1/8" trim from tube edge

// In part coords, Y becomes world Z after rotate([90,0,0])
// Angle holes at 47.625mm and 98.425mm from angle bottom
// Angle bottom is at 3.175mm above tube bottom (in world Z = part Y)
_hole_y_1 = _angle_trim + (_angle_height/2 - _angle_hole_spacing/2);  // ~50.8mm
_hole_y_2 = _angle_trim + (_angle_height/2 + _angle_hole_spacing/2);  // ~101.6mm

// X positions for panel faces (tube centered at X=0)
// Left outer panel inner face
_left_outer_x = -(_track_width/2 + _sandwich_spacing/2);
// Left inner panel inner face  
_left_inner_x = -(_track_width/2 - _sandwich_spacing/2) + _panel_thick;
// Right inner panel inner face
_right_inner_x = (_track_width/2 - _sandwich_spacing/2) - _panel_thick;
// Right outer panel inner face
_right_outer_x = (_track_width/2 + _sandwich_spacing/2);

// Motor plate positions (6" inboard from inner wall plates)
_motor_plate_inboard = 152.4;  // 6"
_motor_plate_thick = 6.35;     // 1/4" motor plate thickness
// Motor plate X is at: (TRACK_WIDTH/2 - SANDWICH_SPACING/2) - MOTOR_PLATE_INBOARD
// Angle is offset by plate_thickness/2 from motor plate centerline
_left_motor_x = -(_track_width/2 - _sandwich_spacing/2) + _motor_plate_inboard + _motor_plate_thick/2;
_right_motor_x = (_track_width/2 - _sandwich_spacing/2) - _motor_plate_inboard - _motor_plate_thick/2;

// Hole X positions (1" inset from panel/plate face toward center)
// For left side angles, hole is at panel_x + angle_thick + hole_inset
// For right side angles (mirrored), hole is at panel_x - angle_thick - hole_inset
// Motor plates: angle extends toward center, so left motor gets + inset, right gets - inset
PART_T1_HOLE_X = [
    _left_outer_x + _angle_thick + _hole_inset,
    _left_inner_x + _hole_inset,
    _left_motor_x + _hole_inset,           // Left motor plate
    _right_motor_x - _hole_inset,          // Right motor plate
    _right_inner_x - _hole_inset,
    _right_outer_x - _angle_thick - _hole_inset
];

// Y positions for holes (become Z height positions after rotation)
PART_T1_HOLE_Y = [_hole_y_1, _hole_y_2];

// =============================================================================
// MAIN MODULE
// =============================================================================

/**
 * Part T1: Front Cross Frame Tube
 * 
 * Orientation:
 *   - Length runs along X axis (side to side)
 *   - Width (6") is in Y direction 
 *   - Height (2") is in Z direction
 *   - Origin at center-bottom of tube at X=0
 *
 * Bolt holes: 16 total (4 positions × 2 faces × 2 holes per face)
 */
module _part_t1_front_frame_tube(show_cutaway=false, show_holes=true) {
    length = PART_T1_LENGTH;
    width = PART_T1_WIDTH;
    height = PART_T1_HEIGHT;
    wall = PART_T1_WALL;
    rad = PART_T1_RADIUS;
    
    color("SteelBlue")
    difference() {
        translate([-length/2, 0, 0])
        difference() {
            // Outer profile with rounded corners
            rotate([0, 90, 0])
            rotate([0, 0, 90])
            linear_extrude(height=length)
            offset(r=rad)
            offset(r=-rad)
            square([width, height]);
            
            // Inner cutout (hollow tube)
            translate([wall, wall, wall])
            rotate([0, 90, 0])
            rotate([0, 0, 90])
            linear_extrude(height=length - 2*wall)
            offset(r=rad - wall/2)
            offset(r=-(rad - wall/2))
            square([width - 2*wall, height - 2*wall]);
            
            // Optional cutaway for visualization
            if (show_cutaway) {
                translate([length/2, -1, -1])
                cube([length, width + 2, height + 2]);
            }
        }
        
        // Bolt holes for angle iron mounts
        // After rotate([90,0,0]) in assembly: part Y→world Z, part Z→world -Y
        // Holes go through Z faces (front/rear after rotation), at Y positions (height after rotation)
        if (show_holes) {
            // Holes in top face (part Z = height, becomes rear Y face after rotation)
            for (x_pos = PART_T1_HOLE_X) {
                for (y_pos = PART_T1_HOLE_Y) {
                    translate([x_pos, y_pos, height - wall - 1])
                    cylinder(d=_bolt_dia, h=wall + 2, $fn=32);
                }
            }
            
            // Holes in bottom face (part Z = 0, becomes front Y face after rotation)
            for (x_pos = PART_T1_HOLE_X) {
                for (y_pos = PART_T1_HOLE_Y) {
                    translate([x_pos, y_pos, -1])
                    cylinder(d=_bolt_dia, h=wall + 2, $fn=32);
                }
            }
        }
    }
}

// =============================================================================
// RENDER
// =============================================================================

_part_t1_front_frame_tube(show_cutaway=false, show_holes=true);

echo("=== PART T1: Front Cross Frame Tube ===");
echo("Length:", PART_T1_LENGTH, "mm (", PART_T1_LENGTH/25.4, "inches)");
echo("Width:", PART_T1_WIDTH, "mm (6\")");
echo("Height:", PART_T1_HEIGHT, "mm (2\")");
echo("Wall thickness:", PART_T1_WALL, "mm (1/4\")");
echo("Bolt holes: 24 (6 positions × 2 faces × 2 per face)");
echo("Quantity needed: 1 piece");
