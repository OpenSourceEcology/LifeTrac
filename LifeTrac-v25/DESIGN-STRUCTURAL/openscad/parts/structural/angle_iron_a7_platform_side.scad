// Part A7: Platform Side Angle Iron (Left and Right)
// Material: 2"×2"×1/4" Angle Iron (50.8mm leg × 6.35mm thickness)
// Quantity: 2 pieces (1 left, 1 right - mirror)
// Location: Folding standing platform, side arms connecting deck to pivot brackets
//
// These are the structural arms that rotate about the pivot point
// They connect the deck plate to the pivot bracket plates

include <../../lifetrac_v25_params.scad>

// =============================================================================
// PART DIMENSIONS
// =============================================================================

_leg = is_undef(PLATFORM_ANGLE_LEG) ? 50.8 : PLATFORM_ANGLE_LEG;       // 2" = 50.8mm
_thick = is_undef(PLATFORM_ANGLE_THICK) ? 6.35 : PLATFORM_ANGLE_THICK; // 1/4" = 6.35mm
_arm_len = is_undef(PLATFORM_ARM_LENGTH) ? 500 : PLATFORM_ARM_LENGTH;
_bracket_width = is_undef(PLATFORM_BRACKET_WIDTH) ? 101.6 : PLATFORM_BRACKET_WIDTH;
_depth = is_undef(PLATFORM_DEPTH) ? 400 : PLATFORM_DEPTH;
_edge_margin = is_undef(PLATFORM_EDGE_MARGIN) ? 12.7 : PLATFORM_EDGE_MARGIN;
_bolt_dia = is_undef(PLATFORM_BOLT_DIA) ? 9.525 : PLATFORM_BOLT_DIA;
_clearance = is_undef(PLATFORM_BOLT_CLEARANCE) ? 0.5 : PLATFORM_BOLT_CLEARANCE;

// Calculate length like in main assembly
_deck_pivot_y = _bracket_width / 2;
_deck_far_y = -(_depth - _bracket_width/2);
_front_corner_y = (_deck_pivot_y - _edge_margin) - _leg;
_rear_corner_y = (_deck_far_y + _edge_margin) + _leg;
_gap = 12.7;
_side_start_y = _front_corner_y - _gap;
_side_end_y = _rear_corner_y + _gap;

PART_A7_LENGTH = _side_start_y - _side_end_y;  // Calculated from geometry
PART_A7_LEG = _leg;
PART_A7_THICK = _thick;

// =============================================================================
// MAIN MODULE
// =============================================================================

/**
 * Part A7: Platform Side Angle Iron
 * L-profile angle iron for platform side arms
 * show_holes: if true, renders bolt holes
 */
module _part_a7_platform_side_angle(show_holes=true) {
    length = PART_A7_LENGTH;
    leg = PART_A7_LEG;
    thick = PART_A7_THICK;
    hole_dia = _bolt_dia + _clearance;
    
    color("DimGray")
    difference() {
        // L-profile extrusion
        linear_extrude(height=length)
        polygon([
            [0, 0],
            [leg, 0],
            [leg, thick],
            [thick, thick],
            [thick, leg],
            [0, leg]
        ]);
        
        if (show_holes) {
            // Pivot bracket attachment holes (through vertical leg)
            // 2 holes near pivot end (start of extrusion)
            _lock_offset = is_undef(PLATFORM_LOCK_OFFSET) ? 130 : PLATFORM_LOCK_OFFSET;
            _side_bolt_y = is_undef(PLATFORM_SIDE_BOLT_Y_OFFSET) ? 25.4 : PLATFORM_SIDE_BOLT_Y_OFFSET;
            _bracket_len = is_undef(PLATFORM_BRACKET_LENGTH) ? 200 : PLATFORM_BRACKET_LENGTH;
            
            _pivot_bolt_offset = (_bracket_len - _bracket_width/2) * 0.30;
            _leg2_len = _bracket_len - _bracket_width/2;
            _bx_1 = _pivot_bolt_offset;
            _bx_2 = _leg2_len - _pivot_bolt_offset;
            
            _corner_y = -_lock_offset + _side_bolt_y;
            _side_angle_start_y_calc = _side_start_y;
            
            z_pos_1 = _side_angle_start_y_calc + _bx_1;
            z_pos_2 = _side_angle_start_y_calc + _bx_2;
            x_pos = leg / 2;
            
            for (z_pos = [z_pos_1, z_pos_2]) {
                translate([x_pos, thick/2, z_pos])
                rotate([90, 0, 0])
                cylinder(d=hole_dia, h=thick + 10, center=true, $fn=32);
            }
            
            // Deck attachment holes (through horizontal leg)
            // 2 holes for deck bolts
            _deck_bolt_offset = max(length * 0.08, 2 * _bolt_dia);
            for (z_pos = [_deck_bolt_offset, length - _deck_bolt_offset]) {
                translate([thick/2, leg/2, z_pos])
                rotate([0, 90, 0])
                cylinder(d=hole_dia, h=thick + 10, center=true, $fn=32);
            }
        }
    }
}

// =============================================================================
// RENDER
// =============================================================================

_part_a7_platform_side_angle(show_holes=true);

echo("=== PART A7: Platform Side Angle Iron ===");
echo("Length:", PART_A7_LENGTH, "mm (", PART_A7_LENGTH/25.4, "in)");
echo("Leg:", PART_A7_LEG, "mm (2 inches)");
echo("Thickness:", PART_A7_THICK, "mm (1/4 inch)");
echo("Holes: 4 total (2 pivot bracket, 2 deck attachment)");
echo("Quantity needed: 2 pieces (left and right, mirrored in assembly)");
