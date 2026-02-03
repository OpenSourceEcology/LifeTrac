// Part A8: Platform Transverse Angle Iron (Front and Rear)
// Material: 2"×2"×1/4" Angle Iron (50.8mm leg × 6.35mm thickness)
// Quantity: 2 pieces (1 front near pivot, 1 rear at far end)
// Location: Folding standing platform, cross-bracing under deck plate
//
// These run transversely (left-right) across the platform deck,
// spaced 1/4" from the side angle mounting brackets

include <../../lifetrac_v25_params.scad>

// =============================================================================
// PART DIMENSIONS
// =============================================================================

_leg = is_undef(PLATFORM_ANGLE_LEG) ? 50.8 : PLATFORM_ANGLE_LEG;       // 2" = 50.8mm
_thick = is_undef(PLATFORM_ANGLE_THICK) ? 6.35 : PLATFORM_ANGLE_THICK; // 1/4" = 6.35mm
_track_width = is_undef(TRACK_WIDTH) ? 900 : TRACK_WIDTH;
_sandwich = is_undef(SANDWICH_SPACING) ? 120 : SANDWICH_SPACING;
_panel_thick = is_undef(PANEL_THICKNESS) ? 12.7 : PANEL_THICKNESS;
_pivot_x = is_undef(PLATFORM_PIVOT_X) ? 300 : PLATFORM_PIVOT_X;
_side_gap = is_undef(PLATFORM_SIDE_GAP) ? 6.35 : PLATFORM_SIDE_GAP;
_bolt_dia = is_undef(PLATFORM_BOLT_DIA) ? 9.525 : PLATFORM_BOLT_DIA;
_clearance = is_undef(PLATFORM_BOLT_CLEARANCE) ? 0.5 : PLATFORM_BOLT_CLEARANCE;
_end_offset = is_undef(PLATFORM_TRANSVERSE_BOLT_END_OFFSET) ? 50.8 : PLATFORM_TRANSVERSE_BOLT_END_OFFSET;

// Calculate length: spans between side angle brackets with 1/4" clearance each side
_angle_x = _pivot_x - _side_gap;  // X position of side angles from center
_clearance_gap = 6.35;  // 1/4" clearance from L brackets

PART_A8_LENGTH = 2 * (_angle_x - _clearance_gap);
PART_A8_LEG = _leg;
PART_A8_THICK = _thick;

// =============================================================================
// MAIN MODULE
// =============================================================================

/**
 * Part A8: Platform Transverse Angle Iron
 * L-profile angle iron running left-right across platform deck
 * show_holes: if true, renders deck mounting holes
 */
module _part_a8_platform_transverse_angle(show_holes=true) {
    length = PART_A8_LENGTH;
    leg = PART_A8_LEG;
    thick = PART_A8_THICK;
    hole_dia = _bolt_dia + _clearance;
    
    color("DimGray")
    difference() {
        union() {
            // Vertical Leg (full length, face at Y=0)
            translate([-length/2, 0, -leg])
            cube([length, thick, leg]);
            
            // Horizontal Leg (face at Z=0)
            translate([-length/2, 0, -thick])
            cube([length, leg, thick]);
        }
        
        if (show_holes) {
            // Deck Mounting Holes (through horizontal leg)
            // 3 holes: near each end and center
            for (x_pos = [-length/2 + _end_offset, 0, length/2 - _end_offset]) {
                translate([x_pos, leg/2, 0])
                cylinder(d=hole_dia, h=thick*3, center=true, $fn=32);
            }
        }
    }
}

// =============================================================================
// RENDER
// =============================================================================

_part_a8_platform_transverse_angle(show_holes=true);

echo("=== PART A8: Platform Transverse Angle Iron ===");
echo("Length:", PART_A8_LENGTH, "mm (", PART_A8_LENGTH/25.4, "in)");
echo("Leg:", PART_A8_LEG, "mm (2 inches)");
echo("Thickness:", PART_A8_THICK, "mm (1/4 inch)");
echo("Holes: 3 per piece (deck mounting, 3/8\" dia)");
echo("Quantity needed: 2 pieces (front and rear)");
