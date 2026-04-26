// Part A6: Bottom Stiffener Plate - Horizontal Angle Iron (Full Assembly Module)
// Material: 2"×2"×1/4" Angle Iron (50.8mm × 6.35mm)
// Quantity: 24 total pieces (6 locations × 3 segments + motor plate segments)
// Location: Bottom stiffener plate, all four wall panel faces
//
// This is a HORIZONTAL angle iron that runs along the Y axis (forward/backward).
// It sits on top of the bottom plate with:
//   - Horizontal leg flat ON the plate surface (extends toward center)
//   - Vertical leg going UP against the wall panel
//
// NOTE: Due to wheel clearance requirements, each run is split into 3 segments
// with 8" (203.2mm) gaps centered on each wheel axis.

include <../../lifetrac_v25_params.scad>

// =============================================================================
// SEGMENT LENGTH CALCULATIONS
// =============================================================================

// These values match lifetrac_v25.scad calculations
// The actual lengths are calculated from wheel positions in the main assembly

// Default segment lengths (actual values come from params at render time)
// Segment 1: Rear section (from plate start to rear wheel gap)
// Segment 2: Middle section (between wheel gaps)
// Segment 3: Front section (from front wheel gap to plate end)

PART_A6_LEG = 50.8;       // 2" leg
PART_A6_THICK = 6.35;     // 1/4" thickness
PART_A6_BOLT_DIA = 9.525; // 3/8" bolts
PART_A6_HOLE_OFFSET = PART_A6_LEG * 0.6;

// =============================================================================
// HOLE PATTERN FUNCTIONS
// =============================================================================

// Horizontal leg holes (bolts go through in Z direction - into plate)
function get_horizontal_holes_a(length) = 
    let(
        start_offset = 50,
        end_offset = 50,
        spacing = 150,
        count = max(1, floor((length - start_offset - end_offset) / spacing) + 1)
    )
    [for (i = [0:count-1]) start_offset + i * spacing];

// Vertical leg holes (bolts go through in X direction - into wall)
function get_horizontal_holes_b(length) = 
    let(
        start_offset = 75,
        end_offset = 50,
        spacing = 150,
        count = max(1, floor((length - start_offset - end_offset) / spacing) + 1)
    )
    [for (i = [0:count-1]) start_offset + i * spacing];

// =============================================================================
// SINGLE SEGMENT MODULE
// =============================================================================

/**
 * Creates a single horizontal angle iron segment
 * 
 * Orientation:
 *   - Runs along Y axis (length)
 *   - Origin at corner [0, 0, 0]
 *   - Horizontal leg extends in +X direction (toward center, flat on plate)
 *   - Vertical leg extends in +Z direction (up against wall)
 *
 * @param length Length of this segment
 * @param show_holes If true, drill bolt holes
 */
module _bottom_angle_segment(length, show_holes=true) {
    leg = PART_A6_LEG;
    thick = PART_A6_THICK;
    bolt_dia = PART_A6_BOLT_DIA;
    hole_offset = PART_A6_HOLE_OFFSET;
    
    if (length > 0) {
        color("DarkGray")
        difference() {
            union() {
                // Horizontal leg - flat on plate, extends in +X direction
                cube([leg, length, thick]);
                // Vertical leg - extends upward in +Z direction
                cube([thick, length, leg]);
            }
            
            if (show_holes) {
                // Holes on horizontal leg (bolts go through in Z direction)
                for (y = get_horizontal_holes_a(length)) {
                    translate([hole_offset, y, thick/2])
                    cylinder(d=bolt_dia, h=thick+2, center=true, $fn=16);
                }
                
                // Holes on vertical leg (bolts go through in X direction)
                for (y = get_horizontal_holes_b(length)) {
                    translate([thick/2, y, hole_offset])
                    rotate([0, 90, 0])
                    cylinder(d=bolt_dia, h=thick+2, center=true, $fn=16);
                }
            }
        }
    }
}

// =============================================================================
// INDIVIDUAL SEGMENT PARTS (for fabrication)
// =============================================================================

// Segment 1 (Rear) - from plate start to rear wheel gap
module _part_a6_bottom_segment_1(show_holes=true) {
    // Use the calculated segment length from params
    _seg1_len = is_undef(ANGLE_SEGMENT_1_LENGTH) ? 200 : ANGLE_SEGMENT_1_LENGTH;
    _bottom_angle_segment(_seg1_len, show_holes);
}

// Segment 2 (Middle) - between wheel gaps
module _part_a6_bottom_segment_2(show_holes=true) {
    _seg2_len = is_undef(ANGLE_SEGMENT_2_LENGTH) ? 300 : ANGLE_SEGMENT_2_LENGTH;
    _bottom_angle_segment(_seg2_len, show_holes);
}

// Segment 3 (Front) - from front wheel gap to plate end
module _part_a6_bottom_segment_3(show_holes=true) {
    _seg3_len = is_undef(ANGLE_SEGMENT_3_LENGTH) ? 250 : ANGLE_SEGMENT_3_LENGTH;
    _bottom_angle_segment(_seg3_len, show_holes);
}

// =============================================================================
// COMPLETE SPLIT ASSEMBLY (for visualization)
// =============================================================================

/**
 * Creates all 3 segments in their correct positions
 * Matches the split_horizontal_angle_iron module in lifetrac_v25.scad
 */
module _part_a6_split_horizontal_angle_iron(show_holes=true) {
    _gap = is_undef(ANGLE_IRON_GAP) ? 203.2 : ANGLE_IRON_GAP;  // 8" gap
    
    _seg1_len = is_undef(ANGLE_SEGMENT_1_LENGTH) ? 200 : ANGLE_SEGMENT_1_LENGTH;
    _seg2_len = is_undef(ANGLE_SEGMENT_2_LENGTH) ? 300 : ANGLE_SEGMENT_2_LENGTH;
    _seg3_len = is_undef(ANGLE_SEGMENT_3_LENGTH) ? 250 : ANGLE_SEGMENT_3_LENGTH;
    
    _seg1_start = is_undef(ANGLE_SEGMENT_1_START) ? 0 : ANGLE_SEGMENT_1_START;
    _seg2_start = is_undef(ANGLE_SEGMENT_2_START) ? _seg1_len + _gap : ANGLE_SEGMENT_2_START;
    _seg3_start = is_undef(ANGLE_SEGMENT_3_START) ? _seg2_start + _seg2_len + _gap : ANGLE_SEGMENT_3_START;
    
    // Segment 1 (rear)
    translate([0, _seg1_start, 0])
    _bottom_angle_segment(_seg1_len, show_holes);
    
    // Segment 2 (middle)
    translate([0, _seg2_start, 0])
    _bottom_angle_segment(_seg2_len, show_holes);
    
    // Segment 3 (front)
    translate([0, _seg3_start, 0])
    _bottom_angle_segment(_seg3_len, show_holes);
}

// =============================================================================
// RENDER
// =============================================================================

// Show segment 2 (middle, typically the longest) for preview
_part_a6_bottom_segment_2(show_holes=true);

echo("=== PART A6: Bottom Stiffener Horizontal Angle (Split) ===");
echo("Total unique segments: 3 per run");
echo("Total runs: 8 (4 walls × outer + inner sides)");
echo("Total segment pieces: 24");
echo("Leg size:", PART_A6_LEG, "mm (2\")");
echo("Bolt diameter:", PART_A6_BOLT_DIA, "mm (3/8\")");
