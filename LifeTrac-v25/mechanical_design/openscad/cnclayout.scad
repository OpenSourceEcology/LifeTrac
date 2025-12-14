// cnclayout.scad
// CNC layout for all plate steel parts
// Generates 2D projection for plasma cutting
// Part of LifeTrac v25 OpenSCAD design
// Following CEB-Press pattern with individual part files
// 
// Layout arrangement: left-to-right
// No text labels (for direct CNC machine use)
// Parametric spacing to prevent overlaps

// Import individual part modules
use <parts/side_panel.scad>
use <parts/rear_crossmember.scad>
use <parts/wheel_mount.scad>
use <parts/cylinder_lug.scad>
use <parts/standing_deck.scad>
use <parts/bucket_bottom.scad>
use <parts/bucket_side.scad>

// Import parameters
include <lifetrac_v25_params.scad>

// Layout spacing - approximately 1 inch between parts
SPACING = 25.4; // mm between parts (~1 inch)
START_X = 10;
START_Y = 10;

// Helper module to layout a part without labels (for CNC)
module layout_part(x, y, angle=0) {
    translate([x, y, 0])
    rotate([0, 0, angle])
    projection(cut=true)
    children();
}

// =============================================================================
// PART DIMENSIONS (for parametric layout)
// =============================================================================
// Define bounding box dimensions for each part type
// When rotated 90°, width/height swap

// Side panels (rotated 90 degrees in layout)
side_panel_width_rotated = MACHINE_HEIGHT;  // ~1000mm (becomes width when rotated)
side_panel_height_rotated = WHEEL_BASE;      // ~1400mm (becomes height when rotated)

// Rear crossmember dimensions
crossmember_width = TRACK_WIDTH + SANDWICH_SPACING + PANEL_THICKNESS * 2;  // ~1045mm
crossmember_height = MACHINE_HEIGHT * 0.55;  // ~550mm

// Wheel mount dimensions
wheel_mount_size = 250;  // 250mm x 250mm

// Cylinder lug dimensions
cylinder_lug_width = 100;
cylinder_lug_height = 150;

// Bucket bottom dimensions
bucket_bottom_width = BUCKET_WIDTH;   // 1100mm
bucket_bottom_height = BUCKET_DEPTH;  // 600mm

// Standing deck dimensions
standing_deck_width = DECK_WIDTH;   // 700mm
standing_deck_height = DECK_DEPTH;  // 400mm

// Bucket side dimensions (rotated 90 degrees in layout)
bucket_side_width_rotated = BUCKET_HEIGHT; // 450mm (becomes width when rotated)
bucket_side_height_rotated = BUCKET_DEPTH;  // 600mm (becomes height when rotated)

// =============================================================================
// CALCULATE X POSITIONS (parametric, left-to-right)
// =============================================================================
// Note: OpenSCAD requires all variables to be defined at once (no reassignment)
// This sequential calculation ensures proper spacing and no overlaps

// X positions calculated sequentially
x_pos_0 = START_X;
x_pos_1 = x_pos_0 + side_panel_width_rotated + SPACING;
x_pos_2 = x_pos_1 + side_panel_width_rotated + SPACING;
x_pos_3 = x_pos_2 + side_panel_width_rotated + SPACING;
x_pos_4 = x_pos_3 + side_panel_width_rotated + SPACING;
x_pos_5 = x_pos_4 + crossmember_width + SPACING;
x_pos_6 = x_pos_5 + bucket_bottom_width + SPACING;
x_pos_7 = x_pos_6 + standing_deck_width + SPACING;
x_pos_8 = x_pos_7 + bucket_side_width_rotated + SPACING;
x_pos_9 = x_pos_8 + bucket_side_width_rotated + SPACING;
x_pos_10 = x_pos_9 + wheel_mount_size + SPACING;
x_pos_11 = x_pos_10 + wheel_mount_size + SPACING;
x_pos_12 = x_pos_11 + wheel_mount_size + SPACING;
x_pos_13 = x_pos_12 + wheel_mount_size + SPACING;
x_pos_14 = x_pos_13 + cylinder_lug_width + SPACING;
x_pos_15 = x_pos_14 + cylinder_lug_width + SPACING;
x_pos_16 = x_pos_15 + cylinder_lug_width + SPACING;
x_pos_17 = x_pos_16 + cylinder_lug_width + SPACING;
x_pos_18 = x_pos_17 + cylinder_lug_width + SPACING;

// Calculate total layout width (last part position + last part width)
total_layout_width = x_pos_18 + cylinder_lug_width;

// =============================================================================
// LAYOUT ALL PARTS (LEFT TO RIGHT)
// =============================================================================

// Part 0: Side panel outer - Left side (rotated 90°)
layout_part(x_pos_0, START_Y, 90)
side_panel(is_inner=false);

// Part 1: Side panel inner - Left side (rotated 90°)
layout_part(x_pos_1, START_Y, 90)
side_panel(is_inner=true);

// Part 2: Side panel inner - Right side (rotated 90°)
// Note: Four panels total create a sandwich design with arms between inner pair
layout_part(x_pos_2, START_Y, 90)
side_panel(is_inner=true);

// Part 3: Side panel outer - Right side (rotated 90°)
layout_part(x_pos_3, START_Y, 90)
side_panel(is_inner=false);

// Part 4: Rear crossmember
layout_part(x_pos_4, START_Y, 0)
rear_crossmember();

// Part 5: Bucket bottom
layout_part(x_pos_5, START_Y, 0)
bucket_bottom();

// Part 6: Standing deck
layout_part(x_pos_6, START_Y, 0)
standing_deck();

// Part 7: Bucket side - Left (rotated 90°)
layout_part(x_pos_7, START_Y, 90)
bucket_side();

// Part 8: Bucket side - Right (rotated 90°)
layout_part(x_pos_8, START_Y, 90)
bucket_side();

// Part 9: Wheel mount 1
layout_part(x_pos_9, START_Y, 0)
wheel_mount();

// Part 10: Wheel mount 2
layout_part(x_pos_10, START_Y, 0)
wheel_mount();

// Part 11: Wheel mount 3
layout_part(x_pos_11, START_Y, 0)
wheel_mount();

// Part 12: Wheel mount 4
layout_part(x_pos_12, START_Y, 0)
wheel_mount();

// Part 13: Cylinder lug 1
layout_part(x_pos_13, START_Y, 0)
cylinder_lug();

// Part 14: Cylinder lug 2
layout_part(x_pos_14, START_Y, 0)
cylinder_lug();

// Part 15: Cylinder lug 3
layout_part(x_pos_15, START_Y, 0)
cylinder_lug();

// Part 16: Cylinder lug 4
layout_part(x_pos_16, START_Y, 0)
cylinder_lug();

// Part 17: Cylinder lug 5
layout_part(x_pos_17, START_Y, 0)
cylinder_lug();

// Part 18: Cylinder lug 6
layout_part(x_pos_18, START_Y, 0)
cylinder_lug();
