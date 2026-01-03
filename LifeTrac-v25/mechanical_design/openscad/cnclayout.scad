// cnclayout.scad
// CNC layout for all plate steel parts
// Generates 2D projection for plasma cutting
// Part of LifeTrac v25 OpenSCAD design
// 
// Layout arrangement: left-to-right
// No text labels (for direct CNC machine use)
// Parametric spacing to prevent overlaps

// Import individual part modules
use <parts/side_panel.scad>
use <parts/wheel_mount.scad>
use <parts/cylinder_lug.scad>
use <parts/platform_deck.scad>
use <parts/platform_pivot_bracket.scad>
use <parts/arm_plate.scad>

// Import main assembly to access bucket plate modules
use <lifetrac_v25.scad>

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

// Bucket Back Plate (rotated 90 degrees around X to lay flat)
// Dimensions: Width = BUCKET_WIDTH, Height = BUCKET_HEIGHT
bucket_back_width = BUCKET_WIDTH;
bucket_back_height = BUCKET_HEIGHT;

// Bucket Bottom Plate (translated to Z=0)
// Dimensions: Width = BUCKET_WIDTH, Height = BUCKET_DEPTH
bucket_bottom_width = BUCKET_WIDTH;
bucket_bottom_height = BUCKET_DEPTH;

// Bucket Side Plate (rotated to lay flat)
// Dimensions: Width = BUCKET_HEIGHT, Height = BUCKET_DEPTH
bucket_side_width_rotated = BUCKET_HEIGHT; 
bucket_side_height_rotated = BUCKET_DEPTH;

// Wheel mount dimensions
wheel_mount_size = 250;  // 250mm x 250mm

// Cylinder lug dimensions
cylinder_lug_width = 100;
cylinder_lug_height = 150;

// Platform deck dimensions
platform_deck_width = PLATFORM_WIDTH;     // ~850mm
platform_deck_height = PLATFORM_DEPTH;    // 400mm

// Platform pivot bracket dimensions (2 required)
pivot_bracket_width = PLATFORM_BRACKET_LENGTH;   // 150mm
pivot_bracket_height = PLATFORM_BRACKET_WIDTH;   // 100mm

// Arm Plate Dimensions
arm_plate_width = 1600;
arm_plate_height = 600;

// =============================================================================
// CALCULATE X POSITIONS (parametric, left-to-right)
// =============================================================================

// X positions calculated sequentially
x_pos_0 = START_X;
x_pos_1 = x_pos_0 + side_panel_width_rotated + SPACING;
x_pos_2 = x_pos_1 + side_panel_width_rotated + SPACING;
x_pos_3 = x_pos_2 + side_panel_width_rotated + SPACING;
x_pos_4 = x_pos_3 + side_panel_width_rotated + SPACING;

// Bucket Parts
x_pos_5 = x_pos_4 + bucket_back_width + SPACING;          // Bucket Back
x_pos_6 = x_pos_5 + bucket_bottom_width + SPACING;        // Bucket Bottom
x_pos_7 = x_pos_6 + bucket_side_width_rotated + SPACING;  // Bucket Side Left
x_pos_8 = x_pos_7 + bucket_side_width_rotated + SPACING;  // Bucket Side Right

// Platform Parts
x_pos_9 = x_pos_8 + platform_deck_width + SPACING;         // Platform Deck
x_pos_10 = x_pos_9 + pivot_bracket_width + SPACING;        // Pivot Bracket 1
x_pos_11 = x_pos_10 + pivot_bracket_width + SPACING;       // Pivot Bracket 2

// Wheel Mounts
x_pos_12 = x_pos_11 + wheel_mount_size + SPACING;
x_pos_13 = x_pos_12 + wheel_mount_size + SPACING;
x_pos_14 = x_pos_13 + wheel_mount_size + SPACING;
x_pos_15 = x_pos_14 + wheel_mount_size + SPACING;

// Cylinder Lugs
x_pos_16 = x_pos_15 + cylinder_lug_width + SPACING;
x_pos_17 = x_pos_16 + cylinder_lug_width + SPACING;
x_pos_18 = x_pos_17 + cylinder_lug_width + SPACING;
x_pos_19 = x_pos_18 + cylinder_lug_width + SPACING;
x_pos_20 = x_pos_19 + cylinder_lug_width + SPACING;
x_pos_21 = x_pos_20 + cylinder_lug_width + SPACING;

// Arm Plates
x_pos_22 = x_pos_21 + cylinder_lug_width + SPACING;
x_pos_23 = x_pos_22 + arm_plate_width + SPACING;
x_pos_24 = x_pos_23 + arm_plate_width + SPACING;
x_pos_25 = x_pos_24 + arm_plate_width + SPACING;

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
layout_part(x_pos_2, START_Y, 90)
side_panel(is_inner=true);

// Part 3: Side panel outer - Right side (rotated 90°)
layout_part(x_pos_3, START_Y, 90)
side_panel(is_inner=false);

// Part 4: Bucket Back Plate
// Needs rotation: It's in XZ plane. Rotate 90 around X to put on XY.
// Translate to Z=0 (from -BUCKET_HEIGHT)
layout_part(x_pos_4, START_Y, 0)
translate([BUCKET_WIDTH/2, 0, 0]) // Center X
rotate([90, 0, 0])
translate([0, 0, BUCKET_HEIGHT]) // Move up to Z=0
bucket_back_plate();

// Part 5: Bucket Bottom Plate
// Needs translation: It's in XY plane but at Z=-BUCKET_HEIGHT.
layout_part(x_pos_5, START_Y, 0)
translate([BUCKET_WIDTH/2, 0, 0]) // Center X
translate([0, 0, BUCKET_HEIGHT]) // Move up to Z=0
bucket_bottom_plate();

// Part 6: Bucket Side Plate - Left
// Needs rotation: It's in YZ plane. Rotate 90 around Y.
layout_part(x_pos_6, START_Y, 90)
rotate([0, 90, 0])
translate([BUCKET_WIDTH/2 + PLATE_1_4_INCH, 0, BUCKET_HEIGHT]) // Adjust position
bucket_side_plate(is_left=true);

// Part 7: Bucket Side Plate - Right
// Needs rotation: It's in YZ plane. Rotate 90 around Y.
layout_part(x_pos_7, START_Y, 90)
rotate([0, 90, 0])
translate([-BUCKET_WIDTH/2, 0, BUCKET_HEIGHT]) // Adjust position
bucket_side_plate(is_left=false);

// Part 8: Platform Deck
layout_part(x_pos_9, START_Y, 0)
platform_deck();

// Part 9: Platform Pivot Bracket - Left
layout_part(x_pos_10, START_Y, 0)
platform_pivot_bracket();

// Part 10: Platform Pivot Bracket - Right
layout_part(x_pos_11, START_Y, 0)
platform_pivot_bracket();

// Part 11-14: Wheel Mounts
layout_part(x_pos_12, START_Y, 0) wheel_mount();
layout_part(x_pos_13, START_Y, 0) wheel_mount();
layout_part(x_pos_14, START_Y, 0) wheel_mount();
layout_part(x_pos_15, START_Y, 0) wheel_mount();

// Part 15-20: Cylinder Lugs
layout_part(x_pos_16, START_Y, 0) cylinder_lug();
layout_part(x_pos_17, START_Y, 0) cylinder_lug();
layout_part(x_pos_18, START_Y, 0) cylinder_lug();
layout_part(x_pos_19, START_Y, 0) cylinder_lug();
layout_part(x_pos_20, START_Y, 0) cylinder_lug();
layout_part(x_pos_21, START_Y, 0) cylinder_lug();

// Part 22: Arm Plate Inner (Left)
layout_part(x_pos_22, START_Y, 0)
rotate([90, 0, 0]) // Rotate to flat (it's in XZ plane in module)
arm_plate(is_inner_plate=true);

// Part 23: Arm Plate Inner (Right)
layout_part(x_pos_23, START_Y, 0)
rotate([90, 0, 0])
arm_plate(is_inner_plate=true);

// Part 24: Arm Plate Outer (Left)
layout_part(x_pos_24, START_Y, 0)
rotate([90, 0, 0])
arm_plate(is_inner_plate=false);

// Part 25: Arm Plate Outer (Right)
layout_part(x_pos_25, START_Y, 0)
rotate([90, 0, 0])
arm_plate(is_inner_plate=false);
