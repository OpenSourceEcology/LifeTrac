// cnclayout.scad
// CNC layout for all plate steel parts
// Generates 2D projection for plasma cutting
// Part of LifeTrac v25 OpenSCAD design
// Following CEB-Press pattern with individual part files

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

// Layout spacing
SPACING = 20; // mm between parts
START_X = 10;
START_Y = 10;

// Helper module to layout a part with label
module layout_part(part_name, x, y, angle=0) {
    translate([x, y, 0])
    rotate([0, 0, angle])
    projection(cut=true)
    children();
}

// =============================================================================
// LAYOUT ALL PARTS
// =============================================================================

// Row 0: Four triangular side panels (largest parts)
// Left outer panel
layout_part("A1-L-Outer", START_X, START_Y, 90)
side_panel(is_inner=false);

// Left inner panel
layout_part("A1-L-Inner", START_X + 1000 + SPACING, START_Y, 90)
side_panel(is_inner=true);

// Row 0.5: Right side panels
// Right inner panel
layout_part("A1-R-Inner", START_X, START_Y + 1400 + SPACING, 90)
side_panel(is_inner=true);

// Right outer panel
layout_part("A1-R-Outer", START_X + 1000 + SPACING, START_Y + 1400 + SPACING, 90)
side_panel(is_inner=false);

// Base Y for subsequent rows
base_y = START_Y + 2*(1400 + SPACING);

// Row 1: Rear crossmember
layout_part("A2-Rear-Crossmember", START_X + 600, base_y, 90)
rear_crossmember();

// Row 2: Wheel mounts (4 parts) - 1/2" plate
wheel_y = base_y + 600 + SPACING;
layout_part("A4-1-Wheel-Mount-FL", START_X + 125, wheel_y, 0)
wheel_mount();

layout_part("A4-2-Wheel-Mount-FR", START_X + 125 + 250 + SPACING, wheel_y, 0)
wheel_mount();

layout_part("A4-3-Wheel-Mount-RL", START_X + 125 + 2*(250 + SPACING), wheel_y, 0)
wheel_mount();

layout_part("A4-4-Wheel-Mount-RR", START_X + 125 + 3*(250 + SPACING), wheel_y, 0)
wheel_mount();

// Row 3: Hydraulic cylinder lugs - 1/2" plate
lug_y = wheel_y + 250 + SPACING;

layout_part("A5-L-Lift-Cyl-Mount", START_X + 75, lug_y, 0)
cylinder_lug();

layout_part("A5-R-Lift-Cyl-Mount", START_X + 75 + 100 + SPACING, lug_y, 0)
cylinder_lug();

layout_part("Bucket-Cyl-Lug-1", START_X + 75 + 2*(100 + SPACING), lug_y, 0)
cylinder_lug();

layout_part("Bucket-Cyl-Lug-2", START_X + 75 + 3*(100 + SPACING), lug_y, 0)
cylinder_lug();

// Additional mounting lugs
layout_part("Bucket-Attach-L", START_X + 75 + 4*(100 + SPACING), lug_y, 0)
cylinder_lug();

layout_part("Bucket-Attach-R", START_X + 75 + 5*(100 + SPACING), lug_y, 0)
cylinder_lug();

// Row 4: Bucket bottom - 1/4" plate
bucket_y = lug_y + 150 + SPACING;

layout_part("E1-1-Bucket-Bottom", START_X + 550, bucket_y, 0)
bucket_bottom();

// Row 5: Standing deck - 1/4" plate
deck_y = bucket_y + 600 + SPACING;

layout_part("F1-Standing-Deck", START_X + 350, deck_y, 0)
standing_deck();

// Row 6: Bucket sides - 1/4" plate
side_y = deck_y + 400 + SPACING;

layout_part("E1-3-Bucket-Side-L", START_X + 300, side_y, 90)
bucket_side();

layout_part("E1-4-Bucket-Side-R", START_X + 300 + 450 + SPACING, side_y, 90)
bucket_side();

// Add title and legend
title_y = side_y + 600 + 2*SPACING;

translate([10, title_y, 0])
text("LifeTrac v25 CNC Layout - Sheet Metal Parts", size=16, font="Liberation Sans:style=Bold");

translate([10, title_y - 25, 0])
text("All parts include mounting holes, pivot points, and cutouts", size=10);

translate([10, title_y - 45, 0])
text("Maintain 3mm spacing between cuts | Rounded corners per design", size=8);
