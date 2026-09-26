// =============================================================================
// LifeTrac v25 Structural Steel Parts - Master Include File
// =============================================================================
//
// This file provides access to all structural steel parts for the LifeTrac v25
// assembly. Use this file to gain access to all individual part modules.
//
// USAGE:
//   use <parts/structural/structural_parts.scad>
//
// Then call any part module, for example:
//   part_a1_back_outer_vertical();
//   part_t1_front_frame_tube();
//
// NOTE: OpenSCAD 'use' statements don't re-export modules, so we define
// wrapper modules here that call the modules from individual part files.
// =============================================================================

include <../../lifetrac_v25_params.scad>

// =============================================================================
// ANGLE IRON PARTS (A1-A10) - Wrapper modules
// =============================================================================

// A1: Back Stiffener - Outer Wall Vertical
use <angle_iron_a1_back_outer_vertical.scad>
module part_a1_back_outer_vertical(show_holes=true) {
    echo(BOM_PART = "A1");  // counted by drawings/generate_part_drawings.py
    _part_a1_back_outer_vertical(show_holes);
}

// A2: Back Stiffener - Inner Wall Vertical
use <angle_iron_a2_back_inner_vertical.scad>
module part_a2_back_inner_vertical(show_holes=true) {
    echo(BOM_PART = "A2");  // counted by drawings/generate_part_drawings.py
    _part_a2_back_inner_vertical(show_holes);
}

// A3: Front Stiffener - Outer Section Vertical
use <angle_iron_a3_front_outer_vertical.scad>
module part_a3_front_outer_vertical(show_holes=true) {
    echo(BOM_PART = "A3");  // counted by drawings/generate_part_drawings.py
    _part_a3_front_outer_vertical(show_holes);
}

// A4: Frame Tube Mount Angles (frame tubes, motor plates, front stiffener)
use <angle_iron_a4_frame_tube_mount.scad>
module part_a4_frame_tube_mount(show_holes=true) {
    echo(BOM_PART = "A4");  // counted by drawings/generate_part_drawings.py
    _part_a4_frame_tube_mount(show_holes);
}

// A5: Arm Crossbeam Mount Angles
use <angle_iron_a5_arm_crossbeam_mount.scad>
module part_a5_arm_crossbeam_mount(show_holes=true) {
    echo(BOM_PART = "A5");  // counted by drawings/generate_part_drawings.py
    _part_a5_arm_crossbeam_mount(show_holes);
}

// A6: Bottom Stiffener Horizontal Angles - Split into 3 segments (bottom stiffener, motor plates)
use <angle_iron_a6_bottom_horizontal.scad>
module part_a6_bottom_segment_1(show_holes=true) { echo(BOM_PART = "A6-1"); _part_a6_bottom_segment_1(show_holes); }
module part_a6_bottom_segment_2(show_holes=true) { echo(BOM_PART = "A6-2"); _part_a6_bottom_segment_2(show_holes); }
module part_a6_bottom_segment_3(show_holes=true) { echo(BOM_PART = "A6-3"); _part_a6_bottom_segment_3(show_holes); }
module part_a6_split_horizontal_angle_iron(show_holes=true) { echo(BOM_PART = "A6-1"); echo(BOM_PART = "A6-2"); echo(BOM_PART = "A6-3"); _part_a6_split_horizontal_angle_iron(show_holes); }

// A7: Platform Side Angle Irons
use <angle_iron_a7_platform_side.scad>
module part_a7_platform_side_angle(show_holes=true) {
    echo(BOM_PART = "A7");  // counted by drawings/generate_part_drawings.py
    _part_a7_platform_side_angle(show_holes);
}

// A8: Platform Transverse Angle Irons
use <angle_iron_a8_platform_transverse.scad>
module part_a8_platform_transverse_angle(show_holes=true) {
    echo(BOM_PART = "A8");  // counted by drawings/generate_part_drawings.py
    _part_a8_platform_transverse_angle(show_holes);
}

// A9: Front Stiffener Center Section Angles
use <angle_iron_a9_front_center.scad>
module part_a9_front_center_angle(show_holes=true) {
    echo(BOM_PART = "A9");  // counted by drawings/generate_part_drawings.py
    _part_a9_front_center_angle(show_holes);
}

// A10: Front Stiffener Outer Section Angles
use <angle_iron_a10_front_outer.scad>
module part_a10_front_outer_angle(show_holes=true) {
    echo(BOM_PART = "A10");  // counted by drawings/generate_part_drawings.py
    _part_a10_front_outer_angle(show_holes);
}

// =============================================================================
// TUBING PARTS (T1-T5) - Wrapper modules
// =============================================================================

// T1: Front Cross Frame Tube
use <tube_t1_front_frame.scad>
module part_t1_front_frame_tube(show_cutaway=false, show_holes=true) {
    echo(BOM_PART = "T1");  // counted by drawings/generate_part_drawings.py
    _part_t1_front_frame_tube(show_cutaway, show_holes);
}

// T2: Rear Cross Frame Tube
use <tube_t2_rear_frame.scad>
module part_t2_rear_frame_tube(show_cutaway=false, show_holes=true) {
    echo(BOM_PART = "T2");  // counted by drawings/generate_part_drawings.py
    _part_t2_rear_frame_tube(show_cutaway, show_holes);
}

// T3: Arm Crossbeam Tube
use <tube_t3_arm_crossbeam.scad>
module part_t3_arm_crossbeam(show_holes=true) {
    echo(BOM_PART = "T3");  // counted by drawings/generate_part_drawings.py
    _part_t3_arm_crossbeam(show_holes);
}

// T4: Main Arm Tubes
use <tube_t4_arm_main.scad>
module part_t4_arm_main(show_holes=true) {
    echo(BOM_PART = "T4");  // counted by drawings/generate_part_drawings.py
    _part_t4_arm_main(show_holes);
}

// T5: Arm Leg Spacer Tubes
use <tube_t5_arm_leg_spacer.scad>
module part_t5_arm_leg_spacer_raw() { echo(BOM_PART = "T5"); _part_t5_arm_leg_spacer_raw(); }
module part_t5_arm_leg_spacer_cut(show_holes=true) { echo(BOM_PART = "T5"); _part_t5_arm_leg_spacer_cut(show_holes); }

// =============================================================================
// QUANTITIES
// =============================================================================
// Piece counts are not typed here, because typed counts go stale whenever the
// assembly changes. Each wrapper above echoes a BOM_PART marker, and
// drawings/generate_part_drawings.py counts the markers when it evaluates
// lifetrac_v25.scad. DESIGN-STRUCTURAL/drawings/generated/INDEX.md lists the
// quantity per machine of every part, with totals and the total cut length per
// stock size. A wrapper that the assembly does not call counts zero.
//
// Stock: angle iron A1-A10 is 2"x2"x1/4"; tubing T1-T5 is 2"x6"x1/4".
// =============================================================================

// Display all parts for preview
module show_all_structural_parts() {
    // Arrange parts in a row for visualization
    spacing = 250;
    
    // Angle irons
    translate([0*spacing, 0, 0]) part_a1_back_outer_vertical();
    translate([1*spacing, 0, 0]) part_a2_back_inner_vertical();
    translate([2*spacing, 0, 0]) part_a3_front_outer_vertical();
    translate([3*spacing, 0, 0]) part_a4_frame_tube_mount();
    translate([4*spacing, 0, 0]) part_a5_arm_crossbeam_mount();
    translate([5*spacing, 0, 0]) part_a6_bottom_segment_1();
    translate([6*spacing, 0, 0]) part_a7_platform_side_angle();
    translate([7*spacing, 0, 0]) part_a8_platform_transverse_angle();
    translate([8*spacing, 0, 0]) part_a9_front_center_angle();
    translate([9*spacing, 0, 0]) part_a10_front_outer_angle();
    
    // Tubing - offset in Y
    translate([0*spacing, 500, 0]) part_t1_front_frame_tube();
    translate([2*spacing, 500, 0]) part_t2_rear_frame_tube();
    translate([4*spacing, 500, 0]) part_t3_arm_crossbeam();
    translate([6*spacing, 500, 0]) part_t4_arm_main();
    translate([8*spacing, 500, 0]) part_t5_arm_leg_spacer_raw();
}

// Uncomment to preview all parts:
// show_all_structural_parts();

echo("=== STRUCTURAL PARTS LOADED ===");
echo("Angle iron parts: A1-A10 (2x2x1/4 angle)");
echo("Tubing parts: T1-T5 (2x6x1/4 rectangular tube)");
echo("Quantities per machine: DESIGN-STRUCTURAL/drawings/generated/INDEX.md");
echo("See STRUCTURAL_PARTS_CATALOG.md for details");
