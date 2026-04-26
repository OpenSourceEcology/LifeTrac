// tube_drilling_jig.scad
// 3D Printable Drilling Jig for 2x6 Rectangular Tubing
// LifeTrac v25 - Open Source Ecology
// License: GPL v3
//
// PURPOSE:
// This jig wraps around 2x6 rectangular tubing and provides accurately positioned
// holes for marking the centers of angle iron mounting holes. The jig includes
// alignment marks for tape measure reference.
//
// USAGE:
// 1. Print 4-6 copies of this jig (for marking all holes simultaneously)
// 2. Slide jig onto 2x6 tube at the desired position
// 3. Align the center reference mark with tape measure from tube end or other holes
// 4. Use a marker through the guide holes to mark hole centers, OR
// 5. Use a center punch (1/4" OD recommended) through the guide holes
// 6. Remove jig and drill marked holes
//
// HOLE PATTERN:
// The jig marks holes for 2"x2" angle iron brackets that mount the 2x6 cross
// frame tubes to the side wall plates. Two holes per side, spaced 2" apart
// vertically (for tube face) and positioned 1" from the angle corner.

// Try to import params, but provide defaults if not available
include <../lifetrac_v25_params.scad>

// =============================================================================
// JIG PARAMETERS
// =============================================================================

// Tube dimensions - 2"x6" rectangular tubing with 1/4" wall
// TUBE_2X6_1_4 = [50.8, 152.4, 6.35] from params (width, height, wall)
_TUBE_WIDTH = is_undef(TUBE_2X6_1_4) ? 50.8 : TUBE_2X6_1_4[0];    // 2" = 50.8mm
_TUBE_HEIGHT = is_undef(TUBE_2X6_1_4) ? 152.4 : TUBE_2X6_1_4[1];  // 6" = 152.4mm

// Jig clearance (gap between jig inner surface and tube outer surface)
JIG_CLEARANCE = 0.5;  // 0.5mm clearance for easy sliding fit

// Jig wall thickness
JIG_WALL = 4.0;  // 4mm walls for strength

// Jig length along tube axis
JIG_LENGTH = 101.6;  // 4 inches = 101.6mm

// Inner dimensions (tube + clearance)
JIG_INNER_WIDTH = _TUBE_WIDTH + 2*JIG_CLEARANCE;    // ~51.8mm
JIG_INNER_HEIGHT = _TUBE_HEIGHT + 2*JIG_CLEARANCE;  // ~153.4mm

// Outer dimensions (per user request: approximately 3" x 7")
JIG_OUTER_WIDTH = JIG_INNER_WIDTH + 2*JIG_WALL;     // ~59.8mm ≈ 2.4"
JIG_OUTER_HEIGHT = JIG_INNER_HEIGHT + 2*JIG_WALL;   // ~161.4mm ≈ 6.4"

// Actual outer dimensions (can be increased to match user request)
// User requested ~3" x 7" outer dimensions
JIG_OUTER_WIDTH_FINAL = 76.2;   // 3 inches
JIG_OUTER_HEIGHT_FINAL = 177.8; // 7 inches

// Recalculate wall thickness to achieve requested outer dimensions
JIG_WALL_X = (JIG_OUTER_WIDTH_FINAL - JIG_INNER_WIDTH) / 2;   // ~12.2mm
JIG_WALL_Z = (JIG_OUTER_HEIGHT_FINAL - JIG_INNER_HEIGHT) / 2; // ~12.2mm

// =============================================================================
// HOLE PATTERN PARAMETERS
// Based on frame_tube_angle_iron() in lifetrac_v25.scad
// =============================================================================

// Bolt hole diameter - 1/2" bolts (12.7mm) with clearance for marker/punch
BOLT_DIA = is_undef(BOLT_DIA_1_2) ? 12.7 : BOLT_DIA_1_2;

// Guide hole diameter - sized for 1/4" OD center punch (6.35mm)
// or use larger for marker tip
PUNCH_GUIDE_DIA = 6.35;    // 1/4" = 6.35mm (for center punch)
MARKER_GUIDE_DIA = 9.525;  // 3/8" = 9.525mm (for marker tip)

// Default guide hole size (use punch size, user can scale if needed)
GUIDE_HOLE_DIA = PUNCH_GUIDE_DIA;

// Hole spacing on tube face (from angle iron mounting pattern)
// Two holes spaced 2" apart vertically, centered on tube height
TUBE_HOLE_SPACING = 50.8;  // 2 inches between holes
TUBE_HOLE_OFFSET = 25.4;   // 1" from tube center (each direction)

// Hole position from tube edge (angle corner to hole center)
HOLE_INSET = 25.4;  // 1 inch from corner/edge

// =============================================================================
// ALIGNMENT MARK PARAMETERS
// =============================================================================

// Reference marks depth (engraved into surface)
MARK_DEPTH = 1.0;  // 1mm deep engraving
MARK_WIDTH = 1.0;  // 1mm wide lines

// Center cross size
CENTER_CROSS_LENGTH = 20;  // 20mm long arms

// Text height
TEXT_HEIGHT = 6;   // 6mm tall text
TEXT_DEPTH = 0.8;  // 0.8mm deep engraving

// =============================================================================
// RENDER OPTIONS
// =============================================================================

$fn = 32;

// Set to true to show the tube inside the jig (for visualization)
SHOW_TUBE = false;

// =============================================================================
// MAIN JIG MODULE
// =============================================================================

module tube_drilling_jig() {
    difference() {
        // Main jig body
        jig_body();
        
        // Subtract tube cavity
        tube_cavity();
        
        // Subtract guide holes on both narrow sides (tube faces)
        guide_holes();
        
        // Subtract alignment marks
        alignment_marks();
    }
}

// Outer shell of the jig
module jig_body() {
    // Centered on tube, extending in Y direction (along tube axis)
    translate([-JIG_OUTER_WIDTH_FINAL/2, -JIG_LENGTH/2, -JIG_OUTER_HEIGHT_FINAL/2])
    difference() {
        // Solid block with rounded corners for comfortable handling
        minkowski() {
            cube([JIG_OUTER_WIDTH_FINAL - 4, JIG_LENGTH - 4, JIG_OUTER_HEIGHT_FINAL - 4]);
            sphere(r=2, $fn=16);
        }
    }
}

// Tube-shaped cavity (U-shape that wraps around tube)
module tube_cavity() {
    // Main tube cavity - runs full length
    // The tube is 2"W x 6"H, jig wraps around the 6" tall sides
    // Opening is on the wide (6") face to slide over tube easily
    translate([-JIG_INNER_WIDTH/2, -JIG_LENGTH/2 - 1, -JIG_INNER_HEIGHT/2])
    cube([JIG_INNER_WIDTH, JIG_LENGTH + 2, JIG_INNER_HEIGHT]);
    
    // Opening slot on TOP (one of the 6" wide faces) to slide jig onto tube
    // The slot needs to be wide enough to pass over the 2" tube width
    // Make it exactly the tube width plus clearance
    slot_width = JIG_INNER_WIDTH;  // Full tube width for the slot
    translate([-slot_width/2, -JIG_LENGTH/2 - 1, JIG_INNER_HEIGHT/2 - JIG_CLEARANCE])
    cube([slot_width, JIG_LENGTH + 2, JIG_OUTER_HEIGHT_FINAL]);
}

// Guide holes for marking drill positions
module guide_holes() {
    // Holes are on the narrow faces (2" faces) of the tube
    // Two holes per face, spaced 2" apart vertically
    
    // Positive X face (right side when looking at tube end)
    for (z_offset = [-TUBE_HOLE_OFFSET, TUBE_HOLE_OFFSET]) {
        translate([JIG_INNER_WIDTH/2 - 1, 0, z_offset])
        rotate([0, 90, 0])
        cylinder(d=GUIDE_HOLE_DIA, h=JIG_WALL_X + 10);
    }
    
    // Negative X face (left side when looking at tube end)
    for (z_offset = [-TUBE_HOLE_OFFSET, TUBE_HOLE_OFFSET]) {
        translate([-JIG_INNER_WIDTH/2 - JIG_WALL_X - 5, 0, z_offset])
        rotate([0, 90, 0])
        cylinder(d=GUIDE_HOLE_DIA, h=JIG_WALL_X + 10);
    }
}

// Alignment marks for measurement reference
module alignment_marks() {
    // Center line mark on top surface (for tape measure alignment)
    // Y=0 is the center reference point
    
    // Top surface center cross
    translate([0, 0, JIG_OUTER_HEIGHT_FINAL/2 - MARK_DEPTH])
    linear_extrude(height=MARK_DEPTH + 1) {
        // Horizontal line (across tube width)
        translate([-CENTER_CROSS_LENGTH/2, -MARK_WIDTH/2])
        square([CENTER_CROSS_LENGTH, MARK_WIDTH]);
        // Vertical line (along tube length)
        translate([-MARK_WIDTH/2, -CENTER_CROSS_LENGTH/2])
        square([MARK_WIDTH, CENTER_CROSS_LENGTH]);
    }
    
    // Reference text on top surface
    translate([0, -JIG_LENGTH/4, JIG_OUTER_HEIGHT_FINAL/2 - TEXT_DEPTH])
    linear_extrude(height=TEXT_DEPTH + 1)
    text("CENTER", size=TEXT_HEIGHT, halign="center", valign="center", font="Liberation Sans:style=Bold");
    
    // Side marks showing hole positions relative to center
    // Each hole has a small arrow or line pointing to it
    
    // Right side (+X) marks
    translate([JIG_OUTER_WIDTH_FINAL/2 - MARK_DEPTH, 0, 0])
    rotate([0, 90, 0])
    linear_extrude(height=MARK_DEPTH + 1) {
        // Upper hole marker
        translate([-TUBE_HOLE_OFFSET - 5, -MARK_WIDTH/2])
        square([10, MARK_WIDTH]);
        // Lower hole marker  
        translate([TUBE_HOLE_OFFSET - 5, -MARK_WIDTH/2])
        square([10, MARK_WIDTH]);
    }
    
    // Left side (-X) marks
    translate([-JIG_OUTER_WIDTH_FINAL/2 - 1, 0, 0])
    rotate([0, 90, 0])
    linear_extrude(height=MARK_DEPTH + 1) {
        // Upper hole marker
        translate([-TUBE_HOLE_OFFSET - 5, -MARK_WIDTH/2])
        square([10, MARK_WIDTH]);
        // Lower hole marker
        translate([TUBE_HOLE_OFFSET - 5, -MARK_WIDTH/2])
        square([10, MARK_WIDTH]);
    }
    
    // Distance labels on front face
    translate([0, JIG_LENGTH/2 - TEXT_DEPTH, 0])
    rotate([90, 0, 0])
    linear_extrude(height=TEXT_DEPTH + 1) {
        // Upper hole distance (1" from center)
        translate([JIG_OUTER_WIDTH_FINAL/4, TUBE_HOLE_OFFSET])
        text("1\"", size=TEXT_HEIGHT*0.8, halign="center", valign="center", font="Liberation Sans");
        // Lower hole distance (1" from center)
        translate([JIG_OUTER_WIDTH_FINAL/4, -TUBE_HOLE_OFFSET])
        text("1\"", size=TEXT_HEIGHT*0.8, halign="center", valign="center", font="Liberation Sans");
    }
}

// Visualization of the tube inside the jig
module tube_visualization() {
    if (SHOW_TUBE) {
        color("SteelBlue", 0.3)
        translate([-_TUBE_WIDTH/2, -JIG_LENGTH*0.75, -_TUBE_HEIGHT/2])
        cube([_TUBE_WIDTH, JIG_LENGTH*1.5, _TUBE_HEIGHT]);
    }
}

// =============================================================================
// ALTERNATIVE DESIGNS
// =============================================================================

// Version with larger holes for marker use
module tube_drilling_jig_marker_version() {
    difference() {
        jig_body();
        tube_cavity();
        
        // Larger guide holes for marker tips
        // Positive X face
        for (z_offset = [-TUBE_HOLE_OFFSET, TUBE_HOLE_OFFSET]) {
            translate([JIG_INNER_WIDTH/2 - 1, 0, z_offset])
            rotate([0, 90, 0])
            cylinder(d=MARKER_GUIDE_DIA, h=JIG_WALL_X + 10);
        }
        
        // Negative X face
        for (z_offset = [-TUBE_HOLE_OFFSET, TUBE_HOLE_OFFSET]) {
            translate([-JIG_INNER_WIDTH/2 - JIG_WALL_X - 5, 0, z_offset])
            rotate([0, 90, 0])
            cylinder(d=MARKER_GUIDE_DIA, h=JIG_WALL_X + 10);
        }
        
        alignment_marks();
    }
}

// Minimal version - less material, faster to print
module tube_drilling_jig_minimal() {
    min_wall = 3;  // Thinner walls
    min_length = 50.8;  // 2" long instead of 4"
    
    difference() {
        // Smaller body
        translate([-(JIG_INNER_WIDTH/2 + min_wall), -min_length/2, -(JIG_INNER_HEIGHT/2 + min_wall)])
        cube([JIG_INNER_WIDTH + 2*min_wall, min_length, JIG_INNER_HEIGHT + 2*min_wall]);
        
        // Tube cavity
        translate([-JIG_INNER_WIDTH/2, -min_length/2 - 1, -JIG_INNER_HEIGHT/2])
        cube([JIG_INNER_WIDTH, min_length + 2, JIG_INNER_HEIGHT]);
        
        // Top opening - full width to slide over tube
        translate([-JIG_INNER_WIDTH/2, -min_length/2 - 1, JIG_INNER_HEIGHT/2 - JIG_CLEARANCE])
        cube([JIG_INNER_WIDTH, min_length + 2, JIG_INNER_HEIGHT]);
        
        // Guide holes
        for (z_offset = [-TUBE_HOLE_OFFSET, TUBE_HOLE_OFFSET]) {
            // Right side
            translate([JIG_INNER_WIDTH/2 - 1, 0, z_offset])
            rotate([0, 90, 0])
            cylinder(d=GUIDE_HOLE_DIA, h=min_wall + 10);
            // Left side
            translate([-JIG_INNER_WIDTH/2 - min_wall - 5, 0, z_offset])
            rotate([0, 90, 0])
            cylinder(d=GUIDE_HOLE_DIA, h=min_wall + 10);
        }
    }
}

// =============================================================================
// RENDER
// =============================================================================

// Main jig (uncomment the version you want to render)
tube_drilling_jig();
// tube_drilling_jig_marker_version();
// tube_drilling_jig_minimal();

// Show tube for reference (set SHOW_TUBE = true above)
tube_visualization();

// =============================================================================
// ASSEMBLY NOTES
// =============================================================================
/*
PRINTING RECOMMENDATIONS:
- Material: PLA or PETG
- Layer height: 0.2mm
- Infill: 20-30%
- Walls: 3 perimeters minimum
- Supports: May be needed for the cavity overhang
- Orientation: Print with opening facing up (standing on the 7" face)

RECOMMENDED QUANTITY:
Print 4-6 jigs to mark all holes on a tube simultaneously:
- 2 jigs per tube face × 2 faces = 4 jigs minimum
- Extra jigs allow marking multiple positions without moving jigs

USAGE INSTRUCTIONS:
1. Measure and mark the Y position (along tube length) for each jig
2. Slide jigs onto tube, aligning CENTER mark with your measurement
3. The jigs will self-align to the tube faces
4. Through each guide hole, either:
   a) Insert marker and make a dot at center
   b) Insert center punch (1/4" OD) and tap with hammer
5. Remove jigs and drill at marked centers

HOLE PATTERN DETAILS:
- 4 holes total per jig position (2 per narrow face)
- Holes are 2" (50.8mm) apart vertically
- Holes are centered on the 6" face height
- Pattern matches 2"x2"x1/4" angle iron mounting
- Use 1/2" (12.7mm) drill bit for final holes

CUSTOMIZATION:
- Adjust GUIDE_HOLE_DIA for different punch/marker sizes
- Use tube_drilling_jig_marker_version() for larger marker holes
- Use tube_drilling_jig_minimal() for faster printing
- Set JIG_CLEARANCE tighter (0.3mm) for snug fit or looser (0.8mm) for easy sliding
*/
