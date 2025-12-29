// platform_angle_arm.scad
// Angle iron arm for LifeTrac v25 folding platform
// NOT for CNC cutting - this is purchased 2"x2"x1/4" angle iron stock
// Part of the 5-component folding platform system
//
// This module is for 3D visualization and drilling template only.
// The angle iron should be purchased as stock and drilled per this template.
//
// Two arms required (left and right side)
//
// Installation orientation (when deployed/horizontal):
//   - Angle iron corner points UP (apex at top)
//   - One leg is horizontal (deck sits on top of this)
//   - Other leg is vertical (pivot bracket bolts to this)
//   - Arm extends from pivot point toward rear of machine (operator side)
//
// Drilling instructions:
//   - Pivot end: 3 holes through VERTICAL leg for pivot bracket attachment
//   - Deck end: 3 holes through HORIZONTAL leg for deck plate attachment

include <../lifetrac_v25_params.scad>

/**
 * Creates the angle iron arm with bolt holes for visualization/drilling template
 * 
 * Coordinate system when rendered (matches installed orientation when deployed):
 *   - Arm runs along Y axis (length), extending in -Y direction (toward rear)
 *   - Corner of L-profile at top (+Z)
 *   - One leg horizontal (in XY plane), one leg vertical (in XZ plane)
 *   - Origin at pivot end of arm
 *
 * @param show_holes If true, shows bolt holes (for visualization/drilling template)
 */
module platform_angle_arm(show_holes=true) {
    // Local variables for clarity
    length = PLATFORM_ARM_LENGTH - PLATFORM_BRACKET_WIDTH/2;
    leg = PLATFORM_ANGLE_LEG;        // 50.8mm (2")
    thick = PLATFORM_ANGLE_THICK;    // 6.35mm (1/4")
    
    // Bolt hole parameters
    bolt_hole_dia = PLATFORM_BOLT_DIA + PLATFORM_BOLT_CLEARANCE;
    
    color("DimGray")
    difference() {
        // Angle iron profile - corner at top
        // Horizontal leg extends in +X, vertical leg extends in -Z
        translate([0, -length, 0])
        rotate([-90, 0, 0])
        linear_extrude(height=length)
        polygon([
            // Corner at origin, horizontal leg to +X, vertical leg to -Y (which becomes -Z after rotation)
            [-thick, 0],           // Inside corner of vertical leg
            [leg - thick, 0],      // Outside edge of horizontal leg
            [leg - thick, -thick], // Bottom of horizontal leg
            [0, -thick],           // Inside corner
            [0, -leg + thick],     // Inside of vertical leg
            [-thick, -leg + thick] // Outside edge of vertical leg
        ]);
        
        if (show_holes) {
            // =============================================================
            // PIVOT BRACKET END BOLT HOLES (2 holes through vertical leg)
            // =============================================================
            // These holes go through the vertical leg (in X direction)
            // For attaching the pivot bracket plate
            // Positions from pivot end (Y=0) extending in -Y
            for (y_pos = [-PLATFORM_SIDE_BOLT_START, -(PLATFORM_SIDE_BOLT_START + PLATFORM_SIDE_BOLT_SPACING)]) {
                translate([-leg/2, y_pos, -leg/2])
                rotate([0, 90, 0])
                cylinder(d=bolt_hole_dia, h=leg + 4, center=true, $fn=24);
            }
            
            // =============================================================
            // DECK END BOLT HOLES (2 holes through horizontal leg)
            // =============================================================
            // These holes go through the horizontal leg (in Z direction)
            // For attaching the deck plate from above
            // Centered on length
            center_y = -length / 2;
            for (y_pos = [center_y - PLATFORM_SIDE_DECK_BOLT_SPACING/2, center_y + PLATFORM_SIDE_DECK_BOLT_SPACING/2]) {
                translate([leg/2 - thick/2, y_pos, -thick/2])
                cylinder(d=bolt_hole_dia, h=thick + 4, center=true, $fn=24);
            }
        }
    }
}

/**
 * Mirror version for right side (flips the L-profile for opposite side)
 */
module platform_angle_arm_mirrored(show_holes=true) {
    mirror([1, 0, 0])
    platform_angle_arm(show_holes);
}

// Render the part for preview
if ($preview || len(search("platform_angle_arm.scad", parent_modules())) == 0) {
    // Show left arm
    platform_angle_arm();
    
    // Show right arm (mirrored) offset for comparison
    translate([150, 0, 0])
    platform_angle_arm_mirrored();
    
    // Labels
    color("Red") {
        translate([0, 0, 30]) text("LEFT", size=10);
        translate([150, 0, 30]) text("RIGHT", size=10);
        translate([0, -25, 20]) text("PIVOT END", size=6);
        translate([0, -PLATFORM_ARM_LENGTH + 25, 20]) text("DECK END", size=6);
    }
}
