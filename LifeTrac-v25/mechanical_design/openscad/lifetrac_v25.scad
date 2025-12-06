// lifetrac_v25.scad
// Main LifeTrac v25 Design File
// Open Source Ecology - Compact Remote Controlled Utility Loader
// License: GPL v3

// Import modules
use <../modules/plate_steel.scad>
use <../modules/structural_steel.scad>
use <../modules/fasteners.scad>
use <../modules/hydraulics.scad>
use <../modules/wheels.scad>

// Global parameters
$fn = 32;

// Animation parameter (0 to 1)
animation_time = $t;

// Display options
show_wheels = true;
show_hydraulics = true;
show_frame = true;
show_loader_arms = true;
show_bucket = true;
show_standing_deck = true;
exploded_view = false;
explode_distance = exploded_view ? 200 : 0;

// Machine dimensions (in mm)
// Target size: between Toro Dingo (915mm wide) and Bobcat (1830mm wide)
// Chosen dimensions: ~1200mm wide, ~1800mm long, ~1000mm tall (without loader)

MACHINE_WIDTH = 1200;   // Overall width
MACHINE_LENGTH = 1800;  // Overall length
MACHINE_HEIGHT = 1000;  // Height to top of frame
WHEEL_BASE = 1400;      // Distance between front and rear axles
TRACK_WIDTH = 1000;     // Distance between left and right wheels

FRAME_TUBE_SIZE = TUBE_4X4_1_4;  // 4x4" square tubing
WHEEL_DIAMETER = 500;            // 500mm diameter wheels
WHEEL_WIDTH = 200;               // 200mm wide wheels

// Loader arm dimensions
ARM_LENGTH = 1200;           // Loader arm length
ARM_HEIGHT_RETRACTED = 300;  // Height above frame when retracted
ARM_HEIGHT_EXTENDED = 2000;  // Maximum height when fully raised
ARM_LIFT_ANGLE = animation_time * 60;  // 0-60 degrees based on animation
BUCKET_WIDTH = 1100;         // Bucket width
BUCKET_DEPTH = 600;          // Bucket depth
BUCKET_TILT_ANGLE = animation_time * 90;  // 0-90 degrees

// Hydraulic cylinder dimensions
LIFT_CYLINDER_STROKE = 400;
BUCKET_CYLINDER_STROKE = 300;

/**
 * Part A1 - Base Frame Assembly
 * Main structural frame using 4x4" square tubing
 * Connects to: All wheel assemblies (B1-B4), loader arm pivots (C1), standing deck (F1)
 */
module base_frame() {
    color("Silver")
    {
        // Longitudinal frame members (left and right)
        translate([-TRACK_WIDTH/2, 0, 0])
        rotate([90, 0, 0])
        square_tubing(MACHINE_LENGTH, FRAME_TUBE_SIZE, true);
        
        translate([TRACK_WIDTH/2, 0, 0])
        rotate([90, 0, 0])
        square_tubing(MACHINE_LENGTH, FRAME_TUBE_SIZE, true);
        
        // Cross members
        // Front cross member
        translate([0, MACHINE_LENGTH/2, 0])
        rotate([0, 90, 0])
        square_tubing(TRACK_WIDTH + FRAME_TUBE_SIZE[0], FRAME_TUBE_SIZE, true);
        
        // Center cross member
        rotate([0, 90, 0])
        square_tubing(TRACK_WIDTH + FRAME_TUBE_SIZE[0], FRAME_TUBE_SIZE, true);
        
        // Rear cross member
        translate([0, -MACHINE_LENGTH/2, 0])
        rotate([0, 90, 0])
        square_tubing(TRACK_WIDTH + FRAME_TUBE_SIZE[0], FRAME_TUBE_SIZE, true);
        
        // Front upright supports for loader mounting
        translate([-TRACK_WIDTH/2, MACHINE_LENGTH/2, 0])
        square_tubing(MACHINE_HEIGHT, FRAME_TUBE_SIZE, false);
        
        translate([TRACK_WIDTH/2, MACHINE_LENGTH/2, 0])
        square_tubing(MACHINE_HEIGHT, FRAME_TUBE_SIZE, false);
        
        // Top cross member for loader pivot
        translate([0, MACHINE_LENGTH/2, MACHINE_HEIGHT])
        rotate([0, 90, 0])
        square_tubing(TRACK_WIDTH + FRAME_TUBE_SIZE[0], FRAME_TUBE_SIZE, true);
    }
}

/**
 * Part A2 - Wheel Mounting Plates
 * 1/2" plate steel mounting plates for wheel assemblies
 * Connects to: Base frame (A1), wheel assemblies (B1-B4)
 */
module wheel_mounting_plates() {
    plate_size = 300;
    
    positions = [
        [-TRACK_WIDTH/2, MACHINE_LENGTH/2 - 200],   // Front left
        [TRACK_WIDTH/2, MACHINE_LENGTH/2 - 200],    // Front right
        [-TRACK_WIDTH/2, -MACHINE_LENGTH/2 + 200],  // Rear left
        [TRACK_WIDTH/2, -MACHINE_LENGTH/2 + 200]    // Rear right
    ];
    
    for (pos = positions) {
        translate([pos[0], pos[1], -50])
        color("DarkSlateGray")
        plate_with_holes(plate_size, plate_size, PLATE_1_2_INCH, 13, 40);
    }
}

/**
 * Part B1-B4 - Wheel Assemblies with Hydraulic Motors
 * Four wheel assemblies (B1=FL, B2=FR, B3=RL, B4=RR)
 * Connects to: Wheel mounting plates (A2), hydraulic system
 */
module wheel_assemblies() {
    if (show_wheels) {
        positions = [
            [-TRACK_WIDTH/2 - explode_distance, MACHINE_LENGTH/2 - 200 + explode_distance, 0],   // Front left
            [TRACK_WIDTH/2 + explode_distance, MACHINE_LENGTH/2 - 200 + explode_distance, 0],    // Front right
            [-TRACK_WIDTH/2 - explode_distance, -MACHINE_LENGTH/2 + 200 - explode_distance, 0],  // Rear left
            [TRACK_WIDTH/2 + explode_distance, -MACHINE_LENGTH/2 + 200 - explode_distance, 0]    // Rear right
        ];
        
        for (i = [0:3]) {
            translate(positions[i])
            rotate([0, 0, animation_time * 360])  // Animated wheel rotation
            {
                // Wheel
                rotate([0, 90, 0])
                wheel(WHEEL_DIAMETER, WHEEL_WIDTH, 150, true);
                
                // Hydraulic motor (for powered wheels - typically front 2 or all 4)
                if (i < 2 || true) {  // All 4 wheels powered in this config
                    translate([0, 0, -80])
                    rotate([0, 0, 0])
                    hydraulic_motor(100, 25.4, 80, true);
                }
            }
        }
    }
}

/**
 * Part C1 - Loader Arm Structure
 * Main loader arms using plate steel and square tubing
 * Connects to: Base frame uprights (A1), lift cylinders (D1-D2), bucket attachment (C2)
 */
module loader_arms() {
    if (show_loader_arms) {
        arm_width = 150;
        
        color("Orange")
        translate([0, MACHINE_LENGTH/2, MACHINE_HEIGHT])
        rotate([ARM_LIFT_ANGLE, 0, 0])
        {
            // Left arm
            translate([-TRACK_WIDTH/2, 0, 0])
            {
                // Main arm beam
                rotate([0, 90, 0])
                square_tubing(ARM_LENGTH, TUBE_3X3_1_4, false);
                
                // Reinforcement plates
                translate([0, 0, 0])
                rotate([0, 90, 0])
                color("DarkSlateGray")
                plate_steel(arm_width, ARM_LENGTH, PLATE_1_4_INCH, 6.35);
            }
            
            // Right arm  
            translate([TRACK_WIDTH/2, 0, 0])
            {
                // Main arm beam
                rotate([0, 90, 0])
                square_tubing(ARM_LENGTH, TUBE_3X3_1_4, false);
                
                // Reinforcement plates
                translate([0, 0, 0])
                rotate([0, 90, 0])
                color("DarkSlateGray")
                plate_steel(arm_width, ARM_LENGTH, PLATE_1_4_INCH, 6.35);
            }
            
            // Cross brace
            translate([0, ARM_LENGTH - 200, 0])
            rotate([0, 90, 0])
            square_tubing(TRACK_WIDTH + 200, TUBE_3X3_1_4, true);
        }
    }
}

/**
 * Part C2 - Bucket Attachment Interface
 * Quick-attach bucket mounting system
 * Connects to: Loader arms (C1), bucket (E1), bucket cylinders (D3-D4)
 */
module bucket_attachment() {
    if (show_bucket) {
        color("Yellow")
        translate([0, MACHINE_LENGTH/2, MACHINE_HEIGHT])
        rotate([ARM_LIFT_ANGLE, 0, 0])
        translate([0, ARM_LENGTH, 0])
        rotate([BUCKET_TILT_ANGLE, 0, 0])
        {
            // Bucket
            bucket();
        }
    }
}

/**
 * Part E1 - Bucket Assembly
 * Main bucket using 1/4" plate steel with rounded corners
 * Connects to: Bucket attachment (C2)
 */
module bucket() {
    bucket_height = 400;
    
    // Bottom plate
    color("Yellow")
    translate([-BUCKET_WIDTH/2, 0, -bucket_height])
    plate_steel(BUCKET_WIDTH, BUCKET_DEPTH, PLATE_1_4_INCH);
    
    // Back plate
    color("Yellow")
    translate([-BUCKET_WIDTH/2, 0, -bucket_height])
    rotate([90, 0, 0])
    plate_steel(BUCKET_WIDTH, bucket_height, PLATE_1_4_INCH);
    
    // Side plates
    color("Yellow")
    translate([-BUCKET_WIDTH/2, 0, -bucket_height])
    rotate([0, 90, 0])
    rotate([0, 0, 90])
    plate_steel(BUCKET_DEPTH, bucket_height, PLATE_1_4_INCH);
    
    color("Yellow")
    translate([BUCKET_WIDTH/2, 0, -bucket_height])
    rotate([0, 90, 0])
    rotate([0, 0, 90])
    plate_steel(BUCKET_DEPTH, bucket_height, PLATE_1_4_INCH);
}

/**
 * Part D1-D2 - Loader Lift Cylinders
 * Two hydraulic cylinders for raising/lowering loader arms
 * Connects to: Base frame (A1), loader arms (C1)
 */
module lift_cylinders() {
    if (show_hydraulics) {
        cylinder_positions = [
            [-TRACK_WIDTH/2 + 100, MACHINE_LENGTH/2, MACHINE_HEIGHT/2],
            [TRACK_WIDTH/2 - 100, MACHINE_LENGTH/2, MACHINE_HEIGHT/2]
        ];
        
        for (pos = cylinder_positions) {
            translate(pos)
            rotate([ARM_LIFT_ANGLE/2, 0, 0])
            hydraulic_cylinder(63.5, 31.75, LIFT_CYLINDER_STROKE, 
                             false, animation_time * LIFT_CYLINDER_STROKE, "clevis");
        }
    }
}

/**
 * Part D3-D4 - Bucket Tilt Cylinders
 * Two hydraulic cylinders for tilting bucket
 * Connects to: Loader arms (C1), bucket attachment (C2)
 */
module bucket_cylinders() {
    if (show_hydraulics) {
        translate([0, MACHINE_LENGTH/2, MACHINE_HEIGHT])
        rotate([ARM_LIFT_ANGLE, 0, 0])
        {
            cylinder_positions = [
                [-TRACK_WIDTH/2 + 150, ARM_LENGTH - 300, 0],
                [TRACK_WIDTH/2 - 150, ARM_LENGTH - 300, 0]
            ];
            
            for (pos = cylinder_positions) {
                translate(pos)
                rotate([BUCKET_TILT_ANGLE/2 + 20, 0, 0])
                hydraulic_cylinder(50, 25, BUCKET_CYLINDER_STROKE,
                                 false, animation_time * BUCKET_CYLINDER_STROKE, "clevis");
            }
        }
    }
}

/**
 * Part F1 - Standing Deck (Optional)
 * Platform for operator to stand while controlling machine
 * Connects to: Base frame rear (A1)
 */
module standing_deck() {
    if (show_standing_deck) {
        deck_width = TRACK_WIDTH;
        deck_depth = 400;
        
        translate([0, -MACHINE_LENGTH/2 - explode_distance, 200])
        {
            // Deck plate with anti-slip pattern
            color("DarkSlateGray")
            difference() {
                plate_steel(deck_width, deck_depth, PLATE_1_4_INCH, 6.35, true);
                
                // Anti-slip holes pattern
                for (x = [-deck_width/2+50:100:deck_width/2-50]) {
                    for (y = [-deck_depth/2+50:100:deck_depth/2-50]) {
                        translate([x, y, -1])
                        cylinder(d=20, h=10);
                    }
                }
            }
            
            // Support brackets
            for (x = [-deck_width/2 + 100, deck_width/2 - 100]) {
                translate([x, -deck_depth/2, -100])
                color("Silver")
                square_tubing(100, TUBE_2X2_1_4, false);
            }
        }
    }
}

/**
 * Part G1 - Control System Housing
 * Protective housing for Arduino Opta controller and electronics
 * Connects to: Base frame (A1)
 */
module control_housing() {
    housing_width = 300;
    housing_height = 400;
    housing_depth = 200;
    
    translate([0, 0, 150])
    {
        // Housing box
        color("Blue", 0.3)
        translate([-housing_width/2, -housing_depth/2, 0])
        difference() {
            cube([housing_width, housing_depth, housing_height]);
            translate([10, 10, 10])
            cube([housing_width-20, housing_depth-20, housing_height-10]);
        }
        
        // Mounting plate
        color("DarkSlateGray")
        translate([0, 0, -10])
        plate_with_holes(housing_width, housing_depth, PLATE_1_4_INCH);
    }
}

// Main assembly
module lifetrac_v25_assembly() {
    if (show_frame) {
        base_frame();
        wheel_mounting_plates();
        control_housing();
    }
    
    wheel_assemblies();
    loader_arms();
    bucket_attachment();
    lift_cylinders();
    bucket_cylinders();
    standing_deck();
}

// Render the complete assembly
lifetrac_v25_assembly();

// Add ground plane for reference
color("Green", 0.2)
translate([-2000, -2000, -200])
cube([4000, 4000, 1]);
