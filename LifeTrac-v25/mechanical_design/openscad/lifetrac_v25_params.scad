// lifetrac_v25_params.scad
// Essential parameters for LifeTrac v25 individual parts
// This file contains only the constants needed for part design
// Full assembly uses lifetrac_v25.scad

// -----------------------------------------------------------------------------
// Compatibility helper
// OpenSCAD throws warnings when the BOSL2 helper `parent_modules()` is missing.
// We provide a stub that returns an empty list so part files can run cleanly
// when opened directly without pulling in BOSL2.
function parent_modules() = is_undef($parent_modules) ? [] : $parent_modules;

// =============================================================================
// MATERIAL CONSTANTS
// =============================================================================

PLATE_1_4_INCH = 6.35;   // 1/4" = 6.35mm
PLATE_1_2_INCH = 12.7;   // 1/2" = 12.7mm
PLATE_3_4_INCH = 19.05;  // 3/4" = 19.05mm
TUBE_3X3_1_4 = [76.2, 6.35];    // 3"x3" x 1/4" wall
TUBE_4X4_1_4 = [101.6, 6.35];   // 4"x4" x 1/4" wall (OSE standard)
TUBE_2X2_1_4 = [50.8, 6.35];    // 2"x2" x 1/4" wall
TUBE_2X4_1_4 = [50.8, 101.6, 6.35]; // 2"x4" rectangular

// Angle iron sizes [leg_size, thickness]
ANGLE_2X2_1_4 = [50.8, 6.35];   // 2"x2" x 1/4" angle iron
ANGLE_3X3_1_4 = [76.2, 6.35];   // 3"x3" x 1/4" angle iron
ANGLE_4X4_1_4 = [101.6, 6.35];  // 4"x4" x 1/4" angle iron

// Bolt/pin diameters
PIVOT_PIN_DIA = 38.1;    // 1.5" pivot pin
BOLT_DIA_1_2 = 12.7;     // 1/2" bolt
BOLT_DIA_3_4 = 19.05;    // 3/4" bolt
BOLT_DIA_1 = 25.4;       // 1" bolt

// =============================================================================
// MACHINE DIMENSIONS
// =============================================================================

// Target size: between Toro Dingo (915mm wide) and Bobcat (1830mm wide)
MACHINE_WIDTH = 1200;    // Overall width
MACHINE_LENGTH = 1800;   // Overall length
MACHINE_HEIGHT = 1000;   // Height to top of frame
WHEEL_BASE = 1400;       // Distance between front and rear axles
TRACK_WIDTH = 900;       // Distance between centerlines of left/right side panels

// Side panel sandwich configuration
SANDWICH_SPACING = 120;  // Gap between inner and outer panels (for arm pivot)
PANEL_THICKNESS = PLATE_1_2_INCH;

// Wheel dimensions
WHEEL_DIAMETER = 500;    // 500mm diameter wheels
WHEEL_WIDTH = 200;       // 200mm wide wheels
WHEEL_CLEARANCE = 50;    // Clearance between wheel and outer panel

// Ground clearance - bottom of frame above ground
GROUND_CLEARANCE = 150;  // 150mm ground clearance under frame
WHEEL_RADIUS = WHEEL_DIAMETER / 2;

// Frame sits with bottom at GROUND_CLEARANCE height
FRAME_Z_OFFSET = GROUND_CLEARANCE;

// Calculate wheel X offset - wheels positioned outside the outer panels
WHEEL_X_OFFSET = TRACK_WIDTH/2 + SANDWICH_SPACING/2 + PANEL_THICKNESS + WHEEL_WIDTH/2 + WHEEL_CLEARANCE;

// =============================================================================
// LOADER ARM DIMENSIONS
// =============================================================================

// Design constraints
BUCKET_GROUND_CLEARANCE = 0;      // Bucket bottom at ground level when lowered
BUCKET_FRONT_CLEARANCE = 100;     // 100mm (~4") clearance between bucket back and front of machine

// Arm pivot point (at top rear of side panels - tall end)
ARM_PIVOT_Y = 200;  // Near rear of machine, between sandwich plates  
ARM_PIVOT_Z_HEIGHT = MACHINE_HEIGHT - 50;  // Height above frame bottom
ARM_PIVOT_Z = FRAME_Z_OFFSET + ARM_PIVOT_Z_HEIGHT;  // Absolute Z position

// Calculate required arm length
MIN_HORIZONTAL_REACH = WHEEL_BASE + BUCKET_FRONT_CLEARANCE - ARM_PIVOT_Y;
ARM_LENGTH = ceil(sqrt(pow(ARM_PIVOT_Z, 2) + pow(MIN_HORIZONTAL_REACH, 2)));
ARM_GROUND_ANGLE = asin(ARM_PIVOT_Z / ARM_LENGTH);  // Angle below horizontal (degrees)

ARM_TUBE_SIZE = TUBE_3X3_1_4;         // 3"x3" arm tubing
ARM_SPACING = TRACK_WIDTH;            // Distance between arm centerlines

// Arm angle limits
ARM_MIN_ANGLE = -ARM_GROUND_ANGLE;     // Lowest position (at ground)
ARM_MAX_ANGLE = 60;                     // Maximum raised position

// Cross beam configuration
CROSS_BEAM_SIZE = TUBE_2X2_1_4[0];    // 2"x2" tube size
CROSS_BEAM_CLEARANCE = 15;             // Extra clearance around cross beam in cutout
CROSS_BEAM_1_POS = ARM_LENGTH * 0.65;   // First cross beam position
CROSS_BEAM_2_POS = ARM_LENGTH * 0.95;   // Second cross beam position (near bucket)

// Pivot position in panel coordinates (for arc slot calculations)
PIVOT_PANEL_X = ARM_PIVOT_Y;                          // Pivot X in panel coords
PIVOT_PANEL_Y = ARM_PIVOT_Z - FRAME_Z_OFFSET;         // Pivot Y in panel coords

// =============================================================================
// HYDRAULIC CYLINDER MOUNTING POINTS
// =============================================================================

// Lift cylinder mounting points (arms pivot at rear, cylinders attach forward)
// Optimized for leverage at bottom position (cylinder pushes up/forward from low rear point)

// 1. Define Arm Attachment Point
LIFT_CYL_ARM_OFFSET = ARM_LENGTH * 0.25;     // Attachment point along arm from pivot (proportional)

// 2. Calculate Arm Attachment Position in World Coords at Lowest Angle (Bucket on Ground)
//    Pivot is at [0, ARM_PIVOT_Y, ARM_PIVOT_Z]
_theta_min = -ARM_GROUND_ANGLE;
_attach_y_world = ARM_PIVOT_Y + LIFT_CYL_ARM_OFFSET * cos(_theta_min);
_attach_z_world = ARM_PIVOT_Z + LIFT_CYL_ARM_OFFSET * sin(_theta_min);

// 3. Determine Base Z Height (Constrained by Axle Clearance)
//    Must clear the rear axle (FRAME_Z_OFFSET + WHEEL_DIAMETER/2)
_min_base_z = FRAME_Z_OFFSET + WHEEL_DIAMETER/2 + 80; // 80mm clearance
LIFT_CYL_BASE_Z = _min_base_z;

// 4. Calculate Base Y for Optimal Leverage Angle
//    We want the cylinder to push at an angle ~45-50 degrees from horizontal
//    to be roughly perpendicular to the arm (which is at ~-45 degrees).
//    Slope m = tan(target_angle).
//    Y = Attach_Y - (Attach_Z - Base_Z) / m
_target_cyl_angle = 50; // Degrees
_calc_base_y = _attach_y_world - (_attach_z_world - LIFT_CYL_BASE_Z) / tan(_target_cyl_angle);

// 5. Clamp Y to valid frame range (0 to WHEEL_BASE/2)
LIFT_CYL_BASE_Y = max(50, min(WHEEL_BASE/2, _calc_base_y));

// =============================================================================
// ENGINE CONFIGURATION
// =============================================================================

ENGINE_HP = 25; // Desired horsepower (e.g., 25HP V-Twin)
// Estimate engine weight (kg) based on HP (approximate for small engines)
// Base 30kg + 1.2kg per HP
ENGINE_WEIGHT_KG = 30 + (ENGINE_HP * 1.2); 

// Engine Position (Middle near back)
ENGINE_POS_Y = 400; // Forward from rear of frame
ENGINE_POS_Z = FRAME_Z_OFFSET + 200; // Height above frame bottom

// =============================================================================
// BUCKET DIMENSIONS
// =============================================================================

BOBCAT_QA_WIDTH = 1168;           // 46" standard width
BOBCAT_QA_HEIGHT = 457;           // 18" plate height
BOBCAT_QA_PIN_DIA = 25.4;         // 1" locking pin

BUCKET_WIDTH = 1100;              // Slightly narrower than QA plate
BUCKET_DEPTH = 600;
BUCKET_HEIGHT = 450;

// =============================================================================
// STANDING DECK DIMENSIONS (Legacy - kept for reference)
// =============================================================================

DECK_WIDTH = 700;
DECK_DEPTH = 400;
DECK_HEIGHT = 250;  // Height above ground

// =============================================================================
// FOLDING PLATFORM PARAMETERS
// =============================================================================
// 5-component folding platform: 1x deck plate + 2x pivot brackets + 2x angle arms
// Folds from stowed (vertical against rear) to deployed (horizontal)

// --- Configurable dimensions ---
PLATFORM_THICKNESS = PLATE_1_4_INCH;  // Deck plate thickness
PLATFORM_DEPTH = 400;           // Front-to-back depth of deck (mm)
PLATFORM_ARM_LENGTH = 350;      // Length of angle iron arms (mm)
PLATFORM_PIVOT_HEIGHT = 250;    // Height of pivot point / deck surface when deployed (mm)

// --- Derived dimensions (calculated from frame geometry) ---
// Platform width fits between inner side panels with clearance
PLATFORM_GAP = 3.175; // 1/8 inch gap between deck and pivot plate
// Inner wall inner face X position
_inner_wall_inner_face_x = TRACK_WIDTH/2 - SANDWICH_SPACING/2 - PANEL_THICKNESS;
// Platform width calculated to leave GAP between deck and pivot plate face
// Pivot plate is PLATFORM_THICKNESS thick and sits against inner wall
PLATFORM_WIDTH = 2 * (_inner_wall_inner_face_x - PLATFORM_THICKNESS - PLATFORM_GAP);

// Inner side panel X position (from center of machine)
// Inner panel inner face is at: TRACK_WIDTH/2 - SANDWICH_SPACING/2
_INNER_PANEL_X = TRACK_WIDTH/2 - SANDWICH_SPACING/2;  // = 450 - 60 = 390mm

// Pivot X position - at the inner side panel (pivot pin goes through panel)
// Inset slightly from panel edge for structural strength
PLATFORM_PIVOT_X_INSET = 0;  // Distance inward from inner panel face (mm)
PLATFORM_PIVOT_X = _INNER_PANEL_X - PLATFORM_PIVOT_X_INSET;  // X position of pivot from center (~390mm)

// --- Pivot bracket dimensions ---
PLATFORM_BRACKET_LENGTH = 150;  // Length of pivot bracket plate (mm)
PLATFORM_BRACKET_WIDTH = 100;   // Width of pivot bracket plate (mm)

// --- Hardware dimensions ---
PLATFORM_PIVOT_PIN_DIA = BOLT_DIA_1;    // 1" (25.4mm) pivot pin diameter
PLATFORM_LOCK_PIN_DIA = 9.525;          // 3/8" (9.525mm) cotter/lock pin diameter
PLATFORM_BOLT_DIA = BOLT_DIA_1_2;       // 1/2" (12.7mm) mounting bolts
PLATFORM_BOLT_CLEARANCE = 2;            // Clearance for bolt holes (mm)
PLATFORM_LOCK_OFFSET = 30;              // Distance from pivot center to lock pin hole (mm)

// --- Anti-slip pattern ---
PLATFORM_ANTISLIP_HOLE_DIA = 15;   // Diameter of anti-slip holes (mm)
PLATFORM_ANTISLIP_SPACING = 80;    // Spacing between anti-slip holes (mm)
PLATFORM_ANTISLIP_EDGE_MARGIN = 60; // Margin from edge to first anti-slip hole (mm)

// --- Angle iron specification ---
PLATFORM_ANGLE_SIZE = ANGLE_2X2_1_4;  // 2"x2" x 1/4" angle iron [50.8, 6.35]
PLATFORM_ANGLE_LEG = PLATFORM_ANGLE_SIZE[0];      // Angle iron leg size (50.8mm)
PLATFORM_ANGLE_THICK = PLATFORM_ANGLE_SIZE[1];    // Angle iron thickness (6.35mm)

// --- Bolt hole positions on angle iron (relative to corner) ---
PLATFORM_ANGLE_BOLT_OFFSET = PLATFORM_ANGLE_LEG * 0.6;  // Bolt holes centered on leg

// =============================================================================
// FOLDING PLATFORM PARAMETER VALIDATION
// =============================================================================
// Assert statements ensure parameter relationships are valid

// Arm must be long enough to reach from pivot to deck attachment
assert(PLATFORM_ARM_LENGTH >= PLATFORM_DEPTH/2, 
    "PLATFORM_ARM_LENGTH must be at least half of PLATFORM_DEPTH");

// Pivot X must be inside the inner frame walls
_inner_wall_x = TRACK_WIDTH/2 - SANDWICH_SPACING/2;
assert(PLATFORM_PIVOT_X < _inner_wall_x, 
    "PLATFORM_PIVOT_X must be inside inner frame walls");

// Pivot must be above frame bottom
assert(PLATFORM_PIVOT_HEIGHT > FRAME_Z_OFFSET, 
    "PLATFORM_PIVOT_HEIGHT must be above FRAME_Z_OFFSET (ground clearance)");

// Platform must fit within frame width
assert(PLATFORM_WIDTH <= TRACK_WIDTH - SANDWICH_SPACING, 
    "PLATFORM_WIDTH too wide for frame");

// Bracket must be long enough to fit angle iron attachment holes
assert(PLATFORM_BRACKET_LENGTH > PLATFORM_ANGLE_LEG * 2, 
    "PLATFORM_BRACKET_LENGTH must accommodate angle iron bolt pattern");

// Pivot pin must be larger than lock pin
assert(PLATFORM_PIVOT_PIN_DIA > PLATFORM_LOCK_PIN_DIA, 
    "PLATFORM_PIVOT_PIN_DIA must be larger than PLATFORM_LOCK_PIN_DIA");

// Stowed lock hole must be within rear crossmember height
_crossmember_top_z = FRAME_Z_OFFSET + MACHINE_HEIGHT * 0.55;
assert(PLATFORM_PIVOT_HEIGHT + PLATFORM_ARM_LENGTH < _crossmember_top_z, 
    "Stowed position lock hole must be within rear crossmember height");
