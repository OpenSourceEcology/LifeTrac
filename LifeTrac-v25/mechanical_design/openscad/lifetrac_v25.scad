// lifetrac_v25.scad
// Main LifeTrac v25 Design File
// Open Source Ecology - Compact Remote Controlled Utility Loader
// License: GPL v3

// Include parameters
include <lifetrac_v25_params.scad>

// Import modules
use <modules/plate_steel.scad>
use <modules/structural_steel.scad>
use <modules/fasteners.scad>
use <modules/hydraulics.scad>
use <modules/wheels.scad>
use <modules/loader_arm_v2.scad>

// Import individual part files
use <parts/side_panel.scad>
use <parts/rear_crossmember.scad>
use <parts/platform_deck.scad>
use <parts/platform_pivot_bracket.scad>
use <parts/platform_angle_arm.scad>
use <parts/wheel_mount.scad>
use <parts/cylinder_lug.scad>
use <parts/bucket_bottom.scad>
use <parts/bucket_side.scad>

// =============================================================================
// GLOBAL PARAMETERS
// =============================================================================

$fn = 32;

// Animation parameter (0 to 1)
animation_time = $t;

// Display options
show_wheels = true;
show_hydraulics = true;
show_frame = true;
show_loader_arms = true;
show_bucket = true;
show_folding_platform = true;  // Folding standing platform
platform_fold_angle = 90;      // 0 = stowed (vertical), 90 = deployed (horizontal)
exploded_view = false;
explode_distance = exploded_view ? 200 : 0;

// Drive configuration
all_wheel_drive = true;  // true = 4 wheels powered, false = 2 front wheels only

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
// LOADER ARM DIMENSIONS - PARAMETRIC CALCULATION
// =============================================================================

// Design constraints
BUCKET_GROUND_CLEARANCE = 0;      // Bucket bottom at ground level when lowered
BUCKET_FRONT_CLEARANCE = 100;     // 100mm (~4") clearance between bucket back and front of machine

// Arm pivot point (at top rear of side panels - tall end)
// ARM_PIVOT_Y, ARM_PIVOT_Z defined in lifetrac_v25_params.scad

// Calculate required arm length using trigonometry
// Logic moved to lifetrac_v25_params.scad

// Echo calculated values for debugging
echo("=== ARM GEOMETRY CALCULATION ===");
echo("ARM_PIVOT_Z =", ARM_PIVOT_Z);
echo("MIN_HORIZONTAL_REACH =", MIN_HORIZONTAL_REACH);
echo("CALCULATED ARM_LENGTH =", ARM_LENGTH);
echo("ARM_GROUND_ANGLE (degrees below horizontal) =", ARM_GROUND_ANGLE);
// ARM_HORIZONTAL_REACH not defined in params, calculate locally if needed
ARM_HORIZONTAL_REACH = ARM_LENGTH * cos(ARM_GROUND_ANGLE);
echo("ACTUAL_HORIZONTAL_REACH =", ARM_HORIZONTAL_REACH);
echo("BUCKET_TIP_Y (at ground) =", ARM_PIVOT_Y + ARM_HORIZONTAL_REACH);
echo("CLEARANCE_FROM_FRONT_WHEELS =", ARM_PIVOT_Y + ARM_HORIZONTAL_REACH - WHEEL_BASE);

// ARM_TUBE_SIZE, ARM_SPACING defined in lifetrac_v25_params.scad

// Looping animation: 0->0.5 = arms go up, 0.5->1 = arms come back down
// This creates a smooth loop where the animation returns to start position
animation_phase = animation_time < 0.5 ? (animation_time * 2) : (2 - animation_time * 2);

// Default arm position: lowered to ground (animation_phase=0)
// ARM_GROUND_ANGLE is the angle below horizontal needed to reach ground
ARM_LIFT_ANGLE = -ARM_GROUND_ANGLE + ARM_V2_OFFSET_ANGLE + (animation_phase * (60 + ARM_GROUND_ANGLE)); // Animate from ground to raised and back

// Cross beam positions along arm length (from pivot)
// First cross beam positioned for bucket cylinder attachment
// CROSS_BEAM_1_POS = ARM_LENGTH * 0.65;   // Aligned with bucket cylinder arm mount position
CROSS_BEAM_2_POS = ARM_LENGTH - 200;    // Second cross beam near bucket

// Cross beam size (for cutout calculations)
CROSS_BEAM_SIZE = TUBE_2X2_1_4[0];    // 2"x2" tube size
CROSS_BEAM_CLEARANCE = 15;             // Extra clearance around cross beam in cutout

// Arm rotation range for cutout calculations
// ARM_MIN_ANGLE and ARM_MAX_ANGLE are defined in lifetrac_v25_params.scad
// ARM_MIN_ANGLE = -ARM_GROUND_ANGLE;     // Lowest position (at ground)
// ARM_MAX_ANGLE = 60;                     // Maximum raised position

// =============================================================================
// CROSS BEAM CUTOUT GEOMETRY FUNCTIONS
// =============================================================================

// Calculate cross beam center position in panel local coordinates
// Panel local: X = world Y (forward), Y = world Z (up)
// Cross beams rotate with arm around pivot at (ARM_PIVOT_Y, ARM_PIVOT_Z) in world
// In panel local coords, pivot is at (ARM_PIVOT_Y, ARM_PIVOT_Z - FRAME_Z_OFFSET)
function cross_beam_local_pos(beam_dist, angle) = 
    let(
        // Position relative to pivot in arm local coords
        arm_local_y = beam_dist * cos(angle),
        arm_local_z = beam_dist * sin(angle),
        // Transform to panel local coords (panel origin at world Y=0, Z=FRAME_Z_OFFSET)
        panel_x = ARM_PIVOT_Y + arm_local_y,
        panel_y = (ARM_PIVOT_Z - FRAME_Z_OFFSET) + arm_local_z
    )
    [panel_x, panel_y];

// Get Y range (world Z) for a cross beam at given distance over full arm rotation
function cross_beam_z_range(beam_dist) =
    let(
        pos_low = cross_beam_local_pos(beam_dist, ARM_MIN_ANGLE),
        pos_high = cross_beam_local_pos(beam_dist, ARM_MAX_ANGLE)
    )
    [min(pos_low[1], pos_high[1]), max(pos_low[1], pos_high[1])];

// Calculate arc slot parameters for a cross beam
// Returns [center_x, center_y, radius, start_angle, end_angle] in panel local coords
PIVOT_PANEL_X = ARM_PIVOT_Y;                          // Pivot X in panel coords
PIVOT_PANEL_Y = ARM_PIVOT_Z - FRAME_Z_OFFSET;         // Pivot Y in panel coords

// Debug output for cross beam cutouts
echo("=== CROSS BEAM CUTOUT GEOMETRY ===");
echo("PIVOT_PANEL_X (Y in world) =", PIVOT_PANEL_X);
echo("PIVOT_PANEL_Y (Z in world, relative to frame) =", PIVOT_PANEL_Y);
echo("CROSS_BEAM_1_POS (radius) =", CROSS_BEAM_1_POS);
echo("CROSS_BEAM_2_POS (radius) =", CROSS_BEAM_2_POS);
echo("ARM_MIN_ANGLE =", ARM_MIN_ANGLE);
echo("ARM_MAX_ANGLE =", ARM_MAX_ANGLE);
echo("SLOT_WIDTH =", CROSS_BEAM_SIZE + 2 * CROSS_BEAM_CLEARANCE);

// =============================================================================
// BUCKET DIMENSIONS (Bobcat Standard Quick Attach)
// =============================================================================

// Bobcat quick attach standard dimensions (approximate)
BOBCAT_QA_WIDTH = 1168;           // 46" standard width
BOBCAT_QA_HEIGHT = 457;           // 18" plate height
BOBCAT_QA_HOOK_HEIGHT = 203;      // 8" from bottom to hook bar
BOBCAT_QA_WEDGE_HEIGHT = 76;      // 3" wedge pocket height
BOBCAT_QA_PIN_DIA = 25.4;         // 1" locking pin

BUCKET_WIDTH = 1100;              // Slightly narrower than QA plate
BUCKET_DEPTH = 600;
BUCKET_HEIGHT = 450;

// Bucket tilt angle calculation:
// ANIMATION: Sweep from Max Curl (at bottom) to Max Dump (at top)
// We calculate the Target ABSOLUTE angle based on animation phase
// Then subtract the current Arm Angle to get the Relative Tilt Angle needed for the transform
// MD: Default view ($t=0) overrides to 0 (flat) as per user request.
_anim_abs_angle = ($t == 0) ? 0 : (BUCKET_ABS_CURL_ANGLE + (animation_phase * (BUCKET_ABS_DUMP_ANGLE - BUCKET_ABS_CURL_ANGLE)));
BUCKET_TILT_ANGLE = _anim_abs_angle - ARM_LIFT_ANGLE;

// Legacy definitions for reference (unused in animation now)
BUCKET_GROUND_TILT = -ARM_MIN_ANGLE; 
BUCKET_MAX_CURL = 60;  
BUCKET_MAX_DUMP = -45;

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

// Bucket cylinder mounting points
// Cylinders now attach to the cross beam instead of individual arms
BUCKET_CYL_ARM_POS = CROSS_BEAM_1_POS;       // Mount at cross beam position
BUCKET_CYL_ARM_Z = CROSS_BEAM_SIZE/2 + 30;   // Height above cross beam centerline

// Cylinder X spacing - positioned inward to clear side wall plates
// Inner panels are at TRACK_WIDTH/2 - SANDWICH_SPACING/2 from center
INNER_PANEL_X = TRACK_WIDTH/2 - SANDWICH_SPACING/2;  // 390mm from center
BUCKET_CYL_X_SPACING = INNER_PANEL_X - 100;          // 100mm clearance from inner panels

// Bucket cylinder attaches to QA plate near top
BUCKET_CYL_BUCKET_Y = 100;                   // Mount point on bucket QA plate (forward of pivot)
BUCKET_CYL_BUCKET_Z = BOBCAT_QA_HEIGHT - 60; // Height above pivot (where cylinder attaches at top of QA)

// =============================================================================
// HYDRAULIC CYLINDER PARAMETRIC SIZING
// =============================================================================

// Design parameters for cylinder sizing
CYLINDER_STROKE_MARGIN = 1.15;    // 15% extra stroke for safety margin
CYLINDER_CLOSED_MARGIN = 1.10;    // 10% margin on closed length

// --- LIFT CYLINDER GEOMETRY CALCULATION ---
// The lift cylinder connects:
//   - Base: Frame side wall at (LIFT_CYL_BASE_Y, LIFT_CYL_BASE_Z)
//   - Rod end: Arm at LIFT_CYL_ARM_OFFSET from pivot, below arm centerline

// Arm attachment point in arm local coords (Y along arm, Z perpendicular)
LIFT_CYL_ARM_LOCAL_Y = LIFT_CYL_ARM_OFFSET;
LIFT_CYL_ARM_LOCAL_Z = -ARM_TUBE_SIZE[0]/2 - 30;  // Below arm

// Function to calculate lift cylinder length for a given arm angle
function lift_cyl_length(arm_angle) = 
    let(
        // Arm attachment in world coordinates (pivot at origin for this calc)
        arm_y = LIFT_CYL_ARM_LOCAL_Y * cos(arm_angle) - LIFT_CYL_ARM_LOCAL_Z * sin(arm_angle),
        arm_z = LIFT_CYL_ARM_LOCAL_Y * sin(arm_angle) + LIFT_CYL_ARM_LOCAL_Z * cos(arm_angle),
        // Add pivot offset to get world position
        arm_world_y = ARM_PIVOT_Y + arm_y,
        arm_world_z = ARM_PIVOT_Z + arm_z,
        // Base point (using center X for length calculation)
        base_y = LIFT_CYL_BASE_Y,
        base_z = LIFT_CYL_BASE_Z,
        // Distance
        dy = arm_world_y - base_y,
        dz = arm_world_z - base_z
    )
    sqrt(dy*dy + dz*dz);

// Calculate cylinder lengths at extreme positions
// Use calculated values if possible, otherwise fallback to defaults from params
_lift_cyl_len_min_calc = lift_cyl_length(ARM_MIN_ANGLE);
_lift_cyl_len_max_calc = lift_cyl_length(ARM_MAX_ANGLE);

LIFT_CYL_LEN_MIN = is_undef(_lift_cyl_len_min_calc) ? LIFT_CYL_LEN_MIN_DEF : _lift_cyl_len_min_calc;
LIFT_CYL_LEN_MAX = is_undef(_lift_cyl_len_max_calc) ? (LIFT_CYL_LEN_MIN_DEF + LIFT_CYLINDER_STROKE_DEF) : _lift_cyl_len_max_calc;

// Required stroke = difference in lengths
LIFT_CYL_REQUIRED_STROKE = LIFT_CYL_LEN_MAX - LIFT_CYL_LEN_MIN;

// Add safety margin and round up to standard stroke
LIFT_CYL_STROKE_WITH_MARGIN = ceil(LIFT_CYL_REQUIRED_STROKE * CYLINDER_STROKE_MARGIN);

// Closed length (minimum length minus some margin for mounting hardware)
LIFT_CYL_CLOSED_LENGTH = ceil(LIFT_CYL_LEN_MIN * CYLINDER_CLOSED_MARGIN);

// Standard cylinder strokes (in mm): 150, 200, 250, 300, 350, 400, 450, 500, 600
// Select appropriate stroke
LIFT_CYLINDER_STROKE_CALC = 
    LIFT_CYL_STROKE_WITH_MARGIN <= 150 ? 150 :
    LIFT_CYL_STROKE_WITH_MARGIN <= 200 ? 200 :
    LIFT_CYL_STROKE_WITH_MARGIN <= 250 ? 250 :
    LIFT_CYL_STROKE_WITH_MARGIN <= 300 ? 300 :
    LIFT_CYL_STROKE_WITH_MARGIN <= 350 ? 350 :
    LIFT_CYL_STROKE_WITH_MARGIN <= 400 ? 400 :
    LIFT_CYL_STROKE_WITH_MARGIN <= 450 ? 450 :
    LIFT_CYL_STROKE_WITH_MARGIN <= 500 ? 500 :
    LIFT_CYL_STROKE_WITH_MARGIN <= 600 ? 600 : 
    ceil(LIFT_CYL_STROKE_WITH_MARGIN / 50) * 50;  // Round up to nearest 50

// --- BUCKET CYLINDER GEOMETRY CALCULATION ---
// The bucket cylinder connects:
//   - Base: On cross beam at BUCKET_CYL_ARM_POS from pivot, above cross beam centerline
//   - Rod end: On bucket QA plate at BUCKET_CYL_BUCKET_Y forward, BUCKET_CYL_BUCKET_Z up from bucket pivot

// Bucket tilt range
BUCKET_MIN_TILT = BUCKET_GROUND_TILT;  // Flat on ground (matches arm angle compensation)
BUCKET_MAX_TILT = 90;                   // Fully curled (bucket bottom facing up)

// Function to calculate bucket cylinder length for given arm angle and bucket tilt
// Note: arm_attach is relative to arm pivot, bucket_attach is relative to bucket pivot (at arm tip)
function bucket_cyl_length(arm_angle, bucket_tilt) = 
    let(
        // Cross beam attachment point (in arm local coords, then rotated)
        arm_local_y = CROSS_BEAM_1_POS,
        arm_local_z = CROSS_BEAM_MOUNT_Z_OFFSET,
        arm_y = arm_local_y * cos(arm_angle) - arm_local_z * sin(arm_angle),
        arm_z = arm_local_y * sin(arm_angle) + arm_local_z * cos(arm_angle),
        
        // Bucket attachment (in bucket local, tilt about arm tip, then arm rotation)
        // Must match the translation in bucket_attachment
        // Bucket is at Y = BUCKET_LUG_OFFSET. Cylinder Lug is on back plate.
        // Assuming Cylinder Lug is also rotated to point backwards, Pivot is at Y=0 relative to Arm Tip?
        // Let's assume Cylinder Lug Pivot is at Y = 0 for now (flush with Arm Pivot Y)
        bucket_local_y = 0,
        bucket_local_z = BUCKET_CYL_MOUNT_Z_OFFSET + (BUCKET_HEIGHT - BUCKET_PIVOT_HEIGHT_FROM_BOTTOM),
        // Tilt about bucket pivot (at arm tip)
        bucket_tilted_y = bucket_local_y * cos(bucket_tilt) - bucket_local_z * sin(bucket_tilt),
        bucket_tilted_z = bucket_local_y * sin(bucket_tilt) + bucket_local_z * cos(bucket_tilt),
        // Add arm length to get position relative to arm pivot
        bucket_arm_y = ARM_TIP_X + bucket_tilted_y,
        bucket_arm_z = ARM_TIP_Z + bucket_tilted_z,
        // Rotate by arm angle
        bucket_y = bucket_arm_y * cos(arm_angle) - bucket_arm_z * sin(arm_angle),
        bucket_z = bucket_arm_y * sin(arm_angle) + bucket_arm_z * cos(arm_angle),
        
        // Distance between attachment points
        dy = bucket_y - arm_y,
        dz = bucket_z - arm_z
    )
    sqrt(dy*dy + dz*dz);

// Calculate bucket cylinder lengths at extreme positions
// Check multiple combinations to find true min/max
_bc_len_1 = bucket_cyl_length(ARM_MIN_ANGLE, BUCKET_MIN_TILT);
_bc_len_2 = bucket_cyl_length(ARM_MIN_ANGLE, BUCKET_MAX_TILT);
_bc_len_3 = bucket_cyl_length(ARM_MAX_ANGLE, BUCKET_MIN_TILT);
_bc_len_4 = bucket_cyl_length(ARM_MAX_ANGLE, BUCKET_MAX_TILT);
_bc_len_5 = bucket_cyl_length(0, BUCKET_MIN_TILT);  // Arm horizontal
_bc_len_6 = bucket_cyl_length(0, BUCKET_MAX_TILT);

// Safe min/max functions that handle undef
function safe_min(v1, v2, v3, v4, v5, v6) = 
    min(
        is_undef(v1)?9999:v1, is_undef(v2)?9999:v2, is_undef(v3)?9999:v3, 
        is_undef(v4)?9999:v4, is_undef(v5)?9999:v5, is_undef(v6)?9999:v6
    );

function safe_max(v1, v2, v3, v4, v5, v6) = 
    max(
        is_undef(v1)?0:v1, is_undef(v2)?0:v2, is_undef(v3)?0:v3, 
        is_undef(v4)?0:v4, is_undef(v5)?0:v5, is_undef(v6)?0:v6
    );

_bc_min_calc = safe_min(_bc_len_1, _bc_len_2, _bc_len_3, _bc_len_4, _bc_len_5, _bc_len_6);
_bc_max_calc = safe_max(_bc_len_1, _bc_len_2, _bc_len_3, _bc_len_4, _bc_len_5, _bc_len_6);

// Debug output to trace undefined values
echo("DEBUG: _bc_min_calc", _bc_min_calc);
echo("DEBUG: _bc_max_calc", _bc_max_calc);

BUCKET_CYL_LEN_MIN = (is_undef(_bc_min_calc) || _bc_min_calc == 9999) ? BUCKET_CYL_LEN_MIN_DEF : _bc_min_calc;
BUCKET_CYL_LEN_MAX = (is_undef(_bc_max_calc) || _bc_max_calc == 0) ? (BUCKET_CYL_LEN_MIN_DEF + BUCKET_CYLINDER_STROKE_DEF) : _bc_max_calc;

// Required stroke
BUCKET_CYL_REQUIRED_STROKE = (is_undef(BUCKET_CYL_LEN_MAX) || is_undef(BUCKET_CYL_LEN_MIN)) ? 0 : (BUCKET_CYL_LEN_MAX - BUCKET_CYL_LEN_MIN);
BUCKET_CYL_STROKE_WITH_MARGIN = ceil(BUCKET_CYL_REQUIRED_STROKE * CYLINDER_STROKE_MARGIN);
BUCKET_CYL_CLOSED_LENGTH = ceil(BUCKET_CYL_LEN_MIN * CYLINDER_CLOSED_MARGIN);

echo("=== BUCKET CYLINDER FULL RANGE ===");
echo("MIN LENGTH =", BUCKET_CYL_LEN_MIN);
echo("MAX LENGTH =", BUCKET_CYL_LEN_MAX);
echo("REQUIRED STROKE (Full Range) =", BUCKET_CYL_REQUIRED_STROKE);
echo("STROKE WITH MARGIN =", BUCKET_CYL_STROKE_WITH_MARGIN);

// Select standard stroke
BUCKET_CYLINDER_STROKE_CALC = 
    is_undef(BUCKET_CYL_STROKE_WITH_MARGIN) ? 200 :
    BUCKET_CYL_STROKE_WITH_MARGIN <= 150 ? 150 :
    BUCKET_CYL_STROKE_WITH_MARGIN <= 200 ? 200 :
    BUCKET_CYL_STROKE_WITH_MARGIN <= 250 ? 250 :
    BUCKET_CYL_STROKE_WITH_MARGIN <= 300 ? 300 :
    BUCKET_CYL_STROKE_WITH_MARGIN <= 350 ? 350 :
    BUCKET_CYL_STROKE_WITH_MARGIN <= 400 ? 400 :
    BUCKET_CYL_STROKE_WITH_MARGIN <= 450 ? 450 :
    BUCKET_CYL_STROKE_WITH_MARGIN <= 500 ? 500 :
    ceil(BUCKET_CYL_STROKE_WITH_MARGIN / 50) * 50;

// --- FINAL CYLINDER SPECIFICATIONS ---
// Use calculated values (override manual values)
LIFT_CYLINDER_BORE = 63.5;        // 2.5" bore (force capacity)
LIFT_CYLINDER_ROD = 38.1;         // 1.5" rod
// Use calculated stroke if available, otherwise keep default from params
LIFT_CYLINDER_STROKE = is_undef(LIFT_CYLINDER_STROKE_CALC) ? LIFT_CYLINDER_STROKE_DEF : LIFT_CYLINDER_STROKE_CALC;

BUCKET_CYLINDER_BORE = 50.8;      // 2" bore
BUCKET_CYLINDER_ROD = 31.75;      // 1.25" rod
// Use calculated stroke if available, otherwise keep default from params
BUCKET_CYLINDER_STROKE = is_undef(BUCKET_CYLINDER_STROKE_CALC) ? BUCKET_CYLINDER_STROKE_DEF : BUCKET_CYLINDER_STROKE_CALC;

// Debug output
echo("=== LIFT CYLINDER SIZING ===");
echo("Arm angle range:", ARM_MIN_ANGLE, "to", ARM_MAX_ANGLE, "degrees");
echo("Cylinder length at min angle (retracted):", LIFT_CYL_LEN_MIN, "mm");
echo("Cylinder length at max angle (extended):", LIFT_CYL_LEN_MAX, "mm");
echo("Required stroke:", LIFT_CYL_REQUIRED_STROKE, "mm");
echo("Stroke with margin:", LIFT_CYL_STROKE_WITH_MARGIN, "mm");
echo("SELECTED LIFT CYLINDER STROKE:", LIFT_CYLINDER_STROKE, "mm");
echo("Recommended closed length:", LIFT_CYL_CLOSED_LENGTH, "mm");

echo("=== BUCKET CYLINDER SIZING ===");
echo("Bucket tilt range:", BUCKET_MIN_TILT, "to", BUCKET_MAX_TILT, "degrees");
echo("Cylinder length range:", BUCKET_CYL_LEN_MIN, "to", BUCKET_CYL_LEN_MAX, "mm");
echo("Required stroke:", BUCKET_CYL_REQUIRED_STROKE, "mm");
echo("Stroke with margin:", BUCKET_CYL_STROKE_WITH_MARGIN, "mm");
echo("SELECTED BUCKET CYLINDER STROKE:", BUCKET_CYLINDER_STROKE, "mm");
echo("Recommended closed length:", BUCKET_CYL_CLOSED_LENGTH, "mm");

// =============================================================================
// LIFTING CAPACITY CALCULATIONS
// =============================================================================
// Calculate theoretical lifting capacity based on hydraulic cylinder force
// and lever arm geometry at various arm positions

// --- HYDRAULIC SYSTEM PARAMETERS ---
HYDRAULIC_PRESSURE_PSI = 3000;         // Operating pressure (PSI)
HYDRAULIC_PRESSURE_MPA = HYDRAULIC_PRESSURE_PSI * 0.00689476;  // Convert to MPa

// Lift cylinder force calculation
LIFT_CYL_AREA_MM2 = PI * pow(LIFT_CYLINDER_BORE/2, 2);  // Piston area in mm²
LIFT_CYL_AREA_IN2 = LIFT_CYL_AREA_MM2 / 645.16;          // Convert to in²
LIFT_CYL_FORCE_LBF = HYDRAULIC_PRESSURE_PSI * LIFT_CYL_AREA_IN2;  // Force in lbf
LIFT_CYL_FORCE_N = LIFT_CYL_FORCE_LBF * 4.44822;         // Convert to Newtons
LIFT_CYL_FORCE_KG = LIFT_CYL_FORCE_N / 9.81;             // Equivalent mass (kg)

// Two lift cylinders total
TOTAL_LIFT_FORCE_N = 2 * LIFT_CYL_FORCE_N;
TOTAL_LIFT_FORCE_LBF = 2 * LIFT_CYL_FORCE_LBF;

echo("=== HYDRAULIC FORCE CALCULATION ===");
echo("Operating pressure:", HYDRAULIC_PRESSURE_PSI, "PSI (", HYDRAULIC_PRESSURE_MPA, "MPa)");
echo("Lift cylinder bore:", LIFT_CYLINDER_BORE, "mm (", LIFT_CYLINDER_BORE/25.4, "inches)");
echo("Lift cylinder piston area:", LIFT_CYL_AREA_IN2, "in²");
echo("Force per lift cylinder:", LIFT_CYL_FORCE_LBF, "lbf (", LIFT_CYL_FORCE_N, "N)");
echo("Total lift cylinder force (2 cyls):", TOTAL_LIFT_FORCE_LBF, "lbf (", TOTAL_LIFT_FORCE_N, "N)");

// --- LEVER ARM GEOMETRY ANALYSIS ---
// At any arm position, the lifting capacity depends on:
// 1. Cylinder force
// 2. Moment arm of cylinder about arm pivot
// 3. Moment arm of load about arm pivot

// Function to calculate cylinder moment arm about pivot for given arm angle
// Returns perpendicular distance from pivot to cylinder force line
function lift_cyl_moment_arm(arm_angle) = 
    let(
        // Arm attachment in world coordinates (pivot at origin)
        arm_y = LIFT_CYL_ARM_LOCAL_Y * cos(arm_angle) - LIFT_CYL_ARM_LOCAL_Z * sin(arm_angle),
        arm_z = LIFT_CYL_ARM_LOCAL_Y * sin(arm_angle) + LIFT_CYL_ARM_LOCAL_Z * cos(arm_angle),
        // Add pivot offset to get world position
        arm_world_y = ARM_PIVOT_Y + arm_y,
        arm_world_z = ARM_PIVOT_Z + arm_z,
        // Base point
        base_y = LIFT_CYL_BASE_Y,
        base_z = LIFT_CYL_BASE_Z,
        // Vector from base to arm attachment
        dy = arm_world_y - base_y,
        dz = arm_world_z - base_z,
        cyl_length = sqrt(dy*dy + dz*dz),
        // Unit vector along cylinder
        uy = dy / cyl_length,
        uz = dz / cyl_length,
        // Vector from pivot to arm attachment point
        py = arm_y,
        pz = arm_z,
        // Cross product magnitude gives moment arm (perpendicular distance)
        // moment_arm = |pivot_to_attach × cylinder_direction|
        moment_arm = abs(py * uz - pz * uy)
    )
    moment_arm;

// Calculate lifting capacity at different arm angles
// Capacity = (Cylinder_Force × Cylinder_Moment_Arm) / Load_Moment_Arm
// Load is applied at bucket (end of arm)

// Load moment arm = horizontal distance from pivot to bucket CG
function load_moment_arm(arm_angle) = ARM_LENGTH * cos(arm_angle);

// Calculate lift capacity in kg at given arm angle
function lift_capacity_kg(arm_angle) = 
    let(
        cyl_moment = lift_cyl_moment_arm(arm_angle),
        load_moment = load_moment_arm(arm_angle),
        // Total moment from two cylinders
        total_cyl_moment_Nm = TOTAL_LIFT_FORCE_N * cyl_moment / 1000,
        // Load capacity
        load_N = (cyl_moment > 0 && load_moment > 0) ? 
                 (TOTAL_LIFT_FORCE_N * cyl_moment) / load_moment : 0,
        load_kg = load_N / 9.81
    )
    load_kg;

// Calculate capacity at key positions
CAPACITY_AT_GROUND = lift_capacity_kg(ARM_MIN_ANGLE);        // Arms down
CAPACITY_AT_HORIZONTAL = lift_capacity_kg(0);                 // Arms level
CAPACITY_AT_45DEG = lift_capacity_kg(45);                     // Arms at 45°
CAPACITY_AT_MAX = lift_capacity_kg(ARM_MAX_ANGLE);            // Arms fully up

// Moment arms at ground position (for reference)
CYL_MOMENT_ARM_GROUND = lift_cyl_moment_arm(ARM_MIN_ANGLE);
LOAD_MOMENT_ARM_GROUND = load_moment_arm(ARM_MIN_ANGLE);

echo("=== LIFTING CAPACITY ANALYSIS ===");
echo("Arm length:", ARM_LENGTH, "mm");
echo("Cylinder attachment distance from pivot:", LIFT_CYL_ARM_OFFSET, "mm");
echo("--- At Ground Position (angle =", ARM_MIN_ANGLE, "°) ---");
echo("  Cylinder moment arm:", CYL_MOMENT_ARM_GROUND, "mm");
echo("  Load moment arm:", LOAD_MOMENT_ARM_GROUND, "mm");
echo("  Mechanical advantage:", CYL_MOMENT_ARM_GROUND / LOAD_MOMENT_ARM_GROUND);
echo("  LIFTING CAPACITY:", CAPACITY_AT_GROUND, "kg (", CAPACITY_AT_GROUND * 2.205, "lbs)");
echo("--- At Horizontal Position (angle = 0°) ---");
echo("  LIFTING CAPACITY:", CAPACITY_AT_HORIZONTAL, "kg (", CAPACITY_AT_HORIZONTAL * 2.205, "lbs)");
echo("--- At 45° Position ---");
echo("  LIFTING CAPACITY:", CAPACITY_AT_45DEG, "kg (", CAPACITY_AT_45DEG * 2.205, "lbs)");
echo("--- At Max Raised Position (angle =", ARM_MAX_ANGLE, "°) ---");
echo("  LIFTING CAPACITY:", CAPACITY_AT_MAX, "kg (", CAPACITY_AT_MAX * 2.205, "lbs)");

// Use minimum capacity as rated capacity (conservative)
RATED_LIFT_CAPACITY_KG = min(CAPACITY_AT_GROUND, CAPACITY_AT_HORIZONTAL);
RATED_LIFT_CAPACITY_LBS = RATED_LIFT_CAPACITY_KG * 2.205;

echo("=== RATED LIFT CAPACITY ===");
echo("Minimum lift capacity:", RATED_LIFT_CAPACITY_KG, "kg (", RATED_LIFT_CAPACITY_LBS, "lbs)");

// =============================================================================
// STRUCTURAL ANALYSIS - MECHANICS OF DEFORMABLE BODIES
// =============================================================================
// Check that arms, pivot pins, and bolts can safely support the rated load

// --- MATERIAL PROPERTIES ---
// A36 Structural Steel (common for HSS tubing)
STEEL_YIELD_STRENGTH_MPA = 250;        // Yield strength (MPa)
STEEL_YIELD_STRENGTH_PSI = 36000;      // Yield strength (PSI)
STEEL_ULTIMATE_STRENGTH_MPA = 400;     // Ultimate tensile strength (MPa)
STEEL_ELASTIC_MODULUS_MPA = 200000;    // Young's modulus (MPa)
STEEL_SHEAR_MODULUS_MPA = 77200;       // Shear modulus (MPa)

// Grade 8 Bolt Properties (for high-strength fasteners)
BOLT_GRADE8_YIELD_PSI = 130000;        // Yield strength
BOLT_GRADE8_TENSILE_PSI = 150000;      // Tensile strength
BOLT_GRADE8_SHEAR_PSI = 90000;         // Shear strength (approx 60% of tensile)

// Safety factors
SAFETY_FACTOR_STATIC = 2.0;            // For static loads
SAFETY_FACTOR_FATIGUE = 3.0;           // For fatigue considerations
SAFETY_FACTOR_BOLT = 2.5;              // For bolted connections

// Design load (apply safety factor)
DESIGN_LOAD_N = RATED_LIFT_CAPACITY_KG * 9.81 * SAFETY_FACTOR_STATIC;
DESIGN_LOAD_LBF = DESIGN_LOAD_N / 4.44822;

echo("=== STRUCTURAL ANALYSIS PARAMETERS ===");
echo("Steel yield strength:", STEEL_YIELD_STRENGTH_MPA, "MPa (", STEEL_YIELD_STRENGTH_PSI, "PSI)");
echo("Safety factor (static):", SAFETY_FACTOR_STATIC);
echo("Design load (with SF):", DESIGN_LOAD_N, "N (", DESIGN_LOAD_LBF, "lbf)");

// --- ARM BENDING STRESS ANALYSIS ---
// Each arm is a 3"x3" x 1/4" wall square tube loaded as a cantilever
// Load applied at tip, supported at pivot

ARM_TUBE_OUTER = ARM_TUBE_SIZE[0];     // 76.2mm outer dimension
ARM_TUBE_WALL = ARM_TUBE_SIZE[1];      // 6.35mm wall thickness
ARM_TUBE_INNER = ARM_TUBE_OUTER - 2 * ARM_TUBE_WALL;

// Section properties of hollow square tube
// Moment of inertia: I = (b^4 - b_inner^4) / 12
ARM_MOMENT_OF_INERTIA = (pow(ARM_TUBE_OUTER, 4) - pow(ARM_TUBE_INNER, 4)) / 12;

// Section modulus: S = I / c, where c = outer dimension / 2
ARM_SECTION_MODULUS = ARM_MOMENT_OF_INERTIA / (ARM_TUBE_OUTER / 2);

// Load per arm (total load divided by 2 arms)
LOAD_PER_ARM_N = DESIGN_LOAD_N / 2;

// Maximum bending moment at pivot (cantilever with tip load)
ARM_MAX_MOMENT_NMM = LOAD_PER_ARM_N * ARM_LENGTH;  // N·mm

// Maximum bending stress: σ = M / S
ARM_BENDING_STRESS_MPA = ARM_MAX_MOMENT_NMM / ARM_SECTION_MODULUS;

// Allowable stress with safety factor
ARM_ALLOWABLE_STRESS_MPA = STEEL_YIELD_STRENGTH_MPA / SAFETY_FACTOR_STATIC;

// Check pass/fail
ARM_STRESS_RATIO = ARM_BENDING_STRESS_MPA / ARM_ALLOWABLE_STRESS_MPA;
ARM_STRESS_OK = ARM_STRESS_RATIO < 1.0;

echo("=== ARM BENDING STRESS ANALYSIS ===");
echo("Arm tube:", ARM_TUBE_OUTER, "x", ARM_TUBE_OUTER, "x", ARM_TUBE_WALL, "mm");
echo("Moment of inertia:", ARM_MOMENT_OF_INERTIA, "mm⁴");
echo("Section modulus:", ARM_SECTION_MODULUS, "mm³");
echo("Load per arm:", LOAD_PER_ARM_N, "N");
echo("Max bending moment at pivot:", ARM_MAX_MOMENT_NMM, "N·mm");
echo("Calculated bending stress:", ARM_BENDING_STRESS_MPA, "MPa");
echo("Allowable stress:", ARM_ALLOWABLE_STRESS_MPA, "MPa");
echo("Stress ratio (should be < 1.0):", ARM_STRESS_RATIO);
echo("ARM STRESS CHECK:", ARM_STRESS_OK ? "PASS ✓" : "FAIL ✗");

// --- ARM DEFLECTION ANALYSIS ---
// Maximum tip deflection of cantilever: δ = PL³ / (3EI)
ARM_TIP_DEFLECTION_MM = (LOAD_PER_ARM_N * pow(ARM_LENGTH, 3)) / 
                         (3 * STEEL_ELASTIC_MODULUS_MPA * ARM_MOMENT_OF_INERTIA);

// Allowable deflection (typically L/180 to L/360 for structural members)
ARM_ALLOWABLE_DEFLECTION = ARM_LENGTH / 180;
ARM_DEFLECTION_OK = ARM_TIP_DEFLECTION_MM < ARM_ALLOWABLE_DEFLECTION;

echo("=== ARM DEFLECTION ANALYSIS ===");
echo("Calculated tip deflection:", ARM_TIP_DEFLECTION_MM, "mm");
echo("Allowable deflection (L/180):", ARM_ALLOWABLE_DEFLECTION, "mm");
echo("ARM DEFLECTION CHECK:", ARM_DEFLECTION_OK ? "PASS ✓" : "FAIL ✗");

// --- PIVOT PIN SHEAR STRESS ANALYSIS ---
// 1.5" pivot pin in double shear (pin passes through both sides of sandwich)

PIVOT_PIN_AREA_MM2 = PI * pow(PIVOT_PIN_DIA/2, 2);  // Cross-sectional area

// Double shear means two shear planes resist the load
PIVOT_SHEAR_AREA_MM2 = 2 * PIVOT_PIN_AREA_MM2;

// Load on each arm pivot (half of total design load, single arm)
PIVOT_LOAD_N = LOAD_PER_ARM_N;

// Shear stress: τ = F / A
PIVOT_SHEAR_STRESS_MPA = PIVOT_LOAD_N / PIVOT_SHEAR_AREA_MM2;

// Allowable shear stress (typically 0.6 × yield for ductile steel)
PIVOT_ALLOWABLE_SHEAR_MPA = 0.6 * STEEL_YIELD_STRENGTH_MPA / SAFETY_FACTOR_STATIC;

PIVOT_SHEAR_RATIO = PIVOT_SHEAR_STRESS_MPA / PIVOT_ALLOWABLE_SHEAR_MPA;
PIVOT_SHEAR_OK = PIVOT_SHEAR_RATIO < 1.0;

echo("=== PIVOT PIN SHEAR ANALYSIS ===");
echo("Pivot pin diameter:", PIVOT_PIN_DIA, "mm (", PIVOT_PIN_DIA/25.4, "inches)");
echo("Shear area (double shear):", PIVOT_SHEAR_AREA_MM2, "mm²");
echo("Load on pivot:", PIVOT_LOAD_N, "N");
echo("Calculated shear stress:", PIVOT_SHEAR_STRESS_MPA, "MPa");
echo("Allowable shear stress:", PIVOT_ALLOWABLE_SHEAR_MPA, "MPa");
echo("Shear ratio (should be < 1.0):", PIVOT_SHEAR_RATIO);
echo("PIVOT PIN SHEAR CHECK:", PIVOT_SHEAR_OK ? "PASS ✓" : "FAIL ✗");

// --- PIVOT PIN BEARING STRESS ANALYSIS ---
// Bearing stress on pivot pin hole in arm tube
// Bearing area = pin diameter × wall thickness × 2 (two walls)

ARM_BEARING_AREA_MM2 = PIVOT_PIN_DIA * ARM_TUBE_WALL * 2;

// Bearing stress
ARM_BEARING_STRESS_MPA = PIVOT_LOAD_N / ARM_BEARING_AREA_MM2;

// Allowable bearing stress (typically 1.5 × yield for confined bearing)
ARM_ALLOWABLE_BEARING_MPA = 1.5 * STEEL_YIELD_STRENGTH_MPA / SAFETY_FACTOR_STATIC;

ARM_BEARING_RATIO = ARM_BEARING_STRESS_MPA / ARM_ALLOWABLE_BEARING_MPA;
ARM_BEARING_OK = ARM_BEARING_RATIO < 1.0;

echo("=== PIVOT BEARING STRESS ANALYSIS ===");
echo("Bearing area in arm tube:", ARM_BEARING_AREA_MM2, "mm²");
echo("Calculated bearing stress:", ARM_BEARING_STRESS_MPA, "MPa");
echo("Allowable bearing stress:", ARM_ALLOWABLE_BEARING_MPA, "MPa");
echo("Bearing ratio (should be < 1.0):", ARM_BEARING_RATIO);
echo("PIVOT BEARING CHECK:", ARM_BEARING_OK ? "PASS ✓" : "FAIL ✗");

// --- CYLINDER CLEVIS PIN ANALYSIS ---
// 1" clevis pins in double shear at cylinder attachments

CLEVIS_PIN_DIA = BOLT_DIA_1;  // 1" = 25.4mm
CLEVIS_PIN_AREA_MM2 = PI * pow(CLEVIS_PIN_DIA/2, 2);
CLEVIS_SHEAR_AREA_MM2 = 2 * CLEVIS_PIN_AREA_MM2;  // Double shear

// Cylinder force on clevis
CLEVIS_LOAD_N = LIFT_CYL_FORCE_N;  // One cylinder

CLEVIS_SHEAR_STRESS_MPA = CLEVIS_LOAD_N / CLEVIS_SHEAR_AREA_MM2;
CLEVIS_ALLOWABLE_SHEAR_MPA = BOLT_GRADE8_SHEAR_PSI * 0.00689476 / SAFETY_FACTOR_BOLT;

CLEVIS_SHEAR_RATIO = CLEVIS_SHEAR_STRESS_MPA / CLEVIS_ALLOWABLE_SHEAR_MPA;
CLEVIS_SHEAR_OK = CLEVIS_SHEAR_RATIO < 1.0;

echo("=== CYLINDER CLEVIS PIN ANALYSIS ===");
echo("Clevis pin diameter:", CLEVIS_PIN_DIA, "mm");
echo("Cylinder force:", CLEVIS_LOAD_N, "N");
echo("Shear stress:", CLEVIS_SHEAR_STRESS_MPA, "MPa");
echo("Allowable shear (Grade 8):", CLEVIS_ALLOWABLE_SHEAR_MPA, "MPa");
echo("Shear ratio:", CLEVIS_SHEAR_RATIO);
echo("CLEVIS PIN CHECK:", CLEVIS_SHEAR_OK ? "PASS ✓" : "FAIL ✗");

// --- SUMMARY OF STRUCTURAL CHECKS ---
ALL_CHECKS_PASS = ARM_STRESS_OK && ARM_DEFLECTION_OK && PIVOT_SHEAR_OK && 
                   ARM_BEARING_OK && CLEVIS_SHEAR_OK;

echo("=== STRUCTURAL ANALYSIS SUMMARY ===");
echo("Rated Lift Capacity:", RATED_LIFT_CAPACITY_KG, "kg (", RATED_LIFT_CAPACITY_LBS, "lbs)");
echo("Design Load (with 2x SF):", DESIGN_LOAD_N / 9.81, "kg");
echo("Arm Bending Stress:", ARM_STRESS_OK ? "PASS" : "FAIL");
echo("Arm Deflection:", ARM_DEFLECTION_OK ? "PASS" : "FAIL");
echo("Pivot Pin Shear:", PIVOT_SHEAR_OK ? "PASS" : "FAIL");
echo("Pivot Bearing:", ARM_BEARING_OK ? "PASS" : "FAIL");
echo("Clevis Pin Shear:", CLEVIS_SHEAR_OK ? "PASS" : "FAIL");
echo("*** ALL STRUCTURAL CHECKS:", ALL_CHECKS_PASS ? "PASS ✓✓✓" : "REVIEW NEEDED ✗");

// =============================================================================
// CROSS BEAM STRESS ANALYSIS
// =============================================================================
// Cross beams span between the two arms and must resist bucket cylinder forces

CROSS_TUBE_OUTER = TUBE_2X2_1_4[0];     // 50.8mm outer
CROSS_TUBE_WALL = TUBE_2X2_1_4[1];      // 6.35mm wall
CROSS_TUBE_INNER = CROSS_TUBE_OUTER - 2 * CROSS_TUBE_WALL;

// Cross beam span (between arm centerlines)
CROSS_BEAM_SPAN = ARM_SPACING;  // 900mm

// Moment of inertia for cross beam
CROSS_MOMENT_OF_INERTIA = (pow(CROSS_TUBE_OUTER, 4) - pow(CROSS_TUBE_INNER, 4)) / 12;
CROSS_SECTION_MODULUS = CROSS_MOMENT_OF_INERTIA / (CROSS_TUBE_OUTER / 2);

// Bucket cylinder forces apply point loads on the cross beam
// Two cylinders create bending in the cross beam
BUCKET_CYL_AREA_MM2 = PI * pow(BUCKET_CYLINDER_BORE/2, 2);
BUCKET_CYL_AREA_IN2 = BUCKET_CYL_AREA_MM2 / 645.16;
BUCKET_CYL_FORCE_LBF = HYDRAULIC_PRESSURE_PSI * BUCKET_CYL_AREA_IN2;
BUCKET_CYL_FORCE_N = BUCKET_CYL_FORCE_LBF * 4.44822;

// For bending analysis, consider worst case: point load at center
// Maximum moment for point load at center: M = PL/4
CROSS_MAX_MOMENT_NMM = BUCKET_CYL_FORCE_N * CROSS_BEAM_SPAN / 4;

// Bending stress in cross beam
CROSS_BENDING_STRESS_MPA = CROSS_MAX_MOMENT_NMM / CROSS_SECTION_MODULUS;
CROSS_ALLOWABLE_STRESS_MPA = STEEL_YIELD_STRENGTH_MPA / SAFETY_FACTOR_STATIC;

CROSS_STRESS_RATIO = CROSS_BENDING_STRESS_MPA / CROSS_ALLOWABLE_STRESS_MPA;
CROSS_STRESS_OK = CROSS_STRESS_RATIO < 1.0;

echo("=== CROSS BEAM STRESS ANALYSIS ===");
echo("Cross beam:", CROSS_TUBE_OUTER, "x", CROSS_TUBE_OUTER, "x", CROSS_TUBE_WALL, "mm");
echo("Span between arms:", CROSS_BEAM_SPAN, "mm");
echo("Section modulus:", CROSS_SECTION_MODULUS, "mm³");
echo("Bucket cylinder force:", BUCKET_CYL_FORCE_N, "N per cylinder");
echo("Max bending moment:", CROSS_MAX_MOMENT_NMM, "N·mm");
echo("Bending stress:", CROSS_BENDING_STRESS_MPA, "MPa");
echo("Allowable stress:", CROSS_ALLOWABLE_STRESS_MPA, "MPa");
echo("Stress ratio:", CROSS_STRESS_RATIO);
echo("CROSS BEAM CHECK:", CROSS_STRESS_OK ? "PASS ✓" : "FAIL ✗");

// =============================================================================
// WELD STRESS ANALYSIS
// =============================================================================
// Analyze critical welds: arm-to-pivot-boss and cross-beam-to-arm connections

// --- ARM PIVOT RING WELDS ---
// Two 3/4" plate rings welded to each side of arm tube at pivot
// Weld must transfer bending moment from arm to pivot assembly

// Fillet weld around tube perimeter
WELD_SIZE_PIVOT = 6.35;                    // 1/4" fillet weld
PIVOT_RING_OUTER = ARM_TUBE_OUTER + 30;    // Ring outer diameter (matches pivot_ring_large)
WELD_LENGTH_PIVOT = 4 * ARM_TUBE_OUTER;    // Weld around square tube perimeter

// Weld throat thickness (0.707 × leg size for fillet weld)
WELD_THROAT_PIVOT = 0.707 * WELD_SIZE_PIVOT;

// Weld area per ring (two rings per arm)
WELD_AREA_PER_RING = WELD_LENGTH_PIVOT * WELD_THROAT_PIVOT;
TOTAL_WELD_AREA_PIVOT = 2 * WELD_AREA_PER_RING;  // Two rings

// Section modulus of weld group (rectangular tube with weld around perimeter)
// For fillet weld around rectangle: S_w ≈ t_throat × d² (approximate)
WELD_SECTION_MODULUS_PIVOT = WELD_THROAT_PIVOT * pow(ARM_TUBE_OUTER, 2);

// Stress in pivot welds from bending moment
WELD_BENDING_STRESS_PIVOT = ARM_MAX_MOMENT_NMM / (2 * WELD_SECTION_MODULUS_PIVOT);

// Allowable weld stress (E70XX electrode: 0.3 × 70ksi = 21ksi = 145 MPa)
WELD_ALLOWABLE_STRESS_MPA = 145 / SAFETY_FACTOR_STATIC;

WELD_PIVOT_RATIO = WELD_BENDING_STRESS_PIVOT / WELD_ALLOWABLE_STRESS_MPA;
WELD_PIVOT_OK = WELD_PIVOT_RATIO < 1.0;

echo("=== PIVOT RING WELD ANALYSIS ===");
echo("Weld size (fillet leg):", WELD_SIZE_PIVOT, "mm");
echo("Weld throat thickness:", WELD_THROAT_PIVOT, "mm");
echo("Total weld length per arm:", 2 * WELD_LENGTH_PIVOT, "mm");
echo("Weld section modulus:", 2 * WELD_SECTION_MODULUS_PIVOT, "mm³");
echo("Weld bending stress:", WELD_BENDING_STRESS_PIVOT, "MPa");
echo("Allowable weld stress:", WELD_ALLOWABLE_STRESS_MPA, "MPa");
echo("Weld stress ratio:", WELD_PIVOT_RATIO);
echo("PIVOT WELD CHECK:", WELD_PIVOT_OK ? "PASS ✓" : "FAIL ✗");

// --- CROSS BEAM WELD ANALYSIS ---
// Cross beam welded to arm tubes

WELD_SIZE_CROSS = 6.35;                    // 1/4" fillet weld
WELD_LENGTH_CROSS = 4 * CROSS_TUBE_OUTER;  // Weld around tube perimeter
WELD_THROAT_CROSS = 0.707 * WELD_SIZE_CROSS;

// Shear force on weld (half of cylinder force per connection)
WELD_SHEAR_FORCE_CROSS = BUCKET_CYL_FORCE_N / 2;

// Weld area at each arm connection
WELD_AREA_CROSS = WELD_LENGTH_CROSS * WELD_THROAT_CROSS;

// Shear stress in weld
WELD_SHEAR_STRESS_CROSS = WELD_SHEAR_FORCE_CROSS / WELD_AREA_CROSS;

WELD_CROSS_RATIO = WELD_SHEAR_STRESS_CROSS / WELD_ALLOWABLE_STRESS_MPA;
WELD_CROSS_OK = WELD_CROSS_RATIO < 1.0;

echo("=== CROSS BEAM WELD ANALYSIS ===");
echo("Weld length per connection:", WELD_LENGTH_CROSS, "mm");
echo("Weld area per connection:", WELD_AREA_CROSS, "mm²");
echo("Shear force on weld:", WELD_SHEAR_FORCE_CROSS, "N");
echo("Weld shear stress:", WELD_SHEAR_STRESS_CROSS, "MPa");
echo("Allowable weld stress:", WELD_ALLOWABLE_STRESS_MPA, "MPa");
echo("Weld stress ratio:", WELD_CROSS_RATIO);
echo("CROSS BEAM WELD CHECK:", WELD_CROSS_OK ? "PASS ✓" : "FAIL ✗");

// =============================================================================
// UPDATED STRUCTURAL SUMMARY
// =============================================================================

ALL_CHECKS_PASS_FULL = ARM_STRESS_OK && ARM_DEFLECTION_OK && PIVOT_SHEAR_OK && 
                        ARM_BEARING_OK && CLEVIS_SHEAR_OK && CROSS_STRESS_OK &&
                        WELD_PIVOT_OK && WELD_CROSS_OK;

echo("========================================");
echo("    COMPLETE STRUCTURAL ANALYSIS SUMMARY    ");
echo("========================================");
echo("RATED LIFT CAPACITY:", RATED_LIFT_CAPACITY_KG, "kg (", RATED_LIFT_CAPACITY_LBS, "lbs)");
echo("");
echo("Component              | Stress Ratio | Status");
echo("---------------------- | ------------ | ------");
echo("Arm Bending            |", ARM_STRESS_RATIO, "|", ARM_STRESS_OK ? "PASS" : "FAIL");
echo("Arm Deflection         |", ARM_TIP_DEFLECTION_MM / ARM_ALLOWABLE_DEFLECTION, "|", ARM_DEFLECTION_OK ? "PASS" : "FAIL");
echo("Pivot Pin Shear        |", PIVOT_SHEAR_RATIO, "|", PIVOT_SHEAR_OK ? "PASS" : "FAIL");
echo("Pivot Bearing          |", ARM_BEARING_RATIO, "|", ARM_BEARING_OK ? "PASS" : "FAIL");
echo("Clevis Pin Shear       |", CLEVIS_SHEAR_RATIO, "|", CLEVIS_SHEAR_OK ? "PASS" : "FAIL");
echo("Cross Beam Bending     |", CROSS_STRESS_RATIO, "|", CROSS_STRESS_OK ? "PASS" : "FAIL");
echo("Pivot Ring Welds       |", WELD_PIVOT_RATIO, "|", WELD_PIVOT_OK ? "PASS" : "FAIL");
echo("Cross Beam Welds       |", WELD_CROSS_RATIO, "|", WELD_CROSS_OK ? "PASS" : "FAIL");
echo("");
echo("*** OVERALL STRUCTURAL ASSESSMENT:", ALL_CHECKS_PASS_FULL ? "ALL CHECKS PASS ✓✓✓" : "REVIEW REQUIRED ✗");
echo("========================================");

// =============================================================================
// STANDING DECK DIMENSIONS (Legacy - kept for reference)
// =============================================================================

DECK_WIDTH = 700;
DECK_DEPTH = 400;
DECK_HEIGHT = 250;  // Height above ground

// =============================================================================
// ANGLE IRON CONSTANTS
// =============================================================================

ANGLE_2X2_1_4 = [50.8, 6.35];   // 2"x2" x 1/4" angle iron [leg_size, thickness]
ANGLE_3X3_1_4 = [76.2, 6.35];   // 3"x3" x 1/4" angle iron
ANGLE_4X4_1_4 = [101.6, 6.35];  // 4"x4" x 1/4" angle iron

// =============================================================================
// FOLDING PLATFORM PARAMETERS
// =============================================================================
// Moved to lifetrac_v25_params.scad

// =============================================================================
// FOLDING PLATFORM PARAMETER VALIDATION
// =============================================================================
// Moved to lifetrac_v25_params.scad



// =============================================================================
// VECTOR MATH HELPERS
// =============================================================================

function vec_len(v) = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
function vec_norm(v) = v / vec_len(v);
function vec_cross(a, b) = [
    a[1]*b[2] - a[2]*b[1],
    a[2]*b[0] - a[0]*b[2],
    a[0]*b[1] - a[1]*b[0]
];

// Rotate a vector about X axis by angle (degrees)
function rot_x(v, ang) = [
    v[0],
    v[1]*cos(ang) - v[2]*sin(ang),
    v[1]*sin(ang) + v[2]*cos(ang)
];

// Calculate cylinder orientation from base to target point
module oriented_cylinder(base_pt, target_pt, bore, rod, stroke, extension) {
    dir = target_pt - base_pt;
    len = vec_len(dir);
    dir_norm = vec_norm(dir);
    
    // Calculate rotation to align Z axis with direction
    axis = vec_cross([0, 0, 1], dir_norm);
    angle = acos(dir_norm[2]);
    
    // Clevis pin dimensions
    clevis_pin_dia = rod * 0.6;  // Pin diameter based on rod size
    clevis_pin_length = bore * 1.8;  // Pin spans clevis width
    nut_h = nut_height(clevis_pin_dia);
    
    translate(base_pt)
    rotate(a = angle, v = axis) {
        // Scale cylinder to exactly span between mounts using total_length
        hydraulic_cylinder(bore, rod, stroke, false, extension, "clevis", len);
        
        // Base clevis pin with nuts (pin axis along X to match lug holes)
        translate([0, 0, 0])
        rotate([0, 90, 0]) {
            // Pin
            color("Silver")
            cylinder(d=clevis_pin_dia, h=clevis_pin_length, center=true, $fn=24);
            // Nut on one end
            translate([0, 0, clevis_pin_length/2 - nut_h/2])
            hex_nut(clevis_pin_dia);
            // Nut on other end
            translate([0, 0, -clevis_pin_length/2 + nut_h/2])
            hex_nut(clevis_pin_dia);
        }
    }
    
    // Rod end clevis pin at target point
    rod_pin_dia = clevis_pin_dia * 0.8;
    rod_pin_length = rod * 3;
    rod_nut_h = nut_height(rod_pin_dia);
    
    translate(target_pt)
    rotate(a = angle, v = axis)
    rotate([0, 90, 0]) {  // Pin axis along X to match lug holes
        // Pin
        color("Silver")
        cylinder(d=rod_pin_dia, h=rod_pin_length, center=true, $fn=24);
        // Nuts
        translate([0, 0, rod_pin_length/2 - rod_nut_h/2])
        hex_nut(rod_pin_dia);
        translate([0, 0, -rod_pin_length/2 + rod_nut_h/2])
        hex_nut(rod_pin_dia);
    }
}

// =============================================================================
// BOLT HOLE MODULE
// =============================================================================

module bolt_hole(diameter, depth) {
    cylinder(d=diameter, h=depth+2, center=true, $fn=24);
}

// =============================================================================
// STANDARDIZED PART MODULES
// =============================================================================

// TYPE A BRACKET: U-Channel Lug made from square tubing
// Made by torch-cutting top and bottom of square tube to create U-shape
// Parameters: tube_size = source tube dimensions [size, wall], length, hole_dia
module u_channel_lug(tube_size, length, hole_dia) {
    size = tube_size[0];
    wall = tube_size[1];
    rad = 12.7; // 1/2 inch radius
    
    color("DarkSlateGray")
    difference() {
        intersection() {
            // 1. The U-channel profile (Rounded in X-Z plane)
            rotate([90, 0, 0])
            linear_extrude(height=length, center=true)
            difference() {
                // Outer rounded square
                hull() {
                    translate([-(size/2 - rad), -(size/2 - rad)]) circle(r=rad);
                    translate([(size/2 - rad), -(size/2 - rad)]) circle(r=rad);
                    translate([(size/2 - rad), (size/2 - rad)]) circle(r=rad);
                    translate([-(size/2 - rad), (size/2 - rad)]) circle(r=rad);
                }
                // Inner rounded square
                offset(r=-wall)
                hull() {
                    translate([-(size/2 - rad), -(size/2 - rad)]) circle(r=rad);
                    translate([(size/2 - rad), -(size/2 - rad)]) circle(r=rad);
                    translate([(size/2 - rad), (size/2 - rad)]) circle(r=rad);
                    translate([-(size/2 - rad), (size/2 - rad)]) circle(r=rad);
                }
                // Cut top to make U (remove top wall)
                // Top wall is at +Z (in 2D profile, +Y)
                // We want to remove everything above size/2 - wall
                translate([-size, size/2 - wall*1.5]) square([size*2, size]);
            }
            
            // 2. The Side Profile (Rounded in Y-Z plane)
            // We want to round the top corners of the side walls.
            rotate([0, 90, 0])
            linear_extrude(height=size+10, center=true)
            hull() {
                // Bottom corners
                translate([-length/2 + rad, -size/2 + rad]) circle(r=rad);
                translate([length/2 - rad, -size/2 + rad]) circle(r=rad);
                // Top corners
                translate([-length/2 + rad, size/2 - rad]) circle(r=rad);
                translate([length/2 - rad, size/2 - rad]) circle(r=rad);
            }
        }
        
        // Pivot hole through both walls
        rotate([0, 90, 0])
        cylinder(d=hole_dia, h=size+4, center=true, $fn=32);
        
        // Base mounting holes (4 holes)
        // 1/4 inch bolts
        bolt_dia = 6.35; 
        x_spacing = 40; // Fits inside 3" tube (63.5mm ID)
        y_spacing = length - 20;
        
        for (x = [-x_spacing/2, x_spacing/2]) {
            for (y = [-y_spacing/2, y_spacing/2]) {
                translate([x, y, -(size/2 - wall/2)])
                cylinder(d=bolt_dia, h=wall*3, center=true, $fn=16);
            }
        }
    }
}

// TYPE A BRACKET WITH PIN: U-Channel Lug with clevis pin installed
// Same as u_channel_lug but includes the clevis pin and cotter pins
module u_channel_lug_with_pin(tube_size, length, hole_dia) {
    // Render the lug
    u_channel_lug(tube_size, length, hole_dia);
    
    // Add clevis pin through the lug holes
    size = tube_size[0];
    pin_length = size + 30;  // Extends beyond lug walls
    
    rotate([0, 90, 0])
    clevis_pin(hole_dia - 2, pin_length);
}

// TYPE B RING: Large Pivot Boss Ring (CNC plasma cut from 3/4" plate)
// Used for arm pivot reinforcement
module pivot_ring_large(outer_dia, inner_dia) {
    thickness = PLATE_3_4_INCH;
    color("DarkOrange")
    difference() {
        cylinder(d=outer_dia, h=thickness, center=true, $fn=48);
        cylinder(d=inner_dia, h=thickness+2, center=true, $fn=32);
    }
}

// TYPE C RING: Small Pivot Boss Ring (CNC plasma cut from 1" plate)
// Used for QA plate pivot bosses
module pivot_ring_small(outer_dia, inner_dia) {
    thickness = 25.4;  // 1" plate
    color("DarkSlateGray")
    difference() {
        cylinder(d=outer_dia, h=thickness, center=true, $fn=48);
        cylinder(d=inner_dia, h=thickness+2, center=true, $fn=32);
    }
}

// Legacy pivot_boss for backward compatibility
module pivot_boss(outer_dia, inner_dia, thickness) {
    difference() {
        cylinder(d=outer_dia, h=thickness, center=true, $fn=48);
        cylinder(d=inner_dia, h=thickness+2, center=true, $fn=32);
    }
}

// =============================================================================
// FASTENER MODULES - Hex Bolts and Nuts
// =============================================================================

// Hex head dimensions based on bolt diameter (approximate standard)
function hex_head_width(bolt_dia) = bolt_dia * 1.5;  // Across flats
function hex_head_height(bolt_dia) = bolt_dia * 0.7;
function nut_height(bolt_dia) = bolt_dia * 0.8;

// Hexagon shape for extrusion
module hex_shape(across_flats) {
    circle(d=across_flats / cos(30), $fn=6);
}

// Hex bolt head
module hex_bolt_head(bolt_dia) {
    head_width = hex_head_width(bolt_dia);
    head_height = hex_head_height(bolt_dia);
    
    color("DimGray")
    linear_extrude(height=head_height)
    hex_shape(head_width);
}

// Hex nut
module hex_nut(bolt_dia) {
    nut_width = hex_head_width(bolt_dia);
    nut_h = nut_height(bolt_dia);
    
    color("DimGray")
    difference() {
        linear_extrude(height=nut_h)
        hex_shape(nut_width);
        
        translate([0, 0, -1])
        cylinder(d=bolt_dia, h=nut_h+2, $fn=32);
    }
}

// Complete bolt with head, shank, and optional nut
// length = shank length, nut_offset = distance from head to nut (0 = no nut)
module hex_bolt_assembly(bolt_dia, length, show_nut=true) {
    head_height = hex_head_height(bolt_dia);
    nut_h = nut_height(bolt_dia);
    
    // Bolt head
    hex_bolt_head(bolt_dia);
    
    // Shank
    color("DimGray")
    translate([0, 0, head_height])
    cylinder(d=bolt_dia, h=length, $fn=32);
    
    // Nut at end
    if (show_nut) {
        translate([0, 0, head_height + length])
        hex_nut(bolt_dia);
    }
}

// Clevis pin with cotter pin holes
module clevis_pin(pin_dia, length) {
    color("Silver")
    difference() {
        cylinder(d=pin_dia, h=length, center=true, $fn=48);
        
        // Cotter pin holes at each end
        for (z = [-length/2 + pin_dia/2, length/2 - pin_dia/2]) {
            translate([0, 0, z])
            rotate([90, 0, 0])
            cylinder(d=pin_dia/6, h=pin_dia*2, center=true, $fn=16);
        }
    }
    
    // Cotter pins (simple representation)
    color("Gold")
    for (z = [-length/2 + pin_dia/2, length/2 - pin_dia/2]) {
        translate([0, -pin_dia/2, z])
        rotate([90, 0, 0])
        cylinder(d=pin_dia/8, h=pin_dia, $fn=12);
    }
}

// =============================================================================
// SIDE PANELS - TRIANGULAR SANDWICH STRUCTURE
// =============================================================================

module side_panel_profile() {
    // Triangular profile for side panels
    // Tall end at rear (Y=0), shorter sloped end at front (Y=WHEEL_BASE)
    offset(r=10) offset(r=-10)  // Rounded corners
    polygon([
        [0, 0],                              // Rear bottom
        [WHEEL_BASE, 0],                     // Front bottom
        [WHEEL_BASE, MACHINE_HEIGHT * 0.65], // Front top (sloped down)
        [200, MACHINE_HEIGHT],               // Near-rear top (arm pivot area)
        [0, MACHINE_HEIGHT]                  // Rear top (full height)
    ]);
}

// Arc slot cutout for cross beam clearance
// Creates an arc-shaped slot that allows the cross beam to rotate through
module cross_beam_arc_slot(beam_distance, slot_width) {
    // Pivot point in panel local coordinates
    pivot_x = PIVOT_PANEL_X;
    pivot_y = PIVOT_PANEL_Y;
    
    // Arc radius = distance from pivot to cross beam center
    arc_radius = beam_distance;
    
    // Half-width of the slot (beam size + clearance)
    half_slot = slot_width / 2;
    
    // Generate arc by rotating around pivot
    // The arc sweeps from ARM_MIN_ANGLE to ARM_MAX_ANGLE
    // We extend slightly beyond to ensure full clearance
    arc_start = ARM_MIN_ANGLE - 5;
    arc_end = ARM_MAX_ANGLE + 5;
    arc_steps = 60;
    
    translate([pivot_x, pivot_y])
    for (i = [0:arc_steps-1]) {
        angle1 = arc_start + i * (arc_end - arc_start) / arc_steps;
        angle2 = arc_start + (i + 1) * (arc_end - arc_start) / arc_steps;
        
        // Inner and outer points for this arc segment
        hull() {
            // Point at angle1, inner radius
            translate([(arc_radius - half_slot) * cos(angle1), (arc_radius - half_slot) * sin(angle1)])
            circle(d=5, $fn=8);
            
            // Point at angle1, outer radius
            translate([(arc_radius + half_slot) * cos(angle1), (arc_radius + half_slot) * sin(angle1)])
            circle(d=5, $fn=8);
            
            // Point at angle2, inner radius
            translate([(arc_radius - half_slot) * cos(angle2), (arc_radius - half_slot) * sin(angle2)])
            circle(d=5, $fn=8);
            
            // Point at angle2, outer radius
            translate([(arc_radius + half_slot) * cos(angle2), (arc_radius + half_slot) * sin(angle2)])
            circle(d=5, $fn=8);
        }
    }
}

module side_panel_with_holes(is_inner = false) {
    slot_width = CROSS_BEAM_SIZE + 2 * CROSS_BEAM_CLEARANCE;  // Total slot width with clearance
    
    difference() {
        linear_extrude(height=PANEL_THICKNESS)
        difference() {
            side_panel_profile();
            
            // Cross beam arc slots (only on inner panels)
            if (is_inner) {
                // First cross beam slot (near pivot)
                cross_beam_arc_slot(CROSS_BEAM_1_POS, slot_width);
                
                // Second cross beam slot (near bucket) - only if it intersects the panel
                // The second beam is far forward and may not need a cutout
                // Check if any part of the arc passes through the panel
                if (CROSS_BEAM_2_POS < WHEEL_BASE + 200) {
                    cross_beam_arc_slot(CROSS_BEAM_2_POS, slot_width);
                }
            }
        }
        
        // Arm pivot hole near rear top (where the tall section is)
        // Position matches ARM_PIVOT_Y in world coordinates
        translate([ARM_PIVOT_Y, MACHINE_HEIGHT - 50, PANEL_THICKNESS/2])
        cylinder(d=PIVOT_PIN_DIA + 2, h=PANEL_THICKNESS+4, center=true, $fn=48);
        
        // Lift cylinder base mount hole (only on inner panels)
        if (is_inner) {
            translate([WHEEL_BASE - LIFT_CYL_BASE_Y, LIFT_CYL_BASE_Z, PANEL_THICKNESS/2])
            cylinder(d=BOLT_DIA_1 + 2, h=PANEL_THICKNESS+4, center=true, $fn=32);
        }
        
        // Wheel axle holes
        translate([0, WHEEL_DIAMETER/2 - 50, PANEL_THICKNESS/2])
        cylinder(d=BOLT_DIA_3_4 + 2, h=PANEL_THICKNESS+4, center=true, $fn=32);
        
        translate([WHEEL_BASE, WHEEL_DIAMETER/2 - 50, PANEL_THICKNESS/2])
        cylinder(d=BOLT_DIA_3_4 + 2, h=PANEL_THICKNESS+4, center=true, $fn=32);
    }
}

// Left outer panel
module side_panel_left_outer() {
    color("DarkSlateGray")
    translate([-(TRACK_WIDTH/2 + SANDWICH_SPACING/2 + PANEL_THICKNESS), 0, FRAME_Z_OFFSET])
    rotate([90, 0, 90])
    difference() {
        side_panel(is_inner=false);
        
        // Subtract stiffener holes
        rotate([-90, 0, 0]) rotate([0, 0, -90])
        translate([(TRACK_WIDTH/2 + SANDWICH_SPACING/2 + PANEL_THICKNESS), 0, -FRAME_Z_OFFSET])
        stiffener_side_panel_cutters(is_inner=false, is_left=true);
    }
}

// Left inner panel
module side_panel_left_inner() {
    color("DarkSlateGray")
    translate([-(TRACK_WIDTH/2 - SANDWICH_SPACING/2), 0, FRAME_Z_OFFSET])
    rotate([90, 0, 90])
    difference() {
        side_panel(is_inner=true);
        // Trim back for stiffener plate (Y < 31.75 in world coords -> X < 31.75 in part coords)
        translate([-10, -50, -10]) // Start before 0 to be safe, Z range covers panel thickness
        cube([31.75 + 10, MACHINE_HEIGHT + 100, 50]); // Cutout
        
        // Subtract stiffener holes
        rotate([-90, 0, 0]) rotate([0, 0, -90])
        translate([(TRACK_WIDTH/2 - SANDWICH_SPACING/2), 0, -FRAME_Z_OFFSET])
        stiffener_side_panel_cutters(is_inner=true, is_left=true);
    }
}

// Right inner panel
module side_panel_right_inner() {
    color("DarkSlateGray")
    translate([(TRACK_WIDTH/2 - SANDWICH_SPACING/2 - PANEL_THICKNESS), 0, FRAME_Z_OFFSET])
    rotate([90, 0, 90])
    difference() {
        side_panel(is_inner=true);
        // Trim back for stiffener plate (Y < 31.75 in world coords -> X < 31.75 in part coords)
        translate([-10, -50, -10]) // Start before 0 to be safe, Z range covers panel thickness
        cube([31.75 + 10, MACHINE_HEIGHT + 100, 50]); // Cutout
        
        // Subtract stiffener holes
        rotate([-90, 0, 0]) rotate([0, 0, -90])
        translate([-(TRACK_WIDTH/2 - SANDWICH_SPACING/2 - PANEL_THICKNESS), 0, -FRAME_Z_OFFSET])
        stiffener_side_panel_cutters(is_inner=true, is_left=false);
    }
}

// Right outer panel
module side_panel_right_outer() {
    color("DarkSlateGray")
    translate([(TRACK_WIDTH/2 + SANDWICH_SPACING/2), 0, FRAME_Z_OFFSET])
    rotate([90, 0, 90])
    difference() {
        side_panel(is_inner=false);
        
        // Subtract stiffener holes
        rotate([-90, 0, 0]) rotate([0, 0, -90])
        translate([-(TRACK_WIDTH/2 + SANDWICH_SPACING/2), 0, -FRAME_Z_OFFSET])
        stiffener_side_panel_cutters(is_inner=false, is_left=false);
    }
}

// =============================================================================
// CYLINDER MOUNTING LUGS - Using Standardized U-Channel Type A Bracket
// =============================================================================

// Lift cylinder base mount - uses 4"x4" tube cut to U-channel
module cylinder_mounting_lug() {
    lug_length = 100;  // Length along Y axis
    
    // Rotate to orient correctly - U opens toward center of machine
    rotate([0, 0, 90])
    rotate([90, 0, 0])
    u_channel_lug_with_pin(TUBE_4X4_1_4, lug_length, BOLT_DIA_1 + 2);
}

module cylinder_mounting_lugs() {
    // Position lugs between the sandwich plates on each side wall
    // X position is centered in the sandwich gap on each side
    lug_x = TRACK_WIDTH/2;  // At each side wall
    
    // Left side lug - between left inner and outer panels
    translate([-lug_x, LIFT_CYL_BASE_Y, LIFT_CYL_BASE_Z])
    cylinder_mounting_lug();
    
    // Right side lug - between right inner and outer panels  
    translate([lug_x, LIFT_CYL_BASE_Y, LIFT_CYL_BASE_Z])
    cylinder_mounting_lug();
}

// =============================================================================
// ARM PIVOT BOSSES
// =============================================================================

module arm_pivot_assembly() {
    // Pivot pins that go through the sandwich and arm
    nut_h = nut_height(PIVOT_PIN_DIA);  // Use function for nut height
    // Extend length to ensure nuts are outside the plates
    // Plates are at +/- (SANDWICH_SPACING/2 + PANEL_THICKNESS) from center of sandwich
    // Total width of assembly = SANDWICH_SPACING + 2*PANEL_THICKNESS
    // Add 2*nut_h + extra clearance
    pivot_length = SANDWICH_SPACING + PANEL_THICKNESS * 2 + 2 * nut_h + 20;
    
    // Left pivot (at rear where tall section is)
    translate([-(TRACK_WIDTH/2), ARM_PIVOT_Y, ARM_PIVOT_Z]) {
        // Pin
        color("Silver")
        rotate([0, 90, 0])
        cylinder(d=PIVOT_PIN_DIA, h=pivot_length, center=true, $fn=48);
        
        // Inner nut (machine side)
        translate([pivot_length/2 - nut_h/2, 0, 0])
        rotate([0, 90, 0])
        hex_nut(PIVOT_PIN_DIA);
        
        // Outer nut
        translate([-pivot_length/2 + nut_h/2, 0, 0])
        rotate([0, 90, 0])
        hex_nut(PIVOT_PIN_DIA);
    }
    
    // Right pivot (at rear where tall section is)
    translate([(TRACK_WIDTH/2), ARM_PIVOT_Y, ARM_PIVOT_Z]) {
        // Pin
        color("Silver")
        rotate([0, 90, 0])
        cylinder(d=PIVOT_PIN_DIA, h=pivot_length, center=true, $fn=48);
        
        // Inner nut (machine side)
        translate([-pivot_length/2 + nut_h/2, 0, 0])
        rotate([0, 90, 0])
        hex_nut(PIVOT_PIN_DIA);
        
        // Outer nut
        translate([pivot_length/2 - nut_h/2, 0, 0])
        rotate([0, 90, 0])
        hex_nut(PIVOT_PIN_DIA);
    }
}

// =============================================================================
// VERTICAL STIFFENER PLATES
// =============================================================================

// Helper function for bolt heights
function get_stiffener_holes_a(h) = [for(i=[0:3]) h * (0.2 + i*0.2)];
function get_stiffener_holes_b(h) = [for(i=[0:3]) h * (0.1 + i*0.2)];

module vertical_angle_iron_smart(height, size=[50.8, 6.35]) {
    leg = size[0];
    thick = size[1];
    bolt_dia = 9.525; // 3/8" bolts
    hole_offset = leg * 0.6; // Shift holes away from corner for tool clearance
    
    difference() {
        // Extrude L-shape along Z
        // Origin at corner [0,0,0]. Legs along +X and +Y.
        linear_extrude(height=height)
        polygon([
            [0, 0],
            [leg, 0],
            [leg, thick],
            [thick, thick],
            [thick, leg],
            [0, leg]
        ]);
        
        // Holes on Face A (X-leg)
        // Leg extends along X. Normal is Y (inner face at y=thick, outer at y=0).
        // We assume bolting through the leg in Y direction.
        for (z = get_stiffener_holes_a(height)) {
            translate([hole_offset, 0, z]) // Centered on Y=0 plane
            rotate([90, 0, 0])
            cylinder(d=bolt_dia, h=40, center=true, $fn=16);
        }
        
        // Holes on Face B (Y-leg)
        // Leg extends along Y. Normal is X.
        // We assume bolting through the leg in X direction.
        for (z = get_stiffener_holes_b(height)) {
            translate([0, hole_offset, z]) // Centered on X=0 plane
            rotate([0, 90, 0])
            cylinder(d=bolt_dia, h=40, center=true, $fn=16);
        }
    }
}

// Module to generate hole cutters for the side panels
module stiffener_side_panel_cutters(is_inner, is_left) {
    // Mid Plate Parameters
    mid_y = 800;
    mid_z_start = FRAME_Z_OFFSET + 100;
    mid_z_end = FRAME_Z_OFFSET + 350;
    mid_h = mid_z_end - mid_z_start;
    mid_angle_size = [50.8, 6.35];
    mid_leg = mid_angle_size[0];
    mid_hole_offset = mid_leg * 0.6;
    
    // Back Plate Parameters
    back_y = 25.4;
    back_z_start = FRAME_Z_OFFSET + 350;
    back_z_end = FRAME_Z_OFFSET + MACHINE_HEIGHT;
    back_h = back_z_end - back_z_start;
    back_angle_size = [50.8, 6.35];
    back_leg = back_angle_size[0];
    back_hole_offset = back_leg * 0.6;
    
    bolt_dia = 9.525;
    
    // Mid Plate Holes (Only on Inner Panels)
    if (is_inner) {
        // Left Inner Panel (is_left=true)
        if (is_left) {
            // Left Front: Face B (Y-leg). Rot 0.
            // Global Hole: [-plate_width/2, mid_y + thick + leg/2, z]
            for (z = get_stiffener_holes_b(mid_h)) {
                translate([-(TRACK_WIDTH - SANDWICH_SPACING - 2*PANEL_THICKNESS)/2, mid_y + PLATE_1_4_INCH + mid_hole_offset, mid_z_start + z])
                rotate([0, 90, 0])
                cylinder(d=bolt_dia, h=40, center=true, $fn=16);
            }
            
            // Left Rear: Face A (X-leg -> -Y leg). Rot -90.
            // Global Hole: [-plate_width/2, mid_y - leg/2, z]
            for (z = get_stiffener_holes_a(mid_h)) {
                translate([-(TRACK_WIDTH - SANDWICH_SPACING - 2*PANEL_THICKNESS)/2, mid_y - mid_hole_offset, mid_z_start + z])
                rotate([0, 90, 0])
                cylinder(d=bolt_dia, h=40, center=true, $fn=16);
            }
        }
        
        // Right Inner Panel (is_left=false)
        if (!is_left) {
             // Right Front: Face A (X-leg -> +Y leg). Rot 90.
             // Global Hole: [plate_width/2, mid_y + thick + leg/2, z]
             for (z = get_stiffener_holes_a(mid_h)) {
                translate([(TRACK_WIDTH - SANDWICH_SPACING - 2*PANEL_THICKNESS)/2, mid_y + PLATE_1_4_INCH + mid_hole_offset, mid_z_start + z])
                rotate([0, 90, 0])
                cylinder(d=bolt_dia, h=40, center=true, $fn=16);
            }
            
            // Right Rear: Face B (Y-leg -> -Y leg). Rot 180.
            // Global Hole: [plate_width/2, mid_y - leg/2, z]
            for (z = get_stiffener_holes_b(mid_h)) {
                translate([(TRACK_WIDTH - SANDWICH_SPACING - 2*PANEL_THICKNESS)/2, mid_y - mid_hole_offset, mid_z_start + z])
                rotate([0, 90, 0])
                cylinder(d=bolt_dia, h=40, center=true, $fn=16);
            }
        }
        
        // Back Plate Inner Angles (Inner Panels)
        // Left Inner Panel (is_left=true)
        if (is_left) {
            // Left Side Angle: Face A (Y-leg -> +Y leg). Rot 90.
            // Corner at [-(TW/2 - SS/2), back_y + thick, z].
            // Global Hole: [corner_x, back_y + thick + leg/2, z].
            for (z = get_stiffener_holes_a(back_h)) {
                translate([-(TRACK_WIDTH/2 - SANDWICH_SPACING/2), back_y + PLATE_1_4_INCH + back_hole_offset, back_z_start + z])
                rotate([0, 90, 0])
                cylinder(d=bolt_dia, h=40, center=true, $fn=16);
            }
            
            // Right Side Angle: Face B (Y-leg). Rot 0.
            // Corner at [-(TW/2 - SS/2) + PT, back_y + thick, z].
            // Global Hole: [corner_x, back_y + thick + leg/2, z].
            for (z = get_stiffener_holes_b(back_h)) {
                translate([-(TRACK_WIDTH/2 - SANDWICH_SPACING/2) + PANEL_THICKNESS, back_y + PLATE_1_4_INCH + back_hole_offset, back_z_start + z])
                rotate([0, 90, 0])
                cylinder(d=bolt_dia, h=40, center=true, $fn=16);
            }
        }
        
        // Right Inner Panel (is_left=false)
        if (!is_left) {
            // Left Side Angle: Face A (Y-leg -> +Y leg). Rot 90.
            // Corner at [inner_offset - PT, back_y + thick, z].
            for (z = get_stiffener_holes_a(back_h)) {
                translate([(TRACK_WIDTH/2 - SANDWICH_SPACING/2) - PANEL_THICKNESS, back_y + PLATE_1_4_INCH + back_hole_offset, back_z_start + z])
                rotate([0, 90, 0])
                cylinder(d=bolt_dia, h=40, center=true, $fn=16);
            }
            
            // Right Side Angle: Face B (Y-leg). Rot 0.
            // Corner at [inner_offset, back_y + thick, z].
            for (z = get_stiffener_holes_b(back_h)) {
                translate([(TRACK_WIDTH/2 - SANDWICH_SPACING/2), back_y + PLATE_1_4_INCH + back_hole_offset, back_z_start + z])
                rotate([0, 90, 0])
                cylinder(d=bolt_dia, h=40, center=true, $fn=16);
            }
        }
    }
    
    // Back Plate Outer Angles (Outer Panels)
    if (!is_inner) {
        // Left Outer Panel (is_left=true)
        if (is_left) {
            // Legs +X, +Y. Rot 0. Face B (Y-leg) touches panel.
            // Corner at [-(TW/2 + SS/2 + PT), back_y + thick, z].
            for (z = get_stiffener_holes_b(back_h)) {
                translate([-(TRACK_WIDTH/2 + SANDWICH_SPACING/2 + PANEL_THICKNESS), back_y + PLATE_1_4_INCH + back_hole_offset, back_z_start + z])
                rotate([0, 90, 0])
                cylinder(d=bolt_dia, h=40, center=true, $fn=16);
            }
        }
        
        // Right Outer Panel (is_left=false)
        if (!is_left) {
            // Legs -X, +Y. Rot 90. Face A (X-leg -> +Y leg) touches panel.
            // Corner at [TW/2 + SS/2, back_y + thick, z].
            for (z = get_stiffener_holes_a(back_h)) {
                translate([(TRACK_WIDTH/2 + SANDWICH_SPACING/2), back_y + PLATE_1_4_INCH + back_hole_offset, back_z_start + z])
                rotate([0, 90, 0])
                cylinder(d=bolt_dia, h=40, center=true, $fn=16);
            }
        }
    }
}

module mid_stiffener_plate(y_pos, z_start, z_end) {
    plate_width = TRACK_WIDTH - SANDWICH_SPACING - 2*PANEL_THICKNESS;
    plate_height = z_end - z_start;
    plate_thickness = PLATE_1_4_INCH; 
    angle_size = [50.8, 6.35];
    leg = angle_size[0];
    bolt_dia = 9.525;
    hole_offset = leg * 0.6;
    
    difference() {
        // Plate
        color("Silver")
        translate([-plate_width/2, y_pos, z_start])
        cube([plate_width, plate_thickness, plate_height]);
        
        // Subtract holes for angle irons
        // Left Front: Legs +X, +Y. Face A (X-leg) on plate. Holes z_A.
        // Pos: [-plate_width/2, y_pos + thick, z].
        // Hole: [-plate_width/2 + offset, y_pos + thick, z].
        for (z = get_stiffener_holes_a(plate_height)) {
            translate([-plate_width/2 + hole_offset, y_pos + plate_thickness, z_start + z])
            rotate([90, 0, 0])
            cylinder(d=bolt_dia, h=20, center=true, $fn=16);
        }
        
        // Left Rear: Legs +X, -Y. Rot -90. Face B (Y->X) on plate. Holes z_B.
        // Pos: [-plate_width/2, y_pos, z].
        // Hole: [-plate_width/2 + offset, y_pos, z].
        for (z = get_stiffener_holes_b(plate_height)) {
            translate([-plate_width/2 + hole_offset, y_pos, z_start + z])
            rotate([90, 0, 0])
            cylinder(d=bolt_dia, h=20, center=true, $fn=16);
        }
        
        // Right Front: Legs -X, +Y. Rot 90. Face B (Y->-X) on plate. Holes z_B.
        // Pos: [plate_width/2, y_pos + thick, z].
        // Hole: [plate_width/2 - offset, y_pos + thick, z].
        for (z = get_stiffener_holes_b(plate_height)) {
            translate([plate_width/2 - hole_offset, y_pos + plate_thickness, z_start + z])
            rotate([90, 0, 0])
            cylinder(d=bolt_dia, h=20, center=true, $fn=16);
        }
        
        // Right Rear: Legs -X, -Y. Rot 180. Face A (X->-X) on plate. Holes z_A.
        // Pos: [plate_width/2, y_pos, z].
        // Hole: [plate_width/2 - offset, y_pos, z].
        for (z = get_stiffener_holes_a(plate_height)) {
            translate([plate_width/2 - hole_offset, y_pos, z_start + z])
            rotate([90, 0, 0])
            cylinder(d=bolt_dia, h=20, center=true, $fn=16);
        }
    }
    
    // Angles
    // Left Front (Legs +X, +Y)
    translate([-plate_width/2, y_pos + plate_thickness, z_start])
    vertical_angle_iron_smart(plate_height, angle_size);
    
    // Left Rear (Legs +X, -Y) -> Rot Z -90
    translate([-plate_width/2, y_pos, z_start])
    rotate([0, 0, -90])
    vertical_angle_iron_smart(plate_height, angle_size);
    
    // Right Front (Legs -X, +Y) -> Rot Z 90
    translate([plate_width/2, y_pos + plate_thickness, z_start])
    rotate([0, 0, 90])
    vertical_angle_iron_smart(plate_height, angle_size);
    
    // Right Rear (Legs -X, -Y) -> Rot Z 180
    translate([plate_width/2, y_pos, z_start])
    rotate([0, 0, 180])
    vertical_angle_iron_smart(plate_height, angle_size);
}

module back_stiffener_plate() {
    plate_width = TRACK_WIDTH + SANDWICH_SPACING; 
    plate_thickness = PLATE_1_4_INCH;
    y_pos = 25.4; 
    z_start = FRAME_Z_OFFSET + 350;
    z_end = FRAME_Z_OFFSET + MACHINE_HEIGHT;
    plate_height = z_end - z_start;
    angle_size = [50.8, 6.35];
    leg = angle_size[0];
    bolt_dia = 9.525;
    hole_offset = leg * 0.6;
    
    inner_offset = TRACK_WIDTH/2 - SANDWICH_SPACING/2;
    
    difference() {
        // Plate
        color("Silver")
        translate([-plate_width/2, y_pos, z_start])
        cube([plate_width, plate_thickness, plate_height]);
        
        // Subtract holes
        // Left Outer: Legs +X, +Y. Face A (X) on plate. Holes z_A.
        for (z = get_stiffener_holes_a(plate_height)) {
            translate([-plate_width/2 + hole_offset, y_pos + plate_thickness, z_start + z])
            rotate([90, 0, 0])
            cylinder(d=bolt_dia, h=20, center=true, $fn=16);
        }
        
        // Right Outer: Legs -X, +Y. Rot 90. Face B (Y->-X) on plate. Holes z_B.
        for (z = get_stiffener_holes_b(plate_height)) {
            translate([plate_width/2 - hole_offset, y_pos + plate_thickness, z_start + z])
            rotate([90, 0, 0])
            cylinder(d=bolt_dia, h=20, center=true, $fn=16);
        }
        
        // Left Inner Panel Left Side: Legs -X, +Y. Rot 90. Face B (Y->-X) on plate. Holes z_B.
        for (z = get_stiffener_holes_b(plate_height)) {
            translate([-inner_offset - hole_offset, y_pos + plate_thickness, z_start + z])
            rotate([90, 0, 0])
            cylinder(d=bolt_dia, h=20, center=true, $fn=16);
        }
        
        // Left Inner Panel Right Side: Legs +X, +Y. Rot 0. Face A (X) on plate. Holes z_A.
        for (z = get_stiffener_holes_a(plate_height)) {
            translate([-inner_offset + PANEL_THICKNESS + hole_offset, y_pos + plate_thickness, z_start + z])
            rotate([90, 0, 0])
            cylinder(d=bolt_dia, h=20, center=true, $fn=16);
        }
        
        // Right Inner Panel Left Side: Legs -X, +Y. Rot 90. Face B (Y->-X) on plate. Holes z_B.
        for (z = get_stiffener_holes_b(plate_height)) {
            translate([inner_offset - PANEL_THICKNESS - hole_offset, y_pos + plate_thickness, z_start + z])
            rotate([90, 0, 0])
            cylinder(d=bolt_dia, h=20, center=true, $fn=16);
        }
        
        // Right Inner Panel Right Side: Legs +X, +Y. Rot 0. Face A (X) on plate. Holes z_A.
        for (z = get_stiffener_holes_a(plate_height)) {
            translate([inner_offset + hole_offset, y_pos + plate_thickness, z_start + z])
            rotate([90, 0, 0])
            cylinder(d=bolt_dia, h=20, center=true, $fn=16);
        }
    }
    
    // Angles
    // Left Outer (Legs +X, +Y)
    translate([-plate_width/2, y_pos + plate_thickness, z_start])
    vertical_angle_iron_smart(plate_height, angle_size);
    
    // Right Outer (Legs -X, +Y) -> Rot Z 90
    translate([plate_width/2, y_pos + plate_thickness, z_start])
    rotate([0, 0, 90])
    vertical_angle_iron_smart(plate_height, angle_size);
    
    // Inner Angles
    // Left Inner Panel
    // Left Side (Legs -X, +Y) -> Rot Z 90
    translate([-inner_offset, y_pos + plate_thickness, z_start])
    rotate([0, 0, 90])
    vertical_angle_iron_smart(plate_height, angle_size);
    
    // Right Side (Legs +X, +Y)
    translate([-inner_offset + PANEL_THICKNESS, y_pos + plate_thickness, z_start])
    vertical_angle_iron_smart(plate_height, angle_size);
    
    // Right Inner Panel
    // Left Side (Legs -X, +Y) -> Rot Z 90
    translate([inner_offset - PANEL_THICKNESS, y_pos + plate_thickness, z_start])
    rotate([0, 0, 90])
    vertical_angle_iron_smart(plate_height, angle_size);
    
    // Right Side (Legs +X, +Y)
    translate([inner_offset, y_pos + plate_thickness, z_start])
    vertical_angle_iron_smart(plate_height, angle_size);
}

// =============================================================================
// BASE FRAME ASSEMBLY
// =============================================================================

module base_frame() {
    side_panel_left_outer();
    side_panel_left_inner();
    side_panel_right_inner();
    side_panel_right_outer();
    cylinder_mounting_lugs();
    arm_pivot_assembly();
    
    // Vertical stiffener plates
    // Back plate (1 inch from rear)
    back_stiffener_plate();
    
    // Mid plate (Centered between wheels)
    // Calculate centered position between wheels
    front_wheel_y = WHEEL_BASE - WHEEL_RADIUS;
    rear_wheel_target_y = LIFT_CYL_BASE_Y + WHEEL_RADIUS + 100; 
    max_rear_y = front_wheel_y - (WHEEL_DIAMETER + 50);
    final_rear_y = min(rear_wheel_target_y, max_rear_y);
    
    mid_plate_y = (front_wheel_y + final_rear_y) / 2;
    
    // Calculate wall height at this Y position (Replicating logic from side_panel.scad)
    pivot_y_panel = ARM_PIVOT_Y;
    pivot_z_panel = MACHINE_HEIGHT - 50;
    arm_angle = is_undef(ARM_MIN_ANGLE) ? -45 : ARM_MIN_ANGLE;
    arm_clearance_z = 80;
    
    dy = mid_plate_y - pivot_y_panel;
    dz = dy * tan(arm_angle);
    wall_height_at_y = pivot_z_panel + dz - arm_clearance_z;
    
    // Target top Z: 2 inches (50.8mm) below wall top
    // Wall top in world coords = FRAME_Z_OFFSET + wall_height_at_y
    mid_plate_top_z = FRAME_Z_OFFSET + wall_height_at_y - 50.8;
    
    mid_stiffener_plate(mid_plate_y, FRAME_Z_OFFSET + 100, mid_plate_top_z);
}

// =============================================================================
// ENGINE
// =============================================================================

module engine() {
    // V-Twin Engine Representation
    // Dimensions approx for 25HP engine
    // TODO: Design proper engine mounting plate and bolt pattern
    // Currently floating in space
    
    eng_width = 450;
    eng_depth = 400;
    eng_height = 450;
    
    translate([0, ENGINE_POS_Y, ENGINE_POS_Z])
    {
        color("DimGray")
        union() {
            // Crankcase base
            translate([0, 0, 100])
            cube([eng_width, eng_depth, 200], center=true);
            
            // Cylinders (V-Twin)
            for (rot = [-45, 45]) {
                rotate([0, rot, 0])
                translate([0, 0, 200])
                cylinder(d=120, h=150, center=true);
            }
            
            // Flywheel/Fan cover
            translate([0, -eng_depth/2 - 20, 150])
            rotate([90, 0, 0])
            cylinder(d=250, h=50, center=true);
            
            // Muffler
            translate([0, eng_depth/2 + 50, 150])
            cube([300, 100, 100], center=true);
        }
    }
}

// =============================================================================
// WEIGHT DISTRIBUTION & STABILITY ANALYSIS
// =============================================================================

module calculate_cog() {
    // --- WEIGHT ESTIMATES (kg) ---
    w_frame = 350;      // Chassis, panels, crossmembers
    w_wheel = 40;       // Tire + Rim + Motor (x4)
    w_arm = 180;        // Arms + Crossbeams + Cylinders
    w_bucket = 120;     // Bucket + QA Plate
    w_engine = ENGINE_WEIGHT_KG;
    w_operator = 90;    // Operator on rear deck
    w_fluids = 40;      // Hydraulic oil + fuel
    
    // Max Load Calculation (Tipping Load)
    // We calculate moments about the FRONT AXLE
    
    // --- POSITIONS (Y, Z) relative to World Origin ---
    // Frame: Centroid approx middle of machine
    pos_frame = [MACHINE_LENGTH/2, FRAME_Z_OFFSET + 300];
    
    // Wheels:
    // Front axle Y is calculated in wheel_assemblies
    front_axle_y = WHEEL_BASE - WHEEL_RADIUS;
    // Rear axle Y is calculated in wheel_assemblies (re-calculating here for logic)
    rear_axle_y_target = LIFT_CYL_BASE_Y + WHEEL_RADIUS + 100;
    
    // Re-calculate wheel positions to match wheel_assemblies logic
    _front_y = WHEEL_BASE - WHEEL_RADIUS;
    _rear_y_target = LIFT_CYL_BASE_Y + WHEEL_RADIUS + 100; // Moved forward of cylinder
    _min_wb = WHEEL_DIAMETER + 50;
    _max_rear_y = _front_y - _min_wb;
    _rear_y = min(_rear_y_target, _max_rear_y);
    
    pos_wheels_front = [_front_y, WHEEL_RADIUS];
    pos_wheels_rear = [_rear_y, WHEEL_RADIUS];
    
    // Engine
    pos_engine = [ENGINE_POS_Y, ENGINE_POS_Z + 200];
    
    // Operator (Standing on deck at rear)
    pos_operator = [-200, 300]; // Behind machine
    
    // Arms (Dynamic based on angle)
    // Centroid approx 40% along arm
    arm_cg_dist = ARM_LENGTH * 0.4;
    pos_arm = [
        ARM_PIVOT_Y + arm_cg_dist * cos(ARM_LIFT_ANGLE),
        ARM_PIVOT_Z + arm_cg_dist * sin(ARM_LIFT_ANGLE)
    ];
    
    // Bucket (Dynamic)
    // At tip of arm
    pos_bucket = [
        ARM_PIVOT_Y + ARM_LENGTH * cos(ARM_LIFT_ANGLE),
        ARM_PIVOT_Z + ARM_LENGTH * sin(ARM_LIFT_ANGLE)
    ];
    
    // --- MOMENT CALCULATION (About Rear Axle for CoG Y) ---
    // We use World Y=0 (Rear of frame) as reference datum.
    
    moment_frame = w_frame * pos_frame[0];
    moment_wheels = 2 * w_wheel * pos_wheels_front[0] + 2 * w_wheel * pos_wheels_rear[0];
    moment_engine = w_engine * pos_engine[0];
    moment_operator = w_operator * pos_operator[0];
    moment_arm = w_arm * pos_arm[0];
    moment_bucket = w_bucket * pos_bucket[0];
    moment_fluids = w_fluids * (MACHINE_LENGTH * 0.3); // Tank usually near back
    
    total_weight_empty = w_frame + 4*w_wheel + w_engine + w_arm + w_bucket + w_fluids; 
    total_moment_empty = moment_frame + moment_wheels + moment_engine + moment_arm + moment_bucket + moment_fluids;
    
    cog_y_empty = total_moment_empty / total_weight_empty;
    
    // --- TIPPING LOAD CALCULATION ---
    // Tipping happens when moment about Front Axle is zero.
    // Counter-balance moments (behind front axle) vs Load moments (in front)
    // Distances relative to Front Axle
    
    // Note: Positive distance = Behind axle (Counter-weight)
    //       Negative distance = In front of axle (Load)
    
    // Sum of counter-moments (Weight * Distance from Front Axle)
    // We want the machine to stay flat, so Sum(Weight_i * (Front_Y - Y_i)) > 0
    
    tipping_moment_capacity = 
        w_frame * (_front_y - pos_frame[0]) +
        2 * w_wheel * 0 + // Front wheels have 0 moment arm
        2 * w_wheel * (_front_y - pos_wheels_rear[0]) +
        w_engine * (_front_y - pos_engine[0]) +
        w_operator * (_front_y - pos_operator[0]) +
        w_fluids * (_front_y - (MACHINE_LENGTH * 0.3)) +
        w_arm * (_front_y - pos_arm[0]) +
        w_bucket * (_front_y - pos_bucket[0]);
        
    // Load is at bucket position. 
    // Load Moment = Load_Weight * (pos_bucket[0] - _front_y)  <-- Distance in front
    // Tipping occurs when Load_Moment = Tipping_Moment_Capacity
    
    load_dist_from_axle = pos_bucket[0] - _front_y;
    
    // If load is behind axle (e.g. fully raised and back), it won't tip forward
    tipping_load_kg = (load_dist_from_axle > 0) ? (tipping_moment_capacity / load_dist_from_axle) : 9999;
    
    rated_load_kg = tipping_load_kg * 0.5; // 50% of tipping load is safe rating (ISO standard)
    
    // --- VISUALIZATION ---
    // Draw CoG Sphere (Empty)
    color("Magenta")
    translate([0, cog_y_empty, FRAME_Z_OFFSET + 400])
    sphere(d=100);
    
    // Console Output
    echo("=========================================");
    echo("STABILITY ANALYSIS");
    echo("=========================================");
    echo("Engine HP:", ENGINE_HP);
    echo("Engine Weight:", w_engine, "kg");
    echo("Total Machine Weight (Empty):", total_weight_empty, "kg");
    echo("CoG Y Position (Empty):", cog_y_empty, "mm from rear");
    echo("Front Axle Position:", _front_y, "mm");
    echo("Rear Axle Position:", _rear_y, "mm");
    echo("Wheelbase:", _front_y - _rear_y, "mm");
    echo("Load Distance from Front Axle:", load_dist_from_axle, "mm");
    echo("TIPPING LOAD (at current arm reach):", tipping_load_kg, "kg");
    echo("RATED OPERATING CAPACITY (50%):", rated_load_kg, "kg");
    echo("=========================================");
}

// =============================================================================
// WHEEL ASSEMBLIES
// =============================================================================

module wheel_assemblies() {
    if (show_wheels) {
        // Wheel positions - outside the outer panels with clearance
        // Wheel centers at Z=WHEEL_RADIUS so bottom of wheel touches Z=0 (ground)
        
        // Front wheels: Standard position relative to frame front
        front_wheel_y = WHEEL_BASE - WHEEL_RADIUS;
        
        // Rear wheels: Moved forward of cylinder mount
        // Cylinder mount is at LIFT_CYL_BASE_Y
        // We want rear axle > LIFT_CYL_BASE_Y
        // Add clearance for cylinder body and wheel rotation
        rear_wheel_target_y = LIFT_CYL_BASE_Y + WHEEL_RADIUS + 100; 
        
        // Check for collision with front wheels
        // Ensure minimum wheelbase
        min_wheelbase = WHEEL_DIAMETER + 50; // 50mm gap between tires
        
        // Let's calculate the max Y the rear wheel can be at without hitting front wheel
        max_rear_y = front_wheel_y - min_wheelbase;
        
        // Use the target, but clamp to max
        final_rear_y = min(rear_wheel_target_y, max_rear_y);
        
        // Warn if we couldn't meet the "in front of cylinder" constraint due to space
        if (final_rear_y < LIFT_CYL_BASE_Y) {
            echo("WARNING: Rear wheels could not be placed in front of cylinder mount due to lack of space!");
        }
        
        positions = [
            [-WHEEL_X_OFFSET, front_wheel_y, WHEEL_RADIUS],   // Front left
            [ WHEEL_X_OFFSET, front_wheel_y, WHEEL_RADIUS],   // Front right
            [-WHEEL_X_OFFSET, final_rear_y, WHEEL_RADIUS],    // Rear left
            [ WHEEL_X_OFFSET, final_rear_y, WHEEL_RADIUS]     // Rear right
        ];
        
        for (i = [0:3]) {
            translate(positions[i])
            rotate([0, 0, 90])
            rotate([0, animation_time * 360, 0])  // Spin around axle (wheel axis is Y)
            {
                wheel(WHEEL_DIAMETER, WHEEL_WIDTH, 150, true);
                
                // Hydraulic motor on powered wheels
                if (i < 2 || all_wheel_drive) {
                    translate([0, 0, -WHEEL_WIDTH/2 - 40])
                    rotate([0, 90, 0])
                    hydraulic_motor(100, 25.4, 60, true);
                }
            }
        }
    }
}

// =============================================================================
// LOADER ARMS
// =============================================================================



module loader_arms() {
    if (show_loader_arms) {
        translate([0, ARM_PIVOT_Y, ARM_PIVOT_Z])
        rotate([is_undef(ARM_LIFT_ANGLE) ? 0 : ARM_LIFT_ANGLE, 0, 0])
        {
            // Left Arm
            translate([-ARM_SPACING/2, 0, 0]) 
                rotate([0, 0, 90])
                translate([0, -TUBE_2X6_1_4[0]/2, -TUBE_2X6_1_4[1]/2])
                loader_arm_v2(angle=0, side="left");
            
            // Right Arm
            translate([ARM_SPACING/2, 0, 0]) 
                rotate([0, 0, 90])
                translate([0, -TUBE_2X6_1_4[0]/2, -TUBE_2X6_1_4[1]/2])
                loader_arm_v2(angle=0, side="right");
            
            // Cross Beam
            // Connects the elbow assemblies at the new mounting hole position
            // Manual adjustment: Shift back by 3 inches (76.2mm) to align with angle irons
            translate([0, CROSS_BEAM_1_POS - 76.2, 0]) {
                difference() {
                    rotate([0, 90, 0])
                    // Rounded 2x6 tube
                    // Oriented flat: 6" wide (Y), 2" tall (Z)
                    linear_extrude(height=ARM_SPACING, center=true)
                    hull() {
                        w = 50.8;  // 2 inches (Vertical Z)
                        h = 152.4; // 6 inches (Along arm Y)
                        r = 12.7;  // 1/2 inch radius
                        translate([-w/2 + r, -h/2 + r]) circle(r=r);
                        translate([w/2 - r, -h/2 + r]) circle(r=r);
                        translate([w/2 - r, h/2 - r]) circle(r=r);
                        translate([-w/2 + r, h/2 - r]) circle(r=r);
                    }
                    
                    // Mounting Holes for Angle Irons
                    // Located at ends of beam, top and bottom
                    // X position: ARM_SPACING/2 - (Tube/2 + Plate + AngleHoleOffset)
                    // AngleHoleOffset = 25.4 (1 inch)
                    // Tube/2 = 25.4
                    // Plate = 6.35
                    // X = +/- (ARM_SPACING/2 - 57.15)
                    
                    hole_x_offset = ARM_SPACING/2 - (25.4 + ARM_PLATE_THICKNESS + 25.4);
                    hole_y_spacing = 25.4; // Offset from center of beam (2 inch spacing total)
                    
                    for (x_pos = [-hole_x_offset, hole_x_offset]) {
                        for (y_pos = [-hole_y_spacing, hole_y_spacing]) {
                            translate([x_pos, y_pos, 0])
                            cylinder(d=BOLT_DIA_1_2, h=100, center=true, $fn=32);
                        }
                    }
                }
                
                // Cylinder Mounts on Cross Beam
                for (side = [-1, 1]) {
                    translate([side * BUCKET_CYL_X_SPACING, 0, CROSS_BEAM_MOUNT_Z_OFFSET])
                    rotate([180, 0, 0])
                    u_channel_lug_with_pin(TUBE_3X3_1_4, 80, BOLT_DIA_3_4 + 2);
                }
            }
        }
    }
}

// =============================================================================
// BOBCAT QUICK ATTACH INTERFACE
// =============================================================================



// =============================================================================
// BUCKET
// =============================================================================

// =============================================================================
// BUCKET
// =============================================================================

module bucket_back_plate() {
    tab_len = 50;
    difference() {
        union() {
            // Main Plate
            color("Yellow")
            translate([-BUCKET_WIDTH/2, 0, -BUCKET_HEIGHT])
            cube([BUCKET_WIDTH, PLATE_1_4_INCH, BUCKET_HEIGHT]);
            
            // Tabs for Side Plates (Left and Right)
            for (z = [-BUCKET_HEIGHT + 50 : 150 : -50]) {
                // Left Tabs
                translate([-BUCKET_WIDTH/2 - PLATE_1_4_INCH, 0, z])
                cube([PLATE_1_4_INCH, PLATE_1_4_INCH, tab_len]);
                
                // Right Tabs
                translate([BUCKET_WIDTH/2, 0, z])
                cube([PLATE_1_4_INCH, PLATE_1_4_INCH, tab_len]);
            }
        }
        
        // Slots for Bottom Plate tabs
        for (x = [-BUCKET_WIDTH/2 + 50 : 150 : BUCKET_WIDTH/2 - 50]) {
            translate([x, -1, -BUCKET_HEIGHT])
            cube([tab_len, PLATE_1_4_INCH + 2, PLATE_1_4_INCH]);
        }
    }
}

module bucket_bottom_plate() {
    tab_len = 50;
    difference() {
        union() {
            // Main Plate
            color("Yellow")
            translate([-BUCKET_WIDTH/2, PLATE_1_4_INCH, -BUCKET_HEIGHT])
            cube([BUCKET_WIDTH, BUCKET_DEPTH - PLATE_1_4_INCH, PLATE_1_4_INCH]);
            
            // Tabs for Back Plate (Rear edge)
            for (x = [-BUCKET_WIDTH/2 + 50 : 150 : BUCKET_WIDTH/2 - 50]) {
                translate([x, 0, -BUCKET_HEIGHT])
                cube([tab_len, PLATE_1_4_INCH, PLATE_1_4_INCH]);
            }
            
            // Tabs for Side Plates (Left and Right)
            for (y = [PLATE_1_4_INCH + 50 : 150 : BUCKET_DEPTH - 50]) {
                // Left Tabs
                translate([-BUCKET_WIDTH/2 - PLATE_1_4_INCH, y, -BUCKET_HEIGHT])
                cube([PLATE_1_4_INCH, tab_len, PLATE_1_4_INCH]);
                
                // Right Tabs
                translate([BUCKET_WIDTH/2, y, -BUCKET_HEIGHT])
                cube([PLATE_1_4_INCH, tab_len, PLATE_1_4_INCH]);
            }
        }
    }
}

module bucket_side_plate(is_left=true) {
    tab_len = 50;
    side = is_left ? -1 : 1;
    x_pos = (side == -1) ? -BUCKET_WIDTH/2 - PLATE_1_4_INCH : BUCKET_WIDTH/2;
    
    difference() {
        // Main Shape
        color("Yellow")
        translate([x_pos, 0, -BUCKET_HEIGHT])
        rotate([0, -90, 0])
        linear_extrude(height=PLATE_1_4_INCH)
        polygon([
            [0, 0],
            [BUCKET_HEIGHT, 0],
            [BUCKET_HEIGHT, PLATE_1_2_INCH],
            [BUCKET_HEIGHT * 0.3, BUCKET_DEPTH],
            [0, BUCKET_DEPTH]
        ]);
        
        // Slots for Back Plate tabs
        for (z = [-BUCKET_HEIGHT + 50 : 150 : -50]) {
            translate([x_pos - 1, -1, z])
            cube([PLATE_1_4_INCH + 2, PLATE_1_4_INCH + 2, tab_len]);
        }
        
        // Slots for Bottom Plate tabs
        for (y = [PLATE_1_4_INCH + 50 : 150 : BUCKET_DEPTH - 50]) {
            translate([x_pos - 1, y, -BUCKET_HEIGHT - 1])
            cube([PLATE_1_4_INCH + 2, tab_len, PLATE_1_4_INCH + 2]);
        }
    }
}

module bucket() {
    // Standard bucket with Tab & Slot construction for welding
    
    bucket_back_plate();
    bucket_bottom_plate();
    bucket_side_plate(is_left=true);
    bucket_side_plate(is_left=false);
    
    // Cylinder Lugs are now handled in bucket_attachment() to ensure correct alignment with arm
    // See bucket_attachment() module for lug placement.
    
    // Cutting edge
    color("DarkSlateGray")
    translate([-BUCKET_WIDTH/2, BUCKET_DEPTH - PLATE_3_4_INCH, -BUCKET_HEIGHT])
    cube([BUCKET_WIDTH, PLATE_3_4_INCH, 80]);
    
    // Weld Beads
    // 1. Bottom-Back Corner (Inside)
    translate([0, PLATE_1_4_INCH, -BUCKET_HEIGHT + PLATE_1_4_INCH])
    rotate([0, 0, 90])
    weld_bead(length=BUCKET_WIDTH - 20, diameter=8);
    
    // 2. Bottom-Side Corners (Inside)
    for (x = [-BUCKET_WIDTH/2, BUCKET_WIDTH/2]) {
        translate([x, BUCKET_DEPTH/2, -BUCKET_HEIGHT + PLATE_1_4_INCH])
        weld_bead(length=BUCKET_DEPTH - 20, diameter=8);
    }
    
    // 3. Back-Side Corners (Inside)
    for (x = [-BUCKET_WIDTH/2, BUCKET_WIDTH/2]) {
        translate([x, PLATE_1_4_INCH, -BUCKET_HEIGHT/2])
        rotate([90, 0, 0])
        weld_bead(length=BUCKET_HEIGHT - 20, diameter=8);
    }
}

// =============================================================================
// BUCKET ATTACHMENT WITH PIVOT
// =============================================================================

module bucket_attachment() {
    if (show_bucket) {
        translate([0, ARM_PIVOT_Y, ARM_PIVOT_Z])
        rotate([is_undef(ARM_LIFT_ANGLE) ? 0 : ARM_LIFT_ANGLE, 0, 0])
        translate([0, ARM_TIP_X, ARM_TIP_Z])
        {
            // Bucket rotates about this pivot
            rotate([BUCKET_TILT_ANGLE, 0, 0]) {
                // New Attachment: U-Channel Lugs bolted to bucket
                // Lugs are positioned at arm spacing
                // Lug Z position must match the kinematic calculation (BUCKET_CYL_MOUNT_Z_OFFSET relative to bucket top/pivot)
                _lug_z_pos = BUCKET_CYL_MOUNT_Z_OFFSET + (BUCKET_HEIGHT - BUCKET_PIVOT_HEIGHT_FROM_BOTTOM);
                for (x_offset = [-ARM_SPACING/2, ARM_SPACING/2]) {
                    translate([x_offset, 0, _lug_z_pos])
                    rotate([-90, 0, 0]) // Rotate so base faces +Y (Bucket Back), Legs face -Y (Backwards)
                    u_channel_lug_with_pin(TUBE_3X3_1_4, 100, BUCKET_PIVOT_PIN_DIA + 2);
                }

                // Bucket
                // Shift bucket Y to match lug height so back plate touches lugs
                // Keep Z offset to maintain pivot height relative to bucket
                translate([0, BUCKET_LUG_OFFSET, BUCKET_HEIGHT - BUCKET_PIVOT_HEIGHT_FROM_BOTTOM])  
                bucket();
            }
        }
    }
}

// =============================================================================
// LIFT CYLINDERS
// =============================================================================

module lift_cylinders() {
    if (show_hydraulics) {
        // Base attachment point (on each side wall, between sandwich plates)
        base_x_offset = TRACK_WIDTH/2;  // At each side wall
        
        // Arm attachment point (in arm local coordinates)
        arm_local_attach = [0, LIFT_CYL_ARM_OFFSET, -ARM_TUBE_SIZE[0]/2 - 30];
        
        // Calculate actual extension based on current arm angle
        current_cyl_length = lift_cyl_length(ARM_LIFT_ANGLE);
        lift_extension = current_cyl_length - LIFT_CYL_LEN_MIN;
        
        for (side = [-1, 1]) {
            // Base point on frame - mounted between sandwich plates on side wall
            base_pt = [side * base_x_offset, LIFT_CYL_BASE_Y, LIFT_CYL_BASE_Z];
            
            // Calculate arm attachment in world coordinates
            arm_local = [side * ARM_SPACING/2 + arm_local_attach[0], 
                        arm_local_attach[1], 
                        arm_local_attach[2]];
            
            // Rotate by arm angle and add pivot offset
            arm_rotated = rot_x(arm_local, ARM_LIFT_ANGLE);
            arm_world = [arm_rotated[0], 
                        ARM_PIVOT_Y + arm_rotated[1], 
                        ARM_PIVOT_Z + arm_rotated[2]];
            
            // Orient and place cylinder with calculated extension
            oriented_cylinder(base_pt, arm_world, 
                            LIFT_CYLINDER_BORE, LIFT_CYLINDER_ROD, 
                            LIFT_CYLINDER_STROKE, lift_extension);
        }
    }
}

// =============================================================================
// BUCKET TILT CYLINDERS
// =============================================================================

module bucket_cylinders() {
    if (show_hydraulics) {
        // Cylinder base on cross beam (arm local coordinates, at cross beam position)
        // Now uses BUCKET_CYL_X_SPACING for proper clearance from side walls
        arm_attach_local = [0, CROSS_BEAM_1_POS, CROSS_BEAM_MOUNT_Z_OFFSET];
        
        // Cylinder rod end on bucket (bucket local coordinates, before tilt)
        // Must match the translation in bucket_attachment
        bucket_attach_local = [0, 0, BUCKET_CYL_MOUNT_Z_OFFSET + (BUCKET_HEIGHT - BUCKET_PIVOT_HEIGHT_FROM_BOTTOM)];
        
        // Calculate actual extension based on current arm angle and bucket tilt
        current_bucket_cyl_length = bucket_cyl_length(ARM_LIFT_ANGLE, BUCKET_TILT_ANGLE);
        bucket_extension = current_bucket_cyl_length - BUCKET_CYL_LEN_MIN;
        
        for (side = [-1, 1]) {
            // Use BUCKET_CYL_X_SPACING to ensure clearance from inner panels
            arm_side_offset = side * BUCKET_CYL_X_SPACING;
            
            // Arm/cross beam attachment point
            arm_local = [arm_side_offset + arm_attach_local[0],
                        arm_attach_local[1],
                        arm_attach_local[2]];
            
            // Transform to world (rotate by arm lift angle)
            arm_rotated = rot_x(arm_local, ARM_LIFT_ANGLE);
            arm_world = [arm_rotated[0],
                        ARM_PIVOT_Y + arm_rotated[1],
                        ARM_PIVOT_Z + arm_rotated[2]];
            
            // Bucket attachment - first apply tilt, then arm rotation
            bucket_local = [arm_side_offset + bucket_attach_local[0],
                           bucket_attach_local[1],
                           bucket_attach_local[2]];
            
            // Tilt about X at arm tip
            bucket_tilted = rot_x(bucket_local, BUCKET_TILT_ANGLE);
            
            // Position relative to arm tip
            bucket_at_arm_tip = [bucket_tilted[0],
                                ARM_TIP_X + bucket_tilted[1],
                                ARM_TIP_Z + bucket_tilted[2]];
            
            // Rotate by arm angle
            bucket_rotated = rot_x(bucket_at_arm_tip, ARM_LIFT_ANGLE);
            bucket_world = [bucket_rotated[0],
                           ARM_PIVOT_Y + bucket_rotated[1],
                           ARM_PIVOT_Z + bucket_rotated[2]];
            
            // Place cylinder with calculated extension
            oriented_cylinder(arm_world, bucket_world,
                            BUCKET_CYLINDER_BORE, BUCKET_CYLINDER_ROD,
                            BUCKET_CYLINDER_STROKE, bucket_extension);
        }
    }
}

// =============================================================================
// FOLDING STANDING PLATFORM
// =============================================================================
// 5-component folding platform system:
//   - 1x deck plate (CNC cut)
//   - 2x pivot bracket plates (CNC cut)
//   - 2x angle iron arms (purchased stock, drilled)
//
// Folds from stowed (vertical against rear crossmember) to deployed (horizontal)
// Uses 1" pivot pin and 3/8" cotter pin for locking in both positions

module platform_transverse_angle(length) {
    angle_bolt_hole_dia = PLATFORM_BOLT_DIA + PLATFORM_BOLT_CLEARANCE;
    leg = PLATFORM_ANGLE_LEG;
    thick = PLATFORM_ANGLE_THICK;
    
    // L-profile with corner at top (Z=0)
    // Vertical Leg extends Down (-Z)
    // Horizontal Leg extends Y
    
    difference() {
        union() {
            // Vertical Leg (Full Length)
            // Face at Y=0. Thickness along Y.
            translate([-length/2, 0, -leg])
            cube([length, thick, leg]);
            
            // Horizontal Leg
            // Face at Z=0. Thickness along Z.
            translate([-length/2, 0, -thick])
            cube([length, leg, thick]);
        }
        
        // Deck Mounting Holes (Vertical through Horizontal Leg)
        // 3 bolts: Near ends and Center
        for (x_pos = [-length/2 + PLATFORM_TRANSVERSE_BOLT_END_OFFSET, 0, length/2 - PLATFORM_TRANSVERSE_BOLT_END_OFFSET]) {
            translate([x_pos, leg/2, 0])
            cylinder(d=angle_bolt_hole_dia, h=thick*3, center=true, $fn=100);
        }
    }
}

module platform_angle_iron(length=0, is_side=false) {
    angle_bolt_hole_dia = PLATFORM_BOLT_DIA + PLATFORM_BOLT_CLEARANCE;
    // Use provided length or default
    height = (length > 0) ? length : (PLATFORM_ARM_LENGTH - PLATFORM_BRACKET_WIDTH/2);
    
    difference() {
        // Extrusion
        linear_extrude(height=height)
        polygon([
            [0, 0],
            [PLATFORM_ANGLE_LEG, 0],
            [PLATFORM_ANGLE_LEG, PLATFORM_ANGLE_THICK],
            [PLATFORM_ANGLE_THICK, PLATFORM_ANGLE_THICK],
            [PLATFORM_ANGLE_THICK, PLATFORM_ANGLE_LEG],
            [0, PLATFORM_ANGLE_LEG]
        ]);
        
        if (is_side) {
            // Pivot Holes (Start) - Horizontal Holes through Vertical Leg (Leg 1)
            // Leg 1 is along X. Normal Y. Holes along Y.
            // 2 holes for bolts into the L shaped pivot plate
            
            // Replicate calculations to align with platform_pivot_bracket
            
            // 1. Calculate side_angle_start_y (World Y of Angle Iron Start)
            // From assembly:
            deck_pivot_y = PLATFORM_BRACKET_WIDTH/2;
            front_angle_corner_y = deck_pivot_y - PLATFORM_EDGE_MARGIN - PLATFORM_ANGLE_LEG;
            gap = 12.7;
            side_angle_start_y = front_angle_corner_y - gap;
            
            // 2. Calculate bolt positions
            // Bracket bolts are on Leg 2 (File X -> Assembly -Y).
            // Bracket Y is constant `bolt_y`.
            // Bracket X varies: bolt_x_1, bolt_x_2.
            
            corner_y = -PLATFORM_LOCK_OFFSET + PLATFORM_SIDE_BOLT_Y_OFFSET;
            bracket_width = PLATFORM_BRACKET_WIDTH;
            leg2_length = PLATFORM_BRACKET_LENGTH - bracket_width/2;
            bx_1 = pivot_bolt_offset;
            bx_2 = leg2_length - pivot_bolt_offset;
            
            // 3. Map to Angle Iron Coordinates
            // Angle Iron Local Z (Longitudinal) corresponds to Assembly -Y.
            // Angle Iron Local X (Leg 1) corresponds to Assembly Z.
            
            // Longitudinal Position (z_pos)
            // Matches Bracket X (bx).
            // z_pos = bx.
            // Wait. Bracket X is distance from Pivot (along Leg 2).
            // Angle Iron Z is distance from Angle Start.
            // Angle Start is at side_angle_start_y.
            // Pivot is at 0 (in Y).
            // But Angle Iron is translated by side_angle_z_trans.
            // side_angle_z_trans = PLATFORM_LOCK_OFFSET - side_angle_start_y.
            // This is confusing.
            // Let's use the fact that Bracket X=0 is at Corner.
            // Corner is at Y = corner_y (in Bracket Local Y).
            // But Bracket Leg 2 starts at X=0.
            // In Assembly, Bracket Leg 2 starts at Pivot Y? No.
            // Bracket Leg 2 starts at Corner Y?
            // Bracket Leg 2 is the Horizontal Leg.
            // It extends from Corner (Y=-130) backwards?
            // No, Bracket Leg 2 is along Local X.
            // Local X maps to World -Y.
            // So it extends from Y=0 (Pivot) to Y=-Length?
            // No, Bracket Local X=0 is at Pivot Y?
            // In `platform_pivot_bracket`, X=0 is at Pivot X.
            // So Bracket Leg 2 starts at Pivot X (which is 0).
            // So Bracket X is distance from Pivot.
            // So bx is distance from Pivot.
            
            // Angle Iron Start is at side_angle_start_y (World Y).
            // Angle Iron extends along -Y.
            // So Angle Iron Z=0 is at side_angle_start_y.
            // Angle Iron Z=z_pos is at Y = side_angle_start_y - z_pos.
            // We want Y = -bx (since Bracket X maps to -Y).
            // side_angle_start_y - z_pos = -bx.
            // z_pos = side_angle_start_y + bx.
            
            z_pos_1 = side_angle_start_y + bx_1;
            z_pos_2 = side_angle_start_y + bx_2;
            
            // Vertical Position (x_pos on profile)
            // Matches Bracket Y (bolt_y).
            // Bracket Y maps to World Z.
            // Angle Iron X maps to World Z? No.
            // Angle Iron Local X (Leg 1) -> Frame -Y -> World -Z.
            // Angle Iron Local Y (Leg 2) -> Frame X -> World X.
            // Angle Iron Local Z -> Frame Z -> World -Y.
            
            // Wait. If Angle Iron Local X -> World -Z.
            // Then Angle Iron X is Vertical (Down).
            // Bracket Y is Vertical (Up/Down).
            // Bracket Y = corner_y - width/2 + LEG/2.
            // This is a negative value (e.g. -155).
            // Angle Iron Origin Z is PLATFORM_ANGLE_Z_OFFSET.
            // Angle Iron X is distance DOWN from Origin.
            // World Z = Origin Z - Angle_X.
            // Angle_X = Origin Z - World Z.
            // Angle_X = PLATFORM_ANGLE_Z_OFFSET - bolt_y.
            
            // Substitute PLATFORM_ANGLE_Z_OFFSET = corner_y - width/2 + LEG.
            // Substitute bolt_y = corner_y - width/2 + LEG/2.
            // Angle_X = (corner_y - width/2 + LEG) - (corner_y - width/2 + LEG/2).
            // Angle_X = LEG - LEG/2 = LEG/2.
            
            x_pos = PLATFORM_ANGLE_LEG/2;
            
            for (z_pos = [z_pos_1, z_pos_2]) {
                translate([x_pos, PLATFORM_ANGLE_THICK/2, z_pos])
                rotate([90, 0, 0]) // Align with Y axis
                cylinder(d=angle_bolt_hole_dia, h=PLATFORM_ANGLE_THICK + 10, center=true, $fn=100);
            }
            
            // Deck Holes (Middle) - Vertical Holes through Horizontal Leg (Leg 2)
            // Leg 2 is along Y. Normal X. Holes along X.
            // 2 bolts upward connecting to the platform itself
            // Symmetrical from end to end
            // Offset from ends: PLATFORM_SIDE_DECK_BOLT_END_OFFSET
            // If not defined, use 1/4 length or similar.
            // Let's use a fixed offset from ends.
            // deck_bolt_offset = 50; // Removed local override to use global parameter
            for (z_pos = [deck_bolt_offset, height - deck_bolt_offset]) {
                translate([PLATFORM_ANGLE_THICK/2, PLATFORM_ANGLE_LEG/2, z_pos])
                rotate([0, 90, 0]) // Align with X axis
                cylinder(d=angle_bolt_hole_dia, h=PLATFORM_ANGLE_THICK + 10, center=true, $fn=100);
            }
        } else {
            // Transverse Angle Iron
            // Deck Holes only
            // 2 bolts: Near ends
            // Note: Transverse angle iron logic is handled by platform_transverse_angle module above
            // This else block might be unused if platform_transverse_angle is used instead
            // But if it IS used, we should update it too.
            for (z_pos = [PLATFORM_TRANSVERSE_BOLT_END_OFFSET, height - PLATFORM_TRANSVERSE_BOLT_END_OFFSET]) {
                translate([PLATFORM_ANGLE_THICK/2, PLATFORM_ANGLE_LEG/2, z_pos])
                rotate([0, 90, 0]) // Align with X axis
                cylinder(d=angle_bolt_hole_dia, h=PLATFORM_ANGLE_THICK + 10, center=true, $fn=24);
            }
        }
    }
}

/**
 * Folding platform assembly with parametric fold angle
 * @param fold_angle Angle in degrees: 0 = stowed (vertical), 90 = deployed (horizontal)
 */
module folding_platform_assembly(fold_angle=90) {
    if (show_folding_platform) {
        // =================================================================
        // PARAMETRIC POSITION CALCULATIONS
        // =================================================================
        
        pivot_y = PLATFORM_MOUNT_Y;  // Parametric forward position
        pivot_z = PLATFORM_PIVOT_HEIGHT;
        
        // Inner wall inner face X position
        inner_wall_inner_face_x = TRACK_WIDTH/2 - SANDWICH_SPACING/2 - PANEL_THICKNESS;
        
        // Angle iron positioning
        angle_iron_clearance = PLATFORM_TRANSVERSE_GAP;
        // Align angle irons with the pivot/bolt holes defined in the deck
        // Side Angle Irons should be flush against the inner face of the pivot bracket.
        // Pivot Bracket is centered at PLATFORM_PIVOT_X.
        // Bracket Thickness is assumed to be PLATE_1_4_INCH (6.35mm).
        // Angle Iron Vertical Leg (Outer Face) should be at Bracket Inner Face.
        // Bracket Inner Face X = PLATFORM_PIVOT_X - PLATE_1_4_INCH/2.
        // So Angle Iron X = PLATFORM_PIVOT_X - PLATFORM_SIDE_GAP.
        angle_iron_x_from_center = PLATFORM_PIVOT_X - PLATFORM_SIDE_GAP;
        
        // Shift platform forward by 0.5 inches (12.7mm)
        shift_y = 0;
        
        // Additional deck shift forward (towards front of machine)
        // User requested ~2 inches more
        deck_shift_y = 0;
        
        // Transverse angles now run full width (approx)
        // Side angles fit between them with gap
        // Shorten transverse angles by gap on each side
        // Transverse Angle Length:
        // "parametrically lengthen the transverse angle irons so that they are 1/4 of an inch from the L plates on either side."
        // L Plate Inner Face is at `angle_iron_x_from_center`.
        // 1/4 inch = 6.35mm.
        transverse_len = 2 * (angle_iron_x_from_center - 6.35);
        
        // Calculate positions of transverse angles (Inner faces)
        // The outside edge of the transverse angle irons should be 1/2 inch (PLATFORM_EDGE_MARGIN)
        // from the front and back edge of the platform plate.
        
        // World Frame Y Coordinates (Relative to Pivot Group Origin):
        // Pivot is at Y=0.
        // Deck extends from +Width/2 to -(Depth - Width/2).
        // Deck Start (Pivot End) = PLATFORM_BRACKET_WIDTH/2.
        // Deck End (Far End) = -(PLATFORM_DEPTH - PLATFORM_BRACKET_WIDTH/2).
        deck_pivot_y = PLATFORM_BRACKET_WIDTH/2;
        deck_far_y = -(PLATFORM_DEPTH - PLATFORM_BRACKET_WIDTH/2);
        
        // Pivot End Transverse Angle (Front):
        // Tip of Horizontal Leg at deck_pivot_y - PLATFORM_EDGE_MARGIN.
        // Orientation: Standard (Horizontal Leg points +Y, towards Pivot Edge).
        // Corner Position = Tip - Leg.
        front_angle_corner_y = (deck_pivot_y - PLATFORM_EDGE_MARGIN) - PLATFORM_ANGLE_LEG;

        // Far End Transverse Angle (Rear):
        // Tip of Horizontal Leg at deck_far_y + PLATFORM_EDGE_MARGIN.
        // Orientation: Mirrored (Horizontal Leg points -Y, towards Far Edge).
        // Corner Position = Tip + Leg.
        rear_angle_corner_y = (deck_far_y + PLATFORM_EDGE_MARGIN) + PLATFORM_ANGLE_LEG;
        
        // Side Angle Irons (Front-to-Back):
        // Fit between the inner faces (vertical legs) of the transverse angles.
        // Front Angle Vertical Leg is at front_angle_corner_y.
        // Rear Angle Vertical Leg is at rear_angle_corner_y.
        // Gap = 1/2 inch.
        gap = 12.7; 
        side_angle_start_y = front_angle_corner_y - gap; // Start of side angle (closest to pivot)
        side_angle_end_y = rear_angle_corner_y + gap;   // End of side angle (furthest from pivot)
        side_angle_len = side_angle_start_y - side_angle_end_y;
        
        // Side angle Z offset (in local frame before fold rotation)
        // Maps to World -Y.
        // We want World Y = side_angle_start_y at Extrusion 0.
        // World Y = -(z_trans - PLATFORM_LOCK_OFFSET).
        // side_angle_start_y = -z_trans + PLATFORM_LOCK_OFFSET.
        // z_trans = PLATFORM_LOCK_OFFSET - side_angle_start_y.
        side_angle_z_trans = PLATFORM_LOCK_OFFSET - side_angle_start_y;

        // =================================================================
        // ANGLE IRON ARMS (2x - Left and Right)
        // =================================================================
        
        // LEFT SIDE angle iron
        color("DimGray")
        translate([-angle_iron_x_from_center, pivot_y, pivot_z])
        rotate([fold_angle, 0, 0])
        translate([0, PLATFORM_ANGLE_Z_OFFSET, side_angle_z_trans - PLATFORM_LOCK_OFFSET])
        rotate([0, 0, -90])
        platform_angle_iron(length=side_angle_len, is_side=true);
        
        // RIGHT SIDE angle iron
        color("DimGray")
        translate([angle_iron_x_from_center, pivot_y, pivot_z])
        rotate([fold_angle, 0, 0])
        translate([0, PLATFORM_ANGLE_Z_OFFSET, side_angle_z_trans - PLATFORM_LOCK_OFFSET])
        mirror([1, 0, 0])
        rotate([0, 0, -90])
        platform_angle_iron(length=side_angle_len, is_side=true);
        
        // =================================================================
        // TRANSVERSE ANGLE IRONS (2x - Front and Rear)
        // =================================================================
        
        // Front Transverse Angle (Pivot End)
        color("DimGray")
        translate([0, pivot_y, pivot_z])
        rotate([fold_angle - 90, 0, 0])
        // Position: Y=Depth, Z=Height
        translate([0, front_angle_corner_y, PLATFORM_ANGLE_Z_OFFSET])
        // Standard orientation points Horizontal Leg +Y (towards Pivot Edge)
        platform_transverse_angle(transverse_len);
        
        // Rear Transverse Angle (Deck End)
        color("DimGray")
        translate([0, pivot_y, pivot_z])
        rotate([fold_angle - 90, 0, 0])
        // Position: Y=Depth, Z=Height
        translate([0, rear_angle_corner_y, PLATFORM_ANGLE_Z_OFFSET])
        mirror([0, 1, 0]) // Mirror Y to point Horizontal Leg -Y (towards Far Edge)
        platform_transverse_angle(transverse_len);        // And `PLATFORM_ANGLE_Z_OFFSET` (Vertical position) SHOULD be in Y.
        // My previous analysis of `rotate([0, 0, -90])` was in the *inner* scope.
        // But the `translate` is in the *outer* scope (relative to `fold_angle`).
        // So:
        // Local X = Width.
        // Local Y = Thickness (Vertical).
        // Local Z = Depth (Rearward).
        
        // So `side_angle_z_trans` is correct in Z.
        // And `PLATFORM_ANGLE_Z_OFFSET` is correct in Y.
        
        // Now, Transverse Angles.
        // Front (Pivot End): `front_angle_pos_y`.
        // This is a Depth position. So it should be in Z (Local).
        // `translate([0, ..., front_angle_pos_y])`.
        // But previous code had `translate([0, front_angle_pos_y, ...])`.
        // Why?
        // `rotate([fold_angle - 90, 0, 0])`.
        // If fold=90, rot=0.
        // So Local Frame is World Frame.
        // X=Width, Y=Depth, Z=Height.
        // Wait, if rot=0, then Y is Depth?
        // But for Side Angles (inside `rotate([fold_angle...])`), Y was Thickness.
        // Because `rotate([90, 0, 0])` swaps Y and Z.
        // But Transverse Angles have `rotate([fold_angle - 90, 0, 0])`.
        // If fold=90, rot=0. No swap.
        // So Y is Depth, Z is Height.
        // So `front_angle_pos_y` (Depth) should be in Y.
        // And `PLATFORM_ANGLE_Z_OFFSET` (Height) should be in Z.
        
        // So:
        // Side Angles: Translate Y=Height, Z=Depth.
        // Transverse Angles: Translate Y=Depth, Z=Height.
        
        // Values:
        // `front_angle_pos_y` = -PLATFORM_EDGE_MARGIN.
        // `rear_angle_pos_y` = -(PLATFORM_DEPTH - PLATFORM_EDGE_MARGIN).
        
        // Orientation of Transverse Angles:
        // Front (Pivot): Needs Horizontal Leg along -Y (Local Depth).
        // Standard `platform_transverse_angle` (using `platform_angle_iron`):
        // `platform_angle_iron` extrudes X. Profile YZ.
        // Corner 0,0. Horiz +Y. Vert -Z.
        // We want Horiz -Y. Vert -Z.
        // So we need `mirror([0, 1, 0])`.
        
        // Rear (Far): Needs Horizontal Leg along +Y.
        // Standard is Horiz +Y. Vert -Z.
        // So this matches Standard.
        
        // Wait, `platform_transverse_angle` calls `platform_angle_iron`.
        // Does it rotate it?
        // `platform_transverse_angle` is not defined in the snippet.
        // It likely just calls `platform_angle_iron`.
        // Let's assume it's a wrapper or just `platform_angle_iron`.
        // If it's `platform_angle_iron`, we need to handle the rotation/mirroring.
        // The previous code had `rotate([0, 0, 180])` for the Rear angle.
        // `rotate([0, 0, 180])` rotates around Z.
        // X->-X, Y->-Y.
        // Extrusion (X) becomes -X.
        // Profile (Y) becomes -Y.
        // So Horiz Leg (+Y) becomes -Y.
        // This was used for Rear Angle.
        // But I calculated Rear Angle needs +Y?
        // Let's re-verify "Under Deck".
        // Deck is at Z = Offset + Thickness/2?
        // Deck `translate([0, Thickness/2 + Offset, ...])`.
        // So Deck Bottom is at Z = Offset.
        // Angle Iron Vertical Leg is -Z (from Corner).
        // So Corner must be at Z = Offset.
        // Then Vertical Leg goes down to Offset - Leg.
        // Horizontal Leg is at Z=0 (relative to Corner)? No.
        // Profile: `[leg-thick, 0], [leg-thick, -thick]`.
        // So Horizontal Leg is at Z=0 to -Thick.
        // So Top of Horiz Leg is at Z=0 (Corner).
        // So Deck sits on top of Horiz Leg. Correct.
        // Direction of Horiz Leg:
        // Front (Pivot): At Y = -Margin. Deck is at Y < -Margin.
        // So Horiz Leg must point -Y.
        // Rear (Far): At Y = -Depth + Margin. Deck is at Y > -Depth + Margin.
        // So Horiz Leg must point +Y.
        
        // So:
        // Front (Pivot): Needs -Y.
        // Rear (Far): Needs +Y.
        
        // Standard Profile: +Y.
        // So Rear needs Standard.
        // Front needs Mirror Y (or Rotate Z 180 if X-extrusion doesn't matter).
        // But X-extrusion direction matters for alignment with side angles?
        // Side angles are at +/- Width.
        // Transverse angles span between them.
        // If we rotate 180, X becomes -X.
        // So we need to translate X to compensate?
        // Or just use Mirror([0, 1, 0]).
        
        // Previous code:
        // Front: `platform_transverse_angle`. (Standard).
        // Rear: `rotate([0, 0, 180]) platform_transverse_angle`. (Rotated).
        // This implies Previous Front was +Y, Previous Rear was -Y.
        // Previous Front was at +Y (Pivot End).
        // Previous Rear was at -Y (Far End).
        // Deck was between them.
        // So Front (+Y) needed -Y to point to deck? Yes.
        // Rear (-Y) needed +Y to point to deck? Yes.
        // So Previous Front (Standard) was +Y? No, Standard is +Y.
        // So Previous Front (Standard) pointed Away from deck?
        // That seems wrong.
        // Unless `platform_transverse_angle` has built-in rotation.
        // Or my coordinate system understanding is flipped.
        
        // Let's trust my derivation:
        // Front (Pivot, Y~0): Needs -Y.
        // Rear (Far, Y~-Depth): Needs +Y.
        // Standard: +Y.
        // So Front needs Modification. Rear needs Standard.
        
        // Modification: `mirror([0, 1, 0])`.
        
        // Side Angle Length:
        // `side_angle_len`.
        // Side Angle Start (Pivot End): `transverse_pivot_y - Leg - 12.7`.
        // Side Angle End (Far End): `transverse_far_y + Leg + 12.7`.
        // Note: `transverse_pivot_y` is -Margin.
        // `transverse_far_y` is -Depth + Margin.
        // So Start = -Margin - Leg - 12.7.
        // End = -Depth + Margin + Leg + 12.7.
        // Length = Start - End = Depth - 2*Margin - 2*Leg - 25.4.
        
        // Let's write the code.


        // =================================================================
        // PIVOT BRACKET PLATES (2x - Left and Right)  
        // =================================================================
        
        for (side = [-1, 1]) {
            bracket_x = side * (inner_wall_inner_face_x - PLATFORM_THICKNESS/2);
            
            color("DarkSlateGray")
            translate([bracket_x, pivot_y, pivot_z])
            rotate([fold_angle, 0, 0])
            rotate([0, -90, 0]) // Reverted rotation: X->Z (Up), Y->Y (Back), Z->-X (Side)
            linear_extrude(height=PLATFORM_THICKNESS, center=true)
            projection(cut=true)
            platform_pivot_bracket();
        }
        
        // =================================================================
        // DECK PLATE (1x)
        // =================================================================
        
        color("DarkSlateGray")
        translate([0, pivot_y, pivot_z])
        rotate([fold_angle, 0, 0])
        translate([0, PLATFORM_THICKNESS/2 + PLATFORM_ANGLE_Z_OFFSET, PLATFORM_DEPTH/2 - PLATFORM_BRACKET_WIDTH/2])
        rotate([90, 0, 0])
        platform_deck();
        
        // =================================================================
        // PIVOT PINS (2x - visualization)
        // =================================================================
        
        // Pivot pins go through inner side panel and pivot bracket
        // Oriented in X direction (through the panel thickness)
        color("Silver")
        for (side = [-1, 1]) {
            // Pin at the inner panel location
            translate([side * PLATFORM_PIVOT_X, pivot_y, pivot_z])
            rotate([0, 90, 0])
            cylinder(d=PLATFORM_PIVOT_PIN_DIA, h=PANEL_THICKNESS * 2 + 30, center=true, $fn=32);
        }
        
        // =================================================================
        // LOCK PINS (visualization)
        // =================================================================
        
        color("Gold")
        for (side = [-1, 1]) {
            if (fold_angle > 45) {
                // Deployed position - lock pin below pivot
                translate([side * PLATFORM_PIVOT_X, pivot_y, pivot_z - PLATFORM_LOCK_OFFSET])
                rotate([0, 90, 0])
                cylinder(d=PLATFORM_LOCK_PIN_DIA, h=PANEL_THICKNESS + 20, center=true, $fn=24);
            } else {
                // Stowed position - lock pin at arm end position
                lock_y = pivot_y - PLATFORM_ARM_LENGTH * sin(fold_angle);
                lock_z = pivot_z + PLATFORM_ARM_LENGTH * cos(fold_angle);
                translate([side * PLATFORM_PIVOT_X, lock_y, lock_z])
                rotate([0, 90, 0])
                cylinder(d=PLATFORM_LOCK_PIN_DIA, h=PANEL_THICKNESS + 20, center=true, $fn=24);
            }
        }
    }
}

// Legacy module for backward compatibility (calls new folding platform)
module standing_deck() {
    folding_platform_assembly(platform_fold_angle);
}

// =============================================================================
// MAIN ASSEMBLY
// =============================================================================

module lifetrac_v25_assembly() {
    if (show_frame) {
        base_frame();
        engine();
    }
    
    wheel_assemblies();
    loader_arms();
    bucket_attachment();
    lift_cylinders();
    bucket_cylinders();
    folding_platform_assembly(platform_fold_angle);
}

// =============================================================================
// RENDER
// =============================================================================

lifetrac_v25_assembly();
calculate_cog();

// Ground plane reference removed per user request
// color("Green", 0.2)
// translate([0, WHEEL_BASE/2, -1])
// cube([4000, 4000, 2], center=true);
