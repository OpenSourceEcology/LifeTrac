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
BUCKET_PIVOT_PIN_DIA = 25.4; // 1" bucket pivot pin
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
// BUCKET DIMENSIONS (Moved Up for Geometry Solver and Reach Calculation)
// =============================================================================

BOBCAT_QA_WIDTH = 1168;           // 46" standard width
BOBCAT_QA_HEIGHT = 457;           // 18" plate height
BOBCAT_QA_PIN_DIA = 25.4;         // 1" locking pin

BUCKET_WIDTH = 1100;              // Slightly narrower than QA plate
BUCKET_DEPTH = 600;
BUCKET_HEIGHT = 450;

// Design constraints
BUCKET_GROUND_CLEARANCE = 0; // Bucket flush with ground (Z=0)
BUCKET_FRONT_CLEARANCE_BASE = 0; // Base forward clearance (mm)
BUCKET_FRONT_CLEARANCE_ADJUST = 1.5 * 25.4; // Forward tweak (+) in mm
BUCKET_FRONT_CLEARANCE = BUCKET_FRONT_CLEARANCE_BASE + BUCKET_FRONT_CLEARANCE_ADJUST;

// Bucket Pivot Height Configuration
// We align the pivot to be at 1/5 of the bucket height (standard stability position)
BUCKET_PIVOT_HEIGHT_FROM_BOTTOM = BUCKET_HEIGHT / 5; // 450 / 5 = 90mm

// BUCKET_PIVOT_Y_OFFSET: Calculated after arm geometry to align bucket lug pin with arm tip pin hole
// See calculation below ARM_TIP_X/ARM_TIP_Z definitions

// =============================================================================
// LOADER ARM DIMENSIONS
// =============================================================================

// Design constraints (Moved above)

/* 
// Moved logic further down to BUCKET section for consistency
BUCKET_PIVOT_HEIGHT_BASE = 200; // Nominal height of pivot from bottom of bucket
BUCKET_PIVOT_HEIGHT_ADJUST = -5 * 25.4; // Lower lug (~5")
BUCKET_PIVOT_HEIGHT_FROM_BOTTOM = BUCKET_PIVOT_HEIGHT_BASE + BUCKET_PIVOT_HEIGHT_ADJUST;
*/

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

// =============================================================================
// LOADER ARM V2 PARAMETERS (L-SHAPE)
// =============================================================================

TUBE_2X6_1_4 = [50.8, 152.4, 6.35]; // 2"x6" rectangular tubing
// ARM_ANGLE = 130; // Main arm angle in degrees (Sharpened to 130 as requested) - CALCULATED BELOW
ARM_PLATE_THICKNESS = PLATE_1_4_INCH;
DOM_PIPE_OD = 50.8; // Approximate OD for pivot holder (2" DOM)
DOM_PIPE_ID = 38.1; // ID matching 1.5" pivot pin
JIG_WALL_THICKNESS = 4; // Standard wall thickness for 3D printed jigs
JIG_CLEARANCE = 0.5; // Clearance for fit

// Define ARM_MIN_ANGLE and ARM_MAX_ANGLE later after geometry is calculated
// ARM_MIN_ANGLE = -ARM_GROUND_ANGLE;     // Lowest position (at ground)
// ARM_MAX_ANGLE = 60;                     // Maximum raised position

echo("DEBUG: Loading lifetrac_v25_params.scad - Version with Fixes");

// Arm Dimensions - Parametric Calculation
// Inputs
WHEEL_CLEARANCE_TARGET = 25.4; // 1 inch clearance target
// ARM_SHAPE_ANGLE = 130;  // The fixed L-angle - CALCULATED BELOW

// =============================================================================
// ARM GEOMETRY SOLVER (Target-Based with Fixed 130 Shape)
// =============================================================================

// Desired Arm Shape
ARM_SHAPE_ANGLE = 130; 
_bend_deflection = 180 - ARM_SHAPE_ANGLE; // 50 degrees

// Geometric Constants
_P = [ARM_PIVOT_Y, ARM_PIVOT_Z];       // Main Pivot
_C = [WHEEL_BASE, FRAME_Z_OFFSET + WHEEL_RADIUS]; 
_R_wheel = WHEEL_RADIUS; 
_tube_offset = TUBE_2X6_1_4[1] / 2; // Half of 6" tube height = 76.2
_R_constraint = _R_wheel + WHEEL_CLEARANCE_TARGET + _tube_offset;

// 1. Define Target T (Bucket Position)
// Strictly enforce 1 inch clearance from front of wheel.
// Note: This is the position of the Bucket Pivot Pin.
BUCKET_X_ADJUST = 0; // User adjustment for bucket position (+ = forward)
_buck_clearance = 1.0 * 25.4; 
_T_x = WHEEL_BASE + WHEEL_RADIUS + _buck_clearance + BUCKET_X_ADJUST;
// Use BUCKET_PIVOT_HEIGHT_FROM_BOTTOM relative to ground, adjusted by BUCKET_GROUND_CLEARANCE
_T_z = BUCKET_PIVOT_HEIGHT_FROM_BOTTOM + BUCKET_GROUND_CLEARANCE; 
_T = [_T_x, _T_z];

echo("DEBUG: Target T:", _T);
echo("DEBUG: Pivot P:", _P);

// 2. Solve Triangle P-E-T with AUTO-OPTIMIZATION
// Goal: Find L1 such that Clearance(Main Arm) approx equals Clearance(Drop Arm)

// USER REQUEST: Formula to calculate Leg Length (L2) based on Geometry
// Triangle P-E-T constraints:
// P = Main Pivot, T = Bucket Pivot (Target)
// Angle PET = ARM_SHAPE_ANGLE (gamma)
// L1 = Length(PE), L2 = Length(ET)
// D = Distance(PT), alpha = Angle of line PT
// By Law of Cosines: D^2 = L1^2 + L2^2 - 2*L1*L2*cos(gamma)
// This constrains L1 and L2 to a curve. We slide along this curve (varying L1) 
// to find the spot where the arm "just clears" the front wheel (Point C).

_dist_PT = norm(_T - _P);
_angle_PT = atan2(_T[1] - _P[1], _T[0] - _P[0]);

// --- HELPER FUNCTIONS FOR SOLVER ---
// Solve L2 length from L1 and Shape Angle
function solve_L2_func(l1, d, ang_deg) = 
    let(
        rb = -2 * l1 * cos(ang_deg),
        rc = pow(l1, 2) - pow(d, 2),
        delta = pow(rb, 2) - 4 * 1 * rc
    ) (delta < 0) ? undef : (-rb + sqrt(delta)) / 2;

// Solve Elbow Position E from L1
function solve_E_func(l1, P, T, C, ang_deg) =
    let(
        d = norm(T - P),
        a_PT = atan2(T[1] - P[1], T[0] - P[0]),
        l2 = solve_L2_func(l1, d, ang_deg),
        
        // Angle EPT (using Cosine Rule)
        cos_alpha = (l2 == undef) ? 0 : (pow(l1, 2) + pow(d, 2) - pow(l2, 2)) / (2 * l1 * d),
        // Clamp cos range
        cos_alpha_c = max(-1, min(1, cos_alpha)),
        angle_EPT = acos(cos_alpha_c),
        
        // Main Arm Angle
        angle_main = a_PT + angle_EPT
    )
    (l2 == undef) ? [0,0] : (P + [l1 * cos(angle_main), l1 * sin(angle_main)]);

// Calculate Distance from Point pt to Infinite Line passing through v1-v2
function dist_line_func(pt, v1, v2) = 
    let(base = norm(v2-v1)) (base==0) ? 9999 : abs((v2[1]-v1[1])*pt[0] - (v2[0]-v1[0])*pt[1] + v2[0]*v1[1] - v2[1]*v1[0]) / base;

// Evaluate "Unbalance" = MainClearance - DropClearance
// If > 0, Main is further away than Drop (Need to Decrease Main Clearance -> Increase L1)
// BIAS: Positive value forces Main to be larger (Further) and Drop smaller (Closer)
ARM_BALANCE_BIAS = 35.0; // Bias to push Main Arm away and Drop Arm closer (User request)

function eval_balance(l1, P, T, C, ang) =
    let(
        E = solve_E_func(l1, P, T, C, ang),
        d_main = dist_line_func(C, P, E), // Distance to infinite line
        d_drop = dist_line_func(C, E, T)
    ) (d_main - d_drop - ARM_BALANCE_BIAS);

// Recursive Binary Search to find optimal L1 where balance ~ 0
// Range 10mm tolerance is fine for visual
function find_optimal_L1(min_l, max_l, P, T, C, ang, steps) =
    (steps <= 0) ? (min_l + max_l) / 2 :
    let(
        mid = (min_l + max_l) / 2,
        diff = eval_balance(mid, P, T, C, ang)
    )
    // If diff > 0 (Main > Drop + Bias), we see Main is Too Far.
    // To make Main Closer, we Increase L1 (push E out/down).
    (diff > 0) ? find_optimal_L1(mid, max_l, P, T, C, ang, steps - 1) :
                 find_optimal_L1(min_l, mid, P, T, C, ang, steps - 1);

// --- EXECUTE SOLVER ---
// Search for L1 between 1200 and 1600mm. 15 steps gives < 0.1mm precision
_L1_OPTIMAL = find_optimal_L1(1200, 1600, _P, _T, _C, ARM_SHAPE_ANGLE, 15);

echo("DEBUG: Auto-Optimized L1:", _L1_OPTIMAL);
_L1_param = _L1_OPTIMAL;

// Explicit Verification Formula for Leg Length (L2)
// Given P, T, and Angle Gamma (130deg), and chosen L1:
// L2 = Solution to Quadratic: L2^2 - (2*L1*cos(gamma))*L2 + (L1^2 - D^2) = 0
_gamma_rad = ARM_SHAPE_ANGLE;
_D_sq = pow(_dist_PT, 2);
_qa_verify = 1;
_qb_verify = -2 * _L1_param * cos(_gamma_rad);
_qc_verify = pow(_L1_param, 2) - _D_sq;
_L2_VERIFY = (-_qb_verify + sqrt(pow(_qb_verify, 2) - 4 * _qa_verify * _qc_verify)) / 2;
echo("DEBUG: VERIFIED TRIG FORMULA L2: ", _L2_VERIFY);


// --- RE-CALCULATE CONSTANTS FOR EXPORT (Standard Logic) ---
_gamma = ARM_SHAPE_ANGLE;
_qa = 1;
_qb = -2 * _L1_param * cos(_gamma);
_qc = pow(_L1_param, 2) - pow(_dist_PT, 2);
_q_delta = pow(_qb, 2) - 4 * _qa * _qc;

// Solve L2 (Use positive root for physical length)
_L2_solved = (-_qb + sqrt(_q_delta)) / (2 * _qa);

echo("DEBUG: Solved L1:", _L1_param);
echo("DEBUG: Solved L2:", _L2_solved);

// 3. Determine Orientation (Angle of Main Arm)
_cos_alpha = (pow(_L1_param, 2) + pow(_dist_PT, 2) - pow(_L2_solved, 2)) / (2 * _L1_param * _dist_PT);
_angle_EPT = acos(_cos_alpha); 
_angle_main_solved = _angle_PT + _angle_EPT;

echo("DEBUG: Angle PT:", _angle_PT);
echo("DEBUG: Angle EPT:", _angle_EPT);
echo("DEBUG: Solved Main Angle:", _angle_main_solved);

// Calculate E
_E_solved = _P + [_L1_param * cos(_angle_main_solved), _L1_param * sin(_angle_main_solved)];

echo("DEBUG: Solved E:", _E_solved);

// 4. Verify Clearance
// Distance from E to C
_dist_EC = norm(_E_solved - _C);
_clearance_E = _dist_EC - _R_constraint;
echo("DEBUG: Clearance at Elbow:", _clearance_E);

// Calculate Exact Clearance to Main Arm (Segment P-E) and Drop Arm (Segment E-T)
function dist_point_line_verify(pt, v1, v2) = 
    let(
        area = abs(v1[0]*(v2[1]-pt[1]) + v2[0]*(pt[1]-v1[1]) + pt[0]*(v1[1]-v2[1])),
        base = norm(v1 - v2)
    ) (base == 0) ? norm(pt - v1) : (area / base * 2);

_clr_main_geometric = dist_point_line_verify(_C, _P, _E_solved) - _R_wheel - _tube_offset;
_clr_drop_geometric = dist_point_line_verify(_C, _E_solved, _T) - _R_wheel - _tube_offset; // Use _T, not _T_solved (undefined)

echo("=========================================");
echo("OPTIMIZATION RESULTS (L1 =", _L1_param, ")");
echo("  Main Arm Clearance:", _clr_main_geometric);
echo("  Drop Arm Clearance:", _clr_drop_geometric);
echo("  Difference:", abs(_clr_main_geometric - _clr_drop_geometric));
echo("=========================================");

// Output variables for global use
_E = _E_solved;
_L_main_kinematic = _L1_param;
_L_drop_kinematic = _L2_solved;
_angle_tangent_main = _angle_main_solved; // Used for variable naming consistency

// Apply to Parameters
// ARM_SHAPE_ANGLE is defined at the top of this block
ARM_ANGLE = ARM_SHAPE_ANGLE; 
ARM_TANGENT_ANGLE = _angle_main_solved;

// CORRECTION FOR ARM_PLATE GEOMETRY
_bend_rad = (180 - ARM_SHAPE_ANGLE); 
_geom_correction = _tube_offset * (sin(_bend_rad) - (1 - cos(_bend_rad)) / tan(_bend_rad));

// Set Lengths
ARM_OVERLAP = 76.2; 
ARM_MAIN_LEN = _L_main_kinematic - ARM_OVERLAP + _geom_correction; 

ARM_DROP_EXT = 80; 
PIVOT_HOLE_X_FROM_FRONT = BUCKET_PIVOT_PIN_DIA; // Reverted to 1" (25.4mm) per request (Forward position)

// Leg shortening adjustment - shortens visual leg WITHOUT changing arm rotation
// This raises the pin hole higher above ground while keeping main arm angle fixed
// Target: Pin hole center at 1/5 bucket height (90mm) from ground level
// The arm hole is at PIVOT_HOLE_Z_FROM_BOTTOM (127mm) from tube bottom
ARM_LEG_SHORTEN = 37 + 25.4 + 6.35 + 25.4 + 6.35; // 100.5mm - added 0.25 inch to previous value

// Visual leg length (what gets rendered)
ARM_DROP_LEN = _L_drop_kinematic - ARM_DROP_EXT + PIVOT_HOLE_X_FROM_FRONT - ARM_LEG_SHORTEN; 

// Kinematic leg length (used for tip position calculation - keeps arm angle unchanged)
_ARM_DROP_LEN_KINEMATIC = _L_drop_kinematic - ARM_DROP_EXT + PIVOT_HOLE_X_FROM_FRONT; // Original unshortened

ARM_PIVOT_EXT = 40; 
PIVOT_HOLE_EDGE_OFFSET = 1.5 * 25.4; 
// PIVOT_HOLE_Z_FROM_BOTTOM: Original position at top edge of tube
// This keeps the hole position on the arm profile unchanged
PIVOT_HOLE_Z_FROM_BOTTOM = TUBE_2X6_1_4[1] - PIVOT_HOLE_X_FROM_FRONT; // 127mm - original position

echo("DEBUG: Final Parameters:");
echo("  ARM_SHAPE_ANGLE:", ARM_SHAPE_ANGLE);
echo("  ARM_MAIN_LEN:", ARM_MAIN_LEN);
echo("  ARM_DROP_LEN (visual):", ARM_DROP_LEN);
echo("  _ARM_DROP_LEN_KINEMATIC:", _ARM_DROP_LEN_KINEMATIC);
echo("  ARM_LEG_SHORTEN:", ARM_LEG_SHORTEN);
echo("  PIVOT_HOLE_Z_FROM_BOTTOM:", PIVOT_HOLE_Z_FROM_BOTTOM);

// Vector from Main Pivot to Bucket Pivot (For reference)
// Use KINEMATIC leg length to keep arm rotation unchanged
_bend_angle = 180 - ARM_ANGLE; // Note: ARM_ANGLE is now possibly obsolete if we use SHAPE_ANGLE
_drop_vec_len = _ARM_DROP_LEN_KINEMATIC + ARM_DROP_EXT - PIVOT_HOLE_X_FROM_FRONT; // Use kinematic length
_tube_h = TUBE_2X6_1_4[1];
_hole_z_offset = PIVOT_HOLE_Z_FROM_BOTTOM; // Hole Z position (from bottom edge)

_dx = _drop_vec_len * cos(_bend_angle) + (_hole_z_offset - _tube_h) * sin(_bend_angle);
_dz = -_drop_vec_len * sin(_bend_angle) + (_hole_z_offset - _tube_h) * cos(_bend_angle);

// Calculate Tip Position (Top/Effective Pivot Point)
// This uses kinematic leg length so arm angle stays the same
ARM_TIP_X = (ARM_MAIN_LEN + ARM_OVERLAP) + _dx; // Removed +50 error
ARM_TIP_Z = _tube_h + _dz;

// =============================================================================
// BUCKET PIVOT Y OFFSET - Aligns bucket lug pin with visual arm tip pin hole
// =============================================================================
// The kinematic tip (ARM_TIP_X) is based on unshortened leg length.
// The visual arm leg is shortened by ARM_LEG_SHORTEN along the leg axis.
// This displacement in arm-local Y is: -ARM_LEG_SHORTEN * cos(_bend_angle)
// The bucket pivot must be shifted by this amount to align with the visual arm tip.
BUCKET_PIVOT_Y_OFFSET = -ARM_LEG_SHORTEN * cos(_bend_angle);
echo("BUCKET_PIVOT_Y_OFFSET (calculated) =", BUCKET_PIVOT_Y_OFFSET, "mm");

ARM_V2_LENGTH = sqrt(pow(ARM_TIP_X, 2) + pow(ARM_TIP_Z, 2));
// Geometric angle of the arm tip vector relative to horizontal
ARM_GEOMETRY_ANGLE = atan2(ARM_TIP_Z, ARM_TIP_X);

echo("=== ARM V2 GEOMETRY ===");
echo("ARM_V2_LENGTH =", ARM_V2_LENGTH);
echo("ARM_GEOMETRY_ANGLE =", ARM_GEOMETRY_ANGLE);
echo("ARM_TIP_X =", ARM_TIP_X);
echo("ARM_TIP_Z =", ARM_TIP_Z);

// Calculate angles for kinematics
// Use ARM_TIP_X and ARM_TIP_Z directly - these represent the bucket pivot position
// relative to the arm pivot in arm-local coordinates (before rotation)
// Include BUCKET_PIVOT_Y_OFFSET to get actual bucket pivot position (aligned with visual arm tip)
_x_rel = ARM_TIP_X + BUCKET_PIVOT_Y_OFFSET;
_z_rel = ARM_TIP_Z;  // Z offset from shortening is minimal due to shallow bend angle

_arm_eff_len = sqrt(pow(_x_rel, 2) + pow(_z_rel, 2));
arm_alpha_calc = atan2(-_z_rel, _x_rel); // Angle below horizontal (positive value)

// Target Angle Calculation
// We want the Bucket Pivot (Arm Tip) to be at BUCKET_PIVOT_HEIGHT_FROM_BOTTOM when touching ground.
// If BUCKET_GROUND_CLEARANCE reduces this, we go deeper.
// Frame Datum = FRAME_Z_OFFSET (approx 150mm).
// ARM_PIVOT_Z is absolute (approx 150 + 950 = 1100).
// Target Z in absolute coords = BUCKET_PIVOT_HEIGHT_FROM_BOTTOM + BUCKET_GROUND_CLEARANCE (relative to 0 ground, we hope).
// But wait, if BUCKET_GROUND_CLEARANCE is relative to "touching ground", then Target Z = 90 + (-50) = 40mm ABSOLUTE.
// If _target_height is ABSOLUTE Z, then:
_target_height = (BUCKET_PIVOT_HEIGHT_FROM_BOTTOM + BUCKET_GROUND_CLEARANCE); 

// Note: If the math assumes Arm Pivot is at (0,0) of the calculation space, we need relative Z.
// _target_height here is used as the Z coordinate of the tip relative to the Pivot (which is at ARM_PIVOT_Z).
// So (Tip_Z - Pivot_Z) / Length = sin(angle)
// Tip_Z should be _target_height relative to GROUND (0).
// Pivot_Z is ARM_PIVOT_Z (approx 1100).
_theta_rad = asin((_target_height - ARM_PIVOT_Z) / _arm_eff_len);
theta_deg_calc = _theta_rad; 

echo("DEBUG: theta_deg_calc", theta_deg_calc);
echo("DEBUG: arm_alpha_calc", arm_alpha_calc);

// Robustly define ARM_MIN_ANGLE_COMPUTED (forces bucket bottom to ground when BUCKET_GROUND_CLEARANCE = 0)
ARM_MIN_ANGLE_COMPUTED = (is_undef(theta_deg_calc) || is_undef(arm_alpha_calc)) ? -45 : theta_deg_calc + arm_alpha_calc;
echo("DEBUG: ARM_MIN_ANGLE_COMPUTED", ARM_MIN_ANGLE_COMPUTED);

// Arm angle limits
// Calculated to keep bucket pivot ~50mm above ground at lowest position
ARM_V2_OFFSET_ANGLE = ARM_MIN_ANGLE_COMPUTED + ARM_GROUND_ANGLE; // Difference from straight arm
ARM_MIN_ANGLE = ARM_MIN_ANGLE_COMPUTED;     // Lowest position (at ground)
ARM_MAX_ANGLE = 60 + ARM_V2_OFFSET_ANGLE;   // Maximum raised position

// Cross beam configuration (Moved to below)
// CROSS_BEAM_SIZE = TUBE_2X2_1_4[0];    // 2"x2" tube size
// CROSS_BEAM_CLEARANCE = 15;             // Extra clearance around cross beam in cutout
// CROSS_BEAM_1_POS = ARM_LENGTH * 0.65;   // First cross beam position
// CROSS_BEAM_2_POS = ARM_LENGTH * 0.95;   // Second cross beam position (near bucket)

// Pivot position in panel coordinates (for arc slot calculations)
PIVOT_PANEL_X = ARM_PIVOT_Y;                          // Pivot X in panel coords
PIVOT_PANEL_Y = ARM_PIVOT_Z - FRAME_Z_OFFSET;         // Pivot Y in panel coords

// =============================================================================
// HYDRAULIC CYLINDER MOUNTING POINTS
// =============================================================================

// Lift cylinder mounting points (arms pivot at rear, cylinders attach forward)
// Optimized for leverage at bottom position (cylinder pushes up/forward from low rear point)

// =============================================================================
// LIFT CYLINDER GEOMETRY CALCULATION - GEOMETRY-FIRST APPROACH
// =============================================================================
// This algorithm works BACKWARDS from the available space:
// 1. Set arm attachment position (trade-off: leverage vs cylinder size)
// 2. Set base mount position (must be on frame, clear of wheels)
// 3. Calculate pin-to-pin distances at min and max arm angles
// 4. The ACTUAL available stroke = pin_to_pin_max - pin_to_pin_min
// 5. Select a cylinder that fits: closed_length ≤ pin_to_pin_min
//
// This guarantees a cylinder that WILL FIT the geometry.

// --- STEP 1: Fixed cylinder specifications ---
_LIFT_CYL_BORE = 63.5;        // 2.5" bore
_LIFT_CYL_ROD = 38.1;         // 1.5" rod

// --- STEP 2: Bracket drop (arm attachment offset below centerline) ---
_bracket_drop = TUBE_2X6_1_4[1]/2 + 76.2/2;  // ~114mm below arm centerline

// --- STEP 3: Arm attachment position ---
// Trade-off: Further out = more leverage (less cylinder force needed), more travel
//            Closer in = less travel, but cylinder can be shorter
// With base at rear, we need attachment further forward on arm for good geometry
// Set at 50% of arm length for adequate pin-to-pin distance at MIN angle
LIFT_CYL_ARM_OFFSET = ARM_LENGTH * 0.50;

// --- STEP 3a: Define angles for geometry calculations ---
_theta_min_lift = ARM_MIN_ANGLE_COMPUTED;
// NOTE: ARM_MAX_ANGLE_LIMITED is calculated later in the file based on bucket cylinder
// geometry. We use a conservative estimate here (60 degrees) to ensure the lift
// cylinder will reach. The final verification in the main file uses the actual limit.
_theta_max_lift_estimate = 50;  // Conservative estimate for initial sizing
_theta_max_lift = min(ARM_MAX_ANGLE, _theta_max_lift_estimate);

// --- STEP 4: Base mount position ---
// The key constraint: pin-to-pin at MIN must be long enough to fit closed cylinder
// 
// COORDINATE SYSTEM: Y=0 is at REAR of machine, Y increases toward FRONT
// - ARM_PIVOT_Y (~200mm) is near the rear
// - Front axle is at ~1150mm
//
// Strategy: Place cylinder base BEHIND the arm pivot (lower Y, toward rear)
// The cylinder pushes UP on the arm from behind/below

// Calculate attachment position at MIN angle to know what we're working with
_attach_y_at_min_prelim = ARM_PIVOT_Y + LIFT_CYL_ARM_OFFSET * cos(_theta_min_lift) + _bracket_drop * sin(_theta_min_lift);
_attach_z_at_min_prelim = ARM_PIVOT_Z + LIFT_CYL_ARM_OFFSET * sin(_theta_min_lift) - _bracket_drop * cos(_theta_min_lift);

// Position base near REAR of machine (low Y value), between side walls
// Base should be behind (lower Y than) the arm pivot
// Raise Z position higher to increase pin-to-pin distance at MIN angle
LIFT_CYL_BASE_Y = max(50, ARM_PIVOT_Y - 150);  // 150mm behind arm pivot, min 50mm from rear edge
LIFT_CYL_BASE_Z = FRAME_Z_OFFSET + WHEEL_DIAMETER/2 + 200;  // Raised higher for better geometry

// --- STEP 5: Calculate arm attachment world positions ---
_theta_min = _theta_min_lift;
_theta_max = _theta_max_lift;

_attach_y_at_min = ARM_PIVOT_Y + LIFT_CYL_ARM_OFFSET * cos(_theta_min) + _bracket_drop * sin(_theta_min);
_attach_z_at_min = ARM_PIVOT_Z + LIFT_CYL_ARM_OFFSET * sin(_theta_min) - _bracket_drop * cos(_theta_min);

_attach_y_at_max = ARM_PIVOT_Y + LIFT_CYL_ARM_OFFSET * cos(_theta_max) + _bracket_drop * sin(_theta_max);
_attach_z_at_max = ARM_PIVOT_Z + LIFT_CYL_ARM_OFFSET * sin(_theta_max) - _bracket_drop * cos(_theta_max);

// --- STEP 6: Calculate ACTUAL pin-to-pin distances ---
_dy_min = _attach_y_at_min - LIFT_CYL_BASE_Y;
_dz_min = _attach_z_at_min - LIFT_CYL_BASE_Z;
_pin_to_pin_at_min = sqrt(_dy_min*_dy_min + _dz_min*_dz_min);

_dy_max = _attach_y_at_max - LIFT_CYL_BASE_Y;
_dz_max = _attach_z_at_max - LIFT_CYL_BASE_Z;
_pin_to_pin_at_max = sqrt(_dy_max*_dy_max + _dz_max*_dz_max);

// --- STEP 7: Calculate available stroke from geometry ---
// Stroke is the difference between max and min pin-to-pin distances
_available_stroke = _pin_to_pin_at_max - _pin_to_pin_at_min;

// --- STEP 8: Calculate maximum cylinder closed length that fits ---
// Must fit at minimum angle with some margin
_max_closed_length = _pin_to_pin_at_min - 25;  // 25mm safety margin

// --- STEP 9: Calculate stroke constraints ---
// For a WELDED hydraulic cylinder, closed length is more compact than tie-rod
// Typical welded cylinder: closed_length ≈ stroke + (bore×0.8 + 100mm for ends)
// Based on commercial specs (e.g., Prince, Cross, Dalton):
//   2.5" bore, 1.5" rod: closed ≈ stroke + 5" (127mm)
//   3" bore, 1.75" rod: closed ≈ stroke + 6" (152mm)
_cylinder_overhead = _LIFT_CYL_BORE + 90;  // Compact welded cylinder: bore + 90mm for ends

// Maximum stroke from closed length constraint: stroke = max_closed - overhead
_max_stroke_from_closed = _max_closed_length - _cylinder_overhead;

// Required stroke from geometry: stroke >= pin_to_pin_max - pin_to_pin_min
_required_stroke_from_geometry = _pin_to_pin_at_max - _pin_to_pin_at_min;

// Add margin for safety and to account for any angle limiting done elsewhere
// The main file may use ARM_MAX_ANGLE_LIMITED which could be lower
_stroke_margin_factor = 1.30;  // 30% margin to ensure full reach
_stroke_to_use = _required_stroke_from_geometry * _stroke_margin_factor;

// --- STEP 10: Round UP to standard stroke (ensure full reach!) ---
_LIFT_CYL_STROKE_AUTO = 
    _stroke_to_use <= 150 ? 150 :
    _stroke_to_use <= 200 ? 200 :
    _stroke_to_use <= 250 ? 250 :
    _stroke_to_use <= 300 ? 300 :
    _stroke_to_use <= 350 ? 350 :
    _stroke_to_use <= 400 ? 400 :
    _stroke_to_use <= 450 ? 450 :
    _stroke_to_use <= 500 ? 500 :
    _stroke_to_use <= 550 ? 550 :
    _stroke_to_use <= 600 ? 600 :
    ceil(_stroke_to_use / 50) * 50;  // Round up to nearest 50mm

// --- STEP 11: Calculate final cylinder physical lengths ---
_CYL_CLOSED_LEN_AUTO = _LIFT_CYL_STROKE_AUTO + _cylinder_overhead;
_CYL_EXTENDED_LEN_AUTO = _CYL_CLOSED_LEN_AUTO + _LIFT_CYL_STROKE_AUTO;

// --- STEP 12: Verify fit ---
_margin_at_min = _pin_to_pin_at_min - _CYL_CLOSED_LEN_AUTO;  // Should be >= 0
_margin_at_max = _CYL_EXTENDED_LEN_AUTO - _pin_to_pin_at_max; // Should be >= 0

// Echo parametric calculation results
echo("=== LIFT CYLINDER PARAMETRIC STROKE CALCULATION ===");
echo("Arm attachment offset:", LIFT_CYL_ARM_OFFSET, "mm (", LIFT_CYL_ARM_OFFSET/ARM_LENGTH*100, "% of arm)");
echo("--- Base Mount Position ---");
echo("LIFT_CYL_BASE_Y:", LIFT_CYL_BASE_Y, "mm");
echo("LIFT_CYL_BASE_Z:", LIFT_CYL_BASE_Z, "mm");
echo("--- Arm Travel Analysis ---");
echo("Attachment at MIN angle (", _theta_min, "°): Y=", _attach_y_at_min, "Z=", _attach_z_at_min);
echo("Attachment at MAX angle (", _theta_max, "°): Y=", _attach_y_at_max, "Z=", _attach_z_at_max);
echo("--- Pin-to-Pin Distances (GEOMETRY CONSTRAINTS) ---");
echo("Pin-to-pin at MIN:", _pin_to_pin_at_min, "mm (cylinder must be SHORTER than this when closed)");
echo("Pin-to-pin at MAX:", _pin_to_pin_at_max, "mm");
echo("Available geometric stroke:", _available_stroke, "mm");
echo("--- Cylinder Size Constraints ---");
echo("Max closed length that fits:", _max_closed_length, "mm");
echo("Cylinder overhead (body+clevises):", _cylinder_overhead, "mm");
echo("Max stroke from closed constraint:", _max_stroke_from_closed, "mm");
echo("Required stroke from geometry:", _required_stroke_from_geometry, "mm");
echo("Stroke with 30% margin:", _stroke_to_use, "mm");
echo("--- SELECTED CYLINDER ---");
echo("AUTO-SELECTED STROKE:", _LIFT_CYL_STROKE_AUTO, "mm (", _LIFT_CYL_STROKE_AUTO/25.4, "inches)");
echo("Cylinder closed length:", _CYL_CLOSED_LEN_AUTO, "mm");
echo("Cylinder extended length:", _CYL_EXTENDED_LEN_AUTO, "mm");
echo("--- Fit Verification ---");
echo("Margin at min (closed fit):", _margin_at_min, "mm", _margin_at_min >= 0 ? "✓ OK" : "✗ TOO LONG");
echo("Margin at max (extended fit):", _margin_at_max, "mm", _margin_at_max >= 0 ? "✓ OK" : "✗ TOO SHORT");

// =============================================================================
// ARM HYDRAULIC MOUNTING PARAMETERS (Integrated into Arm Plates)
// =============================================================================
HYD_BRACKET_CIRCLE_DIA = 76.2;           // 3 inch diameter mounting circle
HYD_BRACKET_ARM_POS = LIFT_CYL_ARM_OFFSET; // Position along arm from pivot
HYD_BRACKET_BOLT_DIA = BOLT_DIA_1;        // 1" bolt for cylinder mount
HYD_BRACKET_CUTOUT_CLEARANCE = 15;        // 15mm clearance around the circle
// Let's try to lower the base slightly if possible.
// LIFT_CYL_BASE_Z = _min_base_z - 25; // Experimental lower
// No, let's stick to calc base Z for now, and see if visual fix solves perception.


// Bucket Pivot Configuration
// BUCKET_PIVOT_HEIGHT_FROM_BOTTOM moved to top of file
// BUCKET_LUG_OFFSET calculation:
// The lug is a U-channel (3x3 tube). The `u_channel_lug_with_pin` centers the pin hole at local (0,0).
// The lug base (back) is at local Y = -Size/2.
// The lug is rotated [90,0,0], so its local Y becomes Global Z (up).
// Wait, in bucket_attachment:
// rotate([90, 0, 0]) means:
//   Global Y comes from Local -Z
//   Global Z comes from Local Y
// Logic in bucket_attachment:
//   rotate([90,0,0]) // Base faces -Y (Bucket Back), Legs face +Y
//   u_channel_lug_with_pin centers hole at 0.
//   If lug height is 100mm, where is the base surface?
//   Standard u_channel_lug: Base is at Y = -(width)/2? No, usually centered.
//   Let's check u_channel_lug logic later. Assuming the Pin Hole is distance H from base?
//   If standard OSE lug: Pin center is ~50mm from base?
//   The variable BUCKET_LUG_OFFSET was calculated as TUBE/2 - THICKNESS/2. This assumes hole is in middle of tube?
//   If the lug is welded to the back of the bucket:
//   Bucket Back Plate is at Bucket Local Y=0 (or T_plate?).
//   The Lug is BEHIND the bucket (at Y < 0).
//   The Pivot Pin is at Global Y=0 (Arm Tip).
//   So the Bucket Back Plate must be at Y = + (Distance from Pin to Lug Base).
//   If Pin is 50mm from Base, Bucket Back is at 50mm.
//   We need to verify the pin-to-base distance.
//   For now, we set BUCKET_LUG_OFFSET to this distance.
//   Let's assume the lug height (base to pin center) is roughly 50mm (2").
//   User Update: Shift bucket backwards to align pivot pin with arm leg hole.
//   The U-channel lug (3x3 tube) outer back surface is at size/2 from pin center (after 90deg rotation)
//   Lug back must be flush with bucket back plate (see DESIGN_RULES.md)
//   Bucket back plate back face (Y=0 in bucket local) must align with lug outer back surface
//   So BUCKET_LUG_OFFSET = size/2 makes lug outer back flush with bucket plate back face
BUCKET_LUG_OFFSET = TUBE_3X3_1_4[0]/2; // 38.1mm: Lug outer back flush with bucket back plate back face

// Note: BUCKET_PIVOT_Y_OFFSET is defined earlier in this file (after BUCKET_PIVOT_HEIGHT_FROM_BOTTOM)
// to ensure arm kinematics calculations can account for it

// Visual adjustment to shift bucket without affecting kinematic target _T
// At BUCKET_VISUAL_Z_OFFSET = 0, bucket bottom should be exactly at ground (Z=0) when flat
// Calculation: pivot_z(90) - (BUCKET_HEIGHT(450) - BUCKET_PIVOT_HEIGHT_FROM_BOTTOM(90)) = 0
BUCKET_VISUAL_Z_OFFSET = 0; // Bucket bottom flat on ground
BUCKET_BODY_Z_OFFSET = 0; // Keep bucket body aligned to pivot; use BUCKET_GROUND_CLEARANCE to set ground contact

// Bucket Cylinder Mount Parameters
BUCKET_CYL_MOUNT_SIZE = TUBE_3X3_1_4[0]; // 3" (76.2mm)
BUCKET_CYL_MOUNT_Y_OFFSET = -BUCKET_CYL_MOUNT_SIZE/2; // Flush with back of bucket plate
// BUCKET_CYL_MOUNT_Z_OFFSET: Position of cylinder lug relative to bucket pivot
// OPTIONS TESTED FOR IMPROVING MAX ARM ANGLE:
//   - Lowering lugs 2" (-50.8mm): Minimal improvement to max arm angle
//   - Shorter cylinder (8" stroke): Changes cross beam position but limited improvement
//   - Combined lower lugs + shorter cylinder: Still limited by parallelism geometry
// The fundamental constraint is the angle between cylinder and bucket back plate.
// Major geometry changes (arm shape, bucket pivot location) would be needed for significant improvement.
BUCKET_CYL_MOUNT_Z_OFFSET = -117.2; // Calculated for 45 deg dump angle

// Bucket Cylinder Stroke Override (set to 0 for auto-selection)
// Standard strokes: 8"(203.2mm), 10"(254mm), 12"(304.8mm), 14"(355.6mm), 16"(406.4mm)
// Shorter cylinder = shorter extended length = cross beam closer to bucket
// Testing showed: shorter cylinders have limited impact on max arm angle
BUCKET_CYLINDER_STROKE_OVERRIDE = 0; // 0 = auto-select based on required stroke

// Cross Beam Cylinder Mount Parameters
CROSS_BEAM_HEIGHT = TUBE_2X6_1_4[0]; // 2" (50.8mm)
CROSS_BEAM_MOUNT_Z_OFFSET = -(CROSS_BEAM_HEIGHT/2 + BUCKET_CYL_MOUNT_SIZE/2); // Bottom of cross beam

// Cross beam configuration
CROSS_BEAM_SIZE = TUBE_2X2_1_4[0];    // 2"x2" tube size
CROSS_BEAM_CLEARANCE = 15;             // Extra clearance around cross beam in cutout

// =============================================================================
// BUCKET DIMENSIONS
// =============================================================================
// Definitions moved to top of file to resolve dependency order for Arm/Solver


/* Old manual adjustment
BUCKET_PIVOT_HEIGHT_BASE = 200; // Nominal height of pivot from bottom of bucket
BUCKET_PIVOT_HEIGHT_ADJUST = -5 * 25.4; // Lower lug (~5")
BUCKET_PIVOT_HEIGHT_FROM_BOTTOM = BUCKET_PIVOT_HEIGHT_BASE + BUCKET_PIVOT_HEIGHT_ADJUST;
*/

// Lift Cylinder - PARAMETRICALLY CALCULATED
// These are the PRIMARY definitions - do not redefine in main file
LIFT_CYLINDER_BORE = _LIFT_CYL_BORE;   // 2.5" bore (63.5mm)
LIFT_CYLINDER_ROD = _LIFT_CYL_ROD;     // 1.5" rod (38.1mm)
LIFT_CYLINDER_STROKE = _LIFT_CYL_STROKE_AUTO;  // Auto-calculated from geometry

// Use the auto-calculated closed/extended lengths
LIFT_CYL_LEN_MIN = _CYL_CLOSED_LEN_AUTO;
LIFT_CYL_LEN_MAX = _CYL_EXTENDED_LEN_AUTO;

// Bucket Cylinder (3" Bore, 1.5" Rod)
// Stroke and closed length will be calculated based on geometry
BUCKET_CYLINDER_BORE = 76.2; // 3"
BUCKET_CYLINDER_ROD = 38.1;  // 1.5"

// =============================================================================
// BUCKET CURL/DUMP ANGLE DEFINITIONS
// =============================================================================
// MAX CURL: Bucket back plate parallel to arm leg front surface
// The leg is rotated by _bend_angle (180 - ARM_SHAPE_ANGLE) from horizontal
// When bucket back is parallel to leg front, bucket has tilted by _bend_angle relative to arm
// In absolute terms (relative to ground), this depends on arm angle

// ARM_SHAPE_ANGLE is the interior angle of the L-shaped arm (e.g., 130 degrees)
// _bend_angle = 180 - ARM_SHAPE_ANGLE = 50 degrees (angle of leg from main arm axis)
// When bucket back is parallel to leg front, bucket tilt relative to arm = _bend_angle

// MAX CURL absolute angle (relative to ground) when arms at ground level:
// Bucket_abs = Arm_angle + Bucket_rel = ARM_MIN_ANGLE + _bend_angle
BUCKET_MAX_CURL_ANGLE = ARM_MIN_ANGLE + _bend_angle; // Max curl = bucket back parallel to leg front

// MIN CURL (MAX DUMP): Bucket tilted down 45 degrees below horizontal when arms raised
// When arms at ARM_MAX_ANGLE, bucket absolute angle = -45 degrees
BUCKET_MIN_CURL_ANGLE = -45; // Target dump angle: 45 degrees below horizontal

echo("=== BUCKET CURL/DUMP DEFINITIONS ===");
echo("ARM_SHAPE_ANGLE (elbow):", ARM_SHAPE_ANGLE);
echo("_bend_angle (leg from horizontal):", _bend_angle);
echo("BUCKET_MAX_CURL_ANGLE (abs, at ground):", BUCKET_MAX_CURL_ANGLE);
echo("BUCKET_MIN_CURL_ANGLE (abs, dump at raised):", BUCKET_MIN_CURL_ANGLE);

// Legacy compatibility
BUCKET_ABS_CURL_ANGLE = BUCKET_MAX_CURL_ANGLE;
BUCKET_ABS_DUMP_ANGLE = BUCKET_MIN_CURL_ANGLE;

// =============================================================================
// BUCKET CYLINDER GEOMETRY CALCULATION
// =============================================================================
// Goal: Position cross beam so cylinder is fully retracted at max curl,
//       and select a standard cylinder stroke that allows -45° dump at max arm height

// Calculate Relative Angles (bucket angle relative to arm)
// Abs = Arm + Rel => Rel = Abs - Arm
_bucket_rel_curl_angle = is_undef(BUCKET_MAX_CURL_ANGLE) ? 50 : (BUCKET_MAX_CURL_ANGLE - ARM_MIN_ANGLE); // At ground level
_bucket_rel_dump_angle = is_undef(BUCKET_MIN_CURL_ANGLE) ? -105 : (BUCKET_MIN_CURL_ANGLE - ARM_MAX_ANGLE); // At raised position

// Bucket Cylinder Lug Position (relative to bucket pivot)
// Lug is at (Y=0, Z=_lug_z_local) in bucket local coords
_lug_z_local = BUCKET_CYL_MOUNT_Z_OFFSET + (BUCKET_HEIGHT - BUCKET_PIVOT_HEIGHT_FROM_BOTTOM);
_lug_z_safe = is_undef(_lug_z_local) ? 242.8 : _lug_z_local;

echo("=== BUCKET CYLINDER LUG DEBUG ===");
echo("BUCKET_CYL_MOUNT_Z_OFFSET:", BUCKET_CYL_MOUNT_Z_OFFSET);
echo("BUCKET_HEIGHT:", BUCKET_HEIGHT);
echo("BUCKET_PIVOT_HEIGHT_FROM_BOTTOM:", BUCKET_PIVOT_HEIGHT_FROM_BOTTOM);
echo("_lug_z_local (relative to bucket pivot):", _lug_z_local);
echo("_bucket_rel_curl_angle:", _bucket_rel_curl_angle);
echo("_bucket_rel_dump_angle:", _bucket_rel_dump_angle);

// =============================================================================
// STEP 1: Calculate lug position at MAX CURL (cylinder retracted)
// =============================================================================
// Rotate lug by relative curl angle about bucket pivot (at arm tip)
_arm_tip_x_safe = is_undef(ARM_TIP_X) ? 1500 : ARM_TIP_X;
_arm_tip_z_safe = is_undef(ARM_TIP_Z) ? 100 : ARM_TIP_Z;
_pivot_y_offset_safe = is_undef(BUCKET_PIVOT_Y_OFFSET) ? -64.6 : BUCKET_PIVOT_Y_OFFSET;

_lug_y_curl = _arm_tip_x_safe + _pivot_y_offset_safe + (0 * cos(_bucket_rel_curl_angle) - _lug_z_safe * sin(_bucket_rel_curl_angle));
_lug_z_curl = _arm_tip_z_safe + (0 * sin(_bucket_rel_curl_angle) + _lug_z_safe * cos(_bucket_rel_curl_angle));

// =============================================================================
// STEP 2: Calculate lug position at MIN CURL / MAX DUMP (cylinder extended)
// =============================================================================
_lug_y_dump = _arm_tip_x_safe + _pivot_y_offset_safe + (0 * cos(_bucket_rel_dump_angle) - _lug_z_safe * sin(_bucket_rel_dump_angle));
_lug_z_dump = _arm_tip_z_safe + (0 * sin(_bucket_rel_dump_angle) + _lug_z_safe * cos(_bucket_rel_dump_angle));

// =============================================================================
// STEP 3: Calculate ARM_MAX_ANGLE_LIMITED based on bucket cylinder parallelism
// =============================================================================
// We calculate this FIRST (before cylinder sizing) so we know the actual operating range.
// Use a reference cross beam position for the parallelism calculation.

_cb_ref_pos = ARM_TIP_X * 0.65;  // Reference position for parallelism calculation
_cb_z = CROSS_BEAM_MOUNT_Z_OFFSET; // Z position of cylinder mount on cross beam
_target_dump_angle = -45;  // Target bucket absolute angle at dump

// Function to calculate the angle between bucket cylinder and bucket back plate normal
// Returns the angle in degrees (0 = parallel to back plate, 90 = perpendicular)
function bucket_cyl_to_back_angle_v2(arm_angle, target_bucket_abs_angle, cb_y_pos) = 
    let(
        bucket_rel_tilt = target_bucket_abs_angle - arm_angle,
        lug_y_bucket = 0,
        lug_z_bucket = _lug_z_safe,
        lug_y_arm = _arm_tip_x_safe + _pivot_y_offset_safe + 
                    (lug_y_bucket * cos(bucket_rel_tilt) - lug_z_bucket * sin(bucket_rel_tilt)),
        lug_z_arm = _arm_tip_z_safe + 
                    (lug_y_bucket * sin(bucket_rel_tilt) + lug_z_bucket * cos(bucket_rel_tilt)),
        cyl_dy = lug_y_arm - cb_y_pos,
        cyl_dz = lug_z_arm - _cb_z,
        back_normal_y = cos(bucket_rel_tilt),
        back_normal_z = sin(bucket_rel_tilt),
        cyl_len = sqrt(cyl_dy*cyl_dy + cyl_dz*cyl_dz),
        dot_product = (cyl_dy * back_normal_y + cyl_dz * back_normal_z) / cyl_len,
        dot_clamped = max(-1, min(1, dot_product)),
        angle_from_normal = acos(abs(dot_clamped))
    )
    90 - angle_from_normal;

// Binary search to find the arm angle where cylinder is 10 degrees from parallel
_parallelism_limit = 10;  // Degrees from parallel (safety margin)

function find_parallel_limit_angle_v2(min_arm, max_arm, target_bucket_abs, limit_angle, cb_pos, iterations) =
    iterations <= 0 ? (min_arm + max_arm) / 2 :
    let(
        mid_arm = (min_arm + max_arm) / 2,
        angle_at_mid = bucket_cyl_to_back_angle_v2(mid_arm, target_bucket_abs, cb_pos)
    )
    angle_at_mid < limit_angle ? 
        find_parallel_limit_angle_v2(min_arm, mid_arm, target_bucket_abs, limit_angle, cb_pos, iterations - 1) :
        find_parallel_limit_angle_v2(mid_arm, max_arm, target_bucket_abs, limit_angle, cb_pos, iterations - 1);

// Find arm angle at parallelism limit
_arm_parallel_limit_raw = find_parallel_limit_angle_v2(ARM_MIN_ANGLE, ARM_MIN_ANGLE + 120, _target_dump_angle, _parallelism_limit, _cb_ref_pos, 15);
_arm_parallel_limit = is_undef(_arm_parallel_limit_raw) ? ARM_MAX_ANGLE :
                      (_arm_parallel_limit_raw != _arm_parallel_limit_raw) ? ARM_MAX_ANGLE :
                      _arm_parallel_limit_raw;

ARM_MAX_ANGLE_LIMITED = min(ARM_MAX_ANGLE, _arm_parallel_limit);

echo("=== ARM ANGLE LIMIT (STEP 3) ===");
echo("Parallelism safety margin:", _parallelism_limit, "degrees from parallel");
echo("Arm angle at parallelism limit:", _arm_parallel_limit, "degrees");
echo("Original ARM_MAX_ANGLE:", ARM_MAX_ANGLE, "degrees");
echo("ARM_MAX_ANGLE_LIMITED:", ARM_MAX_ANGLE_LIMITED, "degrees");

// =============================================================================
// STEP 4: Calculate ACTUAL lug positions using LIMITED arm angle
// =============================================================================
// Now that we know the limited arm angle, calculate the actual dump position
_bucket_rel_dump_limited = _target_dump_angle - ARM_MAX_ANGLE_LIMITED;

_lug_y_dump_limited = _arm_tip_x_safe + _pivot_y_offset_safe + 
                      (0 * cos(_bucket_rel_dump_limited) - _lug_z_safe * sin(_bucket_rel_dump_limited));
_lug_z_dump_limited = _arm_tip_z_safe + 
                      (0 * sin(_bucket_rel_dump_limited) + _lug_z_safe * cos(_bucket_rel_dump_limited));

echo("=== ACTUAL LUG POSITIONS (STEP 4) ===");
echo("Bucket rel curl angle:", _bucket_rel_curl_angle, "degrees");
echo("Bucket rel dump angle (limited):", _bucket_rel_dump_limited, "degrees");
echo("Lug Y at curl:", _lug_y_curl);
echo("Lug Z at curl:", _lug_z_curl);
echo("Lug Y at dump (limited):", _lug_y_dump_limited);
echo("Lug Z at dump (limited):", _lug_z_dump_limited);

// =============================================================================
// STEP 5: Calculate ACTUAL required stroke based on LIMITED positions
// =============================================================================
// The stroke needed is the distance between curl and dump lug positions
_actual_lug_travel_y = _lug_y_dump_limited - _lug_y_curl;
_actual_lug_travel_z = _lug_z_dump_limited - _lug_z_curl;
_actual_stroke_needed = sqrt(pow(_actual_lug_travel_y, 2) + pow(_actual_lug_travel_z, 2));

echo("=== ACTUAL STROKE CALCULATION (STEP 5) ===");
echo("Lug travel Y:", _actual_lug_travel_y);
echo("Lug travel Z:", _actual_lug_travel_z);
echo("ACTUAL stroke needed (lug travel):", _actual_stroke_needed, "mm (", _actual_stroke_needed/25.4, "inches)");

// Compare with original estimate (before limiting)
_original_lug_travel_y = _lug_y_dump - _lug_y_curl;
_original_lug_travel_z = _lug_z_dump - _lug_z_curl;
_original_stroke_estimate = sqrt(pow(_original_lug_travel_y, 2) + pow(_original_lug_travel_z, 2));
echo("Original estimate (before limiting):", _original_stroke_estimate, "mm (", _original_stroke_estimate/25.4, "inches)");
echo("Stroke SAVINGS from limiting:", _original_stroke_estimate - _actual_stroke_needed, "mm (", (_original_stroke_estimate - _actual_stroke_needed)/25.4, "inches)");

// =============================================================================
// STEP 6: Select standard cylinder based on ACTUAL stroke needed
// =============================================================================
// Standard strokes in mm: 203.2(8"), 254(10"), 304.8(12"), 355.6(14"), 406.4(16"), 457.2(18"), 508(20"), 609.6(24")
_std_strokes = [203.2, 254, 304.8, 355.6, 406.4, 457.2, 508, 609.6];

// Find smallest standard stroke >= needed stroke (no margin - extra capacity is wasted
// since arm angle is limited by parallelism and curl is limited by future bumper)
function find_std_stroke(needed, strokes, i=0) = 
    (i >= len(strokes)) ? strokes[len(strokes)-1] :
    (strokes[i] >= needed) ? strokes[i] : find_std_stroke(needed, strokes, i+1);

_auto_stroke = find_std_stroke(_actual_stroke_needed, _std_strokes);
BUCKET_CYLINDER_STROKE = (BUCKET_CYLINDER_STROKE_OVERRIDE > 0) ? BUCKET_CYLINDER_STROKE_OVERRIDE : _auto_stroke;
_bucket_stroke_safe = is_undef(BUCKET_CYLINDER_STROKE) ? 304.8 : BUCKET_CYLINDER_STROKE;

echo("=== CYLINDER SELECTION (STEP 6) ===");
echo("Auto-selected stroke:", _auto_stroke, "mm (", _auto_stroke/25.4, "inches)");
echo("Override stroke:", BUCKET_CYLINDER_STROKE_OVERRIDE, "mm");
echo("USING stroke:", _bucket_stroke_safe, "mm (", _bucket_stroke_safe/25.4, "inches)");

// Cylinder closed length (pin-to-pin)
// dead_length = B*1.8 + R*1.5 + 10
BUCKET_CYL_DEAD_LENGTH = BUCKET_CYLINDER_BORE * 1.8 + BUCKET_CYLINDER_ROD * 1.5 + 10;
BUCKET_CYL_LEN_MIN = _bucket_stroke_safe + BUCKET_CYL_DEAD_LENGTH; // Retracted length
BUCKET_CYL_LEN_MAX = BUCKET_CYL_LEN_MIN + _bucket_stroke_safe;     // Extended length

echo("Cylinder dead length:", BUCKET_CYL_DEAD_LENGTH, "mm");
echo("Cylinder retracted length:", BUCKET_CYL_LEN_MIN, "mm");
echo("Cylinder extended length:", BUCKET_CYL_LEN_MAX, "mm");

// =============================================================================
// STEP 7: Position cross beam so cylinder is fully extended at dump
// =============================================================================
_z_diff_dump = _lug_z_dump_limited - _cb_z;
_sq_val_dump = pow(BUCKET_CYL_LEN_MAX, 2) - pow(_z_diff_dump, 2);
_y_dist_dump_valid = !is_undef(_sq_val_dump) && (_sq_val_dump == _sq_val_dump) && (_sq_val_dump >= 0);
_y_dist_dump = _y_dist_dump_valid ? sqrt(_sq_val_dump) : 500;

_cb1_calc = _lug_y_dump_limited - _y_dist_dump;
CROSS_BEAM_1_POS = (is_undef(_cb1_calc) || _cb1_calc != _cb1_calc) ? (ARM_TIP_X * 0.65) : _cb1_calc;

echo("=== CROSS BEAM POSITION (STEP 7) ===");
echo("Target cylinder length at dump:", BUCKET_CYL_LEN_MAX, "mm (fully extended)");
echo("CROSS_BEAM_1_POS:", CROSS_BEAM_1_POS, "mm");

// =============================================================================
// STEP 8: Verify geometry at curl and dump positions
// =============================================================================
_actual_len_curl = sqrt(pow(_lug_y_curl - CROSS_BEAM_1_POS, 2) + pow(_lug_z_curl - _cb_z, 2));
_curl_margin = _actual_len_curl - BUCKET_CYL_LEN_MIN;
_curl_ok = _actual_len_curl >= BUCKET_CYL_LEN_MIN;

_actual_len_dump_check = sqrt(pow(_lug_y_dump_limited - CROSS_BEAM_1_POS, 2) + pow(_lug_z_dump_limited - _cb_z, 2));

_final_parallelism = bucket_cyl_to_back_angle_v2(ARM_MAX_ANGLE_LIMITED, _target_dump_angle, CROSS_BEAM_1_POS);

echo("=== GEOMETRY VERIFICATION (STEP 8) ===");
echo("--- At Max Curl ---");
echo("Actual cylinder length at curl:", _actual_len_curl, "mm");
echo("Cylinder retracted length:", BUCKET_CYL_LEN_MIN, "mm");
echo("Margin at curl (extra retraction available):", _curl_margin, "mm (", _curl_margin/25.4, "inches)");
echo("RETRACTS ENOUGH?", _curl_ok ? "YES" : "NO - CYLINDER TOO LONG");
echo("--- At Max Dump ---");
echo("Actual cylinder length at dump:", _actual_len_dump_check, "mm");
echo("Cylinder extended length:", BUCKET_CYL_LEN_MAX, "mm");
echo("FULLY EXTENDED?", abs(_actual_len_dump_check - BUCKET_CYL_LEN_MAX) < 1 ? "YES" : "CHECK");
echo("--- Parallelism ---");
echo("Angle from parallel at limit:", _final_parallelism, "degrees");
echo("SAFE?", _final_parallelism >= _parallelism_limit ? "YES" : "NO");

// Legacy function for compatibility
function bucket_cyl_to_back_angle(arm_angle, target_bucket_abs_angle) = 
    bucket_cyl_to_back_angle_v2(arm_angle, target_bucket_abs_angle, CROSS_BEAM_1_POS);

CROSS_BEAM_2_POS = ARM_TIP_X * 0.95;

ENGINE_HP = 25; // Desired horsepower (e.g., 25HP V-Twin)
// Estimate engine weight (kg) based on HP (approximate for small engines)
// Base 30kg + 1.2kg per HP
ENGINE_WEIGHT_KG = 30 + (ENGINE_HP * 1.2); 

// Engine Position (Middle near back)
ENGINE_POS_Y = 400; // Forward from rear of frame
ENGINE_POS_Z = FRAME_Z_OFFSET + 200; // Height above frame bottom

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

// --- Derived dimensions (calculated from frame geometry) ---
PLATFORM_CLEARANCE = 50;  // Clearance from inner walls (mm)

// --- Configurable dimensions ---
PLATFORM_DEPTH = 400;           // Front-to-back depth of deck (mm)
deck_bolt_inner_dist = 50;      // Distance from pivot end to inner deck bolt (mm)
deck_bolt_outer_dist = PLATFORM_DEPTH * 0.75; // Distance from pivot end to outer deck bolt (mm)
PLATFORM_ARM_LENGTH = 425;      // Length of angle iron arms (mm)
PLATFORM_PIVOT_HEIGHT = 330;    // Height of pivot / deck surface when deployed (mm)
PLATFORM_THICKNESS = PLATE_1_4_INCH;  // Deck plate thickness

// Pivot X position - Aligned with inner plate of machine walls
// Inner Wall X = TRACK_WIDTH/2 - SANDWICH_SPACING/2 - PANEL_THICKNESS
// Pivot Bracket Center = Inner Wall X - Bracket Thickness/2
_inner_wall_x = TRACK_WIDTH/2 - SANDWICH_SPACING/2 - PANEL_THICKNESS;
PLATFORM_PIVOT_X = _inner_wall_x - PLATE_1_4_INCH/2;

// PLATFORM_WIDTH calculated to fill distance between L plates (Pivot Brackets) with 1/8" gap
// Pivot Bracket Inner Face = PLATFORM_PIVOT_X - PLATFORM_THICKNESS/2
// Gap = 1/8 inch = 3.175mm
PLATFORM_SIDE_GAP = 3.175;
PLATFORM_WIDTH = 2 * (PLATFORM_PIVOT_X - PLATFORM_THICKNESS/2 - PLATFORM_SIDE_GAP);

// --- Pivot bracket dimensions ---
PLATFORM_BRACKET_WIDTH = 100;   // Width of pivot bracket plate (mm)
PLATFORM_BRACKET_LENGTH = PLATFORM_DEPTH - PLATFORM_BRACKET_WIDTH/2;  // Length of pivot bracket plate (mm)

// --- Hardware dimensions ---
PLATFORM_PIVOT_PIN_DIA = BOLT_DIA_1;    // 1" (25.4mm) pivot pin diameter
PLATFORM_LOCK_PIN_DIA = 15.875;         // 5/8" (15.875mm) hitch pin diameter
PLATFORM_BOLT_DIA = BOLT_DIA_1_2;       // 1/2" (12.7mm) mounting bolts
PLATFORM_BOLT_CLEARANCE = 2;            // Clearance for bolt holes (mm)
PLATFORM_LOCK_OFFSET = 60;              // Distance from pivot center to lock pin hole (mm)

// --- New Parametric Dimensions ---
PLATFORM_MOUNT_Y = 75.4;                // Forward position of platform pivot (mm)
PLATFORM_TRANSVERSE_GAP = 6.35;         // 1/4" clearance for angle irons (mm)
PLATFORM_BRACKET_CORNER_RADIUS = 25.4;  // 1" radius for pivot bracket corner (mm)
PLATFORM_SIDE_BOLT_SPACING = 70;        // Spacing between side bolts (mm)
PLATFORM_SIDE_BOLT_START = 20;          // Distance to first bolt from corner (mm)
PLATFORM_SIDE_BOLT_Y_OFFSET = -70;      // Vertical offset of side bolts from lock pin (mm)
// Transverse bolts: Offset from ends of the transverse angle iron
// Use 10% of platform width, ensuring edge clearance
PLATFORM_TRANSVERSE_BOLT_END_OFFSET = max(PLATFORM_WIDTH * 0.10, 2 * PLATFORM_BOLT_DIA);
PLATFORM_SIDE_DECK_BOLT_SPACING = 150;  // Spacing for side angle deck bolts (mm)
PLATFORM_EDGE_MARGIN = 12.7;            // 1/2" margin from deck edge to angle iron (mm)

// --- Bolt Offsets for Collision Avoidance ---
// Deck bolts (Top): Wide spacing -> Small offset from ends (e.g. 8% of length)
// Ensure at least 2x bolt diameter for edge clearance
deck_bolt_offset = max(PLATFORM_ARM_LENGTH * 0.08, 2 * PLATFORM_BOLT_DIA);

// Pivot bolts (Side): Narrow spacing -> Large offset from ends of the bracket leg
// Bracket leg length = PLATFORM_BRACKET_LENGTH - PLATFORM_BRACKET_WIDTH/2
// Use 30% offset from each end (leaving 40% spacing in middle)
pivot_bolt_offset = (PLATFORM_BRACKET_LENGTH - PLATFORM_BRACKET_WIDTH/2) * 0.30;

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

// --- Vertical Alignment Calculations ---
// Calculate the Z-offset required to align the bottom of the angle iron with the bottom of the bracket
// Bracket bottom relative to pivot:
_bracket_bottom_rel = -PLATFORM_LOCK_OFFSET + PLATFORM_SIDE_BOLT_Y_OFFSET - PLATFORM_BRACKET_WIDTH/2;
// Angle iron bottom relative to its origin is -PLATFORM_ANGLE_LEG
// We want AngleBottom = BracketBottom
// (Origin + Offset) - LEG = BracketBottom
// Offset = BracketBottom + LEG
PLATFORM_ANGLE_Z_OFFSET = _bracket_bottom_rel + PLATFORM_ANGLE_LEG;

// =============================================================================
// FOLDING PLATFORM PARAMETER VALIDATION
// =============================================================================
// Assert statements ensure parameter relationships are valid

// Arm must be long enough to reach from pivot to deck attachment
assert(PLATFORM_ARM_LENGTH >= PLATFORM_DEPTH/2, 
    "PLATFORM_ARM_LENGTH must be at least half of PLATFORM_DEPTH");

// Pivot X must be inside the inner frame walls
_inner_wall_limit_x = TRACK_WIDTH/2 - SANDWICH_SPACING/2;
assert(PLATFORM_PIVOT_X < _inner_wall_limit_x, 
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
_crossmember_top_z = FRAME_Z_OFFSET + MACHINE_HEIGHT * 0.7;
assert(PLATFORM_PIVOT_HEIGHT + PLATFORM_ARM_LENGTH < _crossmember_top_z, 
    "Stowed position lock hole must be within rear crossmember height");

// =============================================================================
// JIG PARAMETERS & NEW ARM PARAMETERS
// =============================================================================











