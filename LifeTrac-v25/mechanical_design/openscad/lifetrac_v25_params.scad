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

ARM_DROP_LEN = _L_drop_kinematic - ARM_DROP_EXT + PIVOT_HOLE_X_FROM_FRONT; 

ARM_PIVOT_EXT = 40; 
PIVOT_HOLE_EDGE_OFFSET = 1.5 * 25.4; 
// PIVOT_HOLE_Z_FROM_BOTTOM set to Top Edge (Tangent to Top)
// This aligns the pivot with the "Upper/Outer" edge of the arm profile.
PIVOT_HOLE_Z_FROM_BOTTOM = TUBE_2X6_1_4[1] - PIVOT_HOLE_X_FROM_FRONT; 

echo("DEBUG: Final Parameters:");
echo("  ARM_SHAPE_ANGLE:", ARM_SHAPE_ANGLE);
echo("  ARM_MAIN_LEN:", ARM_MAIN_LEN);
echo("  ARM_DROP_LEN:", ARM_DROP_LEN);

// Vector from Main Pivot to Bucket Pivot (For reference)
_bend_angle = 180 - ARM_ANGLE; // Note: ARM_ANGLE is now possibly obsolete if we use SHAPE_ANGLE
_drop_vec_len = ARM_DROP_LEN + ARM_DROP_EXT - PIVOT_HOLE_X_FROM_FRONT; // Hole X position (from front edge)
_tube_h = TUBE_2X6_1_4[1];
_hole_z_offset = PIVOT_HOLE_Z_FROM_BOTTOM; // Hole Z position (from bottom edge)

_dx = _drop_vec_len * cos(_bend_angle) + (_hole_z_offset - _tube_h) * sin(_bend_angle);
_dz = -_drop_vec_len * sin(_bend_angle) + (_hole_z_offset - _tube_h) * cos(_bend_angle);

// Calculate Tip Position (Top/Effective Pivot Point)
ARM_TIP_X = (ARM_MAIN_LEN + ARM_OVERLAP) + _dx; // Removed +50 error
ARM_TIP_Z = _tube_h + _dz;

ARM_V2_LENGTH = sqrt(pow(ARM_TIP_X, 2) + pow(ARM_TIP_Z, 2));
// Geometric angle of the arm tip vector relative to horizontal
ARM_GEOMETRY_ANGLE = atan2(ARM_TIP_Z, ARM_TIP_X);

echo("=== ARM V2 GEOMETRY ===");
echo("ARM_V2_LENGTH =", ARM_V2_LENGTH);
echo("ARM_GEOMETRY_ANGLE =", ARM_GEOMETRY_ANGLE);
echo("ARM_TIP_X =", ARM_TIP_X);
echo("ARM_TIP_Z =", ARM_TIP_Z);

// Calculate angles for kinematics
_x_rel = ARM_TIP_X;
_z_rel = ARM_TIP_Z - _tube_h/2;

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

// 1. Define Arm Attachment Point
LIFT_CYL_ARM_OFFSET = ARM_LENGTH * 0.25;     // Attachment point along arm from pivot (proportional)

// 2. Calculate Arm Attachment Position in World Coords at Lowest Angle (Bucket on Ground)
//    Pivot is at [0, ARM_PIVOT_Y, ARM_PIVOT_Z]
// _theta_min = -ARM_GROUND_ANGLE; // OLD: Used fixed 45 deg
_theta_min = ARM_MIN_ANGLE_COMPUTED; // NEW: Use calculated tangent angle (-25)
_attach_y_world = ARM_PIVOT_Y + LIFT_CYL_ARM_OFFSET * cos(_theta_min);
_attach_z_world = ARM_PIVOT_Z + LIFT_CYL_ARM_OFFSET * sin(_theta_min);

// 3. Determine Base Z Height (Constrained by Axle Clearance)
//    Must clear the rear axle (FRAME_Z_OFFSET + WHEEL_DIAMETER/2)
_min_base_z = FRAME_Z_OFFSET + WHEEL_DIAMETER/2 + 65; // Reduced clearance to fit 16" cylinder
LIFT_CYL_BASE_Z = _min_base_z;

// 4. Calculate Base Y for Optimal Leverage Angle
//    We want the cylinder to push at an angle ~45-50 degrees from horizontal
//    to be roughly perpendicular to the arm (which is at ~-45 degrees).
//    Slope m = tan(target_angle).
//    Y = Attach_Y - (Attach_Z - Base_Z) / m
_target_cyl_angle = 50; // Degrees
_calc_base_y = _attach_y_world - (_attach_z_world - LIFT_CYL_BASE_Z) / tan(_target_cyl_angle);

// 5. Clamp Y to valid frame range (0 to WHEEL_BASE/2)
// Adjusted for 16" cylinder fit
// LIFT_CYL_BASE_Y = max(50, min(WHEEL_BASE/2, _calc_base_y + 10)); // Original
LIFT_CYL_BASE_Y = max(50, min(WHEEL_BASE/2, _calc_base_y + 10)); // Current logic seems fine, relying on calc update
// Note: If cylinders bottom out, we need to move Base FORWARD (bigger Y) or DOWN (smaller Z).
// But we are constrained.
// If _attach_z_world is HIGHER (because angle is -25 vs -45), the cylinder is LONGER.
// So Bottoming Out should not be an issue with higher arm position.
// Unless "lowered" meant "Lower the mount to get MORE extension"?
// If the user says "lowered so that they do not bottom out" -> "Bottom out" usually means COMPRESSED fully.
// If the cylinder is too SHORT, we need to make the distance LARGER.
// To make distance larger: Move Base Away from Arm.
// Arm is Up/Right. Base is Down/Left.
// Moving Base Down (Lower Z) increases distance.
// Moving Base Back (Lower Y) increases distance.
// Let's check clearance.
// _min_base_z = FRAME_Z_OFFSET + WHEEL_DIAMETER/2 + 65;
// We can't go lower than this without hitting axle.
// Can we go smoother on the axle? CLEARANCE = 65 -> 40?
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
//   User Update: Shift bucket backwards closely to align pivot holes.
BUCKET_LUG_OFFSET = 25; // Adjusted (35->25) to align pin with leg hole (Shift backwards)

// Visual adjustment to shift bucket down without affecting kinematic target _T
// Used to calibrate visual ground contact
BUCKET_VISUAL_Z_OFFSET = -10;
BUCKET_BODY_Z_OFFSET = 0; // Keep bucket body aligned to pivot; use BUCKET_GROUND_CLEARANCE to set ground contact

// Bucket Cylinder Mount Parameters
BUCKET_CYL_MOUNT_SIZE = TUBE_3X3_1_4[0]; // 3" (76.2mm)
BUCKET_CYL_MOUNT_Y_OFFSET = -BUCKET_CYL_MOUNT_SIZE/2; // Flush with back of bucket plate
BUCKET_CYL_MOUNT_Z_OFFSET = -117.2; // Calculated for 45 deg dump angle

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

// Lift Cylinder (3" Bore, 1.5" Rod, 16" Stroke)
LIFT_CYLINDER_BORE = 76.2; // 3"
LIFT_CYLINDER_ROD = 38.1;  // 1.5"
LIFT_CYLINDER_STROKE_DEF = 406.4; // 16" - Updated for better reach
LIFT_CYLINDER_STROKE = LIFT_CYLINDER_STROKE_DEF;
LIFT_CYL_LEN_MIN_DEF = 406.4 + 205; // Stroke + Dead Length (approx 8")
LIFT_CYL_LEN_MIN = LIFT_CYL_LEN_MIN_DEF;

// Bucket Cylinder (3" Bore, 1.5" Rod, 18" Stroke)
BUCKET_CYLINDER_BORE = 76.2; // 3"
BUCKET_CYLINDER_ROD = 38.1;  // 1.5"
BUCKET_CYLINDER_STROKE_DEF = 457.2; // 18" - Default/Design value
BUCKET_CYLINDER_STROKE = BUCKET_CYLINDER_STROKE_DEF;
BUCKET_CYL_LEN_MIN_DEF = 457.2 + 305; // Explicit calculation
BUCKET_CYL_LEN_MIN = BUCKET_CYL_LEN_MIN_DEF;

// Parametric Cross Beam Position
// Calculated based on bucket cylinder retracted length to ensure proper geometry
// We want the cylinder to fit when retracted and bucket is curled back (MAX_CURL)
// USER REQUIREMENT: Curl/Dump angles are relative to GROUND.
BUCKET_ABS_CURL_ANGLE = 45;  // Curl Angle relative to ground (deg)
BUCKET_ABS_DUMP_ANGLE = -90; // Dump Angle relative to ground (deg)

// Calculate Relative Curl Angle at Ground Level (Design Condition)
// At ground level, Arm Angle is ARM_MIN_ANGLE
// Abs = Arm + Rel => Rel = Abs - Arm
_bucket_rel_curl_angle = BUCKET_ABS_CURL_ANGLE - ARM_MIN_ANGLE;

// Calculate Lug Position in Arm Frame at Max Curl
// Lug Local Z relative to Pivot
_lug_z_local = BUCKET_CYL_MOUNT_Z_OFFSET + (BUCKET_HEIGHT - BUCKET_PIVOT_HEIGHT_FROM_BOTTOM);

// Rotate Lug by Relative Curl Angle (relative to Arm)
// Lug Local Y is 0.
_lug_y_arm = ARM_TIP_X + (0 * cos(_bucket_rel_curl_angle) - _lug_z_local * sin(_bucket_rel_curl_angle));
_lug_z_arm = ARM_TIP_Z + (0 * sin(_bucket_rel_curl_angle) + _lug_z_local * cos(_bucket_rel_curl_angle));

// Solve for Cross Beam Y Position (CROSS_BEAM_1_POS)
// Distance^2 = (Y_lug - Y_cb)^2 + (Z_lug - Z_cb)^2
// Y_cb = Y_lug - sqrt(Distance^2 - (Z_lug - Z_cb)^2)
_z_diff = _lug_z_arm - CROSS_BEAM_MOUNT_Z_OFFSET;
_sq_val = pow(BUCKET_CYL_LEN_MIN, 2) - pow(_z_diff, 2);

// Safety: Handle cases where geometry is impossible (dist < z_diff) or NaN
_y_dist_valid = !is_undef(_sq_val) && (_sq_val == _sq_val) && (_sq_val >= 0);
_y_dist = _y_dist_valid ? sqrt(_sq_val) : 500; // Default fallback if geometry fails

_cb1_calc = _lug_y_arm - _y_dist;
CROSS_BEAM_1_POS = (is_undef(_cb1_calc) || _cb1_calc != _cb1_calc) ? (ARM_LENGTH * 0.65) : _cb1_calc;

// =============================================================================
// DUMP GEOMETRY VERIFICATION
// =============================================================================
// Verify if the cylinder stroke allows for full vertical dump (-90 deg ground) at MAX HEIGHT

// Design Condition for Dump: Max Height (ARM_MAX_ANGLE)
// Rel = Abs - Arm
_bucket_rel_dump_angle = BUCKET_ABS_DUMP_ANGLE - ARM_MAX_ANGLE;

// Calculate required cylinder positions for check
_lug_y_dump = ARM_TIP_X + (0 * cos(_bucket_rel_dump_angle) - _lug_z_local * sin(_bucket_rel_dump_angle));
_lug_z_dump = ARM_TIP_Z + (0 * sin(_bucket_rel_dump_angle) + _lug_z_local * cos(_bucket_rel_dump_angle));
_req_len_dump = sqrt(pow(_lug_y_dump - CROSS_BEAM_1_POS, 2) + pow(_lug_z_dump - CROSS_BEAM_MOUNT_Z_OFFSET, 2));

BUCKET_CYL_LEN_MAX = BUCKET_CYL_LEN_MIN + BUCKET_CYLINDER_STROKE;

echo("=== BUCKET GEOMETRY VERIFICATION ===");
echo("Condition: Ground Level (ARM_MIN_ANGLE):", ARM_MIN_ANGLE);
echo("Target Abs Curl Angle:", BUCKET_ABS_CURL_ANGLE);
echo("Calculated Rel Curl Angle:", _bucket_rel_curl_angle);
echo("Condition: Max Height (ARM_MAX_ANGLE):", ARM_MAX_ANGLE);
echo("Target Abs Dump Angle:", BUCKET_ABS_DUMP_ANGLE);
echo("Calculated Rel Dump Angle:", _bucket_rel_dump_angle);
echo("---");
echo("Resulting Cross Beam Position:", CROSS_BEAM_1_POS);
echo("Cylinder Retracted Length (Min):", BUCKET_CYL_LEN_MIN);
echo("Cylinder Extended Length (Max):", BUCKET_CYL_LEN_MAX);
echo("Required Length for Full Dump at Max Height:", _req_len_dump);
echo("REACHES FULL DUMP?", BUCKET_CYL_LEN_MAX >= _req_len_dump ? "YES" : "NO - INCREASE STROKE");

CROSS_BEAM_2_POS = ARM_LENGTH * 0.95;   // Second cross beam position (near bucket)

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











