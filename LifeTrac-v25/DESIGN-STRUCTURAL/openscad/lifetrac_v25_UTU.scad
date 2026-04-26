// lifetrac_v25_UTU.scad
// LifeTrac v25 UTU (Universal Track Unit) Variant Assembly
// Open Source Ecology - Compact Remote Controlled Utility Loader
// License: GPL v3
//
// This file creates a tracked variant of the LifeTrac v25 using the
// Universal Track Unit (UTU) instead of the Universal Wheel Unit (UWU).
//
// Key design changes from UWU version:
//   1. Extended 2x6 frame tubes pass through outer bearing mount plates
//   2. Motor mounting plates (between walls) are REMOVED - UTU motor plate
//      is the inner sandwich plate of the machine walls
//   3. Outer sandwich plate serves as bearing mount for idlers and motor axis
//   4. New triangular bearing mount plates at ends of extended 2x6 tubes
//      covering the area between the 3 UTU axes
//   5. Same angle iron mounting used for 2x6 tubes at new plate

// ============================================================
// INCLUDES
// ============================================================

// Include params directly (executes parameter definitions)
include <lifetrac_v25_params.scad>

// Use the main assembly file to import all modules (side_panel_*, stiffener_*,
// loader_arms, bucket_attachment, lift_cylinders, etc.) WITHOUT executing
// the top-level lifetrac_v25_assembly() call or render statements.
// OpenSCAD's `use` imports modules/functions but skips top-level statements.
use <lifetrac_v25.scad>

// Also use the individual module/part files for any direct references
use <modules/plate_steel.scad>
use <modules/structural_steel.scad>
use <modules/fasteners.scad>
use <modules/hydraulics.scad>
use <modules/wheels.scad>
use <modules/loader_arm_v2.scad>
use <parts/structural/structural_parts.scad>

// ============================================================
// RE-DERIVED CONSTANTS
// ============================================================
// Variables computed at top-level scope in lifetrac_v25.scad are NOT
// imported by `use`. We re-derive all variables needed by the modules
// we call (side panels, stiffeners, loader arms, etc.).

// --- Display toggles (we override some) ---
show_wheels = false;    // No UWU wheels in UTU variant
show_uwu = false;      // No UWU assemblies
all_wheel_drive = true;
exploded_view = false;
explode_distance = 0;

// --- Engine ---
ENGINE_HP = 25;
ENGINE_WEIGHT_KG = 30 + (ENGINE_HP * 1.2);
ENGINE_POS_Y = 400;
ENGINE_POS_Z = FRAME_Z_OFFSET + 200;

// --- Misc bolt sizes ---
BOLT_DIA_3_8 = 9.525;

// --- Animation ---
animation_time = $t;
animation_phase = animation_time < 0.5 ? (animation_time * 2) : (2 - animation_time * 2);
_arm_max_for_animation = is_undef(ARM_MAX_ANGLE_LIMITED) ? ARM_MAX_ANGLE : ARM_MAX_ANGLE_LIMITED;
ARM_LIFT_ANGLE = ARM_MIN_ANGLE + (animation_phase * (_arm_max_for_animation - ARM_MIN_ANGLE));

// --- Bucket animation ---
_bucket_abs_at_ground = 0;
_bucket_abs_at_raised = -45;
_anim_abs_angle = ($t == 0) ? _bucket_abs_at_ground
    : (_bucket_abs_at_ground + (animation_phase * (_bucket_abs_at_raised - _bucket_abs_at_ground)));
BUCKET_TILT_ANGLE = _anim_abs_angle - ARM_LIFT_ANGLE;
BUCKET_GROUND_TILT = -ARM_MIN_ANGLE;
BUCKET_MAX_CURL = 60;
BUCKET_MAX_DUMP = -45;

// --- Bobcat QA (needed by bucket modules) ---
BOBCAT_QA_HOOK_HEIGHT = 203;
BOBCAT_QA_WEDGE_HEIGHT = 76;

// --- Arm geometry ---
ARM_HORIZONTAL_REACH = ARM_LENGTH * cos(ARM_GROUND_ANGLE);

// --- Bottom plate parameters ---
BOTTOM_PLATE_FRONT_OFFSET = 50.8;
BOTTOM_PLATE_REAR_OFFSET = 152.4;
BOTTOM_PLATE_Y_START = BOTTOM_PLATE_REAR_OFFSET;
BOTTOM_PLATE_Y_END = WHEEL_BASE - BOTTOM_PLATE_FRONT_OFFSET;
BOTTOM_PLATE_LENGTH = BOTTOM_PLATE_Y_END - BOTTOM_PLATE_Y_START;
BOTTOM_PLATE_INNER_TRIM = 31.75;

// --- Cross frame tube parameters ---
FRAME_TUBE_WIDTH = 50.8;    // 2" depth in Y
FRAME_TUBE_HEIGHT = 152.4;  // 6" height in Z
FRAME_TUBE_WALL = 6.35;     // 1/4" wall
FRAME_TUBE_RADIUS = 12.7;   // 1/2" corner radius
FRAME_TUBE_CNC_RADIUS = 3.175;  // 1/8" CNC radius
FRAME_TUBE_GAP = 50.8;      // 2" gap above bottom plate
FRAME_TUBE_Z = FRAME_Z_OFFSET + BOTTOM_PLATE_INNER_TRIM + FRAME_TUBE_GAP;

// --- Wheel axis positions ---
_FRONT_WHEEL_AXIS_Y = WHEEL_BASE - WHEEL_RADIUS;
_REAR_WHEEL_TARGET_Y = LIFT_CYL_BASE_Y + WHEEL_RADIUS + 100;
_MIN_WHEELBASE = WHEEL_DIAMETER + 50;
_MAX_REAR_Y = _FRONT_WHEEL_AXIS_Y - _MIN_WHEELBASE;
_REAR_WHEEL_AXIS_Y = min(_REAR_WHEEL_TARGET_Y, _MAX_REAR_Y);

// --- Frame tube Y positions ---
FRAME_TUBE_OFFSET_FROM_WHEEL = 152.4;  // 6"
FRONT_FRAME_TUBE_Y = _FRONT_WHEEL_AXIS_Y - FRAME_TUBE_OFFSET_FROM_WHEEL - FRAME_TUBE_WIDTH;
REAR_FRAME_TUBE_Y = _REAR_WHEEL_AXIS_Y + FRAME_TUBE_OFFSET_FROM_WHEEL;
FRAME_TUBE_EXTENSION = 12.7;
FRAME_TUBE_LENGTH = TRACK_WIDTH + SANDWICH_SPACING + 2 * PANEL_THICKNESS + 2 * FRAME_TUBE_EXTENSION;

// --- Split angle iron segments ---
ANGLE_IRON_GAP = 203.2;
_REAR_GAP_CENTER_REL = _REAR_WHEEL_AXIS_Y - BOTTOM_PLATE_Y_START;
_FRONT_GAP_CENTER_REL = _FRONT_WHEEL_AXIS_Y - BOTTOM_PLATE_Y_START;
ANGLE_SEGMENT_1_START = 0;
ANGLE_SEGMENT_1_END = _REAR_GAP_CENTER_REL - ANGLE_IRON_GAP / 2;
ANGLE_SEGMENT_1_LENGTH = ANGLE_SEGMENT_1_END - ANGLE_SEGMENT_1_START;
ANGLE_SEGMENT_2_START = _REAR_GAP_CENTER_REL + ANGLE_IRON_GAP / 2;
ANGLE_SEGMENT_2_END = _FRONT_GAP_CENTER_REL - ANGLE_IRON_GAP / 2;
ANGLE_SEGMENT_2_LENGTH = ANGLE_SEGMENT_2_END - ANGLE_SEGMENT_2_START;
ANGLE_SEGMENT_3_START = _FRONT_GAP_CENTER_REL + ANGLE_IRON_GAP / 2;
ANGLE_SEGMENT_3_END = BOTTOM_PLATE_LENGTH;
ANGLE_SEGMENT_3_LENGTH = ANGLE_SEGMENT_3_END - ANGLE_SEGMENT_3_START;

// --- Motor plate parameters (needed by stiffener/bucket modules) ---
MOTOR_PLATE_HEIGHT = 254.0;       // 10"
MOTOR_PLATE_INBOARD = 152.4;     // 6"
MOTOR_PLATE_THICKNESS = PLATE_1_4_INCH;
MOTOR_PLATE_X = (TRACK_WIDTH / 2 - SANDWICH_SPACING / 2) - MOTOR_PLATE_INBOARD;

// --- Bolt spacing ---
MIN_BOLT_SPACING = 101.6;

// --- UWU positioning (needed by side panel bearing holes) ---
UWU_SHAFT_Z = FRAME_Z_OFFSET + BOTTOM_PLATE_INNER_TRIM + UWU_SHAFT_AXIS_HEIGHT_ON_PLATE;
INNER_WALL_PANEL_X = TRACK_WIDTH / 2 - SANDWICH_SPACING / 2;
UWU_MOTOR_TO_WALL_DIST = INNER_WALL_PANEL_X - MOTOR_PLATE_X;

// --- Bucket cylinder mount points ---
INNER_PANEL_X = TRACK_WIDTH / 2 - SANDWICH_SPACING / 2;
BUCKET_CYL_ARM_POS = CROSS_BEAM_1_POS;
BUCKET_CYL_ARM_Z = CROSS_BEAM_SIZE / 2 + 30;
BUCKET_CYL_X_SPACING = INNER_PANEL_X - 100;
BUCKET_CYL_BUCKET_Y = 100;
BUCKET_CYL_BUCKET_Z = BOBCAT_QA_HEIGHT - 60;

// --- Hydraulic analysis constants (for echo output) ---
HYDRAULIC_PRESSURE_PSI = 3000;
HYDRAULIC_PRESSURE_MPA = HYDRAULIC_PRESSURE_PSI * 0.00689476;
CYLINDER_STROKE_MARGIN = 1.15;
CYLINDER_CLOSED_MARGIN = 1.10;

// --- Bucket tilt range ---
BUCKET_MIN_TILT = BUCKET_GROUND_TILT;
BUCKET_MAX_TILT = 90;

$fn = 32;

// ============================================================
// DISPLAY OPTIONS
// ============================================================

show_frame = true;
show_hydraulics = true;
show_loader_arms = true;
show_bucket = true;
show_folding_platform = true;
platform_fold_angle = 90;
show_utu = true;           // Show UTU assemblies
show_track = true;          // Show track chain
animate_track = false;      // Animate track (use View > Animate)

// ============================================================
// UTU GEOMETRY PARAMETERS (from utu_v25.scad)
// ============================================================

inch = 25.4;
PI = 3.14159265359;
eps = 0.1;

// UTU axis layout - derived from UWU wheel axis positions
utu_idler_spacing = _FRONT_WHEEL_AXIS_Y - _REAR_WHEEL_AXIS_Y;  // Matches UWU wheelbase

// Sprocket parameters (from UTU)
utu_num_teeth = 12;
utu_track_bar_width = 2.5 * inch;
utu_track_gap = 0.5 * inch;
utu_chain_pitch = utu_track_bar_width + utu_track_gap;
utu_sprocket_pitch_radius = utu_chain_pitch / (2 * sin(180 / utu_num_teeth));
utu_link_rod_diam = 1 * inch;
utu_rod_clearance = 2;
utu_rod_seat_r = (utu_link_rod_diam + utu_rod_clearance) / 2;
utu_sprocket_tip_r = utu_sprocket_pitch_radius + utu_rod_seat_r;
utu_sprocket_od = 2 * utu_sprocket_tip_r;

// Drive height: motor axis positioned just above frame tube top + sprocket clearance
utu_drive_height = (FRAME_TUBE_Z + FRAME_TUBE_HEIGHT + utu_sprocket_tip_r + 25.4) - UWU_SHAFT_Z;

// Track bar dimensions
utu_track_bar_length = 10 * inch;
utu_track_bar_thickness = 0.5 * inch;
utu_connector_height = 2 * inch;
utu_rod_z_above_bar = utu_track_bar_thickness + utu_connector_height / 2;

// Bearing/motor plate dimensions (matching UWU interface)
utu_plate_t = 0.25 * inch;
utu_shaft_diam = 1.25 * inch;
utu_shaft_length = 14 * inch;
utu_sprocket_thickness = 0.5 * inch;

// Motor dimensions
utu_motor_body_diam = 6 * inch;
utu_motor_body_len = 5 * inch;

// ============================================================
// UTU POSITIONING ON LIFETRAC FRAME
// ============================================================

// The UTU replaces front and rear wheel sets on each side.
// Each side gets ONE UTU (not two separate units).
// The UTU re-uses the UWU wheel axis bearing positions on the wall panels.

// UTU center Y position: midpoint between front and rear wheel axes
UTU_CENTER_Y = (_FRONT_WHEEL_AXIS_Y + _REAR_WHEEL_AXIS_Y) / 2;

// ============================================================
// OUTER WALL PANEL X POSITIONS (from main assembly)
// ============================================================

// Inner wall panels (where UTU motor plate mounts = inner sandwich plate)
INNER_WALL_X = TRACK_WIDTH / 2 - SANDWICH_SPACING / 2;

// Outer wall panels (where bearing mounts go)
OUTER_WALL_X = TRACK_WIDTH / 2 + SANDWICH_SPACING / 2;

// ============================================================
// EXTENDED FRAME TUBE PARAMETERS
// ============================================================

// Original frame tube length (spans between outer panels + small extension)
ORIG_FRAME_TUBE_LENGTH = TRACK_WIDTH + SANDWICH_SPACING + 2 * PANEL_THICKNESS + 2 * 12.7;

// Extended frame tube: passes through outer wall, continues to new bearing plate
// Extension distance = enough to reach past the UTU outer bearing plate
// The new plate is mounted outside the outer wall panel
UTU_OUTER_PLATE_OFFSET = 8 * inch;  // 8" beyond outer wall panel face
UTU_OUTER_PLATE_THICKNESS = PLATE_1_2_INCH;  // 1/2" plate for bearing mount

// Lateral clearance between outer wall panel outer face and the inside edge
// of the UTU track ground bars. The bars are utu_track_bar_length wide in
// the cross-machine direction (after the 90° rotation in utu_side_assembly)
// and centered on the sprocket plane, so half the bar length sits inboard
// of the sprocket. Push the sprocket outward enough to clear the wall plate
// by this gap.
UTU_WALL_TO_TRACK_GAP = 1 * inch;

// Sprocket X position: shifted outward so the inside edge of the track
// ground bars sits UTU_WALL_TO_TRACK_GAP outboard of the outer wall panel.
UTU_SPROCKET_X = OUTER_WALL_X + PANEL_THICKNESS
                 + UTU_WALL_TO_TRACK_GAP
                 + utu_track_bar_length / 2;

// Extended tube length: from centerline to new plate on each side
// Tube center is at X=0, extends symmetrically
// Need to reach: OUTER_WALL_X + PANEL_THICKNESS + UTU_OUTER_PLATE_OFFSET + UTU_OUTER_PLATE_THICKNESS
UTU_TUBE_HALF_LENGTH = OUTER_WALL_X + PANEL_THICKNESS + UTU_OUTER_PLATE_OFFSET + UTU_OUTER_PLATE_THICKNESS + 12.7;
UTU_FRAME_TUBE_LENGTH = 2 * UTU_TUBE_HALF_LENGTH;

// Frame tube dimensions (same as main assembly)
UTU_FRAME_TUBE_WIDTH = 50.8;    // 2" depth in Y
UTU_FRAME_TUBE_HEIGHT = 152.4;  // 6" height in Z

// Z position from re-derived constants
UTU_FRAME_TUBE_Z = FRAME_TUBE_Z;

echo("=== UTU FRAME TUBE EXTENSION ===");
echo("Original tube length:", ORIG_FRAME_TUBE_LENGTH, "mm");
echo("Extended tube length:", UTU_FRAME_TUBE_LENGTH, "mm");
echo("Extension per side:", UTU_TUBE_HALF_LENGTH - ORIG_FRAME_TUBE_LENGTH/2, "mm");

// ============================================================
// UTU AXIS POSITIONS (WORLD COORDINATES)
// ============================================================

// Front idler is shifted FORWARD of the original UWU wheel axis so the
// front of the tracks sits ~UTU_FRONT_TRACK_TO_BUCKET_GAP behind the
// back of the bucket. The "front of the tracks" world Y =
//   UTU_FRONT_IDLER_Y + utu_sprocket_tip_r
// (i.e., the forward edge of where the chain wraps around the front idler).
//
// The bucket back Y depends on the loader-arm kinematics (ARM_PIVOT_Y,
// ARM_TIP_X, BUCKET_PIVOT_Y_OFFSET, ARM_LIFT_ANGLE, BUCKET_LUG_OFFSET, ...)
// so we expose a simple shift parameter rather than try to solve for it
// here. Verify visually and adjust UTU_FRONT_IDLER_FORWARD_SHIFT until the
// echoed "Front of tracks Y" sits 4" behind your bucket back.
UTU_FRONT_TRACK_TO_BUCKET_GAP = 4 * inch;  // target gap (for documentation)
UTU_FRONT_IDLER_FORWARD_SHIFT = 150;        // mm forward of UWU front axis
// Minimum desired rearward shift; the actual shift is bumped up below so
// the full 3-sprocket chain loop length is an integer multiple of the
// chain pitch (so every tooth engages cleanly).
UTU_REAR_IDLER_BACKWARD_SHIFT_MIN = 2 * inch;

UTU_FRONT_IDLER_Y = _FRONT_WHEEL_AXIS_Y + UTU_FRONT_IDLER_FORWARD_SHIFT;

// Drive axis Y: centered over the rear cross frame tube. (REAR_FRAME_TUBE_Y
// is the rear face of the tube; add half the tube width to land on its
// centerline.)
UTU_DRIVE_Y = REAR_FRAME_TUBE_Y + FRAME_TUBE_WIDTH / 2;

// Z positions:
//   - Drive axis: kept at its original height above the frame tubes so the
//     hydraulic motor still mounts at the same place on the inner wall panel.
//   - Idler axes: LOWERED away from the original UWU shaft mount height so
//     that the bottom run of the chain sits at ground level (Z = 0). The
//     bottom of the sprocket pitch circle then sits at the chain-pin height
//     above the ground bar (utu_rod_z_above_bar). The UWU shaft mounting
//     points on the wall panels are unchanged.
UTU_DRIVE_Z = FRAME_TUBE_Z + FRAME_TUBE_HEIGHT + utu_sprocket_tip_r + 0.25 * inch;
UTU_IDLER_Z = utu_rod_z_above_bar + utu_sprocket_pitch_radius;

// ----------------------------------------------------------------
// Chain-pitch snapping for the 3-sprocket loop
// ----------------------------------------------------------------
// Compute the closed-loop chain length around: front idler (F) -> drive
// (D) at top -> rear idler (R) -> back to F along the bottom run.
// All sprockets share the same pitch radius `utu_sprocket_pitch_radius`,
// so the chain wraps over each by an angle determined only by the
// triangle of axis centers (external tangents to equal-radius circles).
// Total loop length = sum of three straight tangent segments + sum of
// three arc lengths = (perimeter of triangle of centers) + 2*PI*R.
function _utu_loop_length_for_rear_y(rear_y) =
    let(
        F = [UTU_FRONT_IDLER_Y, UTU_IDLER_Z],
        D = [UTU_DRIVE_Y,        UTU_DRIVE_Z],
        R = [rear_y,             UTU_IDLER_Z],
        d_FD = norm(D - F),
        d_DR = norm(R - D),
        d_RF = norm(F - R)
    )
    d_FD + d_DR + d_RF + 2 * PI * utu_sprocket_pitch_radius;

// Initial (un-snapped) rear-idler Y assuming the minimum backward shift.
_utu_rear_y_initial = _REAR_WHEEL_AXIS_Y - UTU_REAR_IDLER_BACKWARD_SHIFT_MIN;
_utu_loop_initial   = _utu_loop_length_for_rear_y(_utu_rear_y_initial);
// Round UP so the rear idler can only move further back (never closer
// than UTU_REAR_IDLER_BACKWARD_SHIFT_MIN).
_utu_target_links   = ceil(_utu_loop_initial / utu_chain_pitch);
_utu_target_loop    = _utu_target_links * utu_chain_pitch;
// The loop length grows almost 1:1 with rear-idler movement away from
// the drive (since both bottom run and one diagonal lengthen). Iterate
// a few times to converge precisely.
function _utu_solve_rear_y(rear_y, iter) =
    iter <= 0 ? rear_y
    : _utu_solve_rear_y(
          rear_y - (_utu_target_loop - _utu_loop_length_for_rear_y(rear_y)),
          iter - 1);
UTU_REAR_IDLER_Y = _utu_solve_rear_y(_utu_rear_y_initial, 20);
UTU_REAR_IDLER_BACKWARD_SHIFT = _REAR_WHEEL_AXIS_Y - UTU_REAR_IDLER_Y;
UTU_CHAIN_LOOP_LENGTH = _utu_loop_length_for_rear_y(UTU_REAR_IDLER_Y);
UTU_CHAIN_NUM_LINKS = round(UTU_CHAIN_LOOP_LENGTH / utu_chain_pitch);

// ----------------------------------------------------------------
// Per-sprocket phase angles (deg) so a tooth lands on a chain pin
// ----------------------------------------------------------------
// Build the same chain geometry as utu_track_chain so we can compute
// where each sprocket's arc starts along the loop and what angle the
// first chain pin on that arc occupies. Phase = pin_angle + 90 (the +90
// converts chain-frame angle to sprocket-local angle, accounting for the
// rotate([0,90,0]) used to place the sprocket).
function _utu_perp_cw(v)    = [v[1], -v[0]];
function _utu_unit(v)       = v / norm(v);
function _utu_tan_off(P, Q) = utu_sprocket_pitch_radius * _utu_perp_cw(_utu_unit(Q - P));
function _utu_ccw(from_a, to_a) =
    let(d = ((to_a - from_a) % 360 + 360) % 360) d;

// Sprocket centers in chain-local XZ (X = world Y, Z = world Z):
_utu_A = [UTU_REAR_IDLER_Y,  UTU_IDLER_Z];   // rear idler
_utu_B = [UTU_FRONT_IDLER_Y, UTU_IDLER_Z];   // front idler
_utu_C = [UTU_DRIVE_Y,       UTU_DRIVE_Z];   // drive

_utu_A_dep = _utu_A + _utu_tan_off(_utu_A, _utu_B);
_utu_B_arr = _utu_B + _utu_tan_off(_utu_A, _utu_B);
_utu_B_dep = _utu_B + _utu_tan_off(_utu_B, _utu_C);
_utu_C_arr = _utu_C + _utu_tan_off(_utu_B, _utu_C);
_utu_C_dep = _utu_C + _utu_tan_off(_utu_C, _utu_A);
_utu_A_arr = _utu_A + _utu_tan_off(_utu_C, _utu_A);

function _utu_ang(c, p) = atan2(p[1] - c[1], p[0] - c[0]);
_utu_aA_arr = _utu_ang(_utu_A, _utu_A_arr);
_utu_aA_dep = _utu_ang(_utu_A, _utu_A_dep);
_utu_aB_arr = _utu_ang(_utu_B, _utu_B_arr);
_utu_aB_dep = _utu_ang(_utu_B, _utu_B_dep);
_utu_aC_arr = _utu_ang(_utu_C, _utu_C_arr);
_utu_aC_dep = _utu_ang(_utu_C, _utu_C_dep);

_utu_wrapA = _utu_ccw(_utu_aA_arr, _utu_aA_dep);
_utu_wrapB = _utu_ccw(_utu_aB_arr, _utu_aB_dep);
_utu_wrapC = _utu_ccw(_utu_aC_arr, _utu_aC_dep);

_utu_segAB = norm(_utu_B_arr - _utu_A_dep);
_utu_segBC = norm(_utu_C_arr - _utu_B_dep);
_utu_segCA = norm(_utu_A_arr - _utu_C_dep);
_utu_arcA  = utu_sprocket_pitch_radius * _utu_wrapA * PI / 180;
_utu_arcB  = utu_sprocket_pitch_radius * _utu_wrapB * PI / 180;
_utu_arcC  = utu_sprocket_pitch_radius * _utu_wrapC * PI / 180;

// Arc-start arclength along the loop (loop starts at A_dep, s=0):
_utu_sB_start = _utu_segAB;
_utu_sC_start = _utu_segAB + _utu_arcB + _utu_segBC;
_utu_sA_start = _utu_segAB + _utu_arcB + _utu_segBC + _utu_arcC + _utu_segCA;

// First chain-pin position (rod) on each arc (rods are at integer pitch
// multiples starting from s=0, the A_dep boundary):
function _utu_first_pin_on_arc(s_start) =
    let(t = (utu_chain_pitch - (s_start - floor(s_start / utu_chain_pitch) * utu_chain_pitch))
            % utu_chain_pitch)
    (t < 1e-6) ? 0 : t;

_utu_tA = _utu_first_pin_on_arc(_utu_sA_start);
_utu_tB = _utu_first_pin_on_arc(_utu_sB_start);
_utu_tC = _utu_first_pin_on_arc(_utu_sC_start);

// First-pin angle on each sprocket arc, then convert to sprocket phase
// (phase = chain-frame pin angle + 90, taken mod tooth pitch).
_utu_tooth_pitch_deg = 360 / utu_num_teeth;
function _utu_phase_for(arr_deg, t_first) =
    ((arr_deg + (t_first / utu_sprocket_pitch_radius) * (180 / PI) + 90)
        % _utu_tooth_pitch_deg + _utu_tooth_pitch_deg) % _utu_tooth_pitch_deg;

UTU_REAR_SPROCKET_PHASE  = _utu_phase_for(_utu_aA_arr, _utu_tA);
UTU_FRONT_SPROCKET_PHASE = _utu_phase_for(_utu_aB_arr, _utu_tB);
UTU_DRIVE_SPROCKET_PHASE = _utu_phase_for(_utu_aC_arr, _utu_tC);

echo("=== UTU SPROCKET PHASES ===");
echo("Rear  sprocket phase:", UTU_REAR_SPROCKET_PHASE,  "deg");
echo("Front sprocket phase:", UTU_FRONT_SPROCKET_PHASE, "deg");
echo("Drive sprocket phase:", UTU_DRIVE_SPROCKET_PHASE, "deg");

// Keep utu_drive_height as the (idler -> drive) Z delta for the chain layout
// and other consumers. (utu_drive_height was previously a top-level constant;
// derive it here from the now-independent UTU_DRIVE_Z / UTU_IDLER_Z.)
utu_drive_height_actual = UTU_DRIVE_Z - UTU_IDLER_Z;

echo("=== UTU AXIS POSITIONS ===");
echo("Front idler Y:", UTU_FRONT_IDLER_Y, "mm");
echo("Rear idler Y:", UTU_REAR_IDLER_Y, "mm");
echo("Drive axis Y:", UTU_DRIVE_Y, "mm");
echo("Idler Z:", UTU_IDLER_Z, "mm");
echo("Drive Z:", UTU_DRIVE_Z, "mm");
echo("UTU Center Y:", UTU_CENTER_Y, "mm");
echo("Front of tracks Y:", UTU_FRONT_IDLER_Y + utu_sprocket_tip_r, "mm");
echo("Front idler forward shift:", UTU_FRONT_IDLER_FORWARD_SHIFT, "mm",
     "  (target track-to-bucket gap:", UTU_FRONT_TRACK_TO_BUCKET_GAP, "mm)");
echo("Rear idler backward shift (snapped):", UTU_REAR_IDLER_BACKWARD_SHIFT, "mm",
     "  (min:", UTU_REAR_IDLER_BACKWARD_SHIFT_MIN, "mm)");
echo("Chain loop length:", UTU_CHAIN_LOOP_LENGTH, "mm",
     "  num links:", UTU_CHAIN_NUM_LINKS,
     "  pitch:", utu_chain_pitch, "mm");

// ============================================================
// NEW OUTER BEARING MOUNT PLATE
// ============================================================
// Triangular plate with rounded corners covering the area between
// the 3 UTU axis positions. Mounted to the extended 2x6 frame tubes
// using angle irons. Has bearing mount holes at all 3 axis positions.

// X position of new plate (outside outer wall)
UTU_BEARING_PLATE_X = OUTER_WALL_X + PANEL_THICKNESS + UTU_OUTER_PLATE_OFFSET;

// Plate corner positions (YZ plane) - the 3 axis centers
// These define the triangle vertices
_bp_y1 = UTU_REAR_IDLER_Y;   // Rear idler
_bp_z1 = UTU_IDLER_Z;
_bp_y2 = UTU_FRONT_IDLER_Y;  // Front idler
_bp_z2 = UTU_IDLER_Z;
_bp_y3 = UTU_DRIVE_Y;         // Drive (top center)
_bp_z3 = UTU_DRIVE_Z;

// Padding around triangle vertices for bearing bolt clearance
UTU_BEARING_PLATE_PADDING = UWU_BOLT_CIRCLE_DIAM / 2 + UWU_BOLT_HOLE_DIAM + 25.4;  // BCD/2 + hole + 1" margin

echo("=== UTU BEARING PLATE ===");
echo("Plate X position:", UTU_BEARING_PLATE_X, "mm");
echo("Plate padding:", UTU_BEARING_PLATE_PADDING, "mm");

// ============================================================
// NEW INNER BEARING MOUNT PLATE (idlers only)
// ============================================================
// A second bearing-mount plate placed between the outer wall panel and
// the sprocket plane. It picks up the inner bearings of the front and
// rear idler shafts (the drive shaft is unaffected and still uses the
// outer wall panel bearing). The plate is a rounded oblong covering
// only the two idler axis positions (no top/drive vertex).

// Outboard face X of the new inner plate. We mirror the spacing used on
// the outer side: the outer plate's inboard face sits some distance from
// the sprocket's outboard face; place this inner plate's outboard face
// the same distance from the sprocket's inboard face. By symmetry around
// the sprocket centerline this works out to:
//   UTU_INNER_BEARING_PLATE_X = 2*UTU_SPROCKET_X - UTU_BEARING_PLATE_X
UTU_INNER_PLATE_THICKNESS = UTU_OUTER_PLATE_THICKNESS;  // same 1/2" plate
UTU_OUTER_PLATE_TO_SPROCKET_GAP = UTU_BEARING_PLATE_X
                                  - (UTU_SPROCKET_X + utu_sprocket_thickness / 2);
UTU_INNER_PLATE_SPROCKET_CLEARANCE = UTU_OUTER_PLATE_TO_SPROCKET_GAP;
UTU_INNER_BEARING_PLATE_X = UTU_SPROCKET_X
                            - utu_sprocket_thickness / 2
                            - UTU_INNER_PLATE_SPROCKET_CLEARANCE;  // outboard face of plate

echo("=== UTU INNER BEARING PLATE ===");
echo("Inner plate outboard X:", UTU_INNER_BEARING_PLATE_X, "mm");
echo("Inner plate inboard X:", UTU_INNER_BEARING_PLATE_X - UTU_INNER_PLATE_THICKNESS, "mm");

// ============================================================
// MODULES - UTU Outer Bearing Mount Plate
// ============================================================

// Triangular plate with rounded corners covering 3 UTU axis positions
// Oriented in YZ plane at the bearing plate X position
// Has bearing mount holes at all 3 positions and frame tube bolt holes
module utu_outer_bearing_plate() {
    pad = UTU_BEARING_PLATE_PADDING;
    plate_t = UTU_OUTER_PLATE_THICKNESS;

    // Triangle vertices (in local YZ, but we'll work in 2D then extrude)
    // Local coordinates: Y = world Y, Z = world Z
    p1 = [_bp_y1, _bp_z1];  // Rear idler
    p2 = [_bp_y2, _bp_z2];  // Front idler
    p3 = [_bp_y3, _bp_z3];  // Drive axis (top)

    color("DarkBlue")
    translate([0, 0, 0])
    rotate([0, 0, 0])
    difference() {
        // Rounded triangle plate
        linear_extrude(height = plate_t)
        offset(r = 25.4)      // 1" rounding on outer corners
        offset(r = -25.4)
        offset(r = pad)        // Expand triangle by padding
        polygon([p1, p2, p3]);

        // Bearing mount holes at all 3 axis positions
        // Front idler
        translate([_bp_y2, _bp_z2, plate_t / 2])
        utu_bearing_hole_pattern(plate_t);

        // Rear idler
        translate([_bp_y1, _bp_z1, plate_t / 2])
        utu_bearing_hole_pattern(plate_t);

        // Drive axis
        translate([_bp_y3, _bp_z3, plate_t / 2])
        utu_bearing_hole_pattern(plate_t);

        // Frame tube angle iron bolt holes
        // Front tube passes through at FRONT_FRAME_TUBE_Y
        // Need bolt holes for angle irons on both faces of each tube
        utu_plate_frame_tube_holes(_FRONT_WHEEL_AXIS_Y - 152.4 - UTU_FRAME_TUBE_WIDTH, plate_t);
        utu_plate_frame_tube_holes(_REAR_WHEEL_AXIS_Y + 152.4, plate_t);

        // Rectangular pass-through cutouts for the 2"x6" frame tubes
        utu_plate_frame_tube_cutout(_FRONT_WHEEL_AXIS_Y - 152.4 - UTU_FRAME_TUBE_WIDTH, plate_t);
        utu_plate_frame_tube_cutout(_REAR_WHEEL_AXIS_Y + 152.4, plate_t);
    }
}

// Inner bearing mount plate: covers the rear and front idler axes (now
// lowered to ground-engagement height) and extends UPWARD to anchor on the
// 2x6 cross frame tubes via angle irons. The outer plate (triangular)
// already auto-extends downward through its `offset` padding around the
// idler vertices; the inner plate is built explicitly so that its upper
// section keeps the original frame-tube attachment height while only the
// lower portion follows the lowered idler bearings.
module utu_inner_idler_bearing_plate() {
    pad = UTU_BEARING_PLATE_PADDING;
    plate_t = UTU_INNER_PLATE_THICKNESS;

    p1 = [_bp_y1, _bp_z1];  // Rear idler (lowered)
    p2 = [_bp_y2, _bp_z2];  // Front idler (lowered)

    // Frame tube anchor rectangles (Y x Z) at the original tube heights so
    // the angle iron bolt holes near the top of the plate stay covered.
    // Extend the rectangles upward by UTU_INNER_PLATE_TOP_MARGIN to leave
    // material above the cross frame tubes.
    UTU_INNER_PLATE_TOP_MARGIN = 3 * inch;
    front_tube_y = _FRONT_WHEEL_AXIS_Y - 152.4 - UTU_FRAME_TUBE_WIDTH;
    rear_tube_y  = _REAR_WHEEL_AXIS_Y + 152.4;
    tube_w = UTU_FRAME_TUBE_WIDTH;
    tube_h = UTU_FRAME_TUBE_HEIGHT + UTU_INNER_PLATE_TOP_MARGIN;
    tube_z = UTU_FRAME_TUBE_Z;

    color("DarkBlue")
    difference() {
        // Convex hull of the two lowered idler bearing pads + the two frame
        // tube anchor rectangles. Smooth + round the resulting outline.
        linear_extrude(height = plate_t)
        offset(r = 25.4) offset(r = -25.4)
        hull() {
            translate(p1) circle(r = pad, $fn = 48);
            translate(p2) circle(r = pad, $fn = 48);
            translate([front_tube_y, tube_z]) square([tube_w, tube_h]);
            translate([rear_tube_y,  tube_z]) square([tube_w, tube_h]);
        }

        // Bearing mount holes at both lowered idler positions
        translate([_bp_y1, _bp_z1, plate_t / 2])
        utu_bearing_hole_pattern(plate_t);

        translate([_bp_y2, _bp_z2, plate_t / 2])
        utu_bearing_hole_pattern(plate_t);

        // Frame tube angle iron bolt holes (same tubes pass through here)
        utu_plate_frame_tube_holes(front_tube_y, plate_t);
        utu_plate_frame_tube_holes(rear_tube_y,  plate_t);

        // Rectangular pass-through cutouts for the 2"x6" frame tubes
        utu_plate_frame_tube_cutout(front_tube_y, plate_t);
        utu_plate_frame_tube_cutout(rear_tube_y,  plate_t);
    }
}

// UWU-compatible 4-bolt bearing hole pattern (for the new plate)
// Applied in the YZ plane
module utu_bearing_hole_pattern(plate_thickness) {
    // Center clearance hole
    cylinder(d = UWU_CENTER_HOLE_DIAM, h = plate_thickness + 4, center = true, $fn = 32);

    // 4-bolt bearing mount pattern at 45-degree orientation
    for (angle = UWU_BOLT_ANGLES) {
        rotate([0, 0, angle])
        translate([UWU_BOLT_CIRCLE_DIAM / 2, 0, 0])
        cylinder(d = UWU_BOLT_HOLE_DIAM, h = plate_thickness + 4, center = true, $fn = 16);
    }
}

// Frame tube angle iron bolt holes in the new bearing plate
module utu_plate_frame_tube_holes(tube_y, plate_thickness) {
    angle_leg = 50.8;
    bolt_dia = BOLT_DIA_1_2;
    plate_bolt_offset = 50.8;  // 2" spacing from center
    angle_trim = 3.175;
    angle_len = UTU_FRAME_TUBE_HEIGHT - 2 * angle_trim;

    // Z position of angle centered on tube
    angle_z = UTU_FRAME_TUBE_Z + angle_trim;
    angle_center_z = angle_z + angle_len / 2;

    // Front face of tube
    for (dz = [-plate_bolt_offset, plate_bolt_offset]) {
        translate([tube_y + UTU_FRAME_TUBE_WIDTH + angle_leg / 2, angle_center_z + dz, plate_thickness / 2])
        cylinder(d = bolt_dia, h = plate_thickness + 4, center = true, $fn = 16);
    }

    // Rear face of tube
    for (dz = [-plate_bolt_offset, plate_bolt_offset]) {
        translate([tube_y - angle_leg / 2, angle_center_z + dz, plate_thickness / 2])
        cylinder(d = bolt_dia, h = plate_thickness + 4, center = true, $fn = 16);
    }
}

// Rectangular through-cutout for a 2"x6" frame tube to pass through the
// plate. tube_y is the tube's near (lower-Y) face; tube extends +Y by
// UTU_FRAME_TUBE_WIDTH and +Z from UTU_FRAME_TUBE_Z by UTU_FRAME_TUBE_HEIGHT.
// Inside corners are rounded to match the tube's outside corner radius
// (1/2") since CNC plasma/torch cuts cannot produce sharp inside corners.
UTU_PLATE_TUBE_CLEARANCE = 1.5;  // ~1/16" all around (snug slip fit)
UTU_PLATE_TUBE_CORNER_R  = 12.7; // 1/2" matches the tube's outer corner R
module utu_plate_frame_tube_cutout(tube_y, plate_thickness) {
    c = UTU_PLATE_TUBE_CLEARANCE;
    r = UTU_PLATE_TUBE_CORNER_R + c;  // hole inner radius >= tube outer R
    translate([0, 0, -1])
    linear_extrude(height = plate_thickness + 2)
    offset(r = r) offset(delta = -r)
    translate([tube_y - c, UTU_FRAME_TUBE_Z - c])
    square([UTU_FRAME_TUBE_WIDTH + 2 * c,
            UTU_FRAME_TUBE_HEIGHT + 2 * c]);
}

// ============================================================
// MODULES - Extended Frame Tubes
// ============================================================

// Extended 2x6 cross frame tube for UTU variant
// Same 2"x6" cross-section as original but longer to reach new bearing plates
// Profile: 6" wide (Y when installed) × 2" tall (Z when installed)
// Oriented in part coords: X = length, Y = 6" width, Z = 2" height
module utu_extended_frame_tube(length) {
    wall = 6.35;      // 1/4" wall
    r = 12.7;          // 1/2" corner radius
    w = 152.4;         // 6" tube width (becomes Y depth when installed)
    h = 50.8;          // 2" tube height (becomes Z height when installed)

    color("SteelBlue")
    translate([-length / 2, 0, 0])
    rotate([0, 90, 0])
    rotate([0, 0, 90])
    linear_extrude(height = length)
    difference() {
        offset(r = r) offset(r = -r)
        square([w, h]);

        translate([wall, wall])
        offset(r = max(0, r - wall)) offset(r = -max(0, r - wall))
        square([w - 2 * wall, h - 2 * wall]);
    }
}

// Place both extended frame tubes
module utu_cross_frame_tubes() {
    // Front frame tube
    front_tube_y = FRONT_FRAME_TUBE_Y;
    translate([0, front_tube_y + UTU_FRAME_TUBE_WIDTH, UTU_FRAME_TUBE_Z])
    rotate([90, 0, 0])
    utu_extended_frame_tube(UTU_FRAME_TUBE_LENGTH);

    // Rear frame tube
    rear_tube_y = REAR_FRAME_TUBE_Y;
    translate([0, rear_tube_y + UTU_FRAME_TUBE_WIDTH, UTU_FRAME_TUBE_Z])
    rotate([90, 0, 0])
    utu_extended_frame_tube(UTU_FRAME_TUBE_LENGTH);

    // Angle iron mounts on all wall panels + new bearing plates
    utu_frame_tube_angle_irons();
}

// ============================================================
// MODULES - Angle Iron Mounts for Frame Tubes
// ============================================================

// Standard angle iron for frame tube mounting (same as A4 part)
module utu_frame_tube_angle_iron() {
    angle_leg = 50.8;
    angle_thick = 6.35;
    angle_trim = 3.175;
    angle_len = UTU_FRAME_TUBE_HEIGHT - 2 * angle_trim;
    bolt_dia = 9.525;  // 3/8" bolts
    hole_offset = angle_leg * 0.6;

    color("Goldenrod")
    difference() {
        linear_extrude(height = angle_len)
        polygon([
            [0, 0],
            [angle_leg, 0],
            [angle_leg, angle_thick],
            [angle_thick, angle_thick],
            [angle_thick, angle_leg],
            [0, angle_leg]
        ]);

        // Bolt holes on each face
        for (z = get_stiffener_holes_a(angle_len)) {
            translate([hole_offset, 0, z])
            rotate([90, 0, 0])
            cylinder(d = bolt_dia, h = 40, center = true, $fn = 16);
        }
        for (z = get_stiffener_holes_b(angle_len)) {
            translate([0, hole_offset, z])
            rotate([0, 90, 0])
            cylinder(d = bolt_dia, h = 40, center = true, $fn = 16);
        }
    }
}

// Place all angle irons for frame tube mounting
// Includes original 4 wall panels + 2 new bearing plates per tube
module utu_frame_tube_angle_irons() {
    angle_leg = 50.8;
    angle_thick = 6.35;
    angle_trim = 3.175;
    angle_z = UTU_FRAME_TUBE_Z + angle_trim;

    // X positions for inner faces of panels
    left_inner_face = -(TRACK_WIDTH / 2 - SANDWICH_SPACING / 2) + PANEL_THICKNESS;
    left_outer_face = -(TRACK_WIDTH / 2 + SANDWICH_SPACING / 2);
    right_inner_face = (TRACK_WIDTH / 2 - SANDWICH_SPACING / 2) - PANEL_THICKNESS;
    right_outer_face = (TRACK_WIDTH / 2 + SANDWICH_SPACING / 2);

    // New bearing plate faces
    left_bp_face = -UTU_BEARING_PLATE_X;
    right_bp_face = UTU_BEARING_PLATE_X;

    // New inner idler bearing plate faces (inboard face of each plate,
    // since outboard face is too close to the sprocket for an angle iron)
    left_ibp_inboard = -(UTU_INNER_BEARING_PLATE_X - UTU_INNER_PLATE_THICKNESS);
    right_ibp_inboard = (UTU_INNER_BEARING_PLATE_X - UTU_INNER_PLATE_THICKNESS);

    // Y positions of tube faces
    front_tube_y = _FRONT_WHEEL_AXIS_Y - 152.4 - UTU_FRAME_TUBE_WIDTH;
    rear_tube_y = _REAR_WHEEL_AXIS_Y + 152.4;

    front_rear_y = front_tube_y;
    front_front_y = front_tube_y + UTU_FRAME_TUBE_WIDTH;
    rear_rear_y = rear_tube_y;
    rear_front_y = rear_tube_y + UTU_FRAME_TUBE_WIDTH;

    // Helper: place angle irons at a given panel face X for both tubes
    // panel_x: X position of panel inner face
    // toward_center: if true, horizontal leg points toward X=0
    module _place_angles(panel_x, toward_center) {
        for (tube = [0, 1]) {
            tube_rear_y = (tube == 0) ? front_rear_y : rear_rear_y;
            tube_front_y = (tube == 0) ? front_front_y : rear_front_y;

            if (toward_center) {
                // Horizontal leg toward center (+X for left panels)
                translate([panel_x, tube_rear_y, angle_z])
                mirror([0, 1, 0])
                utu_frame_tube_angle_iron();

                translate([panel_x, tube_front_y, angle_z])
                utu_frame_tube_angle_iron();
            } else {
                // Horizontal leg away from center (mirror X)
                translate([panel_x, tube_rear_y, angle_z])
                mirror([1, 1, 0])
                utu_frame_tube_angle_iron();

                translate([panel_x, tube_front_y, angle_z])
                mirror([1, 0, 0])
                utu_frame_tube_angle_iron();
            }
        }
    }

    // Original 4 wall panels (same as standard lifetrac)
    _place_angles(left_inner_face, true);
    _place_angles(left_outer_face + angle_thick, true);
    _place_angles(right_inner_face, false);
    _place_angles(right_outer_face - angle_thick, false);

    // New bearing plates (angle irons on inner face of new plates)
    // Left bearing plate: inner face at -UTU_BEARING_PLATE_X
    _place_angles(left_bp_face + angle_thick, true);
    // Right bearing plate: inner face at +UTU_BEARING_PLATE_X
    _place_angles(right_bp_face - angle_thick, false);

    // New inner idler bearing plate: angle iron on inboard face,
    // leg pointing toward center (matches inner-wall-panel pattern)
    _place_angles(left_ibp_inboard, true);
    _place_angles(right_ibp_inboard, false);
}

// ============================================================
// MODULES - UTU Sprocket
// ============================================================

// Chain sprocket with filleted tooth roots (from utu_v25.scad)
module utu_sprocket(phase = 0) {
    fillet_r = utu_link_rod_diam / 2;

    color("Gold")
    rotate([0, 0, phase])
    translate([0, 0, -utu_sprocket_thickness / 2])
    linear_extrude(height = utu_sprocket_thickness) {
        difference() {
            offset(r = fillet_r)
            offset(r = -fillet_r)
            difference() {
                circle(r = utu_sprocket_tip_r);
                for (i = [0:utu_num_teeth - 1])
                    rotate([0, 0, i * 360 / utu_num_teeth])
                    translate([utu_sprocket_pitch_radius, 0])
                    circle(r = utu_rod_seat_r);
            }
            circle(d = utu_shaft_diam);
        }
    }
}

// ============================================================
// MODULES - UTU Bearing
// ============================================================

module utu_bearing() {
    housing_diam = 2.5 * inch;
    flange_h = 0.5 * inch;
    lobe_diam = 1 * inch;
    housing_len = 0.75 * inch;

    color("DarkGreen") {
        difference() {
            union() {
                hull() {
                    for (angle = UWU_BOLT_ANGLES)
                        rotate([0, 0, angle])
                        translate([UWU_BOLT_CIRCLE_DIAM / 2, 0, 0])
                        cylinder(h = flange_h, d = lobe_diam, $fn = 24);
                    cylinder(h = flange_h, d = housing_diam, $fn = 32);
                }
                translate([0, 0, flange_h / 2])
                cylinder(h = housing_len, d = housing_diam, $fn = 32);
            }
            translate([0, 0, -eps])
            cylinder(h = flange_h + housing_len + 2 * eps, d = utu_shaft_diam, $fn = 32);
            for (angle = UWU_BOLT_ANGLES)
                rotate([0, 0, angle])
                translate([UWU_BOLT_CIRCLE_DIAM / 2, 0, -eps])
                cylinder(h = flange_h + 2 * eps, d = UWU_BOLT_HOLE_DIAM, $fn = 16);
        }
    }
}

// ============================================================
// MODULES - UTU Hydraulic Motor
// ============================================================

module utu_hydraulic_motor() {
    motor_shaft_diam = 1 * inch;
    motor_shaft_len = 1.5 * inch;
    motor_flange_size = 7.5 * inch;
    motor_flange_t = 0.5 * inch;

    color("Orange") {
        // Body
        translate([0, 0, -utu_motor_body_len])
        cylinder(h = utu_motor_body_len, d = utu_motor_body_diam, $fn = 48);

        // Flange
        difference() {
            translate([0, 0, -motor_flange_t / 2])
            cube([motor_flange_size, motor_flange_size, motor_flange_t], center = true);

            for (angle = UWU_BOLT_ANGLES)
                rotate([0, 0, angle])
                translate([UWU_MOTOR_BCD / 2, 0, -motor_flange_t / 2])
                cylinder(h = motor_flange_t + 2 * eps, d = UWU_BOLT_HOLE_DIAM, center = true, $fn = 16);
        }

        // Shaft
        cylinder(h = motor_shaft_len, d = motor_shaft_diam, $fn = 32);
    }
}

// ============================================================
// MODULES - UTU Track Link & Chain
// ============================================================

// Single track link
module utu_track_link(is_inner = true) {
    inner_face_gap = 2 * inch;
    stagger_gap = 0.125 * inch;
    connector_thickness = 0.25 * inch;
    inner_offset = inner_face_gap / 2 + connector_thickness / 2;
    outer_offset = inner_offset + connector_thickness / 2 + stagger_gap + connector_thickness / 2;
    offset_val = is_inner ? inner_offset : outer_offset;
    rod_hole_clearance = 1;
    rod_hole_x_off = utu_chain_pitch / 2;

    // Ground contact bar
    color("DarkGray")
    translate([-utu_track_bar_width / 2, -utu_track_bar_length / 2, 0])
    cube([utu_track_bar_width, utu_track_bar_length, utu_track_bar_thickness]);

    // Connector plates
    color("Gray")
    for (side = [-1, 1]) {
        translate([0, side * offset_val, utu_track_bar_thickness]) {
            difference() {
                union() {
                    translate([-utu_chain_pitch / 2, -connector_thickness / 2, 0])
                    cube([utu_chain_pitch, connector_thickness, utu_connector_height]);
                    for (dx = [-rod_hole_x_off, rod_hole_x_off])
                        translate([dx, 0, utu_connector_height / 2])
                        rotate([90, 0, 0])
                        cylinder(h = connector_thickness, d = utu_connector_height, center = true, $fn = 24);
                }
                for (dx = [-rod_hole_x_off, rod_hole_x_off])
                    translate([dx, 0, utu_connector_height / 2])
                    rotate([90, 0, 0])
                    cylinder(h = connector_thickness + 2 * eps, d = utu_link_rod_diam + rod_hole_clearance, center = true, $fn = 16);
            }
        }
    }
}

// Connecting rod
module utu_connecting_rod() {
    inner_face_gap = 2 * inch;
    stagger_gap = 0.125 * inch;
    connector_thickness = 0.25 * inch;
    inner_offset = inner_face_gap / 2 + connector_thickness / 2;
    outer_offset = inner_offset + connector_thickness / 2 + stagger_gap + connector_thickness / 2;
    rod_length = 2 * outer_offset + 2 * connector_thickness;

    color("DimGray")
    rotate([90, 0, 0])
    translate([0, 0, -rod_length / 2])
    cylinder(h = rod_length, d = utu_link_rod_diam, $fn = 24);
}

// ============================================================
// MODULES - UTU Chain Path Computation
// ============================================================

// Compute chain path for a single UTU side and place links/rods along the
// FULL 3-sprocket loop: bottom run (rear -> front), arc around front idler,
// top-front diagonal (front -> drive), arc around drive, top-rear diagonal
// (drive -> rear), arc around rear idler, back to start. All three
// sprockets share the same pitch radius, so the loop consists of three
// straight tangent segments (the sides of the triangle of axis centers)
// and three arcs that together cover 360°.
//
// Arguments are in local XZ coordinates (X = fore-aft, Z = vertical):
//   left_x       : rear idler X (more negative / smaller X)
//   right_x      : front idler X (more positive / larger X)
//   drive_x_local: drive axis X (between left_x and right_x)
//   drive_z_rel  : drive Z above the idler line
//   idler_z      : idler Z (both idlers share this)
module utu_track_chain(left_x, right_x, drive_x_local, drive_z_rel, idler_z) {
    R = utu_sprocket_pitch_radius;

    // Sprocket centers (local XZ). Walking the loop counter-clockwise
    // starting at the rear idler (A), to the front idler (B) along the
    // bottom run, then up to drive (C), then back to A.
    A = [left_x,        idler_z];
    B = [right_x,       idler_z];
    C = [drive_x_local, idler_z + drive_z_rel];

    // Outward-tangent unit vectors for each segment between equal-radius
    // sprockets: the chain runs along an external common tangent. For a
    // tangent traveling from P -> Q on the LEFT (counter-clockwise) side,
    // the tangent point on each circle is offset by R perpendicular to
    // the P->Q direction, rotated 90° clockwise (i.e., to the right of
    // the travel direction when viewed in standard XZ where Z is up and
    // the chain wraps the sprockets going CCW around the assembly).
    function _perp_cw(v) = [v[1], -v[0]];                  // 90° CW
    function _unit(v)    = v / norm(v);
    function _tan_offset(P, Q) = R * _perp_cw(_unit(Q - P));

    // Bottom run: A -> B. Wheels lie on top of chain, so the chain on the
    // bottom run sits BELOW the centers (negative Z offset). For travel
    // +X (left to right), 90° CW perp is (0,-1). Good.
    A_dep = A + _tan_offset(A, B);
    B_arr = B + _tan_offset(A, B);

    // Front-top diagonal: B -> C (going up and to the left).
    B_dep = B + _tan_offset(B, C);
    C_arr = C + _tan_offset(B, C);

    // Rear-top diagonal: C -> A (going down and to the left).
    C_dep = C + _tan_offset(C, A);
    A_arr = A + _tan_offset(C, A);

    // Tangent angles (measured from each sprocket center)
    function _ang(center, pt) = atan2(pt[1] - center[1], pt[0] - center[0]);
    a_A_arr = _ang(A, A_arr);
    a_A_dep = _ang(A, A_dep);
    a_B_arr = _ang(B, B_arr);
    a_B_dep = _ang(B, B_dep);
    a_C_arr = _ang(C, C_arr);
    a_C_dep = _ang(C, C_dep);

    // For a CCW loop, the wrap from arr -> dep on each sprocket is
    // measured CCW. Normalize each to [0, 360).
    function _ccw(from_a, to_a) =
        let(d = ((to_a - from_a) % 360 + 360) % 360) d;

    wrap_A = _ccw(a_A_arr, a_A_dep);
    wrap_B = _ccw(a_B_arr, a_B_dep);
    wrap_C = _ccw(a_C_arr, a_C_dep);

    seg_AB    = norm(B_arr - A_dep);
    seg_BC    = norm(C_arr - B_dep);
    seg_CA    = norm(A_arr - C_dep);
    arc_A     = R * wrap_A * PI / 180;
    arc_B     = R * wrap_B * PI / 180;
    arc_C     = R * wrap_C * PI / 180;
    total     = seg_AB + arc_B + seg_BC + arc_C + seg_CA + arc_A;
    num_links = round(total / utu_chain_pitch);

    // Animation: shift links along the loop by fractional pitch
    anim_offset = animate_track ? ($t * utu_chain_pitch) : 0;

    // Helper: given an arclength `s` measured along the loop starting at
    // A_dep, return [pos, dir_angle_deg] in local XZ. The loop order is:
    //   AB (straight), B arc (arr -> dep), BC (straight), C arc, CA, A arc.
    starts = [
        0,
        seg_AB,
        seg_AB + arc_B,
        seg_AB + arc_B + seg_BC,
        seg_AB + arc_B + seg_BC + arc_C,
        seg_AB + arc_B + seg_BC + arc_C + seg_CA
    ];

    function _wrap_s(s_raw) = ((s_raw % total) + total) % total;

    function _pose(s_raw) =
        let(s = _wrap_s(s_raw))
            (s < starts[1]) ? (
                let(t = s - starts[0],
                    dir = _unit(B_arr - A_dep),
                    pos = A_dep + dir * t)
                [pos, atan2(dir[1], dir[0])]
            )
            : (s < starts[2]) ? (
                let(t = s - starts[1],
                    a = a_B_arr + (t / R) * (180 / PI),
                    pos = B + R * [cos(a), sin(a)],
                    dir_a = a + 90)
                [pos, dir_a]
            )
            : (s < starts[3]) ? (
                let(t = s - starts[2],
                    dir = _unit(C_arr - B_dep),
                    pos = B_dep + dir * t)
                [pos, atan2(dir[1], dir[0])]
            )
            : (s < starts[4]) ? (
                let(t = s - starts[3],
                    a = a_C_arr + (t / R) * (180 / PI),
                    pos = C + R * [cos(a), sin(a)],
                    dir_a = a + 90)
                [pos, dir_a]
            )
            : (s < starts[5]) ? (
                let(t = s - starts[4],
                    dir = _unit(A_arr - C_dep),
                    pos = C_dep + dir * t)
                [pos, atan2(dir[1], dir[0])]
            )
            : (
                let(t = s - starts[5],
                    a = a_A_arr + (t / R) * (180 / PI),
                    pos = A + R * [cos(a), sin(a)],
                    dir_a = a + 90)
                [pos, dir_a]
            );

    // Place each link at its CENTER position, and a connecting rod at
    // each link BOUNDARY (which is where the link's connector lugs sit).
    for (i = [0:num_links - 1]) {
        // Link center is between rods at i*pitch and (i+1)*pitch.
        link_pose = _pose(i * utu_chain_pitch + anim_offset + utu_chain_pitch / 2);
        link_pos  = link_pose[0];
        link_ang  = link_pose[1];

        // Place the link at the chain-pin midpoint position. The link's
        // local +X runs along the chain travel direction; its -Z points
        // outward (ground for bottom run, radially outward on arcs).
        translate([link_pos[0], 0, link_pos[1]])
        rotate([0, -link_ang, 0])
        translate([0, 0, -utu_rod_z_above_bar])
        utu_track_link(is_inner = (i % 2 == 0));

        // Connecting rod at the link boundary (where two adjacent links
        // share their connector lugs and the rod passes through them).
        rod_pose = _pose(i * utu_chain_pitch + anim_offset);
        rod_pos  = rod_pose[0];
        translate([rod_pos[0], 0, rod_pos[1]])
        utu_connecting_rod();
    }

    echo(str("UTU track chain: ", num_links, " links, path length ",
             round(total * 100) / 100, "mm,",
             " pitch fit error ", round((total - num_links * utu_chain_pitch) * 100) / 100, "mm"));
}

// ============================================================
// MODULES - UTU Axis Assemblies (positioned in world coords)
// ============================================================

// Single UTU idler axis: shaft + sprocket + 2 bearings
// Oriented with shaft along X axis (cross-machine)
// Bearings mount to outer wall panel and new bearing plate
module utu_idler_axis(y_pos, z_pos, side = "left", sprocket_phase = 0) {
    side_mult = (side == "left") ? -1 : 1;

    // Bearing housing total length (flange 0.5" + housing 0.75"); see utu_bearing()
    bearing_total_len = (0.5 + 0.75) * inch;
    shaft_overhang = 1 * inch;  // stick-out past outer end of each bearing housing

    // Inner bearing inboard tip (toward machine center) X (positive magnitude)
    inner_brg_inboard_x = UTU_INNER_BEARING_PLATE_X - bearing_total_len;
    // Outer bearing outboard tip X (positive magnitude)
    outer_brg_outboard_x = UTU_BEARING_PLATE_X + UTU_OUTER_PLATE_THICKNESS + bearing_total_len;

    // Shaft spans from inboard tip + 1" overhang to outboard tip + 1" overhang
    idler_shaft_len = (outer_brg_outboard_x + shaft_overhang)
                      - (inner_brg_inboard_x - shaft_overhang);
    idler_shaft_center_mag = (inner_brg_inboard_x - shaft_overhang
                              + outer_brg_outboard_x + shaft_overhang) / 2;

    // Shaft along X - extends 1" past the outer end of each bearing housing
    color("Silver")
    translate([side_mult * idler_shaft_center_mag, y_pos, z_pos])
    rotate([0, 90, 0])
    cylinder(h = idler_shaft_len, d = utu_shaft_diam, center = true, $fn = 32);

    // Sprocket on shaft - outside the main walls
    translate([side_mult * UTU_SPROCKET_X, y_pos, z_pos])
    rotate([0, 90, 0])
    utu_sprocket(sprocket_phase);

    // Inner bearing - moved from outer wall panel onto the new inner
    // bearing plate that sits between the wall and the sprocket.
    // Bearing flange seats on plate's outboard face, housing extends inboard.
    translate([side_mult * UTU_INNER_BEARING_PLATE_X, y_pos, z_pos])
    rotate([0, side_mult * -90, 0])
    utu_bearing();

    // Outer bearing on outer triangular bearing plate
    // Mounted on the OUTBOARD face of the plate, housing extending outward.
    translate([side_mult * (UTU_BEARING_PLATE_X + UTU_OUTER_PLATE_THICKNESS), y_pos, z_pos])
    rotate([0, side_mult * 90, 0])
    utu_bearing();
}

// Single UTU drive axis: shaft + sprocket + motor + bearings
module utu_drive_axis(y_pos, z_pos, side = "left", sprocket_phase = 0) {
    side_mult = (side == "left") ? -1 : 1;

    // Bearing housing total length (flange 0.5" + housing 0.75"); see utu_bearing()
    bearing_total_len = (0.5 + 0.75) * inch;
    shaft_overhang = 1 * inch;  // stick-out past outer end of outer bearing housing

    // Shaft along X - spans from motor (inner wall) to 1" past the outer
    // bearing's outboard tip on the outer bearing plate (matches the
    // idler-shaft overhang convention).
    drive_shaft_inner_x  = INNER_WALL_X;
    drive_shaft_outer_x  = UTU_BEARING_PLATE_X + UTU_OUTER_PLATE_THICKNESS
                           + bearing_total_len + shaft_overhang;
    drive_shaft_len      = drive_shaft_outer_x - drive_shaft_inner_x;
    drive_shaft_center   = (drive_shaft_inner_x + drive_shaft_outer_x) / 2;
    color("Silver")
    translate([side_mult * drive_shaft_center, y_pos, z_pos])
    rotate([0, 90, 0])
    cylinder(h = drive_shaft_len, d = utu_shaft_diam, center = true, $fn = 32);

    // Sprocket on shaft - outside the main walls
    translate([side_mult * UTU_SPROCKET_X, y_pos, z_pos])
    rotate([0, 90, 0])
    utu_sprocket(sprocket_phase);

    // Motor mounts to inner wall panel (the inner sandwich plate)
    // Motor body extends toward center, shaft goes outward through wall
    translate([side_mult * INNER_WALL_X, y_pos, z_pos])
    rotate([0, side_mult * 90, 0])
    utu_hydraulic_motor();

    // Bearing on inner wall panel (opposite side from motor)
    translate([side_mult * INNER_WALL_X, y_pos, z_pos])
    rotate([0, side_mult * 90, 0])
    utu_bearing();

    // Bearing on outer wall panel
    translate([side_mult * (OUTER_WALL_X + PANEL_THICKNESS), y_pos, z_pos])
    rotate([0, side_mult * 90, 0])
    utu_bearing();

    // Bearing on new bearing plate
    // Mounted on the OUTBOARD face of the plate, housing extending outward.
    translate([side_mult * (UTU_BEARING_PLATE_X + UTU_OUTER_PLATE_THICKNESS), y_pos, z_pos])
    rotate([0, side_mult * 90, 0])
    utu_bearing();
}

// ============================================================
// MODULES - Complete UTU Side Assembly
// ============================================================

module utu_side_assembly(side = "left") {
    side_mult = (side == "left") ? -1 : 1;

    // Front idler
    utu_idler_axis(UTU_FRONT_IDLER_Y, UTU_IDLER_Z, side, UTU_FRONT_SPROCKET_PHASE);

    // Rear idler
    utu_idler_axis(UTU_REAR_IDLER_Y, UTU_IDLER_Z, side, UTU_REAR_SPROCKET_PHASE);

    // Drive axis (motor at top center)
    utu_drive_axis(UTU_DRIVE_Y, UTU_DRIVE_Z, side, UTU_DRIVE_SPROCKET_PHASE);

    // Outer bearing mount plate (in YZ plane, facing outward in X)
    if (side == "right") {
        translate([UTU_BEARING_PLATE_X, 0, 0])
        rotate([90, 0, 90])
        utu_outer_bearing_plate();
    } else {
        mirror([1, 0, 0])
        translate([UTU_BEARING_PLATE_X, 0, 0])
        rotate([90, 0, 90])
        utu_outer_bearing_plate();
    }

    // Inner idler bearing plate (between outer wall and sprocket)
    // Same orientation pattern as the outer plate. Inboard face of the
    // plate sits at UTU_INNER_BEARING_PLATE_X - UTU_INNER_PLATE_THICKNESS.
    inner_plate_inboard_x = UTU_INNER_BEARING_PLATE_X - UTU_INNER_PLATE_THICKNESS;
    if (side == "right") {
        translate([inner_plate_inboard_x, 0, 0])
        rotate([90, 0, 90])
        utu_inner_idler_bearing_plate();
    } else {
        mirror([1, 0, 0])
        translate([inner_plate_inboard_x, 0, 0])
        rotate([90, 0, 90])
        utu_inner_idler_bearing_plate();
    }

    // Track chain (simplified visualization)
    if (show_track) {
        // Rotate 90° to align track fore-aft (Y axis) instead of cross-machine (X)
        // Place at sprocket X position outside the walls
        translate([side_mult * UTU_SPROCKET_X, 0, 0])
        rotate([0, 0, 90]) {
            utu_track_chain(
                UTU_REAR_IDLER_Y, UTU_FRONT_IDLER_Y,
                UTU_DRIVE_Y,
                utu_drive_height_actual,
                UTU_IDLER_Z
            );
        }
    }
}

// All UTU assemblies (both sides)
module utu_assemblies() {
    if (show_utu) {
        utu_side_assembly("left");
        utu_side_assembly("right");
    }
}

// ============================================================
// MODIFIED BASE FRAME (no motor mounting plates)
// ============================================================

// Re-use existing side panel modules from main assembly
// The inner sandwich plates now serve as UTU motor mount plates
// (they already have UWU bearing holes which are compatible)

// Import the assembly modules we need from the main file
// We selectively call what we need to exclude motor_mounting_plate()

module utu_base_frame() {
    // Side panels (unchanged - inner panels now serve as motor plates)
    side_panel_left_outer();
    side_panel_left_inner();
    side_panel_right_inner();
    side_panel_right_outer();

    // Standard frame components
    cylinder_mounting_lugs();
    arm_pivot_assembly();

    // Stiffener plates (unchanged)
    back_stiffener_plate();
    bottom_stiffener_plate();
    front_stiffener_plate();

    // REMOVED: motor_mounting_plate()
    // The UTU motor plate IS the inner sandwich plate.
    // UWU motor mount plates between the walls are not needed.

    // REPLACED: cross_frame_tubes() with extended UTU version
    utu_cross_frame_tubes();
}

// ============================================================
// MAIN ASSEMBLY
// ============================================================

module lifetrac_v25_utu_assembly() {
    if (show_frame) {
        utu_base_frame();
    }

    // UTU track assemblies (replaces wheel_assemblies and uwu_assemblies)
    utu_assemblies();

    // Loader arms (unchanged)
    if (show_loader_arms) {
        loader_arms();
    }

    // Bucket (unchanged)
    if (show_bucket) {
        bucket_attachment();
    }

    // Hydraulics (unchanged)
    if (show_hydraulics) {
        lift_cylinders();
        bucket_cylinders();
    }

    // Folding platform (unchanged)
    if (show_folding_platform) {
        folding_platform_assembly(platform_fold_angle);
    }
}

// ============================================================
// RENDER
// ============================================================

lifetrac_v25_utu_assembly();

// ============================================================
// DESIGN NOTES
// ============================================================
//
// UTU Variant Changes from UWU (Standard) LifeTrac v25:
//
// 1. EXTENDED 2x6 FRAME TUBES
//    The two cross-frame 2x6 tubes are extended beyond the outer wall
//    panels to reach new triangular bearing mount plates. They pass
//    through the new plates and are bolted using the same angle iron
//    mounting pattern (2"x2"x1/4" angle iron, A4 parts).
//
// 2. MOTOR MOUNTING PLATES REMOVED
//    The separate motor mounting plates that sit between the machine's
//    walls (at MOTOR_PLATE_X) are removed. The UTU's hydraulic motor
//    mounts directly to the inner sandwich plate (inner wall panel)
//    using the same UWU bolt pattern already cut into those panels.
//
// 3. OUTER WALL = BEARING MOUNT
//    The machine's outer sandwich plate (outer wall panel) serves as
//    the bearing mount for the idler and motor axes. The UWU bearing
//    hole pattern is already cut into these panels.
//
// 4. NEW TRIANGULAR BEARING PLATES
//    New 1/2" steel plates mounted at the ends of the extended 2x6
//    tubes. These plates are triangular with rounded corners, covering
//    the area between the 3 UTU axes (2 bottom idlers + 1 top drive).
//    They provide outer bearing support for all 3 axes.
//    The triangle is padded to clear the bearing bolt patterns.
//
// 5. SHARED PARTS
//    - All side panels (inner/outer) unchanged
//    - All stiffener plates unchanged
//    - Loader arms, bucket, hydraulics unchanged
//    - Folding platform unchanged
//    - Same angle iron parts (A4) for frame tube mounting
//    - Same UWU bearing/motor bolt pattern interface
//    - Same 2x6 tube cross-section (just longer)
//
