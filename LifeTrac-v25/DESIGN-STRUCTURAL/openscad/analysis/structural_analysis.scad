// structural_analysis.scad
// Static structural and stability checks for the LifeTrac v25 loader.
//
// lifetrac_v25.scad includes this file at its end, after every parameter it reads is
// defined, so the checks use the same geometry the model draws. For the same reason the
// file can't be opened on its own. It replaces the earlier analysis block, which checked
// a straight 3"x3" arm and applied its safety factor twice (finding B4 of
// AI NOTES/CODE REVIEWS/2026-09-25_v25_OpenSCAD_Full_Review_Claude_v1_0.md).
// STRUCTURAL_ANALYSIS.md describes the method.
//
// LOAD CASES (static, at the hydraulic relief pressure, no dynamic factor)
//   LIFT    Both lift cylinders push at relief. The load sits at the bucket's load centre,
//           with the bucket level, and the case is repeated at arm angles across the
//           working range. The arm hangs on its pivot pin and the lift cylinder props it.
//   BUCKET  Both bucket cylinders push at relief with the dump stroke resisted (prying, or
//           pushing down with the bucket). Each bucket pivot pin then carries at least the
//           cylinder's push, and the cross beam T3 carries both cylinders.
//
// CRITERIA (AISC 360, allowable stress design)
//   Bending 0.6 Fy, pin shear 0.4 Fy of the pin steel, fillet welds 0.3 F_EXX, bolts and
//   pin holes Rn / 2.
//   A pin hole's strength is the lower of tear-out (1.2 lc t Fu) and bearing (2.4 d t Fu),
//   with lc measured to the nearest edge whatever the load direction.
//
// RATED OPERATING CAPACITY
//   50 % of the tipping load (the usual rating for skid-steer loaders, ISO 14397-1), using
//   the lowest tipping load over the arm's working range, and no more than the hydraulic
//   lift capacity. The masses are estimates until parts get masses from their geometry.
//
// KNOWN ISSUES
//   A failing check that needs a design decision is listed in STRUCT_KNOWN_ISSUES with the
//   review finding that covers it. The structural-analysis CI job fails on any other
//   failing check, and warns when a listed check starts to pass.

// =============================================================================
// MATERIALS AND CRITERIA
// =============================================================================

AN_FY = 250;           // Plate, tube and angle: ASTM A36 yield (MPa)
AN_FU = 400;           // ASTM A36 minimum tensile strength (MPa)
AN_PIN_FY = 530;       // Pins: AISI 1045 cold drawn (assumed until the pins are specified)
AN_BOLT_FU = 827;      // Bolts: SAE J429 Grade 5, 120 ksi (MPa)
AN_WELD_FEXX = 482;    // E70 electrode (MPa)
AN_OMEGA = 2.0;        // AISC ASD safety factor for bolts, pin holes and their failure modes

AN_ALLOW_BENDING = 0.6 * AN_FY;
AN_ALLOW_PIN_SHEAR = 0.4 * AN_PIN_FY;
AN_ALLOW_WELD = 0.3 * AN_WELD_FEXX;   // AISC fillet-weld allowable (already includes Omega)

// u_channel_lug() in lifetrac_v25.scad holds every lug with four 1/4-20 UNC bolts
AN_LUG_BOLT_COUNT = 4;
AN_LUG_BOLT_DIA = 6.35;
AN_LUG_BOLT_AT = 20.5;   // Tensile stress area (mm^2)

// HYDRAULIC_PRESSURE_PSI (lifetrac_v25.scad) matches the 3,000 psi relief valve in
// DESIGN-HYDRAULIC/HYDRAULIC_BOM.md
AN_RELIEF_MPA = HYDRAULIC_PRESSURE_PSI * 0.00689476;
AN_G = 9.81;

// =============================================================================
// HYDRAULIC FORCES AT RELIEF (per cylinder, N)
// =============================================================================

function an_area(d) = PI * d * d / 4;

AN_LIFT_PUSH_N = AN_RELIEF_MPA * an_area(LIFT_CYLINDER_BORE);
AN_BUCKET_PUSH_N = AN_RELIEF_MPA * an_area(BUCKET_CYLINDER_BORE);
AN_BUCKET_PULL_N = AN_RELIEF_MPA * (an_area(BUCKET_CYLINDER_BORE) - an_area(BUCKET_CYLINDER_ROD));

// =============================================================================
// SIDE-VIEW GEOMETRY (Y forward, Z up, mm)
// =============================================================================
// Arm-local points have the arm pivot at the origin and +Y along the main tube; they
// rotate with the arm angle.

function an_rot(p, a) = [p[0] * cos(a) - p[1] * sin(a), p[0] * sin(a) + p[1] * cos(a)];

AN_PIVOT = [ARM_PIVOT_Y, ARM_PIVOT_Z];
AN_LIFT_BASE = [LIFT_CYL_BASE_Y, LIFT_CYL_BASE_Z];
AN_LIFT_ATTACH = [HYD_BRACKET_ARM_POS, -(TUBE_2X6_1_4[1] / 2 + HYD_BRACKET_CIRCLE_DIA / 2)];
AN_BUCKET_PIN = [ARM_TIP_X + BUCKET_PIVOT_Y_OFFSET, ARM_TIP_Z];

// Load centre relative to the bucket pin, with the bucket level: half the bucket depth
// ahead of its back plate, and 40 % of the bucket height above its floor
AN_LOAD_CENTRE = [BUCKET_LUG_OFFSET + BUCKET_DEPTH / 2,
                  0.4 * BUCKET_HEIGHT - BUCKET_PIVOT_HEIGHT_FROM_BOTTOM];

function an_world(p, a) = AN_PIVOT + an_rot(p, a);
function an_load_point(a) = an_world(AN_BUCKET_PIN, a) + AN_LOAD_CENTRE;
function an_load_lever(a) = an_load_point(a)[0] - AN_PIVOT[0];

// Unit vector along the lift cylinder, from its base toward the arm
function an_lift_dir(a) = let(d = an_world(AN_LIFT_ATTACH, a) - AN_LIFT_BASE) d / norm(d);

// Perpendicular distance from the arm pivot to the lift cylinder's line
function an_lift_lever(a) =
    let(r = an_rot(AN_LIFT_ATTACH, a), u = an_lift_dir(a))
    abs(r[0] * u[1] - r[1] * u[0]);

// Arm angles checked: the working range from arms down to the bucket-cylinder limit
AN_STEPS = 8;
AN_ANGLES = [for (i = [0:AN_STEPS])
    ARM_MIN_ANGLE + i * (ARM_MAX_ANGLE_LIMITED - ARM_MIN_ANGLE) / AN_STEPS];

// =============================================================================
// LIFT CASE: ONE ARM WITH ITS LIFT CYLINDER AT RELIEF
// =============================================================================

// Load at the load centre that one lift cylinder holds (N)
function an_tip_load(a) = AN_LIFT_PUSH_N * an_lift_lever(a) / an_load_lever(a);

// Force the arm puts on its pivot pin: the cylinder's push plus the load (N)
function an_pin_force(a) = AN_LIFT_PUSH_N * an_lift_dir(a) + [0, -an_tip_load(a)];

// Bending moment in the arm just outboard of the lift-bracket gusset, which runs
// 3 x the tube width along the arm (parts/arm_plate.scad). The pin carries no moment,
// so the arm's largest moment is here, not at the pivot (N mm).
AN_GUSSET_HALF = 1.5 * TUBE_2X6_1_4[0];
function an_arm_moment(a) =
    let(section = an_world([HYD_BRACKET_ARM_POS + AN_GUSSET_HALF, 0], a))
    an_tip_load(a) * (an_load_point(a)[0] - section[0]);

AN_PIN_FORCE_MAX = max([for (a = AN_ANGLES) norm(an_pin_force(a))]);
AN_ARM_MOMENT_MAX = max([for (a = AN_ANGLES) an_arm_moment(a)]);

// =============================================================================
// STABILITY AND RATED OPERATING CAPACITY
// =============================================================================

// Mass estimates (kg), carried over from the earlier stability check
AN_MASS_FRAME = 350;     // Chassis, side panels and crossmembers
AN_MASS_WHEEL = 40;      // Tyre, rim and motor, each of 4
AN_MASS_ARMS = 180;      // Both arms, the cross beam and the cylinders
AN_MASS_BUCKET = 120;    // Bucket
AN_MASS_FLUIDS = 40;     // Hydraulic oil and fuel
AN_MASS_OPERATOR = 90;   // Only when someone rides the rear platform

AN_FRONT_AXLE_Y = _FRONT_WHEEL_AXIS_Y;
AN_REAR_AXLE_Y = _REAR_WHEEL_AXIS_Y;
AN_OPERATOR_Y = -200;                    // Standing on the rear platform
AN_FLUIDS_Y = MACHINE_LENGTH * 0.3;      // Tank toward the rear

// The arms' centre of mass is taken 40 % of the way from the pivot to the bucket pin,
// and the bucket's at the load centre
function an_arms_y(a) = AN_PIVOT[0] + 0.4 * an_rot(AN_BUCKET_PIN, a)[0];

AN_MASS_EMPTY = AN_MASS_FRAME + 4 * AN_MASS_WHEEL + ENGINE_WEIGHT_KG + AN_MASS_FLUIDS
                + AN_MASS_ARMS + AN_MASS_BUCKET;

// Centre of gravity of the empty machine along Y, without an operator (drawn by
// calculate_cog() in lifetrac_v25.scad)
function an_cog_y(a) =
    (AN_MASS_FRAME * WHEEL_BASE / 2
     + 2 * AN_MASS_WHEEL * (AN_FRONT_AXLE_Y + AN_REAR_AXLE_Y)
     + ENGINE_WEIGHT_KG * ENGINE_POS_Y
     + AN_MASS_FLUIDS * AN_FLUIDS_Y
     + AN_MASS_ARMS * an_arms_y(a)
     + AN_MASS_BUCKET * an_load_point(a)[0]) / AN_MASS_EMPTY;

// Moment of the machine's own weight about the front axle; positive holds the rear down
// (kg mm)
function an_counter_moment(a, with_operator) =
      AN_MASS_FRAME * (AN_FRONT_AXLE_Y - WHEEL_BASE / 2)
    + 2 * AN_MASS_WHEEL * (AN_FRONT_AXLE_Y - AN_REAR_AXLE_Y)
    + ENGINE_WEIGHT_KG * (AN_FRONT_AXLE_Y - ENGINE_POS_Y)
    + AN_MASS_FLUIDS * (AN_FRONT_AXLE_Y - AN_FLUIDS_Y)
    + AN_MASS_ARMS * (AN_FRONT_AXLE_Y - an_arms_y(a))
    + AN_MASS_BUCKET * (AN_FRONT_AXLE_Y - an_load_point(a)[0])
    + (with_operator ? AN_MASS_OPERATOR * (AN_FRONT_AXLE_Y - AN_OPERATOR_Y) : 0);

// Load at the load centre that lifts the rear wheels off the ground (kg)
function an_tipping_load(a, with_operator = false) =
    let(d = an_load_point(a)[0] - AN_FRONT_AXLE_Y)
    d > 0 ? an_counter_moment(a, with_operator) / d : 1e9;

// Load at the load centre that both lift cylinders hold at relief, after carrying the
// arms and the bucket (kg)
function an_hydraulic_capacity(a) =
    (2 * AN_LIFT_PUSH_N * an_lift_lever(a)
     - AN_G * (AN_MASS_ARMS * (an_arms_y(a) - AN_PIVOT[0])
               + AN_MASS_BUCKET * an_load_lever(a)))
    / (AN_G * an_load_lever(a));

AN_TIPPING_MIN = min([for (a = AN_ANGLES) an_tipping_load(a)]);
AN_TIPPING_MIN_OPERATOR = min([for (a = AN_ANGLES) an_tipping_load(a, true)]);
AN_HYDRAULIC_MIN = min([for (a = AN_ANGLES) an_hydraulic_capacity(a)]);
AN_RATED_CAPACITY_KG = min(0.5 * AN_TIPPING_MIN, AN_HYDRAULIC_MIN);

// =============================================================================
// CHECKS
// =============================================================================

// Section modulus of a rectangular tube bent about the axis parallel to b, with depth h
function an_box_s(b, h, t) = (b * pow(h, 3) - (b - 2 * t) * pow(h - 2 * t, 3)) / 12 / (h / 2);

// Strength of a pin hole through n plates (N): the lower of tear-out and bearing
// (AISC J3.10), with lc the clear distance from the hole to the nearest edge
function an_hole_rn(n, lc, t, d) = n * min(1.2 * lc * t * AN_FU, 2.4 * d * t * AN_FU);

AN_S_ARM_TUBE = an_box_s(TUBE_2X6_1_4[0], TUBE_2X6_1_4[1], TUBE_2X6_1_4[2]);
AN_S_ARM_PLATES = 2 * ARM_PLATE_THICKNESS * pow(TUBE_2X6_1_4[1], 2) / 6;

// Side panels: the pivot hole's nearest edge is the top edge or the boss (finding P3)
AN_PIVOT_HOLE_LC = min(SIDE_PANEL_PIVOT_BOSS_R, MACHINE_HEIGHT - PIVOT_PANEL_Y)
                   - (PIVOT_PIN_DIA + 2) / 2;

// Pivot mount: 1/2" bolts on a 4.5" circle in a 6" plate, one shear plane each
AN_PIVOT_BOLT_LC = (PIVOT_MOUNT_PLATE_DIA - PIVOT_MOUNT_BOLT_CIRCLE_DIA) / 2
                   - PIVOT_MOUNT_BOLT_DIA / 2;
AN_PIVOT_BOLT_RN = min(0.45 * AN_BOLT_FU * an_area(PIVOT_MOUNT_BOLT_DIA),
                       an_hole_rn(1, AN_PIVOT_BOLT_LC, PIVOT_MOUNT_PLATE_THICK, PIVOT_MOUNT_BOLT_DIA));

// T3 cross beam: each bucket cylinder sits BUCKET_CYL_X_SPACING from the centre, and the
// beam ends at the arm tubes. Its lug pins hang below the beam's axis, so the push also
// twists the beam; closed-section shear stress is T / (2 A_m t).
AN_T3_A = (ARM_SPACING - TUBE_2X6_1_4[0]) / 2 - BUCKET_CYL_X_SPACING;
AN_T3_SIGMA = AN_BUCKET_PUSH_N * AN_T3_A
              / an_box_s(TUBE_2X6_1_4[0], TUBE_2X6_1_4[1], TUBE_2X6_1_4[2]);
AN_T3_TAU = AN_BUCKET_PUSH_N * abs(CROSS_BEAM_MOUNT_Z_OFFSET)
            / (2 * (TUBE_2X6_1_4[0] - TUBE_2X6_1_4[2]) * (TUBE_2X6_1_4[1] - TUBE_2X6_1_4[2])
               * TUBE_2X6_1_4[2]);

// Bucket pivot lug: a U-channel cut from 3x3x1/4 tube, open toward the arm, with the pin
// hole in the middle (u_channel_lug() in lifetrac_v25.scad)
AN_LUG_LC = TUBE_3X3_1_4[0] / 2 - 1.5 * TUBE_3X3_1_4[1] - (BUCKET_PIVOT_PIN_DIA + 2) / 2;

// [id, description, demand, capacity, unit]; forces are in N and reported in kN
AN_CHECKS = [
    ["PIVOT_PIN_SHEAR", "Arm pivot pin, double shear",
        AN_PIN_FORCE_MAX / (2 * an_area(PIVOT_PIN_DIA)), AN_ALLOW_PIN_SHEAR, "MPa"],
    ["PIVOT_HOLE_SIDE_PANELS", "Arm pivot hole in the two side panels",
        AN_PIN_FORCE_MAX,
        an_hole_rn(2, AN_PIVOT_HOLE_LC, PANEL_THICKNESS, PIVOT_PIN_DIA) / AN_OMEGA, "kN"],
    ["PIVOT_MOUNT_BOLTS", "Pivot-mount bolts, 1/2in, per bolt",
        AN_PIN_FORCE_MAX / (2 * PIVOT_MOUNT_BOLT_COUNT), AN_PIVOT_BOLT_RN / AN_OMEGA, "kN"],
    ["PIVOT_MOUNT_WELDS", "Pivot-mount DOM-to-plate fillet welds",
        AN_PIN_FORCE_MAX / 2 / (PI * DOM_PIPE_OD * 0.707 * PIVOT_MOUNT_WELD_DIA),
        AN_ALLOW_WELD, "MPa"],
    ["ARM_BENDING", "Arm at the lift bracket, tube and side plates together",
        AN_ARM_MOMENT_MAX / (AN_S_ARM_TUBE + AN_S_ARM_PLATES), AN_ALLOW_BENDING, "MPa"],
    ["ARM_BENDING_PLATES_ONLY", "Arm at the lift bracket, side plates alone",
        AN_ARM_MOMENT_MAX / AN_S_ARM_PLATES, AN_ALLOW_BENDING, "MPa"],
    ["LIFT_CYL_PINS", "Lift-cylinder pins, double shear",
        AN_LIFT_PUSH_N / (2 * an_area(min(HYD_BRACKET_BOLT_DIA, BOLT_DIA_1))),
        AN_ALLOW_PIN_SHEAR, "MPa"],
    ["LIFT_BRACKET_HOLE", "Lift-cylinder hole in the two arm plates",
        AN_LIFT_PUSH_N,
        an_hole_rn(2, HYD_BRACKET_CIRCLE_DIA / 2 - (HYD_BRACKET_BOLT_DIA + 2) / 2,
                   ARM_PLATE_THICKNESS, HYD_BRACKET_BOLT_DIA) / AN_OMEGA, "kN"],
    ["LIFT_BASE_HOLE", "Lift-cylinder base hole in the two side panels",
        AN_LIFT_PUSH_N,
        an_hole_rn(2, LIFT_CYL_BASE_Y - (BOLT_DIA_1 + 2) / 2, PANEL_THICKNESS, BOLT_DIA_1)
            / AN_OMEGA, "kN"],
    ["T3_COMBINED", "Cross beam T3, bending and twist (von Mises)",
        sqrt(pow(AN_T3_SIGMA, 2) + 3 * pow(AN_T3_TAU, 2)), AN_ALLOW_BENDING, "MPa"],
    ["BUCKET_CYL_PINS", "Bucket-cylinder pins, 3/4in, double shear",
        AN_BUCKET_PUSH_N / (2 * an_area(BOLT_DIA_3_4)), AN_ALLOW_PIN_SHEAR, "MPa"],
    ["BUCKET_CYL_LUG_BOLTS", "Bucket-cylinder lug bolts on T3, 4 x 1/4in, shear",
        AN_BUCKET_PUSH_N,
        AN_LUG_BOLT_COUNT * 0.45 * AN_BOLT_FU * an_area(AN_LUG_BOLT_DIA) / AN_OMEGA, "kN"],
    ["BUCKET_CYL_LUG_BOLTS_BUCKET", "Bucket-cylinder lug bolts on the bucket, 4 x 1/4in, tension",
        AN_BUCKET_PULL_N,
        AN_LUG_BOLT_COUNT * AN_LUG_BOLT_AT * AN_BOLT_FU / AN_OMEGA, "kN"],
    ["BUCKET_PIN_SHEAR", "Bucket pivot pin, 1in, double shear",
        AN_BUCKET_PUSH_N / (2 * an_area(BUCKET_PIVOT_PIN_DIA)), AN_ALLOW_PIN_SHEAR, "MPa"],
    ["BUCKET_PIN_ARM_TIP", "Bucket pivot hole in the two arm-tip plates",
        AN_BUCKET_PUSH_N,
        an_hole_rn(2, PIVOT_HOLE_X_FROM_FRONT - BUCKET_PIVOT_PIN_DIA / 2,
                   ARM_PLATE_THICKNESS, BUCKET_PIVOT_PIN_DIA) / AN_OMEGA, "kN"],
    ["BUCKET_PIVOT_LUG", "Bucket pivot hole in the U-lug's two walls",
        AN_BUCKET_PUSH_N,
        an_hole_rn(2, AN_LUG_LC, TUBE_3X3_1_4[1], BUCKET_PIVOT_PIN_DIA) / AN_OMEGA, "kN"],
    ["BUCKET_LUG_BOLTS", "Bucket pivot lug bolts, 4 x 1/4in, tension",
        AN_BUCKET_PUSH_N,
        AN_LUG_BOLT_COUNT * AN_LUG_BOLT_AT * AN_BOLT_FU / AN_OMEGA, "kN"],
];

// Failing checks that need a design decision, with the review finding that covers each
STRUCT_KNOWN_ISSUES = [
    ["ARM_BENDING_PLATES_ONLY", "B4: the tube is bolted to the side plates only near its ends"],
    ["T3_COMBINED", "B4: the bucket-cylinder lugs hang below T3 and twist it"],
    ["BUCKET_CYL_LUG_BOLTS", "P14, M8"],
    ["BUCKET_CYL_LUG_BOLTS_BUCKET", "P14, M8"],
    ["BUCKET_PIN_ARM_TIP", "P14"],
    ["BUCKET_PIVOT_LUG", "P14"],
    ["BUCKET_LUG_BOLTS", "P14, M8"],
];

function an_known(id) = [for (k = STRUCT_KNOWN_ISSUES) if (k[0] == id) k[1]];
function an_ratio(c) = c[2] / c[3];
function an_status(c) =
    let(known = an_known(c[0]))
    an_ratio(c) <= 1
        ? (len(known) > 0 ? "PASS (remove from STRUCT_KNOWN_ISSUES)" : "PASS")
        : (len(known) > 0 ? str("known fail (", known[0], ")") : "NEW FAIL");
function an_fmt(x, digits) = str(round(x * pow(10, digits)) / pow(10, digits));
// Two decimals, keeping trailing zeros (0.30, not 0.3), for x >= 0
function an_fmt2(x) = let(r = round(x * 100), f = r % 100) str(floor(r / 100), ".", f < 10 ? "0" : "", f);
function an_value(c, v) = c[4] == "kN" ? an_fmt(v / 1000, 1) : an_fmt(v, 0);

AN_NEW_FAILS = [for (c = AN_CHECKS) if (an_status(c) == "NEW FAIL") c[0]];
AN_KNOWN_FAILS = [for (c = AN_CHECKS) if (an_ratio(c) > 1 && len(an_known(c[0])) > 0) c[0]];

// =============================================================================
// REPORT
// =============================================================================
// Both blocks are Markdown. The CI job copies the lines from "STRUCTURAL ANALYSIS SUMMARY"
// to "OVERALL:" into the pull-request comment, and both blocks into
// STRUCTURAL_ANALYSIS_LOG.md.

echo("STRUCTURAL ANALYSIS DETAIL");
echo("Lift case, per arm, with both lift cylinders at relief and the load at the bucket's load centre:");
echo("");
echo("| Arm angle | Cylinder lever | Load lever | Load held | Pivot-pin force | Arm moment at bracket | Tipping load | Hydraulic capacity |");
echo("|---|---|---|---|---|---|---|---|");
for (a = AN_ANGLES)
    echo(str("| ", an_fmt(a, 1), " deg | ", an_fmt(an_lift_lever(a), 0), " mm | ",
             an_fmt(an_load_lever(a), 0), " mm | ", an_fmt(an_tip_load(a) / 1000, 1), " kN | ",
             an_fmt(norm(an_pin_force(a)) / 1000, 1), " kN | ",
             an_fmt(an_arm_moment(a) / 1e6, 1), " kN m | ",
             an_fmt(an_tipping_load(a), 0), " kg | ", an_fmt(an_hydraulic_capacity(a), 0), " kg |"));

echo("STRUCTURAL ANALYSIS SUMMARY");
echo(str("Static checks at the ", HYDRAULIC_PRESSURE_PSI, " psi relief pressure: each lift cylinder pushes ",
         an_fmt(AN_LIFT_PUSH_N / 1000, 1), " kN and each bucket cylinder ",
         an_fmt(AN_BUCKET_PUSH_N / 1000, 1), " kN. Arm angles ", an_fmt(ARM_MIN_ANGLE, 1),
         " to ", an_fmt(ARM_MAX_ANGLE_LIMITED, 1), " deg. Method and assumptions: ",
         "`openscad/analysis/structural_analysis.scad`."));
echo("");
echo(str("**Rated operating capacity: ", an_fmt(AN_RATED_CAPACITY_KG, 0), " kg (",
         an_fmt(AN_RATED_CAPACITY_KG * 2.20462, 0), " lb)**, half the lowest tipping load of ",
         an_fmt(AN_TIPPING_MIN, 0), " kg (", an_fmt(AN_TIPPING_MIN_OPERATOR, 0),
         " kg with an operator on the platform). The lowest hydraulic lift capacity is ",
         an_fmt(AN_HYDRAULIC_MIN, 0), " kg. The masses are estimates (",
         an_fmt(AN_MASS_EMPTY, 0), " kg empty)."));
echo("");
echo("| Check | Demand | Capacity | Ratio | Status |");
echo("|---|---|---|---|---|");
for (c = AN_CHECKS)
    echo(str("| ", c[1], " (`", c[0], "`) | ", an_value(c, c[2]), " ", c[4], " | ",
             an_value(c, c[3]), " ", c[4], " | ", an_fmt2(an_ratio(c)), " | ",
             an_status(c), " |"));
echo("");
echo(str("OVERALL: ", len(AN_NEW_FAILS) == 0 ? "no new failures" : "NEW FAIL",
         "; ", len(AN_KNOWN_FAILS), " known issues"));
