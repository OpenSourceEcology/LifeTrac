// Test file for cylinder sizing analysis
include <lifetrac_v25_params.scad>

echo("=== KEY PARAMETERS ===");
echo("ARM_MIN_ANGLE =", ARM_MIN_ANGLE);
echo("ARM_MAX_ANGLE =", ARM_MAX_ANGLE);
echo("ARM_PIVOT_Y =", ARM_PIVOT_Y);
echo("ARM_PIVOT_Z =", ARM_PIVOT_Z);
echo("HYD_BRACKET_ARM_POS =", HYD_BRACKET_ARM_POS);
echo("HYD_BRACKET_CIRCLE_DIA =", HYD_BRACKET_CIRCLE_DIA);
echo("TUBE_2X6_1_4 =", TUBE_2X6_1_4);

// Cylinder mounting
LIFT_CYL_ARM_LOCAL_Y = HYD_BRACKET_ARM_POS;
LIFT_CYL_ARM_LOCAL_Z = -(TUBE_2X6_1_4[1]/2 + HYD_BRACKET_CIRCLE_DIA/2);

// Base mounting
FRAME_Z_OFFSET = 150;
WHEEL_DIAMETER = 500;
LIFT_CYL_BASE_Z = FRAME_Z_OFFSET + WHEEL_DIAMETER/2 + 80;

// Calculate base Y
LIFT_CYL_ARM_OFFSET = ARM_LENGTH * 0.25;
target_angle = 50;
arm_z_offset = -(TUBE_2X6_1_4[1]/2 + HYD_BRACKET_CIRCLE_DIA/2);
attach_y = ARM_PIVOT_Y + LIFT_CYL_ARM_OFFSET * cos(ARM_MIN_ANGLE) - arm_z_offset * sin(ARM_MIN_ANGLE);
attach_z = ARM_PIVOT_Z + LIFT_CYL_ARM_OFFSET * sin(ARM_MIN_ANGLE) + arm_z_offset * cos(ARM_MIN_ANGLE);
calc_base = attach_y - (attach_z - LIFT_CYL_BASE_Z) / tan(target_angle);
LIFT_CYL_BASE_Y = max(50, min(WHEEL_BASE/2, calc_base));

echo("=== MOUNTING POSITIONS ===");
echo("LIFT_CYL_BASE_Y =", LIFT_CYL_BASE_Y);
echo("LIFT_CYL_BASE_Z =", LIFT_CYL_BASE_Z);
echo("LIFT_CYL_ARM_LOCAL_Y =", LIFT_CYL_ARM_LOCAL_Y);
echo("LIFT_CYL_ARM_LOCAL_Z =", LIFT_CYL_ARM_LOCAL_Z);

// Calculate cylinder lengths
function lift_cyl_len(arm_angle) = 
    let(
        arm_y = LIFT_CYL_ARM_LOCAL_Y * cos(arm_angle) - LIFT_CYL_ARM_LOCAL_Z * sin(arm_angle),
        arm_z = LIFT_CYL_ARM_LOCAL_Y * sin(arm_angle) + LIFT_CYL_ARM_LOCAL_Z * cos(arm_angle),
        arm_world_y = ARM_PIVOT_Y + arm_y,
        arm_world_z = ARM_PIVOT_Z + arm_z,
        dy = arm_world_y - LIFT_CYL_BASE_Y,
        dz = arm_world_z - LIFT_CYL_BASE_Z
    )
    sqrt(dy*dy + dz*dz);

len_min = lift_cyl_len(ARM_MIN_ANGLE);
len_max = lift_cyl_len(ARM_MAX_ANGLE);
required_stroke = len_max - len_min;

echo("=== CYLINDER LENGTH ANALYSIS ===");
echo("Length at ARM_MIN (closed):", len_min);
echo("Length at ARM_MAX (extended):", len_max);
echo("REQUIRED STROKE:", required_stroke);

// Current cylinder specs
BORE = 63.5;
STROKE = 300;
// From hydraulics.scad module calculation
base_clevis_len = BORE * 0.8;
piston_len = BORE * 0.4;
head_internal = BORE * 0.4;
head_external = BORE * 0.2;
rod_clevis_len = 38.1 * 1.5;  // rod * 1.5
clearance = 5;
tube_len = STROKE + piston_len + head_internal + clearance;
closed_len = base_clevis_len + tube_len + head_external + 5 + rod_clevis_len;
max_len = closed_len + STROKE;

echo("=== CURRENT 300mm STROKE CYLINDER ===");
echo("Stroke:", STROKE);
echo("Calculated closed length:", closed_len);
echo("Calculated max extended length:", max_len);
echo("SHORTFALL at max arm angle:", len_max - max_len);

// Try 350mm stroke
STROKE_350 = 350;
tube_len_350 = STROKE_350 + piston_len + head_internal + clearance;
closed_len_350 = base_clevis_len + tube_len_350 + head_external + 5 + rod_clevis_len;
max_len_350 = closed_len_350 + STROKE_350;
echo("=== WITH 350mm STROKE CYLINDER ===");
echo("Calculated closed length:", closed_len_350);
echo("Calculated max extended length:", max_len_350);
echo("SHORTFALL:", len_max - max_len_350);

// Try 400mm stroke  
STROKE_400 = 400;
tube_len_400 = STROKE_400 + piston_len + head_internal + clearance;
closed_len_400 = base_clevis_len + tube_len_400 + head_external + 5 + rod_clevis_len;
max_len_400 = closed_len_400 + STROKE_400;
echo("=== WITH 400mm STROKE CYLINDER ===");
echo("Calculated closed length:", closed_len_400);
echo("Calculated max extended length:", max_len_400);
echo("SHORTFALL:", len_max - max_len_400);
echo("MARGIN (negative = good):", len_max - max_len_400);
