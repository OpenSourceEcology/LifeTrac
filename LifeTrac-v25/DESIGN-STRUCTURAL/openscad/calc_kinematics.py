import math

# Constants
MACHINE_HEIGHT = 1000
GROUND_CLEARANCE = 150
FRAME_Z_OFFSET = GROUND_CLEARANCE
WHEEL_BASE = 1400
BUCKET_FRONT_CLEARANCE = 0
WHEEL_DIAMETER = 500

# Arm Pivot
ARM_PIVOT_Y = 200
ARM_PIVOT_Z_HEIGHT = MACHINE_HEIGHT - 50
ARM_PIVOT_Z = FRAME_Z_OFFSET + ARM_PIVOT_Z_HEIGHT

# Arm Length
MIN_HORIZONTAL_REACH = WHEEL_BASE + BUCKET_FRONT_CLEARANCE - ARM_PIVOT_Y
ARM_LENGTH = math.ceil(math.sqrt(ARM_PIVOT_Z**2 + MIN_HORIZONTAL_REACH**2))
ARM_GROUND_ANGLE_RAD = math.asin(ARM_PIVOT_Z / ARM_LENGTH)
ARM_GROUND_ANGLE = math.degrees(ARM_GROUND_ANGLE_RAD)

# Arm V2 Geometry (for ARM_MIN_ANGLE calculation)
TUBE_2X6_1_4_H = 152.4
ARM_ANGLE = 120
ARM_MAIN_LEN = 1224
ARM_DROP_LEN = 550
ARM_OVERLAP = 76.2
ARM_DROP_EXT = 80
ARM_PIVOT_EXT = 40

_bend_angle_deg = 180 - ARM_ANGLE
_bend_angle = math.radians(_bend_angle_deg)
_drop_vec_len = ARM_DROP_LEN + ARM_PIVOT_EXT
_tube_h = TUBE_2X6_1_4_H

_dx = _drop_vec_len * math.cos(_bend_angle) + (-_tube_h/2) * math.sin(_bend_angle)
_dz = -_drop_vec_len * math.sin(_bend_angle) + (-_tube_h/2) * math.cos(_bend_angle)

ARM_TIP_X = (ARM_MAIN_LEN + ARM_OVERLAP + 50) + _dx
ARM_TIP_Z = _tube_h + _dz

_x_rel = ARM_TIP_X
_z_rel = ARM_TIP_Z - _tube_h/2

_arm_eff_len = math.sqrt(_x_rel**2 + _z_rel**2)
arm_alpha_calc = math.atan2(-_z_rel, _x_rel) # radians

_target_height = 100
_theta_rad = math.asin((_target_height - ARM_PIVOT_Z) / _arm_eff_len)
theta_deg_calc = math.degrees(_theta_rad)
arm_alpha_calc_deg = math.degrees(arm_alpha_calc)

ARM_MIN_ANGLE_COMPUTED = theta_deg_calc + arm_alpha_calc_deg
ARM_MIN_ANGLE = ARM_MIN_ANGLE_COMPUTED

ARM_V2_OFFSET_ANGLE = ARM_MIN_ANGLE_COMPUTED + ARM_GROUND_ANGLE
ARM_MAX_ANGLE = 60 + ARM_V2_OFFSET_ANGLE

# Cylinder Mounting Points
LIFT_CYL_ARM_OFFSET = ARM_LENGTH * 0.25

_theta_min = -math.radians(ARM_GROUND_ANGLE) # Note: SCAD uses -ARM_GROUND_ANGLE
_attach_y_world = ARM_PIVOT_Y + LIFT_CYL_ARM_OFFSET * math.cos(_theta_min)
_attach_z_world = ARM_PIVOT_Z + LIFT_CYL_ARM_OFFSET * math.sin(_theta_min)

_min_base_z = FRAME_Z_OFFSET + WHEEL_DIAMETER/2 + 80
LIFT_CYL_BASE_Z = _min_base_z

_target_cyl_angle = 50
_calc_base_y = _attach_y_world - (_attach_z_world - LIFT_CYL_BASE_Z) / math.tan(math.radians(_target_cyl_angle))

LIFT_CYL_BASE_Y = max(50, min(WHEEL_BASE/2, _calc_base_y))

print(f"ARM_LENGTH: {ARM_LENGTH}")
print(f"ARM_PIVOT_Y: {ARM_PIVOT_Y}")
print(f"ARM_PIVOT_Z: {ARM_PIVOT_Z}")
print(f"LIFT_CYL_ARM_OFFSET: {LIFT_CYL_ARM_OFFSET}")
print(f"LIFT_CYL_BASE_Y: {LIFT_CYL_BASE_Y}")
print(f"LIFT_CYL_BASE_Z: {LIFT_CYL_BASE_Z}")
print(f"ARM_MIN_ANGLE: {ARM_MIN_ANGLE}")
print(f"ARM_MAX_ANGLE: {ARM_MAX_ANGLE}")

# Kinematics Calculation
def calculate_distance(angle_deg):
    angle_rad = math.radians(angle_deg)
    # Arm attachment point relative to pivot
    # Assuming the arm angle is 0 when horizontal?
    # Wait, the SCAD code says:
    # _theta_min = -ARM_GROUND_ANGLE;
    # _attach_y_world = ARM_PIVOT_Y + LIFT_CYL_ARM_OFFSET * cos(_theta_min);
    # So the angle is relative to horizontal.
    
    # Arm attachment point in world coordinates
    arm_attach_y = ARM_PIVOT_Y + LIFT_CYL_ARM_OFFSET * math.cos(angle_rad)
    arm_attach_z = ARM_PIVOT_Z + LIFT_CYL_ARM_OFFSET * math.sin(angle_rad)
    
    # Base attachment point in world coordinates
    # Base is fixed
    base_y = LIFT_CYL_BASE_Y
    base_z = LIFT_CYL_BASE_Z
    
    dist = math.sqrt((arm_attach_y - base_y)**2 + (arm_attach_z - base_z)**2)
    return dist

dist_min = calculate_distance(ARM_MIN_ANGLE)
dist_max = calculate_distance(ARM_MAX_ANGLE)

print(f"Distance at Min Angle ({ARM_MIN_ANGLE:.2f}): {dist_min:.2f}")
print(f"Distance at Max Angle ({ARM_MAX_ANGLE:.2f}): {dist_max:.2f}")
