
import math

def solve_arm(L1_guess):
    # Constants from SCAD files
    PLATE_1_4_INCH = 6.35
    TUBE_2X6_1_4 = [50.8, 152.4, 6.35]
    
    MACHINE_WIDTH = 1200
    MACHINE_LENGTH = 1800
    MACHINE_HEIGHT = 1000
    WHEEL_BASE = 1400
    TRACK_WIDTH = 900
    
    WHEEL_DIAMETER = 500
    WHEEL_WIDTH = 200
    WHEEL_CLEARANCE = 50
    
    GROUND_CLEARANCE = 150
    WHEEL_RADIUS = WHEEL_DIAMETER / 2
    FRAME_Z_OFFSET = GROUND_CLEARANCE
    
    BUCKET_GROUND_CLEARANCE = -42
    BUCKET_PIVOT_HEIGHT_BASE = 200
    BUCKET_PIVOT_HEIGHT_ADJUST = -5 * 25.4
    BUCKET_PIVOT_HEIGHT_FROM_BOTTOM = BUCKET_PIVOT_HEIGHT_BASE + BUCKET_PIVOT_HEIGHT_ADJUST
    
    ARM_PIVOT_Y = 200
    ARM_PIVOT_Z_HEIGHT = MACHINE_HEIGHT - 50
    ARM_PIVOT_Z = FRAME_Z_OFFSET + ARM_PIVOT_Z_HEIGHT
    
    ARM_SHAPE_ANGLE = 130
    WHEEL_CLEARANCE_TARGET = 25.4
    
    # Derived Points
    P = [ARM_PIVOT_Y, ARM_PIVOT_Z] # Pivot
    C = [WHEEL_BASE, FRAME_Z_OFFSET + WHEEL_RADIUS] # Wheel Center
    
    BUCKET_X_ADJUST = 0
    buck_clearance = 1.0 * 25.4
    T_x = WHEEL_BASE + WHEEL_RADIUS + buck_clearance + BUCKET_X_ADJUST
    T_z = BUCKET_PIVOT_HEIGHT_FROM_BOTTOM
    T = [T_x, T_z] # Bucket Pivot
    
    R_wheel = WHEEL_RADIUS
    tube_offset = TUBE_2X6_1_4[1] / 2
    R_constraint = R_wheel + WHEEL_CLEARANCE_TARGET + tube_offset
    
    # Solver Logic
    dist_PT = math.sqrt((T[1]-P[1])**2 + (T[0]-P[0])**2)
    angle_PT = math.atan2(T[1]-P[1], T[0]-P[0])
    
    gamma_rad = math.radians(ARM_SHAPE_ANGLE)
    
    # Cosine Rule for L2
    # d^2 = L1^2 + L2^2 - 2*L1*L2*cos(gamma)
    # L2^2 + (-2*L1*cos(gamma))*L2 + (L1^2 - d^2) = 0
    
    qa = 1
    qb = -2 * L1_guess * math.cos(gamma_rad)
    qc = L1_guess**2 - dist_PT**2
    
    det = qb**2 - 4 * qa * qc
    if det < 0:
        return None, None, None
        
    L2 = (-qb + math.sqrt(det)) / (2 * qa)
    
    # Determine Orientation
    # L2^2 = L1^2 + d^2 - 2*L1*d*cos(alpha)
    cos_alpha = (L1_guess**2 + dist_PT**2 - L2**2) / (2 * L1_guess * dist_PT)
    
    # Clamp for safety
    cos_alpha = max(-1, min(1, cos_alpha))
    
    angle_EPT = math.acos(cos_alpha)
    angle_main = angle_PT + angle_EPT
    
    # Calculate E (Elbow)
    E = [
        P[0] + L1_guess * math.cos(angle_main),
        P[1] + L1_guess * math.sin(angle_main)
    ]
    
    # Calculate Drop Angle
    # Vector ET
    vec_ET = [T[0]-E[0], T[1]-E[1]]
    angle_drop = math.atan2(vec_ET[1], vec_ET[0])
    
    # Calculate Distance to Center C
    # Dist from C to Line Segment PE (Main Arm)
    # Line defined by P + u * t.
    # Closest point t0 = Dot(C-P, u) / Dot(u, u) (u is unit)
    u_main = [math.cos(angle_main), math.sin(angle_main)]
    vec_PC = [C[0]-P[0], C[1]-P[1]]
    t0_main = vec_PC[0]*u_main[0] + vec_PC[1]*u_main[1]
    
    # We assume closest point is ON segment, but might be near E?
    # Usually C is "under" the arm, so it projects onto the segment.
    closest_main = [P[0] + t0_main*u_main[0], P[1] + t0_main*u_main[1]]
    dist_main_center = math.sqrt((C[0]-closest_main[0])**2 + (C[1]-closest_main[1])**2)
    clearance_main = dist_main_center - R_wheel - tube_offset
    
    # Dist from C to Line Segment ET (Drop Arm)
    u_drop = [math.cos(angle_drop), math.sin(angle_drop)]
    vec_EC = [C[0]-E[0], C[1]-E[1]]
    t0_drop = vec_EC[0]*u_drop[0] + vec_EC[1]*u_drop[1]
    
    closest_drop = [E[0] + t0_drop*u_drop[0], E[1] + t0_drop*u_drop[1]]
    dist_drop_center = math.sqrt((C[0]-closest_drop[0])**2 + (C[1]-closest_drop[1])**2)
    clearance_drop = dist_drop_center - R_wheel - tube_offset
    
    return clearance_main, clearance_drop, E

print("Searching for optimal L1...")
best_L1 = 0
min_diff = 9999
for l1 in range(1300, 1600, 1):
    c_m, c_d, E = solve_arm(l1)
    if c_m is None: continue
    diff = abs(c_m - c_d)
    # print(f"L1: {l1}, Main: {c_m:.2f}, Drop: {c_d:.2f}, Diff: {diff:.2f}")
    if diff < min_diff:
        min_diff = diff
        best_L1 = l1
        best_cm = c_m
        best_cd = c_d

print(f"Optimal L1: {best_L1}")
print(f"Main Clearance: {best_cm}")
print(f"Drop Clearance: {best_cd}")
