import re
import os
import math

# Define paths
base_path = r"c:\Users\dorkm\Documents\GitHub\LifeTrac\LifeTrac-v25\mechanical_design\openscad"
params_file = os.path.join(base_path, "lifetrac_v25_params.scad")
assembly_file = os.path.join(base_path, "lifetrac_v25.scad")
arm_plate_file = os.path.join(base_path, "parts", "arm_plate.scad")

# Helper to read file
def read_scad(path):
    try:
        with open(path, 'r') as f:
            return f.read()
    except FileNotFoundError:
        print(f"Error: Could not find file {path}")
        return ""

# Helper to extract variable value
def get_var(content, var_name):
    pattern = r"^\s*" + re.escape(var_name) + r"\s*=\s*([^;]+);"
    match = re.search(pattern, content, re.MULTILINE)
    if match:
        val_str = match.group(1).split("//")[0].strip()
        try:
            val_str = val_str.replace("PLATE_1_4_INCH", "6.35")
            val_str = val_str.replace("PLATE_1_2_INCH", "12.7")
            val_str = val_str.replace("PLATE_3_4_INCH", "19.05")
            val_str = val_str.replace("BUCKET_HEIGHT", "450") # Resolver stub
            val_str = val_str.replace("BUCKET_PIVOT_PIN_DIA", "25.4")
            
            # Safe eval context
            context = {"__builtins__": None, "max": max, "min": min, "pow": pow, "sqrt": math.sqrt}
            return eval(val_str, context)
        except Exception as e:
            return val_str
    return None

# Load files
print("Loading files...")
params_content = read_scad(params_file)
arm_plate_content = read_scad(arm_plate_file)

if not params_content:
    exit(1)

results = {}

# --- Rule 1 Checks ---
print("\n--- Rule 1: Bucket Geometry & Ground ---")
r1_ground = get_var(params_content, "BUCKET_GROUND_CLEARANCE")
r1_height = get_var(params_content, "BUCKET_HEIGHT")
r1_pivot = get_var(params_content, "BUCKET_PIVOT_HEIGHT_FROM_BOTTOM")

print(f"BUCKET_GROUND_CLEARANCE: {r1_ground}")
if r1_ground is not None and r1_ground <= 0:
    results["R1.1_Zero_Z"] = "PASS"
else:
    results["R1.1_Zero_Z"] = "FAIL"

print(f"BUCKET_HEIGHT: {r1_height}")
print(f"PIVOT_HEIGHT: {r1_pivot}")
if r1_height and r1_pivot:
    expected = r1_height / 5
    if abs(r1_pivot - expected) < 0.01:
        results["R1.2_1/5th_Ratio"] = "PASS"
    else:
        results["R1.2_1/5th_Ratio"] = f"FAIL (Got {r1_pivot}, Expected {expected})"

# --- Rule 2 Checks ---
print("\n--- Rule 2: Kinematics ---")
# Check for max(50 clamp
if "max(50" in params_content and "_T_z" in params_content:
    tz_match = re.search(r"_T_z\s*=\s*(.*);", params_content)
    if tz_match and "max(50" in tz_match.group(1):
        results["R2.1_Target_Priority"] = "FAIL (Clamp found)"
    else:
        results["R2.1_Target_Priority"] = "PASS"
else:
    results["R2.1_Target_Priority"] = "PASS"

# --- Rule 3 Checks ---
print("\n--- Rule 3: Component Alignment ---")
boss_r_match = re.search(r"boss_r\s*=\s*PIVOT_HOLE_X_FROM_FRONT", arm_plate_content)
if boss_r_match:
    results["R3.1_Tip_Radius"] = "PASS"
else:
    results["R3.1_Tip_Radius"] = "FAIL (boss_r link missing)"

# --- Rule 6 Checks ---
print("\n--- Rule 6: Safety ---")
steel_r = get_var(params_content, "PIVOT_HOLE_X_FROM_FRONT")
pin_d = get_var(params_content, "BUCKET_PIVOT_PIN_DIA")

print(f"PIVOT_HOLE_X_FROM_FRONT (Steel Radius): {steel_r}")
print(f"BUCKET_PIVOT_PIN_DIA: {pin_d}")

if steel_r and pin_d:
    margin = steel_r - (pin_d / 2)
    print(f"Calculated Margin: {margin:.2f}mm")
    if margin >= 6.35:
        results["R6.2_Edge_Margin"] = "PASS"
    else:
        results["R6.2_Edge_Margin"] = "FAIL"
else:
    results["R6.2_Edge_Margin"] = "ERROR"

print("\n--- FINAL RESULTS ---")
for k, v in results.items():
    print(f"{k}: {v}")
