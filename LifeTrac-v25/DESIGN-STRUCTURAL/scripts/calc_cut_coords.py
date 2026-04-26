
import math

def calculate_cut_coordinates():
    # 1. Parsing Parameters (Hardcoded from read_file values for reliability)
    # From lifetrac_v25_params.scad
    # TUBE_2X6_1_4 = [50.8, 152.4, 6.35]
    tube_h = 152.4
    
    # From arm_plate.scad
    # drop_ext = ARM_DROP_EXT = 80
    drop_ext = 80.0
    
    # triangle_leg = tube_h / 2
    triangle_leg = tube_h / 2.0 # 76.2
    
    print(f"Parameters:")
    print(f"  tube_h: {tube_h}")
    print(f"  drop_ext: {drop_ext}")
    print(f"  triangle_leg: {triangle_leg}")
    print("-" * 30)

    # 2. Polygon Definition (2D xy plane)
    # polygon([[2, -2], [-triangle_leg, -2], [2, triangle_leg]]);
    poly_vertices_2d = [
        (2, -2),
        (-triangle_leg, -2),
        (2, triangle_leg)
    ]
    
    vertex_names = ["Right-Bottom (P1)", "Left-Bottom (P2)", "Right-Top (P3)"]

    print("Step 1: Polygon Vertices (2D Local):")
    for name, v in zip(vertex_names, poly_vertices_2d):
        print(f"  {name}: {v}")
    print("-" * 30)

    # 3. Simulate Transformations
    # The transformation chain in OpenSCAD is:
    # translate([drop_ext, -1, 0])
    # rotate([-90, 0, 0])
    # linear_extrude...
    
    # We maintain 3D coordinates.
    # Initial State (After Extrusion, before Rotate/Translate):
    # The polygon is on the XY plane. Z is the extrusion axis.
    # We are interested in the profile cut, which is independent of extrusion thickness (Z).
    # But the rotation swaps axes.
    
    # Let's track a point P = (x, y, z_ext)
    # We can effectively ignore z_ext (thickness) for the "Side View" (XZ plane of the final object)
    # strictly speaking, 'y' in the polygon becomes 'z' in the final object (with sign change).
    
    # Operation: rotate([-90, 0, 0])
    # RotMatrix X(-90):
    # [1,  0,       0]
    # [0, cos(-90), -sin(-90)] -> [0, 0, 1]
    # [0, sin(-90),  cos(-90)] -> [0, -1, 0]
    
    # P_rotated = M * P
    # x_new = x
    # y_new = z_ext
    # z_new = -y
    
    # Operation: translate([drop_ext, -1, 0])
    # P_final = P_rotated + [drop_ext, -1, 0]
    # x_final = x + drop_ext
    # y_final = z_ext - 1
    # z_final = -y + 0
    
    print("Step 2: Calculated World Coordinates (Relative to Extension Base):")
    print("  Extension Box Bounds: X[0, 80], Z[0, 152.4]")
    print("  Cut Vertices (x, z):")
    
    min_x, max_x = float('inf'), float('-inf')
    min_z, max_z = float('inf'), float('-inf')

    for name, v in zip(vertex_names, poly_vertices_2d):
        x_poly, y_poly = v
        
        # Transform
        x_world = x_poly + drop_ext
        z_world = -y_poly
        
        print(f"  {name}: X={x_world:.2f}, Z={z_world:.2f}")
        
        min_x = min(min_x, x_world)
        max_x = max(max_x, x_world)
        min_z = min(min_z, z_world)
        max_z = max(max_z, z_world)

    print("-" * 30)
    print("Summary of Cut Position:")
    print(f"  X Range: [{min_x:.2f}, {max_x:.2f}] (Extension is 0 to 80)")
    print(f"  Z Range: [{min_z:.2f}, {max_z:.2f}] (Extension is 0 to 152.4)")
    
    # 4. Analysis
    print("-" * 30)
    print("Analysis:")
    
    # Check X location
    if min_x > 40:
        print("  - The cut is located at the TIP (X=80 end).")
    elif max_x < 40:
        print("  - The cut is located at the HEEL (X=0 end).")
    else:
        print("  - The cut spans the middle.")
        
    # Check Z location
    if max_z <= 0:
        print("  - The cut is completely BELOW the part (Negative Z).")
    elif min_z >= 152.4:
        print("  - The cut is completely ABOVE the part.")
    elif max_z < 5 and min_z < 0:
        print("  - The cut barely touches the BOTTOM edge.")
    else:
        print("  - The cut intersects the part body.")

    # Target: Back Corner Taper at X=0, Z=0?
    print("\nExpected Location for 'Back Corner Taper':")
    print("  - Should be near X=0.")
    print("  - Should be cutting upwards into Z (Positive Z).")
    
if __name__ == "__main__":
    calculate_cut_coordinates()
