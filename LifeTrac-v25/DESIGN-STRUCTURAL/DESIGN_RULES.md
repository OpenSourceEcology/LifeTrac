# LifeTrac v25 Design Rules & Invariants

This document establishes the non-negotiable geometric rules and parametric constraints for the LifeTrac v25 loader arm and bucket assembly. These rules ensure proper kinematics, ground engagement, and mechanical alignment.

## 1. Bucket Geometry & Ground Engagement
*   **Zero-Z Ground Contact**: When the loader arms are in the lowered position (`ARM_MIN_ANGLE`), the bottom plate of the bucket must be perfectly flush with the ground (Z = 0). `BUCKET_GROUND_CLEARANCE` should default to `0` or slightly negative (e.g., `-10` for digging visual), but never positive.
*   **1/5th Pivot Height Rule**: The center of the Bucket Pivot Pin must be located at exactly **20% (1/5th) of the total Bucket Height** measured from the bottom of the bucket.
    *   Formula: `BUCKET_PIVOT_HEIGHT_FROM_BOTTOM = BUCKET_HEIGHT / 5`
*   **Lug Centering**: The mounting lugs on the bucket back plate must be centered vertically on this 1/5th height line.

## 2. Loader Arm Kinematics (Target-Driven)
*   **Target Priorty**: The arm geometry is solved *backwards* from the target. The Target Point `_T` (Bucket Pivot Pin Location) is the fixed constraint **when the arms are in the fully down position**.
    *   `_T_z` = `BUCKET_HEIGHT / 5` (assuming flush ground contact).
*   **Dynamic Leg Sizing**: The Drop Leg Length (`L2` or `ARM_DROP_LEN`) is not a fixed input. It must be calculated by the solver to bridge the gap between the Main Arm geometry and the fixed Target Point `_T`.
    *   *Constraint*: The arm leg must extend or shorten as needed to ensure the pivot pin hits the exact target Z-height.
*   **Crash Prevention**: The solver must prioritize the Target Point, but validity checks must ensure the arm tip material (steel extending below the pivot) does not crash into the ground before the bucket does.

## 3. Physical Component Alignment
*   **Lug Flush Mounting**: The back (base) of any U-channel lug or mounting bracket must be flush with the surface it is intended to be mounted to. For bucket pivot lugs, the lug back must be flush with the back plate of the bucket. No overlap or gap between lug base and mounting surface.
*   **Matching Tip Radii**: The radius of the Loader Arm Tip (`boss_r` in `arm_plate.scad`) must visually and dimensionally match the Bucket Pivot Lugs to ensure a clean hinge assembly.
    *   Constraint: `boss_r` == `PIVOT_HOLE_X_FROM_FRONT`.
*   **Hole Alignment**: The physical hole cut into the *Arm Plate* must strictly match the kinematic point `_T`.
    *   **Z-Position**: Centered on the 6" tube profile (`PIVOT_HOLE_Z_FROM_BOTTOM = 76.2mm`).
    *   **X-Position**: Defined by `PIVOT_HOLE_X_FROM_FRONT` (1.5" / 38.1mm).
*   **Hole Edge Margin**: For any structural plate with a hole (pivot holes, bolt holes), there must be at least a fixed value of **0.25 inch (6.35mm)** of material remaining between the edge of the hole and the edge of the plate.

## 4. Parametric Hierarchy
*   **Single Source of Truth**: All shared dimensions (Pin diameters, tube sizes, critical angles) must be defined in `lifetrac_v25_params.scad`.
*   **No Hardcoding**: Layout files (`lifetrac_v25.scad`) and Part files (`arm_plate.scad`) must never use raw numbers for geometric alignment. They must reference the global parameters.
*   **Solver Verification**: Any manual adjustment to the "Visual" arm model must be cross-checked against the "Kinematic" solver variables (`_L_drop_kinematic`, `_T_z`) to ensures the physical parts match the calculated motion path.

## 5. Bucket Curl & Range of Motion
*   **Maximum Curl Definition**: The bucket is considered curled to its maximum extent ("Full Curl") when the **Back Plate of the Bucket is parallel to the Drop Leg** (lower section) of the Loader Arm.
    *   This physical hard stop prevents cylinder over-extension and defines the maximum material retention angle.
*   **Cylinder Retraction**: The mounting position of the crossbeam must be adjusted so that the bucket's hydraulic tilt cylinders are in the **fully retracted position** when the bucket is at the "Full Curl" position.
    *   This ensures the mechanical limit (curl) aligns with the hydraulic limit (retraction).
*   **Dump Angle**: When the loader arms are in the fully raised position, the bucket must be capable of tilting downwards to at least **45 degrees below horizontal** (relative to ground level).
    *   This ensures proper dumping of material at maximum height.
*   **Cylinder Stroke Usage**: The geometry should utilize at least **80-90% of the hydraulic cylinder stroke**.
    *   Designing for 100% usage is risky (mechanical bottoming), but designing for < 50% usage is wasteful and reduces force efficiency.

## 6. Structural Standards & Clearance
*   **Wheel Clearance**: The Main Arm must maintain a minimum distance of **25mm (1 inch)** from the Front Wheels throughout the *entire* range of motion, not just at the closest approach point or rest position.
*   **Standard Material Sizes**: All major structural elements (arms, frame, lugs) must use dimensions available in standard steel stock (e.g., 3"x3", 2"x6", 4"x4" tubes). 
    *   *Rule*: No custom bent profiles or non-standard extrusions where standard rectangular/square tubing can be used.
