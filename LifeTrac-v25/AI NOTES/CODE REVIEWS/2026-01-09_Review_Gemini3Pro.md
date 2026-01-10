# Code Review: LifeTrac v25
**Reviewer:** Gemini 3 Pro  
**Date:** January 9, 2026

## Executive Summary
The LifeTrac v25 design demonstrates a high degree of parametric modeling, with a centralized parameter file driving the control geometry of the loader arms and frame. This allows for rapid iteration of machine width, length, and arm reach. However, there are several "Single Source of Truth" violations where logic is duplicated across SCAD and Python files, and some manual overrides in the assembly file that undermine the parametric intent.

From a manufacturing perspective, the design is generally sound but contains specific violations of CNC best practices (hard internal corners) that will necessitate manual filing or design changes before cutting.

## Detailed Analysis

### 1. Parametric Design & Automation
**Status:** B+ (Good with specific flaws)

*   **Strengths:**
    *   `lifetrac_v25_params.scad` effectively calculates complex linkages (e.g., `CROSS_BEAM_1_POS` based on cylinder stroke and curl angles).
    *   Key dimensions (Machine Width, Wheel Base) are variables, not magic numbers.
*   **Weaknesses:**
    *   **Manual Overrides:** In `lifetrac_v25.scad`, the cross beam placement has a manual hardcoded offset: `translate([0, CROSS_BEAM_1_POS - 76.2, 0])`. This breaks the automatic layout calculated in the parameters file.
    *   **Shadowing:** `lifetrac_v25.scad` re-declares material constants (e.g., `PLATE_1_4_INCH`) that are already imported from `lifetrac_v25_params.scad`. This creates a risk of divergent values.
    *   **Duplication:** `calc_kinematics.py` duplicates the geometry logic and constants found in `lifetrac_v25_params.scad` but does not import them. It is currently a dead branch of information.

### 2. Weight & Capacity Calculations
**Status:** Incomplete

*   **Observations:**
    *   The `calc_kinematics.py` script and SCAD parameters calculate *geometric* capacity (reach, dump angle, curl angle) but do not calculate *load* capacity.
    *   There are no calculations for tipping load, hydraulic cylinder force (Force = Pressure × Area), or structural stress on the pivot pins.
    *   **Risk:** The 1.5" and 1" pivot pins are standard sizes, but without shear calculation, we cannot confirm they match the machine's breakout force.

### 3. Manufacturing & CNC Best Practices
**Status:** Needs Improvement

*   **Hard Internal Corners:**
    *   **File:** `parts/arm_plate.scad`
    *   **Issue:** The cutout for the cross beam is a perfect rectangle:
        ```openscad
        cube([cross_beam_w, cross_beam_h, plate_thick+2], center=true);
        ```
    *   **Impact:** A CNC plasma or laser cutter cannot cut a perfectly sharp internal 90-degree corner (kerf radius). Furthermore, a square tube with rounded corners (standard ASTM A500) will not fit flush into a square hole without gaps or interference.
    *   **Recommendation:** Add "Dogbrake" or "Mickey Mouse ear" reliefs to the corners of the slot, or specify a file-to-fit operation (labor intensive).

*   **Floating Parts / Connectivity:**
    *   The `angle_iron_mount` in `loader_arm_v2.scad` relies on hardcoded offsets relative to `CROSS_BEAM_1_POS`. If the beam moves (parametrically), verify if these mounts move correctly. The current Manual Override in the main assembly suggests they might not be aligning as expected.
    *   The visually distinct "Inner" and "Outer" plates in `loader_arm_v2` handle the asymmetry well.

### 4. Standardization
**Status:** Good

*   **Standard Parts:**
    *   The design consistently uses 1/4" wall tubing and standard bolt sizes defined in parameters.
*   **Hydraulics:**
    *   `hydraulics.scad` generates cylinder geometry visually based on bore size, but `lifetrac_v25_params.scad` defines `LIFT_CYL_LEN_MIN` based on stroke + fixed dead length constants.
    *   **Check:** Ensure the physical cylinders purchased match the `DEAD_LENGTH` assumed in `lifetrac_v25_params.scad` (currently approx 8 inches).

## Action Plan

1.  **Remove Manual Overrides:** Investigate why `lifetrac_v25.scad` requires `-76.2` offset for the cross beam. Fix the calculation in `lifetrac_v25_params.scad` so the assembly can rely on the variable directly.
2.  **Fix CNC Cutouts:** Modify `arm_plate.scad` to include corner reliefs for the cross-beam slot.
3.  **Single Source of Truth:** Remove the re-declared variables in `lifetrac_v25.scad`.
4.  **Hardware Verification:** Add a simple echo in `lifetrac_v25_params.scad` that outputs the expected Tipping Load based on a standard 3000 PSI hydraulic pressure and the calculated leverage ratios.

