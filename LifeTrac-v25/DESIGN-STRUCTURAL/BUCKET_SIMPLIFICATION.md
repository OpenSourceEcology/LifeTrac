# Bucket Simplification

## Overview
The bucket design has been simplified to remove the complex "Quick Attach" (QA) interface while preserving the hydraulic cylinder mounting points. This change was made to streamline the design and focus on a direct, bolted connection for the loader arms.

## Changes
1.  **Removed `bobcat_quick_attach_plate` Module**:
    *   Deleted the entire module which contained the hook bar, wedge pockets, locking pins, and flange plates.
    *   This removes the "long rod" and "round objects" that were identified as unwanted leftovers.

2.  **Updated `bucket` Module**:
    *   Removed the call to `bobcat_quick_attach_plate()`.
    *   Added the hydraulic cylinder mounting lugs directly to the `bucket` module.
    *   The lugs are positioned using `BUCKET_CYL_X_SPACING` to maintain alignment with the loader arm cylinders.
    *   The lugs are standard U-channel mounts (`u_channel_lug_with_pin`).

## Geometry Verification
*   **Pivot Connection**: Handled in `bucket_attachment()` using bolted U-channel lugs.
*   **Cylinder Connection**: Handled in `bucket()` using welded/bolted U-channel lugs.
*   **Alignment**: Both sets of lugs are positioned to abut the back plate of the bucket (which is offset by 38.1mm from the pivot axis).

## Next Steps
*   Verify the assembly in OpenSCAD.
*   Check if any other parts of the system relied on the QA plate geometry (unlikely, as the main dependency was the cylinder mount position which has been preserved).
