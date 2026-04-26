// pivot_mount_assembly.scad
// Replaceable Pivot Mount Assembly for Loader Arm
// Part of LifeTrac v25 OpenSCAD design
//
// This assembly consists of:
// - 2 circular 6" diameter plates
// - DOM (Drawn Over Mandrel) tube welded between the plates
// - 6 bolt holes on each plate for mounting to arm CNC plates
//
// The assembly slides into a slot cut from the bottom of the arm
// and bolts to both arm plates, allowing easy replacement of the
// pivot without welding the arm plates.

include <../lifetrac_v25_params.scad>

// =============================================================================
// PIVOT MOUNT PARAMETERS
// =============================================================================
// Note: Primary parameters are defined in lifetrac_v25_params.scad
// These local definitions provide fallbacks for standalone use

// Circular plate parameters
_PIVOT_MOUNT_PLATE_DIA = is_undef(PIVOT_MOUNT_PLATE_DIA) ? 152.4 : PIVOT_MOUNT_PLATE_DIA;          // 6" = 152.4mm diameter
_PIVOT_MOUNT_PLATE_THICK = is_undef(PIVOT_MOUNT_PLATE_THICK) ? PLATE_1_4_INCH : PIVOT_MOUNT_PLATE_THICK; // 1/4" plate thickness

// DOM tube parameters (from params file)
// DOM_PIPE_OD = 50.8;  // 2" OD
// DOM_PIPE_ID = 38.1;  // 1.5" ID (fits pivot pin)

// Assembly width (distance between outer faces of plates)
// This should match the arm tube width (2" tube)
_PIVOT_MOUNT_WIDTH = is_undef(PIVOT_MOUNT_WIDTH) ? TUBE_2X6_1_4[0] : PIVOT_MOUNT_WIDTH;    // 50.8mm (2")

// Bolt pattern parameters
_PIVOT_MOUNT_BOLT_COUNT = is_undef(PIVOT_MOUNT_BOLT_COUNT) ? 5 : PIVOT_MOUNT_BOLT_COUNT;             // Number of bolts per plate
_PIVOT_MOUNT_BOLT_DIA = is_undef(PIVOT_MOUNT_BOLT_DIA) ? BOLT_DIA_1_2 : PIVOT_MOUNT_BOLT_DIA;    // 1/2" bolts
_PIVOT_MOUNT_BOLT_CIRCLE_DIA = is_undef(PIVOT_MOUNT_BOLT_CIRCLE_DIA) ? 114.3 : PIVOT_MOUNT_BOLT_CIRCLE_DIA;    // 4.5" bolt circle diameter
// Bolt angles array for non-uniform spacing (larger gap around slot)
_PIVOT_MOUNT_BOLT_ANGLES = is_undef(PIVOT_MOUNT_BOLT_ANGLES) ? 
    [295.5, 372.6, 449.8, 526.9, 604.1] : PIVOT_MOUNT_BOLT_ANGLES;

// DOM tube length (including welding to plates)
_PIVOT_MOUNT_DOM_LENGTH = _PIVOT_MOUNT_WIDTH; // Same as assembly width

// Clearance from plate OD to tube start
_PIVOT_MOUNT_CLEARANCE = is_undef(PIVOT_MOUNT_CLEARANCE) ? 25.4 : PIVOT_MOUNT_CLEARANCE; // 1" clearance

// =============================================================================
// CIRCULAR PLATE MODULE
// =============================================================================

module pivot_mount_plate() {
    // 6" circular plate with:
    // - Center hole for DOM tube (OD)
    // - 5 bolt holes on 4.5" bolt circle (for pivot mount to arm plate)
    // - NON-UNIFORM spacing: larger gap around slot, closer spacing elsewhere
    
    difference() {
        // Main circular plate
        cylinder(d=_PIVOT_MOUNT_PLATE_DIA, h=_PIVOT_MOUNT_PLATE_THICK, $fn=64);
        
        // Center hole for DOM tube
        translate([0, 0, -1])
            cylinder(d=DOM_PIPE_OD, h=_PIVOT_MOUNT_PLATE_THICK + 2, $fn=64);
        
        // 5 bolt holes using pre-calculated angles array (non-uniform spacing)
        for (i = [0:_PIVOT_MOUNT_BOLT_COUNT-1]) {
            angle = _PIVOT_MOUNT_BOLT_ANGLES[i];
            bolt_x = (_PIVOT_MOUNT_BOLT_CIRCLE_DIA / 2) * cos(angle);
            bolt_y = (_PIVOT_MOUNT_BOLT_CIRCLE_DIA / 2) * sin(angle);
            
            translate([bolt_x, bolt_y, -1])
                cylinder(d=_PIVOT_MOUNT_BOLT_DIA, h=_PIVOT_MOUNT_PLATE_THICK + 2, $fn=24);
        }
    }
}

// =============================================================================
// DOM TUBE MODULE
// =============================================================================

module pivot_mount_dom_tube() {
    // DOM tube section
    // Drawn Over Mandrel tubing has precise tolerances
    
    difference() {
        cylinder(d=DOM_PIPE_OD, h=_PIVOT_MOUNT_DOM_LENGTH, $fn=64);
        translate([0, 0, -1])
            cylinder(d=DOM_PIPE_ID, h=_PIVOT_MOUNT_DOM_LENGTH + 2, $fn=64);
    }
}

// =============================================================================
// COMPLETE ASSEMBLY MODULE
// =============================================================================

module pivot_mount_assembly(show_exploded=false) {
    // Complete welded assembly:
    // - Plate 1 at Y=0
    // - DOM tube in center
    // - Plate 2 at Y=PIVOT_MOUNT_WIDTH
    //
    // Assembly is oriented with:
    // - X,Z plane parallel to circular plates
    // - Y axis along the DOM tube (width of arm)
    // - DOM tube centerline at X=0, Z=0
    
    explode_dist = show_exploded ? 30 : 0;
    
    // Assembly is positioned so that when placed in the arm:
    // - DOM center is at arm tube center height (tube_h/2)
    // - Plates bolt to arm plates
    
    color("SteelBlue") {
        // Plate 1 (front face at Y=0)
        translate([0, 0, -explode_dist])
        rotate([90, 0, 0])
            pivot_mount_plate();
        
        // DOM Tube (between plates)
        rotate([90, 0, 0])
        translate([0, 0, -_PIVOT_MOUNT_WIDTH + _PIVOT_MOUNT_PLATE_THICK])
            pivot_mount_dom_tube();
        
        // Plate 2 (back face at Y=_PIVOT_MOUNT_WIDTH - plate thickness)
        translate([0, _PIVOT_MOUNT_WIDTH - _PIVOT_MOUNT_PLATE_THICK, explode_dist])
        rotate([90, 0, 0])
            pivot_mount_plate();
    }
}

// =============================================================================
// INDIVIDUAL PARTS FOR CNC/FABRICATION
// =============================================================================

module pivot_mount_plate_flat() {
    // Flat plate for CNC cutting
    // This is the 2D profile of one plate
    
    difference() {
        circle(d=_PIVOT_MOUNT_PLATE_DIA, $fn=64);
        
        // Center hole for DOM tube
        circle(d=DOM_PIPE_OD, $fn=64);
        
        // 5 bolt holes using pre-calculated angles (non-uniform spacing)
        for (i = [0:_PIVOT_MOUNT_BOLT_COUNT-1]) {
            angle = _PIVOT_MOUNT_BOLT_ANGLES[i];
            bolt_x = (_PIVOT_MOUNT_BOLT_CIRCLE_DIA / 2) * cos(angle);
            bolt_y = (_PIVOT_MOUNT_BOLT_CIRCLE_DIA / 2) * sin(angle);
            
            translate([bolt_x, bolt_y])
                circle(d=_PIVOT_MOUNT_BOLT_DIA, $fn=24);
        }
    }
}

// =============================================================================
// ARM PLATE SLOT CUTOUT PROFILE
// =============================================================================
// This defines the slot that needs to be cut into the arm plates
// to allow the pivot mount assembly to slide in from the bottom

module arm_pivot_slot_profile() {
    // The slot runs from the bottom of the arm plate to the center
    // where the DOM tube will be located
    
    // Slot dimensions:
    // - Width = DOM_PIPE_OD (to fit the DOM tube)
    // - Height = from bottom edge to center of arm tube
    
    tube_h = TUBE_2X6_1_4[1];  // 6" arm tube height (152.4mm)
    slot_height = tube_h / 2;   // From bottom to center
    
    // Slot profile (union of rectangle + semicircle at top for DOM)
    union() {
        // Rectangular slot from bottom
        translate([-DOM_PIPE_OD/2, 0])
            square([DOM_PIPE_OD, slot_height]);
        
        // Semicircle at top for DOM tube clearance
        translate([0, slot_height])
            circle(d=DOM_PIPE_OD, $fn=64);
    }
}

// =============================================================================
// ARM PLATE MOUNTING HOLES PATTERN
// =============================================================================
// These holes need to be added to the arm plates to match the pivot mount

module arm_pivot_mounting_holes_profile() {
    // Same 5-bolt pattern as the pivot mount plates
    // Centered on the arm's pivot point (DOM center)
    // NON-UNIFORM spacing: larger gap around slot
    
    for (i = [0:_PIVOT_MOUNT_BOLT_COUNT-1]) {
        angle = _PIVOT_MOUNT_BOLT_ANGLES[i];
        bolt_x = (_PIVOT_MOUNT_BOLT_CIRCLE_DIA / 2) * cos(angle);
        bolt_y = (_PIVOT_MOUNT_BOLT_CIRCLE_DIA / 2) * sin(angle);
        
        translate([bolt_x, bolt_y])
            circle(d=_PIVOT_MOUNT_BOLT_DIA, $fn=24);
    }
}

// =============================================================================
// RENDER PREVIEW
// =============================================================================

// Show exploded view when run directly
$fn = 32;

echo("=== PIVOT MOUNT ASSEMBLY SPECIFICATIONS ===");
echo("Plate Diameter:", _PIVOT_MOUNT_PLATE_DIA, "mm (6 inches)");
echo("Plate Thickness:", _PIVOT_MOUNT_PLATE_THICK, "mm");
echo("DOM OD:", DOM_PIPE_OD, "mm");
echo("DOM ID:", DOM_PIPE_ID, "mm (fits 1.5\" pivot pin)");
echo("Assembly Width:", _PIVOT_MOUNT_WIDTH, "mm");
echo("Bolt Count:", _PIVOT_MOUNT_BOLT_COUNT, "per plate");
echo("Bolt Circle Diameter:", _PIVOT_MOUNT_BOLT_CIRCLE_DIA, "mm (4.5 inches)");
echo("Bolt Diameter:", _PIVOT_MOUNT_BOLT_DIA, "mm (1/2 inch)");
echo("Clearance to Tube:", _PIVOT_MOUNT_CLEARANCE, "mm (1 inch)");

// Preview the assembly
pivot_mount_assembly(show_exploded=false);

// Show flat plate below for CNC reference
translate([0, -100, 0])
    linear_extrude(height=_PIVOT_MOUNT_PLATE_THICK)
        pivot_mount_plate_flat();
