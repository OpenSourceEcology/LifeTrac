// pivot_mount_welding_jig.scad
// 3D Printed Welding Jig for Pivot Mount Assembly
// Part of LifeTrac v25 OpenSCAD design
//
// This jig holds the two circular plates and DOM tube in proper alignment
// during welding. It ensures:
// - Correct spacing between plates (arm tube width)
// - DOM tube centered on both plates
// - Plates perpendicular to DOM tube
// - Access for welding around the DOM-to-plate joints

include <../lifetrac_v25_params.scad>

// =============================================================================
// JIG PARAMETERS
// =============================================================================

// Inherit from params file
// JIG_WALL_THICKNESS = 4;   // Standard wall for 3D printed parts
// JIG_CLEARANCE = 0.5;      // Fit clearance

// Pivot mount dimensions (use params if available, else fallback)
_JIG_PIVOT_PLATE_DIA = is_undef(PIVOT_MOUNT_PLATE_DIA) ? 152.4 : PIVOT_MOUNT_PLATE_DIA;
_JIG_PIVOT_PLATE_THICK = is_undef(PIVOT_MOUNT_PLATE_THICK) ? PLATE_1_4_INCH : PIVOT_MOUNT_PLATE_THICK;
_JIG_PIVOT_WIDTH = is_undef(PIVOT_MOUNT_WIDTH) ? TUBE_2X6_1_4[0] : PIVOT_MOUNT_WIDTH;
_JIG_PIVOT_BOLT_CIRCLE_DIA = is_undef(PIVOT_MOUNT_BOLT_CIRCLE_DIA) ? 114.3 : PIVOT_MOUNT_BOLT_CIRCLE_DIA;
_JIG_PIVOT_CLEARANCE = is_undef(PIVOT_MOUNT_CLEARANCE) ? 25.4 : PIVOT_MOUNT_CLEARANCE;

// Jig dimensions
JIG_BASE_THICK = 8;                     // Base plate thickness
JIG_SUPPORT_HEIGHT = 30;                // Height of plate supports
JIG_FINGER_COUNT = 6;                   // Number of support fingers around plate
JIG_FINGER_WIDTH = 20;                  // Width of each support finger
JIG_FINGER_THICKNESS = JIG_WALL_THICKNESS;

// DOM alignment features
DOM_CRADLE_DEPTH = 15;                  // How deep the DOM sits in cradle
DOM_CRADLE_ANGLE = 120;                 // Arc angle of DOM cradle (degrees)

// Clearance for welding access
WELD_ACCESS_GAP = 25;                   // Gap between jig parts for welder access

// =============================================================================
// JIG BASE PLATE MODULE
// =============================================================================

module jig_base_plate() {
    // Base plate that sits under the assembly
    // Has cradle for DOM tube and supports for circular plates
    
    base_length = _JIG_PIVOT_WIDTH + _JIG_PIVOT_PLATE_DIA + 40;
    base_width = _JIG_PIVOT_PLATE_DIA + 40;
    
    difference() {
        union() {
            // Main base plate
            translate([-base_length/2, -base_width/2, 0])
                cube([base_length, base_width, JIG_BASE_THICK]);
            
            // DOM tube cradle (V-shaped supports at ends)
            // Left cradle
            translate([-_JIG_PIVOT_WIDTH/2 - 10, 0, JIG_BASE_THICK])
                dom_cradle_support();
            
            // Right cradle  
            translate([_JIG_PIVOT_WIDTH/2 + 10, 0, JIG_BASE_THICK])
                dom_cradle_support();
        }
        
        // Cutout center for weight reduction and ventilation
        translate([0, 0, -1])
            cylinder(d=_JIG_PIVOT_PLATE_DIA - 60, h=JIG_BASE_THICK + 2, $fn=48);
    }
}

// =============================================================================
// DOM CRADLE SUPPORT MODULE
// =============================================================================

module dom_cradle_support() {
    // V-shaped cradle to hold DOM tube at correct height
    // DOM center should be at plate center height
    
    cradle_height = DOM_PIPE_OD/2 + DOM_CRADLE_DEPTH;
    cradle_width = DOM_PIPE_OD + JIG_WALL_THICKNESS * 2;
    
    difference() {
        // Outer block
        translate([-15, -cradle_width/2, 0])
            cube([30, cradle_width, cradle_height]);
        
        // DOM tube cutout (with clearance)
        translate([0, 0, cradle_height])
            rotate([0, 90, 0])
            cylinder(d=DOM_PIPE_OD + JIG_CLEARANCE*2, h=50, center=true, $fn=48);
        
        // V-notch for easier tube placement
        translate([0, 0, cradle_height + 10])
            rotate([0, 90, 0])
            cylinder(d=DOM_PIPE_OD * 1.5, h=50, center=true, $fn=48);
    }
}

// =============================================================================
// PLATE ALIGNMENT RING MODULE
// =============================================================================

module plate_alignment_ring(inner=false) {
    // Ring that fits around the circular plate edge
    // Holds plate perpendicular and centered
    // Has gaps for welding access
    
    ring_id = _JIG_PIVOT_PLATE_DIA + JIG_CLEARANCE * 2;
    ring_od = ring_id + JIG_WALL_THICKNESS * 2 + 10;
    ring_height = _JIG_PIVOT_PLATE_THICK + JIG_SUPPORT_HEIGHT;
    
    // Support ledge for plate to sit on
    ledge_height = JIG_SUPPORT_HEIGHT;
    ledge_width = 8;
    
    difference() {
        union() {
            // Main ring
            difference() {
                cylinder(d=ring_od, h=ring_height, $fn=64);
                translate([0, 0, -1])
                    cylinder(d=ring_id, h=ring_height + 2, $fn=64);
            }
            
            // Inner support ledge (plate sits on this)
            translate([0, 0, 0])
            difference() {
                cylinder(d=ring_id, h=ledge_height, $fn=64);
                translate([0, 0, -1])
                    cylinder(d=ring_id - ledge_width*2, h=ledge_height + 2, $fn=64);
            }
        }
        
        // Welding access gaps (4 equally spaced)
        for (i = [0:3]) {
            angle = i * 90 + 45;
            rotate([0, 0, angle])
            translate([0, 0, ledge_height])
                cube([ring_od + 2, WELD_ACCESS_GAP, ring_height], center=true);
        }
        
        // Center hole for DOM tube (with clearance)
        translate([0, 0, -1])
            cylinder(d=DOM_PIPE_OD + JIG_CLEARANCE * 4, h=ring_height + 2, $fn=48);
        
        // Bolt hole clearances (so jig doesn't interfere with bolt holes)
        for (i = [0:5]) {
            angle = i * 60;
            bx = (_JIG_PIVOT_BOLT_CIRCLE_DIA / 2) * cos(angle);
            by = (_JIG_PIVOT_BOLT_CIRCLE_DIA / 2) * sin(angle);
            translate([bx, by, -1])
                cylinder(d=BOLT_DIA_1_2 + 5, h=ring_height + 2, $fn=24);
        }
    }
}

// =============================================================================
// COMPLETE JIG ASSEMBLY MODULE
// =============================================================================

module pivot_mount_welding_jig_assembly(show_parts=true) {
    // Complete jig showing all components
    // Set show_parts=true to see the pivot mount parts in position
    
    color("Orange", 0.8) {
        // Base with DOM cradles
        jig_base_plate();
        
        // Left plate alignment ring
        translate([-_JIG_PIVOT_WIDTH/2 - _JIG_PIVOT_PLATE_THICK, 0, JIG_BASE_THICK])
        rotate([0, 90, 0])
            plate_alignment_ring();
        
        // Right plate alignment ring
        translate([_JIG_PIVOT_WIDTH/2, 0, JIG_BASE_THICK])
        rotate([0, 90, 0])
            plate_alignment_ring();
    }
    
    // Show the parts in position
    if (show_parts) {
        // DOM tube cradle height
        dom_z = JIG_BASE_THICK + DOM_PIPE_OD/2 + DOM_CRADLE_DEPTH;
        
        color("SteelBlue", 0.6) {
            // DOM tube
            translate([0, 0, dom_z])
            rotate([0, 90, 0])
            difference() {
                cylinder(d=DOM_PIPE_OD, h=_JIG_PIVOT_WIDTH, center=true, $fn=48);
                cylinder(d=DOM_PIPE_ID, h=_JIG_PIVOT_WIDTH + 2, center=true, $fn=48);
            }
            
            // Left plate
            translate([-_JIG_PIVOT_WIDTH/2, 0, dom_z])
            rotate([0, 90, 0])
            difference() {
                cylinder(d=_JIG_PIVOT_PLATE_DIA, h=_JIG_PIVOT_PLATE_THICK, $fn=64);
                cylinder(d=DOM_PIPE_OD, h=_JIG_PIVOT_PLATE_THICK + 1, $fn=48);
            }
            
            // Right plate
            translate([_JIG_PIVOT_WIDTH/2 - _JIG_PIVOT_PLATE_THICK, 0, dom_z])
            rotate([0, 90, 0])
            difference() {
                cylinder(d=_JIG_PIVOT_PLATE_DIA, h=_JIG_PIVOT_PLATE_THICK, $fn=64);
                cylinder(d=DOM_PIPE_OD, h=_JIG_PIVOT_PLATE_THICK + 1, $fn=48);
            }
        }
    }
}

// =============================================================================
// INDIVIDUAL PRINTABLE COMPONENTS
// =============================================================================

module jig_base_printable() {
    // Base plate oriented for printing (flat on bed)
    jig_base_plate();
}

module jig_ring_printable() {
    // Alignment ring oriented for printing
    // Printed standing up for strength
    plate_alignment_ring();
}

// =============================================================================
// EXPLODED VIEW FOR DOCUMENTATION
// =============================================================================

module pivot_mount_welding_jig_exploded() {
    explode = 80;
    
    color("Orange", 0.8) {
        // Base
        jig_base_plate();
        
        // Left ring (exploded)
        translate([-_JIG_PIVOT_WIDTH/2 - _JIG_PIVOT_PLATE_THICK - explode, 0, JIG_BASE_THICK])
        rotate([0, 90, 0])
            plate_alignment_ring();
        
        // Right ring (exploded)
        translate([_JIG_PIVOT_WIDTH/2 + explode, 0, JIG_BASE_THICK])
        rotate([0, 90, 0])
            plate_alignment_ring();
    }
}

// =============================================================================
// RENDER PREVIEW
// =============================================================================

$fn = 32;

echo("=== PIVOT MOUNT WELDING JIG SPECIFICATIONS ===");
echo("Jig Base Thickness:", JIG_BASE_THICK, "mm");
echo("Support Ring Height:", JIG_SUPPORT_HEIGHT, "mm");
echo("Wall Thickness:", JIG_WALL_THICKNESS, "mm");
echo("DOM Cradle Depth:", DOM_CRADLE_DEPTH, "mm");
echo("Weld Access Gap:", WELD_ACCESS_GAP, "mm");
echo("");
echo("Parts to print:");
echo("  1x Base plate");
echo("  2x Alignment rings");

// Show complete assembly with parts
pivot_mount_welding_jig_assembly(show_parts=true);

// Show individual printable parts below
translate([0, -250, 0]) {
    translate([-100, 0, 0])
        jig_base_printable();
    
    translate([100, 0, 0])
        jig_ring_printable();
    
    translate([100, 100, 0])
        jig_ring_printable();
}
