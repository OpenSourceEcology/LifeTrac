// fasteners.scad
// Hex bolts, nuts, and washers
// Part of LifeTrac v25 OpenSCAD design

// Global parameters
$fn = 6; // Hexagon has 6 sides

// Standard bolt sizes (metric conversions of imperial)
BOLT_1_2_INCH = 12.7;  // 1/2" bolt diameter
BOLT_1_INCH = 25.4;    // 1" bolt diameter

// Hex dimensions (across flats)
HEX_1_2_INCH = 19.05;  // 3/4" hex for 1/2" bolt
HEX_1_INCH = 38.1;     // 1.5" hex for 1" bolt

/**
 * Creates a hex bolt
 * @param diameter Shaft diameter in mm
 * @param length Shaft length in mm
 * @param head_height Height of hex head in mm
 * @param head_size Hex head size (across flats) in mm
 * @param threaded Display threads (cosmetic only)
 */
module hex_bolt(diameter=BOLT_1_2_INCH, length=50, 
                head_height=8, head_size=HEX_1_2_INCH,
                threaded=false) {
    // Hex head
    cylinder(d=head_size/cos(30), h=head_height, $fn=6);
    
    // Shaft
    translate([0, 0, -length])
    if (threaded) {
        // Simplified thread representation
        for(i = [0:3:length]) {
            translate([0, 0, i])
                cylinder(d=diameter*1.05, h=1.5);
            translate([0, 0, i+1.5])
                cylinder(d=diameter*0.95, h=1.5);
        }
    } else {
        cylinder(d=diameter, h=length);
    }
}

/**
 * Creates a hex nut
 * @param diameter Hole diameter in mm
 * @param height Nut height in mm
 * @param size Hex size (across flats) in mm
 */
module hex_nut(diameter=BOLT_1_2_INCH, height=10, size=HEX_1_2_INCH) {
    difference() {
        cylinder(d=size/cos(30), h=height, $fn=6);
        translate([0, 0, -1])
            cylinder(d=diameter, h=height+2, $fn=32);
    }
}

/**
 * Creates a washer
 * @param inner_diameter Inner diameter (bolt size) in mm
 * @param outer_diameter Outer diameter in mm
 * @param thickness Washer thickness in mm
 */
module washer(inner_diameter=BOLT_1_2_INCH, outer_diameter=25, thickness=2) {
    difference() {
        cylinder(d=outer_diameter, h=thickness, $fn=32);
        translate([0, 0, -1])
            cylinder(d=inner_diameter, h=thickness+2, $fn=32);
    }
}

/**
 * Creates a complete bolt assembly (bolt + washer + nut)
 * @param diameter Bolt diameter in mm
 * @param length Total length through material in mm
 * @param head_size Hex head size in mm
 * @param nut_size Hex nut size in mm
 */
module bolt_assembly(diameter=BOLT_1_2_INCH, length=50,
                    head_size=HEX_1_2_INCH, nut_size=HEX_1_2_INCH) {
    // Bolt
    hex_bolt(diameter, length);
    
    // Washer under head
    translate([0, 0, -0.1])
        washer(diameter, diameter*2, 2);
    
    // Nut at end
    translate([0, 0, -length])
        hex_nut(diameter, 10, nut_size);
    
    // Washer under nut
    translate([0, 0, -length-10])
        washer(diameter, diameter*2, 2);
}

/**
 * Creates a bolt hole (for difference operations)
 * @param diameter Hole diameter (slightly larger than bolt)
 * @param depth Hole depth in mm
 * @param countersink Add countersink for bolt head if true
 * @param countersink_diameter Diameter of countersink
 * @param countersink_depth Depth of countersink
 */
module bolt_hole(diameter=BOLT_1_2_INCH, depth=50, 
                countersink=false, countersink_diameter=25, 
                countersink_depth=10) {
    // Main hole (add clearance)
    cylinder(d=diameter*1.1, h=depth, $fn=32);
    
    // Optional countersink for bolt head
    if (countersink) {
        translate([0, 0, depth-countersink_depth])
            cylinder(d=countersink_diameter, h=countersink_depth+1, $fn=32);
    }
}

/**
 * Creates a pattern of bolt holes
 * @param positions Array of [x,y] positions
 * @param diameter Hole diameter
 * @param depth Hole depth
 */
module bolt_hole_pattern(positions, diameter=BOLT_1_2_INCH, depth=50) {
    for (pos = positions) {
        translate([pos[0], pos[1], 0])
            bolt_hole(diameter, depth);
    }
}

/**
 * Creates a grid of bolt holes
 * @param rows Number of rows
 * @param cols Number of columns
 * @param spacing Spacing between holes
 * @param diameter Hole diameter
 * @param depth Hole depth
 */
module bolt_hole_grid(rows, cols, spacing, diameter=BOLT_1_2_INCH, depth=50) {
    for (i = [0:rows-1]) {
        for (j = [0:cols-1]) {
            translate([j*spacing, i*spacing, 0])
                bolt_hole(diameter, depth);
        }
    }
}

// Example usage and visualization
if ($preview) {
    // 1/2" bolt assembly
    color("Silver")
    bolt_assembly(BOLT_1_2_INCH, 50, HEX_1_2_INCH, HEX_1_2_INCH);
    
    // 1" bolt assembly
    translate([50, 0, 0])
    color("Silver")
    bolt_assembly(BOLT_1_INCH, 75, HEX_1_INCH, HEX_1_INCH);
    
    // Hex nut
    translate([120, 0, 0])
    color("DimGray")
    hex_nut(BOLT_1_2_INCH);
    
    // Washer
    translate([150, 0, 0])
    color("Silver")
    washer(BOLT_1_2_INCH);
    
    // Demonstration of bolt hole pattern
    translate([0, 50, 0])
    difference() {
        color("DarkSlateGray")
        cube([100, 100, 10]);
        
        translate([0, 0, -1])
        bolt_hole_grid(3, 3, 40, BOLT_1_2_INCH, 12);
    }
}
