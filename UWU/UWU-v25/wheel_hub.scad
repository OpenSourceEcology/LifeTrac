// wheel_hub.scad
// UWU Simplified Wheel Hub Assembly
// Open Source Ecology - LifeTrac v25 / UWU-v25
//
// Fabricated wheel hub that mounts on the UWU shaft and accepts
// standard Bobcat skid-steer wheels (P/N 7232567, 8-lug on 8" BCD).
//
// Construction:
//   1. DOM (Drawn Over Mandrel) tube - slides over the 1.25" shaft
//   2. Triangular plate steel gussets - welded to DOM and lug plate
//   3. Circular lug bolt plate - 8 holes matching Bobcat pattern
//   4. Cross-drilled hole through DOM + shaft for retention bolt
//
// Units: mm (all dimensions converted from inches)

$fn = 48;
inch = 25.4;

// =============================================================================
// PARAMETERS (defaults match lifetrac_v25_params.scad)
// =============================================================================

// DOM tube
_hub_shaft_diam = 31.75;         // 1.25"
_hub_dom_od     = 50.8;          // 2.00" OD
_hub_dom_wall   = 6.35;          // 0.25" wall
_hub_dom_id     = _hub_shaft_diam + 0.79375;  // Shaft OD + 1/32" clearance
_hub_dom_length = 101.6;         // 4.00" long

// Cross-drill retention bolt
_hub_cross_bolt_diam = 12.7;     // 1/2" bolt
_hub_cross_bolt_pos  = _hub_dom_length / 2;  // centered along DOM

// Lug bolt plate (circular)
_hub_plate_diam      = 228.6;    // 9.00" diameter
_hub_plate_thickness = 12.7;     // 1/2" plate

// Bobcat bolt pattern
_bobcat_lug_count = 8;
_bobcat_bcd       = 203.2;       // 8.00" bolt circle
_bobcat_lug_hole  = 15.875;      // 5/8" clearance holes
_bobcat_center_bore = 76.2;      // 3.00" center bore (rim side)
// Hub plate center bore: DOM OD + 1/16" clearance
_hub_plate_center_bore = _hub_dom_od + 1.5875;

// Gusset plates
_hub_gusset_count     = 4;
_hub_gusset_thickness = 6.35;    // 1/4" plate
_hub_gusset_height    = _hub_dom_length * 0.85;
// Offset 22.5° so gussets sit between the 8 lug bolt holes
_hub_gusset_angle_offset = 22.5;

// =============================================================================
// DOM TUBE MODULE
// =============================================================================
// Drawn Over Mandrel steel tube that slides over the shaft.
// Cross-drilled for retention bolt.

module hub_dom_tube(
    od     = _hub_dom_od,
    id     = _hub_dom_id,
    length = _hub_dom_length,
    cross_bolt_diam = _hub_cross_bolt_diam,
    cross_bolt_pos  = _hub_cross_bolt_pos
) {
    color("DimGray")
    difference() {
        // Tube body
        cylinder(d=od, h=length, center=false);

        // Bore
        translate([0, 0, -1])
        cylinder(d=id, h=length + 2, center=false);

        // Cross-drill hole (perpendicular, centered between gussets)
        translate([0, 0, cross_bolt_pos])
        rotate([0, 0, 67.5])
        rotate([0, 90, 0])
        cylinder(d=cross_bolt_diam, h=od + 2, center=true);
    }
}

// =============================================================================
// LUG BOLT PLATE MODULE
// =============================================================================
// Circular plate with Bobcat 8-lug pattern.
// Welded to the gussets/DOM tube at one end.

module hub_lug_plate(
    plate_diam     = _hub_plate_diam,
    plate_thick    = _hub_plate_thickness,
    lug_count      = _bobcat_lug_count,
    bcd            = _bobcat_bcd,
    lug_hole_diam  = _bobcat_lug_hole,
    center_bore    = _hub_plate_center_bore
) {
    color("Silver")
    difference() {
        // Main circular plate
        cylinder(d=plate_diam, h=plate_thick, center=false);

        // Center bore
        translate([0, 0, -1])
        cylinder(d=center_bore, h=plate_thick + 2, center=false);

        // Lug bolt holes (evenly spaced on BCD)
        for (i = [0 : lug_count - 1]) {
            rotate([0, 0, i * (360 / lug_count)])
            translate([bcd / 2, 0, -1])
            cylinder(d=lug_hole_diam, h=plate_thick + 2, $fn=24);
        }
    }
}

// =============================================================================
// TRIANGULAR GUSSET PLATE MODULE
// =============================================================================
// Single triangular gusset that connects the DOM tube to the lug plate.
// Radiates outward from the DOM OD to the lug plate edge.

module hub_gusset(
    dom_od         = _hub_dom_od,
    plate_diam     = _hub_plate_diam,
    gusset_height  = _hub_gusset_height,
    gusset_thick   = _hub_gusset_thickness
) {
    // Triangle profile:
    //   - Inner edge at DOM tube OD/2
    //   - Outer edge at lug plate radius
    //   - Height along the Z axis (DOM axis)
    inner_r = dom_od / 2;
    outer_r = plate_diam / 2 - 5;  // Slight inset from plate edge
    radial_length = outer_r - inner_r;

    color("DarkGray")
    translate([inner_r, 0, 0])
    rotate([-90, 0, 0])
    linear_extrude(height=gusset_thick, center=true)
    // Triangle in radial-axial plane: base at plate, apex up DOM
    polygon(points=[
        [0, 0],                        // Inner at plate level
        [radial_length, 0],            // Outer at plate level
        [0, gusset_height]             // Inner up DOM axis
    ]);
}

// =============================================================================
// COMPLETE WHEEL HUB ASSEMBLY
// =============================================================================
// Full hub: DOM tube + gussets + lug plate
// Origin: center of shaft bore at the outboard end
// Z axis = shaft axis
// DOM tube extends in -Z direction (inboard, over the shaft)
// Lug plate face is at Z=0, plate body extends in +Z direction (outboard)
// Rim/tire mount outboard of the lug plate

module wheel_hub_assembly(
    dom_od         = _hub_dom_od,
    dom_id         = _hub_dom_id,
    dom_length     = _hub_dom_length,
    cross_bolt_diam = _hub_cross_bolt_diam,
    cross_bolt_pos  = _hub_cross_bolt_pos,
    plate_diam     = _hub_plate_diam,
    plate_thick    = _hub_plate_thickness,
    lug_count      = _bobcat_lug_count,
    bcd            = _bobcat_bcd,
    lug_hole_diam  = _bobcat_lug_hole,
    center_bore    = _hub_plate_center_bore,
    gusset_count   = _hub_gusset_count,
    gusset_thick   = _hub_gusset_thickness,
    gusset_height  = _hub_gusset_height
) {
    // DOM tube extending inboard (-Z), sliding over the shaft
    // and 1" past the outer face of the bolt plate (+Z)
    _dom_total = dom_length + plate_thick + 25.4;
    translate([0, 0, -dom_length])
    hub_dom_tube(dom_od, dom_id, _dom_total, cross_bolt_diam, cross_bolt_pos);

    // Lug plate at Z=0, body extends in +Z (outboard toward rim)
    hub_lug_plate(plate_diam, plate_thick, lug_count, bcd,
                  lug_hole_diam, center_bore);

    // Gusset plates: base flush with lug plate face (Z=0), apex inboard along DOM (-Z)
    // Offset 22.5° to avoid the 8 Bobcat lug bolt holes
    for (i = [0 : gusset_count - 1]) {
        rotate([0, 0, _hub_gusset_angle_offset + i * (360 / gusset_count)])
        translate([dom_od / 2, 0, 0])
        rotate([-90, 0, 0])
        linear_extrude(height=gusset_thick, center=true)
        polygon(points=[
            [0, 0],
            [(plate_diam / 2 - dom_od / 2 - 5), 0],
            [0, gusset_height]
        ]);
    }

    // Cross-bolt visualization
    color("DarkRed")
    translate([0, 0, -dom_length + cross_bolt_pos])
    rotate([0, 0, 67.5])
    rotate([0, 90, 0])
    cylinder(d=cross_bolt_diam * 0.8, h=dom_od * 1.3, center=true, $fn=16);
}

// =============================================================================
// BOBCAT RIM MODULE
// =============================================================================
// Simplified representation of Bobcat P/N 7232567 rim
// 16.5" x 9.75", 8-lug on 8" BCD

module bobcat_rim(
    rim_diam    = 419.1,     // 16.5"
    rim_width   = 254.0,     // 10.0"
    rim_offset  = 93.98,     // 3.70" offset (backspacing)
    center_bore = _bobcat_center_bore,
    lug_count   = _bobcat_lug_count,
    bcd         = _bobcat_bcd,
    lug_hole    = _bobcat_lug_hole
) {
    // Simplified rim: plain hollow cylinder + hub disc with bolt holes
    inboard_end = -rim_offset;

    color("DarkOrange", 0.2) {
        // Barrel
        translate([0, 0, inboard_end])
        difference() {
            cylinder(d=rim_diam, h=rim_width, $fn=24);
            // Hollow interior
            translate([0, 0, -1])
            cylinder(d=rim_diam - 20, h=rim_width + 2, $fn=24);
        }

        // Hub disc - inboard face 3.7" from inboard rim edge
        difference() {
            cylinder(d=rim_diam - 20, h=6.35, center=false, $fn=24);
            // Center bore
            cylinder(d=152.4, h=20, center=true, $fn=48);
            // Lug bolt holes
            for (i = [0 : lug_count - 1]) {
                rotate([0, 0, i * (360 / lug_count)])
                translate([bcd / 2, 0, 0])
                cylinder(d=lug_hole, h=20, center=true, $fn=24);
            }
        }
    }
}

// =============================================================================
// SKID STEER TIRE MODULE
// =============================================================================
// 12-16.5 tire profile for Bobcat rim

module skid_steer_tire(
    tire_od     = 851.0,     // ~33.5" overall diameter
    tire_width  = 254.0,     // 10.0" overall width
    rim_diam    = 419.1,     // 16.5" rim diameter
    rim_width   = 254.0,     // 10.0" rim width
    rim_offset  = 93.98      // 3.70" offset
) {
    section_height = (tire_od - rim_diam) / 2;
    tire_radius = tire_od / 2;
    rim_radius = rim_diam / 2;

    // Tire centered on rim mounting face (Z=0)
    inboard_end = -rim_offset;
    outboard_end = rim_width - rim_offset;
    tire_center_z = (inboard_end + outboard_end) / 2;

    color("Black", 0.2)
    translate([0, 0, tire_center_z])
    rotate_extrude($fn=24) {
        // Cylindrical tire with slight edge rounding
        hull() {
            translate([tire_od/2 - 10, tire_width/2 - 10])
            circle(r=10, $fn=24);
            translate([tire_od/2 - 10, -tire_width/2 + 10])
            circle(r=10, $fn=24);
            translate([rim_diam/2 + 5, tire_width/2 - 10])
            circle(r=5, $fn=24);
            translate([rim_diam/2 + 5, -tire_width/2 + 10])
            circle(r=5, $fn=24);
        }
    }
}

// =============================================================================
// COMPLETE WHEEL ASSEMBLY (HUB + RIM + TIRE)
// =============================================================================
// Full assembly: hub bolted to rim with tire mounted
// Origin: center of shaft bore at the hub lug plate face
// Z axis = shaft/axle axis

module bobcat_wheel_assembly(show_hub=true, show_rim=true, show_tire=true) {
    // Hub: DOM extends -Z (inboard over shaft), lug plate at Z=0 facing +Z
    if (show_hub)
        wheel_hub_assembly();

    // Rim mounts outboard of lug plate (at Z = plate thickness)
    if (show_rim)
        translate([0, 0, _hub_plate_thickness])
        bobcat_rim();

    // Tire on rim
    if (show_tire)
        translate([0, 0, _hub_plate_thickness])
        skid_steer_tire();
}

// =============================================================================
// STANDALONE PREVIEW
// =============================================================================

if ($preview) {
    // Full assembly
    bobcat_wheel_assembly();

    // Shaft reference (transparent)
    %translate([0, 0, -50])
    cylinder(d=_hub_shaft_diam, h=200, $fn=32);

    // Exploded view of components
    translate([400, 0, 0]) {
        // DOM tube
        translate([0, 0, 80])
        hub_dom_tube();

        // Lug plate
        translate([0, 0, -20])
        hub_lug_plate();

        // Single gusset
        translate([0, 0, 30])
        hub_gusset();

        // Rim
        translate([0, 0, -120])
        bobcat_rim();

        // Tire
        translate([0, 0, -120])
        skid_steer_tire();
    }
}
