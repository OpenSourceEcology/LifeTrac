// UWU-v25 Universal Wheel Unit Assembly
// Units: mm (converted from inches)

$fn = 50;

inch = 25.4;

// --- Parametric Dimensions ---
plate_w = 8 * inch;
plate_h = 8 * inch;
plate_t = 0.25 * inch;

plate_spacing_1_2 = 3 * inch; // Spacing between motor plate and first bearing plate
plate_spacing_2_3 = 4 * inch; // Spacing between bearing plates

// --- Updated from QC Wheel Diagram ---
shaft_diam = 1.25 * inch; 
center_hole_diam = 2.00 * inch; // From drawing
bolt_circle_diam = 6.19 * inch; // From drawing
bolt_hole_diam = 0.5 * inch;    // Estimated from drawing

motor_mount_radius = 4.125 * inch;
motor_mount_bcd = motor_mount_radius * 2;

shaft_length = 10 * inch;

bearing_flange_size = 4 * inch;
bearing_housing_diam = 2.5 * inch;

motor_body_diam = 6 * inch;
motor_body_len = 5 * inch;
motor_shaft_len = 1.5 * inch;

coupling_diam = 2 * inch;
coupling_len = 2 * inch;

// --- Modules ---

module plate(is_motor_mount=false) {
    difference() {
        color("Silver") 
        cube([plate_w, plate_h, plate_t], center=true);
        
        // Center hole (From Drawing: Ø2.00)
        translate([0,0,0]) cylinder(h=plate_t*2, d=center_hole_diam, center=true); 

        // Bolt Pattern
        // Motor mount uses specialized BCD (8.25"), Bearings use standard (6.19")
        // Uses 45 degree orientation to fit 8.25" BCD on 8"x8" plate
        current_bcd = is_motor_mount ? motor_mount_bcd : bolt_circle_diam;

        for(r = [45, 135, 225, 315]) 
            rotate([0,0,r]) translate([current_bcd/2, 0, 0])
            cylinder(h=plate_t*2, d=bolt_hole_diam, center=true);
    }
}

module bearing_4bolt() {
    color("Green") {
        difference() {
            union() {
                // Flange - adjusted to match mounting pattern
                hull() {
                    for(r = [45, 135, 225, 315])
                        rotate([0,0,r]) translate([bolt_circle_diam/2, 0, 0])
                        cylinder(h=0.5*inch, d=1*inch);
                    cylinder(h=0.5*inch, d=bearing_housing_diam);
                }
                // Housing
                translate([0,0,0.25*inch])
                cylinder(h=0.75*inch, d=bearing_housing_diam);
            }
            // Shaft hole
            translate([0,0,-0.1])
            cylinder(h=3*inch, d=shaft_diam);

            // Bolt holes matching the plate
            for(r = [45, 135, 225, 315])
                rotate([0,0,r]) translate([bolt_circle_diam/2, 0, 0])
                cylinder(h=2*inch, d=bolt_hole_diam, center=true);
        }
    }
}

module hydraulic_motor() {
    color("Orange") {
        // Body
        translate([0,0, -motor_body_len])
        cylinder(h=motor_body_len, d=motor_body_diam);
        
        // Flange
        difference() {
            translate([0,0, -0.25*inch])
            cube([7.5*inch, 7.5*inch, 0.5*inch], center=true);

            // Bolt holes matching the plate (4.125" radius / 8.25" BCD)
            for(r = [45, 135, 225, 315]) 
                rotate([0,0,r]) translate([motor_mount_bcd/2, 0, -0.25*inch])
                cylinder(h=1*inch, d=bolt_hole_diam, center=true);
        }
        
        // Shaft
        cylinder(h=motor_shaft_len, d=1*inch);
    }
}

module shaft_coupling() {
    color("DimGray")
    cylinder(h=coupling_len, d=coupling_diam);
}

module main_shaft() {
    color("Silver")
    cylinder(h=shaft_length, d=shaft_diam);
}

// --- Assembly ---

// Plate 1: Motor Mount
translate([0,0,0]) {
    plate(is_motor_mount=true);
    
    // Motor attached to back of plate (Shaft points +Z through plate)
    translate([0,0, -plate_t/2])
    hydraulic_motor();
}

// Plate 2: Bearing Unit 1
translate([0,0, plate_spacing_1_2]) {
    plate(is_motor_mount=false);
    
    // Bearing attached to front face
    translate([0,0, plate_t/2])
    bearing_4bolt();
}

// Plate 3: Bearing Unit 2
translate([0,0, plate_spacing_1_2 + plate_spacing_2_3]) {
    plate(is_motor_mount=false);
    
    // Bearing attached to front face
    translate([0,0, plate_t/2])
    bearing_4bolt();
}

// Shaft Coupling (between Motor and Plate 2)
translate([0,0, plate_t/2 + 0.5*inch]) // Offset from motor plate
    shaft_coupling();

// Main Shaft (Starting inside coupling, going through bearings)
translate([0,0, plate_t/2 + coupling_len/2])
    main_shaft();

