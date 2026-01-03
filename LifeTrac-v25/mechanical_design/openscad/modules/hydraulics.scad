// hydraulics.scad
// Hydraulic cylinder and motor placeholders
// Part of LifeTrac v25 OpenSCAD design

// Global parameters
$fn = 32;

/**
 * Creates a detailed hydraulic cylinder
 * @param bore_diameter Cylinder bore diameter in mm
 * @param rod_diameter Rod diameter in mm
 * @param stroke Stroke length in mm
 * @param retracted Show in retracted position if true
 * @param extension Current extension (0 to stroke) in mm
 * @param mounting_type Type of mounting ("clevis", "eye", "trunnion")
 * @param pin_to_pin_length If > 0, overrides extension/retracted to match this total length
 */
module hydraulic_cylinder(bore_diameter=63.5, rod_diameter=31.75, 
                         stroke=300, retracted=true, extension=0,
                         mounting_type="clevis", pin_to_pin_length=0) {
    
    // Dimensions derived from bore/rod
    B = bore_diameter;
    R = rod_diameter;
    
    // Component Lengths
    base_clevis_len = B * 0.8;
    piston_len = B * 0.4;
    head_internal = B * 0.4;
    head_external = B * 0.2;
    rod_clevis_len = R * 1.5;
    clearance = 5;
    
    // Calculated Lengths
    tube_len = stroke + piston_len + head_internal + clearance;
    
    // Calculate retracted length (pin-to-pin)
    // Sum of stack at 0 extension:
    // Base Pin (0) -> Tube Start (base_clevis_len) -> Head Top (base_clevis_len + tube_len + head_external)
    // -> Rod Clevis Pin (Head Top + 5mm gap + rod_clevis_len)
    calc_retracted_len = base_clevis_len + tube_len + head_external + 5 + rod_clevis_len;
    
    // Determine actual extension
    target_extension = (pin_to_pin_length > 0) 
        ? pin_to_pin_length - calc_retracted_len 
        : (retracted ? 0 : extension);
        
    actual_extension = max(0, min(stroke, target_extension));
    
    // --- Rendering ---
    
    // 1. Base Clevis (Pin at 0,0,0)
    cylinder_base_clevis(B, base_clevis_len);
    
    // 2. Tube
    translate([0, 0, base_clevis_len])
        cylinder_tube(B * 1.2, tube_len);
        
    // 3. Head/Gland
    translate([0, 0, base_clevis_len + tube_len])
        cylinder_head_gland(B * 1.2, R, head_external);
        
    // 4. Piston & Rod Group
    // Piston bottom starts at base_clevis_len + clearance + actual_extension
    piston_z = base_clevis_len + clearance + actual_extension;
    
    translate([0, 0, piston_z]) {
        cylinder_piston(B, piston_len);
        
        // Rod
        // Rod length needs to go from piston to rod clevis
        // Rod length is constant regardless of extension
        rod_len = stroke + head_internal + head_external + 5;
        
        translate([0, 0, piston_len])
            color("Silver") cylinder(d=R, h=rod_len);
            
        // 5. Rod Clevis
        translate([0, 0, piston_len + rod_len])
            cylinder_rod_clevis(R, rod_clevis_len);
    }
}

// --- Helper Modules ---

module cylinder_base_clevis(bore, length) {
    color("DimGray") {
        // Base cap connection to tube
        translate([0,0,length*0.6]) cylinder(d=bore*1.2, h=length*0.4);
        
        // Clevis ears
        difference() {
            union() {
                // Block for ears
                translate([-bore*0.6, -bore*0.6, 0]) cube([bore*1.2, bore*1.2, length*0.6]);
                // Rounded bottom
                translate([0, 0, 0]) rotate([0,90,0]) cylinder(d=bore*1.2, h=bore*1.2, center=true);
            }
            // Pin hole
            rotate([0,90,0]) cylinder(d=bore*0.4, h=bore*2, center=true);
            // Gap for mounting bracket
            cube([bore*0.4, bore*2, bore*2], center=true);
        }
    }
}

module cylinder_tube(od, length) {
    color("Orange")
        cylinder(d=od, h=length);
}

module cylinder_head_gland(od, rod_d, length) {
    color("Black") {
        cylinder(d=od, h=length);
        // Wiper seal area
        translate([0,0,length]) cylinder(d=od*0.8, h=length*0.5); 
    }
}

module cylinder_piston(bore, thickness) {
    color("Silver")
        cylinder(d=bore-0.5, h=thickness); 
}

module cylinder_rod_clevis(rod_d, length) {
    color("DimGray") {
        // Threaded part
        cylinder(d=rod_d*1.2, h=length*0.4);
        
        // Clevis part
        translate([0,0,length*0.4]) {
             difference() {
                union() {
                    // Block
                    translate([-rod_d*0.8, -rod_d*0.8, 0]) cube([rod_d*1.6, rod_d*1.6, length*0.6]);
                    // Rounded top
                    translate([0, 0, length*0.6]) rotate([0,90,0]) cylinder(d=rod_d*1.6, h=rod_d*1.6, center=true);
                }
                // Pin hole
                translate([0, 0, length*0.6]) rotate([0,90,0]) cylinder(d=rod_d*0.5, h=rod_d*2, center=true);
                // Gap
                translate([0, 0, length*0.6]) cube([rod_d*0.6, rod_d*2, rod_d*2], center=true);
            }
        }
    }
}

/**
 * Creates a hydraulic motor (simplified representation)
 * @param displacement Displacement in cc/rev (affects size)
 * @param shaft_diameter Output shaft diameter in mm
 * @param shaft_length Output shaft length in mm
 * @param mounting_flange Show mounting flange if true
 */
module hydraulic_motor(displacement=100, shaft_diameter=25.4, 
                       shaft_length=100, mounting_flange=true) {
    
    // Calculate body size based on displacement
    body_diameter = sqrt(displacement * 4);
    body_length = body_diameter * 1.5;
    
    // Motor body
    color("DarkSlateBlue")
    cylinder(d=body_diameter, h=body_length);
    
    // Output shaft
    color("Silver")
    translate([0, 0, body_length])
    cylinder(d=shaft_diameter, h=shaft_length);
    
    // Mounting flange
    if (mounting_flange) {
        color("Gray")
        translate([0, 0, -10])
        difference() {
            cylinder(d=body_diameter*1.5, h=10);
            // Mounting holes
            for (angle = [0:90:270]) {
                rotate([0, 0, angle])
                translate([body_diameter*0.6, 0, -1])
                    cylinder(d=13, h=12, $fn=32);
            }
        }
    }
    
    // Port connections
    color("Black")
    for (angle = [45, 135]) {
        rotate([0, 0, angle])
        translate([body_diameter*0.4, 0, body_length*0.5])
        rotate([0, 90, 0])
            cylinder(d=19, h=30);
    }
}

/**
 * Creates a hydraulic pump (similar to motor but includes reservoir)
 * @param displacement Displacement in cc/rev
 * @param drive_shaft_diameter Drive shaft diameter in mm
 */
module hydraulic_pump(displacement=100, drive_shaft_diameter=25.4) {
    // Main pump body
    color("DarkRed")
    cylinder(d=sqrt(displacement * 5), h=150);
    
    // Drive shaft
    color("Silver")
    translate([0, 0, -50])
    cylinder(d=drive_shaft_diameter, h=50);
    
    // Reservoir
    color("Gray", 0.3)
    translate([sqrt(displacement * 5)*0.7, 0, 50])
    cylinder(d=100, h=200);
}

/**
 * Creates a hydraulic hose (simplified)
 * @param start_point [x, y, z] starting position
 * @param end_point [x, y, z] ending position
 * @param diameter Hose outer diameter in mm
 * @param fittings Show end fittings if true
 */
module hydraulic_hose(start_point, end_point, diameter=19, fittings=true) {
    // This is a simplified straight representation
    // In reality, hoses curve and flex
    
    vector = end_point - start_point;
    length = norm(vector);
    
    translate(start_point)
    rotate([0, atan2(sqrt(vector[0]*vector[0] + vector[1]*vector[1]), vector[2]), 
            atan2(vector[1], vector[0])])
    {
        // Hose
        color("Black")
        cylinder(d=diameter, h=length);
        
        if (fittings) {
            // Start fitting
            color("Silver")
            cylinder(d=diameter*1.5, h=25);
            
            // End fitting
            translate([0, 0, length-25])
            color("Silver")
            cylinder(d=diameter*1.5, h=25);
        }
    }
}

/**
 * Creates a hydraulic valve (simplified representation)
 * @param ports Number of ports (4, 6, or 8)
 * @param sections Number of valve sections
 */
module hydraulic_valve(ports=4, sections=1) {
    valve_width = 80;
    valve_height = 100;
    section_length = 60;
    
    color("Gold")
    cube([valve_width, section_length * sections, valve_height]);
    
    // Port connections
    color("Silver")
    for (i = [0:sections-1]) {
        for (j = [0:ports-1]) {
            translate([valve_width/2 + (j-(ports-1)/2)*20, 
                      (i+0.5)*section_length, 
                      -20])
            cylinder(d=16, h=20);
        }
    }
}

// Example usage and visualization
if ($preview) {
    // Hydraulic cylinder - retracted
    hydraulic_cylinder(63.5, 31.75, 300, true, 0, "clevis");
    
    // Hydraulic cylinder - extended
    translate([150, 0, 0])
    hydraulic_cylinder(63.5, 31.75, 300, false, 0, "clevis");
    
    // Hydraulic motor
    translate([0, 150, 0])
    hydraulic_motor(100, 25.4, 100, true);
    
    // Hydraulic pump
    translate([150, 150, 0])
    hydraulic_pump(100, 25.4);
    
    // Hydraulic valve
    translate([300, 0, 0])
    hydraulic_valve(4, 2);
    
    // Hydraulic hose example
    translate([0, 300, 0])
    hydraulic_hose([0, 0, 0], [100, 50, 150], 19, true);
}
