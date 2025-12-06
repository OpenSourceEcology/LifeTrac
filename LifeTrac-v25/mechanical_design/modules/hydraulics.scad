// hydraulics.scad
// Hydraulic cylinder and motor placeholders
// Part of LifeTrac v25 OpenSCAD design

// Global parameters
$fn = 32;

/**
 * Creates a hydraulic cylinder
 * @param bore_diameter Cylinder bore diameter in mm
 * @param rod_diameter Rod diameter in mm
 * @param stroke Stroke length in mm
 * @param retracted Show in retracted position if true
 * @param extension Current extension (0 to stroke) in mm
 * @param mounting_type Type of mounting ("clevis", "eye", "trunnion")
 */
module hydraulic_cylinder(bore_diameter=63.5, rod_diameter=31.75, 
                         stroke=300, retracted=true, extension=0,
                         mounting_type="clevis") {
    
    body_length = stroke + 100; // Body is longer than stroke
    actual_extension = retracted ? 0 : (extension > 0 ? extension : stroke);
    
    // Cylinder body
    color("Orange")
    cylinder(d=bore_diameter, h=body_length);
    
    // Rod
    color("Silver")
    translate([0, 0, body_length])
    cylinder(d=rod_diameter, h=stroke - actual_extension);
    
    // Base mounting
    color("DimGray")
    translate([0, 0, -20])
    if (mounting_type == "clevis") {
        // Clevis mount at base
        cylinder(d=bore_diameter*1.2, h=20);
        translate([0, 0, 10])
        rotate([90, 0, 0])
            cylinder(d=25, h=bore_diameter*1.5, center=true);
    } else if (mounting_type == "eye") {
        // Eye mount at base
        difference() {
            cylinder(d=bore_diameter*1.2, h=20);
            translate([0, 0, 10])
                rotate([90, 0, 0])
                    cylinder(d=25, h=bore_diameter*1.6, center=true);
        }
    } else {
        // Trunnion mount
        rotate([0, 0, 0])
        cylinder(d=bore_diameter*1.3, h=20);
    }
    
    // Rod end mounting
    color("DimGray")
    translate([0, 0, body_length + stroke - actual_extension])
    if (mounting_type == "clevis") {
        // Clevis at rod end
        cylinder(d=rod_diameter*2, h=30);
        translate([0, 0, 15])
        rotate([90, 0, 0])
            cylinder(d=20, h=rod_diameter*3, center=true);
    } else {
        // Eye at rod end
        difference() {
            cylinder(d=rod_diameter*2, h=30);
            translate([0, 0, 15])
                rotate([90, 0, 0])
                    cylinder(d=20, h=rod_diameter*3.2, center=true);
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
