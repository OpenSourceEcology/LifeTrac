// wheels.scad
// Standardized wheel and axle assemblies based on OSE designs
// Part of LifeTrac v25 OpenSCAD design

// Global parameters
$fn = 64;

/**
 * Creates a wheel assembly (simplified representation)
 * @param diameter Overall wheel diameter in mm
 * @param width Wheel width in mm
 * @param hub_diameter Hub diameter in mm
 * @param tire_tread Show tire tread pattern if true
 */
module wheel(diameter=500, width=200, hub_diameter=150, tire_tread=true) {
    
    // Tire
    color("Black")
    rotate([90, 0, 0])
    difference() {
        cylinder(d=diameter, h=width, center=true);
        cylinder(d=hub_diameter, h=width+2, center=true);
        
        // Simplified tread pattern
        if (tire_tread) {
            for (angle = [0:15:360]) {
                rotate([0, 0, angle])
                translate([diameter/2-10, 0, 0])
                    cube([20, 20, width+2], center=true);
            }
        }
    }
    
    // Rim/hub
    color("DimGray")
    rotate([90, 0, 0])
    cylinder(d=hub_diameter, h=width*0.8, center=true);
    
    // Hub face with bolt pattern
    color("Silver")
    rotate([90, 0, 0])
    difference() {
        cylinder(d=hub_diameter*0.7, h=25, center=true);
        // Bolt holes
        for (angle = [0:45:315]) {
            rotate([0, 0, angle])
            translate([hub_diameter*0.4, 0, 0])
                cylinder(d=20, h=30, center=true);
        }
    }
}

/**
 * Creates a wheel with hydraulic motor mount
 * @param diameter Wheel diameter in mm
 * @param width Wheel width in mm
 * @param motor_displacement Motor size in cc/rev
 */
module powered_wheel(diameter=500, width=200, motor_displacement=100) {
    // Wheel
    wheel(diameter, width, 150, true);
    
    // Hydraulic motor mounted to hub
    translate([0, width/2 + 50, 0])
    rotate([90, 0, 0])
    import("hydraulics.scad");
    // Note: In practice, use: use <hydraulics.scad>; hydraulic_motor(motor_displacement);
}

/**
 * Creates a modular wheel unit (MWU) based on OSE design
 * Includes wheel, motor, and mounting frame
 * @param wheel_diameter Wheel diameter in mm
 * @param frame_width Frame width in mm
 */
module modular_wheel_unit(wheel_diameter=500, frame_width=300) {
    // Mounting frame - simplified
    color("Silver")
    difference() {
        union() {
            // Main frame plate
            translate([-frame_width/2, 0, -frame_width/2])
            cube([frame_width, 20, frame_width]);
            
            // Side supports
            translate([-frame_width/2, 0, -frame_width/2])
            cube([20, 100, frame_width]);
            translate([frame_width/2-20, 0, -frame_width/2])
            cube([20, 100, frame_width]);
        }
        
        // Center hole for axle
        rotate([90, 0, 0])
        translate([0, 0, -10])
        cylinder(d=100, h=120);
        
        // Mounting bolt holes
        for (x = [-frame_width/2+40, frame_width/2-40]) {
            for (z = [-frame_width/2+40, frame_width/2-40]) {
                translate([x, -5, z])
                cylinder(d=13, h=30);
            }
        }
    }
    
    // Wheel
    translate([0, 120, 0])
    wheel(wheel_diameter, 200, 150, true);
    
    // Hydraulic motor
    translate([0, 50, 0])
    rotate([90, 0, 0])
    color("DarkSlateBlue")
    cylinder(d=80, h=100);
}

/**
 * Creates an axle shaft
 * @param diameter Axle diameter in mm
 * @param length Axle length in mm
 * @param keyway Add keyway slot if true
 */
module axle(diameter=50.8, length=500, keyway=true) {
    color("Silver")
    difference() {
        cylinder(d=diameter, h=length);
        
        // Keyway slot
        if (keyway) {
            translate([-diameter/4, diameter/2-5, -1])
            cube([diameter/2, 10, length+2]);
        }
    }
}

/**
 * Creates a bearing housing
 * @param outer_diameter Outer housing diameter in mm
 * @param inner_diameter Bearing inner diameter in mm
 * @param height Housing height in mm
 * @param bolt_holes Number of mounting bolt holes
 */
module bearing_housing(outer_diameter=150, inner_diameter=52, 
                       height=50, bolt_holes=4) {
    color("DarkGray")
    difference() {
        union() {
            // Main housing body
            cylinder(d=outer_diameter, h=height);
            
            // Mounting flange
            translate([0, 0, -10])
            cylinder(d=outer_diameter*1.3, h=10);
        }
        
        // Bearing bore
        translate([0, 0, -1])
        cylinder(d=inner_diameter, h=height+2);
        
        // Mounting bolt holes
        translate([0, 0, -11])
        for (angle = [0:360/bolt_holes:360-1]) {
            rotate([0, 0, angle])
            translate([outer_diameter*0.5, 0, 0])
            cylinder(d=13, h=12);
        }
    }
    
    // Bearing (simplified)
    color("Silver")
    translate([0, 0, height/2])
    difference() {
        cylinder(d=outer_diameter*0.8, h=20, center=true);
        cylinder(d=inner_diameter, h=22, center=true);
    }
}

/**
 * Creates a complete wheel assembly with bearings and axle
 * @param wheel_diameter Wheel diameter in mm
 * @param axle_length Length of axle in mm
 * @param bearing_spacing Distance between bearings in mm
 */
module complete_wheel_assembly(wheel_diameter=500, axle_length=600, 
                               bearing_spacing=400) {
    // Axle
    rotate([90, 0, 0])
    axle(50.8, axle_length, true);
    
    // Bearings
    rotate([90, 0, 0])
    translate([0, 0, bearing_spacing/2])
    bearing_housing(150, 52, 50, 4);
    
    rotate([90, 0, 0])
    translate([0, 0, -bearing_spacing/2-50])
    bearing_housing(150, 52, 50, 4);
    
    // Wheel
    translate([0, -axle_length/2, 0])
    wheel(wheel_diameter, 200, 150, true);
}

// Example usage and visualization
if ($preview) {
    // Basic wheel
    wheel(500, 200, 150, true);
    
    // Modular wheel unit
    translate([600, 0, 0])
    modular_wheel_unit(500, 300);
    
    // Complete wheel assembly
    translate([0, 600, 250])
    complete_wheel_assembly(500, 600, 400);
    
    // Axle
    translate([0, 1200, 0])
    rotate([0, 90, 0])
    axle(50.8, 500, true);
    
    // Bearing housing
    translate([600, 1200, 0])
    bearing_housing(150, 52, 50, 4);
}
