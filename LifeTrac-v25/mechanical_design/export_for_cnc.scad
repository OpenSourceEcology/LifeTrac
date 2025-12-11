// export_for_cnc.scad
// Individual part exports for CNC cutting
// Use this file to export DXF files for plasma cutting
//
// Usage:
//   openscad -o output/dxf/part_name.dxf -D "part=\"part_name\"" export_for_cnc.scad
//
// Example:
//   openscad -o output/dxf/wheel_mount_plate.dxf -D "part=\"wheel_mount\"" export_for_cnc.scad

use <openscad/modules/plate_steel.scad>

// Part selection parameter
part = "wheel_mount"; // Options: wheel_mount, deck, bucket_bottom, bucket_back, bucket_side

// Plate thicknesses
PLATE_1_4 = 6.35;  // 1/4"
PLATE_1_2 = 12.7;  // 1/2"

// Which part to export
if (part == "wheel_mount") {
    // Wheel mounting plate - 1/2" plate steel
    // 300mm x 300mm with 4 mounting holes
    projection(cut = false)
    plate_with_holes(300, 300, PLATE_1_2, 13, 40, 6.35);
    
} else if (part == "deck") {
    // Standing deck - 1/4" plate steel
    // 1000mm x 400mm with anti-slip hole pattern
    projection(cut = false)
    difference() {
        plate_steel(1000, 400, PLATE_1_4, 6.35);
        
        // Anti-slip holes
        for (x = [50:100:950]) {
            for (y = [50:100:350]) {
                translate([x, y, -1])
                cylinder(d=20, h=10, $fn=32);
            }
        }
    }
    
} else if (part == "bucket_bottom") {
    // Bucket bottom plate - 1/4" plate steel
    // 1100mm x 600mm
    projection(cut = false)
    plate_steel(1100, 600, PLATE_1_4, 6.35);
    
} else if (part == "bucket_back") {
    // Bucket back plate - 1/4" plate steel
    // 1100mm x 400mm
    projection(cut = false)
    plate_steel(1100, 400, PLATE_1_4, 6.35);
    
} else if (part == "bucket_side") {
    // Bucket side plate - 1/4" plate steel
    // 600mm x 400mm
    projection(cut = false)
    plate_steel(600, 400, PLATE_1_4, 6.35);
    
} else if (part == "arm_reinforcement") {
    // Loader arm reinforcement - 1/4" plate steel
    // 150mm x 1200mm
    projection(cut = false)
    plate_steel(150, 1200, PLATE_1_4, 6.35);
    
} else if (part == "cylinder_lug") {
    // Hydraulic cylinder mounting lug - 1/2" plate steel
    // 100mm x 150mm with center hole
    projection(cut = false)
    difference() {
        plate_steel(100, 150, PLATE_1_2, 6.35);
        
        // Center mounting hole
        translate([50, 75, -1])
        cylinder(d=25, h=15, $fn=32);
    }
    
} else if (part == "bucket_attach") {
    // Bucket attachment plate - 1/2" plate steel
    // 200mm x 200mm with mounting holes
    projection(cut = false)
    plate_with_holes(200, 200, PLATE_1_2, 13, 30, 6.35);
    
} else if (part == "housing_base") {
    // Control housing base - 1/4" plate steel
    // 300mm x 200mm with mounting holes
    projection(cut = false)
    plate_with_holes(300, 200, PLATE_1_4, 13, 30, 6.35);
    
} else {
    // Default: show all available parts as reference
    echo("Available parts:");
    echo("  wheel_mount - Wheel mounting plate (300x300mm, 1/2\")");
    echo("  deck - Standing deck (1000x400mm, 1/4\")");
    echo("  bucket_bottom - Bucket bottom (1100x600mm, 1/4\")");
    echo("  bucket_back - Bucket back (1100x400mm, 1/4\")");
    echo("  bucket_side - Bucket side (600x400mm, 1/4\")");
    echo("  arm_reinforcement - Arm reinforcement (150x1200mm, 1/4\")");
    echo("  cylinder_lug - Cylinder lug (100x150mm, 1/2\")");
    echo("  bucket_attach - Bucket attachment (200x200mm, 1/2\")");
    echo("  housing_base - Control housing base (300x200mm, 1/4\")");
    echo("");
    echo("Usage: openscad -o output/dxf/PARTNAME.dxf -D 'part=\"PARTNAME\"' export_for_cnc.scad");
    
    // Show a sample part
    projection(cut = false)
    plate_steel(100, 100, PLATE_1_4, 6.35);
}

// Add part label as text (will appear in DXF)
translate([10, 10, 0])
text(str("Part: ", part), size=10, font="Liberation Sans:style=Bold");

translate([10, 25, 0])
text(str("Material: ", 
    (part == "wheel_mount" || part == "cylinder_lug" || part == "bucket_attach") ? "1/2\" Plate" : "1/4\" Plate"
), size=8);
