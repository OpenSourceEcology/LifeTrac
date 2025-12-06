// cnclayout.scad
// CNC layout for all plate steel parts
// Generates 2D projection for plasma cutting
// Part of LifeTrac v25 OpenSCAD design

use <modules/plate_steel.scad>

// Plate thicknesses
PLATE_1_4 = 6.35;  // 1/4"
PLATE_1_2 = 12.7;  // 1/2"

// Layout spacing
SPACING = 20; // mm between parts
START_X = 10;
START_Y = 10;

// Color coding by thickness
COLOR_1_4 = "blue";
COLOR_1_2 = "red";

// Current position tracking
x_offset = START_X;
y_offset = START_Y;
row_height = 0;

module layout_part(width, height, thickness, label, x, y) {
    translate([x, y, 0]) {
        projection(cut=false) {
            color(thickness == PLATE_1_4 ? COLOR_1_4 : COLOR_1_2)
            plate_steel(width, height, thickness, 6.35);
        }
        // Part label
        translate([5, 5, 0])
        text(label, size=8, font="Liberation Sans:style=Bold");
        
        // Thickness indicator
        translate([5, height - 15, 0])
        text(str(thickness == PLATE_1_4 ? "1/4\"" : "1/2\""), size=6);
    }
}

// Layout all parts in rows
// Row 1: Half-inch plates (wheel mounts and high-stress parts)
layout_part(300, 300, PLATE_1_2, "A2-1 Wheel Mount FL", START_X, START_Y);
layout_part(300, 300, PLATE_1_2, "A2-2 Wheel Mount FR", START_X + 300 + SPACING, START_Y);
layout_part(300, 300, PLATE_1_2, "A2-3 Wheel Mount RL", START_X + 2*(300 + SPACING), START_Y);
layout_part(300, 300, PLATE_1_2, "A2-4 Wheel Mount RR", START_X + 3*(300 + SPACING), START_Y);

// Row 2: Half-inch plates continued
layout_part(200, 200, PLATE_1_2, "C2-1 Bucket Attach L", START_X, START_Y + 300 + SPACING);
layout_part(200, 200, PLATE_1_2, "C2-2 Bucket Attach R", START_X + 200 + SPACING, START_Y + 300 + SPACING);
layout_part(100, 150, PLATE_1_2, "Cyl Lug 1", START_X + 2*(200 + SPACING), START_Y + 300 + SPACING);
layout_part(100, 150, PLATE_1_2, "Cyl Lug 2", START_X + 2*(200 + SPACING) + 100 + SPACING, START_Y + 300 + SPACING);
layout_part(100, 150, PLATE_1_2, "Cyl Lug 3", START_X + 2*(200 + SPACING) + 2*(100 + SPACING), START_Y + 300 + SPACING);
layout_part(100, 150, PLATE_1_2, "Cyl Lug 4", START_X + 2*(200 + SPACING) + 3*(100 + SPACING), START_Y + 300 + SPACING);

// Row 3: Quarter-inch plates (bucket)
layout_part(1100, 600, PLATE_1_4, "E1-1 Bucket Bottom", START_X, START_Y + 2*(300 + SPACING));

// Row 4: Quarter-inch plates (bucket continued)
layout_part(1100, 400, PLATE_1_4, "E1-2 Bucket Back", START_X, START_Y + 2*(300 + SPACING) + 600 + SPACING);

// Row 5: Quarter-inch plates (bucket sides and arms)
layout_part(600, 400, PLATE_1_4, "E1-3 Bucket Side L", START_X, START_Y + 2*(300 + SPACING) + 600 + 400 + 2*SPACING);
layout_part(600, 400, PLATE_1_4, "E1-4 Bucket Side R", START_X + 600 + SPACING, START_Y + 2*(300 + SPACING) + 600 + 400 + 2*SPACING);

// Row 6: Quarter-inch plates (arm reinforcements and deck)
layout_part(150, 1200, PLATE_1_4, "C1-1 Arm Reinf L", START_X, START_Y + 2*(300 + SPACING) + 600 + 400 + 400 + 3*SPACING);
layout_part(150, 1200, PLATE_1_4, "C1-2 Arm Reinf R", START_X + 150 + SPACING, START_Y + 2*(300 + SPACING) + 600 + 400 + 400 + 3*SPACING);

// Row 7: Quarter-inch plates (standing deck)
layout_part(1000, 400, PLATE_1_4, "F1 Standing Deck", START_X + 2*(150 + SPACING), START_Y + 2*(300 + SPACING) + 600 + 400 + 400 + 3*SPACING);

// Row 8: Quarter-inch plate (control housing base)
layout_part(300, 200, PLATE_1_4, "G1 Housing Base", START_X, START_Y + 2*(300 + SPACING) + 600 + 400 + 400 + 1200 + 4*SPACING);

// Add title and legend
translate([10, START_Y + 2*(300 + SPACING) + 600 + 400 + 400 + 1200 + 200 + 5*SPACING, 0])
text("LifeTrac v25 CNC Layout", size=16, font="Liberation Sans:style=Bold");

translate([10, START_Y + 2*(300 + SPACING) + 600 + 400 + 400 + 1200 + 180 + 5*SPACING, 0])
text("Blue = 1/4\" plate | Red = 1/2\" plate", size=10);

// Add cutting instructions
translate([10, START_Y + 2*(300 + SPACING) + 600 + 400 + 400 + 1200 + 160 + 5*SPACING, 0])
text("Maintain 3mm spacing between cuts", size=8);

translate([10, START_Y + 2*(300 + SPACING) + 600 + 400 + 400 + 1200 + 145 + 5*SPACING, 0])
text("All corners 6.35mm radius", size=8);
