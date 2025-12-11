// standing_deck.scad
// Standing platform deck for LifeTrac v25 operator
// CNC cut from 1/4" plate with anti-slip hole pattern

include <../lifetrac_v25_params.scad>

module standing_deck() {
    // Main deck plate with anti-slip pattern
    difference() {
        // Base plate
        cube([DECK_WIDTH, DECK_DEPTH, PLATE_1_4_INCH], center=true);
        
        // Anti-slip hole pattern
        for (x = [-DECK_WIDTH/2+60 : 80 : DECK_WIDTH/2-60]) {
            for (y = [-DECK_DEPTH/2+60 : 80 : DECK_DEPTH/2-60]) {
                translate([x, y, 0])
                cylinder(d=25, h=PLATE_1_4_INCH+4, center=true, $fn=24);
            }
        }
        
        // Corner mounting holes
        for (x = [-DECK_WIDTH/2+40, DECK_WIDTH/2-40]) {
            for (y = [-DECK_DEPTH/2+40, DECK_DEPTH/2-40]) {
                translate([x, y, 0])
                cylinder(d=BOLT_DIA_1_2 + 2, h=PLATE_1_4_INCH+4, center=true, $fn=32);
            }
        }
    }
}

// Render the part for preview/export
if ($preview || len(search("standing_deck.scad", parent_modules())) == 0) {
    standing_deck();
}
