// export_standing_deck.scad
// 2D export file for standing deck
// Generates SVG for CNC cutting

use <standing_deck.scad>

// Create 2D projection for CNC cutting
projection(cut=true)
standing_deck();
