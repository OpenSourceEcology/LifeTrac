// export_platform_deck.scad
// 2D export file for platform deck plate
// Generates SVG for CNC plasma cutting

use <platform_deck.scad>

// Create 2D projection for CNC cutting
projection(cut=true)
platform_deck();
