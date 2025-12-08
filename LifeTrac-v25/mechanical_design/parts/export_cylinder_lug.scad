// export_cylinder_lug.scad
// 2D export file for hydraulic cylinder lug
// Generates SVG for CNC cutting

use <cylinder_lug.scad>

// Create 2D projection for CNC cutting
projection(cut=true)
cylinder_lug();
