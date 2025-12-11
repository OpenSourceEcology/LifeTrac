// export_bucket_side.scad
// 2D export file for bucket side plate
// Generates SVG for CNC cutting

use <bucket_side.scad>

// Create 2D projection for CNC cutting
projection(cut=true)
rotate([0, -90, 0])  // Rotate to lay flat
bucket_side();
