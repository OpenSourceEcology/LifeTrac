// export_bucket_bottom.scad
// 2D export file for bucket bottom plate
// Generates SVG for CNC cutting

use <bucket_bottom.scad>

// Create 2D projection for CNC cutting
projection(cut=true)
bucket_bottom();
