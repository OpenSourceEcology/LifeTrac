// export_rear_crossmember.scad
// 2D export file for rear crossmember
// Generates SVG for CNC cutting

use <rear_crossmember.scad>

// Create 2D projection for CNC cutting
projection(cut=true)
rear_crossmember();
