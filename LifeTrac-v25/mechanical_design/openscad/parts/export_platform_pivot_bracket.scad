// export_platform_pivot_bracket.scad
// 2D export file for platform pivot bracket plate
// Generates SVG for CNC plasma cutting
// Note: Two brackets required (left and right side)

use <platform_pivot_bracket.scad>

// Create 2D projection for CNC cutting
projection(cut=true)
platform_pivot_bracket();
