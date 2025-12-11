// export_side_panel_inner.scad
// 2D export file for inner side panel
// Generates SVG for CNC cutting

use <side_panel.scad>

// Create 2D projection for CNC cutting
projection(cut=true)
rotate([90, 0, 0])  // Rotate to lay flat
side_panel(is_inner=true);
