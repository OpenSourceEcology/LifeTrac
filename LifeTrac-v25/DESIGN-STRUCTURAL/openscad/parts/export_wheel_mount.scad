// export_wheel_mount.scad
// 2D export file for wheel mount plate
// Generates SVG for CNC cutting

use <wheel_mount.scad>

// Create 2D projection for CNC cutting
projection(cut=true)
wheel_mount();
