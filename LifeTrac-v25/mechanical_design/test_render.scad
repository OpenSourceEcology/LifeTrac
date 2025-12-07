use <modules/plate_steel.scad>

// Simple test to see if projection works
projection(cut=false) {
    plate_steel(300, 200, 6.35, 6.35);
}
