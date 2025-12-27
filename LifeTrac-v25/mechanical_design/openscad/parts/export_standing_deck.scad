// export_standing_deck.scad
// DEPRECATED - Use export_platform_deck.scad instead
// This file redirects to the new platform deck for backward compatibility

use <platform_deck.scad>

// Create 2D projection for CNC cutting
projection(cut=true)
platform_deck();
