include </home/eric/grid_beam.scad>

$fn=36;

module ose_plate(thickness, length) {
	translate([thickness/2,length/2,0]) {
		cube(size=[thickness, length, 4], center=true);
	}
}

length = 72;

ose_beam(1/4,length);
translate([-2,0,4]) { ose_plate(1/4,length); }
translate([0,0,8]) { ose_beam(1/4,length); }
translate([-2,0,12]) { ose_plate(1/4,length); }
