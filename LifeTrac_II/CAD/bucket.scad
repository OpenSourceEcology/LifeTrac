thick = 1/4;
width = 72;
side = 27;

module triangle_plate(points, thickness) {
	o = [0,0,thickness];
	polyhedron(points=[points[0],points[1],points[2],points[0]+o,points[1]+o,points[2]+o ], 
   	triangles= [ [0,1,2], [1, 4, 5, 2], [3,5,4], [0, 3, 4, 1], [0,2,5,3]]);
}

// left
triangle_plate([[0,0,0],[side,0,0],[0,side,0]], thick);

// right
translate([0,0,width-thick]) {
	triangle_plate([[0,0,0],[side,0,0],[0,side,0]], thick);
}

// back
translate([0,0,0]) {
	cube(size=[thick, side, width]);
}

// bottom
translate([0,-thick,0]) {
	cube(size=[side, thick, width]);
}
