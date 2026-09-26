// hardware.scad - reference models of purchased fasteners for the part drawings
//
// Nominal dimensions (inches) for inch-series hardware:
//   hex bolts / cap screws  ASME B18.2.1   (width across flats F, head height H)
//   hex nuts                ASME B18.2.2   (width across flats F, thickness T)
//   flat washers            SAE / USS pattern (ANSI B18.22.1 Type A)
// These are REFERENCE shapes for recognising and ordering hardware; buy to the
// callout on the drawing, not to these models.  Threads are shown simplified
// (ASME Y14.6): a single line marks where the thread starts.

IN = 25.4;

// [nominal dia, threads per inch, bolt F, bolt H, nut F, nut T]
HEX_TABLE = [
    [0.25,  20, 7/16,   5/32,  7/16,  7/32],
    [0.375, 16, 9/16,  15/64,  9/16, 21/64],
    [0.5,   13, 3/4,    5/16,  3/4,   7/16],
    [0.625, 11, 15/16, 25/64, 15/16, 35/64],
    [0.75,  10, 1.125, 15/32, 1.125, 41/64],
    [1.0,    8, 1.5,   39/64, 1.5,   55/64],
    [1.5,    6, 2.25,  15/16, 2.25,  1 + 9/32],
];

// [nominal dia, SAE ID, SAE OD, SAE t, USS ID, USS OD, USS t]
WASHER_TABLE = [
    [0.25,  9/32,  5/8,    0.065, 5/16,  3/4,   0.065],
    [0.375, 13/32, 13/16,  0.065, 7/16,  1,     0.083],
    [0.5,   17/32, 1.0625, 0.095, 9/16,  1.375, 0.109],
    [0.625, 21/32, 1.3125, 0.095, 11/16, 1.75,  0.134],
    [0.75,  13/16, 1.46875, 0.134, 13/16, 2,    0.148],
    [1.0,   1.0625, 2,     0.134, 1.0625, 2.5,  0.165],
];

function _row(table, d) = table[search([d], [for (r = table) r[0]])[0]];

module _hex(af, h) {
    cylinder(d = af / cos(30), h = h, $fn = 6);
}

// Hex bolt, head on the XY plane, shank along -Z.  length_in = under-head length.
module hex_bolt_ref(d_in, length_in) {
    r = _row(HEX_TABLE, d_in);
    d = d_in * IN;
    L = length_in * IN;
    thread = min(L, (length_in <= 6 ? 2 * d_in + 0.25 : 2 * d_in + 0.5) * IN);
    // Chamfered hex head (30 deg chamfer on the top corners).
    intersection() {
        _hex(r[2] * IN, r[3] * IN);
        cylinder(d1 = r[2] * IN / cos(30) + 2 * r[3] * IN, d2 = r[2] * IN * 0.95, h = r[3] * IN, $fn = 64);
    }
    // Plain shank, then the threaded length (shown a hair under major Ø so
    // the drawing gets a line where the thread starts).
    translate([0, 0, -(L - thread)]) cylinder(d = d, h = L - thread + 0.01, $fn = 48);
    translate([0, 0, -L]) {
        cylinder(d = d * 0.985, h = thread, $fn = 48);
    }
}

module hex_nut_ref(d_in) {
    r = _row(HEX_TABLE, d_in);
    difference() {
        intersection() {
            _hex(r[4] * IN, r[5] * IN);
            // Double-chamfered nut faces.
            union() {
                cylinder(d1 = r[4] * IN * 0.95, d2 = r[4] * IN / cos(30) + r[5] * IN, h = r[5] * IN / 2, $fn = 64);
                translate([0, 0, r[5] * IN / 2])
                cylinder(d1 = r[4] * IN / cos(30) + r[5] * IN, d2 = r[4] * IN * 0.95, h = r[5] * IN / 2, $fn = 64);
            }
        }
        translate([0, 0, -1]) cylinder(d = d_in * IN, h = r[5] * IN + 2, $fn = 48);
    }
}

module flat_washer_ref(d_in, pattern = "SAE") {
    r = _row(WASHER_TABLE, d_in);
    k = pattern == "USS" ? 4 : 1;
    difference() {
        cylinder(d = r[k + 1] * IN, h = r[k + 2] * IN, $fn = 96);
        translate([0, 0, -1]) cylinder(d = r[k] * IN, h = r[k + 2] * IN + 2, $fn = 64);
    }
}

// Plain pin (round bar cut to length), axis along X.
module pin_ref(d_mm, length_mm) {
    rotate([0, 90, 0]) cylinder(d = d_mm, h = length_mm, $fn = 64);
}
