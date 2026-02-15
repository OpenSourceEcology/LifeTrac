// UTU-v25 Universal Track Unit Assembly
// Units: mm (converted from inches)
//
// Default layout: 3-axis triangular system
//   - 2 idler axes at bottom, 4 feet apart
//   - 1 drive axis, 2 feet above idlers, centered
//
// Track chain: chainsaw-chain style staggered links
//   - Track feet: 2.5" x 0.5" steel bar, 10" long
//   - Connected by 1" diameter steel rods
//   - Perpendicular connector plates, alternating inner/outer stagger
//
// Compatible with UWU bearing/motor mounting pattern
//
// Gear library: chrisspen/gears (https://github.com/chrisspen/gears)
//   License: CC BY-NC-SA

use <gears.scad>

$fn = 50;

inch = 25.4;
PI = 3.14159265359;
eps = 0.1;  // Small epsilon for boolean cut-through margins

// ============================================================
// CONFIGURATION
// ============================================================

num_axes = 3;              // 2 = idlers only, 3+ = idlers + drive
show_track = true;         // Show track chain links
animate_track = true;      // Animate track rotation (use View > Animate, FPS=10, Steps=100)

// ============================================================
// AXIS LAYOUT (positioned in XZ plane, Y = axle direction)
// ============================================================

idler_spacing = 48 * inch;              // 4 feet between bottom idler centers
drive_height_above_idlers = 24 * inch;  // 2 feet above idler axis height

// ============================================================
// PLATE & SHAFT DIMENSIONS (matching UWU style)
// ============================================================

plate_w = 8 * inch;        // Bearing/motor mount plate width
plate_h = 8 * inch;        // Bearing/motor mount plate height
plate_t = 0.25 * inch;     // Plate thickness

shaft_diam = 1.25 * inch;          // 1.25" main shaft
center_hole_diam = 2.00 * inch;    // Clearance hole for shaft/bearing housing
bolt_circle_diam = 6.19 * inch;    // Bearing mount BCD
bolt_hole_diam = 0.5 * inch;       // Bolt hole diameter

motor_mount_bcd = 8.25 * inch;     // Motor mount BCD (4.125" radius)

shaft_length = 14 * inch;          // Total shaft length (spans track + bearings)

// Distance between bearing plates (wider than track for clearance)
bearing_plate_spacing_1_2 = 3 * inch;  // Motor plate to first bearing
bearing_plate_spacing_2_3 = 4 * inch;  // Between bearing plates (idler)

// ============================================================
// BEARING DIMENSIONS
// ============================================================

bearing_housing_diam = 2.5 * inch;
bearing_flange_h = 0.5 * inch;          // Flange base thickness
bearing_flange_lobe_diam = 1 * inch;    // Diameter of each flange lobe at bolt
bearing_housing_len = 0.75 * inch;      // Housing protrusion above flange

// ============================================================
// MOTOR DIMENSIONS
// ============================================================

motor_body_diam = 6 * inch;
motor_body_len = 5 * inch;
motor_shaft_diam = 1 * inch;            // Motor output shaft diameter
motor_shaft_len = 1.5 * inch;
motor_flange_size = 7.5 * inch;         // Motor flange square side length
motor_flange_t = 0.5 * inch;            // Motor flange thickness

coupling_diam = 2 * inch;
coupling_len = 2 * inch;

// ============================================================
// SHAFT COLLAR DIMENSIONS
// ============================================================

collar_od = 2 * inch;
collar_width = 0.75 * inch;
collar_gap = 0.5;  // mm gap between collar and sprocket

// ============================================================
// TRACK LINK DIMENSIONS
// ============================================================

track_bar_length = 10 * inch;       // Ground contact length (perpendicular to travel)
track_bar_width = 2.5 * inch;       // Width in direction of travel
track_bar_thickness = 0.5 * inch;   // Bar thickness

link_rod_diam = 1 * inch;           // Connecting rod diameter

// ============================================================
// SPROCKET DIMENSIONS (chain sprocket geometry)
// ============================================================

num_teeth = 12;
track_gap = 0.5 * inch;            // Gap between adjacent track bars
chain_pitch = track_bar_width + track_gap;  // Center-to-center of connecting rods

// Standard chain sprocket pitch radius: R = P / (2 * sin(180/N))
sprocket_pitch_radius = chain_pitch / (2 * sin(180 / num_teeth));

// Rod seat (where connecting rod sits in the tooth valley)
rod_clearance = 2;                            // mm clearance around rod
rod_seat_r = (link_rod_diam + rod_clearance) / 2;
rod_hole_clearance = 1;                       // mm diametral clearance for rod holes in plates

// Fillet radius at tooth root = rod radius (matches bar curvature)
fillet_r = link_rod_diam / 2;

// Root radius: gear sits on track bar when engaged
// (bottom of rod seat contacts bar surface)
sprocket_root_r = sprocket_pitch_radius - link_rod_diam / 2;

// Tip radius: teeth extend above pitch circle
sprocket_tip_r = sprocket_pitch_radius + rod_seat_r;
sprocket_od = 2 * sprocket_tip_r;
sprocket_thickness = 0.5 * inch;

// Connector plates (welded perpendicular to bar, standing vertical)
// Each plate has TWO rod holes: front and rear, connecting to adjacent links
connector_height = 2 * inch;        // Height above bar surface
connector_thickness = 0.25 * inch;  // Plate thickness (in Y direction)
connector_width = chain_pitch;      // Width in travel direction (X) — spans full pitch

// Rod hole spacing within each connector plate
// Holes are at ±chain_pitch/2 from bar center in travel (X) direction
rod_hole_x_offset = chain_pitch / 2; // Distance from plate center to each rod hole

// Stagger positions (distance from bar centerline along Y/bar-length axis)
// Inner link: plates closer to center
// Outer link: plates farther from center
// At each chain joint, outer plates straddle inner plates
//
// Inner pair: face-to-face gap → center offset = (face_gap/2) + (connector_thickness/2)
// Outer pair: stagger_gap from inner outer face
inner_face_gap = 2 * inch;                // Face-to-face gap between inner plates
stagger_gap = 0.125 * inch;               // Gap between inner outer face and outer plate
inner_plate_offset = inner_face_gap / 2 + connector_thickness / 2;
outer_plate_offset = inner_plate_offset + connector_thickness / 2 + stagger_gap + connector_thickness / 2;

// Rod center height above bar bottom (centered at midpoint of connector plate)
rod_z_above_bar = track_bar_thickness + connector_height / 2;

// Rod length (spans between outermost plates + clearance)
link_rod_length = 2 * outer_plate_offset + 2 * connector_thickness;

// Gap between adjacent bars in travel direction
bar_gap = track_gap;

// ============================================================
// COMPUTED POSITIONS
// ============================================================

// Idler shaft Z: positions sprocket so bottom track run is near Z=0
idler_shaft_z = sprocket_pitch_radius + rod_z_above_bar;

// Drive axis position
drive_x = idler_spacing / 2;
drive_z = idler_shaft_z + drive_height_above_idlers;

// ============================================================
// CHAIN PATH COMPUTATION
// ============================================================
// Compute full chain loop around 3 sprockets.
// Adjust front (left) idler X so total chain = integer × pitch.

// Direction angles between axis centers (XZ plane, degrees)
angle_AB = 0;  // Both idlers at same Z
angle_BC = atan2(drive_height_above_idlers, drive_x - idler_spacing);
angle_CA_nom = atan2(-drive_height_above_idlers, -drive_x);

// Nominal center-to-center distances (left idler at X=0)
nom_d_AB = idler_spacing;
nom_d_BC = sqrt((drive_x - idler_spacing) * (drive_x - idler_spacing)
             + drive_height_above_idlers * drive_height_above_idlers);
nom_d_CA = sqrt(drive_x * drive_x
             + drive_height_above_idlers * drive_height_above_idlers);

// Chain length = straight spans + sprocket wraps
// Sprocket wraps: total engagement = num_teeth pitches (always, for equal-R)
nom_straight = nom_d_AB + nom_d_BC + nom_d_CA;
nom_chain_len = nom_straight + num_teeth * chain_pitch;

// Round to nearest integer number of links
num_chain_links = round(nom_chain_len / chain_pitch);
target_len = num_chain_links * chain_pitch;

// Front idler X adjustment (Newton linear approx)
// d(straight)/d(delta) ≈ -1 - drive_x / d_CA
front_idler_adjust = -(target_len - nom_chain_len)
                   / (1 + drive_x / nom_d_CA);
left_idler_x = front_idler_adjust;

// Recomputed distances with adjusted front idler
adj_d_AB = idler_spacing - left_idler_x;
adj_d_CA = sqrt((left_idler_x - drive_x) * (left_idler_x - drive_x)
             + drive_height_above_idlers * drive_height_above_idlers);
angle_CA = atan2(-drive_height_above_idlers, left_idler_x - drive_x);

// Tangent contact angles on each sprocket
// Equal-radius external tangent: contact angle = segment_direction - 90°
// Belt wraps CCW at each sprocket (exterior of CW overall loop)
tang_A_dep = angle_AB - 90;      // A departure toward B
tang_A_arr = angle_CA  - 90;     // A arrival from C
tang_B_arr = angle_AB - 90;      // B arrival from A
tang_B_dep = angle_BC - 90;      // B departure toward C
tang_C_arr = angle_BC - 90;      // C arrival from B
tang_C_dep = angle_CA  - 90;     // C departure toward A

// CCW wrap angles at each sprocket (degrees)
function wrap_angle(from_a, to_a) =
    let(w = ((to_a - from_a) % 360 + 360) % 360)
    w < 0.01 ? 360 : w;

wrap_A = wrap_angle(tang_A_arr, tang_A_dep);
wrap_B = wrap_angle(tang_B_arr, tang_B_dep);
wrap_C = wrap_angle(tang_C_arr, tang_C_dep);

// Path segment lengths (arc-length for sprocket wraps)
seg_AB   = adj_d_AB;
seg_arcB = sprocket_pitch_radius * wrap_B * PI / 180;
seg_BC   = nom_d_BC;
seg_arcC = sprocket_pitch_radius * wrap_C * PI / 180;
seg_CA   = adj_d_CA;
seg_arcA = sprocket_pitch_radius * wrap_A * PI / 180;

chain_path_len = seg_AB + seg_arcB + seg_BC + seg_arcC + seg_CA + seg_arcA;

// Cumulative segment boundaries
cum_AB   = seg_AB;
cum_arcB = cum_AB   + seg_arcB;
cum_BC   = cum_arcB + seg_BC;
cum_arcC = cum_BC   + seg_arcC;
cum_CA   = cum_arcC + seg_CA;
cum_arcA = cum_CA   + seg_arcA;  // = chain_path_len

// Straight-segment start points (on pitch circle)
s1x = left_idler_x  + sprocket_pitch_radius * cos(tang_A_dep);
s1z = idler_shaft_z  + sprocket_pitch_radius * sin(tang_A_dep);
s3x = idler_spacing  + sprocket_pitch_radius * cos(tang_B_dep);
s3z = idler_shaft_z  + sprocket_pitch_radius * sin(tang_B_dep);
s5x = drive_x        + sprocket_pitch_radius * cos(tang_C_dep);
s5z = drive_z        + sprocket_pitch_radius * sin(tang_C_dep);

// Path point function: returns [x, z, travel_angle]
// s = cumulative distance along chain loop
function chain_point(s) =
    let(ss = ((s % chain_path_len) + chain_path_len) % chain_path_len,
        R  = sprocket_pitch_radius)
    ss < cum_AB ?
        [s1x + ss * cos(angle_AB),
         s1z + ss * sin(angle_AB),
         angle_AB] :
    ss < cum_arcB ?
        let(ds = ss - cum_AB,
            a  = tang_B_arr + ds / R * 180 / PI)
        [idler_spacing + R * cos(a),
         idler_shaft_z  + R * sin(a),
         a + 90] :
    ss < cum_BC ?
        let(ds = ss - cum_arcB)
        [s3x + ds * cos(angle_BC),
         s3z + ds * sin(angle_BC),
         angle_BC] :
    ss < cum_arcC ?
        let(ds = ss - cum_BC,
            a  = tang_C_arr + ds / R * 180 / PI)
        [drive_x + R * cos(a),
         drive_z  + R * sin(a),
         a + 90] :
    ss < cum_CA ?
        let(ds = ss - cum_arcC)
        [s5x + ds * cos(angle_CA),
         s5z + ds * sin(angle_CA),
         angle_CA] :
        let(ds = ss - cum_CA,
            a  = tang_A_arr + ds / R * 180 / PI)
        [left_idler_x  + R * cos(a),
         idler_shaft_z  + R * sin(a),
         a + 90];

// ============================================================
// SPROCKET PHASE COMPUTATION
// ============================================================
// Each sprocket needs a unique phase rotation so its rod seats
// align with the chain rod positions on its arc.

tooth_angle = 360 / num_teeth;
deg_per_mm_arc = 180 / (PI * sprocket_pitch_radius);

// Given the cumulative distance where the chain arrives on a sprocket
// arc and the arrival tangent angle, compute the rotation phase so
// that a rod seat coincides with where chain rods actually land.
function sprocket_phase(s_start, a_arr) =
    let(
        // Rods are at k*chain_pitch along the loop.
        // Find how far past the last rod the arc start is.
        frac = ((s_start / chain_pitch) % 1.0 + 1.0) % 1.0,
        // Arc distance from arc start back to that previous rod
        ds_back = frac * chain_pitch,
        // That rod's angle on the sprocket
        rod_ang = a_arr - ds_back * deg_per_mm_arc
    )
    ((rod_ang % tooth_angle) + tooth_angle) % tooth_angle;

phase_A = sprocket_phase(cum_CA, tang_A_arr);
phase_B = sprocket_phase(cum_AB, tang_B_arr);
phase_C = sprocket_phase(cum_BC, tang_C_arr);


// ============================================================
// MODULES - Mounting Plates
// ============================================================

module plate(is_motor_mount = false) {
    difference() {
        color("Silver")
        cube([plate_w, plate_h, plate_t], center = true);

        // Center hole
        cylinder(h = plate_t * 2, d = center_hole_diam, center = true);

        // Bolt pattern (45-degree orientation)
        current_bcd = is_motor_mount ? motor_mount_bcd : bolt_circle_diam;
        for (r = [45, 135, 225, 315])
            rotate([0, 0, r])
            translate([current_bcd / 2, 0, 0])
            cylinder(h = plate_t * 2, d = bolt_hole_diam, center = true);
    }
}


// ============================================================
// MODULES - Bearing
// ============================================================

// Total bearing height for cut-through
bearing_total_h = bearing_flange_h + bearing_housing_len;

module bearing_4bolt() {
    color("Green") {
        difference() {
            union() {
                // Flange
                hull() {
                    for (r = [45, 135, 225, 315])
                        rotate([0, 0, r])
                        translate([bolt_circle_diam / 2, 0, 0])
                        cylinder(h = bearing_flange_h, d = bearing_flange_lobe_diam);
                    cylinder(h = bearing_flange_h, d = bearing_housing_diam);
                }
                // Housing
                translate([0, 0, bearing_flange_h / 2])
                cylinder(h = bearing_housing_len, d = bearing_housing_diam);
            }
            // Shaft hole
            translate([0, 0, -eps])
            cylinder(h = bearing_total_h + 2 * eps, d = shaft_diam);

            // Bolt holes
            for (r = [45, 135, 225, 315])
                rotate([0, 0, r])
                translate([bolt_circle_diam / 2, 0, 0])
                cylinder(h = bearing_flange_h + 2 * eps, d = bolt_hole_diam, center = true);
        }
    }
}


// ============================================================
// MODULES - Motor & Coupling
// ============================================================

module hydraulic_motor() {
    color("Orange") {
        // Body
        translate([0, 0, -motor_body_len])
        cylinder(h = motor_body_len, d = motor_body_diam);

        // Flange
        difference() {
            translate([0, 0, -motor_flange_t / 2])
            cube([motor_flange_size, motor_flange_size, motor_flange_t], center = true);

            for (r = [45, 135, 225, 315])
                rotate([0, 0, r])
                translate([motor_mount_bcd / 2, 0, -motor_flange_t / 2])
                cylinder(h = motor_flange_t + 2 * eps, d = bolt_hole_diam, center = true);
        }

        // Shaft
        cylinder(h = motor_shaft_len, d = motor_shaft_diam);
    }
}

module shaft_coupling() {
    color("DimGray")
    cylinder(h = coupling_len, d = coupling_diam);
}

// ============================================================
// MODULES - Shaft & Collar
// ============================================================

module main_shaft(length = shaft_length) {
    color("Silver")
    cylinder(h = length, d = shaft_diam);
}

module shaft_collar() {
    color("SteelBlue")
    difference() {
        cylinder(h = collar_width, d = collar_od);
        translate([0, 0, -eps])
        cylinder(h = collar_width + 2 * eps, d = shaft_diam);
    }
}

// ============================================================
// MODULES - Sprocket (chain sprocket with filleted tooth roots)
// ============================================================

// Custom chain sprocket: teeth engage connecting rods,
// root circle sits on track bar, fillets at tooth base = rod radius
module sprocket(phase = 0) {
    color("Gold")
    rotate([0, 0, phase])
    translate([0, 0, -sprocket_thickness / 2])
    linear_extrude(height = sprocket_thickness) {
        difference() {
            // Filleted tooth profile:
            // offset(r=-fillet_r) shrinks shape, rounding concave (internal) corners
            // offset(r=+fillet_r) expands back, preserving the rounded roots
            offset(r = fillet_r)
            offset(r = -fillet_r)
            difference() {
                // Outer disk to tip radius
                circle(r = sprocket_tip_r);

                // Rod seat cutouts at each tooth position
                for (i = [0 : num_teeth - 1])
                    rotate([0, 0, i * 360 / num_teeth])
                    translate([sprocket_pitch_radius, 0])
                    circle(r = rod_seat_r);
            }

            // Center bore
            circle(d = shaft_diam);
        }
    }
}

// ============================================================
// MODULES - Track Link
// ============================================================

// Single track link (one foot of the track chain)
// is_inner: true = connector plates at inner stagger position
//           false = connector plates at outer stagger position
module track_link(is_inner = true) {
    offset = is_inner ? inner_plate_offset : outer_plate_offset;

    // Ground contact bar: 10" (Y) x 2.5" (X) x 0.5" (Z)
    color("DarkGray")
    translate([-track_bar_width / 2, -track_bar_length / 2, 0])
        cube([track_bar_width, track_bar_length, track_bar_thickness]);

    // Two connector plates welded perpendicular to bar
    // Plates stand vertical (Z), positioned along Y (bar length axis)
    // Each plate has TWO rod holes: front (-X) and rear (+X)
    // Circular pads around rod holes (union), then rod holes subtracted
    color("Gray")
    for (side = [-1, 1]) {
        translate([0, side * offset, track_bar_thickness]) {
            difference() {
                union() {
                    // Rectangular plate spanning full chain pitch
                    translate([-connector_width / 2, -connector_thickness / 2, 0])
                        cube([connector_width, connector_thickness, connector_height]);

                    // Circular pad at front end (-X) centered on rod hole
                    translate([-rod_hole_x_offset, 0, connector_height / 2])
                    rotate([90, 0, 0])
                        cylinder(h = connector_thickness, d = connector_height,
                                 center = true);

                    // Circular pad at rear end (+X) centered on rod hole
                    translate([rod_hole_x_offset, 0, connector_height / 2])
                    rotate([90, 0, 0])
                        cylinder(h = connector_thickness, d = connector_height,
                                 center = true);
                }

                // Front rod hole (-X side, connects to previous link)
                translate([-rod_hole_x_offset, 0, connector_height / 2])
                rotate([90, 0, 0])
                    cylinder(h = connector_thickness + 2 * eps, d = link_rod_diam + rod_hole_clearance,
                             center = true);

                // Rear rod hole (+X side, connects to next link)
                translate([rod_hole_x_offset, 0, connector_height / 2])
                rotate([90, 0, 0])
                    cylinder(h = connector_thickness + 2 * eps, d = link_rod_diam + rod_hole_clearance,
                             center = true);
            }
        }
    }
}

// Connecting rod between adjacent track links
module connecting_rod() {
    color("DimGray")
    rotate([90, 0, 0])
    translate([0, 0, -link_rod_length / 2])
        cylinder(h = link_rod_length, d = link_rod_diam);
}

// ============================================================
// MODULES - Full Track Chain (wraps all sprockets)
// ============================================================

// Animation offset: $t goes 0→1, shift chain by one full pitch per cycle
anim_offset = animate_track ? $t * chain_pitch : 0;

// Sprocket animation: rotate by equivalent angle for chain travel
anim_sprocket_angle = animate_track ? $t * tooth_angle : 0;

module track_chain_full() {
    for (i = [0 : num_chain_links - 1]) {
        // Rod positions bounding this link (on pitch circle)
        s0 = i * chain_pitch + anim_offset;
        s1 = (i + 1) * chain_pitch + anim_offset;
        p0 = chain_point(s0);
        p1 = chain_point(s1);

        // Chord midpoint and angle
        mx = (p0[0] + p1[0]) / 2;
        mz = (p0[1] + p1[1]) / 2;
        th = atan2(p1[1] - p0[1], p1[0] - p0[0]);

        // Place link: bar offset perpendicular outward from rod center line
        translate([mx + rod_z_above_bar * sin(th),
                   0,
                   mz - rod_z_above_bar * cos(th)])
        rotate([0, -th, 0])
            track_link(is_inner = (i % 2 == 0));

        // Connecting rod at rod i (on pitch circle)
        translate([p0[0], 0, p0[1]])
            connecting_rod();
    }
}

// ============================================================
// MODULES - Axis Assemblies
// ============================================================

// Idler axis: shaft + 2 bearing plates + sprocket + shaft collars
// Oriented with shaft along Y axis, sprocket in XZ plane
module idler_axis_assembly(phase = 0) {
    idler_bearing_spacing = bearing_plate_spacing_2_3;

    // Shaft (along Y)
    color("Silver")
    rotate([90, 0, 0])
    translate([0, 0, -shaft_length / 2])
        cylinder(h = shaft_length, d = shaft_diam);

    // Bearing plate 1 (+Y side)
    translate([0, idler_bearing_spacing / 2 + plate_t / 2, 0])
    rotate([90, 0, 0]) {
        plate(is_motor_mount = false);
        translate([0, 0, plate_t / 2])
            bearing_4bolt();
    }

    // Bearing plate 2 (-Y side)
    translate([0, -(idler_bearing_spacing / 2 + plate_t / 2), 0])
    rotate([-90, 0, 0]) {
        plate(is_motor_mount = false);
        translate([0, 0, plate_t / 2])
            bearing_4bolt();
    }

    // Sprocket (centered, in XZ plane)
    rotate([90, 0, 0])
        sprocket(phase + anim_sprocket_angle);

    // Shaft collars sandwiching sprocket
    translate([0, sprocket_thickness / 2 + collar_gap, 0])
    rotate([90, 0, 0])
        shaft_collar();

    translate([0, -(sprocket_thickness / 2 + collar_width + collar_gap), 0])
    rotate([90, 0, 0])
        shaft_collar();
}


// Drive axis: shaft + motor + bearing plates + sprocket + coupling
// Motor on outside (+Y), bearings symmetrically spaced like idlers
module drive_axis_assembly(phase = 0) {
    drive_bearing_spacing = bearing_plate_spacing_2_3;

    // Shaft (along Y)
    color("Silver")
    rotate([90, 0, 0])
    translate([0, 0, -shaft_length / 2])
        cylinder(h = shaft_length, d = shaft_diam);

    // Bearing plate 1 (+Y side, same as idler)
    translate([0, drive_bearing_spacing / 2 + plate_t / 2, 0])
    rotate([90, 0, 0]) {
        plate(is_motor_mount = false);
        translate([0, 0, plate_t / 2])
            bearing_4bolt();
    }

    // Bearing plate 2 (-Y side, same as idler)
    translate([0, -(drive_bearing_spacing / 2 + plate_t / 2), 0])
    rotate([-90, 0, 0]) {
        plate(is_motor_mount = false);
        translate([0, 0, plate_t / 2])
            bearing_4bolt();
    }

    // Motor mount plate (outside +Y, beyond bearing plate 1)
    motor_plate_y = drive_bearing_spacing / 2 + plate_t + bearing_plate_spacing_1_2;
    translate([0, motor_plate_y, 0])
    rotate([90, 0, 0]) {
        plate(is_motor_mount = true);
        // Motor on outer face (+Z in rotated frame = +Y in world)
        translate([0, 0, plate_t / 2])
            hydraulic_motor();
    }

    // Shaft coupling (between motor plate and bearing plate 1)
    coupling_y = motor_plate_y - plate_t / 2 - coupling_len / 2;
    translate([0, coupling_y, 0])
    rotate([-90, 0, 0])
        shaft_coupling();

    // Sprocket (centered between bearing plates, same as idler)
    rotate([90, 0, 0])
        sprocket(phase + anim_sprocket_angle);

    // Shaft collars sandwiching sprocket
    translate([0, sprocket_thickness / 2 + collar_gap, 0])
    rotate([90, 0, 0])
        shaft_collar();

    translate([0, -(sprocket_thickness / 2 + collar_width + collar_gap), 0])
    rotate([90, 0, 0])
        shaft_collar();
}

// ============================================================
// ASSEMBLY
// ============================================================

// --- Left Idler (bottom, adjusted X for pitch fit) ---
translate([left_idler_x, 0, idler_shaft_z])
    idler_axis_assembly(phase_A);

// --- Right Idler (bottom, X=idler_spacing) ---
translate([idler_spacing, 0, idler_shaft_z])
    idler_axis_assembly(phase_B);

// --- Drive Axis (top center) ---
if (num_axes >= 3) {
    translate([drive_x, 0, drive_z])
        drive_axis_assembly(phase_C);
}

// --- Track Chain (full loop) ---
if (show_track) {
    track_chain_full();
}

// ============================================================
// INFO ECHO
// ============================================================

echo(str("--- UTU-v25 Summary ---"));
echo(str("Axes: ", num_axes));
echo(str("Idler spacing (nominal): ", idler_spacing / inch, " inches"));
echo(str("Front idler adjust: ", front_idler_adjust, " mm (",
         front_idler_adjust / inch, " in)"));
echo(str("Left idler X: ", left_idler_x, " mm"));
echo(str("Total chain links: ", num_chain_links));
echo(str("Chain pitch: ", chain_pitch / inch, " in (", chain_pitch, " mm)"));
echo(str("Sprocket pitch radius: ", sprocket_pitch_radius / inch, " in"));
echo(str("Sprocket OD (tip): ", sprocket_od / inch, " in"));
echo(str("Wrap angles: A=", wrap_A, "° B=", wrap_B, "° C=", wrap_C, "°"));
echo(str("Track bar: ", track_bar_length / inch, "\" x ",
         track_bar_width / inch, "\" x ", track_bar_thickness / inch, "\""));
echo(str("Idler shaft Z: ", idler_shaft_z / inch, " in"));
echo(str("Drive shaft Z: ", drive_z / inch, " in"));
echo(str("Chain path (arc approx): ", chain_path_len, " mm"));
