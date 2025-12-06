// lizard_parts.scad
// OpenSCAD recreation of generated lizard parts (units: mm).
// Matches dimensions used previously in the Python STLs.

// -------------------------- PARAMETERS --------------------------
MG90S_length = 22.8;
MG90S_width  = 12.2;
MG90S_height = 28.5;

servo_clearance = [MG90S_length + 4, MG90S_width + 6, MG90S_height]; // body box with clearance

// Servo mounting screw info
mount_spacing = 16.0;   // center-to-center along length (mm)
mount_clearance_d = 2.2; // cylinder clearance for M2 screw (mm)
countersink_outer_d = 4.5; // outer diameter of countersink
countersink_depth = 1.8;   // depth of countersink

// Servo horn / hub
horn_teeth = 20;         // tooth count (20T as requested)
hub_outer_d = 4.9;       // nominal hub outer diameter (mm)
hub_inner_d = 3.0;       // valley diameter approx
hub_thickness = 4.0;     // hub thickness (mm)
horn_arm_len = 18.0;     // arm length (mm)
horn_arm_w = 6.0;        // arm width (mm)
horn_arm_th = hub_thickness; // thickness to match hub

// PCA9685 board approx dims
pca_length = 62.5;
pca_width  = 25.4;
pca_thick  = 3.0;

// Electronics enclosure
elec_x = 60; elec_y = 40; elec_z = 20;

// Body & tail segments (toy scale)
body_total_length = 120.0; // mm
body_segments = 5;
body_seg_len = body_total_length / body_segments;
body_seg_w = 40;
body_seg_h = 20;

tail_total_length = 90.0;
tail_segments = 3;
tail_seg_len = tail_total_length / tail_segments;
tail_seg_w0 = 20;
tail_seg_h0 = 12;

// Head & jaw
head_x = 40; head_y = 30; head_z = 18;
jaw_x = 38; jaw_y = 20; jaw_z = 12;

// Leg blocks
hip_box = [30, 12, 10];
knee_box = [28, 10, 8];

// Spine connectors
spine_box = [10, 12, 14];

// -------------------------- HELPERS --------------------------
module box(s = [10,10,10]) {
    translate([-s[0]/2, -s[1]/2, -s[2]/2]) cube(s, center=false);
}

module cylinder_centered(d = 2, h = 4, $fn=64) {
    translate([0,0,-h/2]) cylinder(d=d, h=h, center=false, $fn=$fn);
}

// create an n-point star polygon for spline approximation
function spline_points(teeth, outer_r, inner_r) =
    [ for(i=[0:teeth*2-1]) let (ang = i*360/(teeth*2))
        ( ( (i % 2 == 0) ? outer_r : inner_r ) * [cos(ang), sin(ang)] )
    ];

// -------------------------- SERVO BODY + HOLES --------------------------
module servo_body() {
    // basic servo body (box centered at origin)
    box(servo_clearance);
}

module servo_mount_holes() {
    // two through-hole cylinders centered on Y=0, spaced along X
    translate([-mount_spacing/2, 0, 0]) rotate([0,0,0]) cylinder(d=mount_clearance_d, h=MG90S_height+8, $fn=64, center=true);
    translate([ mount_spacing/2, 0, 0]) cylinder(d=mount_clearance_d, h=MG90S_height+8, $fn=64, center=true);
}

module countersink(offsetX) {
    // a shallow cone-like countersink (approx with difference of two cones)
    // simple approximation: union of a shallow cone and short cylinder
    translate([offsetX,0,0]) difference() {
        // outer shallow cone: use linear_extrude on 2D circle approximation
        translate([0,0,0]) rotate_extrude($fn=64) translate([countersink_outer_d/2, 0, 0]) square([0.001, countersink_depth], center=false);
        // remove inner cylindrical hole
        translate([0,0,-1]) cylinder(d=mount_clearance_d, h=countersink_depth+2, $fn=64, center=false);
    }
}

module servo_with_holes() {
    // servo body with countersunk mounting holes (holes subtracted)
    difference() {
        servo_body();
        // subtract through-hole cylinders (extend beyond box)
        translate([ -mount_spacing/2, 0, 0 ]) rotate([0,0,0]) cylinder(d=mount_clearance_d, h=servo_clearance[2]+10, center=true, $fn=64);
        translate([  mount_spacing/2, 0, 0 ]) rotate([0,0,0]) cylinder(d=mount_clearance_d, h=servo_clearance[2]+10, center=true, $fn=64);
        // subtract countersink cones (approximated with scaled cones)
        translate([-mount_spacing/2,0,servo_clearance[2]/2 - countersink_depth/2]) rotate([0,0,0]) 
            cylinder(d1=countersink_outer_d, d2=mount_clearance_d, h=countersink_depth, $fn=64);
        translate([ mount_spacing/2,0,servo_clearance[2]/2 - countersink_depth/2]) rotate([0,0,0]) 
            cylinder(d1=countersink_outer_d, d2=mount_clearance_d, h=countersink_depth, $fn=64);
    }
}

// -------------------------- SERVO HORN (20T approximate spline) --------------------------
module servo_horn_20T(teeth = horn_teeth, outer_d = hub_outer_d, inner_d = hub_inner_d, thickness = hub_thickness, arm_len = horn_arm_len, arm_w = horn_arm_w) {
    // Build spline hub by extruding a 2D star polygon
    outer_r = outer_d/2;
    inner_r = inner_d/2;
    pts = spline_points(teeth, outer_r, inner_r);
    // make 2D polygon with center at 0,0
    translate([0,0,0]) linear_extrude(height=thickness, center=true, convexity=10)
        polygon(points = pts);
    // add rectangular arm attached to hub
    translate([ (outer_r + arm_len/2), 0, 0 ]) cube([arm_len, arm_w, thickness], center=true);
}

// -------------------------- OTHER PARTS --------------------------
module pca9685_board() {
    translate([0,0,pca_thick/2]) cube([pca_length, pca_width, pca_thick], center=true);
}

module electronics_enclosure() {
    translate([0,0,elec_z/2]) cube([elec_x, elec_y, elec_z], center=true);
}

module body_segment(i=0) {
    // i is index (0..body_segments-1)
    translate([0,0,0]) box([body_seg_len, body_seg_w, body_seg_h]);
}

module tail_segment(i=0) {
    // taper width/height with segment index
    w = tail_seg_w0 - i*3;
    h = tail_seg_h0 - i*2;
    translate([0,0,0]) box([tail_seg_len, w, h]);
}

module head_piece() { box([head_x, head_y, head_z]); }
module jaw_piece()  { box([jaw_x, jaw_y, jaw_z]); }

module hip_piece(side="L") {
    box(hip_box);
}

module knee_piece(side="L") {
    box(knee_box);
}

module spine_connector(i=0) {
    box(spine_box);
}

// -------------------------- ASSEMBLY EXAMPLES --------------------------
// Uncomment/modify any of these to render a single part for export.
// Use F6 (Render) and then Export as STL to get an STL per part.

// Render servo body (box only)
module servo_body_only_for_export() {
    servo_body();
}

// Render servo body WITH holes (ready-to-print with countersinks)
module servo_body_with_holes_for_export() {
    servo_with_holes();
}

// Render separate hole solids (useful if you want to boolean in another tool)
module servo_holes_for_export() {
    translate([-mount_spacing/2,0,0]) cylinder(d=mount_clearance_d, h=servo_clearance[2]+10, center=true, $fn=64);
    translate([ mount_spacing/2,0,0]) cylinder(d=mount_clearance_d, h=servo_clearance[2]+10, center=true, $fn=64);
    translate([-mount_spacing/2,0,servo_clearance[2]/2 - countersink_depth/2]) cylinder(d1=countersink_outer_d, d2=mount_clearance_d, h=countersink_depth, $fn=64);
    translate([ mount_spacing/2,0,servo_clearance[2]/2 - countersink_depth/2]) cylinder(d1=countersink_outer_d, d2=mount_clearance_d, h=countersink_depth, $fn=64);
}

// Render servo horn
module servo_horn_for_export() {
    servo_horn_20T();
}

// Example: Layout a few parts on the build plate for visual checks
module layout_example() {
    // servo with holes on the left
    translate([-60,0,0]) servo_body_with_holes_for_export();
    // horn to the right
    translate([40,0,0]) servo_horn_for_export();
    // PCA board behind
    translate([0,-60,0]) pca9685_board();
}

// -------------------------- USER CHOICE: what to render --------------------------
// Change this string to the module name you want to render and export as STL.
// Options: "servo_body_only", "servo_body_with_holes", "servo_holes", "servo_horn", "pca", "elec", "body_segments", "tail_segments", "head", "jaw", "legs", "spine", "layout"
generate_separate_part = "layout";

if (generate_separate_part == "servo_body_only") {
    servo_body_only_for_export();
} else if (generate_separate_part == "servo_body_with_holes") {
    servo_body_with_holes_for_export();
} else if (generate_separate_part == "servo_holes") {
    servo_holes_for_export();
} else if (generate_separate_part == "servo_horn") {
    servo_horn_for_export();
} else if (generate_separate_part == "pca") {
    pca9685_board();
} else if (generate_separate_part == "elec") {
    electronics_enclosure();
} else if (generate_separate_part == "body_segments") {
    translate([- (body_total_length/2) + body_seg_len/2, 0, 0])
    for(i=[0:body_segments-1]) {
        translate([i*body_seg_len,0,0]) body_segment(i);
    }
} else if (generate_separate_part == "tail_segments") {
    translate([- (tail_total_length/2) + tail_seg_len/2, 0, 0])
    for(i=[0:tail_segments-1]) {
        translate([i*tail_seg_len,0,0]) tail_segment(i);
    }
} else if (generate_separate_part == "head") {
    head_piece();
} else if (generate_separate_part == "jaw") {
    jaw_piece();
} else if (generate_separate_part == "legs") {
    translate([-30,0,0]) hip_piece();
    translate([30,0,0]) knee_piece();
} else if (generate_separate_part == "spine") {
    for(i=[0:3]) translate([i*15,0,0]) spine_connector(i);
} else {
    // default layout
    layout_example();
}
