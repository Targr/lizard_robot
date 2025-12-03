// lizard_all_parts.scad
// All URDF parts as separate 3D-printable solids (to-scale, mm)
// - body_0..body_4, tail_0..tail_2, head, jaw, hip/knee for 4 legs, eyes, electronics box,
//   plus servo/horn/bracket/pca placeholders.
// Units: millimeters
// Author: ChatGPT (GPT-5 Thinking mini)

$fn = 64;

// ----------------- SCALE & GEOMETRY (converted from URDF meters -> mm) -----------------
scale_m = 1.0; // URDF scale = 1.0 (keep at 1.0 for to-scale)
body_length = 0.35 * 1000 * scale_m;   // 350 mm
body_width  = 0.12 * 1000 * scale_m;   // 120 mm
body_height = 0.08 * 1000 * scale_m;   // 80 mm

tail_length = 0.22 * 1000 * scale_m;   // 220 mm
tail_w = 0.08 * 1000 * scale_m;        // 80 mm (thickness in URDF)
tail_h = 0.06 * 1000 * scale_m;        // 60 mm

leg_length = 0.22 * 1000 * scale_m;    // 220 mm
leg_thin_w = 0.04 * 1000 * scale_m;    // 40 mm
leg_thinner_w = 0.03 * 1000 * scale_m; // 30 mm

head_length = 0.18 * 1000 * scale_m;   // 180 mm
head_width  = 0.12 * 1000 * scale_m;   // 120 mm
head_height = 0.06 * 1000 * scale_m;   // 60 mm

jaw_length = head_length * 1.05;       // 189 mm
jaw_width  = 0.075 * 1000 * scale_m;   // 75 mm
jaw_height = 0.05 * 1000 * scale_m;    // 50 mm

eye_radius = 0.02 * 1000 * scale_m;    // 20 mm

// Electronics box from URDF visual: 0.06 x 0.04 x 0.02 m -> 60 x 40 x 20 mm
elec_x = 60;
elec_y = 40;
elec_z = 20;

// Servo, horn, PCA approximations (for mounting checks)
servo_w = 22.8;
servo_d = 12.2;
servo_h = 28.5;
horn_diameter = 20;
horn_thickness = 3;
pca_len = 66;
pca_wid = 52;
pca_thk = 4;

// Printing/layout settings
grid_x = 220;
grid_y = 180;
z_clear = 2; // small lift above bed when visualizing

// ----------------- PART MODULES -----------------

// Generic body segment module (one printable solid representing a URDF body_i)
module body_segment_solid(index=0) {
    // outer block sized to URDF body box
    seg_len = body_length;
    seg_w = body_width;
    seg_h = body_height;

    // Make body a hollow shell (printable) — subtract interior but keep part independent
    outer = translate([-seg_len/2, -seg_w/2, 0]) cube([seg_len, seg_w, seg_h]);
    inner = translate([-seg_len/2 + 6, -seg_w/2 + 6, 6]) cube([seg_len - 12, seg_w - 12, seg_h - 8]);
    difference() {
        outer;
        inner;
    }
}

// Tail segment (solid with box geometry used in URDF)
module tail_segment_solid(index=0) {
    translate([-tail_length/2, -tail_w/2, 0])
        cube([tail_length, tail_w, tail_h]);
}

// Head piece (solid)
module head_solid() {
    translate([-head_length/2, -head_width/2, 0])
        cube([head_length, head_width, head_height]);
}

// Jaw piece (separate)
module jaw_solid() {
    translate([-jaw_length/2, -jaw_width/2, 0])
        cube([jaw_length, jaw_width, jaw_height]);
}

// Hip piece (for each leg) — approximated as a rectangular link sized like hip visual in URDF
module hip_solid(leg_label = "hip", is_right=false) {
    translate([-leg_length/2, -leg_thin_w/2, 0]) cube([leg_length, leg_thin_w, leg_thin_w]);
}

// Knee piece
module knee_solid() {
    translate([-leg_length/2, -leg_thinner_w/2, 0]) cube([leg_length, leg_thinner_w, leg_thinner_w]);
}

// Eye sphere
module eye_solid(side="L") {
    translate([0,0,eye_radius]) sphere(r=eye_radius);
}

// Electronics box (separate)
module electronics_box_solid() {
    difference() {
        translate([-elec_x/2, -elec_y/2, 0]) cube([elec_x, elec_y, elec_z]);
        translate([-elec_x/2 + 2, -elec_y/2 + 2, 2]) cube([elec_x - 4, elec_y - 4, elec_z - 3]);
    }
    // mounting boss placeholders (four small cylinders)
    for (x=[-elec_x/2 + 8, elec_x/2 - 8]) for (y=[-elec_y/2 + 8, elec_y/2 - 8])
        translate([x,y,elec_z/2]) rotate([90,0,0]) cylinder(h=6, r=3/2);
}

// MG90S servo placeholder (separate solid)
module mg90s_solid() {
    translate([-servo_w/2, -servo_d/2, 0]) cube([servo_w, servo_d, servo_h]);
    // shaft nub
    translate([0,0,servo_h]) cylinder(h=3, r=1.6);
}

// Servo horn (separate)
module servo_horn_solid() {
    difference() {
        translate([-horn_diameter/2, -horn_diameter/2, 0]) cylinder(h=horn_thickness, r=horn_diameter/2, center=true);
        translate([0,0,-1]) cylinder(h=horn_thickness+2, r=3.2/2, center=true);
    }
}

// PCA placeholder
module pca_solid() {
    translate([-pca_len/2, -pca_wid/2, 0]) cube([pca_len, pca_wid, pca_thk]);
}

// Servo bracket placeholder
module servo_bracket_solid() {
    b_w = servo_w + 28;
    b_d = servo_d + 24;
    b_h = 8;
    translate([-b_w/2, -b_d/2, 0]) cube([b_w, b_d, b_h]);
}


// ----------------- LAYOUT: place every part separately on grid -----------------
module layout_all() {
    // Row & column counters to place parts cleanly
    // Row 0: body_0 .. body_4
    for (i = [0:4]) {
        tx = i * grid_x;
        ty = 0;
        translate([tx, ty, z_clear]) body_segment_solid(i);
    }

    // Row 1: tail segments 0..2
    for (t = [0:2]) {
        tx = t * grid_x;
        ty = -grid_y;
        translate([tx, ty, z_clear]) tail_segment_solid(t);
    }

    // Row 2: head and jaw
    translate([0, -2*grid_y, z_clear]) head_solid();
    translate([grid_x, -2*grid_y, z_clear]) jaw_solid();

    // Row 3: legs: front_left, front_right, rear_left, rear_right — hip + knee each
    leg_names = ["front_left","front_right","rear_left","rear_right"];
    for (li = [0:3]) {
        tx = li * grid_x;
        ty = -3*grid_y;
        // hip
        translate([tx, ty, z_clear]) hip_solid();
        // knee next to it
        translate([tx, ty - grid_y*0.25, z_clear]) knee_solid();
    }

    // Row 4: eyes (two)
    translate([0, -4*grid_y, z_clear]) eye_solid("L");
    translate([grid_x, -4*grid_y, z_clear]) eye_solid("R");

    // Row 5: electronics box + PCA + servo rack + brackets + servo + horn
    translate([0, -5*grid_y, z_clear]) electronics_box_solid();
    translate([grid_x, -5*grid_y, z_clear]) pca_solid();
    translate([2*grid_x, -5*grid_y, z_clear]) mg90s_solid();
    translate([3*grid_x, -5*grid_y, z_clear]) servo_horn_solid();
    translate([4*grid_x, -5*grid_y, z_clear]) servo_bracket_solid();
}

// ----------------- TOP LEVEL -----------------
// Render everything (each part is separate physically). To export a single part,
// replace the call below with the single module you want (e.g. "head_solid();").
// By default we render the entire layout grid so you can pick parts and export.
layout_all();

// ----------------- QUICK USAGE -----------------
// To export a single part to STL:
// 1) comment out layout_all(); and uncomment e.g. head_solid(); or translate([...]) head_solid();
// 2) Render (F6) and then Design -> Export -> Export as STL.
//
// If you'd like, I can:
// - provide a version that auto-exports every module to separate STL files using OpenSCAD CLI commands (batch),
// - add more precise cutouts / holes where servos will mount to each body/leg piece so they can be assembled with MG90S (press-fit or screw),
// - shrink the whole robot uniformly (if 350 mm is too large).
