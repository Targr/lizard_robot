//
// SNAP-FIT / LEGO-STYLE LIZARD ROBOT (Expanded Grid)
// MG90S servos + peg/socket connectors
// Every part spaced FAR apart — zero overlap
// --------------------------------------------------

// ---------- PARAMETERS ----------
PEG_D   = 0.005;     // 5 mm peg diameter
PEG_TOL = 0.0006;    // female socket diameter = PEG_D + PEG_TOL
PEG_LEN = 0.012;     // 12 mm peg length

SERVO_CLEAR = 0.0012;

// MG90S
SERVO_L = 0.0228;
SERVO_W = 0.0122;
SERVO_H = 0.0285;

// geometry
body_L = 0.35;
body_W = 0.12;
body_H = 0.08;

tail_L = 0.22;
tail_W = 0.08;
tail_H = 0.06;

head_L = 0.18;
head_W = 0.12;
head_H = 0.06;

leg_L = 0.22;

hip_y = 0.12;
hip_z = 0.02;

// grid spacing
PART_SPACING_X = 0.60;   // WAS 1 — now bigger for safety
PART_SPACING_Y = 0.60;


// ---------- PRIMITIVES ----------
module peg() {
    cylinder(h = PEG_LEN, r = PEG_D/2, $fn=32);
}

module socket() {
    cylinder(h = PEG_LEN+0.004, r = (PEG_D+PEG_TOL)/2, $fn=32);
}

module servo_pocket_y() {
    pocket_L = SERVO_L + 2*SERVO_CLEAR;
    pocket_W = SERVO_W + 2*SERVO_CLEAR;
    pocket_H = SERVO_H*0.95;

    translate([-pocket_W/2, -pocket_L/2, -pocket_H/2])
        cube([pocket_W, pocket_L, pocket_H]);
}


// ---------- BODY SEGMENT ----------
module body_segment(seg_index = 0) {

    difference() {
        translate([-body_L/2, -body_W/2, 0])
            cube([body_L, body_W, body_H]);

        // SERVO POCKETS (hips)
        translate([0.02, body_W/2, hip_z + body_H/2])
            rotate([0,0,90]) servo_pocket_y();

        translate([0.02, -body_W/2, hip_z + body_H/2])
            rotate([0,0,-90]) servo_pocket_y();

        // FRONT SOCKETS
        translate([body_L/2 - PEG_LEN/2, +0.03, body_H/2])
            rotate([0,90,0]) socket();

        translate([body_L/2 - PEG_LEN/2, -0.03, body_H/2])
            rotate([0,90,0]) socket();
    }

    // BACK PEGS
    translate([-body_L/2, +0.03, body_H/2])
        rotate([0,90,0]) peg();

    translate([-body_L/2, -0.03, body_H/2])
        rotate([0,90,0]) peg();
}


// ---------- HEAD ----------
module head_piece() {

    difference() {

        translate([-head_L/2, -head_W/2, 0])
            cube([head_L, head_W, head_H]);

        translate([-head_L*0.25, 0, head_H/2])
            rotate([0,0,90]) servo_pocket_y();

        translate([head_L/2 - PEG_LEN/2, +0.03, head_H/2])
            rotate([0,90,0]) socket();

        translate([head_L/2 - PEG_LEN/2, -0.03, head_H/2])
            rotate([0,90,0]) socket();
    }

    translate([-head_L/2, +0.02, head_H*0.35])
        rotate([0,90,0]) peg();

    translate([-head_L/2, -0.02, head_H*0.35])
        rotate([0,90,0]) peg();
}


// ---------- JAW ----------
module jaw_piece() {

    difference() {
        cube([0.189, 0.075, 0.05]);

        translate([0.189 - PEG_LEN/2, +0.02, 0.025])
            rotate([0,90,0]) socket();

        translate([0.189 - PEG_LEN/2, -0.02, 0.025])
            rotate([0,90,0]) socket();
    }
}


// ---------- TAIL ----------
module tail_seg() {

    difference() {

        translate([-tail_L/2, -tail_W/2, 0])
            cube([tail_L, tail_W, tail_H]);

        translate([ tail_L/2 - SERVO_L/2 - 0.005, 0, tail_H/2 ])
            rotate([0,0,90]) servo_pocket_y();

        translate([ tail_L/2 - PEG_LEN/2, +0.02, tail_H/2])
            rotate([0,90,0]) socket();

        translate([ tail_L/2 - PEG_LEN/2, -0.02, tail_H/2])
            rotate([0,90,0]) socket();
    }

    translate([-tail_L/2, +0.02, tail_H/2])
        rotate([0,90,0]) peg();

    translate([-tail_L/2, -0.02, tail_H/2])
        rotate([0,90,0]) peg();
}


// ---------- HIP ----------
module hip_piece() {

    difference() {

        translate([-leg_L/2, -0.04/2, 0])
            cube([leg_L, 0.04, 0.04]);

        translate([0, 0.04/2, 0.02])
            rotate([0,0,90]) servo_pocket_y();

        translate([-leg_L/2 + PEG_LEN/2, 0, 0.02])
            socket();
    }

    translate([leg_L/2, 0, 0.02])
        peg();
}


// ---------- KNEE ----------
module knee_piece() {

    difference() {

        translate([-leg_L/2, -0.03/2, 0])
            cube([leg_L, 0.03, 0.03]);

        translate([leg_L*0.45, 0, -0.015])
            rotate([0,0,90]) servo_pocket_y();

        translate([-leg_L/2 + PEG_LEN/2, 0, 0.015])
            socket();
    }

    translate([leg_L/2, 0, 0.015])
        peg();
}


// ---------- SERVO BRACKET ----------
module bracket_mg90s() {
    difference() {
        translate([-0.025/2, -0.012/2, 0]) cube([0.025,0.012,0.006]);
        translate([-0.006,0,0.003]) rotate([90,0,0]) cylinder(h=0.012, r=0.0018);
        translate([ 0.006,0,0.003]) rotate([90,0,0]) cylinder(h=0.012, r=0.0018);
    }
}


// =========================================================
// EXPORT GRID — WIDELY SPACED PARTS (NO OVERLAP)
// =========================================================

translate([0*PART_SPACING_X, 0*PART_SPACING_Y, 0]) body_segment(0);
translate([1*PART_SPACING_X, 0*PART_SPACING_Y, 0]) body_segment(1);
translate([2*PART_SPACING_X, 0*PART_SPACING_Y, 0]) body_segment(2);
translate([3*PART_SPACING_X, 0*PART_SPACING_Y, 0]) body_segment(3);
translate([4*PART_SPACING_X, 0*PART_SPACING_Y, 0]) body_segment(4);

translate([0*PART_SPACING_X, 1*PART_SPACING_Y, 0]) head_piece();
translate([1*PART_SPACING_X, 1*PART_SPACING_Y, 0]) jaw_piece();

translate([0*PART_SPACING_X, 2*PART_SPACING_Y, 0]) tail_seg();
translate([1*PART_SPACING_X, 2*PART_SPACING_Y, 0]) tail_seg();
translate([2*PART_SPACING_X, 2*PART_SPACING_Y, 0]) tail_seg();

translate([0*PART_SPACING_X, 3*PART_SPACING_Y, 0]) hip_piece();
translate([1*PART_SPACING_X, 3*PART_SPACING_Y, 0]) hip_piece();
translate([2*PART_SPACING_X, 3*PART_SPACING_Y, 0]) hip_piece();
translate([3*PART_SPACING_X, 3*PART_SPACING_Y, 0]) hip_piece();

translate([0*PART_SPACING_X, 4*PART_SPACING_Y, 0]) knee_piece();
translate([1*PART_SPACING_X, 4*PART_SPACING_Y, 0]) knee_piece();
translate([2*PART_SPACING_X, 4*PART_SPACING_Y, 0]) knee_piece();
translate([3*PART_SPACING_X, 4*PART_SPACING_Y, 0]) knee_piece();

translate([0*PART_SPACING_X, 5*PART_SPACING_Y, 0]) bracket_mg90s();
