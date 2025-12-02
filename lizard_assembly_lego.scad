//
// SNAP-FIT / LEGO-STYLE LIZARD ROBOT
// MG90S servos + peg/socket connectors
// Each part isolated for STL export
// --------------------------------------------------

// ---------- PARAMETERS ----------
PEG_D   = 0.005;     // 5 mm peg diameter
PEG_TOL = 0.0006;    // female socket diameter = PEG_D + PEG_TOL
PEG_LEN = 0.012;     // peg length (12 mm)

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
// Contains:
// - 2 pegs on back
// - 2 sockets on front
// - hip servo pockets
module body_segment(seg_index = 0) {

    difference() {
        // main block
        translate([-body_L/2, -body_W/2, 0])
            cube([body_L, body_W, body_H]);

        // SERVO POCKETS: left/right hip
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
        // head block
        translate([-head_L/2, -head_W/2, 0])
            cube([head_L, head_W, head_H]);

        // servo pocket (nodding axis)
        translate([-head_L*0.25, 0, head_H/2])
            rotate([0,0,90]) servo_pocket_y();

        // rear sockets for neck → body
        translate([head_L/2 - PEG_LEN/2, +0.03, head_H/2])
            rotate([0,90,0]) socket();

        translate([head_L/2 - PEG_LEN/2, -0.03, head_H/2])
            rotate([0,90,0]) socket();
    }

    // front pegs for jaw
    translate([-head_L/2, +0.02, head_H*0.35])
        rotate([0,90,0]) peg();

    translate([-head_L/2, -0.02, head_H*0.35])
        rotate([0,90,0]) peg();
}


// ---------- JAW ----------
module jaw_piece() {
    // simple block with 2 sockets to mate with head pegs
    difference() {
        cube([0.189, 0.075, 0.05]);

        // sockets at rear
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

        // servo pocket on front face
        translate([ tail_L/2 - SERVO_L/2 - 0.005, 0, tail_H/2 ])
            rotate([0,0,90]) servo_pocket_y();

        // FRONT sockets
        translate([ tail_L/2 - PEG_LEN/2, +0.02, tail_H/2])
            rotate([0,90,0]) socket();

        translate([ tail_L/2 - PEG_LEN/2, -0.02, tail_H/2])
            rotate([0,90,0]) socket();
    }

    // BACK pegs
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

        // servo pocket
        translate([0, 0.04/2, 0.02])
            rotate([0,0,90]) servo_pocket_y();

        // socket to connect to body
        translate([-leg_L/2 + PEG_LEN/2, 0, 0.02])
            socket();
    }

    // peg for knee
    translate([leg_L/2, 0, 0.02])
        peg();
}


// ---------- KNEE ----------
module knee_piece() {
    difference() {
        translate([-leg_L/2, -0.03/2, 0])
            cube([leg_L, 0.03, 0.03]);

        // servo pocket
        translate([leg_L*0.45, 0, -0.015])
            rotate([0,0,90]) servo_pocket_y();

        // socket for hip connection
        translate([-leg_L/2 + PEG_LEN/2, 0, 0.015])
            socket();
    }

    // peg for foot or next piece
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
// EXPORT GRID — EACH PART SEPARATE
// =========================================================
translate([0,0,0]) body_segment(0);
translate([1,0,0]) body_segment(1);
translate([2,0,0]) body_segment(2);
translate([3,0,0]) body_segment(3);
translate([4,0,0]) body_segment(4);

translate([0,2,0]) head_piece();
translate([1,2,0]) jaw_piece();

translate([0,4,0]) tail_seg();
translate([1,4,0]) tail_seg();
translate([2,4,0]) tail_seg();

translate([0,6,0]) hip_piece();
translate([1,6,0]) hip_piece();
translate([2,6,0]) hip_piece();
translate([3,6,0]) hip_piece();

translate([0,8,0]) knee_piece();
translate([1,8,0]) knee_piece();
translate([2,8,0]) knee_piece();
translate([3,8,0]) knee_piece();

translate([0,10,0]) bracket_mg90s();
