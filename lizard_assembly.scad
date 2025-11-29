//
// lizard_export_parts.scad
// All parts placed in separate locations for easy STL export
// MG90S ONLY configuration
// Units: meters (scale by 1000 in slicer if needed)
// ---------------------------------------------------------------------------

// ------------------------
// GLOBAL CONFIG
// ------------------------
UNIT_SCALE = 1;        // 1 = meters, 1000 = mm
SERVO_CLEARANCE = 0.0012;  // 1.2mm clearance around MG90S
RIGID_ONE_PIECE = false;   // false = 5 separate body segments, true = one-piece torso

// MG90S servo footprint (meters)
SERVO_L = 0.0228;
SERVO_W = 0.0122;
SERVO_H = 0.0285;
SERVO_BODY = [SERVO_L, SERVO_W, SERVO_H];

// lizard URDF geometry
body_length = 0.35;
body_width  = 0.12;
body_height = 0.08;

tail_length = 0.22;
tail_width  = 0.08;
tail_height = 0.06;

head_length = 0.18;
head_width  = 0.12;
head_height = 0.06;

leg_length = 0.22;
hip_y_offset = 0.12;
hip_z_offset = 0.02;

// ------------------------
// HELPERS
// ------------------------
module servo_pocket(orient="y", clearance=SERVO_CLEARANCE, depth=SERVO_H*0.95) {
    pocket_L = SERVO_L + 2*clearance;
    pocket_W = SERVO_W + 2*clearance;
    pocket_H = depth;

    if (orient=="y")
        translate([-pocket_W/2, -pocket_L/2, -pocket_H/2])
            cube([pocket_W, pocket_L, pocket_H]);
}

module body_segment_with_pockets() {
    difference() {
        // main box
        translate([-body_length/2, -body_width/2, 0])
            cube([body_length, body_width, body_height]);

        // left hip pocket
        translate([0.02, +body_width/2, hip_z_offset])
            rotate([0,0,90]) servo_pocket("y");

        // right hip pocket
        translate([0.02, -body_width/2, hip_z_offset])
            rotate([0,0,-90]) servo_pocket("y");
    }
}

module head_with_pockets() {
    difference() {
        translate([-head_length/2, -head_width/2, 0])
            cube([head_length, head_width, head_height]);

        // head servo pocket (nod axis)
        translate([-head_length*0.25, 0, head_height/2])
            rotate([0,0,90]) servo_pocket("y");
    }
}

module jaw_block() {
    cube([0.189, 0.075, 0.05]);
}

module tail_segment_with_pocket() {
    difference() {
        translate([-tail_length/2, -tail_width/2, 0])
            cube([tail_length, tail_width, tail_height]);
        translate([tail_length/2 - SERVO_L/2 - 0.005, 0, tail_height/2])
            rotate([0,0,90]) servo_pocket("y");
    }
}

module hip_module() {
    difference() {
        translate([-leg_length/2, -0.04/2, 0])
            cube([leg_length, 0.04, 0.04]);

        translate([0, 0.04/2, 0.02])
            rotate([0,0,90]) servo_pocket("y");
    }
}

module knee_module() {
    difference() {
        translate([-leg_length/2, -0.03/2, 0])
            cube([leg_length, 0.03, 0.03]);

        translate([leg_length*0.45, 0, -0.015])
            rotate([0,0,90]) servo_pocket("y");
    }
}

module servo_bracket_mg90s() {
    difference() {
        translate([-0.025/2, -0.012/2, 0]) cube([0.025,0.012,0.006]);
        // screw holes (simple through cylinders)
        translate([-0.006,0,0.003]) rotate([90,0,0]) cylinder(h=0.012, r=0.0018, $fn=20);
        translate([+0.006,0,0.003]) rotate([90,0,0]) cylinder(h=0.012, r=0.0018, $fn=20);
    }
}


// =====================================================================
// EXPORT BLOCK – EACH PART IN ITS OWN TRANSLATE REGION
// =====================================================================

// Body segments (or unified torso)
if (!RIGID_ONE_PIECE) {
    translate([0,0,0]) body_segment_with_pockets();                     // body_0
    translate([1,0,0]) body_segment_with_pockets();                     // body_1
    translate([2,0,0]) body_segment_with_pockets();                     // body_2
    translate([3,0,0]) body_segment_with_pockets();                     // body_3
    translate([4,0,0]) body_segment_with_pockets();                     // body_4
}
else {
    // one-piece rigid torso (rarely recommended but supported)
    translate([0,0,0])
        cube([5*body_length*0.9, body_width, body_height]);
}

// Head
translate([0,2,0]) head_with_pockets();

// Jaw
translate([1,2,0]) jaw_block();

// Tail segments
translate([0,4,0]) tail_segment_with_pocket();   // tail_0
translate([1,4,0]) tail_segment_with_pocket();   // tail_1
translate([2,4,0]) tail_segment_with_pocket();   // tail_2

// Hip modules
translate([0,6,0]) hip_module();                 // hip_0
translate([1,6,0]) hip_module();                 // hip_1
translate([2,6,0]) hip_module();                 // hip_2
translate([3,6,0]) hip_module();                 // hip_3

// Knee modules
translate([0,8,0]) knee_module();                // knee_0
translate([1,8,0]) knee_module();                // knee_1
translate([2,8,0]) knee_module();                // knee_2
translate([3,8,0]) knee_module();                // knee_3

// Brackets
translate([0,10,0]) servo_bracket_mg90s();
