//
// lizard_servo_driven.scad
// Servo-driven snap-fit lizard parts (MG90S + PCA9685 slot)
// All moving joints are driven by MG90S horns; pegs reserved only for rigid connections
// Units: meters (1 unit = 1 meter). Typical values are given in meters (e.g., 0.0228 = 22.8 mm).
//
// Hardware references used when designing dimensions:
//  - MG90S datasheet: typical dimensions ~22.8 x 12.2 x 28.5 mm, weight ~13.4 g. (TowerPro / suppliers). :contentReference[oaicite:3]{index=3}
//  - Adafruit PCA9685 breakout: ~62.5 x 25.4 x 3 mm footprint (board-only), mounting hole Ø ~2.5 mm. :contentReference[oaicite:4]{index=4}
//

// ---------- CONFIG / TUNABLES ----------
UNIT = 1;                // meters

// Print & hardware tolerances (meters)
SERVO_CLEAR = 0.0012;    // clearance around servo body (1.2 mm)
SPLINE_D = 0.0050;       // approx spline outer diameter for MG90S (5.0 mm). You can set to 0.0049..0.0059 depending on part.
SPLINE_DEPTH = 0.006;    // boss depth to accept horn spline (6 mm)
HORNSCREW_D = 0.0025;    // horn screw hole diameter (2.5 mm => M2.5)
HORNSCREW_TOL = 0.0003;  // extra tolerance on screw holes
SERVO_MOUNT_HOLE_D = 0.0026; // mounting screw holes for servo case (slightly > 2.5 mm)
HOLE_TOL = 0.0004;       // general hole tolerance

// MG90S package (meters)
SERVO_L = 0.0228;        // length (front-to-back)
SERVO_W = 0.0122;        // width
SERVO_H = 0.0285;        // height (case height)

// PCA9685 board slot (Adafruit approx dimensions)
PCA_W = 0.0625;          // 62.5 mm width
PCA_H = 0.0254;          // 25.4 mm depth
PCA_THICK = 0.003;       // 3 mm board thickness
PCA_HOLE_D = 0.0025;     // 2.5 mm mounting hole diameter
PCA_HOLE_INSET = 0.004;  // inset from board edges for holes (4 mm)

// geometry (robot)
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

// grid spacing for export
PART_SPACING_X = 0.70;
PART_SPACING_Y = 0.70;


// ---------- PRIMITIVE HELPERS ----------
module cube_center(sz=[1,1,1]) {
    translate([-sz[0]/2, -sz[1]/2, -sz[2]/2]) cube(sz);
}

module cylinder_center(h, r, fn=64) {
    translate([0,0,-h/2]) cylinder(h=h, r=r, $fn=fn);
}

// fast clearance drill (round hole with tolerance)
module hole_cyl(d, h, tol=HOLE_TOL) {
    translate([0,0,-h/2]) cylinder(h=h, r=(d/2.0 + tol), $fn=32);
}

// servo body pocket: oriented so servo's length axis points along +X when placed
// pocket is open on top (Z+), so you can slide servo in from above
module servo_body_pocket(orient="x", recess_depth=SERVO_H + 0.002) {
    L = SERVO_L + 2*SERVO_CLEAR;
    W = SERVO_W + 2*SERVO_CLEAR;
    H = recess_depth;
    if (orient == "x") {
        translate([-L/2, -W/2, 0]) cube([L, W, H]);
    } else if (orient == "y") {
        translate([-W/2, -L/2, 0]) cube([W, L, H]);
    } else {
        translate([-W/2, -L/2, 0]) cube([W, L, H]); // fallback
    }
}

// servo horn boss (on the child link) — a clear circular boss with spline recess + two screw holes
module servo_horn_boss(boss_d=0.014, boss_h=0.008, spline_d=SPLINE_D, spline_depth=SPLINE_DEPTH, screw_offset=0.006) {
    // main boss
    translate([0,0,boss_h/2]) cylinder(h=boss_h, r=boss_d/2, $fn=64);

    // spline recess (cut)
    translate([0,0,boss_h - spline_depth]) rotate([0,0,0]) difference() {
        translate([0,0,boss_h/2]) cylinder(h=spline_depth, r=(spline_d/2 + 0.0002), $fn=64);
        // leave small internal relief for spline seat
    }

    // two horn screw holes (roughly left/right)
    // user may adapt positions; these are typical ~6–8 mm apart centers on many micro-horns
    translate([screw_offset, 0, boss_h/2]) rotate([90,0,0]) hole_cyl(HORNSCREW_D, boss_d + 0.01, HORNSCREW_TOL);
    translate([-screw_offset, 0, boss_h/2]) rotate([90,0,0]) hole_cyl(HORNSCREW_D, boss_d + 0.01, HORNSCREW_TOL);

    // big visual marker: chamfer ring so connection spot is obvious
    translate([0,0,boss_h/2 + 0.0001]) color("orange") cylinder(h=0.001, r=boss_d*0.6, $fn=64);
}

// servo body mounting holes for securing servo to a fixed mount (parent link)
module servo_mount_holes(spacing_x=0.035, spacing_y=0.0, hole_d=SERVO_MOUNT_HOLE_D) {
    // Two holes through the mounting plate
    translate([ spacing_x/2.0, spacing_y/2.0, 0 ]) rotate([90,0,0]) hole_cyl(hole_d, 0.010);
    translate([ -spacing_x/2.0, spacing_y/2.0, 0 ]) rotate([90,0,0]) hole_cyl(hole_d, 0.010);
}

// PCA9685 slot (top-facing recess + four mounting holes)
module pca9685_slot() {
    // main slot (slightly larger for clearance)
    sx = PCA_W + 2*0.002; // add 2 mm clearance total (1 mm per side)
    sy = PCA_H + 2*0.002;
    sh = PCA_THICK + 0.001;
    translate([-sx/2, -sy/2, 0]) cube([sx, sy, sh]);

    // four mounting holes (approx inset)
    hx = (PCA_W/2.0 - PCA_HOLE_INSET);
    hy = (PCA_H/2.0 - PCA_HOLE_INSET);
    // holes through the mount
    translate([ hx,  hy, sh/2 ]) rotate([90,0,0]) hole_cyl(PCA_HOLE_D, sh + 0.004);
    translate([ hx, -hy, sh/2 ]) rotate([90,0,0]) hole_cyl(PCA_HOLE_D, sh + 0.004);
    translate([-hx,  hy, sh/2 ]) rotate([90,0,0]) hole_cyl(PCA_HOLE_D, sh + 0.004);
    translate([-hx, -hy, sh/2 ]) rotate([90,0,0]) hole_cyl(PCA_HOLE_D, sh + 0.004);
    // visual marker
    translate([0,0,sh + 0.001]) color("navy") cube([sx, sy, 0.0005]);
}

// pretty visible mounting tab (colored) for horns/servo slots
module visible_tab(txt="") {
    // small raised rim to draw attention (purely visual)
    translate([0,0,0]) color("red") cube([0.01, 0.01, 0.001]);
}

// ---------- PART DEFINITIONS ----------

// BODY SEGMENT (rigid piece). For servo-driven joints: only provide servo pocket and mount features, no pegs.
module body_segment(seg_index=0, left_hip=true, right_hip=true, pca_slot=false) {
    // main body box
    translate([-body_L/2, -body_W/2, 0]) cube([body_L, body_W, body_H]);

    // hip servo pockets and mount on sides (if this body hosts a hip servo)
    if (left_hip) {
        // pocket oriented so servo length axis aligns along X (pointing forward)
        translate([ 0.02, body_W/2 + 0.0001, hip_z ]) rotate([0,0,90]) difference() {
            servo_body_pocket("y", SERVO_H + 0.002);
            // subtract nothing here; for visualization keep pocket; we already used difference to show typical usage
        }
        // mounting holes for servo body
        translate([ 0.02, body_W/2 + SERVO_W/2 + 0.0005, hip_z + SERVO_H*0.5 ]) rotate([0,0,0]) servo_mount_holes(0.035, 0.0);
        // make the boss for horn on the child - but body piece only shows pocket; boss appears on hip block
        // big visible mark
        translate([0.02, body_W/2 + 0.02, body_H - 0.01]) color("yellow") cube([0.006,0.006,0.002]);
    }
    if (right_hip) {
        translate([ 0.02, -body_W/2 - 0.0001, hip_z ]) rotate([0,0,-90]) difference() {
            servo_body_pocket("y", SERVO_H + 0.002);
        }
        translate([ 0.02, -body_W/2 - SERVO_W/2 - 0.0005, hip_z + SERVO_H*0.5 ]) rotate([0,0,0]) servo_mount_holes(0.035, 0.0);
        translate([0.02, -body_W/2 - 0.02, body_H - 0.01]) color("yellow") cube([0.006,0.006,0.002]);
    }

    // If requested, add PCA9685 slot on top center (for body_2 by default)
    if (pca_slot) {
        translate([0, 0, body_H + 0.002]) pca9685_slot();
        // add a big visible cutout frame so the board slot is obvious
        translate([-PCA_W/2 - 0.003, -PCA_H/2 - 0.003, body_H + 0.001]) color("blue") cube([PCA_W + 0.006, PCA_H + 0.006, 0.0008]);
    }
}

// HIP BLOCK (child part which mounts to servo horn and connects to knee)
// This piece has a large horn boss (so servo horn screws into it) — the servo body is mounted in the parent (body segment)
module hip_block(side="L") {
    // main hip geometry
    translate([-leg_L/4, -0.02, 0]) cube([leg_L/2, 0.04, 0.04]);
    // clearly visible horn boss at local origin (position so the boss faces backward into the servo)
    translate([ -leg_L/4, 0, 0.02 ]) rotate([0,90,0]) servo_horn_boss(boss_d=0.014, boss_h=0.008, spline_d=SPLINE_D, spline_depth=SPLINE_DEPTH, screw_offset=0.006);
    // small registration pin for alignment (non-load-bearing)
    translate([ -leg_L/4 - 0.01, 0.015, 0.02 ]) color("lime") cube([0.002,0.002,0.004]);
}

// KNEE BLOCK (child part which mounts to knee servo horn)
module knee_block() {
    translate([-leg_L/2, -0.015, 0]) cube([leg_L, 0.03, 0.03]);
    // horn boss near front so that knee rotates relative to hip
    translate([ leg_L*0.45, 0, 0.015 ]) rotate([0,90,0]) servo_horn_boss(boss_d=0.012, boss_h=0.007, spline_d=SPLINE_D, spline_depth=SPLINE_DEPTH, screw_offset=0.005);
    // small alignment socket to accept hip peg if you printed one (optional)
    translate([ leg_L*0.45 + 0.01, 0.0, 0.015 ]) color("lime") cube([0.002,0.002,0.004]);
}

// HEAD PIECE (child for head servo)
// head servo is mounted to body_0; head piece contains horn boss and jaw pegs
module head_piece() {
    translate([-head_L/2, -head_W/2, 0]) cube([head_L, head_W, head_H]);
    // horn boss for head rotation
    translate([-head_L*0.25, 0, head_H*0.45]) rotate([0,90,0]) servo_horn_boss(boss_d=0.014, boss_h=0.008, spline_d=SPLINE_D, spline_depth=SPLINE_DEPTH, screw_offset=0.006);
    // jaw pegs (front) — these attach to jaw via horn screw or socket
    translate([-head_L/2, +0.02, head_H*0.35]) rotate([0,90,0]) cylinder(h=PEG_LEN, r=PEG_D/2, $fn=32);
    translate([-head_L/2, -0.02, head_H*0.35]) rotate([0,90,0]) cylinder(h=PEG_LEN, r=PEG_D/2, $fn=32);
    // visible slot where board would align (if head has sensors)
    translate([0.0, head_W/2 + 0.01, head_H*0.5]) color("cyan") cube([0.02, 0.005, 0.001]);
}

// JAW (child) — mounts onto head front pegs OR can be screwed to a small jaw-horn boss
module jaw_piece() {
    translate([0,0,0]) translate([-0.189/2, -0.075/2, 0]) cube([0.189, 0.075, 0.05]);
    // sockets to mate with head pegs
    translate([0.189 - PEG_LEN/2, +0.02, 0.025]) rotate([0,90,0]) cylinder(h=PEG_LEN+0.002, r=(PEG_D+PEG_TOL)/2, $fn=32);
    translate([0.189 - PEG_LEN/2, -0.02, 0.025]) rotate([0,90,0]) cylinder(h=PEG_LEN+0.002, r=(PEG_D+PEG_TOL)/2, $fn=32);
    // optional small horn boss if you prefer jaw driven directly by servo horn (uncomment next line if using)
    // translate([0.02, 0, 0.025]) rotate([0,90,0]) servo_horn_boss();
}

// TAIL SEGMENT (child). Tail joints driven by servo horns. Each tail child contains a horn boss (front face) and a rear registration peg
module tail_segment(idx=0) {
    translate([-tail_L/2, -tail_W/2, 0]) cube([tail_L, tail_W, tail_H]);
    // horn boss at front face (this is where the previous tail/body servo horn screws into)
    translate([ tail_L/2 - 0.005, 0, tail_H*0.5 ]) rotate([0,90,0]) servo_horn_boss(boss_d=0.012, boss_h=0.006, spline_d=SPLINE_D, spline_depth=SPLINE_DEPTH, screw_offset=0.005);
    // rear peg for optional alignment
    translate([-tail_L/2 + 0.005, 0.015, tail_H*0.5]) color("lime") cube([0.002,0.002,0.004]);
}

// HIP MOUNT / BRACKET (holds servo horned hip & provides horn-receiving boss on child)
module hip_assembly() {
    // Parent piece is body segment (servo pocket present there).
    // This module is the *child* (hip block) which attaches to servo horn.
    hip_block();
}

// KNEE ASSEMBLY (child)
module knee_assembly() {
    knee_block();
}

// servo bracket (small plate to strap servo on if needed)
module bracket_mg90s() {
    difference() {
        translate([-0.025/2, -0.012/2, 0]) cube([0.025,0.012,0.006]);
        translate([-0.006,0,0.003]) rotate([90,0,0]) cylinder(h=0.012, r=0.0018);
        translate([ 0.006,0,0.003]) rotate([90,0,0]) cylinder(h=0.012, r=0.0018);
    }
}

// =========================================================
// EXPORT GRID — EACH PART INDEPENDENT, WIDELY SPACED
// (Place each part in its own cell so you can export cleanly)
// =========================================================

/* Row 0: body segments (body_0..body_4).
   body_0 and body_4 host hips; body_2 hosts PCA9685 slot
*/
translate([0*PART_SPACING_X, 0*PART_SPACING_Y, 0]) body_segment(0, true, true, false);
translate([1*PART_SPACING_X, 0*PART_SPACING_Y, 0]) body_segment(1, true, true, false);
translate([2*PART_SPACING_X, 0*PART_SPACING_Y, 0]) body_segment(2, true, true, true); // body_2 has PCA slot
translate([3*PART_SPACING_X, 0*PART_SPACING_Y, 0]) body_segment(3, true, true, false);
translate([4*PART_SPACING_X, 0*PART_SPACING_Y, 0]) body_segment(4, true, true, false);

/* Row 1: head + jaw */
translate([0*PART_SPACING_X, 1*PART_SPACING_Y, 0]) head_piece();
translate([1*PART_SPACING_X, 1*PART_SPACING_Y, 0]) jaw_piece();

/* Row 2: tails (3 segments) */
translate([0*PART_SPACING_X, 2*PART_SPACING_Y, 0]) tail_segment(0);
translate([1*PART_SPACING_X, 2*PART_SPACING_Y, 0]) tail_segment(1);
translate([2*PART_SPACING_X, 2*PART_SPACING_Y, 0]) tail_segment(2);

/* Row 3: hips (4) */
translate([0*PART_SPACING_X, 3*PART_SPACING_Y, 0]) hip_assembly();
translate([1*PART_SPACING_X, 3*PART_SPACING_Y, 0]) hip_assembly();
translate([2*PART_SPACING_X, 3*PART_SPACING_Y, 0]) hip_assembly();
translate([3*PART_SPACING_X, 3*PART_SPACING_Y, 0]) hip_assembly();

/* Row 4: knees (4) */
translate([0*PART_SPACING_X, 4*PART_SPACING_Y, 0]) knee_assembly();
translate([1*PART_SPACING_X, 4*PART_SPACING_Y, 0]) knee_assembly();
translate([2*PART_SPACING_X, 4*PART_SPACING_Y, 0]) knee_assembly();
translate([3*PART_SPACING_X, 4*PART_SPACING_Y, 0]) knee_assembly();

/* Row 5: servo brackets */
translate([0*PART_SPACING_X, 5*PART_SPACING_Y, 0]) bracket_mg90s();
translate([1*PART_SPACING_X, 5*PART_SPACING_Y, 0]) bracket_mg90s();
translate([2*PART_SPACING_X, 5*PART_SPACING_Y, 0]) bracket_mg90s();

