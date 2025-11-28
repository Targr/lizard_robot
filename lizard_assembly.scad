//
// lizard_assembly.scad
// Parametric Lizard assembly (units: meters)
// Paste into OpenSCAD, render, then Export -> Export as STL for any selected object
//

// Parameters (from URDF)
body_length = 0.35;
body_width  = 0.12;
body_height = 0.08;

tail_length = 0.22;
tail_width  = 0.08;
tail_height = 0.06;

head_length = 0.18;
head_width  = 0.12;
head_height = 0.06;

leg_length  = 0.22;
hip_w = 0.04;
hip_h = 0.04;
knee_w = 0.03;
knee_h = 0.03;

eye_radius = 0.02;

// bracket params (approx)
br_mg90s = [0.025, 0.012, 0.006]; // L, W, H
br_mg996r = [0.035, 0.015, 0.008];

// convenience: create a box centered at origin but sitting on z=0 (so bottom is on build plate)
module box_on_plate(sz=[1,1,1]) {
  translate([ -sz[0]/2, -sz[1]/2, 0]) cube(sz);
}

// body segment module (centered on its own origin, so we can place multiple)
module body_segment() {
  translate([-body_length/2, -body_width/2, 0]) cube([body_length, body_width, body_height]);
}

// tail segment
module tail_segment() {
  translate([-tail_length/2, -tail_width/2, 0]) cube([tail_length, tail_width, tail_height]);
}

// head & jaw
module head() {
  translate([-head_length/2, -head_width/2, 0]) cube([head_length, head_width, head_height]);
}
module jaw() {
  // jaw positioned relative to head: small offset forward and slightly lower
  translate([-0.5*head_length + head_length*0.25, -0.075/2, -0.005]) cube([0.189, 0.075, 0.05]);
}

// hip and knee (simple boxes for geometry)
module hip() {
  translate([-leg_length/2, -hip_w/2, 0]) cube([leg_length, hip_w, hip_h]);
}
module knee() {
  translate([-leg_length/2, -knee_w/2, 0]) cube([leg_length, knee_w, knee_h]);
}

// eye
module eye() {
  translate([0,0,eye_radius]) sphere(r=eye_radius, $fn=32);
}

// simple servo bracket: rectangular base (holes not boolean-subtracted for slicer simplicity)
module servo_bracket(sz=[0.025, 0.012, 0.006]) {
  translate([-sz[0]/2, -sz[1]/2, 0]) cube(sz);
}

// ASSEMBLY
// body_0 ... body_4 placed along +X
for(i=[0:4]) {
  translate([ i*(body_length*0.9), 0, 0]) body_segment();
}

// head on body_0: origin: -body_length*0.55 in x, z offset as in URDF
translate([ -body_length*0.55, 0, body_height*0.5 + head_height*0.5 - 0.01 ])
  head();

// jaw offset
translate([ -body_length*0.55 + head_length*0.25, 0, body_height*0.5 + head_height*0.5 - 0.01 - head_height*0.5 - 0.005 + 0.05/2 ])
  jaw();

// tail segments starting after body_4
tail_start = 4*(body_length*0.9) + tail_length*0.9;
for(t=[0:2]) translate([ tail_start + t*(tail_length*0.9), 0, 0 ]) tail_segment();

// legs: using simple placements (front on body_0, rear on body_4)
translate([ 0.02,  0.12, 0.02 ]) hip();
translate([ 0.02, -0.12, 0.02 ]) hip();
translate([ 4*(body_length*0.9)+0.02,  0.12, 0.02 ]) hip();
translate([ 4*(body_length*0.9)+0.02, -0.12, 0.02 ]) hip();

translate([ 0.02 + leg_length*0.45,  0.12, 0.02 - 0.06 ]) knee();
translate([ 0.02 + leg_length*0.45, -0.12, 0.02 - 0.06 ]) knee();
translate([ 4*(body_length*0.9)+0.02 + leg_length*0.45,  0.12, 0.02 - 0.06 ]) knee();
translate([ 4*(body_length*0.9)+0.02 + leg_length*0.45, -0.12, 0.02 - 0.06 ]) knee();

// eyes on head
translate([ -body_length*0.55,  head_width*0.35, body_height*0.5 + head_height*0.5 - 0.01 + eye_radius ]) eye();
translate([ -body_length*0.55, -head_width*0.35, body_height*0.5 + head_height*0.5 - 0.01 + eye_radius ]) eye();

// Brackets (place off to the side in the scene for easy exporting)
translate([ 2.0, 0, 0 ]) servo_bracket(br_mg90s);
translate([ 2.06, 0, 0 ]) servo_bracket(br_mg996r);
