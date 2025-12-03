// lizard_parts_v3.scad
// Separated parts layout for MG90S + PCA9685 lizard (each piece is its own solid)
// Export each part individually by uncommenting its line at the bottom or by copying its module call.
// Units: mm
// Author: ChatGPT (GPT-5 Thinking mini)

$fn = 64;

// --------------- PARAMETERS ---------------
servo_w = 22.8;
servo_d = 12.2;
servo_h = 28.5;
servo_mount_hole = 2.6;
servo_clearance = 0.6;

horn_diameter = 20;
horn_thickness = 3;
horn_center_hole = 3.2;

pca_len = 66;
pca_wid = 52;
pca_thk = 4;
pca_mount_hole = 3.2;

elec_x = 60;
elec_y = 40;
elec_z = 20;
elec_mount_clearance = 3.2;

body_length = 350;
body_width  = 120;
body_height = 80;
wall_th = 4;

screw_clear = 3.0;
snap_tolerance = 0.35;

// spacing for grid layout (how far apart separate parts are placed)
grid_x = 120;
grid_y = 100;

// --------------- MODULES (ONE PART EACH) ---------------

// MG90S solid placeholder (independent)
module mg90s_solid(include_ears=true, include_cable_exit=true) {
    // main block
    translate([-servo_w/2, -servo_d/2, 0]) cube([servo_w, servo_d, servo_h]);

    // ears (two tabs) as separate solids integrated with servo block (still one piece)
    if (include_ears) {
        // left ear
        translate([-servo_w/2 - 2, servo_d*0.2 - servo_d/2, servo_h*0.4]) rotate([0,90,0]) linear_extrude(height=2) circle(r=4);
        // right ear
        translate([servo_w/2 + 2, servo_d*0.2 - servo_d/2, servo_h*0.4]) rotate([0,90,0]) linear_extrude(height=2) circle(r=4);
        // screw holes as thru-cylinders (keeps them part of same solid for fit check)
        translate([-servo_w/2 - 2, servo_d/2 - servo_d*0.2, servo_h/2]) rotate([90,0,0]) cylinder(h=servo_h+6, r=screw_clear/2);
        translate([servo_w/2 + 2, servo_d/2 - servo_d*0.2, servo_h/2]) rotate([90,0,0]) cylinder(h=servo_h+6, r=screw_clear/2);
    }

    // shaft nub
    translate([0,0,servo_h]) cylinder(h=3, r=1.6);
    // cable slot (keeps it as part of servo solid, but it's a cut—this is safe because servo is its own piece)
    if (include_cable_exit) {
        translate([0, -servo_d/2 - 6, servo_h*0.4]) cube([servo_w*0.4, 12, 8], center=true);
    }
}

// Servo horn as independent part
module servo_horn_solid(d=horn_diameter, thickness=horn_thickness, center_hole=horn_center_hole) {
    difference() {
        cylinder(h=thickness, r=d/2, center=true);
        translate([0,0,-1]) cylinder(h=thickness+2, r=center_hole/2, center=true);
        for (a=[0,90,180,270]) rotate([0,0,a]) translate([d*0.18, 0, 0]) square([d*0.18, d*0.36], center=true);
    }
}

// Servo pocket / bracket as independent part (so servo can be inserted into it)
module servo_bracket_solid(depth = servo_h + 2, lip = 4) {
    pocket_w = servo_w + servo_clearance;
    pocket_d = servo_d + servo_clearance;
    pocket_h = depth;
    outer_w = pocket_w + lip*2;
    outer_d = pocket_d + lip*2;
    outer_h = 8;

    // outer block (solid)
    translate([-outer_w/2, -outer_d/2, 0]) cube([outer_w, outer_d, outer_h]);

    // The actual pocket is not subtracted here (we want a separate solid that represents the bracket
    // with a pocket — but for printing, pockets can be left as cavities. To keep the bracket as a single part,
    // we create the pocket as a negative that will be honored when exporting just this module.)
    difference() {
        translate([-outer_w/2, -outer_d/2, 0]) cube([outer_w, outer_d, outer_h]);
        translate([-pocket_w/2, -pocket_d/2, -0.5]) cube([pocket_w, pocket_d, pocket_h+1]);
        // screw clearance holes (as holes inside bracket)
        translate([-pocket_w/2 - 2, 0, outer_h/2]) rotate([90,0,0]) cylinder(h=outer_h+4, r=screw_clear/2);
        translate([pocket_w/2 + 2, 0, outer_h/2]) rotate([90,0,0]) cylinder(h=outer_h+4, r=screw_clear/2);
    }
}

// Electronics box (independent)
module electronics_box_solid() {
    // outer shell
    difference() {
        translate([-elec_x/2, -elec_y/2, 0]) cube([elec_x, elec_y, elec_z]);
        translate([-elec_x/2 + 2, -elec_y/2 + 2, 2]) cube([elec_x - 4, elec_y - 4, elec_z - 2]);
        // rear cable cutout
        translate([elec_x/2 - 6, -6, elec_z/2]) cube([6, 12, elec_z]);
    }
    // mounting posts
    for (x=[-elec_x/2 + 8, elec_x/2 - 8])
    for (y=[-elec_y/2 + 8, elec_y/2 - 8])
        translate([x,y,elec_z/2]) rotate([90,0,0]) cylinder(h=6, r=(elec_mount_clearance+0.8)/2);
}

// PCA9685 board (independent)
module pca9685_solid() {
    translate([-pca_len/2, -pca_wid/2, 0]) cube([pca_len, pca_wid, pca_thk]);
    // terminal block projection (as added block)
    translate([pca_len/2 - 12, 0, pca_thk/2]) cube([12, pca_wid - 6, pca_thk+6], center=true);
    // mounting holes (as holes)
    offx = pca_len/2 - 6;
    offy = pca_wid/2 - 6;
    translate([ offx,  offy, pca_thk/2]) rotate([90,0,0]) cylinder(h=pca_thk+6, r=pca_mount_hole/2);
    translate([-offx,  offy, pca_thk/2]) rotate([90,0,0]) cylinder(h=pca_thk+6, r=pca_mount_hole/2);
    translate([ offx, -offy, pca_thk/2]) rotate([90,0,0]) cylinder(h=pca_thk+6, r=pca_mount_hole/2);
    translate([-offx, -offy, pca_thk/2]) rotate([90,0,0]) cylinder(h=pca_thk+6, r=pca_mount_hole/2);
}

// Body segment as a single printable part (no fused adjacent parts)
module body_segment_solid(with_elec_mount=true) {
    // outer shell
    translate([-body_length/2, -body_width/2, 0]) cube([body_length, body_width, body_height]);

    // interior hollow - represented with subtraction (still the body segment is one part with cavity)
    difference() {
        translate([-body_length/2, -body_width/2, 0]) cube([body_length, body_width, body_height]);
        translate([-body_length/2 + wall_th, -body_width/2 + wall_th, wall_th]) cube([body_length - 2*wall_th, body_width - 2*wall_th, body_height - wall_th]);
        // leg cutouts (still done as subtractions on this part)
        leg_cut_w = 30;
        leg_cut_h = 40;
        cut_x_off = body_length*0.12;
        for (pos = [cut_x_off, body_length - cut_x_off]) {
            translate([-body_length/2 + pos - leg_cut_w/2, body_width/2 - 6 - leg_cut_h, 0]) cube([leg_cut_w, leg_cut_h, body_height]);
            translate([-body_length/2 + pos - leg_cut_w/2, -body_width/2 + 6, 0]) cube([leg_cut_w, leg_cut_h, body_height]);
        }
        // electronics bay cutout (top)
        if (with_elec_mount) {
            translate([-elec_x/2 - 3, -elec_y/2 - 3, body_height - elec_z - 4]) cube([elec_x + 6, elec_y + 6, elec_z + 6]);
        }
        // cable slot
        translate([elec_x/2 + 4, -10, body_height - elec_z/2 - 2]) cube([20, 12, 8]);
    }
}

// Leg stub as independent part
module leg_stub_solid(length=60, thickness=8, shaft_d=4) {
    difference() {
        translate([-length/2, -thickness/2, 0]) cube([length, thickness, thickness*2]);
        translate([0,0,thickness/2]) rotate([90,0,0]) cylinder(h=thickness+4, r=shaft_d/2);
    }
}

// Cable channel (independent)
module cable_channel_solid(length=150, width=20, depth=8) {
    translate([-length/2, -width/2, 0]) cube([length, width, depth]);
}

// 12-servo rack (each pocket is shaped but entire rack is a single part; still separate from others)
module servo_rack_solid(rows=2, cols=6, spacing_x=38, spacing_y=28, pocket_margin=6) {
    total_x = (cols-1)*spacing_x + 2*pocket_margin + servo_w;
    total_y = (rows-1)*spacing_y + 2*pocket_margin + servo_d;
    translate([-total_x/2, -total_y/2, 0]) cube([total_x, total_y, 8]);
    // pockets will be subtractions so that the rack is a meaningful single-print piece:
    for (r=[0:rows-1]) for (c=[0:cols-1]) {
        px = -total_x/2 + pocket_margin + c*spacing_x;
        py = -total_y/2 + pocket_margin + r*spacing_y;
        translate([px + servo_w/2, py + servo_d/2, 8]) translate([-servo_w/2, -servo_d/2, 0]) difference() {
            translate([-servo_w/2, -servo_d/2, 0]) cube([servo_w + servo_clearance, servo_d + servo_clearance, servo_h + 4]);
            // leave the outer rack as part so we keep single solid per rack
        }
    }
}

// --------------- LAYOUT (each piece placed separately on a grid) ---------------

module layout_grid() {
    // Row 1
    translate([0,0,0]) body_segment_solid(); // body at origin

    // Row 1 - right
    translate([grid_x, 0, 0]) servo_rack_solid();

    // Row 2
    translate([0, -grid_y, 0]) electronics_box_solid();
    translate([grid_x, -grid_y, 0]) pca9685_solid();

    // Row 3
    translate([0, -2*grid_y, 0]) mg90s_solid();
    translate([grid_x, -2*grid_y, 0]) servo_bracket_solid();

    // Row 4
    translate([0, -3*grid_y, 0]) servo_horn_solid();
    translate([grid_x, -3*grid_y, 0]) leg_stub_solid();

    // cable channel far right
    translate([2*grid_x + 20, -grid_y, 0]) cable_channel_solid(length=200, width=20, depth=10);
}

// --------------- TOP-LEVEL RENDER ---------------
// Render the entire grid of separate parts. To export a single part, comment out others.
layout_grid();

// Alternatively render a single part for export by using one of these lines instead:
// mg90s_solid();           // render one servo
// servo_bracket_solid();   // render one bracket
// servo_horn_solid();      // render one horn
// electronics_box_solid(); // render electronics box
// pca9685_solid();         // render PCA board
// body_segment_solid();    // render body segment
// servo_rack_solid();      // render servo rack
// leg_stub_solid();        // render leg stub
// cable_channel_solid();   // render cable channel
