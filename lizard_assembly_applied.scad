// lizard_parts_v2.scad
// Parametric parts for MG90S (360° version) + PCA9685 driver mounting and cable routing
// Updated with servo & PCA9685 specs you provided (360° servos, 300mm cable allowance, 2 PCA boards included).
// Units: mm
// Author: ChatGPT (GPT-5 Thinking mini)

$fn = 64;

// ----------------- USER / HARDWARE PARAMETERS -----------------
// MG90S 360° micro servo (typical dimensions; adjust if your batch differs)
servo_w = 22.8;      // width, mm
servo_d = 12.2;      // depth (front-to-back), mm
servo_h = 28.5;      // height, mm
servo_ear_to_ear = servo_w + 6;  // approximate spacing of ear screw centers
servo_mount_hole = 2.6;          // ear screw diameter (M2.6)
servo_clearance = 0.6;           // extra clearance in pockets

// Note: your MG90S are continuous-rotation (factory 90deg center->1500us, 500-2500us range).
// That is a controller / signal property — mechanically this is the same servo size,
// so pockets/slots are unchanged. This code includes a servo horn placeholder and
// optional through-slot for cable routing (300 mm line length allowance).

// PCA9685 board approx footprint (typical 16-channel breakout with terminal block)
pca_len = 66;     // length mm
pca_wid = 52;     // width mm (include terminal-block space)
pca_thk = 4;      // board thickness
pca_term_block_depth = 10; // clearance for terminal block
pca_mount_hole = 3.2;      // mounting hole clearance for M3 screws
pca_header_space = 6;      // header height clearance

// Electronics box (from URDF: 60 x 40 x 20 mm)
elec_x = 60;
elec_y = 40;
elec_z = 20;
elec_mount_clearance = 3.2;

// Cable relief for 300mm lines: we only provide a channel; actual cable length is external
cable_slot_width = 8;
cable_slot_height = 5;

// Body segment (script uses 0.35 m -> 350 mm; shrink if you want a smaller robot)
body_length = 350;  // mm - matches URDF scale = 0.35 m
body_width  = 120;
body_height = 80;
wall_th = 4;

// Fastener clearances
screw_clear = 3.0;   // mm clearance for screws into printed posts
snap_tolerance = 0.35;

// ----------------- MODULES -----------------

module mg90s_placeholder(show_ears=true, include_cable_exit=true) {
    // body rectangular block representing servo
    translate([0,0,servo_h/2])
        cube([servo_w, servo_d, servo_h], center=true);

    // ears & screw holes
    if (show_ears) {
        ear_offset = 2;
        // left ear - extruded tab
        translate([-servo_w/2 - 2, 0, servo_h/2]) rotate([0,90,0]) linear_extrude(height=2) circle(r=4);
        // right ear
        translate([servo_w/2 + 2, 0, servo_h/2]) rotate([0,90,0]) linear_extrude(height=2) circle(r=4);
        // screw hole cylinders (through the ears)
        translate([-servo_w/2 - 2, 0, 0]) rotate([90,0,0]) cylinder(h=servo_h+4, r=servo_mount_hole/2);
        translate([servo_w/2 + 2, 0, 0]) rotate([90,0,0]) cylinder(h=servo_h+4, r=servo_mount_hole/2);
    }

    // servo shaft/horn interface placeholder (center)
    translate([0,0,servo_h + 1]) rotate([0,0,0]) cylinder(h=3, r=1.6); // small shaft nub

    // cable exit slot on back
    if (include_cable_exit) {
        translate([0, -servo_d/2 - cable_slot_height/2, servo_h/2]) rotate([0,0,0]) cube([servo_w*0.4, cable_slot_height, cable_slot_width], center=true);
    }
}

// servo horn (flat 4-arm style)
module servo_horn(d=20, thickness=3, center_hole=3.2) {
    difference() {
        cylinder(r=d/2, h=thickness, center=true);
        translate([0,0,-1]) cylinder(r=center_hole/2, h=thickness+2, center=true);
        // quadrant cutouts to make 4 arms visually
        for (a=[0,90,180,270]) rotate([0,0,a]) translate([d*0.18, 0, 0]) square([d*0.18, d*0.36], center=true);
    }
}

// bracket pocket sized for MG90S; designed to accept servo with ear screw access
module servo_pocket(depth = servo_h + 2, lip = 4) {
    pocket_w = servo_w + servo_clearance;
    pocket_d = servo_d + servo_clearance;
    pocket_h = depth;
    outer_w = pocket_w + lip*2;
    outer_d = pocket_d + lip*2;
    outer_h = 8;

    difference() {
        cube([outer_w, outer_d, outer_h]);
        translate([lip, lip, -0.5]) cube([pocket_w, pocket_d, pocket_h+1]);
        // holes for ear screws (clearance)
        translate([lip + (outer_w/2 - 8), outer_d/2, outer_h/2]) rotate([90,0,0]) cylinder(h=outer_h+4, r=screw_clear/2);
        translate([lip + (8 - outer_w/2), outer_d/2, outer_h/2]) rotate([90,0,0]) cylinder(h=outer_h+4, r=screw_clear/2);
    }
}

// PCA9685 placeholder board (board plus terminal block clearance)
module pca9685_board(show_holes=true) {
    // board plate
    translate([-pca_len/2, -pca_wid/2, 0]) cube([pca_len, pca_wid, pca_thk]);

    // terminal block area (raised region)
    translate([pca_len/2 - pca_term_block_depth - 2, 0, pca_thk/2])
        cube([pca_term_block_depth+2, pca_wid - 6, pca_thk+4], center=true);

    // header standoff area
    translate([-pca_len/2 + 8, -pca_wid/2 + 8, pca_thk + 2]) cube([pca_len - 16, pca_wid - 16, 4]);

    if (show_holes) {
        // four mounting holes roughly near corners
        offx = pca_len/2 - 6;
        offy = pca_wid/2 - 6;
        translate([ offx,  offy, pca_thk/2]) rotate([90,0,0]) cylinder(h=pca_thk+4, r=pca_mount_hole/2);
        translate([-offx,  offy, pca_thk/2]) rotate([90,0,0]) cylinder(h=pca_thk+4, r=pca_mount_hole/2);
        translate([ offx, -offy, pca_thk/2]) rotate([90,0,0]) cylinder(h=pca_thk+4, r=pca_mount_hole/2);
        translate([-offx, -offy, pca_thk/2]) rotate([90,0,0]) cylinder(h=pca_thk+4, r=pca_mount_hole/2);
    }
}

// electronics box (matches URDF visual: 60 x 40 x 20 mm)
module electronics_box(internal_wall=2) {
    difference() {
        translate([-elec_x/2, -elec_y/2, 0]) cube([elec_x, elec_y, elec_z]);
        translate([-elec_x/2 + internal_wall, -elec_y/2 + internal_wall, internal_wall]) cube([elec_x - 2*internal_wall, elec_y - 2*internal_wall, elec_z - internal_wall]);
        // rear cable cutout
        translate([elec_x/2 - 6, -6, elec_z/2]) cube([6, 12, elec_z]);
    }
    // mounting bosses (four)
    for (x=[-elec_x/2 + 8, elec_x/2 - 8])
    for (y=[-elec_y/2 + 8, elec_y/2 - 8])
        translate([x,y,elec_z/2]) rotate([90,0,0]) cylinder(h=internal_wall+6, r=elec_mount_clearance/2);
}

// 12-servo rack (two rows of six pockets) for your 12 MG90S 360° servos
module servo_rack(rows=2, cols=6, spacing_x=38, spacing_y=28, pocket_margin=6) {
    total_x = (cols-1)*spacing_x + 2*pocket_margin + servo_w;
    total_y = (rows-1)*spacing_y + 2*pocket_margin + servo_d;
    // base plate
    translate([-total_x/2, -total_y/2, 0]) cube([total_x, total_y, 6]);

    // pockets
    for (r=[0:rows-1]) for (c=[0:cols-1]) {
        px = -total_x/2 + pocket_margin + c*spacing_x;
        py = -total_y/2 + pocket_margin + r*spacing_y;
        translate([px + servo_w/2, py + servo_d/2, 6]) translate([-servo_w/2, -servo_d/2, 0]) servo_pocket(depth=servo_h+3, lip=4);
    }
}

// Cable routing channel (rectangular channel to route multiple servo wires)
module cable_channel(length=150, width=20, depth=8) {
    translate([-length/2, -width/2, 0]) cube([length, width, depth]);
}

// test leg stub for knee mounting
module leg_stub(length=60, thickness=8, shaft_d=4) {
    difference() {
        translate([-length/2, -thickness/2, 0]) cube([length, thickness, thickness*2]);
        translate([0,0,thickness/2]) rotate([90,0,0]) cylinder(h=thickness+4, r=shaft_d/2);
    }
}

// ----------------- ASSEMBLY PREVIEW -----------------
module preview_full() {
    // Body segment
    translate([0,0,0]) body_segment();

    // Put a servo rack to the side (for mounting before assembly)
    translate([body_length*0.6, 0, 0]) rotate([0,0,0]) servo_rack();

    // Electronics box on top center of body (as URDF)
    translate([0,0,body_height - elec_z - 2]) electronics_box();

    // Place 2 PCA9685 boards near the electronics box (spaced)
    translate([elec_x/2 + pca_len/2 + 6, 0, body_height - pca_thk - 8]) pca9685_board();
    translate([elec_x/2 + pca_len/2 + 6, - (pca_wid + 8), body_height - pca_thk - 8]) pca9685_board();

    // Cable channel between electronics and servo rack
    translate([body_length*0.25, -body_width/2 - 12, 6]) cable_channel(length=body_length*0.6, width=20, depth=10);

    // Show a single servo placed in one rack pocket (visual reference)
    translate([body_length*0.6 - 40, 0, 10]) mg90s_placeholder();

    // A horn next to servo
    translate([body_length*0.6 - 10, 35, 10]) rotate([90,0,0]) servo_horn();
}

// Body segment module: hollow shell with optional electronics bay and cable exit
module body_segment(with_elec_mount=true, include_servo_pad_holes=true) {
    seg_len = body_length;
    seg_w = body_width;
    seg_h = body_height;
    // outer shell
    difference() {
        translate([-seg_len/2, -seg_w/2, 0]) cube([seg_len, seg_w, seg_h]);

        // hollow interior
        translate([-seg_len/2 + wall_th, -seg_w/2 + wall_th, wall_th]) cube([seg_len - 2*wall_th, seg_w - 2*wall_th, seg_h - wall_th]);

        // cutouts for leg placements (approx)
        leg_cut_w = 30;
        leg_cut_h = 40;
        cut_x_off = seg_len*0.12;
        for (pos = [cut_x_off, seg_len - cut_x_off]) {
            // left
            translate([-seg_len/2 + pos - leg_cut_w/2, seg_w/2 - 6 - leg_cut_h, 0]) cube([leg_cut_w, leg_cut_h, seg_h]);
            // right
            translate([-seg_len/2 + pos - leg_cut_w/2, -seg_w/2 + 6, 0]) cube([leg_cut_w, leg_cut_h, seg_h]);
        }

        // servo pad clearance on underside for hips
        if (include_servo_pad_holes) {
            pad_x = seg_len*0.12;
            pad_w = servo_w + 10;
            pad_d = servo_d + 8;
            pad_th = wall_th + 2;
            translate([-seg_len/2 + pad_x - pad_w/2, seg_w/2 - pad_d - 2, -0.5]) cube([pad_w, pad_d, pad_th+1]);
            translate([-seg_len/2 + pad_x - pad_w/2, -seg_w/2 + 2, -0.5]) cube([pad_w, pad_d, pad_th+1]);
            translate([seg_len/2 - pad_x - pad_w/2, seg_w/2 - pad_d - 2, -0.5]) cube([pad_w, pad_d, pad_th+1]);
            translate([seg_len/2 - pad_x - pad_w/2, -seg_w/2 + 2, -0.5]) cube([pad_w, pad_d, pad_th+1]);
        }

        // electronics bay top cutout (URDF sized)
        if (with_elec_mount) {
            translate([-elec_x/2 - 3, -elec_y/2 - 3, seg_h - elec_z - 4]) cube([elec_x + 6, elec_y + 6, elec_z + 6]);
        }
        // cable slot from electronics bay to side (for 300mm cable routing)
        translate([elec_x/2 + 4, -cable_slot_width/2, seg_h - elec_z/2 - 2]) cube([20, cable_slot_width, cable_slot_height]);
    }

    // Add mount posts inside for electronics box if enabled
    if (with_elec_mount) {
        translate([0,0,seg_h - elec_z/2 - 2]) {
            esx = elec_x/2 - 8;
            esy = elec_y/2 - 8;
            for (cx=[-esx, esx]) for (cy=[-esy, esy])
                translate([cx, cy, -2]) cylinder(h=6, r=(elec_mount_clearance+0.8)/2);
        }
    }
}

// ----------------- EXPORT / RENDER CHOICES -----------------
// Uncomment one of these at a time for rendering/export in OpenSCAD:

// -- Full preview: body + rack + electronics + PCAs
preview_full();

// -- Individual parts (uncomment to render only that part):
// body_segment();
// servo_rack();
// pca9685_board();
// electronics_box();
// mg90s_placeholder();
// servo_horn();
// cable_channel(length=200);

// ----------------- NOTES -----------------
// - The MG90S continuous-rotation is electrically configured (range 500-2500us, center 1500us).
//   No mechanical changes were needed for the continuous-rotation variants beyond leaving the horn and
//   shaft area accessible. Use the PCA9685 to send appropriate pulse widths to map 0-360° behavior.
// - Cable channel is a passive routed slot. 300mm lines are long; route them out through the slot and secure with zip-ties.
// - Adjust servo pockets, clearances and wall thickness to match your printer calibration.
// - If you want, I can also:
//     * generate separate SCAD pages (or an assembled STL layout) per-part for batch printing,
//     * add precise drill hole coordinates for a specific PCA9685 PCB if you provide a picture or drill-drawing,
//     * output an optimized layout for printing all 12 servo pockets in one bed-sized STL.
