// ============================================================
// CHECK MOTOR COVER FIT - v1
// Kiem tra nap bit dong co co lot vao mat day body v31 khong
// Don vi: mm
//
// Mo file nay bang OpenSCAD de xem overlay:
// - Xanh la trong suot: mat day/body v31 ngoai D84, long trong D72
// - Vang: nap motor cover D71 day 3.5
// - Cam: duong long trong body D72
// - Do: 6 lo bat oc nap motor
// - Xanh duong: 8 lo ngoai noi body voi base v9, chi de tham chieu
// ============================================================

$fn = 160;
eps = 0.05;

// BODY v31 bottom reference
body_outer_D = 84.0;
body_outer_R = body_outer_D/2;
body_inner_D = 72.0;          // bottom_inner_R = base_R - bottom_tube_wall_t = 42 - 6 = 36
body_inner_R = body_inner_D/2;
body_wall_t  = 6.0;

// Motor cover v1
cover_D = 71.0;               // nho hon body_inner_D 1mm
cover_R = cover_D/2;
cover_H = 3.5;
fit_clearance_radial = (body_inner_D - cover_D)/2; // 0.5mm moi ben

// 6 lo bat nap motor lay tu body v31
motor_body_W = 42.3 + 0.6;
motor_body_L = 42.3 + 0.6;
bottom_cover_pilot_edge_offset = 3;
bottom_cover_pilot_pair_offset = 10;
bottom_cover_pilot_X_edge = motor_body_W/2 + bottom_cover_pilot_edge_offset; // 24.45
bottom_cover_pilot_Y_edge = motor_body_L/2 + bottom_cover_pilot_edge_offset; // 24.45
cover_hole_pts = [
    [ bottom_cover_pilot_X_edge, 0],
    [-bottom_cover_pilot_X_edge, 0],
    [-bottom_cover_pilot_pair_offset,  bottom_cover_pilot_Y_edge],
    [ bottom_cover_pilot_pair_offset,  bottom_cover_pilot_Y_edge],
    [-bottom_cover_pilot_pair_offset, -bottom_cover_pilot_Y_edge],
    [ bottom_cover_pilot_pair_offset, -bottom_cover_pilot_Y_edge]
];
body_pilot_D = 3.30;
cover_clear_D = 4.30;

// 8 lo ngang noi base v9 voi body v31 - tham chieu thoi
radial_mount_count = 8;
radial_mount_center_R = body_outer_R - body_wall_t/2; // R39, nam giua thanh 6mm
radial_mount_D = 3.30;

// Khe day cua nap
wire_slot_W = 18.0;
wire_slot_depth_from_edge = 14.0;
wire_slot_on_negative_Y = true;

// View control
show_3d = true;
show_labels = true;
label_size = 3.2;

module ring2d(outer_r, inner_r) {
    difference() {
        circle(r=outer_r);
        circle(r=inner_r);
    }
}

module wire_slot_2d() {
    ysign = wire_slot_on_negative_Y ? -1 : 1;
    union() {
        translate([0, ysign*(cover_R - wire_slot_depth_from_edge/2 + 0.2)])
            square([wire_slot_W, wire_slot_depth_from_edge + 0.8], center=true);
        translate([0, ysign*(cover_R - wire_slot_depth_from_edge)])
            circle(d=wire_slot_W, $fn=64);
    }
}

module cover2d() {
    difference() {
        circle(r=cover_R);
        for (p = cover_hole_pts)
            translate(p) circle(d=cover_clear_D, $fn=48);
        wire_slot_2d();
    }
}

module hole_markers_6() {
    for (p = cover_hole_pts) {
        translate(p) {
            color("red") circle(d=6.5, $fn=48);
            color("white") circle(d=cover_clear_D, $fn=48);
        }
    }
}

module radial_8_markers() {
    for (i=[0:radial_mount_count-1]) {
        a = i*360/radial_mount_count;
        translate([radial_mount_center_R*cos(a), radial_mount_center_R*sin(a)]) {
            color("blue") circle(d=4.6, $fn=32);
            color("white") circle(d=radial_mount_D, $fn=32);
        }
    }
}

module text2d(t, p, rot=0, col="black", s=label_size) {
    if (show_labels)
        color(col) translate(p) rotate(rot) text(t, size=s, halign="center", valign="center", font="Liberation Sans:style=Bold");
}

module dim_line(r, a0, a1, col="black") {
    color(col) for (a=[a0:2:a1])
        translate([r*cos(a), r*sin(a)]) circle(d=0.55, $fn=12);
}

module check_2d() {
    // Body bottom footprint
    color([0.2,0.7,0.1,0.28]) ring2d(body_outer_R, body_inner_R);

    // Body inner limit and cover outer limit
    color("orange") difference() { circle(r=body_inner_R+0.18); circle(r=body_inner_R-0.18); }
    color([1,0.85,0.05,0.72]) cover2d();
    color([0.0,0.45,1.0,0.9]) difference() { circle(r=cover_R+0.12); circle(r=cover_R-0.12); }

    // holes
    hole_markers_6();
    radial_8_markers();

    // dimension dots/rings like reference
    dim_line(body_outer_R+1.5, 0, 360, "black");
    dim_line(body_inner_R, 0, 360, "orange");
    dim_line(cover_R, 0, 360, "blue");

    // labels
    text2d(str("BODY outer D", body_outer_D), [0, body_outer_R+8], 0, "green", 4);
    text2d(str("BODY inner D", body_inner_D), [0, body_inner_R+3], 0, "orange", 3.5);
    text2d(str("COVER D", cover_D, "  H", cover_H), [0, -8], 0, "blue", 4);
    text2d(str("clearance radial ", fit_clearance_radial, "mm/side"), [0, -body_outer_R-7], 0, "blue", 3.5);
    text2d("6x cover screws", [0, 29], 0, "red", 3.5);
    text2d("8x base/body side holes reference", [0, -33], 0, "blue", 3.0);
    text2d(str("wire slot ", wire_slot_W, " x ", wire_slot_depth_from_edge, " to -Y"), [0, -cover_R+7], 0, "black", 3.0);

    // numeric angle markers around outside
    for (a=[0:20:340]) {
        translate([(body_outer_R+5)*cos(a), (body_outer_R+5)*sin(a)])
            color("black") text(str(a), size=2.6, halign="center", valign="center");
    }
}

module check_3d() {
    // body bottom wall as shallow transparent ring
    color([0.2,0.7,0.1,0.28]) linear_extrude(height=1.0)
        ring2d(body_outer_R, body_inner_R);

    // motor cover placed inside body inner opening
    translate([0,0,1.2]) color([1,0.85,0.05,0.75]) linear_extrude(height=cover_H)
        cover2d();

    // body inner vertical gauge D72
    translate([0,0,1.15]) color([1,0.45,0,0.65]) linear_extrude(height=cover_H+0.3)
        difference() { circle(r=body_inner_R+0.15); circle(r=body_inner_R-0.15); }

    // cover D71 gauge
    translate([0,0,cover_H+1.55]) color([0,0.2,1,0.9]) linear_extrude(height=0.25)
        difference() { circle(r=cover_R+0.13); circle(r=cover_R-0.13); }

    // 6 screw alignment columns
    for (p = cover_hole_pts) {
        translate([p[0],p[1],1.1]) color("red") cylinder(h=cover_H+0.5, d=cover_clear_D, $fn=48);
        translate([p[0],p[1],1.0]) color([0,0.45,0,0.9]) cylinder(h=0.4, d=body_pilot_D, $fn=48);
    }

    // 8 outer side-hole references
    for (i=[0:radial_mount_count-1]) {
        a = i*360/radial_mount_count;
        translate([radial_mount_center_R*cos(a), radial_mount_center_R*sin(a), 1.1])
            color("blue") cylinder(h=0.8, d=radial_mount_D, $fn=32);
    }

    // floating labels as 2D emboss above
    translate([0,0,cover_H+2.2]) linear_extrude(height=0.15) check_2d();
}

if (show_3d)
    check_3d();
else
    check_2d();

echo("CHECK: body inner D", body_inner_D, "cover D", cover_D, "radial clearance", fit_clearance_radial);
echo("CHECK: cover fits by diameter?", cover_D < body_inner_D);
echo("CHECK: 6 hole pts", cover_hole_pts);
