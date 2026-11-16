// ============================================================
// CHECK BODY V31 DIMENSIONS / BAN VE KIEM TRA THONG SO
// Source body: gripper_body_shell_exact_palm_v31_M4_pilot_holes_no_threads.scad
// Don vi: mm
//
// Mo file nay trong OpenSCAD de xem cac thong so chinh cua body v31.
// Mau sac:
// - xanh la: footprint day D84 / long trong D72
// - vang/cam: cac cao do khoang dong co
// - do: 6 lo bat nap motor
// - xanh duong: 8 lo ngang noi body-base
// - tim: NEMA17 / lo truc tren mat phan tach
// ============================================================

$fn = 120;
eps = 0.05;

// ---------------- BODY V31 PARAMS ----------------
base_R = 42;
body_H = 98;
top_cap_H = 3;
wall_t = 3.5;
bottom_tube_wall_t = 6;
bottom_inner_R = base_R - bottom_tube_wall_t; // R36 / D72

finger_cut_height = 41;
motor_partition_H = 3;
motor_partition_top_Z = body_H - finger_cut_height;      // 57
motor_partition_bottom_Z = motor_partition_top_Z - motor_partition_H; // 54

motor_center_hole_R = 11;
motor_mount_pitch = 31;
motor_mount_hole_D = 3.4;

motor_body_W = 42.3 + 0.6; // 42.9
motor_body_L = 42.3 + 0.6; // 42.9
motor_body_H = 39;
motor_body_bottom_Z = motor_partition_bottom_Z - motor_body_H; // 15
motor_cavity_top_gap = 2.0;

lower_motor_wall_bottom_Z = 0;
lower_motor_wall_top_Z = motor_partition_bottom_Z; // 54

radial_mount_count = 8;
radial_mount_pitch_circle_R = base_R - bottom_tube_wall_t/2; // R39
radial_mount_center_Z = 10;
radial_mount_D = 3.30;
radial_mount_start_angle = 0;

m4_pilot_D = 3.30;
bottom_cover_pilot_D = m4_pilot_D;
bottom_cover_pilot_depth = 20;
bottom_cover_pilot_start_Z = motor_body_bottom_Z; // 15
bottom_cover_pilot_edge_offset = 3;
bottom_cover_pilot_pair_offset = 10;
bottom_cover_pilot_X_edge = motor_body_W/2 + bottom_cover_pilot_edge_offset; // 24.45
bottom_cover_pilot_Y_edge = motor_body_L/2 + bottom_cover_pilot_edge_offset; // 24.45
bottom_cover_pilot_pts = [
    [ bottom_cover_pilot_X_edge, 0],
    [-bottom_cover_pilot_X_edge, 0],
    [-bottom_cover_pilot_pair_offset,  bottom_cover_pilot_Y_edge],
    [ bottom_cover_pilot_pair_offset,  bottom_cover_pilot_Y_edge],
    [-bottom_cover_pilot_pair_offset, -bottom_cover_pilot_Y_edge],
    [ bottom_cover_pilot_pair_offset, -bottom_cover_pilot_Y_edge]
];

motor_tail_clear_W = 16;
motor_tail_clear_D = 10;
motor_tail_clear_H = 9.5;
motor_tail_on_negative_Y = true;

// Top rough reference
bottom_D = base_R*2; // 84
bottom_inner_D = bottom_inner_R*2; //72

// ---------------- VIEW OPTIONS ----------------
show_imported_body_ghost = true;
body_stl = "/home/hung/3D/stl/gripper_body_shell_exact_palm_v31_M4_pilot_holes_no_threads.stl";
label_size = 3.2;

module txt(t, p, s=label_size, col="black", rot=[0,0,0]) {
    color(col) translate(p) rotate(rot)
        linear_extrude(height=0.15)
            text(t, size=s, halign="center", valign="center", font="Liberation Sans:style=Bold");
}

module ring2d(outer_r, inner_r) {
    difference(){ circle(r=outer_r); circle(r=inner_r); }
}

module footprint_view(z=0) {
    translate([0,0,z]) {
        color([0,0.8,0,0.25]) linear_extrude(0.4) ring2d(base_R, bottom_inner_R);
        color("orange") linear_extrude(0.55) difference(){circle(r=bottom_inner_R+0.15); circle(r=bottom_inner_R-0.15);}        
        color("black") linear_extrude(0.5) difference(){circle(r=base_R+0.15); circle(r=base_R-0.15);}        
        // 6 lo nap motor
        for (p=bottom_cover_pilot_pts) {
            translate([p[0],p[1],0]) color("red") cylinder(h=1.2,d=bottom_cover_pilot_D,$fn=32);
            translate([p[0],p[1],1.25]) color("red") cylinder(h=0.25,d=6,$fn=32);
        }
        // 8 lo ngang reference
        for (i=[0:radial_mount_count-1]) {
            a=radial_mount_start_angle+i*360/radial_mount_count;
            translate([radial_mount_pitch_circle_R*cos(a), radial_mount_pitch_circle_R*sin(a),0])
                color("blue") cylinder(h=1.0,d=radial_mount_D,$fn=32);
        }
        // motor square and tail clearance top-down
        color([0.6,0,0.8,0.25]) linear_extrude(0.3) square([motor_body_W,motor_body_L], center=true);
        ysign = motor_tail_on_negative_Y ? -1 : 1;
        translate([0, ysign*(motor_body_L/2+motor_tail_clear_D/2),0])
            color([0,0.8,1,0.3]) linear_extrude(0.35) square([motor_tail_clear_W,motor_tail_clear_D], center=true);
    }
}

module side_section_diagram(xoff=90) {
    // khung side view Y-Z, dat tai X=xoff, dung truc X la ngang trong ban ve
    translate([xoff,0,0]) {
        // body total
        color([0,0.6,0,0.18]) cube([14, bottom_D, body_H], center=false);
        // inner lower tube
        translate([2,(bottom_D-bottom_inner_D)/2,0]) color([1,1,1,0.65]) cube([10,bottom_inner_D,motor_body_bottom_Z], center=false);
        // motor cavity
        translate([1,(bottom_D-motor_body_L)/2,motor_body_bottom_Z]) color([1,0.65,0,0.45]) cube([12,motor_body_L,motor_body_H], center=false);
        // partition
        translate([0,0,motor_partition_bottom_Z]) color([0.5,0,1,0.55]) cube([14,bottom_D,motor_partition_H], center=false);
        // top cap
        translate([0,0,body_H-top_cap_H]) color([0,0,1,0.25]) cube([14,bottom_D,top_cap_H], center=false);
        // radial hole Z
        translate([0,0,radial_mount_center_Z]) color("blue") cube([16,bottom_D,0.7], center=false);
        // 6 cover holes start
        translate([0,0,bottom_cover_pilot_start_Z]) color("red") cube([16,bottom_D,0.7], center=false);

        // labels
        txt("BODY H=98", [7,-10,body_H/2], 3, "green", [90,0,90]);
        txt("bottom OD D84", [7,bottom_D+8,3], 3, "black", [90,0,0]);
        txt("inner D72 / wall 6", [7,bottom_D+8,13], 3, "orange", [90,0,0]);
        txt("motor cavity 42.9 x 42.9 x 39", [7,bottom_D+10,motor_body_bottom_Z+motor_body_H/2], 3, "orange", [90,0,0]);
        txt("motor bottom Z=15", [7,-11,motor_body_bottom_Z], 2.8, "orange", [90,0,0]);
        txt("partition Z54..57, H3", [7,bottom_D+12,motor_partition_bottom_Z+1.5], 2.8, "purple", [90,0,0]);
        txt("top cap H3", [7,bottom_D+8,body_H-1.5], 2.8, "blue", [90,0,0]);
        txt("8 side M4 pilot Z=10 D3.3 R39", [7,-13,radial_mount_center_Z], 2.6, "blue", [90,0,0]);
        txt("6 cover M4 pilot start Z=15 depth20", [7,-15,bottom_cover_pilot_start_Z+4], 2.6, "red", [90,0,0]);
    }
}

module top_partition_view(z=62) {
    translate([-90,0,z]) {
        color([0.8,0.8,0.8,0.22]) linear_extrude(0.4) circle(r=base_R);
        color([0.6,0,0.8,0.55]) linear_extrude(0.8) difference(){
            square([60,60], center=true);
            circle(r=motor_center_hole_R);
            for(sx=[-1,1], sy=[-1,1]) translate([sx*motor_mount_pitch/2, sy*motor_mount_pitch/2]) circle(d=motor_mount_hole_D,$fn=32);
        }
        color("purple") cylinder(h=1.2,r=motor_center_hole_R,$fn=64);
        for(sx=[-1,1], sy=[-1,1]) translate([sx*motor_mount_pitch/2, sy*motor_mount_pitch/2,0]) color("red") cylinder(h=1.4,d=motor_mount_hole_D,$fn=32);
        txt("NEMA17 face", [0,45,0], 3.5, "purple");
        txt("center R11 / D22", [0,36,0], 3, "purple");
        txt("4 holes pitch31 D3.4", [0,-38,0], 3, "red");
    }
}

// Ghost imported body at origin for visual comparison
if (show_imported_body_ghost)
    color([0.4,0.4,0.4,0.12]) import(body_stl, convexity=10);

// Bottom footprint at z=-4
footprint_view(-5);
txt("BOTTOM FOOTPRINT: OD D84, INNER D72", [0,55,-4], 4, "black");
txt("6x cover holes: D3.3 at (+/-24.45,0), (+/-10,+/-24.45)", [0,-55,-4], 3, "red");
txt("8x base/body side holes: R39, Z10, D3.3", [0,-62,-4], 3, "blue");

side_section_diagram(70);
top_partition_view(10);

echo("BODY v31 SUMMARY");
echo("overall", "H", body_H, "bottom OD", bottom_D, "bottom inner D", bottom_inner_D, "lower wall", bottom_tube_wall_t);
echo("top cap", top_cap_H, "wall_t upper", wall_t);
echo("motor partition", "bottom Z", motor_partition_bottom_Z, "top Z", motor_partition_top_Z, "H", motor_partition_H);
echo("motor cavity", [motor_body_W, motor_body_L, motor_body_H], "bottom Z", motor_body_bottom_Z, "top approx", motor_body_bottom_Z+motor_body_H-motor_cavity_top_gap);
echo("NEMA17 holes", "center R", motor_center_hole_R, "pitch", motor_mount_pitch, "hole D", motor_mount_hole_D);
echo("8 side holes", "count", radial_mount_count, "R", radial_mount_pitch_circle_R, "Z", radial_mount_center_Z, "D", radial_mount_D);
echo("6 bottom cover holes", bottom_cover_pilot_pts, "D", bottom_cover_pilot_D, "start Z", bottom_cover_pilot_start_Z, "depth", bottom_cover_pilot_depth);
echo("wire/motor tail clearance", [motor_tail_clear_W, motor_tail_clear_D, motor_tail_clear_H], "side", motor_tail_on_negative_Y ? "-Y" : "+Y");
