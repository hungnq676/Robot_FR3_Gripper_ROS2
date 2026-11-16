// ============================================================
//  GRIPPER MOTOR COVER - v1
//  Nap bit khoang dong co cho body v31
//  Don vi: mm
//
//  - Lap vao truoc, sau do moi lap base v9.
//  - Than body v31 co long duoi D72 (R36), nap nho hon 1mm => D71.
//  - Day 3.5mm.
//  - 6 lo bat oc trung voi 6 lo moi M4 trong body v31.
//  - Co o/khe cho day motor di xuong ve phia -Y, trung huong motor_tail_clearance.
// ============================================================

$fn = 160;
eps = 0.05;

// Kich thuoc nap
body_bottom_inner_D = 72.0;          // body v31: bottom_inner_R = 36
cover_clearance_D   = 1.0;           // nho hon duong tron rong cua than 1mm
cover_D             = body_bottom_inner_D - cover_clearance_D; // D71
cover_R             = cover_D/2;
cover_H             = 2.0;

// Lo bat oc: body v31 co 6 lo moi M4 D3.3 de taro/bat oc.
// Tren nap de lo thong M4, cong clearance nhe de de lap.
screw_clear_D = 3.3; // cung size voi 6 lo tren body

// Lay dung toa do tu body v31
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

// O/khe day motor di xuong.
// Body v31: motor_tail_clear_W=16, motor_tail_clear_D=10 ve phia -Y.
// Nap khoet tu mep ngoai vao trong de day thoat xuong, cong rong nhe.
wire_slot_W = 10.0;
wire_slot_depth_from_edge = 10.0;
wire_slot_end_round_D = wire_slot_W;
wire_slot_on_negative_Y = true;

module wire_slot_cut() {
    ysign = wire_slot_on_negative_Y ? -1 : 1;
    // Chi cat nho dung vi tri day, sau vao tam ~4mm.
    translate([0, ysign*(cover_R - wire_slot_depth_from_edge + wire_slot_W/2), cover_H/2])
        cube([wire_slot_W, wire_slot_depth_from_edge, cover_H + 2*eps], center=true);
}

module gripper_motor_cover_v1() {
    difference() {
        cylinder(h=cover_H, d=cover_D);

        // 6 lo thong bat oc len body
        for (p = cover_hole_pts) {
            translate([p[0], p[1], -eps])
                cylinder(h=cover_H + 2*eps, d=screw_clear_D, $fn=48);
        }

        // Khe/o day di xuong
        wire_slot_cut();
    }
}

gripper_motor_cover_v1();

echo("motor cover v1", "D", cover_D, "H", cover_H);
echo("6 holes", cover_hole_pts, "clear D", screw_clear_D);
echo("wire slot", [wire_slot_W, wire_slot_depth_from_edge], "side", wire_slot_on_negative_Y ? "-Y" : "+Y");
