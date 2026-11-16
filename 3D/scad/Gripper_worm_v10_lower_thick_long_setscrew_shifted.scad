// ============================================================
// GRIPPER WORM - v10 lower shaft thicker + longer
// Source: /home/hung/3D/stl/Gripper_worm_v9_M3x0p5_setscrew_5mm.stl
// Don vi: mm
//
// Theo yeu cau:
// - Giu nguyen phan 1: tru nho tren cung.
// - Giu nguyen phan 2: doan tru lon/worm xoan ren.
// - Phan 3 ben duoi: giu nguyen lo theo truc lap truc dong co,
//   nhung tang ban kinh ngoai them 3mm va dai them 9mm.
// - Lo ngang giu nguyen kich thuoc, chi dich xuong 5mm.
//
// Toa do STL v9: truc worm nam theo truc Y, vat nam ngang.
// ============================================================

$fn = 160;
eps = 0.05;

source_stl = "/home/hung/3D/stl/Gripper_worm_v9_M3x0p5_setscrew_5mm.stl";

// Do tu file v9 / cac ban truoc
orig_total_len = 42.0;
orig_lower_len = 11.0;
orig_lower_OD  = 11.0;

// Sua phan 3
lower_radius_add = 3.0;
new_lower_OD = orig_lower_OD + 2*lower_radius_add; // D17
lower_extend_down = 9.0;
new_lower_y0 = -lower_extend_down;
new_lower_len = orig_lower_len + lower_extend_down; // 20mm, tu Y=-9 den Y=11

// Lo D-shaft theo truc: GIU NGUYEN PROFILE nhu v5/v9
shaft_bore_D = 5.50;
shaft_flat_to_round = 4.50;
shaft_flat_angle = 0;
// Cat tu mieng day moi vao du sau de lo chay qua toan bo phan duoi + overlap vao than cu.
shaft_bore_depth = 31.0; // 22 cu + 9mm dai them
entry_chamfer_depth = 0.50;
entry_chamfer_D = 6.20;

// Lo ngang M3x0.5 cua v9 nam tai Y=5mm. Ban moi dich xuong 5mm => Y=0mm.
// Giu kich thuoc ren/pilot nhu v9: dung cutter D3.0 nominal, co the taro M3x0.5 sau in.
setscrew_old_y = 5.0;
setscrew_shift_down = 5.0;
setscrew_y = setscrew_old_y - setscrew_shift_down; // 0mm
setscrew_D = 3.0;
setscrew_len = new_lower_OD + 3.0;
setscrew_axis_angle = 0; // 0: khoet ngang theo truc X tai Z=0

module cyl_y(d, h, center=false) {
    rotate([-90, 0, 0]) cylinder(h=h, d=d, center=center);
}

module cyl_x(d, h, center=true) {
    rotate([0, 90, 0]) cylinder(h=h, d=d, center=center);
}

module d_shaft_profile_2d(d=5.5, flat_to_round=4.5) {
    r = d/2;
    x_flat = r - flat_to_round;
    intersection() {
        circle(d=d);
        translate([x_flat, -d]) square([d*2, d*2], center=false);
    }
}

module d_shaft_bore_y(d=5.5, flat_to_round=4.5, h=31, angle=0) {
    rotate([0, angle, 0])
        rotate([-90, 0, 0])
            linear_extrude(height=h, center=false, convexity=10)
                d_shaft_profile_2d(d=d, flat_to_round=flat_to_round);
}

module original_v9() {
    import(source_stl, convexity=10);
}

module lower_thickener_and_extension() {
    // Cylinder moi phu tu Y=-9 den Y=11, D17.
    // Phan overlap 0..11 se lap day lo ngang cu trong v9; sau do ta cat lai lo moi tai Y=0.
    translate([0, new_lower_y0, 0]) cyl_y(new_lower_OD, new_lower_len + eps);
}

module body_before_new_cuts() {
    union() {
        original_v9();
        lower_thickener_and_extension();
    }
}

module new_setscrew_hole() {
    translate([0, setscrew_y, 0])
        rotate([0, 0, setscrew_axis_angle])
            cyl_x(setscrew_D, setscrew_len, center=true);
}

module final_worm_v10() {
    difference() {
        body_before_new_cuts();

        // Cat lai lo D-shaft tu mieng day moi, giu profile/kich thuoc cu.
        translate([0, new_lower_y0 - eps, 0])
            d_shaft_bore_y(d=shaft_bore_D, flat_to_round=shaft_flat_to_round,
                           h=shaft_bore_depth + 2*eps, angle=shaft_flat_angle);

        // Vat mieng lo nhu ban cu, dat tai mieng day moi.
        if (entry_chamfer_depth > 0)
            translate([0, new_lower_y0 - eps, 0])
                cyl_y(entry_chamfer_D, entry_chamfer_depth + eps);

        // Lo ngang moi: cung kich thuoc, dich xuong 5mm.
        new_setscrew_hole();
    }
}

final_worm_v10();

echo("v10 lower OD", new_lower_OD, "old OD", orig_lower_OD, "radius +", lower_radius_add);
echo("v10 lower Y", [new_lower_y0, orig_lower_len], "length", new_lower_len, "extend down", lower_extend_down);
echo("D-shaft kept", "D", shaft_bore_D, "flat-to-round", shaft_flat_to_round, "depth", shaft_bore_depth);
echo("setscrew moved", "old Y", setscrew_old_y, "new Y", setscrew_y, "D", setscrew_D);
