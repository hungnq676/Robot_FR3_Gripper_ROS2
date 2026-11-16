// ============================================================
//  FR3 GRIPPER QUICK-RELEASE ADAPTER v14
//  v14 + sua ren di ra tan mieng lo de oc/gripper bat ren ngay
//  Self-contained OpenSCAD, no external library
// ============================================================

$fn = 128;
eps = 0.05;

// ---------- THONG SO THEO BAN VE FR3 ----------
adapter_OD   = 63.0;     // Theo ban ve: Ø63
adapter_H    = 12.0;     // Do day phan dia nam ngoai mat flange

// Go dinh tam mat duoi: lap vao lo Ø31.5H7 sau 6.2 tren dau FR3
// De in 3D khong ket, dung 31.35~31.40 thay vi 31.50
locator_OD   = 31.40;
locator_H    = 6.00;     // Nho hon 6.2 mot chut de khong can day lo

bolt_pcd     = 50.0;     // Theo ban ve: PCD Ø50
bolt_angle   = 45;       // 4 lo nam lech 45 do so voi truc X/Y
bolt_d       = 6.4;      // Lo clearance cho vit M6

// ---------- REN TRONG CHO GRIPPER - THU NHO DONG DEU 2 VONG ----------
// Goc cu theo M30x2 gan dung:
base_thread_major_D = 30.3;
base_thread_minor_D = 28.2;

// Giam ca 2 duong kinh lon/nho cua ren cung mot luong,
// nen do lech giua 2 vong van giu nguyen: 30.3-28.2 = 2.1mm.
// Can than: neu gripper that su la ren ngoai M30x2 thi KHONG duoc giam,
// vi se khong xoay vao duoc. Ban nay dang gan voi M28x2 hon.
thread_shrink_D = 2.5;

thread_major_D = base_thread_major_D - thread_shrink_D;   // 27.8 khi shrink=2.5
thread_minor_D = base_thread_minor_D - thread_shrink_D;   // 25.7 khi shrink=2.5
thread_pitch   = 2.0;
thread_width   = 1.20;
thread_steps   = 56;

thread_top_clearance    = -0.90; // AM = cutter vuot qua mat tren, ren mo ra tan mieng lo
thread_bottom_clearance = -0.90; // AM = cutter vuot qua mat duoi, tranh dau ren bi thut vao trong

// ---------- LO REN NGANG M6x1 DE BAT OC CHOT ----------
// Day la lo REN M6x1.0, khong phai lo tron Ø6.4.
// M6 ren tho tieu chuan co buoc p = 1.0mm.
side_thread_enable    = true;
side_thread_angle     = 0;          // 0 do = tu phia +X khoan vao. Co the doi 90/180/270.
side_thread_z         = adapter_H/2; // nam giua doan dia chinh 12mm => z=6mm
side_thread_depth     = 12.0;       // sau 1.2cm tu mat ngoai vao, KHONG xuyen vao loi trung tam
side_thread_major_D   = 6.0;        // dinh ren M6
side_thread_minor_D   = 5.0;        // duong kinh lo moi/tap drill cho M6x1
side_thread_pitch     = 1.0;        // buoc ren M6 tieu chuan
side_thread_width     = 0.55;       // do day cutter ren, gan 0.55~0.65 cho M6x1
side_thread_steps     = 48;         // do min cua ren ngang
side_thread_entry_gap = -0.45;      // AM = ren mo ra tan mat ngoai, oc bat ren ngay tu mieng lo
side_thread_end_gap   = 0.35;       // chua day lo mut, khong cat het ra day lo

// Canh bao: voi adapter_OD=63 va ren gripper hien tai Ø27.8,
// tu mat ngoai vao toi dinh ren trung tam khoang 17.6mm.
// Neu side_thread_depth = 12mm thi oc se KHONG cham vao gripper.
// Neu muon oc that su khoa gripper, can tang depth >= 18mm hoac tao flat/rang khoa tren gripper.

// Bat true de cat doi model va nhin thay ren ben trong
section_view = false;

echo("main thread major D =", thread_major_D);
echo("main thread minor D =", thread_minor_D);
echo("main thread delta =", thread_major_D - thread_minor_D);
echo("wall from OD to main thread major =", adapter_OD/2 - thread_major_D/2);
echo("side threaded hole = M6x1, depth =", side_thread_depth, "minor D =", side_thread_minor_D, "major D =", side_thread_major_D);
echo("NOTE: depth 12mm does not touch gripper if wall > 12mm");
echo("main thread now exits top/bottom because clearances are negative");
echo("side M6 thread now exits outer face because entry gap is negative");

// ---------- HAM / MODULE REN ----------
function helix_point_limited(i, j, steps, z_start, z_end, pitch, minor_r, major_r, w, start_ang) =
    let(
        zc = z_start + (z_end - z_start) * i / steps,
        a  = 360 * zc / pitch + start_ang,
        rr = (j == 1) ? major_r : minor_r,
        zz = zc + ((j == 0) ? -w/2 : ((j == 2) ? w/2 : 0))
    )
    [rr*cos(a), rr*sin(a), zz];

// Cutter dang day xoan tiet dien tam giac chu V.
// Dung trong difference() de cat ren trong theo truc Z.
module internal_thread_cutter_limited(minor_D, major_D, pitch, height, w, top_clearance, bottom_clearance, steps_per_turn=56) {
    minor_r = minor_D/2 - eps;   // overlap nhe de tranh mat trung nhau/non-manifold
    major_r = major_D/2 + eps;

    z_start = bottom_clearance + w/2;
    z_end   = height - top_clearance - w/2;

    steps = max(6, ceil(((z_end - z_start)/pitch) * steps_per_turn));

    points = [
        for (i = [0:steps])
            for (j = [0:2])
                helix_point_limited(i, j, steps, z_start, z_end, pitch, minor_r, major_r, w, 0)
    ];

    faces = concat(
        [ for (i = [0:steps-1]) [3*i+0, 3*(i+1)+0, 3*(i+1)+1, 3*i+1] ],
        [ for (i = [0:steps-1]) [3*i+1, 3*(i+1)+1, 3*(i+1)+2, 3*i+2] ],
        [ for (i = [0:steps-1]) [3*i+2, 3*(i+1)+2, 3*(i+1)+0, 3*i+0] ],
        [[0, 1, 2]],
        [[3*steps+0, 3*steps+2, 3*steps+1]]
    );

    polyhedron(points = points, faces = faces, convexity = 10);
}

// ---------- MODEL CHINH ----------
module solid_body() {
    union() {
        // Dia chinh: mat tren phang hoan toan
        cylinder(h = adapter_H, d = adapter_OD);

        // Go dinh tam nam o MAT DUOI, khong lam loi/lom mat tren
        translate([0, 0, -locator_H])
            cylinder(h = locator_H + eps, d = locator_OD);
    }
}

// Lo ren ngang M6x1 dang lo mut, khoan tu thanh ngoai vao 12mm.
// He truc cuc bo cua cutter: local Z = chieu sau lo.
// Xoay -90 quanh Y de local Z huong vao trong theo truc -X.
module side_M6x1_blind_threaded_hole() {
    outer_x = adapter_OD/2 + eps;

    rotate([0, 0, side_thread_angle])
        translate([outer_x, 0, side_thread_z])
            rotate([0, -90, 0]) {
                // Lo moi/tap drill Ø5.0, sau 12mm
                cylinder(h = side_thread_depth, d = side_thread_minor_D, center = false);

                // Cat ren M6x1: minor Ø5, major Ø6, pitch 1mm
                internal_thread_cutter_limited(
                    minor_D = side_thread_minor_D,
                    major_D = side_thread_major_D,
                    pitch   = side_thread_pitch,
                    height  = side_thread_depth,
                    w       = side_thread_width,
                    top_clearance    = side_thread_end_gap,
                    bottom_clearance = side_thread_entry_gap,
                    steps_per_turn   = side_thread_steps
                );
            }
}

module adapter_model() {
    total_H = adapter_H + locator_H;

    difference() {
        solid_body();

        // Lo trung tam tron xuyen qua ca dia va go dinh tam
        translate([0, 0, -locator_H - eps])
            cylinder(h = total_H + 2*eps, d = thread_minor_D);

        // Cat ren trong thu nho dong deu xuyen qua ca cum
        translate([0, 0, -locator_H])
            internal_thread_cutter_limited(
                minor_D = thread_minor_D,
                major_D = thread_major_D,
                pitch   = thread_pitch,
                height  = total_H,
                w       = thread_width,
                top_clearance    = thread_top_clearance,
                bottom_clearance = thread_bottom_clearance,
                steps_per_turn   = thread_steps
            );

        // Lo ren ngang M6x1, sau 12mm, khong khoan xuyen vao loi trung tam
        if (side_thread_enable)
            side_M6x1_blind_threaded_hole();

        // 4 lo clearance cho vit M6 bat vao dau FR3
        for (i = [0:3]) {
            angle = bolt_angle + i * 90;
            translate([(bolt_pcd/2)*cos(angle), (bolt_pcd/2)*sin(angle), -eps])
                cylinder(h = adapter_H + 2*eps, d = bolt_d);
        }
    }
}

if (section_view) {
    intersection() {
        adapter_model();
        translate([-adapter_OD/2 - 2, 0, -locator_H - 2])
            cube([adapter_OD + 4, adapter_OD/2 + 4, adapter_H + locator_H + 4]);
    }
} else {
    adapter_model();
}
