// ============================================================
//  GRIPPER BASE FOR FR3 QUICK-RELEASE TOOL - v8 INNER SLEEVE 8 SIDE PILOT HOLES
//  - Ke thua v2: ren duc theo file tool, de D84, bac giua, vien ngoai
//  - Them 4 lo bat oc M3 quanh vien ngoai, MO TU MAT PHANG LY DOI DIEN PHIA REN
//  - Them lo chot ngang CO REN M6x1 tren VANH NGOAI, can theo 4 lo M3 va thang voi lo chot tool
//  - v7: Sua REN DUC TRUNG TAM mo sat mat tren/mat duoi, khong bi khuyet dau ren
//  - v8: bo 4 lo M3 tren mat phang noi; them thanh ong trong day 3mm de noi than gripper, co 8 lo ngang M4 pilot
//  - Khong dung thu vien ngoai
// ============================================================

$fn = 160;
eps = 0.04;

// ============================================================
//  THONG SO REN DUC DE XOAY VAO TOOL CU BAN GUI
//  File tool cu co ren trong sau khi shrink:
//      thread_major_D = 30.3 - 2.5 = 27.8
//      thread_minor_D = 28.2 - 2.5 = 25.7
//      pitch          = 2.0
//      thread_width   = 1.20
//
//  Ren duc ben nay lay nho hon ren trong mot chut de in 3D xoay duoc.
//  Neu chat/long thi chi sua fit_clearance_D.
// ============================================================
tool_internal_major_D = 27.8;
tool_internal_minor_D = 25.7;
thread_pitch          = 2.0;
thread_width          = 1.20;

// Clearance theo DUONG KINH, khuyen nghi PETG/PLA: 0.25 -> 0.45mm
fit_clearance_D       = 0.30;

male_thread_major_D   = tool_internal_major_D - fit_clearance_D;
male_thread_minor_D   = tool_internal_minor_D - fit_clearance_D;
male_thread_H         = 17.0;
thread_steps_per_turn = 72;

// ============================================================
//  CAC BAC DE THEO MO TA CUA BAN
// ============================================================
base_D = 84.0;                 // phan ban kinh lon nhat: D84
base_H = 5.0;                  // lop de / nen day 5mm

middle_step_extra_R = 4.0;     // bac vua co ban kinh lon hon ban kinh ren 4mm
middle_step_D       = male_thread_major_D + 2*middle_step_extra_R;
middle_step_H       = 4.0;

// Vien tru bao quanh ngoai:
// ban kinh ngoai = ban kinh lon nhat cua de
// ban kinh trong = ban kinh ngoai - 10mm
outer_wall_enable   = true;
outer_wall_D        = base_D;
outer_wall_thick    = 10.0;
outer_wall_inner_D  = outer_wall_D - 2*outer_wall_thick;

// Cao cua vien = cao bac vua + cao ren
// Bat dau tu mat tren cua base_D84, nen dinh vien bang dinh ren.
outer_wall_H = middle_step_H + male_thread_H;

// ============================================================
//  4 LO BAT OC M3 CHO PHAN GRIPPER TIEP THEO
// ============================================================
m3_holes_enable = false;   // v8: khong can 4 lo M3 tren mat phang noi nua

// Tam lo nam tren duong tron co ban kinh nho hon ban kinh lon nhat 7.5mm:
// R = 84/2 - 7.5 = 34.5mm
m3_hole_R = base_D/2 - 7.5;

// Goc dat 4 lo:
// 45 do: tao hinh chu X, giong bolt_angle=45 trong file tool cu
// 0 do : tao hinh dau cong +
m3_angle_offset = 45;

// Kieu lo:
// true  = lo ren trong M3x0.5 in truc tiep
// false = lo tron pilot de tu taro/taro tay M3 sau khi in
m3_thread_enable = true;

// M3 x 0.5 theo anh ban gui
m3_major_D = 3.05;       // dinh ren M3, cong them nhe de de in
m3_minor_D = 2.55;       // lo moi cho M3, neu khoet/taro PETG co the 2.6~2.8
m3_pitch   = 0.50;
m3_width   = 0.28;
m3_thread_depth = 12.0; // chieu sau lo tu mat phang ly doi dien phia ren di vao trong
m3_steps_per_turn = 36;

// Tuy chon khoet am dau oc.
// Neu bat true, phan khoet dau oc nam ngay tren mat phang ly doi dien phia ren.
m3_counterbore_enable = false;
m3_head_D = 6.2;
m3_head_H = 3.1;

// Tuy chon xem cat doi
section_view = false;

// ============================================================
//  V8 - THANH ONG NOI VOI THAN GRIPPER
// ============================================================
// Than gripper V31 co day ngoai D84, long trong R=36 => D72.
// Ong noi nay chui vao trong long than gripper, de giua rong lap mach dien tu.
connector_sleeve_enable = true;
connector_sleeve_outer_D = 72.0;     // vua long trong cua than gripper D=72
connector_sleeve_wall_t  = 3.0;      // thanh trong day 3mm theo yeu cau
connector_sleeve_inner_D = connector_sleeve_outer_D - 2*connector_sleeve_wall_t; // D66 rong de mach
connector_sleeve_H = 15.0;           // chieu dai ong chui vao than, co the tang/giam
connector_sleeve_Z0 = -connector_sleeve_H; // nho ra tu mat phang z=0 ve phia than gripper

// 8 lo ngang bat vit noi 2 phan. Dung lo moi M4 D=3.3mm de bat/taro sau in.
connector_side_holes_enable = true;
connector_side_hole_count = 8;
connector_side_hole_D = 3.30;
connector_side_hole_Z = connector_sleeve_Z0 + 8.0; // can voi than gripper: tam lo cach mat noi 8mm
connector_side_hole_len = connector_sleeve_wall_t + 3.0; // cat qua thanh 3mm mot chut
connector_side_hole_start_angle = 0;



// ============================================================
//  LO CHOT NGANG / LO ACCESS CHOT TREN VANH NGOAI
// ============================================================
lock_hole_enable = true;

// Vi tri goc XY cua lo chot duoc can theo 4 lo M3:
// - Neu m3_angle_offset = 45  => 4 lo M3 tao chu X, lo chot nam truc +X, giong side_thread_angle=0 cua file tool cu.
// - Neu m3_angle_offset = 0   => 4 lo M3 tao dau +, lo chot tu dong xoay theo de giu cung tuong quan.
lock_hole_angle = m3_angle_offset - 45;

// Do cao lay moc tu mat phang ly vua duc 4 lo M3, tuc z=0.
// Theo yeu cau: cao lop ban kinh lon nhat + lop ban kinh thu 2 + 9mm
// Luu y: KHONG tinh thang vanh ngoai vao cong thuc nay.
lock_hole_z = base_H + middle_step_H + 9.0;

// Lo nay chi khoet qua PHAN VANH NGOAI D84/D64, khong di qua tam.
// outer_wall_thick = 10mm nen cat qua do day vanh mot chut de ren mo ca 2 dau.
// Lo nay CHI nam trong vanh D84/D64, khong chay vao tam.
lock_hole_depth = outer_wall_thick + 0.8;

// Ren chot ngang: lay cung kieu/cung huong voi lo M6x1 tren file tool cu.
// Trong file tool cu: side_thread_major_D=6.0, side_thread_minor_D=5.0, pitch=1.0.
lock_thread_enable = true;
lock_thread_major_D = 6.0;    // dinh ren M6
lock_thread_minor_D = 5.0;    // lo moi/tap drill M6x1
lock_thread_pitch   = 1.0;    // buoc ren M6 tieu chuan
lock_thread_width   = 0.55;   // giong file tool cu
lock_thread_steps_per_turn = 48;

// Neu chi muon lo tron khong ren, dat lock_thread_enable=false va dung duong kinh nay.
lock_hole_clearance_D = 6.4;

// ============================================================
//  HAM TAO REN NGOAI DANG V, CUNG KIEU VOI REN TRONG FILE TOOL
//  v7: Cho gan ren vuot qua mat dau/cuoi roi cat lai bang envelope
//      de ren cham tan mat tren va mat duoi, khong bi khuyet dau ren.
// ============================================================
function ext_thread_point(i, j, steps, z_start, z_end, pitch, minor_r, major_r, w, start_ang) =
    let(
        zc = z_start + (z_end - z_start) * i / steps,
        a  = 360 * zc / pitch + start_ang,
        rr = (j == 1) ? major_r : minor_r,
        zz = zc + ((j == 0) ? -w/2 : ((j == 2) ? w/2 : 0))
    )
    [rr*cos(a), rr*sin(a), zz];

module external_thread_ridge_open_to_faces(
    minor_D,
    major_D,
    pitch,
    height,
    w,
    top_clearance=-0.90,
    bottom_clearance=-0.90,
    steps_per_turn=72
) {
    minor_r = minor_D/2 - eps;
    major_r = major_D/2 + eps;

    // Cach tinh giong module ren trong o file tool:
    // clearance am => cutter/ridge vuot qua mat dau de ren mo sat mieng.
    // Sau do male_thread_body() se cat bang envelope z=0..height.
    z_start = bottom_clearance + w/2;
    z_end   = height - top_clearance - w/2;
    steps = max(12, ceil(((z_end - z_start)/pitch) * steps_per_turn));

    points = [
        for (i = [0:steps])
            for (j = [0:2])
                ext_thread_point(i, j, steps, z_start, z_end, pitch, minor_r, major_r, w, 0)
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

module male_thread_body() {
    // v7: Tao ren dai vuot 2 dau, roi cat lai trong hop z=0..male_thread_H.
    // Nhu vay gan ren cham tan mat duoi va mat tren, khong con bi khuyet mot doan.
    intersection() {
        union() {
            // loi ren
            cylinder(h = male_thread_H, d = male_thread_minor_D);

            // gan ren ngoai mo sat 2 mat dau
            external_thread_ridge_open_to_faces(
                minor_D = male_thread_minor_D,
                major_D = male_thread_major_D,
                pitch   = thread_pitch,
                height  = male_thread_H,
                w       = thread_width,
                top_clearance    = -0.90,
                bottom_clearance = -0.90,
                steps_per_turn = thread_steps_per_turn
            );
        }

        // Envelope cat gon lai, khong cho ren thua vuot khoi chieu cao 17mm.
        translate([0,0,0])
            cylinder(h = male_thread_H, d = male_thread_major_D + 0.20);
    }
}

// ============================================================
//  HAM TAO REN TRONG CHO LO M3
// ============================================================
function int_thread_point(i, j, steps, z_start, z_end, pitch, minor_r, major_r, w, start_ang) =
    let(
        zc = z_start + (z_end - z_start) * i / steps,
        a  = 360 * zc / pitch + start_ang,
        rr = (j == 1) ? major_r : minor_r,
        zz = zc + ((j == 0) ? -w/2 : ((j == 2) ? w/2 : 0))
    )
    [rr*cos(a), rr*sin(a), zz];

// Cutter dang day xoan tiet dien tam giac chu V, dung trong difference() de cat ren trong theo truc Z.
module internal_thread_cutter_limited(minor_D, major_D, pitch, height, w, top_clearance, bottom_clearance, steps_per_turn=36) {
    minor_r = minor_D/2 - eps;
    major_r = major_D/2 + eps;

    z_start = bottom_clearance + w/2;
    z_end   = height - top_clearance - w/2;
    steps = max(6, ceil(((z_end - z_start)/pitch) * steps_per_turn));

    points = [
        for (i = [0:steps])
            for (j = [0:2])
                int_thread_point(i, j, steps, z_start, z_end, pitch, minor_r, major_r, w, 0)
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

// Mot lo M3 thang dung, khoet tu mat phang ly doi dien phia ren di vao trong.
module one_m3_mount_hole() {
    // v4: Lo M3 duoc khoet tu MAT PHANG LY DOI DIEN PHIA REN.
    // Trong toa do file nay, mat do la mat z=0 cua de D84.
    // Ren/bao quanh nam ve phia +Z, nen lo di tu z=0 len tren.
    mouth_z = 0;

    translate([0, 0, mouth_z - eps]) {
        // Lo moi/pilot truoc, mo mieng o mat phang ly z=0
        cylinder(h = m3_thread_depth + 2*eps, d = m3_minor_D);

        // Cat ren M3x0.5 neu bat
        if (m3_thread_enable)
            internal_thread_cutter_limited(
                minor_D = m3_minor_D,
                major_D = m3_major_D,
                pitch   = m3_pitch,
                height  = m3_thread_depth + 2*eps,
                w       = m3_width,
                top_clearance    = 0.35,   // dau trong la lo mut, chua chut day
                bottom_clearance = -0.20,  // ren mo sat mieng lo o mat phang ly
                steps_per_turn   = m3_steps_per_turn
            );
    }

    // Khoet dau oc tuy chon, khoet tu mat phang ly z=0 len tren
    if (m3_counterbore_enable)
        translate([0, 0, mouth_z - eps])
            cylinder(h = m3_head_H + eps, d = m3_head_D);
}

module all_m3_mount_holes() {
    for (i = [0:3]) {
        a = m3_angle_offset + i*90;
        translate([m3_hole_R*cos(a), m3_hole_R*sin(a), 0])
            one_m3_mount_hole();
    }
}


// Lo chot ngang khoet tu mat ngoai cua vanh vao trong theo phuong ban kinh.
// No chi di qua do day vanh, khong chay toi tam.
// Huong ren/local-Z giong cach file tool cu tao lo chot: tu mat ngoai di vao trong.
module one_lock_access_hole() {
    rotate([0, 0, lock_hole_angle])
        translate([outer_wall_D/2 + eps, 0, lock_hole_z])
            rotate([0, -90, 0]) {
                if (lock_thread_enable) {
                    // Lo moi/tap drill cho M6x1
                    cylinder(h = lock_hole_depth + 2*eps, d = lock_thread_minor_D, center = false);

                    // Cat ren trong M6x1, mo ren ngay tu mieng ngoai va thong qua mat trong cua vanh.
                    internal_thread_cutter_limited(
                        minor_D = lock_thread_minor_D,
                        major_D = lock_thread_major_D,
                        pitch   = lock_thread_pitch,
                        height  = lock_hole_depth + 2*eps,
                        w       = lock_thread_width,
                        top_clearance    = -0.20,
                        bottom_clearance = -0.45,
                        steps_per_turn   = lock_thread_steps_per_turn
                    );
                } else {
                    cylinder(h = lock_hole_depth + 2*eps, d = lock_hole_clearance_D, center = false);
                }
            }
}


// ============================================================
//  V8 - ONG NOI + 8 LO NGANG
// ============================================================
module connector_inner_sleeve_solid() {
    if (connector_sleeve_enable) {
        translate([0, 0, connector_sleeve_Z0])
            difference() {
                cylinder(h = connector_sleeve_H, d = connector_sleeve_outer_D);
                translate([0,0,-eps])
                    cylinder(h = connector_sleeve_H + 2*eps, d = connector_sleeve_inner_D);
            }
    }
}

module one_connector_side_pilot_hole(angle_deg) {
    // Lo ngang bat tu ngoai than vao thanh ong noi.
    // Chi cat qua thanh ong 5mm, khong chay vao khoang rong qua sau.
    rotate([0,0,angle_deg])
        translate([connector_sleeve_outer_D/2 + eps, 0, connector_side_hole_Z])
            rotate([0,-90,0])
                cylinder(h = connector_side_hole_len + 2*eps, d = connector_side_hole_D, $fn=48);
}

module connector_side_pilot_holes() {
    if (connector_sleeve_enable && connector_side_holes_enable) {
        for (i=[0:connector_side_hole_count-1])
            one_connector_side_pilot_hole(connector_side_hole_start_angle + i*360/connector_side_hole_count);
    }
}

// ============================================================
//  MODEL RAN
// ============================================================
module gripper_base_solid() {
    union() {
        // Nen lon D84 day 5mm
        cylinder(h = base_H, d = base_D);

        // Bac vua o tam, cao 4mm
        translate([0, 0, base_H])
            cylinder(h = middle_step_H, d = middle_step_D);

        // Ren duc o tam, cao 17mm
        translate([0, 0, base_H + middle_step_H])
            male_thread_body();

        // Vien tru bao quanh ngoai, day 10mm theo ban kinh
        if (outer_wall_enable)
            translate([0, 0, base_H])
                difference() {
                    cylinder(h = outer_wall_H, d = outer_wall_D);
                    translate([0,0,-eps])
                        cylinder(h = outer_wall_H + 2*eps, d = outer_wall_inner_D);
                }

        // v8: thanh ong noi voi than gripper, nam o mat phang z=0 va nho xuong phia than
        connector_inner_sleeve_solid();
    }
}

module gripper_base_v8() {
    difference() {
        gripper_base_solid();

        if (m3_holes_enable)
            all_m3_mount_holes();

        if (lock_hole_enable)
            one_lock_access_hole();

        // v8: 8 lo ngang tren thanh ong noi de bat vit voi than gripper
        connector_side_pilot_holes();
    }
}

if (section_view) {
    intersection() {
        gripper_base_v8();
        translate([-base_D/2-2, 0, -1])
            cube([base_D+4, base_D/2+4, base_H + outer_wall_H + 2]);
    }
} else {
    gripper_base_v8();
}

// Echo de OpenSCAD hien thong so check nhanh
echo("male_thread_major_D", male_thread_major_D);
echo("male_thread_minor_D", male_thread_minor_D);
echo("thread_pitch", thread_pitch);
echo("middle_step_D", middle_step_D);
echo("outer_wall_inner_D", outer_wall_inner_D);
echo("outer_wall_H", outer_wall_H);
echo("m3_hole_R", m3_hole_R);
echo("m3_angle_offset", m3_angle_offset);
echo("m3 thread", m3_major_D, m3_minor_D, m3_pitch);
echo("NOTE: set m3_angle_offset=0 for plus pattern, 45 for X pattern");
echo("v9: M3 face holes disabled; inner connector sleeve D72/D66 H15 below z=0, 3mm wall, 8 horizontal M4 pilot holes D3.3 at Z=-7");
echo("lock_hole_angle", lock_hole_angle);
echo("lock_hole_z = base_H + middle_step_H + 9", lock_hole_z);
echo("lock_hole_depth", lock_hole_depth);
echo("lock thread M6x1 major/minor/pitch", lock_thread_major_D, lock_thread_minor_D, lock_thread_pitch);

echo("connector sleeve", "outer D", connector_sleeve_outer_D, "inner D", connector_sleeve_inner_D, "wall", connector_sleeve_wall_t, "Z", [connector_sleeve_Z0, connector_sleeve_Z0+connector_sleeve_H]); // wall=3mm
echo("connector 8 side pilot holes", "count", connector_side_hole_count, "D", connector_side_hole_D, "Z", connector_side_hole_Z, "len", connector_side_hole_len);
