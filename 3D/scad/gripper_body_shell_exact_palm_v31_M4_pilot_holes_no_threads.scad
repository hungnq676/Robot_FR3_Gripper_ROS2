// ============================================================
//  GRIPPER BODY SHELL - v31 M4 PILOT HOLES NO THREADS
//  Don vi: mm
//
//  Ban nay sua theo yeu cau moi:
//  - Giu than H98, mat tren lay dung OUTLINE mat duoi lon nhat cua file palm STL vua gui.
//  - 3 lo bat oc tren dung theo tam lo that trong palm STL.
//  - O giu dai oc/ecu nam o MAT DUOI cua lop mat tren, khong khoet tren mat tren.
//  - 3 cua so ne co cau duoc do lai tu STL: co cau rong xap xi 28mm,
//    de du in/khong chat nen khe cat dat 29mm va cat len tan mat tren.
//  - Mat tren day 3mm. Lo tron giua duoc mo rong theo vong lom tron cua palm.
//  - Duoi van la mat tron R42; ong duoi day 6mm, bo go/bac ben trong.
//  - 8 lo xung quanh giu dang lo REN M4 de lap oc 4 li.
//  - Mat co 5 lo (1 lo tam + 4 lo NEMA17) duoc DO DAC HOAN TOAN o doan Z=54..57.
//  - Chi giu lai 5 lo tron, bo het 3 lo chu nhat / khe bat ky o lop nay.
//  - Ha dinh tren khoang dong co xuong thap hon de tranh thay vet lom vuong sat duoi lop Z54..57.
//  - V31: bo mo phong ren; dung lo moi M4 D=3.3mm cho 8 lo hong + 6 lo bat nap motor duoi.
//
//  Ghi chu quan trong:
//  Outline duoc trich tu section y=90.532 cua file:
//  Gripper_Assembly - palm-1(2).STL
//  Tam toa do da dua ve trung binh 3 lo bat oc cua palm.
// ============================================================

$fn = 160;
eps = 0.05;

// ============================================================
//  THONG SO CHINH
// ============================================================
base_R = 42;                 // ban kinh mat tron duoi theo de D84
body_H = 98;                 // chieu cao tong ca khoi giu 98mm
top_cap_H = 3;               // lop mat tren day 3mm theo yeu cau moi
wall_t = 3.5;                // do day thanh vo
bottom_tube_wall_t = 6;         // v20: do day ong duoi va mat day tiep dat

// Neu in bi chat/lech do co ngot nhua, chi nen tang rat nho 0.2-0.4.
// De dung "giong y het" thi de 0.
top_fit_offset = 0;

// ============================================================
//  OUTLINE MAT TREN EXACT TU PALM STL
//  Truc X/Y trong day tuong ung voi X/Z cua STL palm, da dua tam ve 3 lo oc.
// ============================================================
top_exact_pts = [
    [3.941, -37.022],
    [3.949, -37.018],
    [8.602, -34.332],
    [8.681, -34.286],
    [15.172, -30.538],
    [16.931, -29.522],
    [17.080, -29.436],
    [19.997, -27.753],
    [21.750, -26.741],
    [21.760, -26.735],
    [23.712, -25.607],
    [23.717, -25.605],
    [29.922, -22.022],
    [29.930, -22.036],
    [31.828, -25.324],
    [31.922, -25.487],
    [31.933, -25.502],
    [31.936, -25.507],
    [32.009, -25.623],
    [32.108, -25.738],
    [32.130, -25.762],
    [32.148, -25.783],
    [32.298, -25.957],
    [32.317, -25.971],
    [32.329, -25.980],
    [32.370, -26.011],
    [32.394, -26.029],
    [32.567, -26.160],
    [32.616, -26.186],
    [32.661, -26.220],
    [32.844, -26.306],
    [32.904, -26.327],
    [32.989, -26.367],
    [33.026, -26.385],
    [33.137, -26.414],
    [33.208, -26.427],
    [33.338, -26.460],
    [33.445, -26.471],
    [33.523, -26.473],
    [33.654, -26.485],
    [33.664, -26.486],
    [33.758, -26.481],
    [33.830, -26.478],
    [33.913, -26.467],
    [33.988, -26.455],
    [34.069, -26.441],
    [34.102, -26.435],
    [34.185, -26.411],
    [34.234, -26.398],
    [34.312, -26.372],
    [34.370, -26.353],
    [34.390, -26.346],
    [34.469, -26.308],
    [34.619, -26.238],
    [34.654, -26.219],
    [36.056, -25.409],
    [37.390, -24.639],
    [37.645, -24.492],
    [38.022, -24.274],
    [39.297, -23.538],
    [39.617, -23.322],
    [39.971, -23.110],
    [40.054, -23.055],
    [40.269, -22.911],
    [40.643, -22.658],
    [41.225, -22.268],
    [41.614, -21.964],
    [41.991, -21.669],
    [42.236, -21.447],
    [42.883, -20.864],
    [43.198, -20.578],
    [43.407, -20.388],
    [43.692, -20.088],
    [44.262, -19.407],
    [44.692, -18.954],
    [44.793, -18.829],
    [45.344, -18.152],
    [45.486, -17.949],
    [45.491, -17.942],
    [45.821, -17.469],
    [46.312, -16.681],
    [46.392, -16.565],
    [46.866, -15.767],
    [46.898, -15.714],
    [46.975, -15.567],
    [47.026, -15.470],
    [47.414, -14.591],
    [47.504, -14.418],
    [47.511, -14.405],
    [47.540, -14.346],
    [47.565, -14.295],
    [47.574, -14.276],
    [47.576, -14.272],
    [47.578, -14.267],
    [47.579, -14.264],
    [47.581, -14.262],
    [48.110, -13.006],
    [48.606, -11.254],
    [48.874, -10.311],
    [49.024, -9.342],
    [49.540, -6.034],
    [49.658, -5.280],
    [49.667, -5.221],
    [49.839, -1.878],
    [49.932, -0.077],
    [49.760, 3.266],
    [49.667, 5.067],
    [49.540, 5.880],
    [49.024, 9.188],
    [48.874, 10.156],
    [48.377, 11.908],
    [48.110, 12.851],
    [47.582, 14.104],
    [47.186, 15.001],
    [47.185, 15.001],
    [47.185, 15.002],
    [47.184, 15.003],
    [47.184, 15.004],
    [47.183, 15.007],
    [47.182, 15.007],
    [47.181, 15.010],
    [47.179, 15.015],
    [47.174, 15.024],
    [47.168, 15.038],
    [47.157, 15.059],
    [47.151, 15.072],
    [47.142, 15.090],
    [47.118, 15.138],
    [47.082, 15.209],
    [47.042, 15.288],
    [47.028, 15.316],
    [46.976, 15.413],
    [46.944, 15.475],
    [46.870, 15.610],
    [46.814, 15.712],
    [46.609, 16.064],
    [46.316, 16.525],
    [45.826, 17.314],
    [45.791, 17.368],
    [45.488, 17.785],
    [45.483, 17.792],
    [45.154, 18.245],
    [44.789, 18.673],
    [44.211, 19.364],
    [43.913, 19.713],
    [43.693, 19.934],
    [43.203, 20.429],
    [43.185, 20.447],
    [42.882, 20.709],
    [42.236, 21.292],
    [41.746, 21.716],
    [41.616, 21.812],
    [41.234, 22.070],
    [40.515, 22.603],
    [40.270, 22.757],
    [40.050, 22.895],
    [39.727, 23.113],
    [39.297, 23.383],
    [38.196, 24.019],
    [38.035, 24.112],
    [37.403, 24.477],
    [35.926, 25.330],
    [35.716, 25.451],
    [35.699, 25.461],
    [35.493, 25.580],
    [35.461, 25.598],
    [35.217, 25.739],
    [35.188, 25.756],
    [34.964, 25.885],
    [34.930, 25.905],
    [34.660, 26.061],
    [34.618, 26.085],
    [34.385, 26.219],
    [34.349, 26.240],
    [34.318, 26.258],
    [34.065, 26.404],
    [34.014, 26.434],
    [33.786, 26.565],
    [33.750, 26.586],
    [33.710, 26.609],
    [33.477, 26.744],
    [33.413, 26.781],
    [32.923, 27.063],
    [32.883, 26.994],
    [29.965, 21.940],
    [29.931, 21.881],
    [29.923, 21.867],
    [29.900, 21.881],
    [25.257, 24.561],
    [25.195, 24.597],
    [18.692, 28.352],
    [16.932, 29.367],
    [16.784, 29.452],
    [13.867, 31.137],
    [12.116, 32.148],
    [12.103, 32.156],
    [10.147, 33.285],
    [3.942, 36.867],
    [6.712, 41.664],
    [6.942, 42.063],
    [6.869, 42.106],
    [6.860, 42.111],
    [6.772, 42.162],
    [6.759, 42.169],
    [6.661, 42.226],
    [6.644, 42.236],
    [6.508, 42.314],
    [6.486, 42.327],
    [6.357, 42.401],
    [6.335, 42.414],
    [6.166, 42.512],
    [6.136, 42.529],
    [5.989, 42.614],
    [5.963, 42.629],
    [5.939, 42.643],
    [5.775, 42.738],
    [5.732, 42.762],
    [3.939, 43.798],
    [2.475, 44.643],
    [2.237, 44.780],
    [1.844, 45.007],
    [0.567, 45.744],
    [0.209, 45.944],
    [0.121, 45.987],
    [-0.110, 46.101],
    [-0.867, 46.470],
    [-1.497, 46.779],
    [-1.954, 46.964],
    [-2.398, 47.143],
    [-3.222, 47.410],
    [-3.542, 47.513],
    [-3.947, 47.643],
    [-4.216, 47.729],
    [-4.618, 47.826],
    [-4.679, 47.836],
    [-5.277, 47.980],
    [-5.434, 48.005],
    [-6.259, 48.150],
    [-7.121, 48.289],
    [-7.367, 48.310],
    [-7.376, 48.311],
    [-7.951, 48.360],
    [-8.879, 48.391],
    [-9.019, 48.403],
    [-9.947, 48.414],
    [-10.009, 48.415],
    [-10.175, 48.408],
    [-10.285, 48.403],
    [-10.667, 48.362],
    [-10.858, 48.354],
    [-11.434, 48.292],
    [-11.449, 48.291],
    [-11.514, 48.287],
    [-11.572, 48.283],
    [-11.592, 48.281],
    [-11.597, 48.281],
    [-11.602, 48.280],
    [-11.605, 48.280],
    [-11.608, 48.280],
    [-12.960, 48.110],
    [-14.726, 47.664],
    [-15.676, 47.424],
    [-18.799, 46.217],
    [-20.480, 45.566],
    [-23.462, 44.044],
    [-25.068, 43.224],
    [-26.228, 42.472],
    [-29.038, 40.651],
    [-29.390, 40.422],
    [-31.997, 38.322],
    [-33.401, 37.191],
    [-34.670, 35.885],
    [-35.353, 35.181],
    [-36.174, 34.098],
    [-36.753, 33.307],
    [-36.753, 33.306],
    [-36.753, 33.305],
    [-36.754, 33.304],
    [-36.755, 33.304],
    [-36.756, 33.301],
    [-36.757, 33.300],
    [-36.759, 33.298],
    [-36.761, 33.294],
    [-36.767, 33.285],
    [-36.776, 33.272],
    [-36.789, 33.253],
    [-36.797, 33.241],
    [-36.808, 33.224],
    [-36.838, 33.180],
    [-36.881, 33.113],
    [-36.930, 33.038],
    [-36.946, 33.012],
    [-37.005, 32.919],
    [-37.042, 32.860],
    [-37.122, 32.729],
    [-37.183, 32.629],
    [-37.385, 32.275],
    [-37.638, 31.791],
    [-38.076, 30.972],
    [-38.106, 30.915],
    [-38.315, 30.444],
    [-38.319, 30.437],
    [-38.546, 29.925],
    [-38.735, 29.394],
    [-39.044, 28.549],
    [-39.197, 28.116],
    [-39.279, 27.815],
    [-39.462, 27.143],
    [-39.469, 27.119],
    [-39.544, 26.726],
    [-39.726, 25.874],
    [-39.848, 25.238],
    [-39.867, 25.077],
    [-39.969, 24.181],
    [-39.980, 23.889],
    [-40.031, 23.161],
    [-40.041, 22.903],
    [-40.049, 22.792],
    [-40.068, 22.283],
    [-40.068, 20.809],
    [-40.068, 20.356],
    [-40.068, 20.081],
    [-40.068, 18.391],
    [-40.068, 18.149],
    [-40.068, 18.128],
    [-40.068, 17.891],
    [-40.068, 17.854],
    [-40.068, 17.573],
    [-40.068, 17.539],
    [-40.068, 17.280],
    [-40.068, 17.241],
    [-40.068, 16.929],
    [-40.068, 16.881],
    [-40.068, 16.612],
    [-40.068, 16.571],
    [-40.068, 16.534],
    [-40.068, 16.243],
    [-40.068, 16.183],
    [-40.068, 15.920],
    [-40.068, 15.878],
    [-40.068, 15.832],
    [-40.068, 15.563],
    [-40.068, 15.489],
    [-40.068, 14.923],
    [-34.170, 14.923],
    [-34.068, 14.923],
    [-34.068, 14.796],
    [-34.068, 9.501],
    [-34.068, 9.426],
    [-34.068, 1.955],
    [-34.067, -0.077],
    [-34.067, -0.248],
    [-34.068, -3.616],
    [-34.068, -5.644],
    [-34.068, -5.646],
    [-34.068, -7.887],
    [-34.068, -7.912],
    [-34.068, -15.077],
    [-39.607, -15.077],
    [-40.068, -15.077],
    [-40.068, -15.161],
    [-40.068, -15.172],
    [-40.068, -15.274],
    [-40.068, -15.288],
    [-40.068, -15.402],
    [-40.068, -15.421],
    [-40.068, -15.578],
    [-40.068, -15.603],
    [-40.068, -15.752],
    [-40.068, -15.778],
    [-40.068, -15.973],
    [-40.068, -16.008],
    [-40.068, -16.178],
    [-40.068, -16.207],
    [-40.068, -16.235],
    [-40.068, -16.425],
    [-40.068, -16.474],
    [-40.068, -18.545],
    [-40.068, -20.250],
    [-40.068, -20.981],
    [-40.068, -21.167],
    [-40.068, -22.438],
    [-40.060, -22.548],
    [-40.054, -22.959],
    [-40.048, -23.058],
    [-40.030, -23.315],
    [-39.979, -24.043],
    [-39.932, -24.743],
    [-39.863, -25.231],
    [-39.797, -25.705],
    [-39.616, -26.552],
    [-39.545, -26.880],
    [-39.455, -27.297],
    [-39.396, -27.572],
    [-39.278, -27.969],
    [-38.973, -28.803],
    [-38.795, -29.402],
    [-38.738, -29.552],
    [-38.428, -30.368],
    [-38.323, -30.592],
    [-38.319, -30.600],
    [-38.075, -31.122],
    [-37.637, -31.942],
    [-37.577, -32.069],
    [-37.123, -32.879],
    [-37.093, -32.933],
    [-37.004, -33.073],
    [-36.945, -33.165],
    [-36.378, -33.941],
    [-36.273, -34.105],
    [-36.266, -34.117],
    [-36.229, -34.172],
    [-36.197, -34.220],
    [-36.185, -34.237],
    [-36.183, -34.241],
    [-36.180, -34.245],
    [-36.178, -34.248],
    [-36.176, -34.250],
    [-35.353, -35.336],
    [-34.869, -35.834],
    [-33.600, -37.140],
    [-33.401, -37.345],
    [-30.794, -39.446],
    [-29.390, -40.577],
    [-29.038, -40.805],
    [-26.228, -42.626],
    [-25.068, -43.378],
    [-22.086, -44.901],
    [-20.480, -45.721],
    [-17.358, -46.928],
    [-15.676, -47.579],
    [-13.911, -48.024],
    [-12.960, -48.265],
    [-11.611, -48.434],
    [-11.027, -48.497],
    [-10.637, -48.539],
    [-10.636, -48.539],
    [-10.635, -48.539],
    [-10.634, -48.539],
    [-10.633, -48.539],
    [-10.630, -48.539],
    [-10.629, -48.539],
    [-10.626, -48.540],
    [-10.622, -48.540],
    [-10.611, -48.541],
    [-10.596, -48.542],
    [-10.572, -48.543],
    [-10.558, -48.544],
    [-10.537, -48.546],
    [-10.484, -48.549],
    [-10.405, -48.554],
    [-10.316, -48.558],
    [-10.285, -48.560],
    [-10.175, -48.564],
    [-10.105, -48.567],
    [-9.951, -48.570],
    [-9.835, -48.573],
    [-9.428, -48.571],
    [-8.882, -48.548],
    [-7.954, -48.518],
    [-7.889, -48.515],
    [-7.376, -48.461],
    [-7.368, -48.460],
    [-6.811, -48.402],
    [-6.258, -48.299],
    [-5.433, -48.155],
    [-4.980, -48.072],
    [-4.919, -48.061],
    [-4.618, -47.981],
    [-3.945, -47.804],
    [-3.920, -47.797],
    [-3.542, -47.666],
    [-2.713, -47.398],
    [-2.101, -47.185],
    [-1.953, -47.121],
    [-1.125, -46.761],
    [-0.867, -46.625],
    [-0.636, -46.503],
    [0.117, -46.136],
    [0.567, -45.899],
    [1.668, -45.263],
    [1.828, -45.170],
    [2.462, -44.805],
    [3.796, -44.034],
    [5.209, -43.219],
    [5.244, -43.198],
    [5.379, -43.104],
    [5.451, -43.054],
    [5.467, -43.040],
    [5.513, -43.000],
    [5.575, -42.945],
    [5.610, -42.909],
    [5.673, -42.849],
    [5.694, -42.824],
    [5.746, -42.760],
    [5.795, -42.701],
    [5.846, -42.635],
    [5.884, -42.575],
    [5.935, -42.495],
    [5.940, -42.485],
    [5.978, -42.415],
    [6.032, -42.298],
    [6.076, -42.200],
    [6.082, -42.179],
    [6.107, -42.109],
    [6.136, -42.003],
    [6.149, -41.933],
    [6.156, -41.906],
    [6.179, -41.825],
    [6.182, -41.786],
    [6.190, -41.695],
    [6.207, -41.494],
    [6.208, -41.437],
    [6.202, -41.382],
    [6.175, -41.166],
    [6.169, -41.120],
    [6.163, -41.071],
    [6.160, -41.047],
    [6.084, -40.830],
    [6.075, -40.798],
    [6.066, -40.773],
    [6.016, -40.630],
    [5.952, -40.509],
    [5.944, -40.492],
    [5.941, -40.487],
    [3.975, -37.081]
];

// 3 tam lo bat oc EXACT tu STL palm, da dua ve cung tam voi top_exact_pts.
// Lo trong palm do duoc xap xi D4.0, nhung than dung lo thong M3 mac dinh 3.4.
top_bolt_pts = [
    [39.973, -0.044],
    [-20.066, 34.663],
    [-19.907, -34.619]
];

// ============================================================
//  LO / BOSS BAT PALM TREN
// ============================================================
top_bolt_D = 3.4;            // lo thong cho vit M3 tu palm tren di xuong
top_boss_enable = true;
top_boss_D = 14;             // boss duoi mat tren, tang cung quanh lo
top_boss_down_H = 16;        // boss keo xuong 16mm tinh tu mat tren

// O am giu dai oc/ecu nam ben DUOI lop mat tren.
top_under_nut_enable = true;
top_under_nut_is_hex = true;
top_under_nut_D = 7.1;       // M3 nut, de rong hon de in 3D de lap
top_under_nut_H = 2.4;       // top cap chi 3mm, giam de khong an xuyen mat tren

// ============================================================
//  LO NE GIUA / NE VONG TRON VA CO CAU
// ============================================================
top_center_clear_enable = true;
top_center_clear_R = 28.8;   // do theo vong/lom tron giua STL: R~28.2, cong du 0.6mm
bottom_inner_R = base_R - bottom_tube_wall_t; // v20: day tiep dat day dung 6mm, khong tao bac/go ben trong

// ============================================================
//  BAT VAO DE DUOI
// ============================================================
bottom_flange_H = 5;
bottom_flange_enable = false; // v11: bo mat phang de duoi, chi con thanh/vien ngoai cua shell
base_bolt_enable = false;     // khong khoet 4 lo o day khi da bo mat de trong
base_bolt_R = 34.5;
base_bolt_D = 3.4;
base_bolt_angle0 = 45;

base_counterbore_enable = false;
base_counterbore_D = 6.4;
base_counterbore_H = 3.0;

// ============================================================
//  3 CUA SO NE NGON
// ============================================================
finger_cut_enable = true;

// Kich thuoc 3 o khoet tren 3 thanh/canh:
// - STL do duoc co cau rong xap xi 28mm; file nay cat 29mm de co du 0.5mm moi ben.
// - Cao phan khoet = 41mm, cat len tan mat tren theo yeu cau moi.
palm_mech_measured_width = 28.0;  // do tu 3 co cau duoi palm: rong xap xi 28mm
finger_cut_side_clearance = 0.5;  // du moi ben 0.5mm de lap/in 3D khong bi chat
finger_cut_half_width = palm_mech_measured_width/2 + finger_cut_side_clearance;
finger_cut_width = finger_cut_half_width * 2;
finger_cut_height = 41;     // cao hon ban cu 6mm: 35 + 6 = 41mm

finger_cut_inner_R = 13.5; // co cau tren STL bat dau tu R~15, cat vao trong de ne het
finger_cut_outer_R = 72;
finger_cut_round_D = finger_cut_width;

// true = giu lop mat tren khong bi cua so cat.
// false = cua so cat len tan mat tren, dung theo yeu cau moi.
finger_cut_below_top_cap_only = false;

// 3 ngon nam o trung diem 3 canh cua tam giac palm: 60, 180, 300 do.
finger_edge_angles = [60, 180, 300];


// ============================================================
//  PHAN TACH 3 NGAN + DE GIU DONG CO NEMA17
// ============================================================
// Mat phang phan tach giua ngan co cau quay (tren) va ngan dong co (giua)
// dat tai chan 3 cua so ben hong.
motor_partition_H = 3;
motor_partition_top_Z = body_H - finger_cut_height;      // 98 - 41 = 57
motor_partition_bottom_Z = motor_partition_top_Z - motor_partition_H;

// Lo tam cho truc/co cau dong co di len. User yeu cau R=11.
motor_center_hole_R = 11;

// 4 lo bat dong co theo NEMA17: 31x31, 4-M3.
motor_mount_pitch = 31;
motor_mount_hole_D = 3.4;

// Than dong co: hinh ve goc 34mm, motor thuc te cua user la 39mm.
motor_body_W = 42.3 + 0.6;      // cong du nhe de lap/in de hon
motor_body_L = 42.3 + 0.6;
motor_body_H = 39;
motor_body_bottom_Z = motor_partition_bottom_Z - motor_body_H;

// v11: do dac toan bo vung ben ngoai motor toi sat thanh vo,
// chi trong doan tu chan khoang dong co len mat bat motor.
motor_cavity_top_gap = 2.00;      // v23: ha dinh tren khoang dong co xuong 2mm duoi mat phan tach, giam vet lom vuong ngay sat lop Z54..57
partition_window_clearance_Z = 0.0; // v32: bo hoan toan cua so/khe; doan Z54..57 chi con 5 lo tron
motor_hole_cut_bottom_Z = motor_body_bottom_Z - 1;
motor_hole_cut_top_Z = motor_partition_top_Z + 1;
motor_support_fill_bottom_Z = motor_body_bottom_Z;
motor_support_fill_top_Z = motor_partition_bottom_Z;

// v17: chi do vat lieu giu motor o vung TRUNG TAM, khong do ra sat thanh ngoai
// de tranh tao go/vong trong o phan de.
motor_support_fill_W = motor_body_W + 12;
motor_support_fill_L = motor_body_L + 12;

// Bien cu giu lai de tuong thich, v12 khong dung block vuong nua.
motor_support_block_W = 56;
motor_support_block_L = 56;

// V20: thanh ong duoi day 6mm, khong them go/vong ben trong.
lower_motor_wall_t = bottom_tube_wall_t;  // thanh ong duoi day 6mm, chi lay vo ngoai
lower_motor_wall_bottom_Z = 0;
lower_motor_wall_top_Z = motor_partition_bottom_Z;

// V32: do dac toan bo doan Z54..57; khong con 3 lo chu nhat / khe tren mat ngan cach.
motor_partition_flat_repair_H = 3.0;

// V13: bo 8 lo dung truc o mat ngan; thay bang 8 lo REN M4 huong xuyen tam,
// vuong goc voi truc vat, nam o vung de duoi cung.
radial_mount_enable = true;
radial_mount_count = 8;
radial_mount_pitch_circle_R = base_R - lower_motor_wall_t/2;   // truc lo nam giua be day thanh 6mm
radial_mount_center_Z = 10;                                    // v32: nang tam 8 lo ngang len +2mm, cao hon mat day 10mm
radial_mount_thread_length = 14;                           // dao cat dai tu ngoai vao trong, dam xuyen thanh ong 6mm
radial_mount_start_angle = 0;

// Thong so ren M4x0.7 de tao lo ren trong nhua.
// V31: khong mo phong ren that trong OpenSCAD.
// Chi khoet lo moi/tap drill de sau in bat vit/taro M4.
radial_thread_nominal_D = 4.0;
radial_thread_pitch = 0.7;
radial_thread_major_D = 4.0;       // chi de echo/tham khao
radial_thread_minor_D = 3.30;      // lo moi M4/tap drill ~3.3mm
radial_thread_profile_H = 0;
radial_thread_slices_per_turn = 0;
m4_pilot_D = 3.30;

// Khe cho day / phan nhô day motor: giu theo yeu cau moi.
// rong X = 16, dai/sau Y = 10, cao Z = 9.5.
motor_tail_clear_W = 16;
motor_tail_clear_D = 10;
motor_tail_clear_H = 9.5;

// Huong phan nhô dat ve phia -Y de de thay doi neu can.
motor_tail_on_negative_Y = true;

// ============================================================
//  HIEN THI THAM CHIEU
// ============================================================
show_top_bolt_markers = false;
show_reference_top = false;

// ============================================================
//  MODULE 2D
// ============================================================
module top_shape_2d() {
    offset(delta = top_fit_offset)
        polygon(points = top_exact_pts, convexity = 10);
}

module top_inner_shape_2d() {
    offset(delta = -wall_t)
        top_shape_2d();
}

// ============================================================
//  THAN NGOAI / TRONG
// ============================================================
module morph_outer() {
    hull() {
        translate([0, 0, 0])
            linear_extrude(height = eps)
                circle(r = base_R);

        translate([0, 0, body_H - eps])
            linear_extrude(height = eps)
                top_shape_2d();
    }
}

module morph_inner() {
    // v20: phan 1 (tu day den z day motor) la ong rong co be day KHONG DOI = 6mm.
    // Khong co bac/go ben trong o gan day; tu z day motor tro len moi chuyen dan de gap phan tren.
    union() {
        translate([0, 0, -eps])
            cylinder(h = motor_body_bottom_Z + eps, r = base_R - lower_motor_wall_t);

        hull() {
            translate([0, 0, motor_body_bottom_Z])
                linear_extrude(height = eps)
                    circle(r = base_R - lower_motor_wall_t);

            translate([0, 0, body_H - top_cap_H])
                linear_extrude(height = eps)
                    top_inner_shape_2d();
        }
    }
}


// ============================================================
//  V31 - 6 LO MOI M4 BAT TAM CHE DONG CO, KHONG MO PHONG REN
// ============================================================
bottom_cover_pilot_enable = true;
bottom_cover_pilot_D = m4_pilot_D;
bottom_cover_pilot_depth = 20;
bottom_cover_pilot_start_Z = motor_body_bottom_Z; // Z=15
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

// ============================================================
//  BOSS GIU OC PALM TREN
// ============================================================
module morph_inner_with_inset(inset_v) {
    hull() {
        translate([0, 0, -eps])
            linear_extrude(height = eps*2)
                circle(r = base_R - inset_v);

        translate([0, 0, body_H - top_cap_H])
            linear_extrude(height = eps*2)
                offset(delta = -inset_v)
                    top_shape_2d();
    }
}

module lower_motor_wall_thickener() {
    // Module cu KHONG goi trong model chinh; giu lai de tham khao.
    difference() {
        intersection() {
            morph_inner();
            translate([0,0,(lower_motor_wall_bottom_Z + lower_motor_wall_top_Z)/2])
                cube([220,220, lower_motor_wall_top_Z - lower_motor_wall_bottom_Z], center=true);
        }
        intersection() {
            morph_inner_with_inset(lower_motor_wall_t);
            translate([0,0,(lower_motor_wall_bottom_Z + lower_motor_wall_top_Z)/2])
                cube([220,220, lower_motor_wall_top_Z - lower_motor_wall_bottom_Z + 0.2], center=true);
        }
    }
}

module m4_thread_tool(len = radial_mount_thread_length) {
    // V31: lo moi tron D=3.3mm, KHONG tao ren.
    cylinder(h = len, d = m4_pilot_D, $fn = 48);
}


module radial_threaded_mount_holes() {
    if (radial_mount_enable) {
        for (i = [0:radial_mount_count-1]) {
            a = radial_mount_start_angle + i*360/radial_mount_count;
            // Bat oc tu NGOAI vao TRONG:
            // dat dao cat bat dau o ngoai han vat the, roi cat xuyen qua vo 6mm vao khoang rong.
            rotate([0,0,a])
                translate([base_R + 5, 0, radial_mount_center_Z])
                    rotate([0,-90,0])
                        m4_thread_tool(radial_mount_thread_length + eps);
        }
    }
}

module bottom_motor_cover_pilot_holes() {
    if (bottom_cover_pilot_enable) {
        for (p = bottom_cover_pilot_pts) {
            translate([p[0], p[1], bottom_cover_pilot_start_Z - eps])
                cylinder(h = bottom_cover_pilot_depth + eps, d = bottom_cover_pilot_D, $fn = 48);
        }
    }
}

module top_bolt_bosses() {
    if (top_boss_enable) {
        for (p = top_bolt_pts) {
            translate([p[0], p[1], body_H - top_boss_down_H])
                cylinder(h = top_boss_down_H, d = top_boss_D);
        }
    }
}

// ============================================================
//  CUTTERS
// ============================================================
module base_bolt_holes() {
    if (base_bolt_enable) {
        for (i = [0:3]) {
            a = base_bolt_angle0 + i*90;
            x = base_bolt_R*cos(a);
            y = base_bolt_R*sin(a);

            translate([x, y, -eps])
                cylinder(h = bottom_flange_H + 2*eps, d = base_bolt_D);

            if (base_counterbore_enable) {
                translate([x, y, -eps])
                    cylinder(h = base_counterbore_H + eps, d = base_counterbore_D);
            }
        }
    }
}

module top_bolt_holes_and_under_nuts() {
    for (p = top_bolt_pts) {
        hole_depth = top_boss_enable ? top_boss_down_H : top_cap_H;

        // Lo thong: vit di tu palm tren xuong than.
        translate([p[0], p[1], body_H - hole_depth - eps])
            cylinder(h = hole_depth + 2*eps, d = top_bolt_D);

        // O am luc giac o MAT DUOI cua cap tren.
        // Khi siet vit tu tren xuong, dai oc bi keo len va khoa vao o nay.
        if (top_under_nut_enable) {
            translate([p[0], p[1], body_H - top_cap_H - eps]) {
                if (top_under_nut_is_hex)
                    rotate([0, 0, 30])
                        cylinder(h = top_under_nut_H + eps, d = top_under_nut_D, $fn = 6);
                else
                    cylinder(h = top_under_nut_H + eps, d = top_under_nut_D);
            }
        }
    }
}

module top_center_clearance() {
    if (top_center_clear_enable) {
        translate([0, 0, body_H - top_cap_H - eps])
            cylinder(h = top_cap_H + 2*eps, r = top_center_clear_R);
    }
}

module one_finger_edge_window(angle_deg) {
    // O khoet ben hong cat len tan mat tren, nhung KHONG an xuong mat
    // phan tach dong co. De lai moi lien nho tren mat ngan cach.
    top_z = finger_cut_below_top_cap_only ? (body_H - top_cap_H) : body_H;
    raw_bottom_z = top_z - finger_cut_height;
    bottom_z = max(raw_bottom_z, motor_partition_top_Z + partition_window_clearance_Z);
    cut_H = top_z - bottom_z + eps;
    cut_center_z = (top_z + bottom_z) / 2;

    cut_depth = finger_cut_outer_R - finger_cut_inner_R;
    cut_center_R = (finger_cut_outer_R + finger_cut_inner_R) / 2;

    rotate([0, 0, angle_deg]) {
        translate([cut_center_R, 0, cut_center_z])
            cube([cut_depth, finger_cut_width, cut_H], center = true);

        for (s = [-1, 1]) {
            translate([cut_center_R, s*finger_cut_width/2, cut_center_z])
                cylinder(h = cut_H, d = finger_cut_round_D, center = true);
        }
    }
}

module finger_edge_windows() {
    if (finger_cut_enable) {
        for (a = finger_edge_angles)
            one_finger_edge_window(a);
    }
}


module motor_partition_plate() {
    translate([0, 0, motor_partition_bottom_Z])
        linear_extrude(height = motor_partition_H)
            top_inner_shape_2d();
}

module motor_partition_flat_repair_skin() {
    // v32: do dac ca khoang Z=54..57; khong chua bat ky khe/chu nhat nao.
    translate([0, 0, motor_partition_bottom_Z])
        linear_extrude(height = motor_partition_H)
            top_inner_shape_2d();
}

module motor_support_fill() {
    // v19: phan 2 phai do dac het tu vo vao toi khoang dong co.
    // Vi the ta do day toan bo the tich long ben trong trong doan z nay,
    // sau do tru lai khoang dong co va khe day o phan difference cuoi.
    intersection() {
        morph_inner();
        translate([0, 0, (motor_support_fill_bottom_Z + motor_support_fill_top_Z)/2])
            cube([220, 220, motor_support_fill_top_Z - motor_support_fill_bottom_Z], center = true);
    }
}

module motor_support_block() {
    // Giu module cu de khong loi neu can goi thu cong; v11 dung motor_support_fill().
    motor_support_fill();
}

module motor_mount_holes() {
    // 4 lo bat vit phai THONG xuong khoang dong co, khong chi la lo nong tren mat ngan.
    for (sx = [-1, 1], sy = [-1, 1]) {
        translate([sx * motor_mount_pitch/2, sy * motor_mount_pitch/2, motor_hole_cut_bottom_Z])
            cylinder(h = motor_hole_cut_top_Z - motor_hole_cut_bottom_Z, d = motor_mount_hole_D);
    }
}

module motor_center_hole() {
    // Lo tam R11 thong xuong khoang dong co de dua truc quay len.
    translate([0, 0, motor_hole_cut_bottom_Z])
        cylinder(h = motor_hole_cut_top_Z - motor_hole_cut_bottom_Z, r = motor_center_hole_R);
}

module motor_body_cavity() {
    // Cat khoang than dong co nhung dung truoc mat phan tach de mat phan tach lien,
    // khong con lo chu nhat tren mat bat motor.
    cavity_H = motor_body_H;
    translate([0, 0, motor_body_bottom_Z + cavity_H/2 - motor_cavity_top_gap])
        cube([motor_body_W, motor_body_L, cavity_H], center = true);
}

module motor_tail_clearance() {
    ysign = motor_tail_on_negative_Y ? -1 : 1;
    // Khoang rong them cho phan giac cam/phan nhô cua motor.
    translate([0, ysign*(motor_body_L/2 + motor_tail_clear_D/2), motor_body_bottom_Z + motor_tail_clear_H/2])
        cube([motor_tail_clear_W, motor_tail_clear_D, motor_tail_clear_H + eps], center = true);
}


// ============================================================
//  MODEL CHINH
// ============================================================
module gripper_body_shell_v31() {
    difference() {
        union() {
            // Than chinh: duoi tron -> tren EXACT palm outline
            difference() {
                morph_outer();
                morph_inner();
            }

            // Vanh/thanh ngoai duoi: v11 bo mat phang de, chi giu thanh shell ngoai.
            if (bottom_flange_enable) {
                linear_extrude(height = bottom_flange_H)
                    difference() {
                        circle(r = base_R);
                        circle(r = bottom_inner_R);
                    }
            }

            // Boss phia duoi 3 lo bat oc palm
            top_bolt_bosses();

            // Mat phang phan tach ngan 1 (co cau quay) va ngan 2 (dong co)
            motor_partition_plate();
            motor_partition_flat_repair_skin();

            // Do dac toan bo phan 2 den sat vo ngoai; khoang motor se duoc cat lai sau.
            motor_support_fill();

            // v16: khong them go/vong ben trong nua; thanh duoi duoc tao truc tiep boi morph_inner 6mm.
        }

        // v20: Khoet day voi cung ban kinh trong R=base_R-6.
        // Khong mo rong R o 5mm dau nua, nen khong con bac/go ben trong.
        translate([0, 0, -eps])
            cylinder(h = bottom_flange_H + 2*eps, r = bottom_inner_R);

        // Lo ne trung tam / co cau tren mat tren
        top_center_clearance();

        // Lo bat de va palm
        base_bolt_holes();
        top_bolt_holes_and_under_nuts();

        // Cua so ne ngon o trung diem cac canh
        finger_edge_windows();

        // Lo tam cho truc dong co va 4 lo bat vit NEMA17 tren mat phang phan tach
        motor_center_hole();
        motor_mount_holes();

        // V31: 8 lo moi M4 bat tu ngoai vao trong, KHONG mo phong ren
        radial_threaded_mount_holes();

        // V31: 6 lo moi M4 bat tam che dong co tu mat Z=15, sau len 20mm
        bottom_motor_cover_pilot_holes();

        // O am giu than dong co, tao ngan 2 o giua
        motor_body_cavity();
        motor_tail_clearance();
    }
}

// reference_parts(); // disabled: original file has no module definition
gripper_body_shell_v31();

echo("v31: M4 pilot holes only, no modeled threads; Z54..57 solid except 5 round holes; M4 threaded 8 side holes; bottom tube 6mm wall; no inner ledge");
echo("body_H", body_H, "top_cap_H", top_cap_H);
echo("top bolt pts", top_bolt_pts);
echo("finger windows total width", finger_cut_width, "height", finger_cut_height);
echo("motor partition Z top", motor_partition_top_Z, "bottom", motor_partition_bottom_Z, "thickness", motor_partition_H);
echo("motor center hole R", motor_center_hole_R);
echo("motor mount pitch", motor_mount_pitch, "hole D", motor_mount_hole_D);
echo("motor holes through Z", motor_hole_cut_bottom_Z, "to", motor_hole_cut_top_Z);
echo("motor body cavity", [motor_body_W, motor_body_L, motor_body_H], "motor body bottom Z", motor_body_bottom_Z, "top gap", motor_cavity_top_gap);
echo("motor support fill", "full slab to shell", true, "Z", [motor_support_fill_bottom_Z, motor_support_fill_top_Z]);
echo("motor tail/wire clearance", [motor_tail_clear_W, motor_tail_clear_D, motor_tail_clear_H]);
echo("bottom open inner R", bottom_inner_R, "outer R", base_R);
echo("finger windows stop at Z", max(body_H - finger_cut_height, motor_partition_top_Z + partition_window_clearance_Z));
echo("part1 lower tube wall thickness", lower_motor_wall_t, "no inner ledge, constant inner R", base_R - lower_motor_wall_t, "up to Z", motor_body_bottom_Z);
echo("radial M4 threaded holes", radial_mount_count, "outside-in", true, "through outer shell", true, "tube wall", lower_motor_wall_t, "center Z", radial_mount_center_Z, "cutter length", radial_mount_thread_length);
echo("M4 pilot only", "pilot D", m4_pilot_D, "no modeled thread", true);

// v23 notes: phan 1 la ong rong day 6mm, bo bac/go ben trong; 8 lo xung quanh la lo ren M4; dai Z=54..57 duoc do dac hoan toan, chi tru 5 lo tron (1 lo tam + 4 lo bat motor); ha dinh tren khoang dong co xuong 2mm de bot thay vet lom vuong sat ben duoi lop nay; 3 lo chu nhat o mep tren lop mat nay da bi bo; phan 3 giu nguyen.

echo("bottom cover 6x M4 pilot holes", bottom_cover_pilot_pts, "D", bottom_cover_pilot_D, "Z start", bottom_cover_pilot_start_Z, "depth", bottom_cover_pilot_depth);
