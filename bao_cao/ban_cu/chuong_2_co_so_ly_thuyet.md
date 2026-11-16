# CHƯƠNG 2. CƠ SỞ LÝ THUYẾT

## 2.1 Biểu diễn tư thế và phép biến đổi đồng nhất

Vị trí và hướng của một vật rắn trong không gian ba chiều được gọi chung là tư thế (pose). Trong đề tài, mọi tư thế được biểu diễn bằng ma trận biến đổi đồng nhất 4 × 4:

$$ T = [ R  p ; 0  1 ],  R ∈ SO(3),  p ∈ ℝ³ $$ (2.1)

trong đó R là ma trận quay 3 × 3 trực chuẩn (định thức bằng 1) và p là véc-tơ tịnh tiến. Ký hiệu ᴬTᴮ là tư thế của hệ B nhìn từ hệ A. Một điểm có toạ độ xᴮ trong hệ B có toạ độ trong hệ A là:

$$ xᴬ = ᴬRᴮ · xᴮ + ᴬpᴮ $$ (2.2)

Phép biến đổi có tính bắc cầu: ᴬTᶜ = ᴬTᴮ · ᴮTᶜ. Đây là cơ sở của cây TF trong ROS: chỉ cần biết biến đổi giữa từng cặp hệ liền kề, TF tự nhân chuỗi để ra biến đổi giữa hai hệ bất kỳ.

Hướng có thể được biểu diễn bằng nhiều cách tương đương: ma trận quay, góc Euler (roll – pitch – yaw), quaternion đơn vị q = (x, y, z, w), hoặc véc-tơ trục – góc (Rodrigues). ROS dùng quaternion trong các thông điệp vì không bị suy biến (gimbal lock) và dễ nội suy. Khi giải tối ưu, đề tài dùng véc-tơ Rodrigues vì nó chỉ có 3 tham số tự do.

## 2.2 Động học tay máy 6 bậc tự do

### 2.2.1 Động học thuận

Tay máy FR3 gồm 6 khớp quay j1…j6 nối 7 khâu từ đế (`base_link`) tới mặt bích (`wrist3_link`). Mỗi khớp quay quanh một trục cố định trong hệ của khâu trước nó. Động học thuận (FK) tính tư thế mặt bích từ 6 góc khớp q = (q₁, …, q₆):

$$ ᵇTₑ(q) = ᵇT₁(q₁) · ¹T₂(q₂) · … · ⁵T₆(q₆) · ⁶Tₑ $$ (2.3)

trong đó ⁶Tₑ là biến đổi cố định từ mặt bích tới dụng cụ, ở đây là điểm làm việc của gripper (TCP, khung `grasp_frame`). Trong ROS, các biến đổi này được mô tả trong file URDF và được `robot_state_publisher` tính liên tục từ `/joint_states`.

### 2.2.2 Động học ngược

Động học ngược (IK) là bài toán ngược: cho tư thế mong muốn của TCP, tìm các góc khớp. Với tay máy 6 bậc tự do, bài toán thường có nhiều nghiệm (tối đa 8 nghiệm cho cấu trúc cổ tay cầu), tương ứng các cấu hình “khuỷu lên/khuỷu xuống”, “vai trái/vai phải”, “cổ tay lật/không lật”. Đề tài dùng bộ giải KDL của MoveIt, một bộ giải số lặp Newton–Raphson bắt đầu từ một nghiệm khởi tạo. Vì là bộ giải số, nghiệm tìm được phụ thuộc vào điểm khởi tạo. MTC gọi bộ giải nhiều lần với các điểm khởi tạo ngẫu nhiên và giữ lại tối đa 16 nghiệm khác nhau (cách nhau ít nhất 0,3 rad).

### 2.2.3 Giới hạn khớp và không gian làm việc

Mỗi khớp của FR3 có giới hạn góc. Riêng khớp đế j1 chỉ quay trong khoảng [−175°; +175°], tức là không quay được trọn vòng. Hệ quả quan trọng mà đề tài gặp phải trong thực nghiệm: với một điểm nằm ở hướng gần ±180° nhìn từ đế robot, cấu hình “bình thường” cần j1 vượt quá 175°, nên robot chỉ có thể với tới bằng cấu hình “lật tay”, tức j1 quay sang phía bên kia khoảng 160°. Vùng này được phân tích kỹ ở mục 4.6.

Tầm với của FR3 khoảng 622 mm tính từ trục j1. Gần biên tầm với, số nghiệm IK giảm và các đường đi thẳng Cartesian thường không thực hiện được trọn vẹn.

## 2.3 Mô hình camera và ảnh độ sâu

### 2.3.1 Mô hình camera lỗ kim

Camera màu được mô hình hoá bằng mô hình lỗ kim (pinhole). Một điểm P = (X, Y, Z) trong hệ quang học của camera (trục z hướng ra phía trước, x sang phải, y xuống dưới) được chiếu lên ảnh tại pixel (u, v):

$$ u = f_x · X / Z + c_x,   v = f_y · Y / Z + c_y $$ (2.4)

trong đó f_x, f_y là tiêu cự tính bằng pixel và (c_x, c_y) là tâm ảnh. Bốn tham số này tạo thành ma trận nội K. Với camera Astra Pro Plus ở độ phân giải 640 × 480, f_x = f_y ≈ 544,2 px và (c_x, c_y) ≈ (315,3; 233,9).

Ngược lại, nếu biết pixel (u, v) và độ sâu Z của nó, ta khôi phục được điểm 3D:

$$ X = (u − c_x) · Z / f_x,   Y = (v − c_y) · Z / f_y $$ (2.5)

Công thức (2.5) được dùng để chuyển toàn bộ ảnh độ sâu thành đám điểm (point cloud).

### 2.3.2 Camera độ sâu và sai số

Camera Astra dùng ánh sáng cấu trúc hồng ngoại: một máy chiếu phát mẫu chấm hồng ngoại lên cảnh, camera hồng ngoại quan sát độ lệch của mẫu chấm và tính độ sâu bằng tam giác lượng. Camera OAK-D Pro W dùng hai camera đen trắng (stereo) và tính độ sâu từ độ chênh lệch (disparity) giữa hai ảnh.

Cả hai loại đều có sai số hệ thống. Với stereo, độ sâu Z = f · b / d, với b là khoảng cách hai camera và d là độ chênh lệch. Một sai lệch nhỏ cố định trong d gây ra sai số tỉ lệ với Z². Với ánh sáng cấu trúc, mặt đen nhám hấp thụ tia hồng ngoại, nên camera không đo được độ sâu trên các vật này. Đề tài đã đo được cả hai hiện tượng:

- Depth của OAK-D Pro W ở 1,05–1,37 m đo xa hơn thật 12,1 %, ở 0,69–0,84 m đo xa hơn 7,6 %.
- Depth của Astra đo xa hơn thật 2–7 % tuỳ vị trí pixel, và gần như không đo được trên khay nhựa đen (chỉ 15 % số pixel có giá trị).

### 2.3.3 Mô hình sửa sai số độ sâu theo pixel

Với Astra, đề tài quan sát thấy mặt bàn (thực tế nằm ngang) hiện ra nghiêng 4,6° trong đám điểm. Nghĩa là tỉ số giữa độ sâu đo được và độ sâu thật thay đổi tuyến tính theo vị trí trên ảnh. Đề tài mô hình hoá tỉ số này bằng một mặt phẳng theo toạ độ pixel chuẩn hoá x = (u − c_x)/f_x, y = (v − c_y)/f_y:

$$ Z_đo / z_thật = a + b·x + c·y $$ (2.6)

Để tìm a, b, c, ta cần biết độ sâu thật của nhiều pixel. Đề tài dùng chính mặt bàn làm chuẩn: biết mặt bàn nằm ở độ cao z = 12 mm trong hệ robot (đo bằng cách cho robot chạm bàn), với mỗi pixel thuộc mặt bàn ta bắn tia từ tâm camera qua pixel đó, giao với mặt phẳng z = 12 mm, và lấy khoảng cách dọc trục quang học làm z_thật. Giải bình phương tối thiểu trên khoảng 29 000 pixel cho kết quả a = 1,0170; b = −0,0734; c = −0,0297.

## 2.4 Hiệu chuẩn camera – robot (hand-eye)

### 2.4.1 Phát biểu bài toán

Để dùng thông tin từ camera cho robot, ta cần biết tư thế của camera trong hệ toạ độ robot, ký hiệu X = ᵇTᶜ. Trong cấu hình eye-to-hand (camera đứng yên, robot di chuyển), ta gắn một mốc (AprilTag) lên robot rồi cho robot đi qua nhiều tư thế. Ở mỗi tư thế i:

- Động học thuận cho biết tư thế khâu mang tag: Aᵢ = ᵇTʷ(qᵢ).
- Camera nhìn thấy tag và ước lượng tư thế của tag trong hệ camera: Bᵢ = ᶜTᵗᵢ.
- Tư thế của tag trên khâu là một hằng số chưa biết: Y = ʷTᵗ.

Đi theo hai đường khác nhau từ gốc robot tới tag, ta được phương trình:

$$ X · Bᵢ = Aᵢ · Y,   i = 1 … N $$ (2.7)

Đây là bài toán AX = YB kinh điển. Với N ≥ 3 tư thế có hướng quay đủ khác nhau, X và Y được xác định duy nhất.

![Hình 2.1. Vòng biến đổi trong hiệu chuẩn eye-to-hand: X và Y là hai ẩn cần tìm](hinh/he_toa_do.png|14)

### 2.4.2 Ước lượng tư thế tag (PnP và IPPE)

AprilTag [10] là mốc hình vuông đen trắng có mã nhận dạng. Bộ phát hiện trả về toạ độ pixel của 4 góc tag. Biết kích thước thật của tag (cạnh s), 4 góc có toạ độ (±s/2, ±s/2, 0) trong hệ của tag. Bài toán tìm tư thế tag từ 4 cặp điểm 3D – 2D là bài toán Perspective-n-Point (PnP). Với 4 điểm đồng phẳng, đề tài dùng thuật toán IPPE [11], cho nghiệm giải tích và rất nhanh.

Một hiện tượng quan trọng: khi tag nhỏ trong ảnh (20–38 px), IPPE trả về hai nghiệm có sai số chiếu gần bằng nhau, tương ứng với tag “nghiêng về phía trước” hoặc “nghiêng về phía sau”. Chọn nhầm nghiệm làm hướng sai 30–45°. Đề tài xử lý bằng hai lớp: bỏ khung hình khi nghiệm thứ hai có sai số chiếu nhỏ hơn 2 lần nghiệm tốt nhất, và trước khi giải hand-eye, loại các mẫu có góc quay lệch quá 6° so với đa số.

Kích thước tag cũng ảnh hưởng trực tiếp tới độ sâu ước lượng: nếu khai báo cạnh nhỏ hơn thật 3 %, mọi khoảng cách PnP đều ngắn đi 3 %. Đề tài xác định cạnh thật bằng cách quét kích thước và chọn giá trị cho sai số chiếu nhỏ nhất: 33,9 mm.

### 2.4.3 Lời giải và tinh chỉnh

Lời giải ban đầu được tính bằng `cv2.calibrateHandEye` với 5 phương pháp (Tsai–Lenz [8], Park–Martin [9], Horaud, Andreff, Daniilidis). Phương pháp có độ tản thấp nhất được chọn làm điểm khởi tạo. Sau đó X và Y được tinh chỉnh đồng thời bằng bình phương tối thiểu phi tuyến trên sai số chiếu của 4 góc tag:

$$ min_{X,Y} Σᵢ Σₖ ‖ π(K, X⁻¹ · Aᵢ · Y · gₖ) − ûᵢₖ ‖² $$ (2.8)

trong đó gₖ là toạ độ góc thứ k trong hệ tag, π là phép chiếu (2.4) và ûᵢₖ là toạ độ pixel phát hiện được. Cách tinh chỉnh này tối ưu trực tiếp đại lượng đo được (pixel), nên không phụ thuộc vào sai số của bước PnP.

### 2.4.4 Đánh giá

Đề tài dùng ba chỉ số để đánh giá kết quả hiệu chuẩn:

- **Sai số chiếu RMS** trên toàn bộ mẫu (px).
- **Sai số kiểm tra chéo LOO:** bỏ ra từng mẫu, giải lại với các mẫu còn lại, rồi đo sai số chiếu trên mẫu bị bỏ ra. Chỉ số này cho biết kết quả có tổng quát hoá được không.
- **Kiểm tra trực tiếp:** đặt robot ở một tư thế mới, so vị trí tag mà camera thấy với vị trí tính từ robot và kết quả hiệu chuẩn (mm).

## 2.5 Các mô hình AI sử dụng

### 2.5.1 Gemini Robotics-ER

Gemini Robotics-ER (Embodied Reasoning) [4] là mô hình thị giác – ngôn ngữ của Google DeepMind được huấn luyện thêm cho suy luận không gian phục vụ robot: chỉ điểm, khoanh vùng vật, ước lượng quỹ đạo. Đề tài gọi mô hình qua API với một ảnh màu và một câu nhắc (prompt) yêu cầu trả về JSON gồm nhãn, khung bao `box_2d` theo quy ước [ymin, xmin, ymax, xmax] chuẩn hoá về 0–1000, và lý do chọn. Câu lệnh có thể bằng tiếng Việt hoặc tiếng Anh.

### 2.5.2 Segment Anything và MobileSAM

Segment Anything (SAM) [2] là mô hình phân vùng được huấn luyện trên hơn 1 tỷ mặt nạ. Với một gợi ý (điểm hoặc khung bao), SAM trả về mặt nạ nhị phân của vật. SAM gồm một bộ mã hoá ảnh lớn (ViT-H, 632 triệu tham số) và một bộ giải mã gợi ý nhẹ. MobileSAM [3] thay bộ mã hoá bằng TinyViT 5 triệu tham số nhờ chưng cất tri thức, giữ được chất lượng gần với SAM nhưng nhanh hơn khoảng 60 lần. Trên GPU GTX 1660 Ti, MobileSAM mất khoảng 2 s mỗi ảnh, chủ yếu cho lần nạp mô hình.

### 2.5.3 GraspNet

GraspNet-baseline [1] nhận đám điểm của cả cảnh (20 000 điểm lấy mẫu) và dự đoán các tư thế gắp cho gripper hai ngón. Mỗi tư thế gắp g gồm:

- tâm gắp t ∈ ℝ³;
- ma trận quay R = [x y z], trong đó x là hướng tiếp cận (gripper tiến vào theo x), y là hướng khép hai má;
- độ mở w (khoảng cách hai má);
- độ sâu d: đầu ngón vượt qua tâm gắp một đoạn d theo hướng tiếp cận;
- điểm số chất lượng s.

Mạng gồm một backbone PointNet++ trích đặc trưng điểm, một mô-đun đề xuất điểm có thể gắp và hướng tiếp cận, và một mô-đun dự đoán góc quay quanh hướng tiếp cận, độ mở và điểm số. Sau khi dự đoán, các tư thế gần nhau được gộp lại (NMS) và sắp xếp theo điểm số.

### 2.5.4 Chuyển tư thế gắp của GraspNet sang TCP của gripper thật

Khung TCP của gripper trong đề tài có trục z là hướng tiếp cận và trục x là hướng khép má, nằm cách đầu ngón 30 mm về phía thân gripper. Vì vậy tư thế TCP tương ứng với tư thế gắp g là:

$$ p_TCP = t + (d + Δ − 0,030) · x,   R_TCP = [ y  z  x ] $$ (2.9)

trong đó Δ là độ ăn sâu thêm do người vận hành chọn (tham số `--deeper`, thường 15 mm). Vì hai má đối xứng, tư thế xoay 180° quanh trục tiếp cận cũng là một tư thế gắp hợp lệ. Đề tài luôn thử cả hai.

## 2.6 Lập kế hoạch chuyển động

### 2.6.1 MoveIt 2 và kiểm tra va chạm

MoveIt 2 duy trì một **planning scene** gồm mô hình robot (lấy từ URDF), trạng thái khớp hiện tại và các vật thể va chạm trong môi trường (bàn, vật cản). Mỗi trạng thái robot đề xuất đều được kiểm tra va chạm bằng thư viện FCL giữa từng cặp khâu của robot, và giữa robot với vật thể. Các cặp khâu luôn chạm nhau (như hai khâu kề nhau) được đánh dấu bỏ qua trong ma trận ACM.

### 2.6.2 Bộ lập kế hoạch Pilz

Pilz Industrial Motion Planner sinh quỹ đạo kiểu công nghiệp: PTP (nội suy tuyến tính trong không gian khớp, tất cả các khớp bắt đầu và kết thúc cùng lúc), LIN (đường thẳng trong không gian Cartesian) và CIRC (cung tròn). Ưu điểm so với các bộ lập kế hoạch ngẫu nhiên như OMPL là quỹ đạo xác định, mượt, dễ đoán trước. Trong thực nghiệm, OMPL (RRTConnect) cho đường đi có khúc gấp làm robot khựng giữa đường, nên đề tài dùng Pilz PTP làm mặc định.

### 2.6.3 MoveIt Task Constructor

MTC [5] mô tả một tác vụ phức tạp như một cây các giai đoạn (stage). Có ba loại giai đoạn:

- **Generator** sinh trạng thái từ đầu, ví dụ `CurrentState` lấy trạng thái hiện tại, `GeneratePose` sinh tư thế đích.
- **Propagator** lan truyền trạng thái theo một hướng, ví dụ `MoveRelative` đi thẳng một đoạn theo một hướng, `MoveTo` đi tới một trạng thái khớp, `ComputeIK` biến tư thế Cartesian thành trạng thái khớp.
- **Connector** nối hai trạng thái đã có bằng một bộ lập kế hoạch, ví dụ `Connect`.

Điểm mạnh của MTC là nó lập kế hoạch cho cả cây cùng lúc, lan truyền ràng buộc cả xuôi lẫn ngược. Chẳng hạn, đoạn “tiến thẳng vào 80 mm” được tính ngược từ tư thế gắp, nên điểm chờ luôn nằm đúng trên đường tiến vào. Mỗi lời giải có một chi phí. Đề tài dùng chi phí độ dài đường đi trong không gian khớp (`PathLength`) để ưu tiên lời giải ít chuyển động.

![Hình 2.2. Cấu trúc tác vụ MTC của kịch bản gắp và bỏ vào hộp: 9 giai đoạn, sinh ra 7 đoạn quỹ đạo](hinh/mtc_task.png|13)

## 2.7 Điều khiển servo bus

Gripper của đề tài dùng một servo bus Hiwonder, giao tiếp nối tiếp bán song công qua bộ chuyển BusLinker. Mỗi gói lệnh có dạng `0x55 0x55 ID LEN CMD PARAM… CHECKSUM`. Servo có hai chế độ: chế độ vị trí (giá trị 0–1000 tương ứng góc 0–240°) và chế độ động cơ (quay liên tục với tốc độ cho trước).

Ở chế độ vị trí, servo là một vòng điều khiển vị trí: mô-men sinh ra tăng theo sai lệch giữa vị trí đích và vị trí thực. Khi hai má bị vật chặn lại, sai lệch này chính là đại lượng quyết định lực kẹp. Đặt đích ngay tại điểm chạm thì lực gần bằng 0. Đặt đích sâu hơn điểm chạm thì lực tăng, nhưng dòng điện và nhiệt độ của servo cũng tăng theo. Đây là cơ sở của thuật toán kẹp ở mục 3.6 và cũng là nguyên nhân của hiện tượng servo quá nhiệt ở mục 4.5.
