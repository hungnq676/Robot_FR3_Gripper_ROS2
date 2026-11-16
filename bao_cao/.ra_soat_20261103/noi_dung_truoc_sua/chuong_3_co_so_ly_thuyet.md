# CHƯƠNG 3. CƠ SỞ LÝ THUYẾT

## 3.1 Biểu diễn tư thế và phép biến đổi đồng nhất

Vị trí và hướng của một vật rắn trong không gian ba chiều được gọi chung là tư thế (pose). Trong đề tài, mọi tư thế được biểu diễn bằng ma trận biến đổi đồng nhất 4 × 4:

$$ T = [ R  p ; 0  1 ],  R ∈ SO(3),  p ∈ ℝ³ $$ (3.1)

trong đó R là ma trận quay 3 × 3 trực chuẩn (định thức bằng 1) và p là véc-tơ tịnh tiến. Ký hiệu ᴬTᴮ là tư thế của hệ B nhìn từ hệ A. Một điểm có toạ độ xᴮ trong hệ B có toạ độ trong hệ A là:

$$ xᴬ = ᴬRᴮ · xᴮ + ᴬpᴮ $$ (3.2)

Phép biến đổi có tính bắc cầu: ᴬTᶜ = ᴬTᴮ · ᴮTᶜ. Đây là cơ sở của cây TF trong ROS: chỉ cần biết biến đổi giữa từng cặp hệ liền kề, TF tự nhân chuỗi để ra biến đổi giữa hai hệ bất kỳ.

Hướng có thể được biểu diễn bằng nhiều cách tương đương: ma trận quay, góc Euler (roll – pitch – yaw), quaternion đơn vị q = (x, y, z, w), hoặc véc-tơ trục – góc (Rodrigues). ROS dùng quaternion trong các thông điệp vì không bị suy biến (gimbal lock) và dễ nội suy. Khi giải tối ưu, đề tài dùng véc-tơ Rodrigues vì nó chỉ có 3 tham số tự do.

## 3.2 Động học tay máy 6 bậc tự do

### 3.2.1 Động học thuận

Tay máy FR3 gồm 6 khớp quay j1…j6 nối 7 khâu từ đế (`base_link`) tới mặt bích (`wrist3_link`). Mỗi khớp quay quanh một trục cố định trong hệ của khâu trước nó. Động học thuận (FK) tính tư thế mặt bích từ 6 góc khớp q = (q₁, …, q₆):

$$ ᵇTₑ(q) = ᵇT₁(q₁) · ¹T₂(q₂) · … · ⁵T₆(q₆) · ⁶Tₑ $$ (3.3)

trong đó ⁶Tₑ là biến đổi cố định từ mặt bích tới dụng cụ, ở đây là điểm làm việc của gripper (TCP, khung `grasp_frame`). Trong ROS, các biến đổi này được mô tả trong file URDF và được `robot_state_publisher` tính liên tục từ `/joint_states`.

### 3.2.2 Động học ngược

Động học ngược (IK) là bài toán ngược: cho tư thế mong muốn của TCP, tìm các góc khớp. Với tay máy 6 bậc tự do, bài toán thường có nhiều nghiệm (tối đa 8 nghiệm cho cấu trúc cổ tay cầu), tương ứng các cấu hình “khuỷu lên/khuỷu xuống”, “vai trái/vai phải”, “cổ tay lật/không lật”. Đề tài dùng bộ giải KDL của MoveIt, một bộ giải số lặp Newton–Raphson bắt đầu từ một nghiệm khởi tạo. Vì là bộ giải số, nghiệm tìm được phụ thuộc vào điểm khởi tạo. MTC gọi bộ giải nhiều lần với các điểm khởi tạo ngẫu nhiên và giữ lại tối đa 16 nghiệm khác nhau (cách nhau ít nhất 0,3 rad).

### 3.2.3 Giới hạn khớp và không gian làm việc

Mỗi khớp của FR3 có giới hạn góc. Riêng khớp đế j1 chỉ quay trong khoảng [−175°; +175°], tức là không quay được trọn vòng. Hệ quả quan trọng mà đề tài gặp phải trong thực nghiệm: với một điểm nằm ở hướng gần ±180° nhìn từ đế robot, cấu hình “bình thường” cần j1 vượt quá 175°, nên robot chỉ có thể với tới bằng cấu hình “lật tay”, tức j1 quay sang phía bên kia khoảng 160°. Vùng này được phân tích kỹ ở mục 5.6.

Tầm với của FR3 khoảng 622 mm tính từ trục j1. Gần biên tầm với, số nghiệm IK giảm và các đường đi thẳng Cartesian thường không thực hiện được trọn vẹn.

## 3.3 Mô hình camera và ảnh độ sâu

### 3.3.1 Mô hình camera lỗ kim

Camera màu được mô hình hoá bằng mô hình lỗ kim (pinhole). Một điểm P = (X, Y, Z) trong hệ quang học của camera (trục z hướng ra phía trước, x sang phải, y xuống dưới) được chiếu lên ảnh tại pixel (u, v):

$$ u = f_x · X / Z + c_x,   v = f_y · Y / Z + c_y $$ (3.4)

trong đó f_x, f_y là tiêu cự tính bằng pixel và (c_x, c_y) là tâm ảnh. Bốn tham số này tạo thành ma trận nội K. Với camera Astra Pro Plus ở độ phân giải 640 × 480, f_x = f_y ≈ 544,2 px và (c_x, c_y) ≈ (315,3; 233,9).

Ngược lại, nếu biết pixel (u, v) và độ sâu Z của nó, ta khôi phục được điểm 3D:

$$ X = (u − c_x) · Z / f_x,   Y = (v − c_y) · Z / f_y $$ (3.5)

Công thức (3.5) được dùng để chuyển toàn bộ ảnh độ sâu thành đám điểm (point cloud).

### 3.3.2 Camera độ sâu và sai số

Camera Astra dùng ánh sáng cấu trúc hồng ngoại: một máy chiếu phát mẫu chấm hồng ngoại lên cảnh, camera hồng ngoại quan sát độ lệch của mẫu chấm và tính độ sâu bằng tam giác lượng. Camera OAK-D Pro W dùng hai camera đen trắng (stereo) và tính độ sâu từ độ chênh lệch (disparity) giữa hai ảnh.

Cả hai loại đều có sai số hệ thống. Với stereo, độ sâu Z = f · b / d, với b là khoảng cách hai camera và d là độ chênh lệch. Một sai lệch nhỏ cố định trong d gây ra sai số tỉ lệ với Z². Với ánh sáng cấu trúc, mặt đen nhám hấp thụ tia hồng ngoại, nên camera không đo được độ sâu trên các vật này. Đề tài đã đo được cả hai hiện tượng:

- Depth của OAK-D Pro W ở 1,05–1,37 m đo xa hơn thật 12,1 %, ở 0,69–0,84 m đo xa hơn 7,6 %.
- Depth của Astra đo xa hơn thật 2–7 % tuỳ vị trí pixel, và gần như không đo được trên khay nhựa đen (chỉ 15 % số pixel có giá trị).

### 3.3.3 Mô hình sửa sai số độ sâu theo pixel

Với Astra, đề tài quan sát thấy mặt bàn (thực tế nằm ngang) hiện ra nghiêng 4,6° trong đám điểm. Nghĩa là tỉ số giữa độ sâu đo được và độ sâu thật thay đổi tuyến tính theo vị trí trên ảnh. Đề tài mô hình hoá tỉ số này bằng một mặt phẳng theo toạ độ pixel chuẩn hoá x = (u − c_x)/f_x, y = (v − c_y)/f_y:

$$ Z_đo / z_thật = a + b·x + c·y $$ (3.6)

Để tìm a, b, c, ta cần biết độ sâu thật của nhiều pixel. Đề tài dùng chính mặt bàn làm chuẩn: biết mặt bàn nằm ở độ cao z = 12 mm trong hệ robot (đo bằng cách cho robot chạm bàn), với mỗi pixel thuộc mặt bàn ta bắn tia từ tâm camera qua pixel đó, giao với mặt phẳng z = 12 mm, và lấy khoảng cách dọc trục quang học làm z_thật. Giải bình phương tối thiểu trên khoảng 29 000 pixel cho kết quả a = 1,0170; b = −0,0734; c = −0,0297.

## 3.4 Hiệu chuẩn camera – robot (hand-eye)

### 3.4.1 Phát biểu bài toán

Để dùng thông tin từ camera cho robot, ta cần biết tư thế của camera trong hệ toạ độ robot, ký hiệu X = ᵇTᶜ. Trong cấu hình eye-to-hand (camera đứng yên, robot di chuyển), ta gắn một mốc (AprilTag) lên robot rồi cho robot đi qua nhiều tư thế. Ở mỗi tư thế i:

- Động học thuận cho biết tư thế khâu mang tag: Aᵢ = ᵇTʷ(qᵢ).
- Camera nhìn thấy tag và ước lượng tư thế của tag trong hệ camera: Bᵢ = ᶜTᵗᵢ.
- Tư thế của tag trên khâu là một hằng số chưa biết: Y = ʷTᵗ.

Đi theo hai đường khác nhau từ gốc robot tới tag, ta được phương trình:

$$ X · Bᵢ = Aᵢ · Y,   i = 1 … N $$ (3.7)

Đây là bài toán AX = YB kinh điển. Với N ≥ 3 tư thế có hướng quay đủ khác nhau, X và Y được xác định duy nhất.

![Hình 3.1. Vòng biến đổi trong hiệu chuẩn eye-to-hand: X và Y là hai ẩn cần tìm](hinh/he_toa_do.png|14)

### 3.4.2 Ước lượng tư thế tag (PnP và IPPE)

AprilTag [10] là mốc hình vuông đen trắng có mã nhận dạng. Bộ phát hiện trả về toạ độ pixel của 4 góc tag. Biết kích thước thật của tag (cạnh s), 4 góc có toạ độ (±s/2, ±s/2, 0) trong hệ của tag. Bài toán tìm tư thế tag từ 4 cặp điểm 3D – 2D là bài toán Perspective-n-Point (PnP). Với 4 điểm đồng phẳng, đề tài dùng thuật toán IPPE [11], cho nghiệm giải tích và rất nhanh.

Một hiện tượng quan trọng: khi tag nhỏ trong ảnh (20–38 px), IPPE trả về hai nghiệm có sai số chiếu gần bằng nhau, tương ứng với tag “nghiêng về phía trước” hoặc “nghiêng về phía sau”. Chọn nhầm nghiệm làm hướng sai 30–45°. Đề tài xử lý bằng hai lớp: bỏ khung hình khi nghiệm thứ hai có sai số chiếu nhỏ hơn 2 lần nghiệm tốt nhất, và trước khi giải hand-eye, loại các mẫu có góc quay lệch quá 6° so với đa số.

Kích thước tag cũng ảnh hưởng trực tiếp tới độ sâu ước lượng: nếu khai báo cạnh nhỏ hơn thật 3 %, mọi khoảng cách PnP đều ngắn đi 3 %. Đề tài xác định cạnh thật bằng cách quét kích thước và chọn giá trị cho sai số chiếu nhỏ nhất: 33,9 mm.

### 3.4.3 Lời giải và tinh chỉnh

Lời giải ban đầu được tính bằng `cv2.calibrateHandEye` với 5 phương pháp (Tsai–Lenz [8], Park–Martin [9], Horaud, Andreff, Daniilidis). Phương pháp có độ tản thấp nhất được chọn làm điểm khởi tạo. Sau đó X và Y được tinh chỉnh đồng thời bằng bình phương tối thiểu phi tuyến trên sai số chiếu của 4 góc tag:

$$ min_{X,Y} Σᵢ Σₖ ‖ π(K, X⁻¹ · Aᵢ · Y · gₖ) − ûᵢₖ ‖² $$ (3.8)

trong đó gₖ là toạ độ góc thứ k trong hệ tag, π là phép chiếu (3.4) và ûᵢₖ là toạ độ pixel phát hiện được. Cách tinh chỉnh này tối ưu trực tiếp đại lượng đo được (pixel), nên không phụ thuộc vào sai số của bước PnP.

### 3.4.4 Đánh giá

Đề tài dùng ba chỉ số để đánh giá kết quả hiệu chuẩn:

- **Sai số chiếu RMS** trên toàn bộ mẫu (px).
- **Sai số kiểm tra chéo LOO:** bỏ ra từng mẫu, giải lại với các mẫu còn lại, rồi đo sai số chiếu trên mẫu bị bỏ ra. Chỉ số này cho biết kết quả có tổng quát hoá được không.
- **Kiểm tra trực tiếp:** đặt robot ở một tư thế mới, so vị trí tag mà camera thấy với vị trí tính từ robot và kết quả hiệu chuẩn (mm).

## 3.5 Các mô hình AI sử dụng

### 3.5.1 Phát hiện vật thể: từ YOLO tới mô hình từ vựng mở

**YOLO** (You Only Look Once) [24] là họ mô hình phát hiện vật thể một giai đoạn. Ảnh được chia thành lưới; tại mỗi ô, mạng tích chập dự đoán đồng thời toạ độ khung bao, độ tin cậy và xác suất của từng lớp vật. Vì chỉ cần một lần lan truyền qua mạng, YOLO chạy được thời gian thực (vài chục tới vài trăm khung hình mỗi giây) và được dùng rất rộng rãi trong công nghiệp. Tuy nhiên, YOLO là mô hình **tập lớp đóng**: nó chỉ nhận ra các lớp vật có trong dữ liệu huấn luyện (ví dụ 80 lớp của bộ COCO). Muốn robot gắp một vật mới, phải chụp hàng trăm ảnh vật đó, gán nhãn khung bao và huấn luyện lại.

**Mô hình từ vựng mở** (open-vocabulary) như Gemini Robotics-ER được huấn luyện trên dữ liệu ảnh – chữ quy mô rất lớn. Mô hình nhận vào ảnh và một câu mô tả bằng ngôn ngữ tự nhiên, và tìm vùng ảnh khớp với câu mô tả. Vì vậy nó tìm được những vật chưa từng được dạy riêng, và hiểu được mô tả theo thuộc tính (“cái màu trắng”, “hộp nhựa vàng”).

TABLE Bảng 3.1. So sánh YOLO và mô hình từ vựng mở cho bài toán của đề tài
| Tiêu chí | YOLO | Gemini Robotics-ER + MobileSAM |
| --- | --- | --- |
| Tập vật nhận diện | Cố định, theo dữ liệu huấn luyện | Bất kỳ, theo câu lệnh |
| Thêm vật mới | Thu ảnh, gán nhãn, huấn luyện lại | Không cần, chỉ đổi câu lệnh |
| Đầu vào lệnh | Tên lớp | Câu tự nhiên tiếng Việt hoặc tiếng Anh |
| Đầu ra | Khung bao | Khung bao, sau đó MobileSAM cho đường viền chính xác |
| Tốc độ | Rất nhanh, chạy cục bộ | 4–8 s mỗi lần (gọi qua mạng) |
| Phụ thuộc | Không | Cần Internet; bản miễn phí giới hạn 20 lần mỗi ngày |

Với bài toán của đề tài (vật trên bàn thay đổi thường xuyên, robot đứng yên khi nhận diện), tốc độ vài giây là chấp nhận được, còn khả năng gắp vật bất kỳ theo lệnh là ưu điểm quyết định. Vì vậy nhóm chọn hướng từ vựng mở thay cho YOLO như dự kiến ban đầu trong đề cương. YOLO vẫn là lựa chọn tốt nếu bài toán chỉ có vài loại vật cố định và cần tốc độ cao, ví dụ dây chuyền phân loại.

### 3.5.2 Gemini Robotics-ER: mô hình thị giác – ngôn ngữ

**Mô hình thị giác – ngôn ngữ** (Vision-Language Model, VLM) là mô hình nhận vào cả ảnh và chữ, và sinh ra chữ. Kiến trúc chung gồm ba phần:

1. **Bộ mã hoá ảnh** dạng Vision Transformer (ViT) [26]: ảnh được cắt thành các ô vuông nhỏ (patch), mỗi ô được chiếu thành một vectơ đặc trưng (token ảnh), cộng thêm mã hoá vị trí để mô hình biết ô đó nằm ở đâu trong ảnh.
2. **Lớp chiếu** đưa token ảnh về cùng không gian với token chữ, để hai loại token được ghép thành một chuỗi duy nhất.
3. **Mô hình ngôn ngữ lớn** dạng Transformer giải mã: đọc chuỗi token (ảnh + câu lệnh) và sinh từng token đầu ra, mỗi token mới dựa trên toàn bộ các token trước đó.

Cốt lõi của Transformer là cơ chế **tự chú ý** (self-attention). Với ma trận truy vấn Q, khoá K và giá trị V được chiếu tuyến tính từ chuỗi token đầu vào:

$$ Attention(Q, K, V) = softmax( Q·Kᵀ / √d_k ) · V $$ (3.9)

trong đó d_k là số chiều của khoá. Mỗi token tính trọng số chú ý tới mọi token khác, kể cả giữa token chữ và token ảnh. Nhờ đó, cụm từ “cái màu trắng” trong câu lệnh được nối với các ô ảnh có màu trắng.

**Gemini Robotics-ER** (Embodied Reasoning) [4] là phiên bản Gemini được huấn luyện thêm cho **suy luận không gian** phục vụ robot: chỉ điểm, khoanh vùng vật, đếm, ước lượng quỹ đạo, xác định điểm có thể cầm nắm. Điểm khác biệt so với VLM thông thường là mô hình trả lời được bằng **toạ độ** trên ảnh. Toạ độ được sinh ra dưới dạng chữ số, nên không cần thêm đầu ra hồi quy riêng. Đây là ý tưởng “định vị bằng ngôn ngữ” (language grounding).

Đề tài gửi cho mô hình một ảnh màu và một câu nhắc yêu cầu trả về JSON gồm nhãn, khung bao `box_2d` và lý do chọn. Khung bao theo quy ước [y_min, x_min, y_max, x_max], chuẩn hoá về khoảng 0–1000 bất kể kích thước ảnh. Đổi sang toạ độ pixel của ảnh W × H:

$$ u = x / 1000 · W,   v = y / 1000 · H $$ (3.10)

Trong mô phỏng, đề tài dùng thêm cách **đánh dấu tập hợp** (set-of-mark prompting): các vật do bộ nhận diện hình học tìm được được đánh số trực tiếp lên ảnh, và mô hình chỉ cần chọn số. Cách này giảm sai số toạ độ, vì mô hình chọn trong một tập rời rạc thay vì phải tự sinh toạ độ.

### 3.5.3 Segment Anything và MobileSAM

**Bài toán phân vùng theo gợi ý.** Cho ảnh I và một gợi ý P (một điểm, một khung bao hoặc một mặt nạ thô), cần tìm mặt nạ nhị phân M của vật mà gợi ý chỉ tới. Segment Anything (SAM) [2] giải bài toán này cho **mọi loại vật**, kể cả vật chưa từng thấy khi huấn luyện.

**Kiến trúc SAM** gồm ba khối:

1. **Bộ mã hoá ảnh:** một ViT-H (632 triệu tham số) được huấn luyện trước bằng phương pháp tự giám sát MAE [27] (che ngẫu nhiên 75 % ô ảnh và học cách khôi phục). Ảnh 1024 × 1024 được mã hoá thành bản đồ đặc trưng 64 × 64 × 256. Đây là phần nặng nhất, nhưng chỉ cần chạy **một lần** cho mỗi ảnh.
2. **Bộ mã hoá gợi ý:** điểm và khung bao được biểu diễn bằng mã hoá vị trí cộng với một vectơ học được cho từng loại gợi ý (điểm thuộc vật, điểm nền, góc trên trái, góc dưới phải của khung). Mặt nạ thô được mã hoá bằng các lớp tích chập.
3. **Bộ giải mã mặt nạ:** một Transformer hai chiều nhẹ. Token gợi ý chú ý tới đặc trưng ảnh, và đặc trưng ảnh chú ý ngược lại token gợi ý. Sau đó đặc trưng được phóng to và nhân với token đầu ra để tạo mặt nạ. Khi gợi ý không rõ ràng (một điểm trên áo có thể là cái áo hay cả người), SAM trả về ba mặt nạ ứng với ba mức chi tiết, kèm điểm IoU dự đoán cho từng mặt nạ.

Khối giải mã chỉ mất khoảng 50 ms, nên với cùng một ảnh có thể thử nhiều gợi ý rất nhanh.

**Huấn luyện.** SAM được huấn luyện trên bộ dữ liệu SA-1B gồm 11 triệu ảnh và 1,1 tỷ mặt nạ. Bộ dữ liệu được xây bằng một “máy dữ liệu” ba giai đoạn: người gán nhãn có mô hình hỗ trợ, rồi bán tự động, rồi tự động hoàn toàn. Hàm mất mát là tổ hợp tuyến tính của focal loss và dice loss theo tỉ lệ 20 : 1:

$$ L_mask = 20 · L_focal + L_dice,   L_dice = 1 − 2·|M ∩ M*| / (|M| + |M*|) $$ (3.11)

trong đó M là mặt nạ dự đoán và M* là mặt nạ đúng. Focal loss tập trung vào các pixel khó (gần biên vật). Dice loss đo độ trùng khớp của cả mặt nạ, không bị lệch khi vật chỉ chiếm phần nhỏ của ảnh.

**MobileSAM** [3]. Bộ mã hoá ViT-H quá nặng cho GPU 6 GB của đề tài. MobileSAM thay nó bằng TinyViT [16] chỉ 5 triệu tham số, huấn luyện bằng **chưng cất tách rời** (decoupled distillation): bộ mã hoá nhỏ được dạy để sinh ra **đúng bản đồ đặc trưng** của bộ mã hoá lớn, với hàm mất mát sai số bình phương:

$$ L_distill = ‖ E_student(I) − E_teacher(I) ‖² $$ (3.12)

Vì đặc trưng giống với bản gốc, bộ giải mã của SAM được giữ nguyên, không cần huấn luyện lại. Kết quả là chất lượng mặt nạ gần với SAM nhưng nhanh hơn khoảng 60 lần.

**Cách dùng trong đề tài.** Khung bao từ Gemini được đưa vào MobileSAM làm gợi ý; mặt nạ có điểm IoU dự đoán cao nhất được giữ lại. Khung bao chỉ cho biết vật nằm trong vùng nào. Mặt nạ cho biết chính xác những pixel nào thuộc vật, nhờ đó tách được điểm của vật khỏi mặt bàn và vật bên cạnh. Trên GPU GTX 1660 Ti, MobileSAM mất khoảng 2 s mỗi ảnh, chủ yếu cho lần nạp mô hình.

### 3.5.4 GraspNet

**Biểu diễn tư thế gắp.** Với gripper hai má song song, một tư thế gắp 6 bậc tự do được biểu diễn bởi:

$$ G = [ R  t  w  d ],   R ∈ SO(3),  t ∈ ℝ³ $$ (3.13)

trong đó t là tâm gắp; R = [x y z] với x là hướng tiếp cận (gripper tiến vào theo x) và y là hướng khép hai má; w là độ mở hai má; d là độ ăn sâu (đầu ngón vượt qua tâm gắp một đoạn d theo hướng tiếp cận). Mỗi tư thế còn có một điểm số chất lượng s.

**Bộ dữ liệu GraspNet-1Billion** [1] gồm 97 280 ảnh RGB-D của 190 cảnh, 88 vật thật, chụp bằng hai camera (RealSense và Kinect), với hơn 1,1 tỷ tư thế gắp được gán nhãn. Nhãn được tính bằng **phân tích lực**, không phải do người gán. Với mỗi tư thế gắp trên mô hình 3D của vật, nhóm tác giả tìm hệ số ma sát nhỏ nhất μ mà tại đó tư thế còn giữ được vật (điều kiện force-closure, mục 3.5.6). Hệ số ma sát cần càng nhỏ thì tư thế càng tốt, nên điểm số được định nghĩa:

$$ s = 1,1 − μ_min,   μ_min ∈ {0,1; 0,2; …; 1,0} $$ (3.14)

Ví dụ, một tư thế giữ được vật ngay cả khi ma sát chỉ 0,1 có điểm 1,0; tư thế cần ma sát tới 1,0 mới giữ được có điểm 0,1.

**Kiến trúc mạng** gồm bốn khối nối tiếp:

1. **Backbone PointNet++** [17]: nhận 20 000 điểm của cảnh, lấy mẫu M = 1024 **điểm hạt giống** (seed) bằng thuật toán lấy mẫu điểm xa nhất, và trích đặc trưng 256 chiều cho mỗi hạt giống từ các điểm lân cận theo nhiều tỉ lệ.
2. **ApproachNet** (mạng hướng tiếp cận): với mỗi hạt giống, dự đoán điểm đó có gắp được không, và chọn hướng tiếp cận tốt nhất trong V = 300 hướng phân bố đều trên mặt cầu. Bài toán hồi quy hướng 3D được chuyển thành bài toán **phân loại** trên 300 lớp, dễ học hơn.
3. **Gom điểm theo hình trụ** (cylinder grouping): với mỗi hạt giống và hướng tiếp cận đã chọn, lấy các điểm nằm trong một hình trụ dọc theo hướng tiếp cận. Đây chính là vùng không gian mà gripper sẽ chiếm khi gắp.
4. **OperationNet** (mạng thao tác) và **ToleranceNet** (mạng dung sai): từ các điểm trong hình trụ, chọn góc xoay quanh hướng tiếp cận trong 12 góc rời rạc, và với mỗi độ sâu trong 4 mức (10, 20, 30, 40 mm) dự đoán độ mở w, điểm số s và độ bền của tư thế khi bị lệch nhẹ.

Hàm mất mát là tổng có trọng số của các thành phần: phân loại điểm gắp được và hướng tiếp cận, phân loại góc xoay, hồi quy độ mở và điểm số, hồi quy dung sai:

$$ L = L_obj + α·(L_view + λ₁·L_angle + λ₂·L_score + λ₃·L_width) + β·L_tol $$ (3.15)

Sau khi dự đoán, các tư thế gần nhau được gộp lại (non-maximum suppression) và sắp xếp theo điểm số. Trên GPU GTX 1660 Ti, một lần suy luận mất khoảng 0,1 s và cho 50–150 tư thế cho một cảnh.

**Vì sao cần đám điểm thay vì ảnh màu.** GraspNet làm việc trên hình học 3D. Mạng không biết vật “là gì”, chỉ biết bề mặt nằm ở đâu và có hình dạng ra sao. Vì vậy GraspNet bổ sung tốt cho Gemini và SAM: Gemini trả lời “gắp vật nào”, SAM trả lời “vật chiếm những điểm nào”, còn GraspNet trả lời “gắp vật đó như thế nào”.

### 3.5.5 Chuyển tư thế gắp của GraspNet sang TCP của gripper thật

Khung TCP của gripper trong đề tài có trục z là hướng tiếp cận và trục x là hướng khép má, nằm cách đầu ngón 30 mm về phía thân gripper. Vì vậy tư thế TCP tương ứng với tư thế gắp g là:

$$ p_TCP = t + (d + Δ − 0,030) · x,   R_TCP = [ y  z  x ] $$ (3.16)

trong đó Δ là độ ăn sâu thêm do người vận hành chọn (tham số `--deeper`, thường 15 mm). Vì hai má đối xứng, tư thế xoay 180° quanh trục tiếp cận cũng là một tư thế gắp hợp lệ. Đề tài luôn thử cả hai.

### 3.5.6 Cơ sở lý thuyết của việc lọc tư thế gắp

GraspNet được huấn luyện cho một gripper hai má **tổng quát** (mở tối đa 10 cm, ngón mỏng, không có thân). Gripper thật của đề tài khác ở nhiều điểm: độ mở chỉ 30–88 mm, chỉ 30 mm ngoài cùng của ngón dùng để kẹp, thân rộng 164 mm. Vì vậy mọi tư thế GraspNet đề xuất phải được kiểm tra lại theo hình học thật của gripper và theo cơ học của việc kẹp. Mục này trình bày các nguyên lý làm cơ sở cho các bộ lọc ở mục 4.7.2.

**Nguyên lý 1: điều kiện đóng lực (force closure).** Một cách gắp gọi là đóng lực nếu các lực tại điểm tiếp xúc có thể sinh ra **mọi** lực và mô-men bên ngoài tác dụng lên vật, tức là chống lại được trọng lực và mọi va chạm nhỏ. Với tiếp xúc có ma sát (hệ số μ), lực tại mỗi điểm tiếp xúc phải nằm trong **nón ma sát**: góc giữa lực và pháp tuyến bề mặt không vượt quá

$$ α = arctan μ $$ (3.17)

Với gripper hai má, V.-D. Nguyen [25] chứng minh rằng hai điểm tiếp xúc p₁, p₂ có pháp tuyến n₁, n₂ tạo thành một cách gắp đóng lực khi và chỉ khi đường thẳng nối hai điểm tiếp xúc nằm trong cả hai nón ma sát (điều kiện **đối đỉnh**, antipodal):

$$ ∠( p₂ − p₁ , −n₁ ) ≤ α   và   ∠( p₁ − p₂ , −n₂ ) ≤ α $$ (3.18)

Hệ quả thực tế: hai má phải kẹp vào **hai mặt gần song song và đối diện nhau** của vật, theo phương gần vuông góc với hai mặt đó. Đây chính là điều kiện mà điểm số của GraspNet đánh giá (công thức (3.14)). Tuy nhiên, điều kiện này giả định hai má **chạm được** vào vật, và đó là chỗ các bộ lọc hình học phát huy tác dụng.

**Nguyên lý 2: hai má phải thực sự ép vào vật.** Lực kẹp chỉ sinh ra khi bề dày phần vật nằm giữa hai má lớn hơn khoảng cách nhỏ nhất mà hai má khép được. Gọi e là bề dày đó, w_min là khoảng cách nhỏ nhất giữa hai má (30 mm) và δ là độ biến dạng cần có để tạo lực (vài mm). Điều kiện là:

$$ e ≥ w_min + δ $$ (3.19)

Ngược lại, độ mở mà tư thế cần phải nhỏ hơn độ mở lớn nhất của gripper, có tính thêm khoảng hở để hai má không quệt vào vật khi tiến vào:

$$ e + 2·c ≤ w_max,   c ≈ 5 mm $$ (3.20)

**Nguyên lý 3: không gian chiếm chỗ của gripper không được giao với cảnh.** Khi tiến vào và khi kẹp, gripper quét một thể tích trong không gian. Thể tích này gồm hai má, thân và phần cổ tay robot. Một tư thế chỉ thực hiện được nếu thể tích quét không chứa điểm nào của cảnh (bàn, vật khác), trừ phần vật nằm giữa hai má. Kiểm tra va chạm được làm bằng cách biểu diễn cảnh thành các ô lập phương (voxel) và kiểm tra xem có ô nào bị chiếm nằm trong hình bao của gripper hay không.

**Nguyên lý 4: tư thế phải với tới được.** Tư thế gắp tốt về mặt hình học vẫn vô dụng nếu tay máy không đặt được gripper tới đó: không có lời giải động học ngược, vượt giới hạn khớp, hoặc phải quay khớp đế quá nhiều (mục 3.2.3). Điều kiện này chỉ kiểm tra được khi lập kế hoạch chuyển động, nên là bước lọc cuối cùng.

**Nguyên lý 5: xếp hạng theo độ khó, không chỉ theo điểm số.** Điểm số của GraspNet chỉ phản ánh độ ổn định của cách gắp. Với robot thật, tư thế nghiêng nhiều, lệch xa tâm vật hoặc đòi hỏi đường đi dài làm tăng rủi ro (sai số tích luỹ theo khoảng cách, nguy cơ va chạm). Vì vậy các tư thế còn lại sau lọc được xếp lại theo một **chi phí độ khó** kết hợp nhiều yếu tố (công thức (4.3) ở mục 4.9.2).

Năm nguyên lý trên tạo thành một **phễu lọc**: từ hàng trăm tư thế do GraspNet đề xuất, mỗi tầng loại bớt những tư thế vi phạm một điều kiện, và chỉ những tư thế thoả mãn tất cả mới được đưa cho robot thực hiện.

## 3.6 Lập kế hoạch chuyển động

### 3.6.1 MoveIt 2 và kiểm tra va chạm

MoveIt 2 duy trì một **planning scene** gồm mô hình robot (lấy từ URDF), trạng thái khớp hiện tại và các vật thể va chạm trong môi trường (bàn, vật cản). Mỗi trạng thái robot đề xuất đều được kiểm tra va chạm bằng thư viện FCL giữa từng cặp khâu của robot, và giữa robot với vật thể. Các cặp khâu luôn chạm nhau (như hai khâu kề nhau) được đánh dấu bỏ qua trong ma trận ACM.

### 3.6.2 Bộ lập kế hoạch Pilz

Pilz Industrial Motion Planner sinh quỹ đạo kiểu công nghiệp: PTP (nội suy tuyến tính trong không gian khớp, tất cả các khớp bắt đầu và kết thúc cùng lúc), LIN (đường thẳng trong không gian Cartesian) và CIRC (cung tròn). Ưu điểm so với các bộ lập kế hoạch ngẫu nhiên như OMPL là quỹ đạo xác định, mượt, dễ đoán trước. Trong thực nghiệm, OMPL (RRTConnect) cho đường đi có khúc gấp làm robot khựng giữa đường, nên đề tài dùng Pilz PTP làm mặc định.

### 3.6.3 MoveIt Task Constructor

MTC [5] mô tả một tác vụ phức tạp như một cây các giai đoạn (stage). Có ba loại giai đoạn:

- **Generator** sinh trạng thái từ đầu, ví dụ `CurrentState` lấy trạng thái hiện tại, `GeneratePose` sinh tư thế đích.
- **Propagator** lan truyền trạng thái theo một hướng, ví dụ `MoveRelative` đi thẳng một đoạn theo một hướng, `MoveTo` đi tới một trạng thái khớp, `ComputeIK` biến tư thế Cartesian thành trạng thái khớp.
- **Connector** nối hai trạng thái đã có bằng một bộ lập kế hoạch, ví dụ `Connect`.

Điểm mạnh của MTC là nó lập kế hoạch cho cả cây cùng lúc, lan truyền ràng buộc cả xuôi lẫn ngược. Chẳng hạn, đoạn “tiến thẳng vào 80 mm” được tính ngược từ tư thế gắp, nên điểm chờ luôn nằm đúng trên đường tiến vào. Mỗi lời giải có một chi phí. Đề tài dùng chi phí độ dài đường đi trong không gian khớp (`PathLength`) để ưu tiên lời giải ít chuyển động.

![Hình 3.2. Cấu trúc tác vụ MTC của kịch bản gắp và bỏ vào hộp: 9 giai đoạn, sinh ra 7 đoạn quỹ đạo](hinh/mtc_task.png|13)

## 3.7 Điều khiển servo bus

Gripper của đề tài dùng một servo bus Hiwonder, giao tiếp nối tiếp bán song công qua bộ chuyển BusLinker. Mỗi gói lệnh có dạng `0x55 0x55 ID LEN CMD PARAM… CHECKSUM`. Servo có hai chế độ: chế độ vị trí (giá trị 0–1000 tương ứng góc 0–240°) và chế độ động cơ (quay liên tục với tốc độ cho trước).

Ở chế độ vị trí, servo là một vòng điều khiển vị trí: mô-men sinh ra tăng theo sai lệch giữa vị trí đích và vị trí thực. Khi hai má bị vật chặn lại, sai lệch này chính là đại lượng quyết định lực kẹp. Đặt đích ngay tại điểm chạm thì lực gần bằng 0. Đặt đích sâu hơn điểm chạm thì lực tăng, nhưng dòng điện và nhiệt độ của servo cũng tăng theo. Đây là cơ sở của thuật toán kẹp ở mục 4.5 và cũng là nguyên nhân của hiện tượng servo quá nhiệt ở mục 5.5.
