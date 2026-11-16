# CHƯƠNG 3. THIẾT KẾ VÀ XÂY DỰNG HỆ THỐNG

## 3.1 Yêu cầu đối với hệ thống

Từ mục tiêu ở phần Mở đầu, hệ thống cần thoả mãn các yêu cầu sau:

TABLE Bảng 3.1. Yêu cầu chức năng và phi chức năng
| Mã | Yêu cầu | Chỉ tiêu |
| --- | --- | --- |
| YC1 | Nhận lệnh bằng ngôn ngữ tự nhiên | Câu lệnh tiếng Việt hoặc tiếng Anh, chọn đúng vật trên ảnh |
| YC2 | Định vị vật trong hệ toạ độ robot | Sai số ngang dưới 10 mm |
| YC3 | Đề xuất cách gắp phù hợp với gripper thật | Độ mở 40–85 mm, ngón ăn sâu tối đa 30 mm, phần kẹp dày trên 33 mm |
| YC4 | Lập kế hoạch tránh va chạm | Tránh bàn, vật khác và chính robot; xem trước trong RViz |
| YC5 | Điều khiển gripper | Mở, kẹp theo bề dày đo được, nhả vật |
| YC6 | Kịch bản trọn vẹn | HOME → gắp → bỏ vào hộp → HOME, chỉ hỏi xác nhận một lần |
| YC7 | An toàn | Không chuyển động khi chưa xác nhận; tự dừng khi lệch quá 10 mm |
| YC8 | Ghi dữ liệu | Ảnh trước/sau, video camera, log kế hoạch, point cloud |

## 3.2 Phần cứng

### 3.2.1 Tay máy FAIRINO FR3

FR3 là tay máy cộng tác 6 bậc tự do của hãng FAIRINO, tải trọng 3 kg, tầm với 622 mm, độ lặp lại ±0,02 mm. Bộ điều khiển kết nối với máy tính qua Ethernet (địa chỉ 192.168.58.2) và nhận lệnh vị trí khớp qua hàm `ServoJ` của SDK `libfairino`. Mỗi lệnh `ServoJ` được bộ điều khiển thực hiện trong 8 ms.

Để an toàn trong quá trình nghiên cứu, tốc độ khớp trong MoveIt được giới hạn ở 0,35 rad/s, khoảng 11 % tốc độ tối đa của FR3 (3,15 rad/s). Ngưỡng ngắt của controller (state tolerance) là 0,05 rad mỗi khớp. Mặt bàn trong planning scene được mô hình hoá bằng một tấm 1,6 × 1,6 × 0,05 m có mặt trên ở độ cao 12 mm so với gốc robot, khớp với độ cao thật đo được bằng cách cho robot chạm bàn.

### 3.2.2 Camera

Trong giai đoạn đầu, đề tài dùng camera Luxonis OAK-D Pro W (stereo, góc rộng 95° × 72°, ảnh màu 1280 × 720). Ngày 30/10/2026, cảm biến ảnh màu IMX378 của OAK bị hỏng phần cứng: mọi luồng có camera màu đều khiến thiết bị khởi động lại sau khoảng 0,2 s, dù camera đen trắng vẫn chạy tốt. Đề tài chuyển sang camera Orbbec Astra Pro Plus, loại dùng ánh sáng cấu trúc hồng ngoại.

TABLE Bảng 3.2. So sánh hai camera đã dùng
| Thông số | OAK-D Pro W | Astra Pro Plus |
| --- | --- | --- |
| Nguyên lý đo sâu | Stereo + chấm hồng ngoại | Ánh sáng cấu trúc hồng ngoại |
| Ảnh màu dùng trong đề tài | 1280 × 720, f_x ≈ 759 px | 640 × 480, f_x ≈ 544 px |
| Tầm đo gần nhất | khoảng 0,2 m | khoảng 0,53 m |
| Nhiễu mặt bàn ở 0,67–0,7 m | độ lệch chuẩn 1,6–1,8 mm | độ lệch chuẩn khoảng 2,9 mm sau hiệu chỉnh |
| Sai số hệ thống | đo xa hơn thật 7–12 %, tăng theo khoảng cách | đo xa hơn thật 0–6 %, thay đổi theo vị trí pixel |
| Điểm yếu | hỏng cảm biến màu ngày 30/10 | không đo được vật đen nhám |

Camera Astra được đặt cố định trên giá, cách mặt bàn khoảng 0,7 m, nhìn gần như thẳng xuống (nghiêng khoảng 9°). Vị trí này được chọn sau nhiều lần thử: đặt thấp 0,63 m thì mặt trên của vật nằm trong vùng camera không đo được (dưới 0,53 m), đặt nghiêng nhiều thì ảnh bị méo ở mép và tag nhỏ.

![Hình 3.1. Camera đặt phía trên bàn, nhìn thẳng xuống; trên bàn là các vật thử nghiệm](hinh/p_cam_gan.jpg|13)

### 3.2.3 Gripper hai ngón tự thiết kế

Gripper do nhóm thiết kế và in 3D, gồm một thân chứa servo, cơ cấu truyền động và hai ngón song song. Gripper được gắn lên mặt bích của FR3 qua một vành chuyển tiếp dày 28 mm. Kích thước chính được đo trực tiếp:

- Từ mặt bích tới đầu ngón: 165 mm. Ban đầu đo bằng thước được 137 mm do đo từ vành chuyển tiếp. Con số chính xác có được khi cho robot hạ dần tới lúc đầu ngón vừa chạm mặt bàn rồi đọc toạ độ mặt bích.
- Thân gripper: 164 × 85 × 73 mm.
- Ngón dài 60 mm, rộng 38 mm. Chỉ 30 mm ngoài cùng của ngón được dùng để kẹp vì kẹp sâu hơn thì khó gắp.
- Hai má mở tối đa 88 mm. Servo hết hành trình khi hai má còn cách nhau khoảng 30 mm.

Gripper được lắp xoay −90° quanh trục mặt bích so với mô hình trong mô phỏng, sao cho khi mặt bích úp xuống thì hai má nằm song song với mặt bàn và camera ESP32 trên thân gripper hướng lên trên.

![Hình 3.2. Kích thước chính của gripper và vị trí điểm TCP (grasp_frame), cách đầu ngón 30 mm](hinh/gripper_hinh_hoc.png|12)

### 3.2.4 Mạch điều khiển gripper

Servo Hiwonder (ID 1) được nối với bộ chuyển BusLinker V3. BusLinker được điều khiển bởi một ESP32-CAM qua UART0 (chân U0T – GPIO1 nối RX của BusLinker, U0R – GPIO3 nối TX, chung GND, tốc độ 115200 baud). ESP32 kết nối Wi-Fi và chạy một máy chủ HTTP nhỏ với các điểm truy cập:

- `GET /servo/status`: trả về vị trí, điện áp, nhiệt độ, trạng thái giữ lực của servo dưới dạng JSON.
- `POST /servo/packet`: gửi một gói lệnh servo, có xác thực mật khẩu. Firmware chỉ cho phép các lệnh an toàn (di chuyển, dừng, bật/tắt giữ lực, chế độ động cơ). Lệnh di chuyển bị từ chối khi servo từ 60 °C trở lên.
- Cập nhật firmware từ xa (OTA) có mật khẩu.

Firmware còn có một cơ chế an toàn: ở chế độ động cơ, nếu không nhận được lệnh mới trong khoảng 1 s thì servo tự dừng.

## 3.3 Kiến trúc phần mềm

Toàn bộ phần mềm chạy trên một máy tính xách tay ROG (Ubuntu 22.04, ROS 2 Humble, GPU GTX 1660 Ti 6 GB). Hình 3.3 trình bày kiến trúc tổng thể.

![Hình 3.3. Kiến trúc phần mềm và phần cứng của hệ thống](hinh/kien_truc.png|16)

Một số quyết định thiết kế quan trọng:

- **Tách mạng ROS của robot thật và mô phỏng.** Robot thật dùng `ROS_DOMAIN_ID=33`, mô phỏng dùng 34, cả hai giới hạn trong máy (`ROS_LOCALHOST_ONLY=1`). Quyết định này được đưa ra sau khi một lần khởi động mô phỏng đã tắt nhầm MoveIt của robot thật.
- **Tách môi trường Python.** ROS Humble dùng Python hệ thống (numpy 1.26, OpenCV 4.10), còn GraspNet và SAM dùng môi trường conda riêng (PyTorch 1.13 + CUDA). Các script khởi động loại conda khỏi `PATH` để tránh xung đột.
- **Định dạng số.** Máy dùng locale tiếng Việt (`LC_NUMERIC=vi_VN`), làm MoveIt đọc giới hạn khớp “3.15” thành chuỗi chữ. Mọi script đặt `LC_NUMERIC=en_US.UTF-8`.

TABLE Bảng 3.3. Các công cụ chính do đề tài xây dựng
| Tệp | Chức năng |
| --- | --- |
| `run_real.sh` | Khởi động robot thật: driver, TF camera, MoveIt, RViz |
| `tools/camera.py` | Cấu hình camera chung: topic, hệ quang học, đọc và áp mô hình sửa độ sâu |
| `tools/tag_check.py` | Ghi mẫu hiệu chuẩn mỗi khi robot đứng yên và thấy tag |
| `tools/handeye_guided.py` | Đề xuất tư thế hiệu chuẩn, lập kế hoạch, chờ xác nhận rồi chạy |
| `tools/handeye_solve.py` | Loại mẫu lật, giải hand-eye, kiểm tra chéo LOO |
| `tools/cam_tf.py`, `cam_check.py` | Phát TF camera; kiểm tra camera có bị xê dịch |
| `tools/depth_plane_fit.py` | Khớp mô hình sửa độ sâu theo pixel từ mặt bàn |
| `tools/grasp_once.py` | Gemini → MobileSAM → GraspNet → lọc → kết quả trong hệ robot |
| `tools/scene_obstacles.py` | Đưa vật trên bàn vào planning scene làm vật cản |
| `tools/mtc_pregrasp.py` | Lập kế hoạch MTC và thực thi chuỗi gắp, bỏ vào hộp, về HOME |
| `tools/gripper.py` | Điều khiển gripper qua HTTP: mở, kẹp theo bề dày, nhả |
| `tools/save_home.py` | Lưu tư thế HOME |
| `tools/run_pick_to_box.sh` | Kịch bản trọn vẹn kèm ghi dữ liệu |
| `tools/record_camera.py`, `snap_rgbd.py` | Quay video camera; chụp ảnh RGB, depth, RGB-D, point cloud |

## 3.4 Sửa driver robot: hàng đợi lệnh ServoJ

### 3.4.1 Hiện tượng

Khi chạy MoveIt trên robot thật, robot thường giật một cái rồi dừng, log báo `Aborted due to state tolerance violation` với độ lệch 0,05 rad ở một khớp. Có lần robot hoàn toàn đứng yên mà driver không báo lỗi gì. Nhóm đã thử ba giả thuyết: chuyển sang chế độ tự động, thêm bộ lọc giới hạn gia tốc vào driver, đặt tốc độ toàn cục 100 %. Không giả thuyết nào giải thích được hiện tượng. Giả thuyết thứ ba trông như đúng, nhưng thực ra robot chạy lại được chỉ vì driver được khởi động lại.

### 3.4.2 Nguyên nhân

Nhóm thêm vào driver phần đọc trạng thái bộ điều khiển (chỉ đọc, mỗi giây một lần) và phát hiện bộ điều khiển xếp hàng các lệnh `ServoJ` rồi thực hiện lần lượt, mỗi lệnh 8 ms. Máy tính gửi khoảng 125 lệnh mỗi giây nhưng không phải hệ thời gian thực: chu kỳ gửi đo được trung bình 8,01 ms, lệch chuẩn 0,32 ms, ngắn nhất 4,8 ms, dài nhất 11 ms. Mỗi khi máy tính gửi nhanh hơn 8 ms, hàng đợi tăng thêm một lệnh và không bao giờ giảm. Số đo trong log: hàng đợi 15 lệnh (robot chậm 0,12 s so với kế hoạch), sau đó 123–124 lệnh (chậm khoảng 1 s). Ở tốc độ 18°/s, chậm 0,12 s đã gần đủ để vượt ngưỡng 2,9°.

Nguyên nhân này giải thích được mọi hiện tượng: khởi động lại driver thì hàng đợi trống nên chạy tốt, chạy một lúc thì hỏng, và chạy càng nhanh càng dễ hỏng.

### 3.4.3 Cách sửa

Driver được sửa để chỉ gửi `ServoJ` khi hàng đợi của bộ điều khiển có tối đa 3 lệnh. Sau khi sửa, hàng đợi luôn ở mức 0–3 lệnh. Trong 31 lần chạy được ghi lại sau đó, 22 lần có độ lệch dưới 1,5°, 8 lần có độ lệch 1,5–2,8°, và 1 lần bất thường chưa giải thích được. Độ lệch còn lại xấp xỉ tốc độ nhân 0,1 s, tức là độ trễ bên trong robot.

![Hình 3.4. Hàng đợi lệnh ServoJ trước và sau khi sửa driver, và thống kê 31 lần chạy sau khi sửa](hinh/hang_doi_servoj.png|15)

## 3.5 Mô hình gripper thật trong URDF

Mô hình URDF ban đầu dùng gripper của mô phỏng: ngắn hơn thật 55 mm, sai hướng 90°, và các hộp va chạm không khớp. Nếu dùng mô hình này để chỉ điểm ở độ cao 10 cm, đầu ngón thật sẽ cắm khoảng 10 mm vào bàn. Đề tài xây dựng mô hình riêng cho robot thật, được chọn tự động khi khởi động với tham số `hardware_type:=fairino`:

- Hình hiển thị dùng file STL thiết kế của gripper và mặt bích FR3.
- Hình va chạm gồm các hộp ôm sát: vành chuyển tiếp 75 × 75 × 28 mm, thân 164 × 85 × 73 mm, và mỗi ngón gồm hai hộp. Ngón có hình tam giác thu nhỏ dần về đầu, nên phần gốc rộng 74 mm còn phần đầu chỉ rộng 30 mm. Mô hình hai hộp cho phép robot hạ thấp thêm khoảng 1 cm khi gắp vật thấp mà không báo va chạm giả.
- Khung TCP `grasp_frame` nằm trên trục mặt bích, cách mặt bích 135 mm, tức cách đầu ngón 30 mm.

Một lỗi khó phát hiện: file khởi động của `move_group` và RViz luôn nạp URDF với `hardware_type: mock`, nên dù `robot_state_publisher` dùng mô hình thật, MoveIt vẫn lập kế hoạch với gripper mô phỏng. Lỗi được phát hiện khi RViz báo kế hoạch hợp lệ nhưng gripper thật đã chạm bàn, và được sửa bằng cách truyền đúng tham số cho cả hai file.

## 3.6 Điều khiển gripper

### 3.6.1 Thư viện gripper.py

Lớp `Gripper` đóng gói các lệnh HTTP tới ESP32. Mỗi lệnh di chuyển đều kiểm tra trước nhiệt độ (từ chối khi từ 60 °C trở lên), đưa servo về chế độ vị trí, bật giữ lực, rồi gửi vị trí đích kèm thời gian di chuyển. Sau đó hàm đọc vị trí liên tục cho tới khi servo tới đích (sai khác tối đa 3 đơn vị) hoặc đứng yên 6 lần đọc liên tiếp (bị vật chặn).

Hiệu chuẩn hành trình: vị trí 1000 (240°) là mở hết, 88 mm; vị trí 0 là đóng hết, hai má còn cách khoảng 30 mm. Lần hiệu chuẩn đầu tiên lưu nhầm chiều (0 là mở), khiến lệnh “mở trước khi tiến vào” thực ra lại khép gripper. Ngón đã chạm vật và người vận hành phải bấm dừng khẩn cấp (mục 4.5).

Wi-Fi tới ESP32 không ổn định: ping dao động 10–760 ms, có lệnh mất hơn 4 s. Mọi lệnh đọc và ghi đều được thử lại tối đa 3 lần với thời gian chờ 6 s. Các lệnh ghi được thiết kế để gửi lặp không đổi kết quả (đặt vị trí, bật/tắt giữ lực), nên thử lại là an toàn.

### 3.6.2 Thuật toán kẹp

Đề tài đã thử ba cách kẹp:

1. **Khép hết rồi giữ ngay sau điểm chạm:** khép về vị trí đóng, phát hiện điểm bị chặn, rồi đặt đích vượt điểm chạm 15 đơn vị (khoảng 1 mm). Lực kẹp quá nhỏ.
2. **Kẹp tới bề rộng đo được trừ 5 mm:** dùng bề dày phần vật giữa hai má (mục 3.7.4) quy đổi tuyến tính sang vị trí servo. Cả khay và giá đều trượt: servo tới đích mà không bị cản, tức hai má chưa ép vào vật. Nguyên nhân là bề rộng đo từ camera và phép quy đổi tuyến tính đều có thể lệch vài mm.
3. **Theo góc tính từ camera, cộng thêm 5°** (cách đang dùng): tính góc servo tại điểm chạm từ bề dày đo được, đặt đích khép thêm 5° (khoảng 21 đơn vị). Nếu servo tới đích mà chưa bị chặn thì tiếp tục khép tới khi bị chặn, rồi mới cộng thêm 5°. Khi đã ở sát cuối hành trình thì không ép thêm, vì lúc đó chỉ làm servo nóng.

Thực nghiệm cho thấy ngón in 3D bị uốn khi siết. Servo có thể khép tới vị trí 3–5 (gần hết hành trình) mà vẫn đang giữ vật. Vì vậy vị trí servo không cho biết có giữ được vật hay không, và hệ thống không dùng nó để quyết định dừng kịch bản.

## 3.7 Khối hiệu chuẩn

### 3.7.1 Gắn tag và mô phỏng trước

AprilTag 36h11 id 2 được dán trên nắp tròn ở cuối trục J6, phía đối diện mặt bích. Theo URDF, nắp này thuộc khâu `wrist2_link` nên không quay theo J6. Quy trình hiệu chuẩn được thử trước trong Gazebo với tag ảo trên nắp J6, đạt sai số 2–4 mm, sau đó mới chạy trên robot thật.

![Hình 3.5. Tag id 2 trên nắp trục J6 trong mô phỏng Gazebo](hinh/p_gazebo_tag.png|9)

![Hình 3.6. Camera phát hiện tag id 2 trên nắp J6 của robot thật](hinh/p_tag_j6.jpg|13)

### 3.7.2 Thu mẫu có dẫn đường

Vùng nhìn của camera nhỏ, nên các tư thế chọn ngẫu nhiên thường đưa tag ra khỏi ảnh. Công cụ `handeye_guided.py` đề xuất các tư thế quanh tư thế hiện tại sao cho:

- tag nằm trong ảnh, cách mép ít nhất 40 px;
- tag nghiêng 20–40° so với trục camera (nhìn thẳng thì dễ bị lật nghiệm);
- tag cách camera ít nhất 0,33 m (gần hơn thì Astra không đo được sâu);
- tư thế đa dạng về vị trí, độ cao và hướng nghiêng.

Với mỗi tư thế, công cụ lập kế hoạch Pilz PTP, hiển thị đường đi trong RViz và chờ người vận hành gõ `y` (chạy), `s` (bỏ qua) hoặc `q` (dừng). Song song, `tag_check.py` tự ghi một mẫu mỗi khi robot đứng yên khoảng 1 s và thấy tag. Khi chưa có kết quả hiệu chuẩn, tuỳ chọn `--camera-from-tag` ước lượng vị trí camera từ một lần nhìn tag để có điểm xuất phát.

### 3.7.3 Phát hiện camera bị xê dịch

Công cụ `cam_check.py` so vị trí tag theo camera với vị trí tính từ robot và kết quả hiệu chuẩn hiện tại. Lệch dưới 5 mm là đạt, trên 12 mm nghĩa là camera đã bị xê dịch. Trong quá trình làm đề tài, chân đế camera bị chạm và xê dịch ít nhất ba lần (một lần xoay 3,9°, một lần xoay 2,1°, một lần dịch 16 mm). Mỗi lần đều phải hiệu chuẩn lại từ đầu.

### 3.7.4 Mô hình sửa độ sâu

Công cụ `depth_plane_fit.py` lấy trung vị 30 khung depth, chọn các pixel thuộc mặt bàn (loại vật và robot), tính độ sâu thật của từng pixel bằng giao tia với mặt phẳng z = 12 mm, rồi khớp mô hình (2.6). Mô hình được lưu thành file và mọi công cụ đọc depth đều áp dụng nó qua hàm `camera.correct_depth`.

![Hình 3.7. Tỉ số giữa độ sâu đo được và độ sâu thật trên toàn ảnh theo mô hình đã khớp: mép trái đo xa hơn thật khoảng 6 %, mép phải gần hơn khoảng 3 %](hinh/mo_hinh_depth.png|12)

## 3.8 Khối nhận thức

### 3.8.1 Quy trình

Công cụ `grasp_once.py` thực hiện các bước sau. Robot phải ở tư thế HOME để không che camera.

1. Lấy ảnh màu và trung vị của 15 khung depth, áp mô hình sửa độ sâu.
2. Gửi ảnh và câu lệnh tới Gemini, nhận khung bao. Bản miễn phí chỉ cho 20 lần gọi mỗi ngày với mô hình Robotics-ER. Khi hết lượt, hệ thống lần lượt thử `gemini-flash-latest`, `gemini-3.5-flash`, `gemini-3-flash-preview`.
3. MobileSAM tách mặt nạ trong khung bao.
4. Chuyển các pixel của mặt nạ thành đám điểm của vật trong hệ robot. Tính hộp bao 3D đứng: đáy là hình chữ nhật diện tích nhỏ nhất (`minAreaRect`) bao 95 % điểm gần tâm nhất, chiều cao là phân vị 95 của độ cao trừ độ cao mặt bàn.
5. Lấy đám điểm cảnh trong bán kính 20 cm quanh vật, từ mặt bàn lên 25 cm, đưa vào GraspNet.
6. Lọc các tư thế gắp, giữ 15 tư thế tốt nhất, lưu kết quả và ảnh minh hoạ.

### 3.8.2 Các bộ lọc

TABLE Bảng 3.4. Các bộ lọc tư thế gắp và lý do
| Bộ lọc | Ngưỡng | Lý do |
| --- | --- | --- |
| Nằm trên vật | tâm gắp cách điểm gần nhất của vật dưới 20 mm | GraspNet đề xuất cả trên bàn, trên robot, trên vật khác |
| Không va chạm | bộ kiểm tra va chạm của GraspNet, voxel 10 mm | Ngón không được cắm vào cảnh |
| Độ mở tối đa | dưới 85 mm | Gripper mở tối đa 88 mm |
| Độ mở tối thiểu | trên 40 mm | Gripper chỉ khép tới khoảng 30 mm, GraspNet thường mở rộng hơn vật khoảng 10 mm |
| Độ ăn sâu | dưới 30 mm | Chỉ 30 mm ngoài cùng của ngón dùng để kẹp |
| Phần bị kẹp | dày trên 33 mm (đo từ đám điểm) | Mỏng hơn thì hai má không ép tới |
| Má không đè lên vật | dưới 20 điểm của vật nằm ngay chỗ má sẽ đi xuống | Má đè lên vật sẽ va khi tiến vào |

Hai bộ lọc cuối được tính trong hệ toạ độ của TCP: một điểm của vật có toạ độ (x, y, z) trong hệ TCP, với x theo hướng khép má, y theo bề ngang ngón, z theo hướng tiếp cận. Điểm nằm giữa hai má khi |y| < 19 mm, 30 − 40 < z < 30 mm (40 mm tính từ đầu ngón vào trong) và |x| < 44 mm. Hai má khép đối xứng, đẩy vật về giữa, nên phần bị kẹp có bề dày:

$$ e = max(x) − min(x)  trên các điểm nằm giữa hai má $$ (3.1)

Điểm nằm ngay chỗ má khi |y| < 19 mm, z trong cùng khoảng trên và 44 ≤ |x| < 82 mm.

### 3.8.3 Vật không đo được độ sâu

Khay nhựa đen hấp thụ tia hồng ngoại nên chỉ 15 % số pixel trên khay có giá trị độ sâu. GraspNet không có đám điểm để làm việc. Đề tài bổ sung chế độ `--assume-height`: biết trước chiều cao vật (47 mm), viền mặt nạ SAM được chiếu lên mặt phẳng miệng khay (độ cao mặt bàn cộng 47 mm) bằng cách giao tia từ camera, rồi tính hộp bao từ đó. Robot gắp thẳng từ trên xuống, kẹp ngang cạnh ngắn của hộp bao. Hộp tính được là 101 × 68 × 47 mm, khớp với số đo thật 100 × 67 × 47 mm.

![Hình 3.8. Kết quả nhận thức cho lệnh “the white object”: mặt nạ SAM (tím), các tư thế GraspNet còn lại sau lọc (cam, vàng), tư thế bị loại vì va chạm (xám)](hinh/p_graspnet_gia.jpg|14)

## 3.9 Khối vật cản

Công cụ `scene_obstacles.py` đưa các vật nhìn thấy trên bàn vào planning scene của MoveIt dưới dạng hộp, để kế hoạch tránh chúng:

1. Lấy trung vị 15 khung depth đã sửa, chuyển sang hệ robot.
2. Giữ các điểm cao hơn mặt bàn 8 mm tới 30 cm. Ngưỡng 8 mm lớn hơn khoảng ba lần nhiễu mặt bàn.
3. Loại các điểm cách chuỗi khâu của robot dưới 9 cm (lấy vị trí các khâu từ TF), để không coi chính robot là vật cản.
4. Gom các vùng liên thông trên ảnh (bỏ vùng nhỏ hơn 120 pixel). Mỗi vùng thành một hộp đứng: đáy là hình chữ nhật nhỏ nhất bao các điểm cộng thêm lề 1 cm mỗi bên, chiều cao từ mặt bàn tới phân vị 95.
5. Bỏ qua vật đích của lần nhận thức gần nhất, vì gripper phải ôm quanh nó.

Hộp đựng vật được xử lý riêng: hộp đặc do bước 4 sinh ra được thay bằng bốn thành dày 12 mm, để ngón và vật có thể đi vào lòng hộp.

![Hình 3.9. Vật cản camera nhìn thấy (xanh lá) và vật đích (nhãn “dich”); hộp vàng có thành mỏng và bóng nên chỉ đo được một phần](hinh/p_vat_can_hop.jpg|13)

## 3.10 Khối lập kế hoạch và thực thi

### 3.10.1 Tác vụ MTC

Công cụ `mtc_pregrasp.py` dựng tác vụ MTC theo Hình 2.2. Các tham số chính:

TABLE Bảng 3.5. Tham số của tác vụ MTC
| Tham số | Giá trị | Ý nghĩa |
| --- | --- | --- |
| `--approach` | 0,08 m | Đoạn tiến thẳng vào vật theo trục TCP |
| `--deeper` | 0,015 m | Ăn sâu thêm so với độ sâu GraspNet |
| `--lift` | 0,20 m | Nhấc thẳng lên sau khi kẹp (tối thiểu 50 %) |
| `--scale` | 0,3 | Hệ số tốc độ và gia tốc (đoạn thẳng dùng một nửa) |
| `--max-base-turn` | 90° | Loại kế hoạch làm khớp đế quay quá xa |
| `--grip-deg` | 5° | Khép thêm so với góc servo tại điểm chạm |
| `--place-above-floor` | 0,02 m | Hạ vật tới cách đáy hộp 2 cm (nếu thân gripper còn ở trên miệng hộp) |
| `max_ik_solutions` | 16 | Số nghiệm IK tối đa mỗi tư thế đích |
| `step_size` Cartesian | 5 mm | Bước nội suy của đường thẳng |

Độ ăn sâu được giới hạn để góc thấp nhất của ngón (theo mô hình hai hộp ở mục 3.5) luôn cao hơn tấm bàn trong planning scene 3 mm. Điều này tránh trường hợp tư thế gắp nghiêng trên vật thấp làm góc ngón cắm vào bàn.

### 3.10.2 Chọn tư thế gắp

Với mỗi tư thế gắp còn lại sau lọc, hệ thống thử cả hai chiều xoay má (0° và 180°) và bốn hướng thả vào hộp (0°, ±90°, 180° quanh trục đứng), rồi lập kế hoạch cho cả chuỗi. Một kế hoạch bị loại nếu:

- khớp đế j1 phải quay quá 90° so với lúc đầu ở bất kỳ thời điểm nào (lần đầu tiên chưa có giới hạn này, kế hoạch rẻ nhất làm đế xoay khoảng 300°);
- phần bị kẹp mỏng hơn 33 mm, hoặc má đè lên vật (mục 3.8.2).

Trong các kế hoạch còn lại, hệ thống chọn kế hoạch có độ khó nhỏ nhất:

$$ độ khó = góc nghiêng (độ) + lệch tâm vật (mm) + chi phí đường đi (rad) $$ (3.2)

Trước đây hệ thống chỉ chọn theo chi phí đường đi, nên hay chọn cách gắp nghiêng, lệch tâm 50 mm, kẹp vào phần mỏng 17 mm. Công thức (3.2) ưu tiên cách gắp gần thẳng đứng và gần tâm vật, là những cách gắp chắc chắn hơn.

### 3.10.3 Thả vào hộp

Điểm thả được chọn trong lòng hộp, cách thành 11 cm (thành 12 mm cộng tầm vươn ngang 82 mm của ngón), tại điểm gần vật nhất. Lý do là tâm hộp ở vị trí thử nghiệm nằm ở hướng 166° nhìn từ đế robot, sát giới hạn 175° của khớp j1, nên thả đúng tâm thì robot phải lật tay.

Ban đầu, vật được thả khi đáy vật cao hơn miệng hộp 8 cm, tức là rơi tự do. Người vận hành yêu cầu đặt sâu vào hộp. Đề tài thử hạ vật xuống cách đáy hộp 2 cm, nhưng lần nào thân gripper cũng va vào thành hộp. Cách dung hoà là hạ tới khi thân gripper vừa ở trên miệng hộp: TCP cao hơn miệng hộp 5 mm, lúc đó ngón dài 60 mm và vật đã nằm trong lòng hộp.

### 3.10.4 Thực thi có kiểm tra

Lời giải MTC gồm 7 đoạn quỹ đạo. Hệ thống không gọi thực thi cả lời giải một lần, mà gửi từng đoạn qua action `/execute_trajectory` của MoveIt để chèn thao tác gripper và các bước kiểm tra vào giữa:

1. Mở gripper. Đi tới điểm chờ.
2. Đọc TCP thật từ TF. Nếu lệch khỏi đường tiến vào quá 10 mm thì dừng.
3. Tiến vào. Đọc TCP. Nếu lệch đích quá 10 mm thì không kẹp.
4. Kẹp theo thuật toán ở mục 3.6.2.
5. Nhấc lên, mang tới hộp, hạ xuống.
6. Mở gripper. Nếu servo quá nóng thì nhả lực để vật rơi vào hộp.
7. Nhấc ra khỏi hộp, về HOME.

Người vận hành chỉ xác nhận một lần trước bước 1, sau khi đã xem toàn bộ đường đi trong RViz. Tuỳ chọn `--step` cho phép hỏi lại trước từng bước.

## 3.11 Ghi dữ liệu và kịch bản

Mỗi lần chạy kịch bản được lưu trong một thư mục riêng `nhat_ky/du_lieu/<ngày_giờ>_<nhãn>/`, gồm:

- ảnh trước và sau khi chạy: RGB, depth tô màu theo thang 0,4–1,0 m, RGB-D chồng 50 %, depth 16 bit tính bằng mm, point cloud màu trong hệ robot (`.ply`);
- video camera ghép ảnh màu và depth, có đồng hồ, 15 khung/s, kèm bản H.264 dễ chia sẻ;
- ảnh vật cản, kết quả nhận thức (`.yaml`, đám điểm `.npy`) và log của MTC.

Kịch bản trọn vẹn được chạy bằng một lệnh:

```
tools/run_pick_to_box.sh "the white object" "the yellow plastic box" gia_trang \
    --deeper 0.015 --lift 0.20 --scale 0.3
```

Lệnh này lần lượt: về HOME nếu chưa ở đó, chụp ảnh “trước”, bắt đầu quay, nhận thức vật, nhận thức hộp, cập nhật vật cản, lập kế hoạch và hỏi xác nhận, thực thi, chụp ảnh “sau”, dừng quay và xuất video.

## 3.12 Các biện pháp an toàn

- Robot thật và mô phỏng ở hai mạng ROS khác nhau.
- Tốc độ khớp giới hạn ở 11 % tốc độ tối đa; kịch bản chạy ở hệ số 0,2–0,3 của mức đó.
- Mọi chuyển động đều hiện trong RViz trước và chỉ chạy khi người vận hành gõ `y`.
- Tự dừng khi TCP lệch quá 10 mm so với kế hoạch.
- Gripper không di chuyển khi từ 60 °C trở lên (kiểm tra ở cả phần mềm và firmware).
- Người vận hành luôn đứng cạnh nút dừng khẩn cấp. Trong đề tài, nút này đã được dùng hai lần, đều trước khi có chuyển động gây hư hại.
