# CHƯƠNG 4. THIẾT KẾ VÀ XÂY DỰNG HỆ THỐNG

## 4.1 Yêu cầu đối với hệ thống

Từ mục tiêu ở phần Mở đầu, hệ thống cần thoả mãn các yêu cầu sau:

TABLE Bảng 4.1. Yêu cầu chức năng và phi chức năng
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

## 4.2 Phần cứng

### 4.2.1 Tay máy FAIRINO FR3

Robot, bộ điều khiển và cách vận hành qua ROS 2 đã được giới thiệu ở Chương 2. Trong hệ thống, FR3 được gắn cố định cạnh bàn thí nghiệm. Hai thông số được cấu hình riêng cho đề tài:

- Tốc độ khớp trong MoveIt giới hạn ở 0,35 rad/s (khoảng 11 % tốc độ tối đa); các kịch bản chạy ở hệ số 0,2–0,3 của mức này.
- Mặt bàn trong planning scene là một tấm 1,6 × 1,6 × 0,05 m có mặt trên ở độ cao 12 mm so với gốc robot, khớp với độ cao thật đo được bằng cách cho robot chạm bàn.

### 4.2.2 Camera

Trong giai đoạn đầu, đề tài dùng camera Luxonis OAK-D Pro W (stereo, góc rộng 95° × 72°, ảnh màu 1280 × 720). Ngày 30/10/2026, cảm biến ảnh màu IMX378 của OAK bị hỏng phần cứng: mọi luồng có camera màu đều khiến thiết bị khởi động lại sau khoảng 0,2 s, dù camera đen trắng vẫn chạy tốt. Đề tài chuyển sang camera Orbbec Astra Pro Plus, loại dùng ánh sáng cấu trúc hồng ngoại.

TABLE Bảng 4.2. So sánh hai camera đã dùng
| Thông số | OAK-D Pro W | Astra Pro Plus |
| --- | --- | --- |
| Nguyên lý đo sâu | Stereo + chấm hồng ngoại | Ánh sáng cấu trúc hồng ngoại |
| Ảnh màu dùng trong đề tài | 1280 × 720, f_x ≈ 759 px | 640 × 480, f_x ≈ 544 px |
| Tầm đo gần nhất | khoảng 0,2 m | khoảng 0,53 m |
| Nhiễu mặt bàn ở 0,67–0,7 m | độ lệch chuẩn 1,6–1,8 mm | độ lệch chuẩn khoảng 2,9 mm sau hiệu chỉnh |
| Sai số hệ thống | đo xa hơn thật 7–12 %, tăng theo khoảng cách | đo xa hơn thật 0–6 %, thay đổi theo vị trí pixel |
| Điểm yếu | hỏng cảm biến màu ngày 30/10 | không đo được vật đen nhám |

Camera Astra được đặt cố định trên giá, cách mặt bàn khoảng 0,7 m, nhìn gần như thẳng xuống (nghiêng khoảng 9°). Vị trí này được chọn sau nhiều lần thử: đặt thấp 0,63 m thì mặt trên của vật nằm trong vùng camera không đo được (dưới 0,53 m), đặt nghiêng nhiều thì ảnh bị méo ở mép và tag nhỏ.

![Hình 4.1. Camera đặt phía trên bàn, nhìn thẳng xuống; trên bàn là các vật thử nghiệm](hinh/p_cam_gan.jpg|13)

### 4.2.3 Tay gắp

Tay gắp hai ngón do nhóm tự thiết kế, chế tạo và lập trình. Toàn bộ phần cơ khí, điện tử, giao thức và phần mềm của tay gắp được trình bày riêng ở mục 4.5.

## 4.3 Kiến trúc phần mềm

Toàn bộ phần mềm chạy trên một máy tính xách tay ROG (Ubuntu 22.04, ROS 2 Humble, GPU GTX 1660 Ti 6 GB). Hình 4.2 trình bày kiến trúc tổng thể.

![Hình 4.2. Kiến trúc phần mềm và phần cứng của hệ thống](hinh/kien_truc.png|16)

Một số quyết định thiết kế quan trọng:

- **Tách mạng ROS của robot thật và mô phỏng.** Robot thật dùng `ROS_DOMAIN_ID=33`, mô phỏng dùng 34, cả hai giới hạn trong máy (`ROS_LOCALHOST_ONLY=1`). Quyết định này được đưa ra sau khi một lần khởi động mô phỏng đã tắt nhầm MoveIt của robot thật.
- **Tách môi trường Python.** ROS Humble dùng Python hệ thống (numpy 1.26, OpenCV 4.10), còn GraspNet và SAM dùng môi trường conda riêng (PyTorch 1.13 + CUDA). Các script khởi động loại conda khỏi `PATH` để tránh xung đột.
- **Định dạng số.** Máy dùng locale tiếng Việt (`LC_NUMERIC=vi_VN`), làm MoveIt đọc giới hạn khớp “3.15” thành chuỗi chữ. Mọi script đặt `LC_NUMERIC=en_US.UTF-8`.

TABLE Bảng 4.3. Các công cụ chính do đề tài xây dựng
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
| `tools/gripper.py` | Điều khiển gripper (qua micro-ROS hoặc cổng HTTP chẩn đoán): mở, kẹp theo bề dày, nhả |
| `tools/save_home.py` | Lưu tư thế HOME |
| `tools/run_pick_to_box.sh` | Kịch bản trọn vẹn kèm ghi dữ liệu |
| `tools/record_camera.py`, `snap_rgbd.py` | Quay video camera; chụp ảnh RGB, depth, RGB-D, point cloud |

## 4.4 Mô hình gripper thật trong URDF

Mô hình URDF ban đầu dùng gripper của mô phỏng: ngắn hơn thật 55 mm, sai hướng 90°, và các hộp va chạm không khớp. Nếu dùng mô hình này để chỉ điểm ở độ cao 10 cm, đầu ngón thật sẽ cắm khoảng 10 mm vào bàn. Đề tài xây dựng mô hình riêng cho robot thật, được chọn tự động khi khởi động với tham số `hardware_type:=fairino`:

- Hình hiển thị dùng file STL thiết kế của gripper và mặt bích FR3.
- Hình va chạm gồm các hộp ôm sát: vành chuyển tiếp 75 × 75 × 28 mm, thân 164 × 85 × 73 mm, và mỗi ngón gồm hai hộp. Ngón có hình tam giác thu nhỏ dần về đầu, nên phần gốc rộng 74 mm còn phần đầu chỉ rộng 30 mm. Mô hình hai hộp cho phép robot hạ thấp thêm khoảng 1 cm khi gắp vật thấp mà không báo va chạm giả.
- Khung TCP `grasp_frame` nằm trên trục mặt bích, cách mặt bích 135 mm, tức cách đầu ngón 30 mm.

Một lỗi khó phát hiện: file khởi động của `move_group` và RViz luôn nạp URDF với `hardware_type: mock`, nên dù `robot_state_publisher` dùng mô hình thật, MoveIt vẫn lập kế hoạch với gripper mô phỏng. Lỗi được phát hiện khi RViz báo kế hoạch hợp lệ nhưng gripper thật đã chạm bàn, và được sửa bằng cách truyền đúng tham số cho cả hai file.

## 4.5 Tay gắp: thiết kế cơ khí, điện tử và phần mềm

Tay gắp (gripper) là phần nhóm tự thiết kế và chế tạo hoàn toàn, thuộc nội dung nghiên cứu thứ nhất. Mục này trình bày tay gắp theo bốn lớp: cơ khí, điện tử, giao thức truyền thông và phần mềm (firmware trên vi điều khiển và thư viện trên máy tính), sau đó là thuật toán kẹp và kết quả thử nghiệm.

### 4.5.1 Yêu cầu và phương án thiết kế

Tay gắp cần đáp ứng các yêu cầu sau:

TABLE Bảng 4.4. Yêu cầu thiết kế tay gắp
| Yêu cầu | Chỉ tiêu | Lý do |
| --- | --- | --- |
| Dải vật gắp được | Bề rộng khoảng 30–85 mm | Phù hợp các vật thử nghiệm: hộp, khay, tay cầm game, giá in 3D |
| Kiểu kẹp | Hai má song song, khép đối xứng | Đơn giản, vật tự được đẩy về giữa, khớp với giả thiết của GraspNet |
| Khối lượng | Nhỏ so với tải 3 kg của FR3 | Để dành tải cho vật |
| Phản hồi trạng thái | Đọc được vị trí, nhiệt độ, điện áp | Biết gripper đang mở hay đóng, phát hiện quá nhiệt |
| Kết nối | Không cần thêm cáp chạy dọc cánh tay robot | Dây chạy dọc tay dễ vướng khi robot xoay khớp |
| An toàn | Tự dừng khi mất liên lạc, chặn lệnh khi quá nóng | Tránh kẹp vỡ vật hoặc hỏng servo |

Phương án được chọn là **một servo bus dẫn động hai má đối xứng**, điều khiển qua Wi-Fi bằng một vi điều khiển ESP32 gắn ngay trên thân gripper. So với servo PWM thông thường, servo bus có ba ưu điểm quan trọng: trả về vị trí, nhiệt độ và điện áp qua đường truyền; nhiều servo dùng chung một đường dây (nếu sau này cần thêm bậc tự do); và có chế độ quay liên tục. So với gripper điện công nghiệp, phương án này rẻ hơn nhiều và cho phép nhóm hiểu, sửa được toàn bộ hệ thống.

### 4.5.2 Cấu trúc cơ khí

Hình 4.3 là mô hình 3D của tay gắp, lắp trên khâu cổ tay J6 của FR3. Hình được dựng trực tiếp từ các file thiết kế (STL), đặt đúng vị trí như trong mô hình robot mà RViz, MoveIt và Gazebo sử dụng, nên những gì thấy trong hình cũng chính là những gì bộ lập kế hoạch dùng để kiểm tra va chạm. Từ trên xuống, tay gắp gồm các phần:

1. **Mặt bích lắp dụng cụ** của FR3 (đường kính 63 mm) và **vành chuyển tiếp** dày 28 mm, in 3D, nối mặt bích với thân gripper.
2. **Phần servo và mạch:** chứa servo bus, bộ chuyển BusLinker và mạch ESP32. Phần này chưa có bản vẽ CAD chi tiết, nên trên Hình 4.3 nó chỉ được vẽ **minh họa** bằng một hình trụ trong suốt, đường kính bằng mặt bích (63 mm) và cao 47 mm. Hình dạng và cách bố trí linh kiện ngoài thực tế khác với hình minh họa này. Riêng trong mô hình va chạm của MoveIt, phần này được bao bằng một hình hộp 164 × 85 × 47 mm, rộng hơn thực tế để bộ lập kế hoạch luôn giữ khoảng an toàn.
3. **Khung gripper** in 3D, dài 164 mm, đỡ hai má và dẫn hướng cho chúng trượt đối xứng.
4. **Hai má kẹp** in 3D, dài 60 mm, rộng 38 mm, có dạng tam giác thu nhỏ dần về đầu: phần gốc rộng 74 mm theo phương vuông góc với hướng kẹp, phần đầu chỉ rộng 30 mm. Một servo dẫn động cả hai má qua cơ cấu đối xứng, nên hai má luôn khép vào giữa cùng một lúc. Chỉ 30 mm ngoài cùng của má được dùng để kẹp, vì kẹp sâu hơn thì khung gripper dễ chạm vào vật.

![Hình 4.3. Mô hình 3D của tay gắp lắp trên khâu cổ tay FR3, dựng từ file thiết kế đúng như trong mô hình robot: (a) hai má mở hết, (b) khép hết, (c) nhìn chính diện. Hình trụ xanh trong suốt chỉ minh họa phần servo và mạch ESP32, thực tế khác. Điểm TCP (grasp_frame) nằm trên trục mặt bích, cách đầu ngón 30 mm](hinh/gripper_3d.png|15)

Các kích thước được đo trực tiếp trên gripper thật. Chiều dài từ mặt bích tới đầu ngón (165 mm) được đo bằng cách cho robot hạ dần tới lúc đầu ngón vừa chạm mặt bàn, rồi đọc toạ độ mặt bích từ robot. Đo bằng thước lần đầu chỉ được 137 mm, do vô tình đo từ vành chuyển tiếp chứ không phải từ mặt bích. Sai 28 mm như vậy đủ để gripper cắm vào bàn, nên mọi kích thước đưa vào mô hình robot đều được kiểm tra lại bằng cách chạm.

Gripper được lắp xoay −90° quanh trục mặt bích so với mô hình ban đầu trong mô phỏng, sao cho khi mặt bích úp xuống thì hai má nằm song song với mặt bàn và mạch ESP32-CAM trên thân gripper hướng lên trên. Mô hình va chạm của gripper trong MoveIt được dựng lại tương ứng (mục 4.4).

### 4.5.3 Phần cứng điện tử

Hình 4.4 là sơ đồ khối phần điện tử của tay gắp. Bảng 4.5 liệt kê các linh kiện và Bảng 4.6 là bảng nối chân.

![Hình 4.4. Sơ đồ khối phần điện tử của tay gắp](hinh/gripper_dien_tu.png|15)

TABLE Bảng 4.5. Linh kiện điện tử của tay gắp
| Linh kiện | Thông số | Vai trò |
| --- | --- | --- |
| Servo bus Hiwonder | Mô-men 35 kg·cm, góc 0–240°, phân giải 1000 bước (0,24°/bước), ID 1 | Dẫn động hai má; tự điều khiển vị trí bên trong; trả về vị trí, nhiệt độ, điện áp |
| BusLinker V3 (Hiwonder) | Đầu vào UART mức TTL hoặc USB, chọn bằng jumper; đầu ra bus bán song công một dây | Chuyển tín hiệu UART hai dây (TX, RX) của vi điều khiển sang bus một dây của servo; cấp nguồn cho servo |
| ESP32-CAM (AI-Thinker) | ESP32 lõi kép 240 MHz, Wi-Fi 2,4 GHz | Chạy micro-ROS client: nhận lệnh từ máy tính qua Wi-Fi, chuyển thành gói lệnh servo qua UART và gửi trạng thái servo về máy tính. Camera OV2640 có sẵn trên mạch không được dùng và bị tắt |
| Nguồn servo | Đo được 13,4 V tại servo | Cấp năng lượng cho servo qua BusLinker |

TABLE Bảng 4.6. Bảng nối chân giữa ESP32-CAM và BusLinker
| Chân ESP32-CAM | Nối tới | Ghi chú |
| --- | --- | --- |
| U0T (GPIO1) | RX của BusLinker | ESP32 gửi lệnh tới servo |
| U0R (GPIO3) | TX của BusLinker | Servo trả lời về ESP32 |
| GND | GND của BusLinker và nguồn | Bắt buộc có đất chung |
| GPIO32 | Chân PWDN của camera OV2640 | Giữ mức cao để tắt camera (không dùng), giảm tiêu thụ và nhiệt |
| GPIO4 | Đèn flash trên mạch | Giữ mức thấp để tắt đèn |

Nhóm chọn mạch ESP32-CAM chủ yếu vì nó nhỏ, rẻ và có sẵn Wi-Fi. Camera OV2640 đi kèm có chất lượng ảnh thấp, góc nhìn hẹp và không có độ sâu, nên không được dùng cho việc nhận diện; mọi thông tin thị giác đều lấy từ camera RGB-D cố định phía trên bàn (mục 4.2.2).

Một số lưu ý khi thiết kế mạch:

- **Dùng UART0 thay cho GPIO16/17.** Nhiều tài liệu ESP32 dùng GPIO16/17 cho UART thứ hai, nhưng trên ESP32-CAM hai chân này đã được bộ nhớ PSRAM dùng. Vì vậy UART0 (GPIO1/3) được dành riêng cho servo.
- **Phải rút dây của mạch nạp khi chạy.** UART0 cũng là cổng nạp chương trình. Khi servo hoạt động, dây TX/RX của mạch nạp USB–UART phải được tháo ra để không tranh đường truyền. Sau lần nạp đầu, firmware được cập nhật qua Wi-Fi (OTA) nên không cần cắm lại mạch nạp.
- **Jumper của BusLinker đặt ở chế độ Servo-TTL**, để BusLinker nhận tín hiệu từ chân UART của ESP32 thay vì từ cổng USB của nó.
- **Nguồn servo tách riêng.** Cổng USB không đủ dòng cho servo 35 kg·cm. Servo được cấp nguồn riêng qua BusLinker, còn ESP32 chỉ nối chung đất.

### 4.5.4 Giao thức truyền thông với servo

Servo bus Hiwonder dùng giao thức nối tiếp 115200 baud, bán song công (gửi và nhận trên cùng một dây, lần lượt). Mỗi gói lệnh có cấu trúc:

TABLE Bảng 4.7. Cấu trúc gói lệnh servo bus Hiwonder
| Byte | Tên | Giá trị |
| --- | --- | --- |
| 0–1 | Đầu gói | 0x55 0x55 |
| 2 | ID | 1 (servo của gripper); 0xFE là địa chỉ phát cho mọi servo |
| 3 | LEN | Số byte tham số + 3 |
| 4 | CMD | Mã lệnh (Bảng 4.8) |
| 5… | Tham số | Tuỳ lệnh, số nguyên 16 bit gửi byte thấp trước |
| cuối | CHECKSUM | Đảo bit của (ID + LEN + CMD + tổng tham số), lấy 8 bit thấp |

Ví dụ, lệnh đưa servo tới vị trí 500 trong 1000 ms (lệnh 1, tham số 500 và 1000) là chuỗi byte `55 55 01 07 01 F4 01 E8 03 16`. Servo nhận lệnh, tự nội suy tới đích trong thời gian yêu cầu. Lệnh đọc (ví dụ đọc vị trí, mã 28) được servo trả lời bằng một gói cùng cấu trúc, mang giá trị cần đọc.

TABLE Bảng 4.8. Các lệnh servo được dùng trong đề tài
| Mã | Tên | Chiều | Ý nghĩa |
| --- | --- | --- | --- |
| 1 | MOVE_TIME_WRITE | Ghi | Tới vị trí p (0–1000) trong thời gian t (ms) |
| 12 | MOVE_STOP | Ghi | Dừng ngay |
| 14 | ID_READ | Đọc | Đọc ID của servo |
| 19 | ANGLE_OFFSET_READ | Đọc | Độ lệch gốc đã lưu (đọc được: 5) |
| 21 | ANGLE_LIMIT_READ | Đọc | Giới hạn góc đã lưu (đọc được: 0–1000) |
| 26 | TEMP_READ | Đọc | Nhiệt độ bên trong servo (°C) |
| 27 | VIN_READ | Đọc | Điện áp nguồn (mV) |
| 28 | POS_READ | Đọc | Vị trí hiện tại |
| 29 | MODE_WRITE | Ghi | Chế độ vị trí hoặc chế độ động cơ quay liên tục với tốc độ cho trước |
| 30 | MODE_READ | Đọc | Đọc chế độ hiện tại |
| 31 | LOAD_WRITE | Ghi | Bật hoặc tắt giữ lực |
| 32 | LOAD_READ | Đọc | Đọc trạng thái giữ lực |

Các lệnh ghi vào bộ nhớ lâu dài của servo (đổi ID, đổi giới hạn góc, đổi độ lệch gốc) bị firmware chặn hoàn toàn, để tránh vô tình làm hỏng cấu hình servo qua mạng.

### 4.5.5 Firmware trên ESP32 và micro-ROS

**micro-ROS là gì.** ROS 2 dùng DDS (Data Distribution Service) làm tầng truyền thông. DDS quá nặng để chạy trên vi điều khiển chỉ có vài trăm kB RAM như ESP32. micro-ROS [28] giải quyết việc này bằng cách tách vai trò thành hai phần:

- **micro-ROS client** chạy trên vi điều khiển. Client dùng thư viện **rclc** (API ROS 2 viết bằng C, tối giản) để tạo nút, publisher, subscriber, bộ hẹn giờ và bộ thực thi (executor), giống như một nút ROS 2 thông thường. Client nói chuyện bằng giao thức **XRCE-DDS** [29] (DDS for eXtremely Resource Constrained Environments), một phiên bản rút gọn của DDS dành cho thiết bị nhỏ.
- **micro-ROS Agent** chạy trên máy tính. Agent nhận các gói XRCE-DDS từ client (qua UDP, UART hoặc USB), rồi thay mặt client tạo các thực thể DDS thật trong đồ thị ROS 2.

Nhờ cơ chế này, với mọi nút khác trên máy tính, ESP32 hiện ra như **một nút ROS 2 bình thường**: liệt kê được bằng `ros2 node list`, các topic của nó xem được bằng `ros2 topic echo`, và các chương trình ROS 2 khác (kịch bản gắp, RViz, bộ ghi rosbag) giao tiếp với gripper bằng đúng cơ chế topic như với robot.

**Cấu trúc firmware.** Firmware được viết cho ESP32 (Arduino, thư viện micro-ROS cho ESP32), gồm hai phần:

- **Khởi động (setup):** mở UART0 ở 115200 baud cho servo; kéo GPIO32 lên mức cao để tắt camera, kéo GPIO4 xuống để tắt đèn; kết nối Wi-Fi ở chế độ trạm; khởi tạo transport UDP tới địa chỉ của micro-ROS Agent trên máy tính; tạo nút gripper, một **subscriber** nhận lệnh kẹp và một **publisher** gửi trạng thái servo; bật cập nhật firmware qua mạng (OTA) có mật khẩu.
- **Vòng lặp (loop):** chạy executor của rclc để xử lý lệnh đến; theo chu kỳ, đọc vị trí, nhiệt độ, điện áp của servo và phát lên topic trạng thái; kiểm tra kết nối tới Agent, nếu mất thì huỷ các thực thể và kết nối lại; kiểm tra hạn chót của lệnh quay liên tục để tự dừng servo.

TABLE Bảng 4.9. Giao diện của gripper trong ROS 2 (qua micro-ROS) và cổng chẩn đoán
| Kênh | Chiều | Nội dung |
| --- | --- | --- |
| Topic lệnh kẹp | Máy tính → ESP32 | Vị trí đích 0–1000 và thời gian chuyển động; các lệnh mở, kẹp, nhả lực, dừng |
| Topic trạng thái | ESP32 → máy tính | Vị trí hiện tại, nhiệt độ (°C), điện áp (mV), trạng thái giữ lực, chế độ |
| Cổng HTTP chẩn đoán | Hai chiều | `GET /servo/status`, `GET /servo/probe` (dò servo trên bus), `POST /servo/packet` (gửi một lệnh trong danh sách cho phép, có mật khẩu): dùng khi thử nghiệm, hiệu chuẩn và khi Agent chưa chạy |
| OTA | Máy tính → ESP32 | Nạp firmware mới qua Wi-Fi, có mật khẩu |

Các cơ chế an toàn nằm ngay trong firmware, nên được áp dụng cho mọi kênh lệnh:

1. **Danh sách lệnh cho phép:** chỉ chấp nhận ID 1 (hoặc địa chỉ phát cho lệnh đọc), chỉ các lệnh ở Bảng 4.8, tham số trong giới hạn (vị trí 0–1000, thời gian 1–30 000 ms, tốc độ −1000…+1000).
2. **Chặn khi quá nóng:** mọi lệnh làm servo chuyển động bị từ chối nếu nhiệt độ từ 60 °C trở lên hoặc không đọc được nhiệt độ.
3. **Lệnh vị trí chỉ chạy ở chế độ vị trí:** tránh trường hợp servo đang quay liên tục mà nhận lệnh vị trí.
4. **Tự dừng sau 1 s:** ở chế độ quay liên tục, nếu sau 1 s không có lệnh mới hoặc mất kết nối, servo tự dừng.
5. **Xác thực mật khẩu** cho các lệnh ghi qua cổng HTTP và cho OTA.

### 4.5.6 Phần mềm trên máy tính

Phía máy tính gồm micro-ROS Agent và các chương trình điều khiển gripper (Hình 4.5). Agent được chạy cùng các thành phần ROS 2 khác của hệ thống và lắng nghe client trên một cổng UDP. Sau khi ESP32 kết nối, các chương trình điều khiển gửi lệnh và đọc trạng thái gripper qua topic.

![Hình 4.5. Các lớp phần mềm điều khiển tay gắp theo kiến trúc micro-ROS, từ kịch bản gắp tới servo](hinh/gripper_phan_mem.png|15)

**Công cụ thử gripper_test.py** là chương trình dòng lệnh để thử và hiệu chuẩn tay gắp bằng tay:

TABLE Bảng 4.10. Các lệnh của công cụ thử tay gắp
| Lệnh | Chức năng |
| --- | --- |
| `s` | Đọc vị trí, điện áp, nhiệt độ, trạng thái giữ lực |
| `scan` | Tìm ID servo trên bus |
| `a <độ> [ms]` | Quay tới góc tuyệt đối 0–240° trong thời gian cho trước |
| `r <độ> [ms]` | Quay tương đối, ví dụ `r -5` để lùi 5° |
| `hold` / `free` | Bật / tắt giữ lực (tắt thì xoay được bằng tay) |
| `motor <tốc độ> [ms]` | Quay liên tục một nhịp ngắn, tối đa 1000 ms, rồi tự dừng |
| `calib` | Nhả lực, người vận hành đưa hai má về vị trí mở hết và đóng hết bằng tay, công cụ ghi lại hai vị trí |
| `o` / `c` / `g` | Mở / đóng / kẹp (khép tới khi bị vật chặn) |
| `w` | Theo dõi vị trí liên tục |

Kết quả hiệu chuẩn: vị trí servo 1000 (240°) là mở hết, hai má cách 88 mm; vị trí 0 là cuối hành trình, hai má còn cách khoảng 30 mm. Giữa hai điểm này, bề rộng giữa hai má được quy đổi tuyến tính:

$$ w(p) = 30 + 58 · p / 1000  (mm),   p ∈ [0; 1000] $$ (4.1)

Vì servo đã dùng hết 240° hành trình mà hai má chưa khép kín, muốn kẹp vật mỏng hơn thì phải đổi tỉ số truyền cơ khí. Do thời gian có hạn, nhóm giữ nguyên cơ khí và giới hạn hệ thống ở các vật dày hơn 33 mm (bộ lọc ở mục 4.7.2).

**Thư viện gripper.py** được kịch bản gắp tự động gọi. Các hàm chính:

TABLE Bảng 4.11. Các hàm của thư viện gripper.py
| Hàm | Chức năng |
| --- | --- |
| `status()` | Đọc trạng thái servo: vị trí, nhiệt độ, điện áp, giữ lực |
| `move(p, ms)` | Kiểm tra nhiệt độ, chuyển về chế độ vị trí, bật giữ lực, gửi lệnh 1, chờ tới đích hoặc bị chặn |
| `open()` | `move(1000)`: mở hết |
| `grip_measured(e, 5°)` | Kẹp theo bề dày vật đo từ camera (mục 4.5.7) |
| `free()` | Tắt giữ lực: hai má lỏng, servo nguội dần |

Hàm `move` theo dõi vị trí servo mỗi 50 ms. Servo được coi là đã tới đích khi lệch không quá 3 đơn vị, và bị vật chặn khi vị trí không đổi quá 1 đơn vị trong 6 lần đọc liên tiếp. Các hàm của thư viện không phụ thuộc vào kênh truyền: trong các thực nghiệm ở Chương 5, thư viện gửi lệnh qua cổng HTTP chẩn đoán của firmware; khi dùng micro-ROS, chỉ lớp truyền bên dưới được thay bằng topic, còn thuật toán kẹp giữ nguyên. Vì Wi-Fi tới ESP32 có lúc rất chậm (mục 4.5.9), mọi yêu cầu đều được thử lại tối đa 3 lần, mỗi lần chờ tối đa 6 s. Các lệnh ghi được thiết kế để gửi lặp không đổi kết quả (đặt vị trí, bật hoặc tắt giữ lực), nên thử lại là an toàn.

### 4.5.7 Thuật toán kẹp

Ở chế độ vị trí, servo là một vòng điều khiển vị trí: mô-men sinh ra tăng theo sai lệch giữa vị trí đích và vị trí thực. Khi hai má bị vật chặn, **sai lệch này quyết định lực kẹp**. Đặt đích đúng tại điểm chạm thì lực gần bằng 0. Đặt đích sâu hơn điểm chạm thì lực tăng, nhưng dòng điện và nhiệt độ của servo cũng tăng. Nhóm đã thử ba cách kẹp:

1. **Khép hết, giữ ngay sau điểm chạm:** khép về vị trí đóng, phát hiện điểm bị chặn, rồi đặt đích vượt điểm chạm 15 đơn vị (khoảng 1 mm). Lực kẹp quá nhỏ.
2. **Kẹp tới bề rộng đo được trừ 5 mm:** dùng bề dày phần vật giữa hai má (công thức (4.2) ở mục 4.7.2) quy đổi sang vị trí servo theo (4.1). Cả khay và giá đều trượt: servo tới đích mà không bị cản, tức hai má chưa ép vào vật. Nguyên nhân là bề rộng đo từ camera và phép quy đổi tuyến tính đều có thể lệch vài milimét.
3. **Theo góc tính từ camera, cộng thêm 5°** (cách đang dùng, Hình 4.6): tính vị trí servo lúc chạm từ bề dày đo được, đặt đích khép thêm 5° (khoảng 21 đơn vị). Nếu servo tới đích mà chưa bị chặn, nghĩa là bề dày đo lệch, thì tiếp tục khép tới khi bị chặn rồi mới cộng thêm 5°. Khi đã ở sát cuối hành trình thì không ép thêm, vì lúc đó chỉ làm servo nóng.

![Hình 4.6. Lưu đồ thuật toán kẹp theo góc tính từ camera](hinh/gripper_luu_do_kep.png|13)

Thực nghiệm cho thấy ngón in 3D bị uốn khi siết: servo có thể khép tới vị trí 3–5 (gần hết hành trình) mà vẫn đang giữ vật. Vì vậy vị trí servo không cho biết có giữ được vật hay không, và hệ thống không dùng nó để quyết định dừng kịch bản.

### 4.5.8 Phương án mở rộng: đo lực kẹp

Để kẹp theo lực thay vì theo vị trí, nhóm đề xuất gắn ở mỗi má một cảm biến lực màng mỏng RFP602, đọc bằng bộ chuyển đổi tương tự – số (ADC) qua một cầu chia áp. Phần mềm của phương án này đã được xây dựng và kiểm chứng đầy đủ trong mô phỏng: một nút giả lập cảm biến RFP602 từ lực tiếp xúc trong Gazebo, cùng bản tin lực của hai má, và thuật toán kẹp theo lực cho hai má khép chậm 5 mm/s, dừng khi cả hai má đo được từ 2 N trở lên (chi tiết và kết quả đo ở mục 5.2.2 và 5.2.3). Phần cứng cảm biến chưa được lắp lên gripper thật, nên phương án này được để lại cho hướng phát triển. Khi có phần cứng, giá trị lực chỉ cần được phát lên một topic ROS 2 cùng định dạng (qua micro-ROS) là thuật toán kẹp theo lực dùng được ngay.

### 4.5.9 Kết quả thử nghiệm tay gắp

TABLE Bảng 4.12. Kết quả đo trên tay gắp
| Đại lượng | Giá trị đo | Nhận xét |
| --- | --- | --- |
| Điện áp tại servo | 13,3–13,6 V | Ổn định trong suốt các thực nghiệm |
| Nhiệt độ khi nghỉ | 34–44 °C | Tăng dần theo thời gian bật nguồn |
| Nhiệt độ khi giữ vật 1 phút | 64 °C | Vượt ngưỡng 60 °C, không mở được bằng lệnh thường |
| Nhiệt độ khi giữ vật khoảng 5 phút | 76–79 °C | Phải nhả lực; sau đó ESP32 có lần mất Wi-Fi |
| Thời gian nguội từ 79 °C xuống 34 °C (nhả lực) | khoảng 5–20 phút | Nguội nhanh hơn khi nhả lực và mở hết |
| Độ trễ một lệnh khi Wi-Fi tốt | 30–100 ms | 6/6 yêu cầu thành công |
| Độ trễ khi Wi-Fi kém | ping 20–760 ms, mất 25–30 % gói | 2/5 yêu cầu quá 6 s |
| Vật gắp và nhấc được | Giá in 3D (phần kẹp dày 55–62 mm) | Nhấc lên 10–20 cm, mang tới hộp |
| Vật kẹp trượt | Khay đen (68 mm) khi kẹp theo bề rộng trừ 5 mm | Đã khắc phục bằng thuật toán thứ ba |

Hai hạn chế lớn của tay gắp hiện tại là servo nóng nhanh khi giữ vật và đường truyền Wi-Fi không ổn định. Hướng khắc phục được đề xuất ở phần Kết luận: giảm lực giữ sau khi kẹp, thêm tản nhiệt, cải thiện đường truyền cho micro-ROS (đặt bộ phát Wi-Fi gần hơn hoặc dùng transport UART/USB có dây), và đưa cảm biến lực vào sử dụng.

## 4.6 Khối hiệu chuẩn

### 4.6.1 Gắn tag và mô phỏng trước

AprilTag 36h11 id 2 được dán trên nắp tròn ở cuối trục J6, phía đối diện mặt bích. Theo URDF, nắp này thuộc khâu `wrist2_link` nên không quay theo J6. Quy trình hiệu chuẩn được thử trước trong Gazebo với tag ảo trên nắp J6, đạt sai số 2–4 mm, sau đó mới chạy trên robot thật.

![Hình 4.7. Tag id 2 trên nắp trục J6 trong mô phỏng Gazebo](hinh/p_gazebo_tag.png|9)

![Hình 4.8. Camera phát hiện tag id 2 trên nắp J6 của robot thật](hinh/p_tag_j6.jpg|13)

### 4.6.2 Thu mẫu có dẫn đường

Vùng nhìn của camera nhỏ, nên các tư thế chọn ngẫu nhiên thường đưa tag ra khỏi ảnh. Công cụ `handeye_guided.py` đề xuất các tư thế quanh tư thế hiện tại sao cho:

- tag nằm trong ảnh, cách mép ít nhất 40 px;
- tag nghiêng 20–40° so với trục camera (nhìn thẳng thì dễ bị lật nghiệm);
- tag cách camera ít nhất 0,33 m (gần hơn thì Astra không đo được sâu);
- tư thế đa dạng về vị trí, độ cao và hướng nghiêng.

Với mỗi tư thế, công cụ lập kế hoạch Pilz PTP, hiển thị đường đi trong RViz và chờ người vận hành gõ `y` (chạy), `s` (bỏ qua) hoặc `q` (dừng). Song song, `tag_check.py` tự ghi một mẫu mỗi khi robot đứng yên khoảng 1 s và thấy tag. Khi chưa có kết quả hiệu chuẩn, tuỳ chọn `--camera-from-tag` ước lượng vị trí camera từ một lần nhìn tag để có điểm xuất phát.

### 4.6.3 Phát hiện camera bị xê dịch

Công cụ `cam_check.py` so vị trí tag theo camera với vị trí tính từ robot và kết quả hiệu chuẩn hiện tại. Lệch dưới 5 mm là đạt, trên 12 mm nghĩa là camera đã bị xê dịch. Trong quá trình làm đề tài, chân đế camera bị chạm và xê dịch ít nhất ba lần (một lần xoay 3,9°, một lần xoay 2,1°, một lần dịch 16 mm). Mỗi lần đều phải hiệu chuẩn lại từ đầu.

### 4.6.4 Mô hình sửa độ sâu

Công cụ `depth_plane_fit.py` lấy trung vị 30 khung depth, chọn các pixel thuộc mặt bàn (loại vật và robot), tính độ sâu thật của từng pixel bằng giao tia với mặt phẳng z = 12 mm, rồi khớp mô hình (3.6). Mô hình được lưu thành file và mọi công cụ đọc depth đều áp dụng nó qua hàm `camera.correct_depth`.

![Hình 4.9. Tỉ số giữa độ sâu đo được và độ sâu thật trên toàn ảnh theo mô hình đã khớp: mép trái đo xa hơn thật khoảng 6 %, mép phải gần hơn khoảng 3 %](hinh/mo_hinh_depth.png|12)

## 4.7 Khối nhận thức

### 4.7.1 Quy trình

Công cụ `grasp_once.py` thực hiện các bước sau. Robot phải ở tư thế HOME để không che camera.

1. Lấy ảnh màu và trung vị của 15 khung depth, áp mô hình sửa độ sâu.
2. Gửi ảnh và câu lệnh tới Gemini, nhận khung bao. Bản miễn phí chỉ cho 20 lần gọi mỗi ngày với mô hình Robotics-ER. Khi hết lượt, hệ thống lần lượt thử `gemini-flash-latest`, `gemini-3.5-flash`, `gemini-3-flash-preview`.
3. MobileSAM tách mặt nạ trong khung bao.
4. Chuyển các pixel của mặt nạ thành đám điểm của vật trong hệ robot. Tính hộp bao 3D đứng: đáy là hình chữ nhật diện tích nhỏ nhất (`minAreaRect`) bao 95 % điểm gần tâm nhất, chiều cao là phân vị 95 của độ cao trừ độ cao mặt bàn.
5. Lấy đám điểm cảnh trong bán kính 20 cm quanh vật, từ mặt bàn lên 25 cm, đưa vào GraspNet.
6. Lọc các tư thế gắp, giữ 15 tư thế tốt nhất, lưu kết quả và ảnh minh hoạ.

### 4.7.2 Lọc tư thế gắp theo gripper thật

GraspNet được huấn luyện cho một gripper hai má tổng quát, nên nhiều tư thế nó đề xuất không thực hiện được với gripper của đề tài. Dựa trên các nguyên lý ở mục 3.5.6, đề tài xây dựng một **phễu lọc** nhiều tầng. Mỗi tầng loại bỏ những tư thế vi phạm một điều kiện, theo thứ tự từ phép kiểm tra rẻ nhất (so sánh một con số) tới đắt nhất (lập kế hoạch chuyển động), để tiết kiệm thời gian tính toán. Bảng 4.13 liệt kê các bộ lọc.

TABLE Bảng 4.13. Các bộ lọc tư thế gắp, ngưỡng và cơ sở
| Tầng | Bộ lọc | Ngưỡng | Cơ sở (mục 3.5.6) |
| --- | --- | --- | --- |
| 1 | Nằm trên vật | Tâm gắp cách điểm gần nhất của vật dưới 20 mm | GraspNet đề xuất cả trên bàn, trên robot, trên vật khác |
| 2 | Không va chạm | Bộ kiểm tra va chạm của GraspNet, voxel 10 mm | Nguyên lý 3: ngón không được cắm vào cảnh |
| 3 | Độ mở tối đa | Dưới 85 mm | Nguyên lý 2, công thức (3.20): gripper mở tối đa 88 mm |
| 4 | Độ mở tối thiểu | Trên 40 mm | Nguyên lý 2: gripper chỉ khép tới khoảng 30 mm; GraspNet thường mở rộng hơn vật khoảng 10 mm |
| 5 | Độ ăn sâu | Dưới 30 mm | Hình học ngón: chỉ 30 mm ngoài cùng của ngón dùng để kẹp |
| 6 | Hai chiều | Thử cả tư thế xoay 180° quanh hướng tiếp cận | Hai má đối xứng nên cả hai chiều đều hợp lệ; chiều nào dễ với tới hơn thì được chọn |
| 7 | Với tới được | MTC tìm được lời giải IK và quỹ đạo không va chạm; đế không quay quá 90° | Nguyên lý 4 |
| 8 | Phần bị kẹp đủ dày | Bề dày e ≥ 33 mm (đo từ đám điểm) | Nguyên lý 2, công thức (3.19): mỏng hơn thì hai má không ép tới |
| 9 | Má không đè lên vật | Dưới 20 điểm của vật nằm ngay chỗ má sẽ đi xuống | Nguyên lý 3: má đè lên vật sẽ va khi tiến vào |
| 10 | Xếp hạng | Chi phí độ khó thấp nhất, công thức (4.3) | Nguyên lý 5 |

Các tầng 1–5 chạy ngay sau GraspNet trong `grasp_once.py`; các tầng 6–10 chạy trong `mtc_pregrasp.py`, vì cần mô hình robot và bộ lập kế hoạch. Hình 4.10 là phễu lọc của một lần gắp thật: từ 133 tư thế GraspNet đề xuất, chỉ còn một tư thế được thực hiện. Có thể thấy tầng 1–2 loại nhiều nhất: phần lớn tư thế GraspNet đề xuất nằm trên mặt bàn hoặc cắm vào vật khác. Các tầng sau loại ít hơn nhưng vẫn cần thiết: thiếu tầng 8 và 9, robot từng kẹp hụt vào phần mỏng của vật và từng đè má lên vật (mục 5.5.2).

![Hình 4.10. Phễu lọc tư thế gắp trên dữ liệu thật. Trái: lần gắp giá trắng nằm ngửa (02/11, 17:38), từ 133 tư thế còn 1 tư thế được thực hiện. Phải: số tư thế còn lại sau từng bộ lọc trong grasp_once ở lần gắp giá trắng đứng (01/11, 21:14)](hinh/loc_grasp_pheu.png|16)

**Cách tính hai bộ lọc 8 và 9.** Hai bộ lọc này cần biết các điểm của vật nằm ở đâu so với hai má, nên được tính trong hệ toạ độ của TCP. Mỗi điểm của vật (lấy từ mặt nạ SAM) được chuyển sang hệ TCP, có toạ độ (x, y, z) với x theo hướng khép má, y theo bề ngang ngón, z theo hướng tiếp cận. Một điểm **nằm giữa hai má** khi |y| < 19 mm (nửa bề ngang ngón), −10 < z < 30 mm (40 mm tính từ đầu ngón vào trong) và |x| < 44 mm (nửa độ mở). Vì hai má khép đối xứng và đẩy vật về giữa, bề dày phần bị kẹp là:

$$ e = max(x) − min(x)  trên các điểm nằm giữa hai má $$ (4.2)

Một điểm **nằm ngay chỗ má đi xuống** khi |y| < 19 mm, z trong cùng khoảng trên và 44 ≤ |x| < 82 mm. Nếu có từ 20 điểm như vậy trở lên, má sẽ đè lên vật khi tiến vào, và tư thế bị loại.

Hình 4.11 minh hoạ hai bộ lọc này với dữ liệu thật của tư thế được chọn ở Hình 4.10. Có 1 400 điểm của giá nằm giữa hai má (vùng xanh), bề dày phần bị kẹp e = 62 mm, lớn hơn ngưỡng 33 mm; không có điểm nào ở chỗ má đi xuống (vùng đỏ). Vì vậy tư thế đạt cả hai bộ lọc.

![Hình 4.11. Các vùng kiểm tra quanh hai má trong hệ toạ độ TCP, với các điểm thật của giá trắng ở tư thế được chọn (02/11, 17:38). Vùng xanh: giữa hai má, dùng để tính bề dày e; vùng đỏ: chỗ má sẽ đi xuống, phải trống](hinh/loc_grasp_vung_ma.png|16)

### 4.7.3 Vật không đo được độ sâu

Khay nhựa đen hấp thụ tia hồng ngoại nên chỉ 15 % số pixel trên khay có giá trị độ sâu. GraspNet không có đám điểm để làm việc. Đề tài bổ sung chế độ `--assume-height`: biết trước chiều cao vật (47 mm), viền mặt nạ SAM được chiếu lên mặt phẳng miệng khay (độ cao mặt bàn cộng 47 mm) bằng cách giao tia từ camera, rồi tính hộp bao từ đó. Robot gắp thẳng từ trên xuống, kẹp ngang cạnh ngắn của hộp bao. Hộp tính được là 101 × 68 × 47 mm, khớp với số đo thật 100 × 67 × 47 mm.

![Hình 4.12. Kết quả nhận thức cho lệnh “the white object”: mặt nạ SAM (tím), các tư thế GraspNet còn lại sau lọc (cam, vàng), tư thế bị loại vì va chạm (xám)](hinh/p_graspnet_gia.jpg|14)

### 4.7.4 Đám mây điểm đưa vào GraspNet

Để hiểu GraspNet “nhìn thấy” gì, các hình dưới đây vẽ lại dữ liệu thật của lần chạy ngày 02/11 lúc 02:16 với lệnh “the white object”. Mọi toạ độ tính bằng milimét trong hệ toạ độ gốc của robot `base_footprint`: trục x và y nằm trên mặt bàn, trục z hướng lên, gốc nằm ở tâm đế robot.

**Bước 1: đám mây điểm của cả bàn.** Mỗi pixel của ảnh độ sâu (đã sửa sai số theo mô hình ở mục 4.6.4) được chuyển thành một điểm 3D bằng công thức (3.5), rồi đổi sang hệ toạ độ robot nhờ kết quả hiệu chuẩn hand-eye. Mỗi điểm mang màu của pixel tương ứng trong ảnh màu. Cả khung hình cho 212 880 điểm. Hình 4.13 vẽ phần thuộc mặt bàn: mặt bàn gỗ ở độ cao z ≈ 12 mm, hộp nhựa vàng cao khoảng 110 mm, cái giá màu trắng (xanh nhạt trong hình) và các vật khác. Những lỗ trống trên mặt bàn là chỗ camera không đo được độ sâu (vùng bị vật che khuất, hoặc vật màu đen).

![Hình 4.13. Đám mây điểm của cả bàn trong hệ toạ độ robot; vòng tròn đỏ đánh dấu vùng được cắt ra để đưa vào GraspNet](hinh/dam_may_ca_ban.png|13)

**Bước 2: cắt vùng quanh vật.** GraspNet không cần cả bàn. Hệ thống chỉ giữ các điểm nằm trong hình trụ bán kính 200 mm quanh tâm vật (tâm tính từ các điểm thuộc mặt nạ SAM), từ mặt bàn lên cao 250 mm. Vùng này còn 61 133 điểm, trong đó 3 821 điểm thuộc vật (tô tím trong Hình 4.14). Giữ lại cả mặt bàn và các vật xung quanh là cần thiết, vì GraspNet dùng chúng để biết tư thế gắp nào sẽ làm ngón chạm bàn hay chạm vật khác. Trước khi vào mạng, đám điểm được lấy mẫu ngẫu nhiên còn 20 000 điểm.

![Hình 4.14. Đầu vào của GraspNet (đám điểm trong bán kính 20 cm quanh vật) và 5 tư thế gắp tốt nhất, mỗi tư thế vẽ thành hình gripper thu nhỏ](hinh/dam_may_graspnet_3d.png|14)

**Bước 3: đọc kết quả.** Mỗi tư thế gắp được vẽ thành hình chữ U: hai cạnh đứng là hai ngón (khoảng cách giữa hai ngón là độ mở), cạnh ngang là lòng bàn tay gripper, và đoạn thẳng nhô ra phía sau là hướng gripper tiến vào. Năm tư thế tốt nhất có điểm số từ 1,15 xuống 0,72, đều có độ mở 75 mm và đều ôm quanh phần thân dày của cái giá. Hình 4.15 chiếu cùng dữ liệu lên hai mặt phẳng để dễ đọc:

- **Nhìn từ trên xuống (x–y):** thấy rõ hai ngón của mỗi tư thế nằm hai bên phần thân màu tím của vật. Tư thế #1 (đỏ) gần tâm vật nhất.
- **Nhìn ngang (x–z):** mặt bàn là dải màu nâu ở z ≈ 12 mm (đường đứt nét), vật nằm ở z ≈ 55–75 mm. Tư thế #1 gần như thẳng đứng, tư thế #5 (xanh dương) nghiêng nhiều. Đầu ngón của mọi tư thế đều nằm trên mặt bàn, nên không tư thế nào làm ngón cắm vào bàn.

![Hình 4.15. Hình chiếu của đầu vào GraspNet và các tư thế gắp: nhìn từ trên xuống (trái) và nhìn ngang (phải)](hinh/dam_may_graspnet_2d.png|16)

Sau bước này, các tư thế còn đi qua bộ lọc theo gripper thật (mục 4.7.2) và bộ chọn tư thế dễ nhất (mục 4.9.2), rồi mới được dùng để lập kế hoạch chuyển động.

## 4.8 Khối vật cản

Công cụ `scene_obstacles.py` đưa các vật nhìn thấy trên bàn vào planning scene của MoveIt dưới dạng hộp, để kế hoạch tránh chúng:

1. Lấy trung vị 15 khung depth đã sửa, chuyển sang hệ robot.
2. Giữ các điểm cao hơn mặt bàn 8 mm tới 30 cm. Ngưỡng 8 mm lớn hơn khoảng ba lần nhiễu mặt bàn.
3. Loại các điểm cách chuỗi khâu của robot dưới 9 cm (lấy vị trí các khâu từ TF), để không coi chính robot là vật cản.
4. Gom các vùng liên thông trên ảnh (bỏ vùng nhỏ hơn 120 pixel). Mỗi vùng thành một hộp đứng: đáy là hình chữ nhật nhỏ nhất bao các điểm cộng thêm lề 1 cm mỗi bên, chiều cao từ mặt bàn tới phân vị 95.
5. Bỏ qua vật đích của lần nhận thức gần nhất, vì gripper phải ôm quanh nó.

Hộp đựng vật được xử lý riêng: hộp đặc do bước 4 sinh ra được thay bằng bốn thành dày 12 mm, để ngón và vật có thể đi vào lòng hộp.

![Hình 4.16. Vật cản camera nhìn thấy (xanh lá) và vật đích (nhãn “dich”); hộp vàng có thành mỏng và bóng nên chỉ đo được một phần](hinh/p_vat_can_hop.jpg|13)

## 4.9 Khối lập kế hoạch và thực thi

### 4.9.1 Tác vụ MTC

Công cụ `mtc_pregrasp.py` dựng tác vụ MTC theo Hình 3.2. Các tham số chính:

TABLE Bảng 4.14. Tham số của tác vụ MTC
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

Độ ăn sâu được giới hạn để góc thấp nhất của ngón (theo mô hình hai hộp ở mục 4.4) luôn cao hơn tấm bàn trong planning scene 3 mm. Điều này tránh trường hợp tư thế gắp nghiêng trên vật thấp làm góc ngón cắm vào bàn.

### 4.9.2 Chọn tư thế gắp

Với mỗi tư thế gắp còn lại sau lọc, hệ thống thử cả hai chiều xoay má (0° và 180°) và bốn hướng thả vào hộp (0°, ±90°, 180° quanh trục đứng), rồi lập kế hoạch cho cả chuỗi. Một kế hoạch bị loại nếu:

- khớp đế j1 phải quay quá 90° so với lúc đầu ở bất kỳ thời điểm nào (lần đầu tiên chưa có giới hạn này, kế hoạch rẻ nhất làm đế xoay khoảng 300°);
- phần bị kẹp mỏng hơn 33 mm, hoặc má đè lên vật (mục 4.7.2).

Trong các kế hoạch còn lại, hệ thống chọn kế hoạch có độ khó nhỏ nhất:

$$ độ khó = góc nghiêng (độ) + lệch tâm vật (mm) + chi phí đường đi (rad) $$ (4.3)

Trước đây hệ thống chỉ chọn theo chi phí đường đi, nên hay chọn cách gắp nghiêng, lệch tâm 50 mm, kẹp vào phần mỏng 17 mm. Công thức (4.3) ưu tiên cách gắp gần thẳng đứng và gần tâm vật, là những cách gắp chắc chắn hơn.

### 4.9.3 Thả vào hộp

Điểm thả được chọn trong lòng hộp, cách thành 11 cm (thành 12 mm cộng tầm vươn ngang 82 mm của ngón), tại điểm gần vật nhất. Lý do là tâm hộp ở vị trí thử nghiệm nằm ở hướng 166° nhìn từ đế robot, sát giới hạn 175° của khớp j1, nên thả đúng tâm thì robot phải lật tay.

Ban đầu, vật được thả khi đáy vật cao hơn miệng hộp 8 cm, tức là rơi tự do. Người vận hành yêu cầu đặt sâu vào hộp. Đề tài thử hạ vật xuống cách đáy hộp 2 cm, nhưng lần nào thân gripper cũng va vào thành hộp. Cách dung hoà là hạ tới khi thân gripper vừa ở trên miệng hộp: TCP cao hơn miệng hộp 5 mm, lúc đó ngón dài 60 mm và vật đã nằm trong lòng hộp.

### 4.9.4 Thực thi có kiểm tra

Lời giải MTC gồm 7 đoạn quỹ đạo. Hệ thống không gọi thực thi cả lời giải một lần, mà gửi từng đoạn qua action `/execute_trajectory` của MoveIt để chèn thao tác gripper và các bước kiểm tra vào giữa:

1. Mở gripper. Đi tới điểm chờ.
2. Đọc TCP thật từ TF. Nếu lệch khỏi đường tiến vào quá 10 mm thì dừng.
3. Tiến vào. Đọc TCP. Nếu lệch đích quá 10 mm thì không kẹp.
4. Kẹp theo thuật toán ở mục 4.5.7.
5. Nhấc lên, mang tới hộp, hạ xuống.
6. Mở gripper. Nếu servo quá nóng thì nhả lực để vật rơi vào hộp.
7. Nhấc ra khỏi hộp, về HOME.

Người vận hành chỉ xác nhận một lần trước bước 1, sau khi đã xem toàn bộ đường đi trong RViz. Tuỳ chọn `--step` cho phép hỏi lại trước từng bước.

## 4.10 Ghi dữ liệu và kịch bản

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

## 4.11 Các biện pháp an toàn

- Robot thật và mô phỏng ở hai mạng ROS khác nhau.
- Tốc độ khớp giới hạn ở 11 % tốc độ tối đa; kịch bản chạy ở hệ số 0,2–0,3 của mức đó.
- Mọi chuyển động đều hiện trong RViz trước và chỉ chạy khi người vận hành gõ `y`.
- Tự dừng khi TCP lệch quá 10 mm so với kế hoạch.
- Gripper không di chuyển khi từ 60 °C trở lên (kiểm tra ở cả phần mềm và firmware).
- Người vận hành luôn đứng cạnh nút dừng khẩn cấp. Trong đề tài, nút này đã được dùng hai lần, đều trước khi có chuyển động gây hư hại.
