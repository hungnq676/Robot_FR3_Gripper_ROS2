# CHƯƠNG 2. ROBOT FAIRINO FR3 VÀ NỀN TẢNG ROS 2

Chương này trình bày nội dung nghiên cứu thứ nhất của đề tài: tìm hiểu và vận hành thử nghiệm cánh tay robot FAIRINO FR3 qua ROS 2. Phần đầu giới thiệu robot, bộ điều khiển và các chế độ vận hành. Phần sau giới thiệu ROS 2, MoveIt 2, RViz và Gazebo, cách chúng được nối với robot thật, và các vấn đề gặp phải khi vận hành robot qua ROS 2.

## 2.1 Giới thiệu robot FAIRINO FR3

### 2.1.1 Tổng quan

FAIRINO FR3 là tay máy cộng tác (cobot) cỡ nhỏ của hãng FAIRINO (Trung Quốc), thuộc dòng FR gồm FR3, FR5, FR10, FR16 và FR20 (con số chỉ tải trọng tính bằng kg). Robot được thiết kế để làm việc gần con người: tốc độ và lực bị giới hạn, có phát hiện va chạm, có thể kéo tay để dạy (drag teaching), và có thể lập trình qua giao diện web trên máy tính bảng mà không cần kiến thức lập trình sâu.

Trong đề tài, FR3 được gắn cố định cạnh bàn thí nghiệm. Đầu robot (mặt bích J6) được gắn gripper hai ngón do nhóm thiết kế (mục 4.5). Camera RGB-D được đặt cố định phía trên bàn (mục 4.2.2).

TABLE Bảng 2.1. Thông số kỹ thuật chính của FAIRINO FR3
| Thông số | Giá trị |
| --- | --- |
| Số bậc tự do | 6 khớp quay |
| Tải trọng | 3 kg |
| Tầm với | 622 mm (tính từ trục khớp 1) |
| Độ lặp lại | ±0,02 mm |
| Tốc độ khớp tối đa | khoảng 3,15 rad/s (180°/s) |
| Giao tiếp với máy tính | Ethernet TCP/IP, địa chỉ mặc định của bộ điều khiển 192.168.58.2 |
| Lập trình | Giao diện web trên máy tính bảng (teach pendant), SDK C++/Python, driver ROS 2 |
| Tính năng an toàn | Phát hiện va chạm, giới hạn mềm từng khớp, nút dừng khẩn cấp |

### 2.1.2 Cấu trúc động học và giới hạn khớp

FR3 có cấu trúc 6 khớp quay giống các cobot phổ biến: ba khớp đầu (vai, cánh tay, khuỷu) quyết định vị trí cổ tay, ba khớp cuối (cổ tay) quyết định hướng của dụng cụ. Giới hạn góc của từng khớp được đọc từ mô hình URDF chính thức của hãng:

TABLE Bảng 2.2. Giới hạn góc các khớp của FR3 (theo URDF)
| Khớp | Tên khâu sau khớp | Giới hạn dưới | Giới hạn trên |
| --- | --- | --- | --- |
| j1 (đế) | shoulder_link | −175° | +175° |
| j2 (vai) | upperarm_link | −265° | +85° |
| j3 (khuỷu) | forearm_link | −162° | +162° |
| j4 (cổ tay 1) | wrist1_link | −265° | +85° |
| j5 (cổ tay 2) | wrist2_link | −175° | +175° |
| j6 (cổ tay 3) | wrist3_link | −175° | +175° |

Giới hạn ±175° của khớp đế j1 ảnh hưởng lớn tới vùng làm việc. Robot không quay được trọn vòng, nên với các vật nằm ở hướng gần ±180° nhìn từ đế, robot phải “lật tay” sang cấu hình khác, khớp đế quay 160° hoặc hơn. Vấn đề này được phân tích chi tiết ở mục 5.6.

### 2.1.3 Bộ điều khiển và teach pendant

Bộ điều khiển của FR3 là một hộp điều khiển riêng, nối với robot bằng cáp và nối với máy tính bằng Ethernet. Người vận hành tương tác với robot qua giao diện web chạy trên máy tính bảng (teach pendant) hoặc trình duyệt. Giao diện này cho phép:

- bật/tắt cấp nguồn cho các khớp (enable);
- điều khiển từng khớp hoặc điều khiển theo toạ độ Cartesian (jog);
- kéo tay robot để dạy điểm (drag teaching);
- xem và xoá lỗi;
- đặt tốc độ toàn cục, giới hạn mềm, vùng an toàn;
- viết và chạy chương trình bằng ngôn ngữ của hãng.

Robot có hai chế độ: **chế độ tay** (manual), dùng khi dạy và chạy thử, và **chế độ tự động** (automatic), dùng khi chạy chương trình. Trong đề tài, robot được điều khiển từ ROS 2 qua SDK ở chế độ tay. Thực nghiệm cho thấy chế độ không phải nguyên nhân làm robot không chuyển động (mục 2.6.1).

### 2.1.4 Các mã lỗi thường gặp

Trong quá trình vận hành, nhóm đã gặp và ghi lại các mã lỗi sau. Đây là những tình huống mà một người mới vận hành FR3 qua ROS 2 rất dễ gặp:

TABLE Bảng 2.3. Các mã lỗi đã gặp khi vận hành FR3 qua ROS 2
| Mã | Ý nghĩa | Tình huống gặp | Cách xử lý |
| --- | --- | --- | --- |
| ServoJ 14 | Bộ điều khiển đang ở trạng thái lỗi | Khớp 5 vượt giới hạn mềm khi đo gripper ở tư thế duỗi thẳng (lỗi 1/17) | Xoá lỗi trên giao diện web, đưa khớp về trong giới hạn |
| ServoJ 99 | Robot đã dừng | Người vận hành bấm dừng khẩn cấp, hoặc robot phát hiện va chạm | Nhả dừng khẩn cấp, xoá lỗi, khởi động lại driver |
| ServoJ −2 | Mất kết nối SDK | Sau dừng khẩn cấp, hoặc cáp mạng chập chờn | Khởi động lại driver |
| err 1/17 | Khớp vượt giới hạn mềm (khớp 5) | Khớp j5 khoảng −180° | Đưa khớp về bằng tay trên giao diện |

## 2.2 Hệ điều hành robot ROS 2

### 2.2.1 Khái niệm cơ bản

ROS 2 (Robot Operating System 2) [7] không phải là một hệ điều hành theo nghĩa thông thường, mà là một bộ thư viện và công cụ chạy trên Linux để xây dựng phần mềm robot. Một hệ thống ROS 2 gồm nhiều chương trình nhỏ gọi là **nút** (node), mỗi nút làm một việc, và các nút trao đổi dữ liệu với nhau qua ba cơ chế:

- **Topic** (chủ đề): kênh phát – nhận liên tục. Ví dụ, driver camera phát ảnh lên topic `/camera/color/image_raw` 30 lần mỗi giây, nút nhận thức đăng ký nhận ảnh từ topic đó.
- **Service** (dịch vụ): hỏi – đáp một lần. Ví dụ, gọi `/compute_ik` của MoveIt để tính động học ngược cho một tư thế.
- **Action** (hành động): yêu cầu dài có phản hồi tiến độ và có thể huỷ. Ví dụ, gửi quỹ đạo tới `/execute_trajectory` để robot thực hiện.

Ngoài ra, ROS 2 có **tham số** (parameter) để cấu hình từng nút, và **TF2** để quản lý cây hệ toạ độ. TF2 đặc biệt quan trọng với đề tài: mọi vị trí (camera, vật, gripper) đều được quy về hệ toạ độ gốc của robot `base_footprint` thông qua cây TF.

Đề tài dùng ROS 2 Humble trên Ubuntu 22.04. Robot thật dùng `ROS_DOMAIN_ID=33`, mô phỏng dùng 34, để hai hệ không nhìn thấy nhau. Quyết định này được đưa ra sau khi một lần khởi động mô phỏng đã tắt nhầm MoveIt của robot thật.

### 2.2.2 ros2_control và driver của FR3

ros2_control [12] là khung điều khiển phần cứng chuẩn của ROS 2. Nó gồm hai lớp:

- **Controller** (bộ điều khiển phần mềm): ở đây là `joint_trajectory_controller`, nhận một quỹ đạo (danh sách các điểm khớp theo thời gian) và nội suy ra lệnh vị trí ở mỗi chu kỳ 8 ms. Controller cũng kiểm tra robot có bám quỹ đạo không: nếu lệch quá 0,05 rad ở bất kỳ khớp nào, quỹ đạo bị huỷ.
- **Hardware interface** (giao diện phần cứng): ở đây là `fairino_hardware` trong driver `frcobot_ros2` (bản V3.8.3) của hãng. Ở mỗi chu kỳ, nó đọc góc khớp thật từ bộ điều khiển và gửi lệnh vị trí mới bằng hàm `ServoJ` của SDK `libfairino`.

Lệnh khởi động `run_real.sh` do đề tài viết lần lượt: kiểm tra kết nối tới bộ điều khiển, phát TF của camera từ kết quả hiệu chuẩn, khởi động ros2_control với driver FR3, khởi động MoveIt 2 và RViz.

## 2.3 MoveIt 2

MoveIt 2 [6] là bộ thư viện lập kế hoạch chuyển động phổ biến nhất trên ROS 2. Thành phần trung tâm là nút `move_group`, cung cấp:

- **Planning scene**: mô hình robot, trạng thái hiện tại, và các vật thể va chạm trong môi trường.
- **Bộ giải động học ngược** (KDL) cho nhóm khớp `arm`.
- **Các bộ lập kế hoạch**: OMPL (ngẫu nhiên, ví dụ RRTConnect) và Pilz Industrial Motion Planner (PTP, LIN, CIRC).
- **Thực thi quỹ đạo**: gửi quỹ đạo đã lập tới controller qua action.

Cấu hình MoveIt cho FR3 (gói `fairino_fr3_moveit_config`) gồm: file SRDF định nghĩa nhóm khớp `arm` (j1…j6), nhóm `gripper`, tư thế đặt tên, và các cặp khâu bỏ qua kiểm tra va chạm; file `joint_limits_real.yaml` giới hạn tốc độ khớp ở 0,35 rad/s (khoảng 11 % tốc độ tối đa) cho robot thật; file `pilz_cartesian_limits_real.yaml` giới hạn tốc độ đường thẳng ở 0,15 m/s.

Trong thực nghiệm, OMPL sinh ra đường đi có khúc gấp làm robot khựng giữa đường, vì là thuật toán lấy mẫu ngẫu nhiên. Pilz PTP sinh quỹ đạo tuyến tính trong không gian khớp, xác định và mượt, nên được chọn làm mặc định cho robot thật.

## 2.4 Giao diện hiển thị RViz

RViz là công cụ hiển thị 3D của ROS. Trong đề tài, RViz là nơi người vận hành **nhìn thấy mọi thứ hệ thống đang “nghĩ”**: robot đang ở đâu, camera đặt ở đâu, vật cản nằm đâu, vật cần gắp và các cách gắp được đề xuất, và robot định đi đường nào. Mọi chuyển động của robot thật đều được xem trước trong RViz rồi mới cho chạy.

Hình 2.1 là giao diện RViz khi hệ thống đang chạy với robot thật. Màn hình chia làm ba vùng: bảng Displays ở trên bên trái, bảng MotionPlanning ở dưới bên trái, và cảnh 3D ở bên phải. Ba hình tiếp theo phóng to từng vùng.

![Hình 2.1. Toàn cảnh giao diện RViz khi vận hành robot thật](hinh/rviz_toan_canh.png|16)

### 2.4.1 Bảng Displays

Bảng Displays (Hình 2.2) liệt kê các **lớp hiển thị**. Mỗi lớp đọc dữ liệu từ một topic ROS và vẽ nó vào cảnh 3D. Ô đánh dấu bên phải dùng để bật hoặc tắt từng lớp.

![Hình 2.2. Bảng Displays: danh sách các lớp hiển thị](hinh/rviz_displays.png|15)

TABLE Bảng 2.4. Các lớp hiển thị trong RViz của đề tài
| Lớp | Dữ liệu | Ý nghĩa |
| --- | --- | --- |
| Global Options, Global Status | Cấu hình chung | Hệ toạ độ gốc của cảnh (`base_footprint`), trạng thái chung (Ok nghĩa là mọi lớp đều nhận được dữ liệu) |
| Grid | – | Lưới nền giúp ước lượng kích thước, mỗi ô 1 m |
| Demo Object Marker | `/fr3_demo/object_cloud` | Đám điểm 3D của vật cần gắp do khối nhận thức tách ra (mục 4.7) |
| Grasp Candidates | `/fr3_demo/grasp_markers` | Các tư thế gắp GraspNet đề xuất, vẽ thành hình gripper thu nhỏ, có đánh số #1, #2… theo điểm số |
| Motion Planning Tasks | Thông tin của MTC | Cây giai đoạn của tác vụ MTC và các lời giải (mục 3.6.3) |
| MotionPlanning | Planning scene của MoveIt | Mô hình robot theo góc khớp thật, các vật thể va chạm, tư thế đích và đường đi đã lập |

### 2.4.2 Bảng MotionPlanning

Bảng MotionPlanning (Hình 2.3) dùng để lập kế hoạch và điều khiển robot trực tiếp từ RViz. Bảng có các thẻ:

![Hình 2.3. Bảng MotionPlanning, thẻ Planning](hinh/rviz_motionplanning.png|15)

- **Context:** chọn bộ lập kế hoạch. Với robot thật phải chọn Pilz, thuật toán **PTP**. Nếu để trống, RViz gửi tên bộ lập kế hoạch rỗng và Pilz báo lỗi.
- **Planning** (thẻ đang mở trong hình):
    - *Commands:* nút **Plan** lập kế hoạch và hiển thị đường đi; nút **Execute** cho robot chạy theo kế hoạch vừa lập; **Plan & Execute** làm cả hai liền nhau; **Stop** dừng robot.
    - *Query:* chọn nhóm khớp (`arm`), trạng thái bắt đầu (`<current>` là tư thế hiện tại của robot thật) và trạng thái đích. Trạng thái đích có thể chọn từ các tư thế đặt tên, hoặc kéo quả cầu điều khiển trong cảnh 3D.
    - *Options:* thời gian lập kế hoạch tối đa (5 s), số lần thử (10), hệ số tốc độ và gia tốc (0,9 nghĩa là 90 % của giới hạn đã đặt, tức khoảng 10 % tốc độ tối đa của FR3), và các tuỳ chọn như đi đường thẳng Cartesian, giải động học ngược có kiểm tra va chạm.
- **Joints:** xem và kéo từng khớp để đặt tư thế đích.
- **Scene Objects:** danh sách các vật thể va chạm trong planning scene: mặt bàn `robot_table`, các hộp `vat_1`, `vat_2`… do khối vật cản sinh ra (mục 4.8), bốn thành hộp `hop_thanh_1…4`.
- **Stored Scenes, Stored States:** lưu và nạp lại cảnh hoặc tư thế.
- **Status, Manipulation:** trạng thái của `move_group` và các chức năng gắp có sẵn của MoveIt. Đề tài không dùng chức năng gắp có sẵn mà dùng MTC.

### 2.4.3 Cảnh 3D

Cảnh 3D (Hình 2.4) là phần quan trọng nhất. Các đối tượng trong cảnh:

![Hình 2.4. Cảnh 3D trong RViz: robot, mặt bàn, vật cản, trục camera và các tư thế gắp được đề xuất](hinh/rviz_canh_3d.png|14)

1. **Robot màu cam:** mô hình robot FR3 kèm gripper, vẽ theo góc khớp thật đọc từ robot. Nếu robot thật di chuyển, mô hình này di chuyển theo.
2. **Robot màu xám mờ:** tư thế đích hoặc đường đi đã lập. Khi bật *Planned Path*, robot mờ chạy lại toàn bộ đường đi để người vận hành kiểm tra trước. Quả cầu và các vòng màu xanh, đỏ quanh cổ tay là bộ điều khiển kéo – xoay để đặt tư thế đích bằng chuột.
3. **Mặt bàn màu xanh lá:** vật thể va chạm `robot_table`. MoveIt từ chối mọi kế hoạch làm robot hoặc gripper chạm vào nó.
4. **Các hộp màu xanh lá trên mặt bàn:** vật cản do khối vật cản sinh ra từ ảnh độ sâu, mỗi hộp bao một vật nhìn thấy trên bàn.
5. **Bộ ba trục đỏ – xanh lá – xanh dương phía trên:** hệ toạ độ của camera sau hiệu chuẩn hand-eye (trục xanh dương là hướng nhìn của camera). Nếu camera bị xê dịch mà chưa hiệu chuẩn lại, bộ trục này sẽ lệch khỏi vị trí thật.
6. **Các đường màu vàng, đỏ có nhãn “GraspNet”, “#1”:** các tư thế gắp được đề xuất. Tư thế tốt nhất có màu đậm nhất.

Một tình huống thực tế cho thấy giá trị của RViz: khi kế hoạch bị từ chối với thông báo va chạm giữa `base_link_gripper` và `robot_table`, nhìn vào cảnh 3D thấy ngay gripper đang chạm mặt bàn ở tư thế hiện tại. Người vận hành chỉ cần nâng robot lên vài centimet là lập được kế hoạch.

## 2.5 Mô phỏng với Gazebo

Trước khi chạy trên robot thật, toàn bộ chuỗi phần mềm được kiểm tra trong Gazebo Harmonic. Mô hình mô phỏng gồm robot FR3, bàn, camera OAK-D Pro W dựng theo thông số hãng (góc nhìn 95° × 72°) và các vật thử nghiệm. Gazebo dùng chung URDF và cấu hình MoveIt với robot thật, chỉ khác giao diện phần cứng (mô phỏng thay cho `fairino_hardware`). Nhờ vậy, một tác vụ chạy được trong mô phỏng có thể chuyển sang robot thật mà không sửa mã.

Mô phỏng cũng được dùng để thử quy trình hiệu chuẩn trước: một AprilTag ảo được đặt lên nắp trục J6 trong Gazebo, và quy trình hiệu chuẩn đạt sai số 2–4 mm trước khi chạy trên robot thật (Hình 4.7). Toàn bộ kết quả mô phỏng, gồm cả cảm biến lực ở hai má kẹp và phần so sánh giữa mô phỏng và thực tế, được trình bày ở mục 5.2.

## 2.6 Vận hành thử nghiệm qua ROS 2 và các vấn đề gặp phải

### 2.6.1 Robot không bám quỹ đạo

Khi chạy MoveIt trên robot thật, robot thường giật một cái rồi dừng. Log báo `Aborted due to state tolerance violation`, lệch 0,05 rad ở một khớp. Có lần robot hoàn toàn đứng yên mà driver không báo lỗi gì.

Nhóm đã thử ba giả thuyết theo thứ tự, mỗi lần chỉ thay đổi một yếu tố:

TABLE Bảng 2.5. Các giả thuyết đã thử khi robot không bám quỹ đạo
| Giả thuyết | Cách thử | Kết quả |
| --- | --- | --- |
| Chế độ tay làm robot chậm | Chuyển sang chế độ tự động qua SDK | Sai: ở chế độ tự động robot vẫn đứng yên |
| Cú dừng gắt khi controller ngắt | Thêm bộ lọc giới hạn gia tốc vào driver | Không giúp, có lần còn làm robot đứng yên; đã gỡ bỏ |
| Tốc độ toàn cục dưới 100 % | Đặt tốc độ 100 % qua SDK trước khi bật driver | Trông như đúng nhưng là kết luận sai: robot chạy lại được vì driver được khởi động lại |

### 2.6.2 Nguyên nhân: hàng đợi lệnh ServoJ

Nhóm thêm vào driver phần đọc trạng thái bộ điều khiển (chỉ đọc, mỗi giây một lần) và phát hiện bộ điều khiển xếp hàng các lệnh `ServoJ` rồi thực hiện lần lượt, mỗi lệnh 8 ms. Máy tính gửi khoảng 125 lệnh mỗi giây nhưng không phải hệ thời gian thực: chu kỳ gửi đo được trung bình 8,01 ms, lệch chuẩn 0,32 ms, ngắn nhất 4,8 ms, dài nhất 11 ms. Mỗi khi máy tính gửi nhanh hơn 8 ms, hàng đợi tăng thêm một lệnh và không bao giờ giảm. Số đo trong log: hàng đợi 15 lệnh (robot chậm 0,12 s so với kế hoạch), sau đó 123–124 lệnh (chậm khoảng 1 s). Ở tốc độ 18°/s, chậm 0,12 s đã gần đủ để vượt ngưỡng 2,9°.

Nguyên nhân này giải thích được mọi hiện tượng: khởi động lại driver thì hàng đợi trống nên chạy tốt, chạy một lúc thì hỏng, và chạy càng nhanh càng dễ hỏng.

### 2.6.3 Cách sửa và kết quả

Driver được sửa để chỉ gửi `ServoJ` khi hàng đợi của bộ điều khiển có tối đa 3 lệnh. Sau khi sửa, hàng đợi luôn ở mức 0–3 lệnh. Trong 31 lần chạy được ghi lại sau đó, 22 lần có độ lệch dưới 1,5°, 8 lần có độ lệch 1,5–2,8°, và 1 lần bất thường chưa giải thích được. Độ lệch còn lại xấp xỉ tốc độ nhân 0,1 s, tức là độ trễ bên trong robot.

![Hình 2.5. Hàng đợi lệnh ServoJ trước và sau khi sửa driver, và thống kê 31 lần chạy sau khi sửa](hinh/hang_doi_servoj.png|15)

### 2.6.4 Các vấn đề cài đặt khác

TABLE Bảng 2.6. Các vấn đề cài đặt khi dựng môi trường ROS 2
| Vấn đề | Nguyên nhân | Cách xử lý |
| --- | --- | --- |
| RViz không nạp được mô hình robot | Máy dùng định dạng số tiếng Việt (dấu phẩy thập phân), MoveIt đọc giới hạn khớp “3.15” thành chuỗi chữ | Đặt `LC_NUMERIC=en_US.UTF-8` trong mọi script |
| Python của conda chen vào ROS | Môi trường conda tự kích hoạt | Tắt tự kích hoạt; script ROS loại conda khỏi `PATH` |
| OpenCV 4.13 xung đột | Đòi numpy 2, ROS Humble dùng numpy 1 | Dùng OpenCV 4.10 |
| MoveIt vẫn dùng gripper mô phỏng | File khởi động nạp URDF với `hardware_type: mock` | Truyền đúng tham số `hardware_type: fairino` cho robot thật |

## 2.7 Kết luận chương

Chương này đã giới thiệu robot FAIRINO FR3, hệ sinh thái ROS 2 – MoveIt 2 – RViz – Gazebo, và cách nhóm vận hành robot thật qua ROS 2. Kết quả quan trọng nhất là tìm ra và sửa nguyên nhân robot không bám quỹ đạo (hàng đợi lệnh ServoJ), giúp robot chạy ổn định qua MoveIt với độ lệch dưới 1,5° trong phần lớn các lần chạy. Đây là nền tảng cho các chương sau.
