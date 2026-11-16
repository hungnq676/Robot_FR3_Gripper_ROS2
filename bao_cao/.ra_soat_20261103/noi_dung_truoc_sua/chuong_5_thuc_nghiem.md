# CHƯƠNG 5. THỰC NGHIỆM VÀ ĐÁNH GIÁ

## 5.1 Môi trường và phương pháp thực nghiệm

Thực nghiệm được tiến hành từ ngày 23/10/2026 đến ngày 02/11/2026 trên hai máy tính: máy hungros2 (phát triển ban đầu và mô phỏng) và máy ROG Strix G712LU (robot thật). Bàn thử nghiệm là bàn gỗ 100 × 50 cm đặt cạnh đế robot. Vật thử nghiệm gồm: khối gỗ đỏ cạnh 30 mm, cốc giấy, tay cầm game màu đen, cục sạc điện thoại, khay nhựa đen 100 × 67 × 47 mm, giá in 3D màu trắng 140 × 60 × 58 mm và một hộp nhựa vàng khoảng 310 × 240 × 112 mm.

Mỗi chức năng được thử theo hai bước: chạy trong mô phỏng Gazebo trước (mục 5.2), rồi mới chạy trên robot thật (mục 5.3–5.6). Phương pháp thực nghiệm tuân theo ba nguyên tắc được rút ra trong quá trình làm:

- **Mỗi lần thử chỉ thay đổi một yếu tố.** Thay đổi nhiều thứ cùng lúc từng dẫn tới quy nhầm nguyên nhân (mục 2.6.1).
- **Dùng phép đo vật lý làm chuẩn, kiểm tra chéo bằng dữ liệu.** Ví dụ: chạm mặt bàn để biết độ cao bàn; đo khe bằng thước; so quãng đường tag di chuyển theo robot và theo camera.
- **Ghi nhật ký có thời gian cho mọi lần chạy**, kể cả lần hỏng, để không lặp lại các giả thuyết đã bị bác bỏ.

## 5.2 Thực nghiệm mô phỏng

Mô phỏng là phần đề tài làm được nhiều nhất và đầy đủ nhất. Trong mô phỏng, nhóm chạy được trọn chuỗi từ câu lệnh bằng lời tới khi vật được mang đi, kể cả những thành phần mà phần cứng thật chưa có, như cảm biến lực ở hai má kẹp. Mục này trình bày lần lượt: mô hình mô phỏng (5.2.1), cảm biến lực mô phỏng (5.2.2), một lần gắp trọn vẹn bằng lệnh nói (5.2.3), hiệu chuẩn trong mô phỏng (5.2.4), và quan trọng nhất là so sánh mô phỏng với thực tế: mô phỏng làm được gì, thiếu gì, và thực tế khác ra sao (5.2.5). Cuối cùng là lần đầu chuyển từ mô phỏng sang robot thật (5.2.6).

Các số liệu trong mục 5.2.2 và 5.2.3 được ghi lại từ một lần chạy mô phỏng ngày 02/11/2026: lực ở hai má được ghi ở tần số 100 Hz, ảnh Gazebo được chụp khoảng 2 giây một lần, và mốc thời gian từng giai đoạn được lấy từ nhật ký của bộ lập kế hoạch MTC.

### 5.2.1 Mô hình mô phỏng

Mô phỏng dùng Gazebo Harmonic, chạy chung với ROS 2 Humble và MoveIt 2. Điểm quan trọng nhất là **mô phỏng và robot thật dùng chung một mô tả robot (URDF/Xacro), chung cấu hình MoveIt và chung mã lập kế hoạch**. Chỉ có phần giao tiếp phần cứng là khác: trong mô phỏng, `gz_ros2_control` thay cho driver `fairino_hardware`. Nhờ vậy, mã đã chạy đúng trong mô phỏng có thể đưa sang robot thật mà không phải sửa.

Mô phỏng chạy trên một mạng ROS riêng (`ROS_DOMAIN_ID=34`, robot thật dùng 33) và chỉ trong máy (`ROS_LOCALHOST_ONLY=1`). Như vậy không thể có chuyện một lệnh của mô phỏng đi nhầm tới robot thật. Toàn bộ hệ thống được khởi động bằng một script, script này mở 6 cửa sổ theo thứ tự và chờ bước trước sẵn sàng rồi mới chạy bước sau: Gazebo và bộ điều khiển → MoveIt và RViz → mặt bàn → nhận diện vật → MTC → Gemini.

TABLE Bảng 5.1. Các thành phần của mô hình mô phỏng
| Thành phần | Mô hình trong Gazebo | Ghi chú |
| --- | --- | --- |
| Robot | FAIRINO FR3 6 khớp, lưới hình (mesh) của hãng | Đế gắn cố định lên bàn; 3 bộ điều khiển: trạng thái khớp, cánh tay, gripper |
| Gripper | Hai má song song đối xứng, mỗi má chạy tối đa khoảng 17 mm | Mỗi má có một cảm biến tiếp xúc (contact sensor) 200 Hz |
| Cảm biến lực | Hai cảm biến màng mỏng RFP602 giả lập từ lực tiếp xúc | Chi tiết ở mục 5.2.2 |
| Camera | OAK-D Pro W theo thông số hãng: ảnh màu 1280 × 960, góc nhìn 95° × 72°, đo sâu 0,2–12 m, nhiễu 5 mm | Đặt phía trên, nhìn xiên xuống bàn; tên topic giống driver thật |
| Bàn | Hộp 1,0 m, mặt bàn cao 1,0 m | Có tấm chắn ảo trong MoveIt để robot không đi xuyên bàn |
| Vật thử | Hộp xanh, chai nước, chuột máy tính (mô hình quét 3D), bút, thùng các-tông | Mỗi vật gắn được vào gripper khi kẹp (mục 5.2.3) |
| Mốc hiệu chuẩn | AprilTag 36h11 id 0 cố định trên bàn; tag id 2 trên nắp trục J6 | Dùng thử hai quy trình hiệu chuẩn (mục 5.2.4) |

![Hình 5.1. Toàn cảnh mô hình mô phỏng trong Gazebo: robot FR3 gắn gripper hai má, bàn, các vật thử và tag hiệu chuẩn](hinh/mo_phong/toan_canh.png|14)

![Hình 5.2. Các vật thử trên bàn mô phỏng: tag hiệu chuẩn, bút, chai nước, hộp xanh và chuột máy tính](hinh/mo_phong/vat_tren_ban.png|14)

Camera ảo cho cả ảnh màu và ảnh độ sâu giống như camera thật (Hình 5.3). Khác biệt lớn nhất nằm ở chất lượng ảnh độ sâu: trong mô phỏng, mọi bề mặt đều đo được, mặt bàn đều màu và các vật nhô lên rõ ràng, nhiễu chỉ khoảng 5 mm. Camera thật cho nhiễu dạng ô caro ±23 mm, sai số lệch khoảng 17 mm và không đo được vật màu đen (so với Hình 5.15 ở mục 5.3.4). Đây là lý do một số phương pháp chạy tốt trong mô phỏng lại không dùng được trên camera thật (mục 5.2.5).

![Hình 5.3. Camera ảo OAK-D Pro W trong mô phỏng: ảnh màu (trái) và ảnh màu phủ độ sâu 0,8–2,0 m (phải). Độ sâu đầy đủ và mượt trên mọi bề mặt, khác hẳn camera thật](hinh/mp_oak_ghep.jpg|16)

### 5.2.2 Cảm biến lực ở hai má kẹp (mô phỏng)

Trên robot thật, nhóm không biết được gripper đã kẹp chặt vật hay chưa: vị trí servo không đủ tin cậy vì ngón in 3D bị uốn (mục 4.5.7). Giải pháp được chọn là gắn ở mỗi má một **cảm biến lực màng mỏng RFP602**. Phần cứng thật của phương án này chưa hoàn thành (mục 4.5.8), nhưng toàn bộ phần phần mềm (bản tin, cách đọc, thuật toán kẹp theo lực) đã được xây dựng và kiểm chứng trong mô phỏng.

**Nguyên lý cảm biến.** RFP602 là một điện trở thay đổi theo lực ép (Force Sensing Resistor, FSR): khi không có lực, điện trở rất lớn (hở mạch); khi bị ép, điện trở giảm gần như tỉ lệ nghịch với lực. Cảm biến được mắc thành cầu chia áp với một điện trở 10 kΩ, điện áp ở giữa được đưa vào bộ ADC 12 bit (0–4095) của vi điều khiển:

$$ R_FSR = k / F,   k = 30 000 Ω·N $$ (5.1)

$$ N_ADC = 4095 · R_d / (R_d + R_FSR),   R_d = 10 kΩ $$ (5.2)

Phần mềm đọc giá trị ADC rồi tính ngược ra lực theo hai công thức trên. Đặc tuyến được vẽ ở Hình 5.4: lực càng lớn thì giá trị ADC càng tăng, nhưng tăng chậm dần, nên cảm biến nhạy nhất ở vùng lực nhỏ (dưới 10 N), đúng vùng cần để phát hiện tiếp xúc.

![Hình 5.4. Đặc tuyến cảm biến RFP602 trong mô hình: điện trở giảm tỉ lệ nghịch với lực (đường đứt, trục phải), giá trị ADC 12 bit tăng theo lực (đường liền, trục trái)](hinh/mp_fsr_dac_tuyen.png|13)

**Cách giả lập trong Gazebo.** Gazebo không có sẵn cảm biến FSR. Nhóm viết một nút ROS 2 (`fsr_emulator`) làm ba việc, 100 lần mỗi giây:

1. Đọc lực tiếp xúc mà Gazebo tính ra ở mặt trong của từng má (cảm biến tiếp xúc 200 Hz).
2. Chỉ giữ những điểm tiếp xúc nằm trên vùng tấm cảm biến (46 × 60 mm, phủ mặt kẹp), rồi lấy thành phần lực vuông góc với mặt má.
3. Cho lực đi qua mô hình cảm biến thật: cộng nhiễu 3 %, dưới 0,2 N thì coi là 0 (ngưỡng kích hoạt của FSR), trên 49 N thì bão hòa, đổi sang giá trị ADC theo (5.2) rồi tính ngược ra lực theo (5.1). Kết quả được phát lên topic `/fr3_demo/gripper/finger_forces`, **đúng định dạng mà bo mạch cảm biến thật sẽ phát**.

Nhờ bước 3, phần mềm phía trên không phân biệt được lực đến từ mô phỏng hay từ cảm biến thật. Khi có phần cứng, chỉ cần thay nút giả lập bằng bo mạch thật, không phải sửa thuật toán kẹp.

TABLE Bảng 5.2. Tham số mô hình cảm biến lực và thuật toán kẹp theo lực
| Tham số | Giá trị | Ý nghĩa |
| --- | --- | --- |
| Hằng số cảm biến k | 30 000 Ω·N | R = k / F, theo dạng đặc tuyến của FSR loại RFP602/FSR402 |
| Điện trở cầu chia | 10 kΩ | Cho độ nhạy cao ở vùng lực nhỏ |
| ADC | 12 bit (0–4095) | Độ phân giải ADC phổ biến của vi điều khiển, giống ADC của ESP32 |
| Ngưỡng kích hoạt | 0,2 N (khoảng 20 g) | Dưới mức này cảm biến báo 0 |
| Lực tối đa | 49 N (khoảng 5 kg) | Trên mức này cảm biến bão hòa |
| Nhiễu | 3 % | Độ chính xác điển hình của FSR là ±10–25 %, nên chỉ dùng để phát hiện tiếp xúc |
| Ngưỡng tiếp xúc | 0,5 N | Cờ `contact` của từng má |
| Ngưỡng dừng kẹp | 2,0 N trên cả hai má | Điều kiện để coi là đã kẹp chắc |
| Tốc độ khép | 5 mm/s mỗi má | Khép chậm để kịp dừng khi chạm |

**Thuật toán kẹp theo lực (guarded close).** Thay vì ra lệnh cho gripper khép tới một độ mở cố định, bộ lập kế hoạch cho hai má khép từ từ với tốc độ 5 mm/s, khép quá bề rộng đo được của vật một khoảng, đồng thời theo dõi lực ở hai má. Ngay khi **cả hai má** cùng đo được từ 2 N trở lên, lệnh khép bị dừng và vật được coi là đã kẹp. Nếu hai má khép hết hành trình mà không đủ cả hai má chạm vật, nghĩa là kẹp trượt hoặc vật không ở đó, robot **không nhấc lên** và báo lỗi. Thuật toán này giải quyết đúng hai vấn đề gặp trên robot thật: kẹp quá nhẹ làm vật tuột, và kẹp quá chặt làm servo nóng.

### 5.2.3 Gắp vật bằng lệnh nói trong mô phỏng

Lần chạy được ghi lại dùng câu lệnh “gắp cái chuột”. Toàn bộ chuỗi tự chạy, không có can thiệp của người.

**Bước 1: nhận diện và chọn vật.** Robot về tư thế quan sát, bộ nhận diện hình học tìm các vật trên bàn từ ảnh độ sâu, đánh số lên ảnh màu và gửi cho Gemini Robotics-ER kèm câu lệnh. Gemini trả lời: “gắp cái chuột” là vật số 3, con chuột máy tính màu trắng (Hình 5.5). Bước này mất 14,1 s, gồm cả thời gian robot về tư thế quan sát và chờ ảnh mới.

![Hình 5.5. Ảnh đánh số vật gửi cho Gemini trong mô phỏng. Gemini chọn đúng số 3 (chuột). Các số 5, 6, 7 là vật nhận nhầm từ chính các ngón gripper trong ảnh](hinh/mp_gemini.png|12)

Hình 5.5 cũng cho thấy một hạn chế ngay trong mô phỏng: bộ nhận diện hình học coi một phần ngón gripper là vật (số 5, 6, 7). Gemini vẫn chọn đúng vì hiểu được câu lệnh, nhưng nếu chỉ dựa vào hình học thì đây là các vật giả.

**Bước 2: lập kế hoạch trọn chuỗi.** MTC lập kế hoạch cho tất cả các đoạn: tiếp cận, hạ thẳng xuống, kẹp, nhấc lên, mang tới vị trí đặt. Kích thước vật đo từ camera là 50,1 × 33,7 mm, nên điểm kẹp (TCP) được đặt cao hơn tâm vật 28,2 mm. MTC tìm được 10 lời giải trong 3,7 s, chọn lời giải có chi phí thấp nhất (13,5), với tổng quãng quay của các khớp là 7,84 rad. Kiểm tra cuối cho thấy TCP trùng tâm vật (lệch 0,0 mm) và trục kẹp vuông góc với cạnh ngắn của vật. Robot chỉ bắt đầu chạy sau khi cả chuỗi đã có lời giải.

**Bước 3: thực thi.** Hình 5.6 là các khung hình Gazebo chụp trong lúc thực thi, Bảng 5.3 là thời gian từng giai đoạn.

![Hình 5.6. Các giai đoạn của lần gắp chuột trong mô phỏng: (a) tiếp cận phía trên vật, (b) hạ thẳng xuống, (c) khép má cho tới khi cảm biến lực báo chạm, (d) nhấc lên, (e) mang tới vị trí đặt](hinh/mp_chuoi_gap.png|16)

TABLE Bảng 5.3. Thời gian các giai đoạn của lần gắp chuột trong mô phỏng
| Giai đoạn | Thời gian (s) | Diễn giải |
| --- | --- | --- |
| Về tư thế quan sát, nhận diện, hỏi Gemini | 14,1 | Gemini chọn vật số 3 |
| Lập kế hoạch MTC (cả chuỗi) | 3,7 | 10 lời giải, robot chưa chuyển động |
| Tiếp cận | 7,8 | Đi tới phía trên vật |
| Hạ thẳng xuống | 19,8 | Đường thẳng Cartesian, chậm để an toàn |
| Kẹp theo lực | 4,0 | Dừng khi cả hai má ≥ 2 N |
| Nhấc lên | 24,1 | Nhấc tối thiểu 15 cm |
| Mang tới vị trí đặt | 11,1 | Quay đế tới vị trí đặt |
| **Tổng từ lúc ra lệnh** | **95,2** | Kết quả `TASK_EXECUTION_SUCCESS` |

**Lực ở hai má.** Hình 5.7 là lực đo được ở hai má trong suốt lần gắp. Trong các giai đoạn tiếp cận và hạ xuống, lực bằng 0 (má chưa chạm gì, và cũng không chạm mặt bàn). Trong giai đoạn kẹp, lực ở cả hai má tăng vọt khi ngón chạm vào chuột. Ở thời điểm cả hai má vượt 2 N, lệnh khép dừng ngay. Do quán tính và độ trễ dừng, lực đỉnh đạt 7,3 N và 6,8 N, nhật ký MTC ghi “Grasp detected: pad forces 6,3 N / 6,2 N”. Hai má có lực gần bằng nhau, nghĩa là vật nằm giữa hai má, đúng như kế hoạch.

![Hình 5.7. Lực đo được ở hai má trong lần gắp chuột (mô phỏng). Trái: toàn bộ lần gắp, các vùng màu là các giai đoạn. Phải: phóng to giai đoạn kẹp, lệnh khép dừng ngay khi cả hai má vượt 2 N](hinh/mp_luc_kep.png|16)

Sau khi kẹp, lực chỉ còn khoảng 0,2–0,7 N. Nguyên nhân là ngay sau khi xác nhận kẹp, phần mềm yêu cầu Gazebo **gắn cứng** vật vào gripper (tạo một khớp cố định). Lý do: mô phỏng ma sát giữa hai má và vật trong Gazebo không ổn định, vật hay bị rung hoặc văng ra dù kẹp đúng. Cách gắn cứng này là cách làm thường gặp khi mô phỏng gắp, nhưng có hệ quả: **trong mô phỏng, vật không bao giờ bị trượt sau khi đã kẹp**. Như vậy, lực trong giai đoạn nhấc và mang đi không phản ánh lực giữ thật, và mô phỏng không kiểm tra được hiện tượng tuột vật. Trên robot thật, đây lại chính là sự cố hay gặp nhất (mục 5.5.2).

**Cơ chế an toàn hoạt động đúng.** Trước lần gắp chuột, nhóm thử ra lệnh gắp hộp xanh. MTC từ chối với thông báo “compute collision-aware grasp IK (0/2): eef in collision: base_link_gripper – obj_2”: cả hai hướng kẹp đều làm thân gripper đụng vật bên cạnh. Robot không chuyển động. Đây là kết quả mong muốn: hệ thống thà không gắp còn hơn gắp mà va chạm.

### 5.2.4 Hiệu chuẩn trong mô phỏng

Mô phỏng có một lợi thế mà robot thật không có: biết chính xác vị trí thật (ground truth) của camera và của vật. Nhờ đó có thể đo được sai số của các quy trình hiệu chuẩn trước khi dùng chúng trên robot thật.

- **Hiệu chuẩn camera theo tag cố định trên bàn:** vị trí camera tính ra lệch 5,9 mm và 0,39° so với vị trí thật; vị trí vật suy ra từ camera lệch 1–4 mm trong vùng làm việc.
- **Hiệu chuẩn hand-eye với tag trên nắp trục J6** (Hình 4.7): sai số 2–4 mm. Mô phỏng cũng giúp phát hiện sớm một chi tiết hình học: nắp trục J6 thuộc khâu J5 nên tag không quay theo J6, và quy trình hiệu chuẩn phải xử lý trường hợp này.

Trên robot thật, hiệu chuẩn hand-eye đạt sai số kiểm tra trực tiếp trên tag 4,6 mm (mục 5.4.1), cùng cỡ với mô phỏng. Điều này cho thấy bản thân quy trình hiệu chuẩn đã đúng ngay từ mô phỏng. Phần sai số lớn hơn trên robot thật đến từ camera (độ sâu) chứ không phải từ quy trình.

### 5.2.5 So sánh mô phỏng và thực tế

Bảng 5.4 đặt cạnh nhau kết quả của từng khâu trong mô phỏng và trên robot thật.

TABLE Bảng 5.4. So sánh kết quả từng khâu giữa mô phỏng và robot thật
| Khâu | Mô phỏng | Robot thật | Nhận xét |
| --- | --- | --- | --- |
| Chọn vật bằng lời (Gemini) | Chọn đúng (chuột, lệnh tiếng Việt) | Chọn đúng 7 loại vật, 4–8 s mỗi lần | Giống nhau: Gemini không phụ thuộc mô phỏng hay thật |
| Nhận diện vật | Hình học từ độ sâu: tìm được vật, có vài vật giả quanh gripper | Hình học không dùng được; phải chuyển sang Gemini + SAM + GraspNet | Khác nhiều nhất, do chất lượng độ sâu |
| Độ sâu camera | Nhiễu 5 mm, mọi bề mặt đều đo được | Nhiễu ô caro ±23 mm, lệch khoảng 17 mm, không đo được vật đen | Phải thêm mô hình sửa độ sâu (mục 4.6.4) |
| Hiệu chuẩn | Camera lệch 5,9 mm; hand-eye 2–4 mm | Hand-eye 4,6 mm trên tag, sai số chiếu 0,75 px | Cùng cỡ: quy trình đúng ngay từ mô phỏng |
| Lập kế hoạch MTC | 10 lời giải trong 3,7 s, chọn chi phí thấp nhất | Lập được kế hoạch 7 đoạn (HOME → hộp → HOME) | Cùng một mã nguồn |
| Bám quỹ đạo | Chính xác tuyệt đối | TCP lệch 0–3 mm sau khi sửa hàng đợi ServoJ | Lỗi hàng đợi chỉ có trên robot thật (mục 2.6) |
| Kẹp | Theo lực, dừng ở 2 N, 6,3/6,2 N | Theo vị trí servo, cộng thêm 5 đơn vị góc | Robot thật chưa có cảm biến lực |
| Giữ vật | Gắn cứng, không bao giờ tuột | Tuột 2 lần (kẹp nhẹ); rơi 8 cm khi servo quá nhiệt | Mô phỏng không kiểm tra được |
| Kết quả cuối | Gắp và mang đi thành công, 95 s | Gắp, nhấc thành công 2 lần; chạy hết chuỗi bỏ vào hộp 1 lần | Thực tế khó hơn nhiều |

Từ bảng trên, có thể chia các yếu tố thành hai nhóm.

**Những gì mô phỏng làm được và chuyển sang thực tế không phải sửa:** cấu trúc phần mềm và cách các nút ROS 2 nối với nhau; lập kế hoạch MTC (tiếp cận, hạ thẳng, nhấc, mang đi, chọn lời giải theo chi phí); kiểm tra va chạm và từ chối kế hoạch khi có va chạm; cách chọn vật bằng lời với Gemini; quy trình hiệu chuẩn hand-eye; cách tách mạng ROS để an toàn. Đây là phần lớn mã nguồn của đề tài, và nhờ mô phỏng mà khi lên robot thật, phần này hầu như chạy được ngay.

**Những gì mô phỏng thiếu,** được liệt kê ở Bảng 5.5. Mỗi yếu tố đều đã gây ra sự cố thật, và đều được xử lý bằng một thay đổi cụ thể.

TABLE Bảng 5.5. Các yếu tố thực tế mà mô phỏng chưa thể hiện
| Yếu tố | Mô phỏng | Ảnh hưởng trên robot thật | Cách đề tài xử lý |
| --- | --- | --- | --- |
| Nhiễu và sai lệch độ sâu | Gần như lý tưởng | Nhận diện hình học thất bại; chỉ điểm lệch 2–3 cm | Gemini + SAM; mô hình sửa độ sâu theo pixel |
| Vật màu đen nhám | Đo được bình thường | Chỉ 15 % điểm ảnh của khay đen có độ sâu | Chiếu viền SAM lên mặt phẳng miệng khay (mục 4.7.3) |
| Vị trí và hướng đế robot | Biết chính xác | Hướng đế lệch 23°, robot đi lệch 13–20 cm (28/10) | Hiệu chuẩn hand-eye trên robot thật |
| Driver và hàng đợi lệnh | Bộ điều khiển lý tưởng | Hàng đợi ServoJ dồn 124 lệnh, robot không bám quỹ đạo | Sửa driver, giữ hàng đợi 0–3 lệnh |
| Ma sát và độ cứng ngón | Gắn cứng vật khi kẹp | Ngón in 3D uốn; kẹp nhẹ thì tuột | Kẹp theo góc cộng thêm; bỏ phát hiện trượt theo vị trí |
| Hành trình gripper thật | Má khép kín được | Hết hành trình khi hai má còn cách khoảng 30 mm | Lọc bỏ vật mỏng hơn 33 mm |
| Nhiệt độ servo | Không có | 64–79 °C sau 1–5 phút giữ vật, không mở được | Giới hạn nhiệt trong firmware; nhả lực |
| Đường truyền Wi-Fi | Không có (lệnh tức thời) | Mất 25–30 % gói, có lúc mất kết nối | Thử lại 3 lần; đề xuất nối dây trực tiếp |
| Cảm biến lực | Có (giả lập) | Chưa có phần cứng | Phần mềm sẵn sàng; chờ lắp cảm biến lên gripper |

Kết luận của phần so sánh: **mô phỏng kiểm chứng tốt phần “bộ não”** (nhận lệnh, lập kế hoạch, tránh va chạm, hiệu chuẩn), nhưng **không thay được thử nghiệm thật cho phần “cảm nhận” và “tay”**: độ sâu của camera, ma sát và độ cứng của ngón, nhiệt độ servo, đường truyền. Gần như mọi sự cố trên robot thật đều thuộc nhóm thứ hai. Vì vậy cách làm hợp lý là dùng mô phỏng để phát triển nhanh và loại bỏ lỗi logic, rồi dành thời gian trên robot thật cho các yếu tố vật lý. Ngược lại, cảm biến lực là một ví dụ mô phỏng đi trước phần cứng: thuật toán đã sẵn sàng và đã được kiểm chứng, khi có phần cứng chỉ cần nối vào.

### 5.2.6 Lần đầu chuyển sang robot thật

Ngày 28/10, chuỗi MTC được chạy lần đầu trên robot thật, với vị trí camera và bàn đo bằng thước. Robot đi tới điểm chờ phía trên tua vít nhưng lệch khoảng 13–20 cm. Nguyên nhân: vị trí đế robot được đo bằng thước, nhưng hướng của đế chỉ được giả định, và hướng thật lệch khoảng 23° so với mô phỏng. Đây là sự khác biệt đầu tiên giữa mô phỏng và thực tế (Bảng 5.5). Kết luận rút ra là phải hiệu chuẩn hand-eye trên robot thật thay vì đo tay.

![Hình 5.8. Bảng điều khiển MTC trong RViz trên robot thật (28/10): danh sách các giai đoạn và lời giải được chọn](hinh/p_mtc_rviz_28.png|15)

![Hình 5.9. Giai đoạn tiến vào theo đường thẳng Cartesian trong cây tác vụ MTC](hinh/p_mtc_cartesian.png|15)

## 5.3 Thực nghiệm nhận thức

### 5.3.1 Nhận diện bằng hình học depth: không đạt

Ở vị trí camera ban đầu (cách bàn 1,0–1,4 m, nhìn xéo), nhóm thử tách vật bằng hình học thuần tuý: tìm mặt phẳng bàn, lấy các điểm nhô lên. Kết quả: hoặc xuất hiện nhiều vật giả quanh chân đế và mép bàn, hoặc khi lọc chặt thì mất luôn vật thật. Bản đồ độ cao mặt bàn cho thấy nhiễu dạng ô caro với độ lệch chuẩn 10–11 mm, phân vị 2 và 98 là −23 và +23 mm, kể cả khi đã lấy trung vị 15 khung. Vật cao 2–5 cm chỉ nhô ngang mức nhiễu, nên không tách được.

![Hình 5.10. Bản đồ độ cao mặt bàn theo depth ở khoảng cách 1,0–1,4 m: nhiễu dạng ô caro khoảng ±23 mm](hinh/p_ban_do_cao.jpg|13)

![Hình 5.11. Nhận diện hình học với bộ lọc chặt: nhiều vật thật bị mất](hinh/p_perception_hh.jpg|13)

Sau thực nghiệm này, camera được đưa xuống còn khoảng 0,67 m và nhìn thẳng xuống. Nhiễu mặt bàn giảm còn 1,6–1,8 mm, tức giảm khoảng 6 lần.

### 5.3.2 Chọn vật bằng Gemini

Nhóm chuyển sang dùng Gemini Robotics-ER để khoanh vật, kết hợp hình học để tính vị trí 3D. Hai cách tính vị trí được so sánh: lấy độ sâu trung vị trong khung, và giao tia qua tâm khung với mặt phẳng bàn. Một số kết quả:

TABLE Bảng 5.6. Kết quả chỉ điểm bằng Gemini (30/10)
| Câu lệnh | Gemini trả về | Thời gian | Vị trí theo depth (mm) | Vị trí theo tia ∩ bàn (mm) |
| --- | --- | --- | --- | --- |
| “cái cốc giấy” | cốc giấy | 8,1 s | (−232; −456) | (−231; −452) |
| “con vật màu tím” | purple figurine | 6,8 s | (−13; −361) | (−8; −354) |

Hai cách tính chỉ lệch nhau 1–7 mm theo phương ngang. Sau này, với camera Astra, Gemini nhận đúng tay cầm game, khay đen, cục sạc, giá trắng và hộp vàng, mỗi lần mất khoảng 4–6 s.

![Hình 5.12. Chỉ điểm vật theo lệnh bằng Gemini trên camera thật](hinh/p_chi_diem_gemini.jpg|13)

### 5.3.3 GraspNet trên camera thật

Lần chạy GraspNet đầu tiên (30/10) gặp ba vấn đề:

- **Robot che camera:** tay robot đang ở ngay trên khối, nên GraspNet tìm cách gắp trên thân robot, mép bàn và sàn nhà. Từ đó, mọi lần nhận thức đều thực hiện khi robot ở tư thế HOME.
- **Độ mở luôn bằng 75 mm:** bản GraspNet trên máy nhân đầu ra độ mở với 10 rồi chặn ở 75 mm, trong khi bản gốc nhân với 1,2. Mọi độ mở đều bão hoà. Đề tài nhân lại với 0,12 trước bước giải mã mà không sửa thư viện. Sau khi sửa, độ mở phân bố trong khoảng 43–75 mm.
- **Depth làm “lùn” vật nhỏ:** khối 30 mm chỉ cao 11–15 mm trong đám điểm của OAK, nên với GraspNet khối gần như là một gờ nhỏ trên bàn. Mọi tư thế gắp trên khối đều bị coi là va chạm.

![Hình 5.13. Lần chạy GraspNet đầu tiên: không có tư thế gắp nào đạt trên khối đỏ](hinh/p_graspnet_red_oak.jpg|13)

Với camera Astra và các bộ lọc theo gripper thật, kết quả tốt hơn rõ rệt. Bảng 5.7 tóm tắt số tư thế gắp còn lại sau từng bộ lọc cho một số vật.

TABLE Bảng 5.7. Số tư thế gắp còn lại sau từng bộ lọc (camera Astra)
| Vật | GraspNet | Trên vật | Không va chạm | Độ mở 40–85 mm | Ăn sâu ≤ 30 mm |
| --- | --- | --- | --- | --- | --- |
| Giá trắng (đứng, 21:14) | 77 | 39 | 24 | 24 | 24 |
| Giá trắng (đứng, 22:27) | – | – | – | 40 | 38 |
| Tay cầm game (21:31) | – | – | – | 45 | 40 |
| Khối đỏ cạnh 30 mm | – | – | – | 0 | 0 |
| Khay đen (không có depth) | – | 0 | – | 0 | 0 |

![Hình 5.14. Tư thế gắp GraspNet trên tay cầm game](hinh/p_graspnet_gamepad.jpg|13)

### 5.3.4 Vật màu đen nhám

Khay nhựa đen gần như không có giá trị độ sâu: chỉ 15 % số pixel trên khay có giá trị, và giá trị trung vị (728 mm) còn sâu hơn mặt bàn (719 mm). Tay cầm game, cũng màu đen nhưng có bề mặt bóng hơn, đạt khoảng 62–66 %. Chế độ chiếu viền SAM với chiều cao giả định (mục 4.7.3) cho hộp bao 101 × 68 × 47 mm, khớp với kích thước thật trong vòng 1 mm.

![Hình 5.15. Ảnh màu (trái) và ảnh màu phủ độ sâu (phải) khi khay đen nằm ở góc dưới bên trái: vùng khay tối hẳn vì camera không đo được độ sâu, trong khi mặt bàn và các vật sáng màu được đo đầy đủ](hinh/ghep_khay_den.jpg|16)

![Hình 5.16. Khay đen được định vị bằng viền SAM chiếu lên mặt phẳng miệng khay, kẹp từ trên xuống ngang cạnh 68 mm](hinh/p_khay_assume.jpg|13)

## 5.4 Thực nghiệm hiệu chuẩn

### 5.4.1 Các lần hiệu chuẩn hand-eye

Hand-eye được hiệu chuẩn tổng cộng 9 lần, do camera bị thay, dời chỗ hoặc vô tình bị xê dịch. Bảng 5.8 tóm tắt kết quả.

TABLE Bảng 5.8. Kết quả các lần hiệu chuẩn hand-eye
| Lần | Ngày | Camera | Số mẫu | RMS (px) | LOO (px) | Ghi chú |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 30/10 | OAK | 6 | 0,48 | – | Ít mẫu, chưa kiểm chứng độc lập |
| 2 | 30/10 | OAK | 9 | 0,52 | 0,67 | Phát hiện depth đo xa hơn thật 7,7 % |
| 3 | 30/10 | OAK | 10 | 0,42 | 0,53 | Chỉ điểm khối đỏ lệch ngang khoảng 5 mm |
| 4 | 30/10 | OAK | 18 | 1,43 | 1,60 | Camera đặt quá gần, tag 0,21–0,44 m |
| 5 | 30/10 | OAK | 16 | 0,93 | 0,73 | Sửa cạnh tag 33,0 → 33,9 mm |
| 6 | 30/10 | OAK | 14 | 0,56 | 0,69 | Phát hiện camera đã xoay 3,9° |
| 7 | 31/10 | Astra | 21 | 0,84 | 0,94 | Camera mới |
| 8 | 01/11 | Astra | 21 | 0,79 | 0,91 | Camera đã xoay 2,1° |
| 9 | 02/11 | Astra | 19 | 0,75 | 0,81 | Camera bị dịch 16 mm; kiểm tra tag lệch 4,6 mm |

![Hình 5.17. Sai số chiếu kiểm tra chéo (LOO) qua các lần hiệu chuẩn](hinh/handeye_cac_lan.png|15)

Kết quả lần 9, dùng cho các thực nghiệm gắp cuối cùng: camera quang học ở (−0,3805; 0,3848; 0,7259) m trong hệ robot, tag trên khâu `wrist2_link` ở (0,0; −49,1; 99,0) mm. Vị trí tag này khớp với thiết kế CAD (0; −50,2; 102) mm trong vòng 3 mm, là một kiểm chứng độc lập cho kết quả hiệu chuẩn.

![Hình 5.18. Kiểm chứng hand-eye bằng hình: khung xương robot tính từ động học và kết quả hiệu chuẩn được vẽ chồng lên ảnh, trùng với robot thật; tâm tag dự đoán lệch 0,9 px](hinh/p_kiem_chung_handeye.jpg|13)

### 5.4.2 Kiểm tra chỉ điểm

Sau mỗi lần hiệu chuẩn, robot được cho chỉ tới một vật (khối đỏ hoặc vật Gemini chọn), rồi người vận hành đo lệch bằng thước:

TABLE Bảng 5.9. Kết quả kiểm tra chỉ điểm
| Thời điểm | Cấu hình | Lệch ngang | Lệch độ cao |
| --- | --- | --- | --- |
| 30/10, lần 2 | OAK, depth chia 1,077 | 8–10 mm | 70 mm (do depth) |
| 30/10, lần 3 | OAK, mô hình disparity | khoảng 5 mm | dưới 3 mm |
| 30/10, lần 5 | OAK, tỉ lệ cố định | 20–30 mm | – (phát hiện depth đọc xa 9,6 %) |
| 30/10, lần 6 | OAK, tâm 3D, k = 0,104 | dưới 10 mm | – |
| 01/11 | Astra, sau hiệu chuẩn lần 8 | – | gripper chạm bàn đúng như tính |

Bài học lớn nhất của phần này: phần lớn sai số chỉ điểm không đến từ hand-eye mà đến từ độ sâu của camera. Hand-eye chỉ dùng ảnh và robot (không dùng depth) cho phương ngang đúng trong khoảng 1 cm. Sai số độ sâu đi theo tia nhìn của camera, nên với camera nhìn xéo nó gây lệch cả phương ngang lẫn độ cao.

### 5.4.3 Mô hình sửa độ sâu

Trước khi sửa, mặt bàn (thực tế nằm ngang) hiện ra nghiêng 4,58° trong đám điểm của Astra, và các mẫu tag lệch trung bình +17,3 mm (RMS 17,8 mm). Sau khi áp mô hình (3.6), mặt bàn chỉ còn nghiêng 0,02°, sai số RMS trên mặt bàn 2,7–2,9 mm, và sai số trên mẫu tag còn −0,1 mm (RMS 3,2 mm).

![Hình 5.19. Ảnh màu (trái) và ảnh màu phủ độ sâu từ 0,4 đến 1,0 m (phải) ở tư thế HOME, sau khi sửa sai số độ sâu: mặt bàn có màu đồng đều, hộp vàng và giá trắng nổi lên rõ](hinh/ghep_home.jpg|16)

## 5.5 Thực nghiệm gắp trên robot thật

### 5.5.1 Các lần chạy

Bảng 5.10 liệt kê các lần chạy trên robot thật trong hai ngày cuối, theo thứ tự thời gian.

TABLE Bảng 5.10. Các lần chạy gắp trên robot thật
| Thời điểm | Vật | Việc | Kết quả |
| --- | --- | --- | --- |
| 01/11 17:57 | Khay đen | Tới tư thế sẵn sàng kẹp | TCP lệch 0,3 mm so với kế hoạch; đầu ngón cao hơn bàn 26 mm |
| 01/11 21:15 | Giá trắng | Tới tư thế sẵn sàng kẹp | TCP lệch 0 mm |
| 01/11 21:49 | Giá trắng | Gắp lần đầu | Hiệu chuẩn gripper bị đảo chiều, ngón khép lại đụng vật; dừng khẩn cấp |
| 01/11 22:00 | Giá trắng (nằm nghiêng) | Gắp | Má đè lên vật khi tiến vào; dừng khẩn cấp |
| 01/11 22:53 | Giá trắng (đứng) | Gắp và nhấc 10 cm, xác nhận từng bước | **Thành công**, TCP lệch 0 mm, kẹp tới 56 mm |
| 01/11 23:29 | Khay đen | Gắp và nhấc, chiều cao giả định | Tới đúng chỗ, lệch 0 mm; kẹp nhẹ, khay tuột |
| 01/11 23:35 | Giá trắng | Gắp và nhấc | Tới đúng chỗ; kẹp nhẹ, giá tuột |
| 01/11 23:51 | Giá trắng | Gắp và nhấc | Dừng ngay bước mở gripper vì Wi-Fi tới ESP32 bị mất |
| 02/11 00:16 | Giá trắng | Gắp và nhấc, kẹp theo góc +5° | **Thành công**; servo lên 79 °C |
| 02/11 01:20 | Giá trắng | HOME → gắp → hộp → HOME | Kẹp được, nhưng phần mềm tưởng là trượt nên mở kẹp |
| 02/11 01:35 | Giá trắng | HOME → gắp → hộp → HOME | **Chạy hết chuỗi**; servo 64 °C không mở được, nhả lực, vật rơi 8 cm vào hộp |
| 02/11 02:16 | Giá trắng | HOME → gắp → hộp (đặt sâu) → HOME | Lập được kế hoạch 7 đoạn, 104 s; chưa chạy |

Ở tất cả các lần robot chuyển động, TCP thật luôn tới đúng vị trí đã lập kế hoạch với sai lệch 0–3 mm, đọc từ TF. Như vậy chuỗi hiệu chuẩn, mô hình độ sâu, URDF và driver đã đủ chính xác cho bài toán. Các lần không thành công đều do gripper hoặc do cách bố trí vật.

![Hình 5.20. Robot ở tư thế sẵn sàng kẹp quanh khay đen (17:57): ảnh màu (trái) và ảnh màu phủ độ sâu (phải); hai má mở ôm hai bên khay, đầu ngón cao hơn mặt bàn 26 mm](hinh/ghep_pregrasp_khay.jpg|16)

![Hình 5.21. Gắp thành công giá trắng (02/11, 00:16): ảnh màu (trái) và ảnh màu phủ độ sâu (phải) khi gripper đã kẹp và nhấc giá lên 10 cm](hinh/ghep_nhac_gia.jpg|16)

### 5.5.2 Phân tích các sự cố

**Hiệu chuẩn gripper bị đảo chiều (21:49).** Lần hiệu chuẩn hành trình servo đầu tiên lưu “mở = 0, đóng = 1000”, trong khi thực tế ngược lại. Lệnh “mở gripper trước khi tiến vào” vì vậy lại khép hai má. Khi robot hạ xuống, ngón chạm vào giá và người vận hành bấm dừng khẩn cấp. Bài học: phải kiểm tra chiều mở/đóng bằng mắt sau mỗi lần hiệu chuẩn.

**Má đè lên vật (22:00).** GraspNet chọn một cách gắp có độ mở 75 mm, đặt dọc theo cái giá dài 141 mm đang nằm nghiêng, nên một ngón đè lên thân giá. Đây là lý do đề tài bổ sung bộ lọc “má không đè lên vật” và “phần bị kẹp phải dày trên 33 mm” (mục 4.7.2). Sau khi có hai bộ lọc này, không còn lần va chạm nào.

**Kẹp quá nhẹ (23:29, 23:35).** Như phân tích ở mục 4.5.7, khi đặt đích kẹp theo bề rộng đo được, hai má chưa ép vào vật. Cách kẹp theo góc cộng thêm, có dự phòng “khép tiếp tới khi bị chặn”, đã khắc phục được.

**Servo quá nhiệt (00:16, 01:35).** Sau khi kẹp, servo giữ vị trí với sai lệch lớn nên dòng điện cao liên tục. Nhiệt độ tăng từ 40 °C lên 76–79 °C sau khoảng 5 phút giữ vật, và lên 64 °C chỉ sau khoảng 1 phút trong kịch bản bỏ vào hộp. Từ 60 °C trở lên, firmware từ chối mọi lệnh di chuyển, kể cả lệnh mở. Hệ thống xử lý bằng cách nhả lực (tắt giữ lực) để vật rơi xuống. Sau một lần quá nhiệt, ESP32 còn mất kết nối Wi-Fi, nhiều khả năng do sụt nguồn.

**Wi-Fi không ổn định (23:51).** Ping tới ESP32 mất 25–30 % gói, độ trễ 20–400 ms, 2 trong 5 lần đọc trạng thái bị quá thời gian 6 s. Đây là lý do mọi lệnh tới gripper đều được thử lại tối đa 3 lần, và là lý do đề tài khuyến nghị nối dây trực tiếp thay cho Wi-Fi (phần Kết luận).

**Phát hiện trượt sai (01:20).** Phần mềm từng dựa vào vị trí servo để đoán có giữ được vật hay không: servo khép gần hết hành trình mà không bị chặn thì coi là trượt, và mở gripper. Thực tế ngón in 3D bị uốn nên servo khép tới vị trí 3 mà vẫn giữ được giá. Phần kiểm tra này đã bị bỏ.

## 5.6 Phân tích vùng làm việc và giới hạn khớp đế

Trong các lần chạy kịch bản bỏ vào hộp, nhiều cách bố trí vật không lập được kế hoạch, hoặc kế hoạch bắt đế robot xoay 160–300°. Để tìm nguyên nhân, nhóm gọi bộ giải IK với điểm khởi tạo là chính tư thế hiện tại, cho từng tư thế gắp. Kết quả cho thấy:

- Các vật nằm ở hướng khoảng 160–162° nhìn từ đế (trong hệ `base_footprint`) chỉ có nghiệm IK ở cấu hình lật tay, j1 khoảng −28°. Điều này đúng cả khi tắt kiểm tra va chạm, và cả khi đổi tư thế gắp thành thẳng đứng.
- Các vật ở hướng 145–155° có nghiệm với j1 khoảng 160–172°, tức đế chỉ cần xoay 30–40° từ HOME.
- Nguyên nhân là giới hạn j1 = ±175°. Với cấu hình cổ tay hiện tại, để với tới hướng 162° robot cần j1 khoảng 178°, vượt quá giới hạn.

![Hình 5.22. Bản đồ vùng làm việc nhìn từ trên: vùng tô đỏ sát giới hạn j1 buộc robot phải lật tay; các điểm là vị trí vật trong các lần thử](hinh/vung_lam_viec.png|13)

Từ phân tích này, đề tài vẽ vùng đặt vật hợp lệ trực tiếp lên ảnh camera (Hình 5.23), giúp người vận hành đặt vật đúng chỗ ngay từ đầu. Ngoài giới hạn j1, hai yếu tố khác cũng làm giảm vùng làm việc: thân gripper dài 164 mm dễ va vào vật bên cạnh nếu các vật đặt cách nhau dưới khoảng 15 cm, và hộp đựng lớn chiếm một phần đáng kể của vùng với tới.

![Hình 5.23. Vùng đặt vật được vẽ lên ảnh camera: vòng xanh là chỗ đặt khuyến nghị, vạch đỏ là giới hạn j1, chấm đỏ là biên tầm với 30 cm và 45 cm](hinh/p_cho_dat_gia.jpg|13)

## 5.7 Đánh giá tổng hợp

TABLE Bảng 5.11. Đối chiếu kết quả với yêu cầu
| Mã | Yêu cầu | Kết quả | Đánh giá |
| --- | --- | --- | --- |
| YC1 | Nhận lệnh ngôn ngữ | Gemini chọn đúng vật trong mọi lần thử (7 loại vật); có mô hình dự phòng khi hết lượt | Đạt |
| YC2 | Sai số ngang dưới 10 mm | Hand-eye lệch 4,6 mm trên tag; chỉ điểm dưới 10 mm; TCP tới đích lệch 0–3 mm | Đạt |
| YC3 | Cách gắp phù hợp gripper | 7 bộ lọc; sau khi bổ sung không còn va chạm | Đạt |
| YC4 | Tránh va chạm, xem trước | Vật cản từ depth, hộp có thành; toàn bộ chuỗi hiện trong RViz | Đạt |
| YC5 | Điều khiển gripper | Mô phỏng: kẹp theo lực, dừng ở 2 N. Thật: mở, kẹp, nhả được; lực kẹp và nhiệt độ còn hạn chế | Đạt một phần |
| YC6 | Kịch bản trọn vẹn | Chạy hết chuỗi một lần (01:35), thả rơi 8 cm do servo nóng | Đạt một phần |
| YC7 | An toàn | Xác nhận một lần, tự dừng khi lệch 10 mm, giới hạn nhiệt | Đạt |
| YC8 | Ghi dữ liệu | Ảnh, video, point cloud, log cho mọi lần chạy | Đạt |

Phần định vị và lập kế hoạch đã đạt yêu cầu và cho kết quả lặp lại tốt. Phần còn yếu là gripper: lực kẹp phụ thuộc vào cách đặt đích, servo nóng nhanh khi giữ vật, và đường truyền Wi-Fi không ổn định. Đây cũng là hướng cải tiến chính được đề xuất ở chương Kết luận.
