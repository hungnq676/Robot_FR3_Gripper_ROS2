# CHƯƠNG 5. THỰC NGHIỆM VÀ ĐÁNH GIÁ

## 5.1 Môi trường và phương pháp thực nghiệm

Thực nghiệm được tiến hành từ ngày 23/10/2026 đến ngày 02/11/2026 trên hai máy tính: máy hungros2 (phát triển ban đầu và mô phỏng) và máy ROG Strix G712LU (robot thật). Bàn thử nghiệm là bàn gỗ 100 × 50 cm đặt cạnh đế robot. Vật thử nghiệm gồm: khối gỗ đỏ cạnh 30 mm, cốc giấy, tay cầm game màu đen, cục sạc điện thoại, khay nhựa đen 100 × 67 × 47 mm, giá in 3D màu trắng 140 × 60 × 58 mm và một hộp nhựa vàng khoảng 310 × 240 × 112 mm.

Phương pháp thực nghiệm tuân theo ba nguyên tắc được rút ra trong quá trình làm:

- **Mỗi lần thử chỉ thay đổi một yếu tố.** Thay đổi nhiều thứ cùng lúc từng dẫn tới quy nhầm nguyên nhân (mục 2.6.1).
- **Dùng phép đo vật lý làm chuẩn, kiểm tra chéo bằng dữ liệu.** Ví dụ: chạm mặt bàn để biết độ cao bàn; đo khe bằng thước; so quãng đường tag di chuyển theo robot và theo camera.
- **Ghi nhật ký có thời gian cho mọi lần chạy**, kể cả lần hỏng, để không lặp lại các giả thuyết đã bị bác bỏ.

## 5.2 Thực nghiệm mô phỏng

### 5.2.1 Mô hình mô phỏng

Mô hình mô phỏng trong Gazebo Harmonic gồm bàn, robot FR3, camera OAK-D Pro W dựng theo thông số hãng (góc nhìn 95° × 72°, depth căn theo ảnh màu) và các vật thử nghiệm: hộp, chai nước, chuột máy tính (mô hình quét 3D), bút, tua vít. Lệnh “gắp cái chuột” được thực hiện trọn vẹn: Gemini chọn đúng vật `obj_4`, MTC lập kế hoạch cho cả chuỗi tiếp cận, hạ, kẹp, nhấc, mang đi, và kết quả thực thi là `TASK_EXECUTION_SUCCESS`.

### 5.2.2 Lần đầu chạy trên robot thật

Ngày 28/10, chuỗi MTC được chạy lần đầu trên robot thật, với vị trí camera và bàn đo bằng thước. Robot đi tới điểm chờ phía trên tua vít nhưng lệch khoảng 13–20 cm. Nguyên nhân: vị trí đế robot được đo bằng thước, nhưng hướng của đế chỉ được giả định, và hướng thật lệch khoảng 23° so với mô phỏng. Kết luận rút ra là phải hiệu chuẩn hand-eye trên robot thật thay vì đo tay.

![Hình 5.1. Bảng điều khiển MTC trong RViz trên robot thật (28/10): danh sách các giai đoạn và lời giải được chọn](hinh/p_mtc_rviz_28.png|15)

![Hình 5.2. Giai đoạn tiến vào theo đường thẳng Cartesian trong cây tác vụ MTC](hinh/p_mtc_cartesian.png|15)

## 5.3 Thực nghiệm nhận thức

### 5.3.1 Nhận diện bằng hình học depth: không đạt

Ở vị trí camera ban đầu (cách bàn 1,0–1,4 m, nhìn xéo), nhóm thử tách vật bằng hình học thuần tuý: tìm mặt phẳng bàn, lấy các điểm nhô lên. Kết quả: hoặc xuất hiện nhiều vật giả quanh chân đế và mép bàn, hoặc khi lọc chặt thì mất luôn vật thật. Bản đồ độ cao mặt bàn cho thấy nhiễu dạng ô caro với độ lệch chuẩn 10–11 mm, phân vị 2 và 98 là −23 và +23 mm, kể cả khi đã lấy trung vị 15 khung. Vật cao 2–5 cm chỉ nhô ngang mức nhiễu, nên không tách được.

![Hình 5.3. Bản đồ độ cao mặt bàn theo depth ở khoảng cách 1,0–1,4 m: nhiễu dạng ô caro khoảng ±23 mm](hinh/p_ban_do_cao.jpg|13)

![Hình 5.4. Nhận diện hình học với bộ lọc chặt: nhiều vật thật bị mất](hinh/p_perception_hh.jpg|13)

Sau thực nghiệm này, camera được đưa xuống còn khoảng 0,67 m và nhìn thẳng xuống. Nhiễu mặt bàn giảm còn 1,6–1,8 mm, tức giảm khoảng 6 lần.

### 5.3.2 Chọn vật bằng Gemini

Nhóm chuyển sang dùng Gemini Robotics-ER để khoanh vật, kết hợp hình học để tính vị trí 3D. Hai cách tính vị trí được so sánh: lấy độ sâu trung vị trong khung, và giao tia qua tâm khung với mặt phẳng bàn. Một số kết quả:

TABLE Bảng 5.1. Kết quả chỉ điểm bằng Gemini (30/10)
| Câu lệnh | Gemini trả về | Thời gian | Vị trí theo depth (mm) | Vị trí theo tia ∩ bàn (mm) |
| --- | --- | --- | --- | --- |
| “cái cốc giấy” | cốc giấy | 8,1 s | (−232; −456) | (−231; −452) |
| “con vật màu tím” | purple figurine | 6,8 s | (−13; −361) | (−8; −354) |

Hai cách tính chỉ lệch nhau 1–7 mm theo phương ngang. Sau này, với camera Astra, Gemini nhận đúng tay cầm game, khay đen, cục sạc, giá trắng và hộp vàng, mỗi lần mất khoảng 4–6 s.

![Hình 5.5. Chỉ điểm vật theo lệnh bằng Gemini trên camera thật](hinh/p_chi_diem_gemini.jpg|13)

### 5.3.3 GraspNet trên camera thật

Lần chạy GraspNet đầu tiên (30/10) gặp ba vấn đề:

- **Robot che camera:** tay robot đang ở ngay trên khối, nên GraspNet tìm cách gắp trên thân robot, mép bàn và sàn nhà. Từ đó, mọi lần nhận thức đều thực hiện khi robot ở tư thế HOME.
- **Độ mở luôn bằng 75 mm:** bản GraspNet trên máy nhân đầu ra độ mở với 10 rồi chặn ở 75 mm, trong khi bản gốc nhân với 1,2. Mọi độ mở đều bão hoà. Đề tài nhân lại với 0,12 trước bước giải mã mà không sửa thư viện. Sau khi sửa, độ mở phân bố trong khoảng 43–75 mm.
- **Depth làm “lùn” vật nhỏ:** khối 30 mm chỉ cao 11–15 mm trong đám điểm của OAK, nên với GraspNet khối gần như là một gờ nhỏ trên bàn. Mọi tư thế gắp trên khối đều bị coi là va chạm.

![Hình 5.6. Lần chạy GraspNet đầu tiên: không có tư thế gắp nào đạt trên khối đỏ](hinh/p_graspnet_red_oak.jpg|13)

Với camera Astra và các bộ lọc theo gripper thật, kết quả tốt hơn rõ rệt. Bảng 5.2 tóm tắt số tư thế gắp còn lại sau từng bộ lọc cho một số vật.

TABLE Bảng 5.2. Số tư thế gắp còn lại sau từng bộ lọc (camera Astra)
| Vật | GraspNet | Trên vật | Không va chạm | Độ mở 40–85 mm | Ăn sâu ≤ 30 mm |
| --- | --- | --- | --- | --- | --- |
| Giá trắng (đứng, 21:14) | 77 | 39 | 24 | 24 | 24 |
| Giá trắng (đứng, 22:27) | – | – | – | 40 | 38 |
| Tay cầm game (21:31) | – | – | – | 45 | 40 |
| Khối đỏ cạnh 30 mm | – | – | – | 0 | 0 |
| Khay đen (không có depth) | – | 0 | – | 0 | 0 |

![Hình 5.7. Tư thế gắp GraspNet trên tay cầm game](hinh/p_graspnet_gamepad.jpg|13)

### 5.3.4 Vật màu đen nhám

Khay nhựa đen gần như không có giá trị độ sâu: chỉ 15 % số pixel trên khay có giá trị, và giá trị trung vị (728 mm) còn sâu hơn mặt bàn (719 mm). Tay cầm game, cũng màu đen nhưng có bề mặt bóng hơn, đạt khoảng 62–66 %. Chế độ chiếu viền SAM với chiều cao giả định (mục 4.7.3) cho hộp bao 101 × 68 × 47 mm, khớp với kích thước thật trong vòng 1 mm.

![Hình 5.8. Ảnh màu (trái) và ảnh màu phủ độ sâu (phải) khi khay đen nằm ở góc dưới bên trái: vùng khay tối hẳn vì camera không đo được độ sâu, trong khi mặt bàn và các vật sáng màu được đo đầy đủ](hinh/ghep_khay_den.jpg|16)

![Hình 5.9. Khay đen được định vị bằng viền SAM chiếu lên mặt phẳng miệng khay, kẹp từ trên xuống ngang cạnh 68 mm](hinh/p_khay_assume.jpg|13)

## 5.4 Thực nghiệm hiệu chuẩn

### 5.4.1 Các lần hiệu chuẩn hand-eye

Hand-eye được hiệu chuẩn tổng cộng 9 lần, do camera bị thay, dời chỗ hoặc vô tình bị xê dịch. Bảng 5.3 tóm tắt kết quả.

TABLE Bảng 5.3. Kết quả các lần hiệu chuẩn hand-eye
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

![Hình 5.10. Sai số chiếu kiểm tra chéo (LOO) qua các lần hiệu chuẩn](hinh/handeye_cac_lan.png|15)

Kết quả lần 9, dùng cho các thực nghiệm gắp cuối cùng: camera quang học ở (−0,3805; 0,3848; 0,7259) m trong hệ robot, tag trên khâu `wrist2_link` ở (0,0; −49,1; 99,0) mm. Vị trí tag này khớp với thiết kế CAD (0; −50,2; 102) mm trong vòng 3 mm, là một kiểm chứng độc lập cho kết quả hiệu chuẩn.

![Hình 5.11. Kiểm chứng hand-eye bằng hình: khung xương robot tính từ động học và kết quả hiệu chuẩn được vẽ chồng lên ảnh, trùng với robot thật; tâm tag dự đoán lệch 0,9 px](hinh/p_kiem_chung_handeye.jpg|13)

### 5.4.2 Kiểm tra chỉ điểm

Sau mỗi lần hiệu chuẩn, robot được cho chỉ tới một vật (khối đỏ hoặc vật Gemini chọn), rồi người vận hành đo lệch bằng thước:

TABLE Bảng 5.4. Kết quả kiểm tra chỉ điểm
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

![Hình 5.12. Ảnh màu (trái) và ảnh màu phủ độ sâu từ 0,4 đến 1,0 m (phải) ở tư thế HOME, sau khi sửa sai số độ sâu: mặt bàn có màu đồng đều, hộp vàng và giá trắng nổi lên rõ](hinh/ghep_home.jpg|16)

## 5.5 Thực nghiệm gắp trên robot thật

### 5.5.1 Các lần chạy

Bảng 5.5 liệt kê các lần chạy trên robot thật trong hai ngày cuối, theo thứ tự thời gian.

TABLE Bảng 5.5. Các lần chạy gắp trên robot thật
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

![Hình 5.13. Robot ở tư thế sẵn sàng kẹp quanh khay đen (17:57): ảnh màu (trái) và ảnh màu phủ độ sâu (phải); hai má mở ôm hai bên khay, đầu ngón cao hơn mặt bàn 26 mm](hinh/ghep_pregrasp_khay.jpg|16)

![Hình 5.14. Gắp thành công giá trắng (02/11, 00:16): ảnh màu (trái) và ảnh màu phủ độ sâu (phải) khi gripper đã kẹp và nhấc giá lên 10 cm](hinh/ghep_nhac_gia.jpg|16)

### 5.5.2 Phân tích các sự cố

**Hiệu chuẩn gripper bị đảo chiều (21:49).** Lần hiệu chuẩn hành trình servo đầu tiên lưu “mở = 0, đóng = 1000”, trong khi thực tế ngược lại. Lệnh “mở gripper trước khi tiến vào” vì vậy lại khép hai má. Khi robot hạ xuống, ngón chạm vào giá và người vận hành bấm dừng khẩn cấp. Bài học: phải kiểm tra chiều mở/đóng bằng mắt sau mỗi lần hiệu chuẩn.

**Má đè lên vật (22:00).** GraspNet chọn một cách gắp có độ mở 75 mm, đặt dọc theo cái giá dài 141 mm đang nằm nghiêng, nên một ngón đè lên thân giá. Đây là lý do đề tài bổ sung bộ lọc “má không đè lên vật” và “phần bị kẹp phải dày trên 33 mm” (mục 4.7.2). Sau khi có hai bộ lọc này, không còn lần va chạm nào.

**Kẹp quá nhẹ (23:29, 23:35).** Như phân tích ở mục 4.5.7, khi đặt đích kẹp theo bề rộng đo được, hai má chưa ép vào vật. Cách kẹp theo góc cộng thêm, có dự phòng “khép tiếp tới khi bị chặn”, đã khắc phục được.

**Servo quá nhiệt (00:16, 01:35).** Sau khi kẹp, servo giữ vị trí với sai lệch lớn nên dòng điện cao liên tục. Nhiệt độ tăng từ 40 °C lên 76–79 °C sau khoảng 5 phút giữ vật, và lên 64 °C chỉ sau khoảng 1 phút trong kịch bản bỏ vào hộp. Từ 60 °C trở lên, firmware từ chối mọi lệnh di chuyển, kể cả lệnh mở. Hệ thống xử lý bằng cách nhả lực (tắt giữ lực) để vật rơi xuống. Sau một lần quá nhiệt, ESP32 còn mất kết nối Wi-Fi, nhiều khả năng do sụt nguồn.

**Wi-Fi không ổn định (23:51).** Ping tới ESP32 mất 25–30 % gói, độ trễ 20–400 ms, 2 trong 5 lần đọc trạng thái bị quá thời gian 6 s. Đây là lý do mọi lệnh tới gripper đều được thử lại tối đa 3 lần, và là lý do đề tài khuyến nghị nối dây trực tiếp thay cho Wi-Fi (mục 5.2).

**Phát hiện trượt sai (01:20).** Phần mềm từng dựa vào vị trí servo để đoán có giữ được vật hay không: servo khép gần hết hành trình mà không bị chặn thì coi là trượt, và mở gripper. Thực tế ngón in 3D bị uốn nên servo khép tới vị trí 3 mà vẫn giữ được giá. Phần kiểm tra này đã bị bỏ.

## 5.6 Phân tích vùng làm việc và giới hạn khớp đế

Trong các lần chạy kịch bản bỏ vào hộp, nhiều cách bố trí vật không lập được kế hoạch, hoặc kế hoạch bắt đế robot xoay 160–300°. Để tìm nguyên nhân, nhóm gọi bộ giải IK với điểm khởi tạo là chính tư thế hiện tại, cho từng tư thế gắp. Kết quả cho thấy:

- Các vật nằm ở hướng khoảng 160–162° nhìn từ đế (trong hệ `base_footprint`) chỉ có nghiệm IK ở cấu hình lật tay, j1 khoảng −28°. Điều này đúng cả khi tắt kiểm tra va chạm, và cả khi đổi tư thế gắp thành thẳng đứng.
- Các vật ở hướng 145–155° có nghiệm với j1 khoảng 160–172°, tức đế chỉ cần xoay 30–40° từ HOME.
- Nguyên nhân là giới hạn j1 = ±175°. Với cấu hình cổ tay hiện tại, để với tới hướng 162° robot cần j1 khoảng 178°, vượt quá giới hạn.

![Hình 5.15. Bản đồ vùng làm việc nhìn từ trên: vùng tô đỏ sát giới hạn j1 buộc robot phải lật tay; các điểm là vị trí vật trong các lần thử](hinh/vung_lam_viec.png|13)

Từ phân tích này, đề tài vẽ vùng đặt vật hợp lệ trực tiếp lên ảnh camera (Hình 5.16), giúp người vận hành đặt vật đúng chỗ ngay từ đầu. Ngoài giới hạn j1, hai yếu tố khác cũng làm giảm vùng làm việc: thân gripper dài 164 mm dễ va vào vật bên cạnh nếu các vật đặt cách nhau dưới khoảng 15 cm, và hộp đựng lớn chiếm một phần đáng kể của vùng với tới.

![Hình 5.16. Vùng đặt vật được vẽ lên ảnh camera: vòng xanh là chỗ đặt khuyến nghị, vạch đỏ là giới hạn j1, chấm đỏ là biên tầm với 30 cm và 45 cm](hinh/p_cho_dat_gia.jpg|13)

## 5.7 Đánh giá tổng hợp

TABLE Bảng 5.6. Đối chiếu kết quả với yêu cầu
| Mã | Yêu cầu | Kết quả | Đánh giá |
| --- | --- | --- | --- |
| YC1 | Nhận lệnh ngôn ngữ | Gemini chọn đúng vật trong mọi lần thử (7 loại vật); có mô hình dự phòng khi hết lượt | Đạt |
| YC2 | Sai số ngang dưới 10 mm | Hand-eye lệch 4,6 mm trên tag; chỉ điểm dưới 10 mm; TCP tới đích lệch 0–3 mm | Đạt |
| YC3 | Cách gắp phù hợp gripper | 7 bộ lọc; sau khi bổ sung không còn va chạm | Đạt |
| YC4 | Tránh va chạm, xem trước | Vật cản từ depth, hộp có thành; toàn bộ chuỗi hiện trong RViz | Đạt |
| YC5 | Điều khiển gripper | Mở, kẹp, nhả được; lực kẹp và nhiệt độ còn hạn chế | Đạt một phần |
| YC6 | Kịch bản trọn vẹn | Chạy hết chuỗi một lần (01:35), thả rơi 8 cm do servo nóng | Đạt một phần |
| YC7 | An toàn | Xác nhận một lần, tự dừng khi lệch 10 mm, giới hạn nhiệt | Đạt |
| YC8 | Ghi dữ liệu | Ảnh, video, point cloud, log cho mọi lần chạy | Đạt |

Phần định vị và lập kế hoạch đã đạt yêu cầu và cho kết quả lặp lại tốt. Phần còn yếu là gripper: lực kẹp phụ thuộc vào cách đặt đích, servo nóng nhanh khi giữ vật, và đường truyền Wi-Fi không ổn định. Đây cũng là hướng cải tiến chính được đề xuất ở chương Kết luận.
