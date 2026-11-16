# KẾT LUẬN VÀ HƯỚNG PHÁT TRIỂN

## 1. Kết luận

Đề tài đã xây dựng được một hệ thống cho phép tay máy cộng tác FAIRINO FR3 gắp vật trên bàn theo lệnh ngôn ngữ tự nhiên, dùng một camera RGB-D đặt cố định, một gripper hai ngón tự thiết kế và các mô hình AI có sẵn. Các kết quả chính:

1. **Chuỗi nhận thức hoàn chỉnh:** Gemini Robotics-ER chọn đúng vật theo câu lệnh (7 loại vật đã thử, mỗi lần 4–8 s); MobileSAM tách mặt nạ trong khoảng 2 s; GraspNet đề xuất tư thế gắp trong khoảng 0,1 s trên GPU. Phễu lọc 10 tầng theo hình học thật của gripper loại bỏ các tư thế không thực hiện được.
2. **Hiệu chuẩn chính xác và có kiểm chứng:** quy trình hand-eye có dẫn đường đạt sai số chiếu 0,75–0,78 px, kiểm tra chéo 0,81–0,87 px, kiểm tra trực tiếp trên tag lệch 0,5–4,6 mm. Mô hình sửa độ sâu theo pixel đưa độ nghiêng giả của mặt bàn từ 4,58° về 0,02°.
3. **Sửa lỗi điều khiển robot:** tìm ra nguyên nhân robot không bám quỹ đạo là hàng đợi lệnh ServoJ bị dồn tới 124 lệnh, và sửa driver để giữ hàng đợi ở mức 0–3 lệnh.
4. **Lập kế hoạch trọn chuỗi:** MoveIt Task Constructor lập kế hoạch cho cả 7 đoạn của kịch bản gắp – bỏ vào hộp – về HOME trong một lần, hiển thị toàn bộ trong RViz trước khi chạy. Hệ thống chọn tư thế gắp dễ thực hiện nhất, tránh các kế hoạch làm đế xoay quá xa.
5. **Mô phỏng đầy đủ trên Gazebo:** chạy trọn chuỗi từ lệnh nói “gắp cái chuột” tới khi vật được mang đi trong 95 s; mô phỏng hai cảm biến lực RFP602 ở hai má và kẹp theo lực (dừng khi cả hai má ≥ 2 N, lực đo 6,3/6,2 N); chạy chuỗi Gemini – MobileSAM – GraspNet trên camera ảo và gắp chuột bỏ vào hộp thành công hai lần; so sánh có hệ thống giữa mô phỏng và thực tế để chỉ ra phần nào chuyển sang robot thật được ngay, phần nào phải thử thật.
6. **Thực nghiệm trên robot thật:** robot gắp và nhấc thành công giá in 3D hai lần, chạy hết kịch bản bỏ vào hộp một lần, và ở mọi lần chuyển động đều tới đúng vị trí đã lập kế hoạch với sai lệch 0–3 mm.

Ngoài kết quả kỹ thuật, đề tài rút ra một số bài học về phương pháp khi đưa AI lên robot thật:

- Mô hình AI chỉ là một khâu. Phần lớn thời gian của đề tài dành cho hiệu chuẩn, sửa sai số cảm biến và đồng bộ mô hình robot với phần cứng thật.
- Sai số lớn nhất không nằm ở chỗ dễ nghi ngờ nhất. Lệch chỉ điểm 2–3 cm từng bị nghi do hand-eye, nhưng thực ra do độ sâu của camera. Robot “không chạy” từng bị nghi do chế độ hay tốc độ, nhưng thực ra do hàng đợi lệnh.
- Xem trước đường đi trong RViz là cần nhưng chưa đủ. Hai lần phải bấm dừng khẩn cấp (01/11) đều xảy ra khi đường đi của robot đúng, còn lỗi nằm ở chỗ RViz không thể hiện: chiều mở/đóng của gripper bị đảo, và má kẹp đè lên vật. Người vận hành vẫn phải đứng cạnh nút dừng và kiểm tra trực tiếp.

## 2. Hạn chế

- **Gripper:** lực kẹp phụ thuộc vào cách đặt đích servo; servo lên 64–79 °C chỉ sau 1–5 phút giữ vật; servo hết hành trình khi hai má còn cách khoảng 30 mm nên không gắp được vật mỏng; ngón in 3D bị uốn nên không phát hiện được vật trượt.
- **Đường truyền tới gripper:** Wi-Fi tới ESP32 mất 25–30 % gói vào những lúc kém, có lúc mất kết nối hoàn toàn sau khi servo quá nhiệt.
- **Camera:** không đo được độ sâu của vật đen nhám; chân đế dễ bị xê dịch, mỗi lần đều phải hiệu chuẩn lại.
- **Vùng làm việc:** giới hạn khớp đế ±175° và thân gripper dài 164 mm làm nhiều vị trí trên bàn không gắp được nếu không lật tay. Người vận hành phải đặt vật trong vùng đã đánh dấu.
- **Mô phỏng:** vật được gắn cứng vào gripper khi kẹp nên không kiểm tra được hiện tượng trượt; độ sâu camera ảo gần như lý tưởng; chưa mô phỏng nhiệt độ servo và đường truyền Wi-Fi.
- **Mô hình AI dịch vụ:** bản miễn phí của Gemini Robotics-ER chỉ cho 20 lần gọi mỗi ngày.
- **GraspNet với vật thấp:** với vật thấp như chuột (34 mm) hoặc khối 30 mm, GraspNet thường chỉ đề xuất tư thế nghiêng nhiều hoặc không có tư thế hợp lệ; hệ thống phải dùng phương án gắp hình học từ trên xuống.
- **Đánh giá:** số lần gắp trên robot thật còn ít, và từ khi phép quy đổi bề dày – vị trí servo sai, việc giữ được vật chỉ được xác nhận bằng quan sát của người vận hành.

## 3. Hướng phát triển

1. **Cải tiến gripper:** đổi tỉ số truyền để khép kín hơn; làm ngón cứng hơn hoặc thêm lớp đệm cao su; gắn cảm biến lực màng mỏng ở hai má (phần mềm và thuật toán kẹp theo lực đã sẵn sàng và kiểm chứng trong mô phỏng, mục 5.2.2) để kẹp theo lực và phát hiện trượt; giảm lực giữ sau khi kẹp và thêm tản nhiệt cho servo.
2. **Đường truyền:** gripper đã là một nút ROS 2 qua micro-ROS, nhưng Wi-Fi vẫn là điểm yếu: chuyển transport của micro-ROS sang UART hoặc USB có dây chạy dọc cánh tay, hoặc dùng bộ phát Wi-Fi riêng đặt sát robot, để thời gian đáp ứng ổn định.
3. **Mở rộng vùng làm việc:** chọn lại tư thế HOME và vị trí đặt robot để vùng bàn nằm giữa tầm quay của j1; thêm ràng buộc khớp vào MTC thay vì lọc sau.
4. **Camera:** gắn camera chắc chắn và tự kiểm tra xê dịch (`cam_check`) mỗi đầu phiên; thử camera stereo chủ động cho vật tối màu; cân nhắc gắn thêm một camera trên cổ tay (eye-in-hand) để tinh chỉnh ở bước cuối.
5. **Nhận thức:** chạy mô hình thị giác – ngôn ngữ cục bộ để không phụ thuộc giới hạn gọi API; học thêm GraspNet trên dữ liệu của gripper thật.
6. **Mô phỏng sát thực tế hơn:** đưa mô hình nhiễu và sai lệch độ sâu đo được của camera thật vào camera ảo; mô phỏng gắp bằng ma sát thay cho gắn cứng để thử được hiện tượng trượt; thêm mô hình nhiệt của servo.
7. **Đánh giá định lượng:** chạy một bộ thử chuẩn với nhiều vật và nhiều vị trí để thống kê tỉ lệ gắp thành công, thời gian chu kỳ và nguyên nhân thất bại.
