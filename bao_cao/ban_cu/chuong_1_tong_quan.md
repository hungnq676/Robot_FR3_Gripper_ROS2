# CHƯƠNG 1. TỔNG QUAN VỀ BÀI TOÁN GẮP VẬT BẰNG TAY MÁY

## 1.1 Bài toán gắp vật dựa trên thị giác

Gắp vật (robotic grasping) là một trong những bài toán cơ bản nhất của robot thao tác. Đầu vào của bài toán là dữ liệu cảm biến về cảnh, thường là ảnh màu và ảnh độ sâu. Đầu ra là một tư thế của dụng cụ kẹp (vị trí, hướng và độ mở của hai má) sao cho khi khép hai má lại, vật được giữ chắc và có thể nhấc lên.

Trong thực tế, một hệ thống gắp hoàn chỉnh phải giải quyết nhiều bài toán con nối tiếp nhau:

1. **Chọn vật:** trong nhiều vật trên bàn, vật nào là vật người dùng muốn gắp.
2. **Định vị vật:** vật đó nằm ở đâu trong không gian ba chiều, so với robot.
3. **Chọn cách gắp:** kẹp vào phần nào của vật, theo hướng nào, mở bao nhiêu.
4. **Lập kế hoạch chuyển động:** tìm đường đi cho cả cánh tay từ tư thế hiện tại tới tư thế gắp mà không va vào bàn, vào vật khác hay vào chính nó.
5. **Thực thi và kẹp:** điều khiển robot đi đúng đường, khép gripper với lực vừa đủ, nhấc vật lên và mang tới đích.

Mỗi bài toán con đều có sai số riêng, và sai số của khâu trước truyền sang khâu sau. Ví dụ, một sai số 1° khi hiệu chuẩn hướng camera ở khoảng cách 0,7 m sẽ làm vị trí vật lệch khoảng 12 mm, đủ để một ngón kẹp đè lên vật thay vì đi vào bên cạnh. Vì vậy, một hệ thống gắp thực tế cần được thiết kế để kiểm soát sai số ở từng khâu và có cơ chế phát hiện khi sai số vượt ngưỡng.

## 1.2 Các hướng tiếp cận chính

### 1.2.1 Gắp dựa trên mô hình (model-based)

Hướng tiếp cận cổ điển giả định đã biết mô hình CAD của vật. Hệ thống nhận diện vật, ước lượng tư thế 6D của nó, rồi tra cứu các cách gắp đã được tính sẵn trên mô hình CAD. Cách này cho độ chính xác cao và ổn định trong nhà máy, nơi chủng loại vật là cố định. Nhược điểm là không xử lý được vật mới chưa có mô hình.

### 1.2.2 Gắp dựa trên hình học

Hướng thứ hai không cần mô hình vật mà dùng trực tiếp hình học của đám điểm: tìm mặt phẳng bàn bằng RANSAC, tách các cụm điểm nhô lên khỏi bàn, ước lượng hộp bao của từng cụm, rồi kẹp theo cạnh ngắn của hộp bao từ trên xuống. Cách này đơn giản, nhanh và dễ giải thích, nhưng chỉ hợp với vật có dạng hộp hoặc trụ và rất nhạy với nhiễu độ sâu. Trong giai đoạn đầu, đề tài đã thử cách này và thấy ở khoảng cách 1,0–1,4 m, nhiễu độ sâu trên mặt bàn trơn có độ lệch chuẩn 10–11 mm, gần bằng chiều cao của vật nhỏ, nên không tách được vật (mục 4.3.1).

### 1.2.3 Gắp dựa trên học sâu

Hướng thứ ba dùng mạng nơ-ron học từ dữ liệu lớn để dự đoán trực tiếp tư thế gắp. Dex-Net [13] dự đoán chất lượng gắp trên ảnh độ sâu. GPD [14] lấy mẫu các tư thế gắp trên đám điểm rồi phân loại. GraspNet-1Billion [1] cung cấp bộ dữ liệu hơn một tỷ tư thế gắp được gán nhãn và mạng GraspNet-baseline dự đoán tư thế gắp 6 bậc tự do trên đám điểm của cả cảnh. Ưu điểm của hướng này là xử lý được vật có hình dạng bất kỳ. Nhược điểm là mạng không biết vật nào người dùng muốn gắp và không biết hình dạng thật của gripper đang dùng.

### 1.2.4 Gắp theo lệnh ngôn ngữ

Hướng mới nhất kết hợp mô hình ngôn ngữ – thị giác để chọn vật theo câu lệnh. Gemini Robotics-ER [4] là mô hình của Google được tinh chỉnh cho các tác vụ robot như chỉ điểm, khoanh vùng và suy luận không gian. Khi nhận ảnh và câu lệnh “the white object”, mô hình trả về khung bao của vật dưới dạng JSON. Kết hợp với SAM [2] để tách đường viền chính xác và GraspNet để đề xuất cách gắp, ta được một chuỗi xử lý linh hoạt: người dùng chỉ cần nói, không cần dạy robot.

Đề tài đi theo hướng thứ tư nhưng giữ lại các kiểm tra hình học của hướng thứ hai để bổ sung cho những gì mạng học sâu không biết, đặc biệt là hình dạng và giới hạn thật của gripper.

## 1.3 Tay máy cộng tác và hệ sinh thái ROS 2

Tay máy cộng tác (cobot) là loại tay máy được thiết kế để làm việc gần con người, với tốc độ và lực giới hạn, có cảm biến phát hiện va chạm và có thể dạy bằng tay. FAIRINO FR3 là một cobot 6 bậc tự do, tải 3 kg, tầm với khoảng 622 mm, có driver ROS 2 chính thức (`frcobot_ros2`) và SDK C++ (`libfairino`).

ROS 2 [7] là nền tảng phần mềm phổ biến nhất cho robot nghiên cứu. Các thành phần chính mà đề tài sử dụng gồm:

- **ros2_control** [12]: khung điều khiển phần cứng, nhận quỹ đạo từ MoveIt và gửi lệnh vị trí khớp xuống driver.
- **TF2**: cây biến đổi hệ toạ độ, cho phép tính vị trí của bất kỳ khâu nào của robot hoặc camera trong hệ toạ độ gốc.
- **MoveIt 2** [6]: bộ lập kế hoạch chuyển động có kiểm tra va chạm, động học ngược và giao diện RViz.
- **MoveIt Task Constructor (MTC)** [5]: thư viện mô tả một tác vụ nhiều bước (tiếp cận, gắp, nhấc, mang đi, đặt) như một cây các giai đoạn và lập kế hoạch cho cả cây cùng lúc.

## 1.4 Tình hình nghiên cứu liên quan

Các công trình như GraspNet-1Billion [1] đạt tỉ lệ gắp thành công trên 80 % với vật lạ trong môi trường thí nghiệm, nhưng dùng camera RealSense đã hiệu chuẩn kỹ và gripper Robotiq có lực kẹp lớn. Các công trình kết hợp mô hình ngôn ngữ với robot như Gemini Robotics [4] cho thấy khả năng hiểu lệnh rất linh hoạt, nhưng thường được trình diễn trên phần cứng chuyên dụng của các phòng thí nghiệm lớn.

Ở Việt Nam, các đề tài sinh viên về gắp vật thường dùng phát hiện vật bằng YOLO trên ảnh màu, sau đó gắp từ trên xuống bằng giác hút hoặc gripper khí nén. Cách này hiệu quả với vật dẹt nằm trên bàn nhưng khó mở rộng cho vật có hình dạng phức tạp.

Đề tài này tìm cách áp dụng các mô hình mới nhất lên phần cứng mà một nhóm sinh viên có thể tiếp cận: một cobot cỡ nhỏ, một camera RGB-D giá rẻ và một gripper tự in 3D. Phần lớn công sức của đề tài vì vậy nằm ở việc làm cho các mô hình này hoạt động đúng trên phần cứng không hoàn hảo, chứ không ở việc đề xuất mô hình mới.

## 1.5 Định hướng của đề tài

Từ các phân tích trên, đề tài chọn định hướng như sau:

- **Chọn vật bằng ngôn ngữ:** dùng Gemini Robotics-ER để khoanh vật theo câu lệnh, có mô hình dự phòng khi hết lượt gọi.
- **Tách vật bằng SAM:** dùng MobileSAM [3], bản rút gọn của SAM, đủ nhanh để chạy cùng GraspNet trên GPU 6 GB.
- **Đề xuất cách gắp bằng GraspNet**, sau đó lọc lại bằng hình học thật của gripper: độ mở 40–85 mm, độ ăn sâu tối đa 30 mm, phần vật bị kẹp phải dày hơn 33 mm.
- **Lập kế hoạch cả chuỗi bằng MTC** với Pilz PTP cho các đoạn dài và đường thẳng Cartesian cho đoạn tiến vào, nhấc lên, hạ xuống.
- **An toàn trước hết:** mọi chuyển động đều được hiển thị trong RViz và chỉ chạy sau khi người vận hành gõ xác nhận.

![Hình 1.1. Chọn vật bằng Gemini Robotics-ER trong mô phỏng: các vật được đánh số trên ảnh (set-of-mark) để mô hình chọn theo câu lệnh](hinh/p_sim_gemini.png|11)
