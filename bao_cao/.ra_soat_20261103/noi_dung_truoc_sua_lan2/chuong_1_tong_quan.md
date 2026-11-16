# CHƯƠNG 1. TỔNG QUAN VỀ BÀI TOÁN GẮP VẬT BẰNG TAY MÁY

## 1.1 Bài toán gắp vật dựa trên thị giác

Gắp vật (robotic grasping) là một trong những bài toán cơ bản nhất của robot thao tác. Đầu vào của bài toán là dữ liệu cảm biến về cảnh, thường là ảnh màu và ảnh độ sâu. Đầu ra là một tư thế của dụng cụ kẹp (vị trí, hướng và độ mở của hai má) sao cho khi khép hai má lại, vật được giữ chắc và có thể nhấc lên.

Trong thực tế, một hệ thống gắp hoàn chỉnh phải giải quyết nhiều bài toán con nối tiếp nhau:

1. **Chọn vật:** trong nhiều vật trên bàn, vật nào là vật người dùng muốn gắp.
2. **Định vị vật:** vật đó nằm ở đâu trong không gian ba chiều, so với robot.
3. **Chọn cách gắp:** kẹp vào phần nào của vật, theo hướng nào, mở bao nhiêu.
4. **Lập kế hoạch chuyển động:** tìm đường đi cho cả cánh tay từ tư thế hiện tại tới tư thế gắp mà không va vào bàn, vào vật khác hay vào chính nó.
5. **Thực thi và kẹp:** điều khiển robot đi đúng đường, khép gripper với lực vừa đủ, nhấc vật lên và mang tới đích.

Một hệ thống giải trọn chuỗi bài toán trên được gọi là **hệ thống thao tác robot định hướng bằng thị giác** (Vision-Guided Robotic Manipulation). Về cấu trúc, hệ thống gồm bốn khối nối tiếp: khối **thị giác và nhận thức** (camera, nhận diện vật, đám điểm 3D), khối **định vị** (hiệu chuẩn camera – robot, chuyển toạ độ vật sang hệ robot, ước lượng tư thế gắp), khối **lập kế hoạch chuyển động** (động học ngược, tránh va chạm, sinh quỹ đạo) và khối **thao tác** (điều khiển robot và tay gắp). Kiến trúc cụ thể của đề tài được trình bày ở mục 4.3.

Mỗi bài toán con đều có sai số riêng, và sai số của khâu trước truyền sang khâu sau. Ví dụ, một sai số 1° khi hiệu chuẩn hướng camera ở khoảng cách 0,7 m sẽ làm vị trí vật lệch khoảng 12 mm, đủ để một ngón kẹp đè lên vật thay vì đi vào bên cạnh. Vì vậy, một hệ thống gắp thực tế cần được thiết kế để kiểm soát sai số ở từng khâu và có cơ chế phát hiện khi sai số vượt ngưỡng.

## 1.2 Các hướng tiếp cận chính

### 1.2.1 Gắp dựa trên mô hình (model-based)

Hướng tiếp cận cổ điển giả định đã biết mô hình CAD của vật. Hệ thống nhận diện vật, ước lượng tư thế 6D của nó, rồi tra cứu các cách gắp đã được tính sẵn trên mô hình CAD. Cách này cho độ chính xác cao và ổn định trong nhà máy, nơi chủng loại vật là cố định. Nhược điểm là không xử lý được vật mới chưa có mô hình.

### 1.2.2 Gắp dựa trên hình học

Hướng thứ hai không cần mô hình vật mà dùng trực tiếp hình học của đám điểm: tìm mặt phẳng bàn bằng RANSAC, tách các cụm điểm nhô lên khỏi bàn, ước lượng hộp bao của từng cụm, rồi kẹp theo cạnh ngắn của hộp bao từ trên xuống. Cách này đơn giản, nhanh và dễ giải thích, nhưng chỉ hợp với vật có dạng hộp hoặc trụ và rất nhạy với nhiễu độ sâu. Trong giai đoạn đầu, đề tài đã thử cách này và thấy ở khoảng cách 1,0–1,4 m, nhiễu độ sâu trên mặt bàn trơn có độ lệch chuẩn 10–11 mm, gần bằng chiều cao của vật nhỏ, nên không tách được vật (mục 5.3.1).

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

## 1.4 Tình hình nghiên cứu trong nước và ngoài nước

### 1.4.1 Ngoài nước

Trên thế giới, nghiên cứu về thao tác robot định hướng bằng thị giác đang phát triển rất mạnh, gắn liền với sự phát triển của trí tuệ nhân tạo. Có thể chia thành ba xu hướng:

- **Phát hiện tư thế gắp bằng học sâu.** Các công trình như Dex-Net [13], GPD [14] và GraspNet-1Billion [1] dùng mạng nơ-ron học từ dữ liệu lớn để dự đoán cách gắp trên ảnh độ sâu hoặc đám điểm. GraspNet-1Billion đạt tỉ lệ gắp thành công trên 80 % với vật lạ trong môi trường thí nghiệm, với camera RealSense đã hiệu chuẩn kỹ và gripper công nghiệp lực lớn.
- **Mô hình nền tảng cho robot.** Các tập đoàn công nghệ lớn đầu tư mạnh vào mô hình kết hợp thị giác, ngôn ngữ và hành động. Google DeepMind đã công bố RT-2 [23], mô hình thị giác – ngôn ngữ – hành động (Vision-Language-Action) cho phép robot hiểu lệnh bằng lời và sinh hành động trực tiếp, và gần đây là Gemini Robotics [4], trong đó có mô hình Gemini Robotics-ER chuyên về suy luận không gian cho robot mà đề tài sử dụng.
- **Nền tảng phần mềm mở.** ROS 2 [7], MoveIt 2 [6] và MoveIt Task Constructor [5] trở thành tiêu chuẩn trong nghiên cứu robot thao tác, giúp các nhóm nghiên cứu không phải viết lại từ đầu bộ giải động học, kiểm tra va chạm hay lập kế hoạch quỹ đạo.

Tuy vậy, nhiều hệ thống hiện nay vẫn có hạn chế: cần phần cứng tính toán mạnh, tích hợp phức tạp giữa nhận thức, lập kế hoạch và điều khiển, và khó chạy ổn định trên robot thật. Các kết quả ấn tượng thường đi kèm phần cứng chuyên dụng của các phòng thí nghiệm lớn.

### 1.4.2 Trong nước

Ở Việt Nam, robot công nghiệp thông minh và thị giác máy tính đang được quan tâm mạnh trong lĩnh vực robot và tự động hoá. Nhiều nghiên cứu ứng dụng ROS và ROS 2 để điều khiển và mô phỏng robot. Các mô hình học sâu như YOLO được dùng rộng rãi để nhận diện vật thời gian thực, thường triển khai trên máy tính nhúng như Raspberry Pi hoặc Jetson. Gazebo và MoveIt được dùng phổ biến cho bài toán động học ngược, lập quỹ đạo và mô phỏng tay máy. Một số doanh nghiệp và nhóm nghiên cứu như VinMotion, Viettel và các nhóm ở các trường đại học kỹ thuật đang phát triển hệ thống robot tích hợp ROS, lập kế hoạch chuyển động và thao tác robot.

Các đề tài sinh viên về gắp vật thường dùng YOLO trên ảnh màu, sau đó gắp từ trên xuống bằng giác hút hoặc gripper khí nén. Cách này hiệu quả với vật dẹt nằm trên bàn nhưng khó mở rộng cho vật có hình dạng phức tạp. Phần lớn các nghiên cứu kết hợp robot cộng tác, camera, ROS 2 và MoveIt 2 cho bài toán gắp vật tự động vẫn dừng ở mô phỏng hoặc thử nghiệm đơn giản, chưa được kiểm chứng đầy đủ trên robot thật. Đây là khoảng trống mà đề tài hướng tới.

### 1.4.3 Nhu cầu thực tế và khả năng áp dụng

Kết quả của đề tài có thể áp dụng cho:

- **Sản xuất và phân loại hàng hoá:** robot tự nhận diện và gắp các loại vật khác nhau trên băng chuyền hoặc trong thùng, không cần dạy lại cho từng loại.
- **Kho vận:** gắp và xếp hàng hoá có hình dạng, vị trí không cố định.
- **Môi trường nguy hiểm:** thay con người thao tác với vật trong môi trường độc hại, nhiệt độ cao, phóng xạ.
- **Đào tạo:** quy trình và công cụ của đề tài (hiệu chuẩn có dẫn đường, xem trước quỹ đạo, ghi dữ liệu) có thể dùng làm bài thực hành về robot thao tác cho sinh viên, giúp nhiều người tiếp cận lĩnh vực này hơn.

## 1.5 Định hướng của đề tài

Từ các phân tích trên, đề tài chọn định hướng như sau:

- **Chọn vật bằng ngôn ngữ:** dùng Gemini Robotics-ER để khoanh vật theo câu lệnh, có mô hình dự phòng khi hết lượt gọi.
- **Tách vật bằng SAM:** dùng MobileSAM [3], bản rút gọn của SAM, đủ nhanh để chạy cùng GraspNet trên GPU 6 GB.
- **Đề xuất cách gắp bằng GraspNet**, sau đó lọc lại bằng hình học thật của gripper: độ mở 40–85 mm, độ ăn sâu tối đa 30 mm, phần vật bị kẹp phải dày hơn 33 mm.
- **Lập kế hoạch cả chuỗi bằng MTC** với Pilz PTP cho các đoạn dài và đường thẳng Cartesian cho đoạn tiến vào, nhấc lên, hạ xuống.
- **An toàn trước hết:** mọi chuyển động đều được hiển thị trong RViz và chỉ chạy sau khi người vận hành gõ xác nhận.

![Hình 1.1. Chọn vật bằng Gemini Robotics-ER trong mô phỏng: các vật được đánh số trên ảnh (set-of-mark) để mô hình chọn theo câu lệnh](hinh/p_sim_gemini.png|11)
