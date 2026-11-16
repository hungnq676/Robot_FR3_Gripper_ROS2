# MỞ ĐẦU

## 1. Tính cấp thiết của đề tài

Trong bối cảnh tự động hoá và chuyển đổi số đang được triển khai mạnh mẽ ở các nhà máy thông minh, kho vận tự động và dây chuyền sản xuất hiện đại, nhu cầu sử dụng robot có khả năng thao tác và tương tác với môi trường thực tế ngày càng tăng. Bài toán robot gắp vật tự động (robotic pick-and-place) là một trong những bài toán quan trọng nhất của lĩnh vực này.

Tay máy công nghiệp hiện nay chủ yếu làm việc trong môi trường được sắp đặt sẵn: vật luôn nằm đúng một vị trí, đúng một tư thế, và chương trình chuyển động được dạy trước bằng tay. Chỉ cần vật lệch vài centimet hay đổi sang loại khác, người vận hành phải dạy lại robot. Để làm việc được trong môi trường thực tế, robot không chỉ cần điều khiển chuyển động chính xác mà còn phải **tự nhận diện vật, tự xác định vị trí và tư thế của vật trong không gian, rồi tự lập quỹ đạo để gắp**. Đó chính là hệ thống thao tác robot định hướng bằng thị giác (Vision-Guided Robotic Manipulation).

Các công cụ như ROS 2 và MoveIt 2 ngày càng phổ biến để giải bài toán động học ngược, lập kế hoạch quỹ đạo và điều khiển robot. Các mô hình học sâu ngày càng mạnh trong nhận diện vật. Tuy nhiên, phần lớn các nghiên cứu kết hợp robot cộng tác, camera, ROS 2 và MoveIt 2 cho bài toán gắp vật tự động vẫn dừng ở mô phỏng, chưa được thử nghiệm đầy đủ trên robot thật. Từ đó, nhóm thực hiện đề tài: **“Nghiên cứu sử dụng robot cánh tay kết hợp các thuật toán xác định vị trí tự động và lên quỹ đạo chuyển động gắp vật thể”** (mã số 52–SV–2026–ĐT1).

## 2. Mục tiêu nghiên cứu

Mục tiêu của đề tài là nghiên cứu, thiết kế và phát triển hệ thống thao tác robot định hướng bằng thị giác, sử dụng thị giác máy tính và hệ điều hành robot ROS 2. Hệ thống cho phép robot nhận diện vật thể, xác định vị trí và tư thế của vật trong không gian, từ đó lập kế hoạch chuyển động và điều khiển cánh tay robot gắp vật một cách tự động và chính xác.

Hệ thống tích hợp bốn mảng: thị giác máy tính (Computer Vision), nhận biết môi trường (Perception), lập kế hoạch chuyển động (Motion Planning) và thao tác robot (Robot Manipulation). Nhóm sử dụng cánh tay robot FAIRINO FR3, gắn thêm camera và tay gắp do nhóm tự thiết kế, chế tạo.

## 3. Nội dung nghiên cứu

Đề tài gồm năm nội dung nghiên cứu theo đề cương đã được duyệt. Bảng 0.2 cho biết mỗi nội dung được trình bày ở chương nào trong báo cáo.

1. **Nội dung 1:** tìm hiểu và vận hành thử nghiệm cánh tay robot FAIRINO FR3 qua ROS 2; thiết kế, chế tạo tay gắp (gripper) và tích hợp camera cho robot.
2. **Nội dung 2:** nghiên cứu cấu trúc tổng thể của hệ thống thao tác robot định hướng bằng thị giác sử dụng ROS 2, gồm các thành phần thị giác máy tính, lập kế hoạch chuyển động và điều khiển robot.
3. **Nội dung 3:** ứng dụng mô hình học sâu để nhận diện và phát hiện vật thể trên ảnh 2D, phục vụ xác định vị trí vật trong không gian làm việc của robot.
4. **Nội dung 4:** nghiên cứu các phương pháp hiệu chuẩn camera và chuyển đổi hệ toạ độ giữa camera và robot; xây dựng cơ chế ước lượng tư thế (pose) của vật thể dựa trên dữ liệu thị giác, phục vụ robot tiếp cận và gắp vật.
5. **Nội dung 5:** ứng dụng ROS 2 và MoveIt 2 để xây dựng hệ thống lập kế hoạch chuyển động, gồm bài toán động học, sinh quỹ đạo và điều khiển robot di chuyển tới vị trí mục tiêu.

TABLE Bảng 0.2. Vị trí các nội dung nghiên cứu trong báo cáo
| Nội dung | Lý thuyết | Thiết kế, xây dựng | Thực nghiệm |
| --- | --- | --- | --- |
| 1. Vận hành FR3 qua ROS 2; gripper; camera | Chương 2 | Mục 4.2, 4.4, 4.5 | Mục 2.6, 5.2, 5.5 |
| 2. Cấu trúc tổng thể hệ thống | Mục 1.1, 1.5 | Mục 4.3 | Mục 5.7 |
| 3. Nhận diện vật thể bằng học sâu | Mục 3.5 | Mục 4.7 | Mục 5.3 |
| 4. Hiệu chuẩn camera – robot, ước lượng tư thế vật | Mục 3.3, 3.4 | Mục 4.6, 4.7 | Mục 5.4 |
| 5. Lập kế hoạch chuyển động với ROS 2 và MoveIt 2 | Mục 2.3, 3.2, 3.6 | Mục 4.8, 4.9 | Mục 5.2, 5.5, 5.6 |

**Về nội dung 3:** đề cương dự kiến dùng mô hình YOLO. Trong quá trình thực hiện, nhóm thấy YOLO chỉ nhận diện được các lớp vật đã huấn luyện, trong khi bài toán cần gắp nhiều loại vật khác nhau theo yêu cầu. Muốn thêm vật mới thì phải thu thập ảnh, gán nhãn và huấn luyện lại. Vì vậy nhóm chuyển sang các mô hình “từ vựng mở” (open-vocabulary): Gemini Robotics-ER để tìm vật theo câu lệnh, MobileSAM để tách đường viền vật, và GraspNet để đề xuất cách gắp. Hướng mới vẫn đúng mục tiêu của nội dung 3 (nhận diện vật trên ảnh 2D để xác định vị trí vật), nhưng không cần huấn luyện lại cho từng loại vật. So sánh chi tiết hai cách tiếp cận ở mục 3.5.1.

## 4. Đối tượng và phạm vi nghiên cứu

**Đối tượng nghiên cứu** là bài toán gắp vật dựa trên thị giác cho tay máy 6 bậc tự do, gồm các khâu: nhận diện vật, hiệu chuẩn camera – robot, ước lượng vị trí và cách gắp, lập kế hoạch chuyển động, điều khiển tay gắp.

**Phạm vi nghiên cứu:**

- Tay máy FAIRINO FR3 và một tay gắp hai ngón song song do nhóm thiết kế, dẫn động bằng một servo bus.
- Một camera RGB-D đặt cố định phía trên bàn (cấu hình eye-to-hand). Nhóm dùng lần lượt Luxonis OAK-D Pro W và Orbbec Astra Pro Plus.
- Vật đặt rời trên mặt bàn phẳng, kích thước 3–15 cm, không trong suốt.
- Robot chạy ở tốc độ thấp (10–30 % tốc độ tối đa), luôn có người giám sát và nút dừng khẩn cấp.

## 5. Phương pháp nghiên cứu

- **Nghiên cứu tài liệu:** các công trình về hiệu chuẩn hand-eye, phát hiện tư thế gắp, phân vùng ảnh, lập kế hoạch chuyển động.
- **Mô phỏng:** dựng bàn, robot, camera và vật trong Gazebo để kiểm tra toàn bộ chuỗi phần mềm trước khi chạy robot thật.
- **Thực nghiệm trên robot thật:** chạy từng khâu trên phần cứng; đo sai số bằng các phép đo độc lập (thước, chạm mặt bàn, AprilTag); mỗi lần thử chỉ thay đổi một yếu tố; ghi nhật ký có thời gian cho mọi lần chạy.

## 6. Tiến độ thực hiện

TABLE Bảng 0.3. Tiến độ thực hiện đề tài
| STT | Nội dung | Thời gian | Kết quả |
| --- | --- | --- | --- |
| 1 | Xây dựng đề cương | 04/2026 | Đề cương được thông qua |
| 2 | Nghiên cứu tổng quan | 05/2026 | Tổng quan về bài toán (Chương 1) |
| 3 | Lựa chọn giải pháp | 05/2026 | Giải pháp kỹ thuật (Chương 3, mục 4.3) |
| 4 | Tích hợp phần cứng | 06/2026 | Gripper, camera, robot (mục 4.2) |
| 5 | Xây dựng phần mềm | 07/2026 | Nhận thức, hiệu chuẩn, lập kế hoạch (Chương 4) |
| 6 | Thử nghiệm sản phẩm | 08–11/2026 | Thực nghiệm trên robot thật (Chương 5) |
| 7 | Viết báo cáo, nghiệm thu | 10–11/2026 | Báo cáo này |

## 7. Kết quả đạt được so với đề cương

Đề cương đặt ra ba kết quả dự kiến. Cả ba đã đạt được:

1. **Xây dựng hệ thống thao tác robot định hướng bằng thị giác** tích hợp thị giác máy tính, ROS 2 và robot FR3 cho bài toán gắp vật tự động. Hệ thống nhận lệnh bằng lời, tìm vật trên ảnh, tính vị trí và cách gắp trong hệ toạ độ robot (Chương 4).
2. **Tích hợp camera và tay gắp cho robot.** Camera được hiệu chuẩn với robot đạt sai số chiếu 0,75 px. Tay gắp do nhóm thiết kế, in 3D, điều khiển qua Wi-Fi, đã kẹp và nhấc được vật thật (mục 4.2, 5.5).
3. **Lập kế hoạch chuyển động và gắp vật trên thực tế** bằng ROS 2 và MoveIt 2. Robot tự giải động học ngược, sinh quỹ đạo tránh va chạm, đi tới vị trí gắp với sai lệch 0–3 mm, và chạy trọn kịch bản HOME → gắp → bỏ vào hộp → về HOME (mục 4.9, 5.5).

## 8. Bố cục báo cáo

Ngoài phần mở đầu, kết luận, tài liệu tham khảo và phụ lục, báo cáo gồm năm chương:

- **Chương 1 – Tổng quan:** bài toán gắp vật bằng robot, tình hình nghiên cứu trong và ngoài nước, các hướng tiếp cận và định hướng của đề tài.
- **Chương 2 – Robot FAIRINO FR3 và nền tảng ROS 2:** giới thiệu robot, bộ điều khiển; ROS 2, MoveIt 2, RViz, Gazebo; vận hành thử nghiệm robot qua ROS 2.
- **Chương 3 – Cơ sở lý thuyết:** động học, mô hình camera và độ sâu, hiệu chuẩn hand-eye, các mô hình học sâu và lý thuyết lập kế hoạch chuyển động.
- **Chương 4 – Thiết kế và xây dựng hệ thống:** phần cứng (gripper, camera), kiến trúc phần mềm và từng khối chức năng.
- **Chương 5 – Thực nghiệm và đánh giá:** thực nghiệm mô phỏng (gồm cảm biến lực ở hai má kẹp) và so sánh với thực tế; kết quả đo đạc, các lần gắp trên robot thật, phân tích sự cố và đối chiếu với mục tiêu.
