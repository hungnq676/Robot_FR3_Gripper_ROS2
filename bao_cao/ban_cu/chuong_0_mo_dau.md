# MỞ ĐẦU

## 1. Lý do chọn đề tài

Tay máy công nghiệp hiện nay chủ yếu làm việc trong môi trường được sắp đặt sẵn: vật luôn nằm đúng một vị trí, đúng một tư thế, và chương trình chuyển động được dạy trước bằng tay trên teach pendant. Chỉ cần vật lệch vài centimet hay đổi sang một chủng loại khác, người vận hành phải dạy lại robot. Cách làm này phù hợp với dây chuyền sản xuất hàng loạt nhưng không phù hợp với các công việc linh hoạt như phân loại hàng hoá lẫn lộn, phục vụ trong phòng thí nghiệm, hay robot hỗ trợ con người trong gia đình.

Trong vài năm gần đây, ba nhóm công nghệ đã phát triển đủ để thay đổi cách tiếp cận này. Thứ nhất, các mô hình thị giác – ngôn ngữ lớn (vision-language model) như Gemini Robotics-ER có thể hiểu một câu lệnh tự nhiên kiểu “gắp cái màu trắng” và chỉ ra vật tương ứng trên ảnh. Thứ hai, các mô hình phân vùng tổng quát như Segment Anything (SAM) có thể tách chính xác đường viền của một vật bất kỳ mà không cần huấn luyện lại. Thứ ba, các mạng học sâu dự đoán cách gắp như GraspNet có thể đề xuất hàng trăm tư thế kẹp trên đám điểm 3D của một cảnh lạ. Khi kết hợp với một bộ lập kế hoạch chuyển động có kiểm tra va chạm như MoveIt 2, ta có thể xây dựng một hệ thống mà người dùng chỉ cần nói tên vật, robot tự tìm, tự tính cách gắp và tự đi tới.

Tuy nhiên, giữa các bài báo và một hệ thống chạy được trên robot thật còn một khoảng cách lớn. Camera phải được hiệu chuẩn chính xác so với robot; độ sâu của camera giá rẻ có sai số hệ thống; gripper tự chế có hành trình, lực kẹp và độ trễ riêng; robot có giới hạn khớp và giới hạn tầm với; và mọi sai lệch nhỏ đều có thể dẫn tới va chạm. Đề tài này được thực hiện để đi trọn khoảng cách đó trên một tay máy cộng tác thực tế, ghi lại đầy đủ số đo và các bài học rút ra.

## 2. Mục tiêu nghiên cứu

Mục tiêu tổng quát của đề tài là xây dựng một hệ thống cho phép tay máy FAIRINO FR3 gắp vật trên bàn theo lệnh ngôn ngữ tự nhiên, sử dụng một camera RGB-D đặt cố định và các mô hình AI có sẵn.

Các mục tiêu cụ thể gồm:

1. Nhận diện đúng vật được nhắc tới trong câu lệnh tiếng Việt hoặc tiếng Anh trên ảnh camera.
2. Xác định vị trí và tư thế gắp trong hệ toạ độ robot với sai số dưới 1 cm theo phương ngang.
3. Lập kế hoạch chuyển động tránh va chạm với bàn và các vật khác, có bước cho người vận hành xem trước trong RViz.
4. Điều khiển gripper hai ngón tự thiết kế để kẹp, nhấc và đặt vật.
5. Hoàn thành một kịch bản trọn vẹn: từ tư thế HOME, gắp vật, bỏ vào hộp, rồi trở về HOME.
6. Ghi lại dữ liệu (ảnh, video, log, point cloud) phục vụ đánh giá và báo cáo.

## 3. Đối tượng và phạm vi nghiên cứu

**Đối tượng nghiên cứu** là bài toán gắp vật dựa trên thị giác cho tay máy 6 bậc tự do, bao gồm các khâu: nhận diện vật theo ngôn ngữ, hiệu chuẩn camera – robot, ước lượng cách gắp, lập kế hoạch chuyển động và điều khiển gripper.

**Phạm vi nghiên cứu** được giới hạn như sau:

- Tay máy FAIRINO FR3 và một gripper hai ngón song song do nhóm tự thiết kế, dẫn động bằng một servo bus.
- Một camera RGB-D đặt cố định phía trên bàn (cấu hình eye-to-hand). Đề tài dùng lần lượt Luxonis OAK-D Pro W và Orbbec Astra Pro Plus.
- Vật đặt rời trên mặt bàn phẳng, kích thước trong khoảng 3–15 cm, không trong suốt.
- Robot chuyển động ở tốc độ thấp (khoảng 10–30 % tốc độ tối đa) và mỗi lần chạy đều có người giám sát, có thể bấm dừng khẩn cấp.

Đề tài không đi sâu vào huấn luyện lại các mô hình AI, không xử lý vật trong suốt hoặc vật chuyển động, và không tối ưu thời gian chu kỳ.

## 4. Phương pháp nghiên cứu

Đề tài kết hợp ba phương pháp:

- **Nghiên cứu tài liệu:** tìm hiểu các công trình về hiệu chuẩn hand-eye, phát hiện tư thế gắp, phân vùng ảnh và lập kế hoạch chuyển động theo tác vụ.
- **Mô phỏng:** dựng mô hình bàn, robot, camera và vật trong Gazebo Harmonic để kiểm tra toàn bộ chuỗi phần mềm trước khi chạy trên robot thật.
- **Thực nghiệm trên robot thật:** chạy từng khâu trên phần cứng, đo sai số bằng các phép đo độc lập (thước, chạm mặt bàn, AprilTag), thay đổi một yếu tố mỗi lần thử và ghi nhật ký có thời gian cho mọi lần chạy.

## 5. Đóng góp của đề tài

- Xây dựng được một hệ thống gắp vật theo lệnh ngôn ngữ chạy trên robot thật, kết hợp Gemini Robotics-ER, MobileSAM, GraspNet và MoveIt Task Constructor.
- Tìm ra và sửa nguyên nhân robot FR3 không bám quỹ đạo khi điều khiển qua ROS 2: hàng đợi lệnh ServoJ của bộ điều khiển bị dồn tới 124 lệnh. Sau khi sửa, hàng đợi còn 0–3 lệnh.
- Xây dựng quy trình hiệu chuẩn hand-eye có dẫn đường và kiểm tra chéo, đạt sai số chiếu 0,75 px, kèm mô hình sửa độ sâu theo vị trí pixel cho camera Astra.
- Đề xuất cách chọn tư thế gắp có tính tới hình học thật của gripper: đo bề dày phần vật nằm giữa hai má, loại các cách gắp làm má đè lên vật, và ưu tiên cách gắp dễ thực hiện.
- Ghi lại một bộ dữ liệu thực nghiệm có thời gian gồm ảnh, video, log và các bài học thực tế khi đưa các mô hình AI lên robot thật.

## 6. Bố cục báo cáo

Ngoài phần mở đầu, kết luận, tài liệu tham khảo và phụ lục, báo cáo gồm bốn chương:

- **Chương 1** trình bày tổng quan về bài toán gắp vật bằng robot, các hướng tiếp cận hiện nay và vị trí của đề tài.
- **Chương 2** trình bày cơ sở lý thuyết: động học tay máy, hiệu chuẩn hand-eye, mô hình camera và độ sâu, các mô hình AI được sử dụng và lý thuyết lập kế hoạch chuyển động.
- **Chương 3** trình bày thiết kế và xây dựng hệ thống: phần cứng, kiến trúc phần mềm, từng khối chức năng và các công cụ hỗ trợ.
- **Chương 4** trình bày thực nghiệm, kết quả đo đạc, các sự cố gặp phải và phân tích nguyên nhân.
