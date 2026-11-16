# Ghi chú tập thuyết trình FR3

Bản nháp: 12 slide chính, 3 slide phụ; dự kiến 12–15 phút.

## Slide 1 — NGHIÊN CỨU KHOA HỌC SINH VIÊN

Mở đầu: giới thiệu đề tài và nhóm. Phần trình bày tập trung vào bài toán, cách xây dựng hệ thống và kiểm chứng từ mô phỏng tới robot thật. Thời lượng dự kiến 12–15 phút.

## Slide 2 — Vật đổi vị trí: robot cần tự tìm cách gắp

Tình huống: trên bàn có nhiều vật. Người dùng yêu cầu lấy một vật; vị trí vật có thể thay đổi giữa các lần thử. Nếu dùng tọa độ dạy sẵn, phải cập nhật chương trình. Đề tài hướng tới tự xác định vật và chuyển động; đây là mục tiêu, không phải kết luận rằng đã đạt độ tin cậy cho mọi tình huống.

## Slide 3 — Mục tiêu và phạm vi kiểm chứng

Nêu rõ đầu vào, đầu ra và điều kiện. Đích đặt được xác định trước hoặc nhận thức theo chế độ chạy. Tiêu chí thành công phải tách từng bước; tới đúng tọa độ chưa đồng nghĩa gắp được. Phạm vi này là bản nháp theo hệ thống hiện có, cần đối chiếu yêu cầu nghiệm thu.

## Slide 4 — Ý tưởng: kết hợp ngữ nghĩa, hình học và chuyển động

Ngữ nghĩa giúp biết vật nào cần lấy. Hình học giúp biết vật ở đâu và kẹp theo hướng nào. Mô hình robot và cảnh giúp kiểm tra khả năng tiếp cận và tính chuyển động. Không tuyên bố đề xuất mô hình học sâu mới; đóng góp được trình bày ở xây dựng, tích hợp và thích nghi hệ thống.

## Slide 5 — Thiết kế phần cứng và giao tiếp

Giới thiệu FR3 sáu bậc tự do, camera RGB-D cố định eye-to-hand và gripper hai ngón tự thiết kế. Hệ thống đã dùng OAK-D Pro W và Astra; các thử nghiệm thật cuối trong báo cáo dùng Astra. Máy tính xử lý nhận thức và lập kế hoạch; ESP32 làm cầu giao tiếp tới servo bus. Cảm biến lực mới mô phỏng, chưa gắn trên gripper thật.

## Slide 6 — Từ vùng ảnh đến vị trí trong hệ robot

Gemini chọn vùng chứa vật; SAM đánh dấu pixel thuộc vật. Depth cùng thông số nội tại camera cho điểm 3D trong hệ camera. Phép biến đổi hiệu chuẩn đưa điểm về hệ robot. Đám điểm chỉ chứa bề mặt quan sát được. Sai số chiếu pixel, sai số depth và sai số TCP là các đại lượng khác nhau.

## Slide 7 — Từ đề xuất GraspNet đến tư thế được chọn

GraspNet nhận đám điểm vùng quanh vật, có bối cảnh bàn; không chỉ đám điểm riêng vật. Mạng đề xuất vị trí, hướng, độ mở, độ sâu và điểm số. Sau các bộ lọc ban đầu, xếp theo điểm mạng và giữ top ứng viên. MTC thử các ứng viên, loại kế hoạch không hợp lệ. Điểm cuối là quy tắc kinh nghiệm; không cộng trực tiếp điểm GraspNet.

## Slide 8 — Lập kế hoạch cả chuỗi trước khi thực thi

Trong chế độ --into-box, mã dựng bảy đoạn trong cùng task MTC. Đóng kẹp sau đoạn tiến vào, mở kẹp sau đoạn hạ tại hộp. Vật cản được xấp xỉ bằng hộp từ depth; mục tiêu được bỏ khỏi hộp cản tương ứng. Cảnh có thể thiếu vật không đo được depth. Chỉ nói có kiểm tra va chạm theo mô hình, không bảo đảm tránh mọi vật cản thực.

## Slide 9 — Mô phỏng kiểm chứng chuỗi thao tác

Kết quả một lần chạy mô phỏng chuột ngày 02/11/2026. MTC có 10 lời giải trong 3,7 giây; thời gian từ lệnh tới mang vật là 95,2 giây theo báo cáo. Có một tình huống hộp xanh bị từ chối do va chạm theo mô hình. Vật được gắn cứng sau khi xác nhận kẹp; do đó không kiểm chứng ma sát giữ vật hoặc chống trượt. Mô phỏng dùng nhận diện hình học và Gemini; không mô tả sai thành toàn bộ pipeline SAM/GraspNet của robot thật.

## Slide 10 — Robot thật: gắp được, độ ổn định còn hạn chế

Theo bảng 5.10 và nhật ký: 01/11 22:53 và 02/11 00:16 có xác nhận kẹp và nhấc giá trắng. Chuỗi 01:35 chạy hết nhưng servo 64 độ không mở, phải nhả lực cho vật rơi khoảng 8 cm vào hộp. Kế hoạch đặt sâu 02:16 được lập, báo cáo ghi chưa chạy. Không trình bày các lần này thành tỉ lệ thành công vì chưa có bộ thử chuẩn cố định.

## Slide 11 — Kết quả cần được đánh giá theo từng khâu

Tách kết quả để không suy rộng quá bằng chứng. Nhận thức đã có ví dụ trên các loại vật nhưng chưa có thống kê đầy đủ. Lập kế hoạch chạy được trong mô phỏng và có kế hoạch, thực thi trên thật. Sai lệch TCP 0–3 mm trong báo cáo đọc từ TF không chứng minh độc lập sai số gắp vật ngoài đời. Giữ và thả vật trên thật còn hạn chế.

## Slide 12 — Kết luận và hướng phát triển

Kết luận ngắn: đã xây dựng hệ thống tích hợp cho FR3; mô phỏng kiểm chứng chuỗi và robot thật cho thấy khả năng tiếp cận, kẹp, nhấc trong một số tình huống. Chưa khẳng định vận hành ổn định. Ưu tiên cải tiến gripper và đường truyền, sau đó thiết kế bộ thử lặp lại. Dừng phần chính và mời câu hỏi.

## Slide 13 — Phụ lục · Điều kiện loại và điểm ưu tiên

Dùng trả lời câu hỏi về chọn tư thế. Điều kiện loại khác tiêu chí ưu tiên. joint_swing đo độ lệch lớn nhất của J1 so với điểm bắt đầu trên toàn kế hoạch, không đo tổng quay tích lũy. Điểm cuối cộng các đại lượng khác đơn vị với hệ số 1; chưa tối ưu trọng số qua thực nghiệm.

## Slide 14 — Phụ lục · Hiệu chuẩn và cách đọc sai số

Lần hiệu chuẩn Astra rạng sáng 02/11 trong báo cáo: RMS 0,75 px, kiểm tra chéo 0,81 px, kiểm tra tag 4,6 mm. Nhật ký sau đó có lần hiệu chuẩn khác do camera dịch; không gọi đây là bộ hiệu chuẩn mới nhất. RMS pixel đo chất lượng chiếu; không chuyển trực tiếp sang sai số gắp mm.

## Slide 15 — Phụ lục · Cảm biến lực trong mô phỏng

Cảm biến lực là mô hình giả lập từ lực tiếp xúc, chưa có trên gripper thật. Ngưỡng dừng là hai má cùng đạt 2 N nhưng có vượt ngưỡng; log ghi 6,3/6,2 N, đỉnh 7,3/6,8 N. Sau đó gắn cứng vật nên đường lực không chứng minh giữ vật bằng ma sát.

