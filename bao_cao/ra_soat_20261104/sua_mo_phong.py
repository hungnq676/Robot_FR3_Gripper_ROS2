from docx import Document
from docx.shared import Pt
from docx.oxml.ns import qn
from pathlib import Path
import re
src=Path('bao_cao/Bao_cao_NCKH_FR3_ra_soat.docx');out=src.with_name('Bao_cao_NCKH_FR3_ra_soat_v2.docx')
d=Document(src)
def put(p,t):
 if p.text==t:return
 # These are prose/caption paragraphs only. Preserve the paragraph style.
 for r in p.runs:r.text=''
 r=p.add_run(t);r.font.name='Times New Roman'
 r.font.size=Pt(12 if t.startswith('Hình ') else 13)
 if t.startswith('Hình '):r.italic=True
patch={
850:'Các số liệu lực kẹp và thời gian trong mục 5.2.2–5.2.3 minh hoạ một lần chạy mô phỏng đã lưu: lực được ghi ở tần số 100 Hz, ảnh được chụp khoảng 2 giây một lần và thời gian các giai đoạn được lấy từ nhật ký MTC. Ảnh đánh số vật trong lần chạy này thuộc phương án khảo sát hình học trước đó, không đại diện cho toàn bộ quy trình nhận thức hiện tại. Nhóm đã chạy lại mô phỏng với chuỗi thuật toán đầy đủ; không dùng các số liệu thời gian của lần khảo sát để thay cho số đo của lần chạy đầy đủ.',
849:'Trong mô phỏng, nhóm đã triển khai và chạy thử chuỗi đầy đủ gồm hiệu chuẩn camera–robot, chọn vật bằng Gemini, phân đoạn bằng MobileSAM, dựng đám mây điểm từ RGB-D, đề xuất tư thế gắp bằng GraspNet, lọc tư thế, giải động học ngược, lập kế hoạch MTC và điều khiển robot. MobileSAM là thành phần chính của chuỗi nhận thức, được sử dụng trong cả mô phỏng và trên robot thật. Nhóm cũng mô phỏng cảm biến lực ở hai má kẹp để thử thuật toán kẹp theo lực khi phần cứng cảm biến chưa được lắp trên gripper thật. Mục này trình bày hệ thống mô phỏng, kết quả khảo sát đã lưu và sự khác biệt khi triển khai trên phần cứng.',
852:'Mô phỏng dùng Gazebo Harmonic cùng ROS 2 Humble và MoveIt 2. Mô phỏng và robot thật cùng sử dụng kiến trúc nhận thức, hiệu chuẩn, sinh và lọc tư thế gắp, lập kế hoạch và điều khiển. Các tham số camera, phép biến đổi hiệu chuẩn và mô hình gripper được cấu hình phù hợp với từng môi trường. Giao diện phần cứng trong mô phỏng dùng gz_ros2_control thay cho driver fairino_hardware; vì vậy cần phân biệt phần thuật toán dùng chung với phần tích hợp và tham số phải kiểm tra lại trên robot thật.',
853:'Mô phỏng và robot thật được tách bằng ROS_DOMAIN_ID, đồng thời giới hạn truyền thông trong máy khi thử nghiệm. Camera ảo cung cấp ảnh màu và độ sâu cho chuỗi nhận thức Gemini → MobileSAM → đám mây điểm → GraspNet → lọc tư thế; kết quả được chuyển sang MTC để lập kế hoạch và thực thi. Phép biến đổi camera–robot được xác định bằng quy trình hiệu chuẩn trong mô phỏng trước khi dùng dữ liệu thị giác để điều khiển.',
860:'Camera ảo cung cấp ảnh màu và độ sâu cho cùng chuỗi xử lý thị giác như trên robot thật (Hình 5.3). Ảnh độ sâu mô phỏng thuận lợi hơn trong việc quan sát bề mặt; dữ liệu thật còn chịu ảnh hưởng của vị trí đặt camera, vật liệu và sai lệch cảm biến. Khác biệt này đòi hỏi kiểm tra và hiệu chỉnh dữ liệu độ sâu khi triển khai thật, không có nghĩa MobileSAM hoặc hiệu chuẩn chỉ được bổ sung sau mô phỏng.',
880:'Quy trình mô phỏng đầy đủ dùng Gemini xác định vật theo câu lệnh, MobileSAM tách mặt nạ, kết hợp ảnh độ sâu và phép biến đổi hiệu chuẩn để dựng đám mây điểm, rồi dùng GraspNet và các bộ lọc để chọn tư thế gắp. MTC lập kế hoạch các giai đoạn tiếp cận, gắp, nhấc và chuyển vật trước khi thực thi. Nhóm đã chạy lại quy trình này trong mô phỏng. Các hình và số liệu tiếp theo minh hoạ một lần khảo sát đã lưu trước đó với câu lệnh “gắp cái chuột”; riêng ảnh đánh số vật thể hiện phương án hình học của lần khảo sát đó.',
881:'Bước 1 trong lần khảo sát minh hoạ: robot về tư thế quan sát, bộ nhận diện hình học tạo các vùng ứng viên và đánh số lên ảnh gửi Gemini. Gemini chọn vật số 3 là chuột máy tính màu trắng (Hình 5.5). Thời gian 14,1 s bao gồm di chuyển về tư thế quan sát và chờ ảnh. Đây là phép thử phương án đánh số ứng viên; quy trình đầy đủ sử dụng MobileSAM để phân đoạn vật mục tiêu, không xem MobileSAM là thuật toán dự phòng.',
883:'Hình 5.5. Ảnh đánh số ứng viên của lần khảo sát hình học trước đó; Gemini chọn số 3 (chuột). Ảnh này không minh hoạ đầu ra MobileSAM của quy trình đầy đủ',
884:'Các vùng số 5, 6 và 7 trong Hình 5.5 là ứng viên nhầm từ ngón gripper của phương án hình học đang khảo sát. Kết quả này chỉ phản ánh hạn chế của cách tạo ứng viên trong lần thử đó. Trong chuỗi đầy đủ, MobileSAM phân đoạn vật đã chọn và mặt nạ được dùng để lấy các điểm 3D của vật trước khi đưa vào GraspNet.',
900:'Quy trình hiệu chuẩn đã được sử dụng trong cả mô phỏng và trên robot thật. Mỗi môi trường cần xác định phép biến đổi camera–robot tương ứng; không lấy trực tiếp phép biến đổi của camera ảo để điều khiển robot thật. Các kết quả kiểm tra trên tag được trình bày ở mục 5.4.1.',
906:'Các khối thuật toán đã được triển khai từ mô phỏng gồm Gemini, MobileSAM, xử lý RGB-D và chuyển hệ toạ độ sau hiệu chuẩn, GraspNet, các bước lọc tư thế, giải IK, MTC và điều khiển. Khi triển khai thật, nhóm giữ kiến trúc này và điều chỉnh tham số cùng các kiểm tra theo camera, gripper và robot thực tế. Mô phỏng đầy đủ dùng để kiểm chứng sự liên kết của chuỗi; các phép khảo sát từng phương án không thay thế mô tả chuỗi chính.',
907:'Bảng 5.5 trình bày các yếu tố phần cứng và vật lý cần kiểm tra thêm khi chuyển từ mô phỏng sang thực tế. Các điều chỉnh tập trung vào dữ liệu cảm biến, hình học gripper, driver và truyền thông; MobileSAM và hiệu chuẩn vẫn là các thành phần của quy trình chung.',
910:'Mô phỏng đã được dùng để triển khai và thử nghiệm toàn bộ chuỗi từ nhận lệnh, phân đoạn MobileSAM, xử lý 3D và hiệu chuẩn đến sinh tư thế gắp, lọc, lập kế hoạch và điều khiển. Thực nghiệm thật kiểm tra khả năng hoạt động của chuỗi đó khi có sai số cảm biến, giới hạn cơ khí, nhiệt độ và độ trễ truyền thông. Cảm biến lực hiện được thử trong mô phỏng; khi lắp cảm biến thật vẫn cần hiệu chuẩn và đánh giá đáp ứng của phần cứng.',
911:'5.2.6 Kiểm tra chuyển đổi hệ toạ độ khi triển khai robot thật',
912:'Trong một lần kiểm tra tích hợp ban đầu, nhóm thử dùng vị trí camera và bàn đo bằng thước; robot đi tới điểm chờ phía trên tua vít nhưng lệch khoảng 13–20 cm. Hướng đế robot thực tế khác với hướng được giả định khoảng 23°. Phép thử này cho thấy không thể dùng các giá trị đo tay hoặc phép biến đổi từ mô phỏng thay cho hiệu chuẩn ở cấu hình thật. Quy trình hand-eye đã được thử trong mô phỏng và tiếp tục được thực hiện để xác định phép biến đổi camera–robot trên hệ thống thật.',
918:'5.3.1 Khảo sát chất lượng độ sâu và vị trí đặt camera',
919:'Nhóm khảo sát ảnh hưởng của vị trí camera tới chất lượng dữ liệu độ sâu bằng cách tìm mặt phẳng bàn và kiểm tra các điểm nhô lên. Với camera đặt cách bàn 1,0–1,4 m và nhìn nghiêng, độ lệch chuẩn độ cao mặt bàn là 10–11 mm; các phân vị 2 và 98 xấp xỉ −23 và +23 mm sau khi lấy trung vị 15 khung. Vì vậy, tách riêng vật thấp chỉ bằng ngưỡng độ cao chưa ổn định ở cấu hình này. Đây là phép khảo sát dữ liệu cảm biến, không phải toàn bộ thuật toán nhận thức và không thay thế chuỗi Gemini–MobileSAM–GraspNet đã thử trong mô phỏng.',
924:'Nhóm điều chỉnh camera xuống khoảng 0,67 m và hướng gần vuông góc với bàn; độ lệch chuẩn độ cao mặt bàn giảm còn 1,6–1,8 mm. Dữ liệu này được tiếp tục hiệu chỉnh và sử dụng cùng mặt nạ MobileSAM để ước lượng hình học 3D trong hệ toạ độ robot.',
926:'Nhóm kiểm tra khả năng Gemini xác định vật theo câu lệnh trong chuỗi nhận thức đã dùng từ mô phỏng. Đồng thời, hai cách tính vị trí được khảo sát để đối chiếu: lấy độ sâu trung vị trong khung bao và giao tia qua tâm khung với mặt phẳng bàn. Đây là phép đánh giá riêng bước định vị; trong quy trình đầy đủ, MobileSAM tiếp tục phân đoạn vật và cung cấp mặt nạ cho xử lý 3D và GraspNet.',
1007:'Thử nghiệm trên Gazebo: đã chạy chuỗi đầy đủ với Gemini, MobileSAM, xử lý RGB-D, hiệu chuẩn camera–robot, GraspNet, lọc tư thế, IK, MTC và điều khiển. Một lần khảo sát đã lưu cho thấy chuỗi gắp chuột hoàn thành trong khoảng 95 s và thuật toán kẹp theo lực hoạt động với hai cảm biến giả lập. Các số liệu của lần khảo sát này không được dùng để suy ra thời gian hay tỷ lệ thành công của lần chạy đầy đủ mới.'
}
for i,t in patch.items():put(d.paragraphs[i],t)
# Explicitly distinguish the full pipeline from archived illustrations.
t=d.tables[27]
updates={
(2,1):'Gemini → MobileSAM → RGB-D → GraspNet; có các bước lọc tư thế',
(2,2):'Cùng chuỗi nhận thức; hiệu chỉnh dữ liệu độ sâu và tham số theo camera, gripper thật',
(2,3):'MobileSAM là thành phần chính trong cả hai môi trường',
(3,2):'Nhiễu phụ thuộc cấu hình camera; sau điều chỉnh vị trí, độ lệch chuẩn mặt bàn 1,6–1,8 mm; vật đen vẫn khó đo',
(3,3):'Kiểm tra và sửa sai số độ sâu; không thay đổi vai trò của SAM',
(4,1):'Thực hiện hiệu chuẩn camera–robot trong mô phỏng; đối chiếu với vị trí chuẩn của môi trường ảo',
(4,2):'Thực hiện lại hiệu chuẩn theo cấu hình phần cứng; kiểm tra trên tag lệch 4,6 mm',
(4,3):'Cùng quy trình; phép biến đổi và dữ liệu hiệu chuẩn riêng cho từng môi trường',
(5,1):'MTC lập kế hoạch tiếp cận, gắp, nhấc và chuyển vật; số liệu chi tiết là của lần khảo sát minh hoạ',
(7,2):'Theo vị trí servo, cộng thêm 5°',
(9,1):'Đã chạy chuỗi thuật toán đầy đủ; lần khảo sát minh hoạ hoàn thành trong khoảng 95 s',
(9,3):'Đánh giá riêng kết quả mô phỏng và mức độ hoàn thành trên phần cứng'
}
for (r,c),s in updates.items():put(t.rows[r].cells[c].paragraphs[0],s)
t=d.tables[28]
for r,c,s in [
(1,2,'Dữ liệu độ sâu ban đầu nhiễu và có sai lệch, ảnh hưởng ước lượng hình học'),
(1,3,'Điều chỉnh vị trí camera và mô hình sửa độ sâu; tiếp tục dùng chuỗi Gemini–MobileSAM–GraspNet'),
(3,1,'Thực hiện hiệu chuẩn; có vị trí chuẩn để đối chiếu'),
(3,2,'Phép thử đo tay ban đầu sai hướng đế khoảng 23°'),
(3,3,'Thực hiện hiệu chuẩn hand-eye cho cấu hình thật, theo quy trình đã thử trong mô phỏng')]:put(t.rows[r].cells[c].paragraphs[0],s)
# Remove unnecessary calendar detail from prose/captions; retain durations and measurements.
allp=list(d.paragraphs)
for table in d.tables:
 for row in table.rows:
  for cell in row.cells:allp.extend(cell.paragraphs)
for p in allp:
 if any(r.font.name=='Courier New' for r in p.runs):continue
 text=p.text
 # Captions often contain test dates/times that are not needed for interpreting the figure.
 if text.startswith('Hình '):
  text=re.sub(r'\s*\(\d{1,2}/\d{1,2}(?:,?\s*\d{1,2}:\d{2})?\)', '',text)
  text=re.sub(r'\d{1,2}/\d{1,2},?\s*\d{1,2}:\d{2}', 'lần thử minh hoạ',text)
  text=re.sub(r'\s*\(\d{1,2}/\d{1,2}\)', '',text)
 if text!=p.text:put(p,text)
# Keep table fonts compact; newly replaced cells inherit the table's existing body sizing.
for ti in [27,28]:
 for row in d.tables[ti].rows:
  for cell in row.cells:
   for p in cell.paragraphs:
    for r in p.runs:r.font.size=Pt(11)
# Update matching TOC labels without disturbing field codes.
for p in d.paragraphs:
 if p.style and p.style.name.lower().startswith('toc '):
  for r in p.runs:
   if r.text:
    text=r.text.replace('5.3.1 Nhận diện bằng hình học depth: không đạt','5.3.1 Khảo sát chất lượng độ sâu và vị trí đặt camera').replace('5.2.6 Lần đầu chuyển sang robot thật','5.2.6 Kiểm tra chuyển đổi hệ toạ độ khi triển khai robot thật')
    if text!=r.text:r.text=text
# Label archived data in corresponding table captions.
for p in d.paragraphs:
 if p.text.startswith('Bảng 5.3. '):put(p,'Bảng 5.3. Thời gian các giai đoạn trong lần khảo sát mô phỏng minh hoạ')
# Replace clock/calendar labels in the real-robot trial table with trial numbers.
for ri,row in enumerate(d.tables[33].rows[1:],1):
 put(row.cells[0].paragraphs[0],f'Lần {ri}')
 for p in row.cells[0].paragraphs:
  for r in p.runs:r.font.size=Pt(11)
for p in d.paragraphs:
 if p.style and p.style.name.lower().startswith('toc '):continue
 if re.match(r'^(Hiệu chuẩn gripper bị đảo chiều|Má đè lên vật|Kẹp quá nhẹ|Servo quá nhiệt|Wi-Fi không ổn định|Phát hiện trượt sai)',p.text):
  put(p,re.sub(r'\s*\([0-9:, ]+\)', '',p.text))
d.save(out)
print(out)
