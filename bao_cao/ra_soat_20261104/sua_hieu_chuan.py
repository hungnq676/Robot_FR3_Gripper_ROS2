from docx import Document
from docx.shared import Pt
from pathlib import Path
root=Path('/home/duong-12a/fr3_hungros2');d=Document(root/'bao_cao/Bao_cao_NCKH_FR3_ra_soat_v5.docx')
def put(p,text):
 if p.runs:
  p.runs[0].text=text
  for r in p.runs[1:]:
   if r.text:r.text=''
 else:p.add_run(text)
def insert_after(p,text):
 from docx.oxml import OxmlElement
 from docx.text.paragraph import Paragraph
 e=OxmlElement('w:p');p._p.addnext(e);q=Paragraph(e,p._parent);q.style='Normal';q.add_run(text);return q
# Keep the existing heading structure and explain the actual data collection/solve.
ps=d.paragraphs
p=next(p for p in ps if p.text.startswith('AprilTag 36h11 id 2 được dán'))
put(p,'Camera được giữ cố định, còn AprilTag 36h11 id 2 được gắn cứng lên robot. Nhóm di chuyển robot qua nhiều tư thế khác nhau, thường đề xuất khoảng 15–16 tư thế cho một đợt thu mẫu; số mẫu hợp lệ có thể ít hơn hoặc được bổ sung qua nhiều đợt. Tại mỗi tư thế, robot dừng để ghi đồng thời ảnh tag và trạng thái góc các khớp. Tập mẫu này được dùng để ước lượng phép biến đổi giữa hệ toạ độ camera và hệ toạ độ gốc robot.')
p=insert_after(p,'Trong code, tag_check.py đọc góc j1–j6 từ trạng thái khớp và lấy phép biến đổi base_footprint → wrist2_link từ TF. TF này được tính từ mô hình động học robot và góc khớp bằng động học thuận. Đồng thời, bốn góc AprilTag trên ảnh cùng thông số nội tại camera được đưa vào solvePnPGeneric với IPPE_SQUARE để ước lượng tư thế tag trong hệ camera. Khâu wrist2_link được dùng vì tag gắn trên phần nắp thuộc khâu này; cần giữ đúng quan hệ cơ khí khi thu mẫu.')
p=insert_after(p,'Mỗi mẫu gồm góc khớp, phép biến đổi của khâu mang tag trong hệ robot, tư thế tag trong hệ camera và toạ độ bốn góc tag trên ảnh. Dữ liệu được lưu trong tệp NPZ để giải và kiểm tra lại. Khi dùng cấu hình --no-depth, bộ giải hand-eye sử dụng ảnh và động học robot; độ sâu được hiệu chỉnh ở bước riêng, tránh đưa sai lệch độ sâu trực tiếp vào phép giải này.')
# Replace headings in body and cached TOC without changing field structure.
for p in d.paragraphs:
 for r in p.runs:
  if '4.6.1 Gắn tag và mô phỏng trước' in r.text:r.text=r.text.replace('4.6.1 Gắn tag và mô phỏng trước','4.6.1 Nguyên lý và dữ liệu hiệu chuẩn')
  if '4.6.2 Thu mẫu có dẫn đường' in r.text:r.text=r.text.replace('4.6.2 Thu mẫu có dẫn đường','4.6.2 Thu mẫu, giải và kiểm tra hiệu chuẩn')
# Remove the inaccurate explanation that the 0.33 m limit guarantees Astra depth.
for p in d.paragraphs:
 if p.text=='tag cách camera ít nhất 0,33 m (gần hơn thì Astra không đo được sâu);':put(p,'giới hạn khoảng cách tag–camera được đặt theo cấu hình thu mẫu; với phép giải --no-depth, mẫu được đánh giá bằng ảnh tag và trạng thái robot;')
 if p.text.startswith('Với mỗi tư thế, công cụ lập kế hoạch Pilz PTP'):
  put(p,'Với mỗi tư thế được đề xuất, handeye_guided.py dùng Pilz PTP lập kế hoạch, hiển thị đường đi trong RViz và chờ người vận hành xác nhận. Song song, tag_check.py chỉ ghi mẫu khi robot đứng yên khoảng 1 s, thấy tag và tư thế đủ khác mẫu trước. Code loại khung hình có hai nghiệm IPPE gần tương đương về sai số chiếu để giảm nguy cơ nhận sai hướng tag. Nếu chưa có biến đổi camera–robot ban đầu, tuỳ chọn --camera-from-tag cung cấp ước lượng khởi tạo để đề xuất tư thế.')
  q=insert_after(p,'Sau thu mẫu, handeye_solve.py loại các mẫu có chuyển động quay không nhất quán giữa camera và robot. Bộ giải thử các phương pháp hand-eye của OpenCV để tìm nghiệm khởi tạo, rồi tối ưu đồng thời X là biến đổi camera → gốc robot và Y là biến đổi tag → khâu mang tag bằng bình phương tối thiểu với hàm mất mát Huber. Với Aᵢ là tư thế khâu trong hệ robot và Bᵢ là tư thế tag trong hệ camera, quan hệ dùng để giải là X·Bᵢ = Aᵢ·Y. Hàm mục tiêu chính là sai số chiếu lại bốn góc tag trên ảnh; số mẫu hợp lệ được lưu cùng kết quả.')
  q=insert_after(q,'Kết quả được kiểm tra bằng sai số chiếu lại, kiểm tra chéo bỏ từng mẫu (leave-one-out) và so sánh vị trí tag suy ra từ camera với vị trí suy ra từ robot. Tệp YAML lưu phép biến đổi, thông tin khâu gắn tag và các chỉ số kiểm tra; cam_tf.py phát biến đổi lên TF để chuyển các điểm quan sát sang hệ robot. Hình 4.9 minh hoạ việc chiếu mô hình robot và vị trí tag dự đoán lên ảnh để kiểm tra sự phù hợp. Kết quả định lượng của các đợt hiệu chuẩn được trình bày tại mục 5.4.')
# Move actual real-tag image to 4.8; use an archived verification image at 4.9.
ps=d.paragraphs
for prefix,path,caption in [
 ('Hình 4.8.',root/'nhat_ky/anh/2026-10-29_ROG/1627_cam_phat_hien_tag_id2_tren_J6.jpg','Hình 4.8. Camera quan sát AprilTag gắn cố định trên robot để thu mẫu hiệu chuẩn'),
 ('Hình 4.9.',root/'nhat_ky/anh/2026-10-30_ROG/0433_kiem_chung_handeye_chieu_robot_len_anh.jpg','Hình 4.9. Kiểm tra hiệu chuẩn bằng cách chiếu mô hình robot và vị trí tag dự đoán lên ảnh camera; sai lệch chiếu tag khoảng 0,9 px trong ảnh minh hoạ')]:
 i=[i for i,p in enumerate(ps) if p.text.startswith(prefix)][-1];pic=ps[i-1]
 width=int(pic._p.xpath('.//wp:extent')[0].get('cx'))
 for r in list(pic.runs):
  if r._r.xpath('.//w:drawing'):pic._p.remove(r._r)
 pic.add_run().add_picture(str(path),width=width)
 for p in ps:
  if p.text.startswith(prefix):put(p,caption)
# Ensure added prose matches the existing main-body font.
for p in d.paragraphs:
 for r in p.runs:
  if r.font.name is None:r.font.name='Times New Roman'
out=root/'bao_cao/Bao_cao_NCKH_FR3_ra_soat_v6.docx';d.save(out);print(out)
