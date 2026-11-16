from docx import Document
from docx.shared import Pt
from docx.oxml.ns import qn
from pathlib import Path
import re, shutil
src=Path('/home/duong-12a/Music/bao_cao_da_sua.docx')
out=Path('/home/duong-12a/fr3_hungros2/bao_cao/Bao_cao_NCKH_FR3_ra_soat.docx')
shutil.copy2(src,out.parent/'ra_soat_20261104/ban_goc_Music.docx')
d=Document(src); log=[]
def replace(p,old,new):
 text=p.text
 if old not in text:return
 start=text.index(old);end=start+len(old);pos=0; done=False
 for r in p.runs:
  a,b=pos,pos+len(r.text);pos=b
  if b<=start or a>=end:continue
  prefix=r.text[:max(0,start-a)];suffix=r.text[max(0,end-a):] if end<b else ''
  r.text=prefix+(new if not done else '')+suffix;done=True
 log.append((old,new))
patches=[
('Những công cụ này là nền tảng để một nhóm sinh viên có thể xây dựng được một hệ thống gắp vật hoàn chỉnh trong thời gian ngắn.','Những công cụ này là nền tảng giúp nhóm xây dựng và thử nghiệm hệ thống gắp vật trong phạm vi đề tài.'),
('Tuy nhiên, phần lớn các nghiên cứu kết hợp robot cộng tác, camera, ROS 2 và MoveIt 2 cho bài toán gắp vật tự động vẫn dừng ở mô phỏng, chưa được thử nghiệm đầy đủ trên robot thật.','Đối với nhóm, khó khăn đặt ra là tích hợp các thành phần nhận thức, lập kế hoạch và điều khiển, sau đó kiểm chứng khả năng hoạt động khi chuyển từ mô phỏng sang robot thật.'),
('Đề cương đặt ra ba kết quả dự kiến. Cả ba đã đạt được:','Đề cương đặt ra ba kết quả dự kiến. Nhóm đã xây dựng và thử nghiệm các thành phần chính; một số yêu cầu trên robot thật mới đạt một phần và cần tiếp tục hoàn thiện. Kết quả hiện tại gồm:'),
('Robot tự giải động học ngược, sinh quỹ đạo tránh va chạm, đi tới vị trí gắp với sai lệch 0–3 mm, và chạy trọn kịch bản HOME → gắp → bỏ vào hộp → về HOME (mục 4.9, 5.5).','Hệ thống đã giải động học ngược, lập kế hoạch có kiểm tra va chạm và thử nghiệm chuỗi HOME → gắp → bỏ vào hộp → về HOME. Một lần thử đã chạy hết chuỗi nhưng vật còn rơi khoảng 8 cm vào hộp do servo quá nhiệt; kết quả này chưa chứng minh hệ thống đặt vật ổn định (mục 4.9, 5.5).'),
('Phần lớn các nghiên cứu kết hợp robot cộng tác, camera, ROS 2 và MoveIt 2 cho bài toán gắp vật tự động vẫn dừng ở mô phỏng hoặc thử nghiệm đơn giản, chưa được kiểm chứng đầy đủ trên robot thật. Đây là khoảng trống mà đề tài hướng tới.','Trong phạm vi đề tài, nhóm tập trung tích hợp các thành phần này và đánh giá những khó khăn khi chuyển từ mô phỏng sang robot thật. Nhóm chưa thực hiện khảo sát hệ thống đủ rộng để kết luận về mức độ triển khai của toàn bộ lĩnh vực.'),
('Đây là bài toán AX = YB kinh điển. Với N ≥ 3 tư thế có hướng quay đủ khác nhau, X và Y được xác định duy nhất.','Đây là bài toán hiệu chuẩn đồng thời hai phép biến đổi chưa biết X và Y, với quan hệ X·Bᵢ = Aᵢ·Y theo ký hiệu ở trên. Cần thu thập nhiều tư thế có hướng quay đa dạng để giảm suy biến và đánh giá độ tin cậy của nghiệm; chỉ số lượng mẫu không bảo đảm nghiệm duy nhất.'),
('L_dice = 1 − 2·|M ∩ M| / (|M| + |M|)','L_dice = 1 − 2·|P ∩ G| / (|P| + |G|)'),
('mặt nạ có điểm IoU dự đoán cao nhất được giữ lại','mã hiện tại lấy mặt nạ đầu tiên do bộ dự đoán trả về (masks.data[0]), không thực hiện bước tự xếp hạng các mặt nạ theo IoU'),
('Mặt nạ cho biết chính xác những pixel nào thuộc vật','Mặt nạ ước lượng những pixel thuộc vật'),
('Phần mềm của phương án này đã được xây dựng và kiểm chứng đầy đủ trong mô phỏng','Phần mềm của phương án này đã được xây dựng và thử nghiệm trong mô phỏng'),
('là thuật toán kẹp theo lực dùng được ngay','để tích hợp với thuật toán kẹp theo lực; vẫn cần hiệu chuẩn cảm biến, kiểm tra độ trễ và thử nghiệm trên phần cứng thật'),
('Nếu có từ 20 điểm như vậy trở lên','Nếu có trên 20 điểm như vậy'),
('Ở tất cả các lần robot chuyển động, TCP thật luôn tới đúng vị trí đã lập kế hoạch với sai lệch 0–3 mm, đọc từ TF. Như vậy chuỗi hiệu chuẩn, mô hình độ sâu, URDF và driver đã đủ chính xác cho bài toán. Các lần không thành công đều do gripper hoặc do cách bố trí vật.','Trong các lần chuyển động được ghi nhận, sai lệch vị trí TCP so với đích kế hoạch, tính từ dữ liệu TF của hệ thống, nằm trong khoảng 0–3 mm. Đây là sai lệch trong mô hình và dữ liệu trạng thái robot, chưa phải phép đo độc lập vị trí đầu ngón ngoài thực tế và chưa đủ để kết luận độ chính xác của toàn bộ chuỗi camera – robot. Các lỗi đã quan sát gồm kẹp không chắc, quá nhiệt, mất liên lạc và bố trí vật bất lợi; chưa loại trừ các nguồn sai số khác.'),
('Phần định vị và lập kế hoạch đã đạt yêu cầu và cho kết quả lặp lại tốt. Phần còn yếu là gripper:', 'Các thử nghiệm bước đầu cho thấy phần định vị và lập kế hoạch có thể hoạt động trong phạm vi đã khảo sát; cần thêm phép đo độc lập và thử nghiệm lặp lại để đánh giá độ chính xác và độ ổn định. Các hạn chế quan sát rõ nhất nằm ở gripper:'),
('Chuỗi nhận thức hoàn chỉnh:','Chuỗi nhận thức đã được tích hợp và thử nghiệm:'),
('Bảy bộ lọc theo hình học thật của gripper loại bỏ các tư thế không thực hiện được.','Hệ thống sử dụng 10 tầng lọc và kiểm tra, gồm điều kiện hình học của gripper, động học và lập kế hoạch (Bảng 4.13), nhằm loại các tư thế không phù hợp.'),
('Mô phỏng đầy đủ trên Gazebo:','Thử nghiệm trên Gazebo:'),
('Thực nghiệm trên robot thật: robot gắp và nhấc thành công giá in 3D hai lần, chạy hết kịch bản bỏ vào hộp một lần, và ở mọi lần chuyển động đều tới đúng vị trí đã lập kế hoạch với sai lệch 0–3 mm.','Thực nghiệm trên robot thật: robot gắp và nhấc thành công giá in 3D hai lần. Một lần thử chạy hết chuỗi bỏ vào hộp nhưng vật còn rơi khoảng 8 cm do servo quá nhiệt. Sai lệch TCP 0–3 mm được tính từ TF, chưa phải kết quả đo vị trí độc lập. Số lần thử còn ít, chưa đủ đánh giá độ ổn định hoặc khả năng tổng quát.'),
('Dưới 20 điểm của vật nằm ngay chỗ má sẽ đi xuống','Không quá 20 điểm của vật nằm ngay chỗ má sẽ đi xuống'),
('Chính xác tuyệt đối','Theo trạng thái robot trong mô phỏng; chưa đánh giá sai số độc lập'),
('TCP lệch 0–3 mm sau khi sửa hàng đợi ServoJ','Sai lệch TCP theo TF: 0–3 mm sau khi sửa hàng đợi ServoJ'),
('Hand-eye lệch 4,6 mm trên tag; chỉ điểm dưới 10 mm; TCP tới đích lệch 0–3 mm','Kiểm tra trên tag lệch 4,6 mm; thử chỉ điểm dưới 10 mm trong các trường hợp đã đo. Sai lệch TCP theo TF 0–3 mm chưa thay thế phép đo độc lập'),
('7 bộ lọc; sau khi bổ sung không còn va chạm','10 tầng lọc và kiểm tra (Bảng 4.13); chưa ghi nhận thêm va chạm trong các lần thử được báo cáo sau khi bổ sung'),
('đã là một nút ROS 2 qua micro-ROS','được thiết kế tích hợp thành một nút ROS 2 qua micro-ROS; các thực nghiệm ở Chương 5 sử dụng cổng HTTP chẩn đoán'),
]
allp=list(d.paragraphs)
for t in d.tables:
 for row in t.rows:
  for c in row.cells:allp.extend(c.paragraphs)
for p in allp:
 for old,new in patches:replace(p,old,new)
 # Preserve code, equations and TOC tabs. Normalize accidental prose spacing only.
 mono=any(r.font.name in ('Courier New','Consolas') for r in p.runs)
 if not mono and not re.search(r'\(\d+\.\d+\)$',p.text.strip()):
  for r in p.runs:
   if '  ' in r.text: r.text=re.sub(r' {2,}',' ',r.text)
 for r in p.runs:
  font='Courier New' if r.font.name in ('Courier New','Consolas') else 'Times New Roman'
  r.font.name=font
  for attr in ['ascii','hAnsi','eastAsia','cs']:r._element.get_or_add_rPr().rFonts.set(qn('w:'+attr),font)
 # Keep existing hierarchy, page breaks, paragraph spacing and tables.
for s in d.styles:
 if s.type==1:
  s.font.name='Times New Roman'
# clarify equation symbols without adding pages unnecessarily
for p in d.paragraphs:
 if p.text.startswith('L_mask ='):
  note=__import__('docx').oxml.OxmlElement('w:p'); p._p.addnext(note); from docx.text.paragraph import Paragraph; np=Paragraph(note,p._parent); np.add_run('Trong đó P là mặt nạ dự đoán và G là mặt nạ chuẩn.').font.size=Pt(12)
# cautious criterion evaluation based on limited tests
for rowidx in [1,2,3,4,7,8]:
 c=d.tables[34].rows[rowidx].cells[-1]
 for p in c.paragraphs:replace(p,'Đạt','Đạt trong phạm vi thử nghiệm')
d.settings.element.append(__import__('docx').oxml.OxmlElement('w:updateFields'))
d.settings.element[-1].set(qn('w:val'),'true')
d.save(out)
Path('bao_cao/ra_soat_20261104/nhat_ky_sua.txt').write_text('\n\n'.join(a+'\n→ '+b for a,b in log))
print(out, 'replacements',len(log))
