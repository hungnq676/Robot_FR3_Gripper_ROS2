from docx import Document
from docx.shared import Pt,Cm
from docx.oxml import OxmlElement
from docx.text.paragraph import Paragraph
from docx.oxml.ns import qn
src='bao_cao/Bao_cao_NCKH_FR3_ra_soat_v2.docx';out='bao_cao/Bao_cao_NCKH_FR3_ra_soat_v3.docx'
d=Document(src)
def setcell(c,s):
 p=c.paragraphs[0]
 for r in p.runs:r.text=''
 r=p.add_run(s);r.font.name='Times New Roman';r.font.size=Pt(10)
def note_after(table,text):
 e=OxmlElement('w:p');table._tbl.addnext(e);p=Paragraph(e,table._parent)
 r=p.add_run(text);r.italic=True;r.font.name='Times New Roman';r.font.size=Pt(11)
 p.paragraph_format.space_after=Pt(6)
for p in list(d.paragraphs):
 if p.text.startswith('Hiệu chuẩn gripper bị đảo chiều.'):
  p._element.getparent().remove(p._element)
t=d.tables[30]
for row in t.rows[1:]:
 for c in row.cells:
  if c.text.strip()=='–':setcell(c,'Không ghi nhận')
setcell(t.rows[1].cells[0],'Giá trắng đứng – lần 1')
setcell(t.rows[2].cells[0],'Giá trắng đứng – lần 2')
setcell(t.rows[3].cells[0],'Tay cầm game')
# No valid object depth: do not imply that GraspNet actually evaluated candidates.
for c in t.rows[5].cells[1:]:setcell(c,'Không áp dụng')
widths=[4.0,2.1,2.1,2.6,2.6,2.6]
t.autofit=False
for col,w in zip(t.columns,widths):col.width=Cm(w)
for row in t.rows:
 for c,w in zip(row.cells,widths):
  c.width=Cm(w)
  for p in c.paragraphs:
   for r in p.runs:r.font.size=Pt(10)
note_after(t,'Ghi chú: “Không ghi nhận” là bước không có số đếm trong dữ liệu đã lưu, không đồng nghĩa với 0 tư thế. “Không áp dụng” ở khay đen là trường hợp không đủ dữ liệu độ sâu của vật để đánh giá theo chuỗi GraspNet; phương án giả định chiều cao được trình bày riêng ở mục 4.7.3. Các con số còn lại được giữ theo kết quả đã ghi nhận.')
t=d.tables[32]
setcell(t.rows[2].cells[3],'Dưới 3 mm khi đối chiếu mặt bích–mặt bàn; chưa kiểm tra riêng độ sâu trên vật')
setcell(t.rows[3].cells[3],'Khoảng 62 mm (suy ra từ hình học và phép đo trong lần thử)')
setcell(t.rows[4].cells[3],'Chưa ghi nhận phép đo độ cao độc lập')
setcell(t.rows[5].cells[2],'Chưa ghi nhận phép đo ngang độc lập')
setcell(t.rows[5].cells[3],'Đầu ngón chạm bàn đúng dự tính (kiểm tra định tính)')
setcell(t.rows[0].cells[0],'Lần thử')
for ri,row in enumerate(t.rows[1:],1):setcell(row.cells[0],str(ri))
note_after(t,'Ghi chú: các giá trị lệch ngang là kết quả đo hoặc ước lượng trong từng lần thử. Giá trị 62 mm là sai lệch độ cao suy ra trong nhật ký, không phải một phép đo độc lập mới. Kiểm tra chạm bàn là kiểm tra định tính; không quy đổi thành sai số 0 mm.')
# Omit the same incident from the trial summary to match the requested scope.
t=d.tables[33]
for row in list(t.rows[1:]):
 if 'đảo chiều' in ' '.join(c.text for c in row.cells):t._tbl.remove(row._tr)
for i,row in enumerate(t.rows[1:],1):setcell(row.cells[0],f'Lần {i}')
d.save(out)
print(out)
