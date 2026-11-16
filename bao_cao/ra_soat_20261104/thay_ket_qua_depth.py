from pathlib import Path
from docx import Document
root=Path('/home/duong-12a/fr3_hungros2')
f1=root/'bao_cao/hinh/ket_qua_depth_da_hieu_chinh.png'
f2=root/'bao_cao/hinh/sam_du_lieu_da_luu.png'
d=Document(root/'bao_cao/Bao_cao_NCKH_FR3_ra_soat_v4.docx')
def replace_text(p,new):
 for r in p.runs:
  if r.text:r.text=''
 r=p.add_run(new)
 # Inherit paragraph styles for headings and table of contents.
 if new.startswith('Hình '):r.italic=not(p._p.getparent().tag.endswith('sdtContent'))
def replace_image(cap_prefix,path):
 paras=d.paragraphs;i=[i for i,p in enumerate(paras) if p.text.startswith(cap_prefix)][-1];p=paras[i-1]
 extent=p._p.xpath('.//wp:extent')[0];width=int(extent.get('cx'))
 for r in list(p.runs):
  if r._r.xpath('.//w:drawing'):p._p.remove(r._r)
 p.add_run().add_picture(str(path),width=width)
oldtitle='5.3.1 Khảo sát chất lượng độ sâu và vị trí đặt camera';newtitle='5.3.1 Kết quả dữ liệu độ sâu sau hiệu chỉnh'
for p in d.paragraphs:
 if oldtitle in p.text:
  for r in p.runs:
   if oldtitle in r.text:r.text=r.text.replace(oldtitle,newtitle)
 if p.text.startswith('Nhóm khảo sát ảnh hưởng của vị trí camera'):
  replace_text(p,'Ở cấu hình làm việc đã hiệu chuẩn, camera Astra cung cấp ảnh RGB-D để phân đoạn vật, dựng đám mây điểm và ước lượng tư thế gắp. Nhóm kiểm tra mô hình hiệu chỉnh độ sâu bằng mặt bàn chuẩn trong hệ toạ độ robot. Nhật ký ghi nhận hai đợt: đợt A dùng 19 mẫu hand-eye, RMS khớp mặt bàn 2,9 mm và độ nghiêng còn 0,02°; đợt B dùng 23 mẫu hand-eye, RMS 2,1 mm và độ nghiêng còn 0,01° (Hình 5.10). Các giá trị này mô tả độ khớp dữ liệu mặt bàn sau hiệu chỉnh, không đại diện cho sai số gắp vật hoặc độ chính xác ở mọi điểm trong vùng làm việc.')
 if p.text.startswith('Nhóm điều chỉnh camera xuống khoảng 0,67 m'):
  replace_text(p,'Trong chuỗi nhận thức, Gemini xác định vật mục tiêu, MobileSAM tạo mặt nạ và các pixel có độ sâu hợp lệ được chuyển thành điểm 3D trong hệ toạ độ robot sau hiệu chuẩn. Hình 5.11 minh hoạ mặt nạ MobileSAM được chạy lại trên ảnh và khung bao đã lưu của một lần thử; đây là kết quả xử lý lại dữ liệu thực nghiệm, không phải một lần gắp mới. Đám mây điểm sau đó được dùng để ước lượng hình học và đưa vào GraspNet. Việc kiểm tra kích thước, tư thế gắp và kết quả robot thực hiện được trình bày ở các mục tiếp theo.')
 if p.text.startswith('Hình 5.10.'):
  replace_text(p,'Hình 5.10. Kết quả khớp mặt bàn sau hiệu chỉnh độ sâu Astra ở hai đợt hiệu chuẩn; số liệu trích từ nhật ký thực nghiệm')
 if p.text.startswith('Hình 5.11.'):
  replace_text(p,'Hình 5.11. Ảnh thực nghiệm và khung bao mục tiêu đã lưu (trái); mặt nạ MobileSAM chạy lại trên dữ liệu đó (phải)')
replace_image('Hình 5.10.',f1);replace_image('Hình 5.11.',f2)
out=root/'bao_cao/Bao_cao_NCKH_FR3_ra_soat_v5.docx';d.save(out);print(out)
