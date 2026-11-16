import fs from 'node:fs/promises';
import {Presentation,PresentationFile} from '@oai/artifact-tool';
const root='/home/duong-12a/fr3_hungros2/slides';
const p=Presentation.create({slideSize:{width:1280,height:720}});const s=p.slides.add();s.background.fill='#FFFFFF';
function rect(x,y,w,h,fill){s.shapes.add({geometry:'rect',position:{left:x,top:y,width:w,height:h},fill,line:{fill:'none',width:0}})}
function tx(t,x,y,w,h,size,color='#252525',bold=false){let q=s.shapes.add({geometry:'textbox',position:{left:x,top:y,width:w,height:h},fill:'none',line:{fill:'none',width:0}});q.text=t;q.text.style={typeface:'DejaVu Sans',fontSize:size,bold,color};}
rect(0,0,1280,12,'#B5121B');
const b=await fs.readFile(root+'/assets/ptit_chinh_thuc.svg');s.images.add({blob:b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength),contentType:'image/svg+xml',alt:'Logo Học viện Công nghệ Bưu chính Viễn thông từ website PTIT',fit:'contain',position:{left:690,top:36,width:530,height:62}});
tx('KHOA KỸ THUẬT ĐIỆN TỬ 1',64,51,570,35,21,'#555555',true);
rect(64,122,1152,2,'#E6E6E6');
tx('NGHIÊN CỨU KHOA HỌC SINH VIÊN',64,159,1145,42,26,'#B5121B',true);
tx('ỨNG DỤNG CÁNH TAY ROBOT',64,225,1150,65,46,'#B5121B',true);
tx('TRONG BÀI TOÁN GẮP VẬT THỂ',64,294,1150,65,46,'#B5121B',true);
tx('DỰA TRÊN THỊ GIÁC',64,363,1150,65,46,'#B5121B',true);
rect(64,451,86,5,'#B5121B');
tx('NHÓM THỰC HIỆN',64,487,690,32,19,'#B5121B',true);
tx('Nguyễn Duy Đông  ·  B22DCDT088\nNguyễn Quang Hưng  ·  B23DCDK061\nNguyễn Văn Hải  ·  B23DCDK045',64,530,690,105,22);
tx('GIẢNG VIÊN HƯỚNG DẪN',815,487,410,32,19,'#B5121B',true);
tx('ThS. Đinh Quang Ngọc',815,530,410,45,23);
rect(0,674,1280,46,'#B5121B');tx('HÀ NỘI, THÁNG 11 NĂM 2026',64,687,1000,28,17,'#FFFFFF');
s.speakerNotes.text='Trang bìa chỉnh theo hướng dẫn của người dùng: đỏ và trắng; tên hiển thị do người dùng yêu cầu. Tên đề tài trong hồ sơ gốc khác tên hiển thị này, cần thống nhất trước bản nộp. Thông tin nhóm từ bao_cao/noi_dung/thong_tin.json. Logo: https://ptit.edu.vn/wp-content/uploads/2024/05/logo-ptit-1.svg';
await(await PresentationFile.exportPptx(p)).save(root+'/.build/cover-v2.pptx');await fs.writeFile(root+'/.build/cover-v2.png',new Uint8Array(await(await p.export({slide:s,format:'png',scale:1})).arrayBuffer()));
