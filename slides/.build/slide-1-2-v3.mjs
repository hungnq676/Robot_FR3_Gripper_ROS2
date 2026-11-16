import fs from 'node:fs/promises';
import {Presentation,PresentationFile} from '@oai/artifact-tool';
const root='/home/duong-12a/fr3_hungros2/slides';
const p=Presentation.create({slideSize:{width:1280,height:720}});let s=p.slides.add();s.background.fill='#FFFFFF';
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

s=p.slides.add();s.background.fill='#FFFFFF';
rect(0,0,1280,12,'#B5121B');
s.images.add({blob:b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength),contentType:'image/svg+xml',alt:'Logo chính thức PTIT',fit:'contain',position:{left:690,top:36,width:530,height:62}});
tx('KHOA KỸ THUẬT ĐIỆN TỬ 1',64,51,570,35,21,'#555555',true);
rect(64,122,1152,2,'#E6E6E6');
tx('NỘI DUNG TRÌNH BÀY',64,153,1152,59,40,'#B5121B',true);
const rows=[
 ['Bài toán và mục tiêu','Xác lập vấn đề, ý nghĩa, phạm vi và yêu cầu cần giải quyết'],
 ['Ý tưởng và thiết kế hệ thống','Đề xuất giải pháp tổng thể và giải thích các lựa chọn thiết kế'],
 ['Phương pháp thực hiện','Trình bày cơ sở, thuật toán và cách triển khai giải pháp'],
 ['Thực nghiệm và đánh giá','Kiểm chứng giải pháp, phân tích kết quả và giới hạn'],
 ['Kết luận','Đối chiếu mục tiêu, xác định đóng góp và những vấn đề còn lại']
];
rows.forEach(([title,desc],i)=>{let y=236+i*82;tx(String(i+1).padStart(2,'0'),64,y+3,77,49,34,'#B5121B',true);tx(title,166,y,1040,38,27,'#252525',true);tx(desc,166,y+39,1040,32,21,'#555555');if(i<4)rect(166,y+75,1050,1,'#EBEBEB');});
rect(0,674,1280,46,'#B5121B');
tx('NGHIÊN CỨU KHOA HỌC SINH VIÊN',64,687,1000,27,17,'#FFFFFF');tx('02',1160,685,60,30,19,'#FFFFFF',true);
s.speakerNotes.text='Giới thiệu năm phần của bài trình bày. Mở đầu xác định bài toán và mục tiêu; tiếp theo trình bày ý tưởng, thiết kế và phương pháp. Phần thực nghiệm cung cấp bằng chứng đánh giá; kết luận đối chiếu kết quả với mục tiêu. Nội dung theo cấu trúc người dùng đã chốt. Logo: https://ptit.edu.vn/wp-content/uploads/2024/05/logo-ptit-1.svg';
await(await PresentationFile.exportPptx(p)).save(root+'/.build/slide-1-2-v3.pptx');
await fs.writeFile(root+'/output/FR3_slide_02_noi_dung.png',new Uint8Array(await(await p.export({slide:s,format:'png',scale:1})).arrayBuffer()));
