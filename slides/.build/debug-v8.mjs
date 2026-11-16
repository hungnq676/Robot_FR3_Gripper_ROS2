import fs from 'node:fs/promises';
import {Presentation,PresentationFile} from '@oai/artifact-tool';
console.log('imported');
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
 ['Lý do chọn đề tài, bài toán và mục tiêu','Trình bày lý do lựa chọn, xác định bài toán, phạm vi và yêu cầu'],
 ['Ý tưởng và thiết kế hệ thống','Đề xuất giải pháp tổng thể và giải thích các lựa chọn thiết kế'],
 ['Phương pháp thực hiện','Trình bày cơ sở, thuật toán và cách triển khai giải pháp'],
 ['Thực nghiệm và đánh giá','Kiểm chứng giải pháp, phân tích kết quả và giới hạn'],
 ['Kết luận','Đối chiếu mục tiêu, xác định đóng góp và những vấn đề còn lại']
];
rows.forEach(([title,desc],i)=>{let y=236+i*82;tx(String(i+1).padStart(2,'0'),64,y+3,77,49,34,'#B5121B',true);tx(title,166,y,1040,38,27,'#252525',true);tx(desc,166,y+39,1040,32,21,'#555555');if(i<4)rect(166,y+75,1050,1,'#EBEBEB');});
rect(0,674,1280,46,'#B5121B');
tx('NGHIÊN CỨU KHOA HỌC SINH VIÊN',64,687,1000,27,17,'#FFFFFF');tx('02',1160,685,60,30,19,'#FFFFFF',true);
s.speakerNotes.text='Giới thiệu năm phần của bài trình bày. Mở đầu xác định bài toán và mục tiêu; tiếp theo trình bày ý tưởng, thiết kế và phương pháp. Phần thực nghiệm cung cấp bằng chứng đánh giá; kết luận đối chiếu kết quả với mục tiêu. Nội dung theo cấu trúc người dùng đã chốt. Logo: https://ptit.edu.vn/wp-content/uploads/2024/05/logo-ptit-1.svg';

function header(title,page){s=p.slides.add();s.background.fill='#FFFFFF';rect(0,0,1280,12,'#B5121B');s.images.add({blob:b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength),contentType:'image/svg+xml',alt:'Logo PTIT',fit:'contain',position:{left:690,top:36,width:530,height:62}});tx('KHOA KỸ THUẬT ĐIỆN TỬ 1',64,51,570,35,21,'#555555',true);rect(64,122,1152,2,'#E6E6E6');tx(title,64,153,1152,62,40,'#B5121B',true);rect(0,674,1280,46,'#B5121B');tx('NGHIÊN CỨU KHOA HỌC SINH VIÊN',64,687,1000,27,17,'#FFFFFF');tx(page,1160,685,60,30,19,'#FFFFFF',true);}
async function photo(name,x,y,w,h){const data=await fs.readFile('/home/duong-12a/fr3_hungros2/bao_cao/hinh/'+name);s.images.add({blob:data.buffer.slice(data.byteOffset,data.byteOffset+data.byteLength),contentType:name.endsWith('.png')?'image/png':'image/jpeg',alt:name,fit:'contain',position:{left:x,top:y,width:w,height:h}});}
header('LÝ DO CHỌN ĐỀ TÀI','03');
tx('Giới hạn của cách gắp dựa trên thông tin biết trước',64,228,1152,39,25,'#555555');
const reasons=[['Vật và cách gắp được biết trước','Thuận lợi với vật hình học đơn giản, vị trí và cách tiếp cận đã được xác định.'],['Hạn chế khi đối tượng thay đổi','Khi vật đổi hình dạng hoặc vị trí, cần xác định lại điểm kẹp và hướng tiếp cận.'],['Vấn đề nhóm hướng tới','Robot nhận biết nhiều loại vật và tự tính cách gắp theo yêu cầu bằng ngôn ngữ tự nhiên.']];
reasons.forEach(([title,desc],i)=>{let y=294+i*113;rect(64,y+7,5,76,'#B5121B');tx(title,87,y,550,38,23,'#252525',true);tx(desc,87,y+41,550,70,21,'#555555');});
const ill=await fs.readFile(root+'/assets/robot_cong_nghiep_nguoi_dung.png');s.images.add({blob:ill.buffer.slice(ill.byteOffset,ill.byteOffset+ill.byteLength),contentType:'image/png',alt:'Ảnh minh họa robot công nghiệp do người dùng cung cấp',fit:'contain',position:{left:681,top:298,width:535,height:312}});
tx('Minh họa robot thực hiện thao tác gắp',681,615,535,32,19,'#555555');
s.speakerNotes.text='Giới hạn phát biểu: đang nói về cách gắp dựa trên thông tin biết trước, không khẳng định mọi robot hiện nay chỉ gắp được vật đơn giản. Các hệ thống gắp tổng quát và vision-guided đã tồn tại: https://graspnet.net/ ; https://docs.universal-robots.com/AI_Accelerator_Documentation/examples/dynamic_picking.html . Lý do chọn đề tài: tác vụ dạy trước gắn với bố trí đã biết; khi thay đổi vật hoặc vị trí, có thể phải cập nhật điểm thao tác. Không khẳng định robot hiện đại chỉ gắp được hình đơn giản. Nhu cầu của nhóm là kết hợp thích ứng với vật và cách chỉ định bằng ngôn ngữ tự nhiên. Ảnh minh họa robot công nghiệp do người dùng cung cấp: slides/assets/robot_cong_nghiep_nguoi_dung.png. Ảnh không phải hệ thống thực nghiệm của nhóm và không chứng minh giới hạn của robot trong ảnh; nội dung xây dựng theo trao đổi đã thống nhất với người dùng.';
header('BÀI TOÁN VÀ MỤC TIÊU','04');
tx('BÀI TOÁN NHÓM ĐẶT RA',64,242,660,38,25,'#B5121B',true);
tx('Robot tiếp nhận yêu cầu bằng ngôn ngữ tự nhiên, xác định vật được chỉ định và tự động gắp nhiều loại vật thể khác nhau.',64,294,650,113,26);
tx('MỤC TIÊU NGHIÊN CỨU',64,445,660,38,25,'#B5121B',true);
tx('Xây dựng thuật toán kết hợp thị giác, lựa chọn tư thế gắp và lập kế hoạch chuyển động; kiểm chứng trong mô phỏng, sau đó triển khai và đánh giá trên robot thật.',64,497,650,153,25);
const gz=await fs.readFile(root+'/assets/fr3_gazebo_nguoi_dung.png');s.images.add({blob:gz.buffer.slice(gz.byteOffset,gz.byteOffset+gz.byteLength),contentType:'image/png',alt:'FR3 trong Gazebo, ảnh người dùng cung cấp',fit:'contain',position:{left:780,top:234,width:436,height:183}});tx('Kiểm chứng trong mô phỏng',780,421,436,29,19,'#555555');
await photo('p_nhac_gia.jpg',780,464,436,153);tx('Triển khai trên robot thật',780,624,436,29,19,'#555555');
s.speakerNotes.text='Phát biểu mục tiêu hướng tới, không coi là kết quả đã hoàn thành. Nhiều loại vật được hiểu trong phạm vi hình học và hành trình gripper, vùng robot với tới và khả năng quan sát của camera. Yêu cầu bằng ngôn ngữ tự nhiên có thể được nhập theo giao diện đang dùng; không đồng nhất việc hiểu yêu cầu với việc nhận dạng giọng nói trên mọi cấu hình. Mục tiêu là xây dựng chuỗi thuật toán nhận thức, tư thế gắp, lập kế hoạch; kiểm chứng mô phỏng rồi triển khai robot thật. Nguồn ảnh: bao_cao/hinh/mo_phong/toan_canh.png và bao_cao/hinh/p_nhac_gia.jpg. Nội dung theo yêu cầu người dùng.';


console.log('first four ready');
header('Ý TƯỞNG PHÁT TRIỂN HỆ THỐNG','05');
tx('Từ bài toán đã đặt ra, nhóm xác định ba câu hỏi cần giải quyết:',64,232,1152,48,26,'#555555');
const questions=[
 ['01','Hiểu yêu cầu và xác định vật mục tiêu','Làm sao để robot biết người dùng muốn gắp vật nào và vật đang ở đâu?'],
 ['02','Ước lượng hình học và lựa chọn cách gắp','Làm sao ước lượng kích thước, hình dạng vật và xác định tư thế gắp phù hợp?'],
 ['03','Lập kế hoạch và thực thi chuyển động','Làm sao tạo kế hoạch chuyển động và chuyển kế hoạch thành thao tác thực tế?']
];
questions.forEach(([num,title,question],i)=>{let y=305+i*105;tx(num,64,y,85,50,34,'#B5121B',true);tx(title,170,y,1045,38,27,'#252525',true);tx(question,170,y+44,1045,56,23,'#555555');if(i<2)rect(170,y+97,1045,1,'#E6E6E6');});
rect(64,633,5,26,'#B5121B');tx('Tiếp theo: các phương pháp và thuật toán nhóm đã thử nghiệm cho từng câu hỏi',83,632,1130,30,21,'#B5121B');
s.speakerNotes.text='Từ bài toán robot gắp nhiều loại vật theo yêu cầu ngôn ngữ tự nhiên, nhóm phân tách thành ba câu hỏi liên kết. Thứ nhất: xác định vật cần lấy và vị trí của vật. Thứ hai: ước lượng hình học và cách gắp phù hợp. Thứ ba: lập kế hoạch và thực thi trên robot thật. Đây là định hướng tổ chức giải pháp, chưa khẳng định đã giải quyết hoàn toàn. Các slide tiếp theo trình bày những phương pháp, thuật toán đã thử nghiệm, lý do lựa chọn và đầu ra tương ứng. Nội dung theo hướng dẫn người dùng. Logo: https://ptit.edu.vn/wp-content/uploads/2024/05/logo-ptit-1.svg';
console.log('exporting');
await(await PresentationFile.exportPptx(p)).save(root+'/.build/slide-1-5-v8.pptx');
await fs.writeFile(root+'/output/FR3_slide_05_v8.png',new Uint8Array(await(await p.export({slide:s,format:'png',scale:1})).arrayBuffer()));
