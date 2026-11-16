import fs from 'node:fs/promises';import {FileBlob,PresentationFile} from '@oai/artifact-tool';
const root='/home/duong-12a/fr3_hungros2/slides';const p=await PresentationFile.importPptx(await FileBlob.load(root+'/output/FR3_slide_01_15_v15b.pptx'));let s;const red='#B5121B';
function tx(t,x,y,w,h,size=24,color='#252525',bold=false){let q=s.shapes.add({geometry:'textbox',position:{left:x,top:y,width:w,height:h},fill:'none',line:{fill:'none',width:0}});q.text=t;q.text.style={typeface:'DejaVu Sans',fontSize:size,bold,color};}
function rect(x,y,w,h,fill){s.shapes.add({geometry:'rect',position:{left:x,top:y,width:w,height:h},fill,line:{fill:'none',width:0}})}
const logo=await fs.readFile(root+'/assets/ptit_chinh_thuc.svg');function header(title,num){s=p.slides.add();s.background.fill='#FFFFFF';rect(0,0,1280,12,red);s.images.add({blob:logo.buffer.slice(logo.byteOffset,logo.byteOffset+logo.byteLength),contentType:'image/svg+xml',alt:'Logo PTIT',fit:'contain',position:{left:690,top:36,width:530,height:62}});tx('KHOA KỸ THUẬT ĐIỆN TỬ 1',64,51,570,35,21,'#555555',true);rect(64,122,1152,2,'#E6E6E6');tx(title,64,153,1152,60,38,red,true);rect(0,674,1280,46,red);tx('NGHIÊN CỨU KHOA HỌC SINH VIÊN',64,687,1000,27,17,'#FFFFFF');tx(num,1160,685,60,30,19,'#FFFFFF',true);}

async function photo(name,x,y,w,h){const d=await fs.readFile('/home/duong-12a/fr3_hungros2/bao_cao/hinh/'+name);s.images.add({blob:d.buffer.slice(d.byteOffset,d.byteOffset+d.byteLength),contentType:name.endsWith('.png')?'image/png':'image/jpeg',alt:name,fit:'contain',position:{left:x,top:y,width:w,height:h}});}


async function asset(name,x,y,w,h){const d=await fs.readFile(root+'/assets/'+name);s.images.add({blob:d.buffer.slice(d.byteOffset,d.byteOffset+d.byteLength),contentType:name.endsWith('.png')?'image/png':'image/jpeg',alt:name,fit:'contain',position:{left:x,top:y,width:w,height:h}});}


const keep=(await p.inspect({select:'$.slides[*]',include:['id']})).records.slice(0,13).map(r=>r.id);p.slides.keep(keep);
header('SƠ ĐỒ TỔNG THỂ: TỪ CAMERA ĐẾN THAO TÁC','14');
function node(t,x,y,w,h,size=21){rect(x,y,w,h,'#FFF3F3');tx(t,x+9,y+5,w-18,h-4,size,red,true);}
function arrow(x,y){tx('↓',x,y,38,33,25,red,true);}
// Camera icon is an editable schematic, not a hardware photograph.
rect(423,231,122,41,'#303030');for(let x of [444,481,518])s.shapes.add({geometry:'ellipse',position:{left:x,top:241,width:20,height:20},fill:'#7E8992',line:{fill:'#D4DADE',width:1}});rect(475,272,20,9,'#555555');
tx('Camera RGB-D',415,283,150,27,17,'#555555');
await photo('ghep_home.jpg',594,226,231,83);tx('Ảnh màu + depth',614,310,211,27,17,'#555555');tx('→',551,249,40,38,27,red,true);
node('Yêu cầu người dùng\n“Gắp cái giá trắng”',64,237,293,67,20);rect(208,305,2,81,red);rect(208,385,120,2,red);tx('→',324,370,35,35,25,red,true);
arrow(621,331);
node('Gemini API → chọn vật · MobileSAM → mặt nạ',358,365,564,43,19);
node('Depth + hiệu chuẩn → đám điểm hệ robot',358,425,564,43,19);arrow(621,406);
node('GraspNet + bộ lọc → tư thế gắp',358,485,564,43,19);arrow(621,466);
node('MoveIt/MTC + ROS 2 → kế hoạch và điều phối',328,545,624,43,19);arrow(621,526);
node('Depth → mô hình vật cản',965,430,251,65,19);tx('↓',1070,498,40,36,25,red,true);tx('←',957,548,45,35,25,red,true);rect(1087,527,2,34,red);rect(991,560,98,2,red);
// Two branches from the plan to arm and gripper.
rect(240,596,800,2,red);rect(638,589,2,9,red);arrow(221,596);arrow(1021,596);
await asset('fr3_gazebo_nguoi_dung.png',64,602,99,62);tx('FR3: controller → driver → SDK/API hãng',173,615,530,35,19,red,true);
await asset('esp32cam_board_photo.jpg',760,614,31,45);s.images.items[s.images.items.length-1].crop={left:0.37,right:0.37,top:0.05,bottom:0.04};await asset('buslinker_v3.jpg',827,614,35,45);await asset('hiwonder_htd35h.jpg',895,614,36,45);s.images.items[s.images.items.length-1].crop={left:0.03,right:0.38,top:0.03,bottom:0.23};tx('→',795,621,30,25,17,red);tx('→',865,621,30,25,17,red);tx('Gripper',962,610,235,30,20,red,true);tx('ESP32 → BusLinker → servo',962,640,246,24,14,'#555555');
tx('Gripper hiện dùng HTTP; micro-ROS là thiết kế tích hợp.',173,652,560,22,13,'#555555');
s.speakerNotes.text='Một sơ đồ tổng thể theo yêu cầu, từ camera raRGB+depth qua nhận thức, gắp và kế hoạch, xuống cánh tayFR3 và gripper. Iconcamera là hình vẽ sơ đồ, không phải ảnh camera thật. ẢnhRGBD từ bao_cao/hinh/ghep_home.jpg; mô hìnhFR3 do người dùng gửi; hìnhESP32/BusLinker/servo theo nguồn các slide điện tử. Yêu cầu người dùng thực tế đi cùngRGB tớiGemini API, không đi vào camera; mũi tên đầu biểu thị tập hợp đầu vào. Depth không được gửi Gemini trong mã, dùng để chiếu ngược và dựng vật cản. Sơ đồ luồng chức năng không phải mỗi khối là một node; một số bước cùngscript và YAML/NumPy. Gemini chạy qua API từ xa; SAM/GraspNet cục bộ. MTC chọn giữa các ứng viên và xét cảnh, mô hìnhrobot và trạng thái khớp. Phản hồi trạng thái không vẽ đầy đủ để giữ hình gọn. Gripper.py hiệnHTTP; microROS mới là thiết kế chưa xác nhậnfirmware. Các bước mô phỏng có thể khác pipeline này. Nguồn: tools/grasp_once.py,scene_obstacles.py,mtc_pregrasp.py,gripper.py; https://github.com/micro-ROS/micro-ROS-Agent .';
await(await PresentationFile.exportPptx(p)).save(root+'/.build/slide-1-14-v16.pptx');await fs.writeFile(root+'/output/FR3_so_do_tong_the_v16.png',new Uint8Array(await(await p.export({slide:s,format:'png',scale:1})).arrayBuffer()));console.log('done');
