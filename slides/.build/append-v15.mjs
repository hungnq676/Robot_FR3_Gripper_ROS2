import fs from 'node:fs/promises';import {FileBlob,PresentationFile} from '@oai/artifact-tool';
const root='/home/duong-12a/fr3_hungros2/slides';const p=await PresentationFile.importPptx(await FileBlob.load(root+'/output/FR3_slide_01_14_v14b.pptx'));let s;const red='#B5121B';
function tx(t,x,y,w,h,size=24,color='#252525',bold=false){let q=s.shapes.add({geometry:'textbox',position:{left:x,top:y,width:w,height:h},fill:'none',line:{fill:'none',width:0}});q.text=t;q.text.style={typeface:'DejaVu Sans',fontSize:size,bold,color};}
function rect(x,y,w,h,fill){s.shapes.add({geometry:'rect',position:{left:x,top:y,width:w,height:h},fill,line:{fill:'none',width:0}})}
const logo=await fs.readFile(root+'/assets/ptit_chinh_thuc.svg');function header(title,num){s=p.slides.add();s.background.fill='#FFFFFF';rect(0,0,1280,12,red);s.images.add({blob:logo.buffer.slice(logo.byteOffset,logo.byteOffset+logo.byteLength),contentType:'image/svg+xml',alt:'Logo PTIT',fit:'contain',position:{left:690,top:36,width:530,height:62}});tx('KHOA KỸ THUẬT ĐIỆN TỬ 1',64,51,570,35,21,'#555555',true);rect(64,122,1152,2,'#E6E6E6');tx(title,64,153,1152,60,38,red,true);rect(0,674,1280,46,red);tx('NGHIÊN CỨU KHOA HỌC SINH VIÊN',64,687,1000,27,17,'#FFFFFF');tx(num,1160,685,60,30,19,'#FFFFFF',true);}

async function photo(name,x,y,w,h){const d=await fs.readFile('/home/duong-12a/fr3_hungros2/bao_cao/hinh/'+name);s.images.add({blob:d.buffer.slice(d.byteOffset,d.byteOffset+d.byteLength),contentType:name.endsWith('.png')?'image/png':'image/jpeg',alt:name,fit:'contain',position:{left:x,top:y,width:w,height:h}});}


async function asset(name,x,y,w,h){const d=await fs.readFile(root+'/assets/'+name);s.images.add({blob:d.buffer.slice(d.byteOffset,d.byteOffset+d.byteLength),contentType:name.endsWith('.png')?'image/png':'image/jpeg',alt:name,fit:'contain',position:{left:x,top:y,width:w,height:h}});}

header('ESP32 VÀ MICRO-ROS: THIẾT KẾ GIAO TIẾP','14');
tx('Đưa lệnh và trạng thái gripper vào hệ thống ROS 2 qua micro-ROS Agent',64,225,1152,50,24,'#555555');
for(const [t,x] of [['Máy tính · ROS 2',64],['ESP32-CAM',390],['BusLinker V3',695],['HTD-35H',1000]])tx(t,x,291,240,40,23,red,true);
rect(72,347,224,139,'#FFF3F3');rect(62,488,244,9,'#B5121B');tx('Node ROS 2\nmicro-ROS Agent',88,383,196,85,18,red,true);
await asset('esp32cam_board_photo.jpg',442,345,104,154);s.images.items[s.images.items.length-1].crop={left:0.37,right:0.37,top:0.05,bottom:0.04};
await asset('buslinker_v3.jpg',739,345,122,154);await asset('hiwonder_htd35h.jpg',1043,345,122,154);s.images.items[s.images.items.length-1].crop={left:0.03,right:0.38,top:0.03,bottom:0.23};
tx('↔',320,402,55,44,32,red,true);tx('Wi-Fi',313,448,82,30,17,'#555555');tx('↔',606,402,55,44,32,red,true);tx('UART',599,448,82,30,17,'#555555');tx('↔',914,402,55,44,32,red,true);tx('Bus TTL',903,448,92,30,17,'#555555');
tx('ROS 2 ↔ Agent',64,520,250,37,22,red,true);tx('Kết nối node trên ESP32\nvới mạng ROS 2',64,561,250,70,20,'#555555');
tx('micro-ROS Client',390,520,260,37,22,red,true);tx('Nhận lệnh, điều phối UART;\nphát trạng thái gripper',390,561,280,70,20,'#555555');
tx('Giao tiếp servo',695,520,250,37,22,red,true);tx('Chuyển UART sang\nbus bán song công',695,561,250,70,20,'#555555');
tx('Servo bus',1000,520,230,37,22,red,true);tx('Vị trí, nhiệt độ\nvà điện áp',1000,561,230,70,20,'#555555');
tx('Sơ đồ micro-ROS ở mức thiết kế; mã điều khiển đã kiểm tra hiện dùng HTTP/Wi-Fi.',64,645,1152,25,17,'#555555');
s.speakerNotes.text='Sơ đồ đề xuất tích hợp micro-ROS theo yêu cầu người dùng, chưa khẳng định firmware hiện hành đã chuyển sang micro-ROS. Chưa tìm thấy firmware rclc/micro_ros trong workspace. tools/gripper.py hiện dùngHTTP/WiFi. Gói FR3/src/fr3_gripper_bridge có node action bridge sử dụng steps0..6500, khác gripper servo1000positions hiện tại nên không gán nó cho HTD35H nếu chưa xác nhận. MicroROS client trên ESP32 giao tiếp Agent trên PC qua UDP/WiFi hoặc transport phù hợp; ROS2 topics đi qua Agent, firmware chuyển lệnh thành UART servo protocol. Trạng thái quay lại từservo. Nguồn: https://github.com/micro-ROS/micro-ROS-Agent ; tools/gripper.py; hình linh kiện theo nguồn ghi ở slide điện tử. Cần đối chiếu firmware thật để cập nhật tên topic, message và transport.';
const software=p.slides.items[11];for(const q of software.shapes.items){let t=String(q.text);if(t.startsWith('Gripper: chương trình'))q.text='Gripper: ESP32 ↔ BusLinker ↔ servo · micro-ROS nối ESP32 với ROS 2 theo thiết kế';if(t.startsWith('ROS 2: topic'))q.text='ROS 2: topic, action, TF · HTTP: mã hiện tại · micro-ROS: thiết kế tích hợp';}
software.speakerNotes.text+='\nCập nhật: micro-ROS là thiết kế tích hợp chưa xác nhận firmware. MãHTTP hiện tại không bị thay đổi. Không coi user ok là bằng chứng triển khai.';
const ids=(await p.inspect({select:'$.slides[*]',include:['id']})).records.map(r=>r.id);const order=[...ids.slice(0,11),ids[12],ids[13],ids[14],ids[11]];p.slides.reorder(order);
for(let i=0;i<p.slides.items.length;i++){for(const q of p.slides.items[i].shapes.items){if(q.position.left>=1150&&q.position.top>=680&&/^\d{2}$/.test(String(q.text)))q.text=String(i+1).padStart(2,'0');}}
await(await PresentationFile.exportPptx(p)).save(root+'/.build/slide-1-15-v15b.pptx');for(let i of [11,12,13,14])await fs.writeFile(root+'/output/FR3_slide_'+String(i+1).padStart(2,'0')+'_v15.png',new Uint8Array(await(await p.export({slide:p.slides.items[i],format:'png',scale:1})).arrayBuffer()));console.log('done');
