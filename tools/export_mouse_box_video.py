#!/usr/bin/env python3
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
import subprocess
root=Path(__file__).resolve().parents[1];p=root/'nhat_ky/du_lieu/2026-11-03_mouse_open_box'
font='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
def card(title,subtitle,imgs,out,footer):
 im=Image.new('RGB',(1920,1080),(18,25,36));d=ImageDraw.Draw(im)
 d.text((55,30),title,font=ImageFont.truetype(font,43),fill='white')
 d.text((55,100),subtitle,font=ImageFont.truetype(font,25),fill=(180,210,240))
 for src,rect in imgs:
  a=Image.open(src).convert('RGB');a.thumbnail((rect[2],rect[3]));im.paste(a,(rect[0]+(rect[2]-a.width)//2,rect[1]+(rect[3]-a.height)//2))
 d.text((55,995),footer,font=ImageFont.truetype(font,24),fill=(240,210,140));im.save(p/out)
card('1. Đầu vào camera RGB-D mô phỏng','Ảnh màu và depth từ camera OAK trong Gazebo.',[(p/'camera_mau_dau.jpg',(40,170,900,760)),(p/'camera_depth_dau.png',(980,170,900,760))],'01_camera.png','FAIRINO FR3 • Gazebo Harmonic • ROS 2 Humble • mô phỏng domain 34')
mask=next(p.glob('*graspnet_computer_mouse.jpg'))
card('2. Gemini → MobileSAM → GraspNet','Gemini chọn chuột; MobileSAM tách vật; GraspNet suy luận trên GPU.',[(mask,(200,160,1520,760))],'02_perception.png','GraspNet nghiêng 72–75°: dùng phương án dự phòng từ hình học SAM + depth để gắp từ trên.')
ff='/home/duong-12a/miniconda3/envs/graspnet/lib/python3.10/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2'
args=[ff,'-y','-hide_banner','-loglevel','warning','-loop','1','-t','5','-i',str(p/'01_camera.png'),'-loop','1','-t','7','-i',str(p/'02_perception.png'),'-i',str(p/'chuot_vao_hop_full.webm'),'-filter_complex','[0:v]fps=10,setsar=1,format=yuv420p[v0];[1:v]fps=10,setsar=1,format=yuv420p[v1];[2:v]split=2[g][r];[g]crop=520:700:10:330,scale=960:1020,pad=960:1080:0:60:black[gg];[r]crop=960:1080:960:0[rr];[gg][rr]hstack,fps=10,setsar=1,format=yuv420p[v2];[v0][v1][v2]concat=n=3:v=1:a=0[v]','-map','[v]','-c:v','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p','-movflags','+faststart',str(p/'FR3_chuot_vao_hop_slide.mp4')]
subprocess.run(args,check=True)
print(p/'FR3_chuot_vao_hop_slide.mp4')
