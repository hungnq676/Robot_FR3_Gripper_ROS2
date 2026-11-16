from pathlib import Path
import json,subprocess,cv2
p=Path(__file__).resolve().parents[1]/'nhat_ky/du_lieu/2026-11-03_pick_camera_moi'
m=json.loads((p/'timing.json').read_text());ff='/home/duong-12a/miniconda3/envs/graspnet/lib/python3.10/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2'
# Preserve the full move, beginning before execution, ending after home.
duration=100
subprocess.run([ff,'-y','-hide_banner','-loglevel','error','-ss',str(m['offset']),'-i',str(p/'camera_live.webm'),'-t',str(duration),'-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(p/'CAMERA_MOI_ROBOT_GAP_CHUOT.mp4')],check=True)
# The XComposite capture was slower than its nominal frame rate; restore wall time.
ratio=m['wall_duration']/m['gazebo_rviz.webm']
subprocess.run([ff,'-y','-hide_banner','-loglevel','error','-i',str(p/'gazebo_rviz.webm'),'-vf',f'crop=960:1080:960:0,setpts={ratio}*PTS','-t',str(duration),'-r','10','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(p/'RVIZ_LUOT_CAMERA_MOI.mp4')],check=True)
c=cv2.VideoCapture(str(p/'CAMERA_MOI_ROBOT_GAP_CHUOT.mp4'));c.set(cv2.CAP_PROP_POS_MSEC,97000);ok,f=c.read();assert ok;cv2.imwrite(str(p/'ket_qua_camera_moi.png'),f)
print('Exported actual RGB-D robot motion and RViz from the new camera run.')
