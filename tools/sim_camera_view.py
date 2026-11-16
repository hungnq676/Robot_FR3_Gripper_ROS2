#!/usr/bin/env python3
"""Live ROS RGB-D evidence from the simulation; no synthetic object overlays."""
import os,time,sys,signal
from pathlib import Path
import cv2,numpy as np,rclpy
from sensor_msgs.msg import Image,CameraInfo
import camera
if os.environ.get('ROS_DOMAIN_ID')!='34' or os.environ.get('FR3_CAMERA_MODE')!='sim':raise RuntimeError('Simulation domain 34 and FR3_CAMERA_MODE=sim required')
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
rclpy.init();n=rclpy.create_node('live_sim_camera_evidence');last={};q=rclpy.qos.qos_profile_sensor_data
for key,topic,typ in [('rgb',camera.RGB_TOPIC,Image),('depth',camera.DEPTH_TOPIC,Image),('info',camera.INFO_TOPIC,CameraInfo)]:
 n.create_subscription(typ,topic,lambda m,k=key:last.__setitem__(k,m),q)
run=True
writer=None
record_end=None
record_finished=False
record_start=None
written=0

def stop(*_):
 global run;run=False
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
start=time.time();saved=False;nextframe=0
while run:
 rclpy.spin_once(n,timeout_sec=.03)
 if not all(k in last for k in ['rgb','depth','info']):
  if time.time()-start>30:raise RuntimeError('Camera topics timeout')
  continue
 if time.time()<nextframe:continue
 nextframe=time.time()+.1
 bgr=camera.to_bgr(last['rgb']);z=camera.depth_meters(last['depth']);ok=np.isfinite(z)&(z>0)
 # Fixed metric scale: the tabletop is about 1.05 m below the camera.
 near,far=.70,1.45
 dz=np.clip((z-near)/(far-near),0,1);dz[~ok]=0
 depth=cv2.applyColorMap((dz*255).astype(np.uint8),cv2.COLORMAP_TURBO);depth[~ok]=0
 panels=[]
 for title,img in [('LIVE RGB /oak/rgb/image_raw',bgr),('LIVE DEPTH 32FC1 metres /oak/stereo/image_raw',depth)]:
  a=cv2.resize(img,(960,720));p=np.zeros((850,960,3),np.uint8);p[60:780]=a
  cv2.putText(p,title,(15,35),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
  t=last['rgb' if title.startswith('LIVE RGB') else 'depth'].header.stamp
  cv2.putText(p,f'ROS stamp {t.sec}.{t.nanosec:09d}',(15,807),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1);panels.append(p)
 frame=np.hstack(panels)
 cv2.putText(frame,'Depth scale: blue 0.70 m -> red 1.45 m; black = invalid',(980,835),cv2.FONT_HERSHEY_SIMPLEX,.48,(255,255,255),1)
 if not saved:
  cv2.imwrite(str(out/'rgb.png'),bgr);cv2.imwrite(str(out/'depth_metric.png'),depth);np.save(out/'depth_metres.npy',z);cv2.imwrite(str(out/'rgb_depth_live.png'),frame)
  info=last['info'];(out/'camera_info.txt').write_text(f'width={info.width} height={info.height}\nK={list(info.k)}\nD={list(info.d)}\nframe={info.header.frame_id}\n')
  print('Saved actual RGB-D topics; depth valid fraction',float(ok.mean()),flush=True);saved=True
  if '--snapshot' in sys.argv:break
 if not record_finished and "--view-only" not in sys.argv:
  if writer is None:
   writer=cv2.VideoWriter(str(out/'camera_live.webm'),cv2.VideoWriter_fourcc(*'VP80'),10,(1920,850));record_start=time.time();record_end=record_start+20
  target=int((time.time()-record_start)*10)+1
  if "--continuous" not in sys.argv:target=min(200,target)
  while written<target:
   writer.write(frame);written+=1
  if time.time()>=record_end and "--continuous" not in sys.argv:
   writer.release();writer=None;record_finished=True;print('Saved 20 s live camera clip; viewer stays open.',flush=True)
 cv2.imshow('LIVE SIM CAMERA - RGB + metric depth',cv2.resize(frame,(1440,638)))
 if cv2.waitKey(1)==27:break
if writer:writer.release()
cv2.destroyAllWindows();n.destroy_node();rclpy.shutdown()
