#!/usr/bin/env python3
"""Capture Gazebo/RViz XComposite backing pixmaps even when another app is focused."""
import ctypes as C, re, subprocess,sys,time,signal
import cv2,numpy as np
X=C.CDLL('libX11.so.6'); XC=C.CDLL('libXcomposite.so.1')
X.XOpenDisplay.restype=C.c_void_p; d=X.XOpenDisplay(None)
X.XGetGeometry.argtypes=[C.c_void_p,C.c_ulong,*([C.c_void_p]*7)]
XC.XCompositeNameWindowPixmap.argtypes=[C.c_void_p,C.c_ulong]; XC.XCompositeNameWindowPixmap.restype=C.c_ulong
class XImage(C.Structure):
 _fields_=[('width',C.c_int),('height',C.c_int),('xoffset',C.c_int),('format',C.c_int),('data',C.c_void_p),('byte_order',C.c_int),('bitmap_unit',C.c_int),('bitmap_bit_order',C.c_int),('bitmap_pad',C.c_int),('depth',C.c_int),('bytes_per_line',C.c_int),('bits_per_pixel',C.c_int)]
X.XGetImage.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_int,C.c_uint,C.c_uint,C.c_ulong,C.c_int]; X.XGetImage.restype=C.POINTER(XImage)
X.XDestroyImage.argtypes=[C.POINTER(XImage)]; X.XFreePixmap.argtypes=[C.c_void_p,C.c_ulong]
tree=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
ids=[int(re.search(p,tree)[1],16) for p in [r'(0x[0-9a-f]+) "Gazebo Sim"',r'(0x[0-9a-f]+) ".* - RViz"']]
# Mutter redirects the decoration frame rather than the client window.
for j,w in enumerate(ids):
 info=subprocess.check_output(['xwininfo','-id',hex(w),'-tree'],text=True)
 ids[j]=int(re.search(r'Parent window id: (0x[0-9a-f]+)',info)[1],16)

def grab(w):
 pix=XC.XCompositeNameWindowPixmap(d,w)
 vals=[C.c_ulong(),C.c_int(),C.c_int(),C.c_uint(),C.c_uint(),C.c_uint(),C.c_uint()]
 X.XGetGeometry(d,pix,*[C.byref(v) for v in vals]); width,height=vals[3].value,vals[4].value
 im=X.XGetImage(d,pix,0,0,width,height,0xffffffff,2); i=im.contents
 a=np.frombuffer(C.string_at(i.data,i.bytes_per_line*height),np.uint8).reshape(height,i.bytes_per_line)[:,:width*4].reshape(height,width,4)[:,:,:3].copy()
 X.XDestroyImage(im); X.XFreePixmap(d,pix); return a
run=True
def stop(*_):
 global run;run=False
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
if len(sys.argv)>1 and sys.argv[1]=='--snapshot':
 for j,w in enumerate(ids):cv2.imwrite(f'/tmp/sim_window_{j}.jpg',grab(w))
 sys.exit()
fps=10; out=cv2.VideoWriter(sys.argv[1],cv2.VideoWriter_fourcc(*'VP80'),fps,(1920,1080))
print('Recording independent Gazebo and RViz views',flush=True)
while run:
 start=time.monotonic(); frames=[]
 for j,w in enumerate(ids):
  a=grab(w); a=cv2.resize(a,(960,1020));
  panel=np.zeros((1080,960,3),np.uint8);panel[60:]=a
  cv2.putText(panel,['GAZEBO - physical simulation','RVIZ - MoveIt / MTC'][j],(20,40),cv2.FONT_HERSHEY_SIMPLEX,.8,(255,255,255),2);frames.append(panel)
 out.write(np.hstack(frames));time.sleep(max(0,1/fps-(time.monotonic()-start)))
out.release()
