#!/usr/bin/env python3
"""Arrange the two XWayland simulation windows side by side for recording."""
import ctypes as C
import re
import subprocess
x=C.CDLL('libX11.so.6'); x.XOpenDisplay.restype=C.c_void_p
D=x.XOpenDisplay(None)
x.XMoveResizeWindow.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_int,C.c_uint,C.c_uint]
x.XMapRaised.argtypes=[C.c_void_p,C.c_ulong]; x.XFlush.argtypes=[C.c_void_p]
tree=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
for pattern,rect in [(r'(0x[0-9a-f]+) "Gazebo Sim"',(0,45,950,950)),(r'(0x[0-9a-f]+) ".* - RViz"',(960,45,950,950))]:
    m=re.search(pattern,tree)
    if m:
        w=int(m[1],16)
        subprocess.run(['xprop','-id',m[1],'-remove','_NET_WM_STATE'],stdout=subprocess.DEVNULL)
        x.XMoveResizeWindow(D,w,*rect); x.XMapRaised(D,w)
x.XFlush(D)
