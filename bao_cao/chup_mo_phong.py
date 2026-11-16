#!/usr/bin/env python3
"""Gazebo GUI screenshots for the report: aim the GUI camera from an eye point at a target
(world frame, m) and save the 3D view to bao_cao/hinh/mo_phong/<name>.png.
Usage: chup_mo_phong.py <name> ex ey ez tx ty tz [collisions]"""
import glob
import math
import os
import shutil
import subprocess
import sys
import time

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hinh", "mo_phong")
PICS = os.path.expanduser("~/.gz/gui/pictures")


def gz(service, reqtype, req):
    subprocess.run(["gz", "service", "-s", service, "--reqtype", reqtype, "--reptype", "gz.msgs.Boolean",
                    "--timeout", "5000", "--req", req], check=True, capture_output=True)


def aim(eye, target):
    d = [t - e for t, e in zip(target, eye)]
    yaw = math.atan2(d[1], d[0])
    pitch = math.atan2(-d[2], math.hypot(d[0], d[1]))
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    q = (cp * cy, -sp * sy, sp * cy, cp * sy)  # w, x, y, z for rpy (0, pitch, yaw)
    gz("/gui/move_to/pose", "gz.msgs.GUICamera",
       f"pose: {{position: {{x: {eye[0]}, y: {eye[1]}, z: {eye[2]}}}, "
       f"orientation: {{w: {q[0]}, x: {q[1]}, y: {q[2]}, z: {q[3]}}}}}")


def shot(name):
    before = set(glob.glob(os.path.join(PICS, "*.png")))
    gz("/gui/screenshot", "gz.msgs.StringMsg", 'data: ""')
    for _ in range(50):
        new = set(glob.glob(os.path.join(PICS, "*.png"))) - before
        if new:
            time.sleep(0.3)
            os.makedirs(OUT, exist_ok=True)
            dst = os.path.join(OUT, name + ".png")
            shutil.move(new.pop(), dst)
            return dst
        time.sleep(0.1)
    raise RuntimeError("no screenshot")


if __name__ == "__main__":
    name, v = sys.argv[1], [float(x) for x in sys.argv[2:8]]
    coll = len(sys.argv) > 8
    if coll:
        gz("/gui/view/collisions", "gz.msgs.StringMsg", 'data: "fairino_fr3"')
    aim(v[:3], v[3:])
    time.sleep(1.5)  # move_to animates the camera
    print(shot(name))
    if coll:
        gz("/gui/view/collisions", "gz.msgs.StringMsg", 'data: "fairino_fr3"')
