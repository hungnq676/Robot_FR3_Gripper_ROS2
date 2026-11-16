#!/usr/bin/env python3
"""Record a simulated pick (ROS_DOMAIN_ID=34) for the report: jaw force sensors, gripper joint,
pick status, and the virtual OAK camera's RGB + depth (first frame and on demand).
Writes to bao_cao/du_lieu_mo_phong/<tag>/: luc.csv, trang_thai.csv, oak_rgb.png, oak_depth_mm.png.
Usage: ghi_mo_phong.py <tag> [seconds]"""
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from fairino_fr3_msgs.msg import FingerForces
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String


class Recorder(Node):
    def __init__(self, out):
        super().__init__("bao_cao_recorder")
        self.out, self.bridge, self.t0 = out, CvBridge(), time.time()
        self.f = open(os.path.join(out, "luc.csv"), "w")
        self.f.write("t,force_1,force_2,raw_1,raw_2,contact_1,contact_2,gripper_joint\n")
        self.s = open(os.path.join(out, "trang_thai.csv"), "w")
        self.s.write("t,status\n")
        self.grip = float("nan")
        self.rgb = self.depth = None
        self.create_subscription(FingerForces, "/fr3_demo/gripper/finger_forces", self.on_force, 50)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 50)
        self.create_subscription(String, "/fr3_demo/pick_status", self.on_status, 10)
        self.create_subscription(Image, "/oak/rgb/image_raw", self.on_rgb, 2)
        self.create_subscription(Image, "/oak/stereo/image_raw", self.on_depth, 2)
        self.create_subscription(Image, "/fr3_demo/gemini/annotated", self.on_gemini, 2)

    def now(self):
        return time.time() - self.t0

    def on_force(self, m):
        self.f.write(f"{self.now():.3f},{m.force_clamp_1:.3f},{m.force_clamp_2:.3f},{m.raw_clamp_1},"
                     f"{m.raw_clamp_2},{int(m.contact_clamp_1)},{int(m.contact_clamp_2)},{self.grip:.5f}\n")

    def on_joints(self, m):
        for n, p in zip(m.name, m.position):
            if "gripper" in n:
                self.grip = p

    def on_status(self, m):
        self.s.write(f"{self.now():.3f},\"{m.data}\"\n"); self.s.flush()
        print(f"{self.now():6.1f} s  {m.data}", flush=True)

    def on_rgb(self, m):
        if self.rgb is None:
            self.rgb = self.bridge.imgmsg_to_cv2(m, "bgr8")
            cv2.imwrite(os.path.join(self.out, "oak_rgb.png"), self.rgb)

    def on_gemini(self, m):
        cv2.imwrite(os.path.join(self.out, "gemini_annotated.png"), self.bridge.imgmsg_to_cv2(m, "bgr8"))

    def on_depth(self, m):
        if self.depth is None:
            d = self.bridge.imgmsg_to_cv2(m)
            d = np.nan_to_num(d.astype(np.float32), nan=0, posinf=0, neginf=0)
            self.depth = (d * 1000).clip(0, 65535).astype(np.uint16) if d.max() < 100 else d.astype(np.uint16)
            cv2.imwrite(os.path.join(self.out, "oak_depth_mm.png"), self.depth)


def main():
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "du_lieu_mo_phong", sys.argv[1])
    os.makedirs(out, exist_ok=True)
    rclpy.init()
    node = Recorder(out)
    end = time.time() + (float(sys.argv[2]) if len(sys.argv) > 2 else 120)
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.f.close(); node.s.close()
    print("saved", out)


if __name__ == "__main__":
    main()
