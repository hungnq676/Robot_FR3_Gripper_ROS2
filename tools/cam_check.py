#!/usr/bin/env python3
"""Has the camera moved since the hand-eye calibration?

With the J6-cap tag in view, compare where the camera sees the tag (PnP) with
where robot FK + the hand-eye file (camera.HANDEYE_FILE) say it must be. A few mm is the
calibration's own error; centimetres mean the camera (or its stand) moved and
the calibration must be redone. Runs on a still arm, moves nothing.

Usage: cam_check.py [--frames 15]
"""
import os
import sys
import time

import cv2
import numpy as np
import rclpy
import tf2_ros
import yaml
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

sys.path.insert(0, os.path.dirname(__file__))
import camera  # noqa: E402
from tag_check import TAG_SIZE  # noqa: E402

TAG_ID = 2


def tf_matrix(t):
    q, p = t.transform.rotation, t.transform.translation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def main():
    frames = int(sys.argv[sys.argv.index("--frames") + 1]) if "--frames" in sys.argv else 15
    cal = yaml.safe_load(open(camera.HANDEYE_FILE))
    tag_w2 = np.r_[cal["tag_in_gripper"]["xyz"], 1.0]
    rclpy.init()
    node = rclpy.create_node("cam_check")
    got = {"img": []}
    q = rclpy.qos.qos_profile_sensor_data
    node.create_subscription(Image, camera.RGB_TOPIC, lambda m: got["img"].append(m), q)
    node.create_subscription(CameraInfo, camera.INFO_TOPIC, lambda m: got.__setitem__("info", m), q)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
    h = TAG_SIZE / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    errs, t0 = [], time.time()
    while len(errs) < frames and time.time() - t0 < 30:
        rclpy.spin_once(node, timeout_sec=0.1)
        if not got["img"] or "info" not in got:
            continue
        m = got["img"].pop()
        got["img"].clear()
        K, D = np.array(got["info"].k).reshape(3, 3), np.array(got["info"].d)
        gray = cv2.cvtColor(camera.to_bgr(m), cv2.COLOR_BGR2GRAY)
        corners, ids, _ = det.detectMarkers(gray)
        if ids is None or TAG_ID not in ids.ravel():
            continue
        c = corners[list(ids.ravel()).index(TAG_ID)][0].astype(np.float64)
        n, rv, tv, e = cv2.solvePnPGeneric(obj, c, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if n == 0:
            continue
        try:
            T_bo = tf_matrix(buf.lookup_transform("base_footprint", m.header.frame_id, Time()))
            T_bw = tf_matrix(buf.lookup_transform("base_footprint", "wrist2_link", Time()))
        except tf2_ros.TransformException:
            continue
        seen = (T_bo @ np.r_[tv[int(np.argmin(np.ravel(e)))].ravel(), 1.0])[:3]
        errs.append(seen - (T_bw @ tag_w2)[:3])
    if not errs:
        print("Khong thay tag 2 (hoac thieu TF). Dua tag vao tam nhin camera roi chay lai.")
        return
    e = np.median(np.array(errs), axis=0) * 1000
    dist = np.linalg.norm(e)
    print(f"Hieu chuan luc {cal.get('calibrated_at')}; {len(errs)} khung hinh.")
    print(f"Tag theo camera - theo robot (he robot): dx={e[0]:+.1f} dy={e[1]:+.1f} dz={e[2]:+.1f} mm, tong {dist:.1f} mm")
    if dist < 5:
        print("=> OK: camera chua bi xe dich.")
    elif dist < 12:
        print("=> Hoi lech: kiem tra o vai tu the khac; neu deu > 5 mm thi nen hieu chuan lai.")
    else:
        print("=> LECH NHIEU: camera (hoac chan de) da bi xe dich. Can co dinh camera va hieu chuan lai.")
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
