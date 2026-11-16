#!/usr/bin/env python3
"""Stills for the report from the Astra: RGB, depth and RGB-D, plus the raw data.

  <time>_<label>_rgb.jpg        colour image
  <time>_<label>_depth.png      depth, colour map 0.4..1.0 m with a scale bar (black = no depth)
  <time>_<label>_rgbd.jpg       colour and depth blended (where depth exists)
  <time>_<label>_depth_mm.png   16-bit depth in mm (median of --frames), for later processing
  <time>_<label>_cloud.ply      coloured point cloud in base_footprint (hand-eye TF, depth model)

Into the running record_session.sh session (anh/), else nhat_ky/anh/<date>. Moves nothing.
Usage: snap_rgbd.py [label] [--frames 5]
"""
import os
import sys
import time
import warnings

import cv2
import numpy as np
import rclpy
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(__file__))
import camera  # noqa: E402

_SESSION = os.path.expanduser("~/.ros/fr3_session_dir")
LOG_DIR = (os.path.join(open(_SESSION).read().strip(), "anh") if os.path.exists(_SESSION)
           else os.path.expanduser(time.strftime("~/fr3_hungros2/nhat_ky/anh/%Y-%m-%d")))
Z_MIN, Z_MAX = 400.0, 1000.0   # mm, colour map range (table at ~0.7 m)


def depth_colour(d):
    img = cv2.applyColorMap(np.clip((d - Z_MIN) / (Z_MAX - Z_MIN) * 255, 0, 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    img[d <= 0] = 0
    return img


def scale_bar(img):
    h, w = img.shape[:2]
    bar = cv2.applyColorMap(np.linspace(0, 255, 200).astype(np.uint8)[None].repeat(12, 0), cv2.COLORMAP_TURBO)
    x0, y0 = w - 220, h - 40
    img[y0:y0 + 12, x0:x0 + 200] = bar
    for v, x in ((Z_MIN, x0), (Z_MAX, x0 + 200)):
        cv2.putText(img, f"{v / 1000:.1f} m", (x - 15, y0 + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return img


def main():
    label = next((a for a in sys.argv[1:] if not a.startswith("--") and not a.isdigit()), "anh")
    frames = int(sys.argv[sys.argv.index("--frames") + 1]) if "--frames" in sys.argv else 5
    rclpy.init()
    node = rclpy.create_node("snap_rgbd")
    tf = Buffer()
    TransformListener(tf, node)
    got = {"depth": []}
    q = rclpy.qos.qos_profile_sensor_data
    node.create_subscription(Image, camera.RGB_TOPIC, lambda m: got.__setitem__("rgb", m), q)
    node.create_subscription(CameraInfo, camera.INFO_TOPIC, lambda m: got.__setitem__("info", m), q)
    node.create_subscription(Image, camera.DEPTH_TOPIC, lambda m: got["depth"].append(
        np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).copy()), q)
    t0 = time.time()
    while len(got["depth"]) < frames or "rgb" not in got or "info" not in got:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0 > 10:
            sys.exit("khong co anh/depth (run_astra.sh da chay chua?)")
    bgr = camera.to_bgr(got["rgb"])
    D = np.stack(got["depth"][-frames:]).astype(np.float32)
    D[D == 0] = np.nan
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        d = np.nan_to_num(np.nanmedian(D, 0)).astype(np.float32)

    os.makedirs(LOG_DIR, exist_ok=True)
    base = os.path.join(LOG_DIR, f"{time.strftime('%H%M%S')}_{label}")
    dc = depth_colour(d)
    rgbd = bgr.copy()
    has = d > 0
    rgbd[has] = (0.5 * bgr[has] + 0.5 * dc[has]).astype(np.uint8)
    cv2.imwrite(base + "_rgb.jpg", bgr)
    cv2.imwrite(base + "_depth.png", scale_bar(dc.copy()))
    cv2.imwrite(base + "_rgbd.jpg", scale_bar(rgbd))
    cv2.imwrite(base + "_depth_mm.png", d.astype(np.uint16))

    try:   # coloured cloud in the robot frame
        t = tf.lookup_transform("base_footprint", camera.OPTICAL_FRAME, Time())
        T = np.eye(4)
        r = t.transform.rotation
        T[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
        T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        K = np.array(got["info"].k).reshape(3, 3)
        z = camera.correct_depth(d / 1000.0, camera.load_depth_model())
        v, u = np.nonzero(z > 0)
        zz = z[v, u]
        P = np.stack([(u - K[0, 2]) * zz / K[0, 0], (v - K[1, 2]) * zz / K[1, 1], zz], 1) @ T[:3, :3].T + T[:3, 3]
        C = bgr[v, u][:, ::-1]
        with open(base + "_cloud.ply", "w") as f:
            f.write(f"ply\nformat ascii 1.0\nelement vertex {len(P)}\nproperty float x\nproperty float y\n"
                    "property float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
            np.savetxt(f, np.hstack([P, C]), fmt="%.4f %.4f %.4f %d %d %d")
    except Exception as e:   # noqa: BLE001 - the images are still useful without TF
        print(f"(khong ghi point cloud: {e})")
    print(f"Anh: {base}_rgb.jpg | _depth.png | _rgbd.jpg (+ _depth_mm.png, _cloud.ply)")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
