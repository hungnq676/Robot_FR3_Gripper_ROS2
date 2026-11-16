#!/usr/bin/env python3
"""Record the Astra camera to video for the report: colour | depth (colour map)
side by side, with the wall-clock time stamped in, plus one still of each at
the start. Moves nothing. Stops on Ctrl+C / SIGTERM and closes the file.

Usage: record_camera.py OUT_DIR [--fps 15]
"""
import os
import signal
import sys
import time

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.dirname(__file__))
import camera  # noqa: E402


def depth_vis(d):
    """Depth in mm -> BGR; real 0.4–1.0 m, simulation 0.70–1.45 m. Invalid depth is black."""
    z = d.astype(np.float32)
    near, far = (700.0, 1450.0) if os.environ.get("FR3_CAMERA_MODE") == "sim" else (400.0, 1000.0)
    valid = np.isfinite(z) & (z > 0)
    scaled = np.where(valid, np.clip((z - near) / (far - near), 0, 1), 0)
    img = cv2.applyColorMap((scaled * 255.0).astype(np.uint8), cv2.COLORMAP_TURBO)
    img[~valid] = 0
    return img


def main():
    out_dir = sys.argv[1]
    fps = float(sys.argv[sys.argv.index("--fps") + 1]) if "--fps" in sys.argv else 15.0
    os.makedirs(out_dir, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("record_camera")
    last = {}
    q = rclpy.qos.qos_profile_sensor_data
    node.create_subscription(Image, camera.RGB_TOPIC, lambda m: last.__setitem__("rgb", m), q)
    node.create_subscription(Image, camera.DEPTH_TOPIC, lambda m: last.__setitem__("depth", m), q)
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))

    path = os.path.join(out_dir, "camera_mau_depth.webm")
    writer, n, t_next = None, 0, time.time()
    while not stop:
        rclpy.spin_once(node, timeout_sec=0.01)
        if "rgb" not in last or "depth" not in last or time.time() < t_next:
            continue
        t_next += 1.0 / fps
        bgr = camera.to_bgr(last["rgb"])
        dm = last["depth"]
        d = camera.depth_meters(dm) * 1000.0
        dv = cv2.resize(depth_vis(d), (bgr.shape[1], bgr.shape[0]), interpolation=cv2.INTER_NEAREST)
        frame = np.hstack([bgr, dv])
        cv2.putText(frame, time.strftime("%Y-%m-%d %H:%M:%S"), (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        if writer is None:
            # VP8/WebM: the GNOME player (Totem) and Firefox play it; mp4v (MPEG-4 part 2) has no decoder here.
            writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"VP80"), fps, (frame.shape[1], frame.shape[0]))
            cv2.imwrite(os.path.join(out_dir, "camera_mau_dau.jpg"), bgr)
            cv2.imwrite(os.path.join(out_dir, "camera_depth_dau.png"), dv)
            print(f"record_camera: ghi {path} ({frame.shape[1]}x{frame.shape[0]} @ {fps:.0f} fps)", flush=True)
        writer.write(frame)
        n += 1
    if writer is not None:
        writer.release()
    print(f"record_camera: dung, {n} khung ({n / fps:.0f} s)", flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
