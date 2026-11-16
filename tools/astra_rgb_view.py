#!/usr/bin/env python3
"""Live colour view of the Orbbec Astra Pro (UVC camera, no driver needed).

Depth needs Orbbec's own driver (ros2_astra_camera); this only shows colour.
Keys: q/Esc quit, s save a png to nhat_ky/anh/astra_view.
Usage: astra_rgb_view.py [--dev 0] [--width 1920 --height 1080]
"""
import argparse
import os
import time

import cv2

SAVE_DIR = os.path.expanduser("~/fr3_hungros2/nhat_ky/anh/astra_view")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", type=int, default=0, help="/dev/videoN")
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    a = ap.parse_args()

    cap = cv2.VideoCapture(a.dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))   # MJPG: full fps on USB 2
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    if not cap.isOpened():
        raise SystemExit(f"Khong mo duoc /dev/video{a.dev}")
    cv2.namedWindow("Astra RGB", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Astra RGB", 960, 540)
    t_fps, n_fps, fps = time.time(), 0, 0.0
    while True:
        ok, bgr = cap.read()
        if not ok:
            cv2.waitKey(5)
            continue
        n_fps += 1
        if time.time() - t_fps > 1.0:
            fps, n_fps, t_fps = n_fps / (time.time() - t_fps), 0, time.time()
        img = bgr.copy()
        cv2.putText(img, f"Astra RGB  {img.shape[1]}x{img.shape[0]}  {fps:.1f} fps",
                    (10, 36), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        cv2.imshow("Astra RGB", img)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("s"):
            os.makedirs(SAVE_DIR, exist_ok=True)
            path = os.path.join(SAVE_DIR, time.strftime("%Y%m%d_%H%M%S") + "_rgb.png")
            cv2.imwrite(path, bgr)
            print(f"Da luu {path}")
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
