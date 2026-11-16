#!/usr/bin/env python3
"""Live OAK-D Pro W streams with Luxonis' own library (depthai 2.x), no ROS.

Windows (pick with --mode, default all three):
  RGB    colour camera, 1080p scaled to 1280x720 (same as the ROS driver)
  Depth  stereo depth aligned to RGB, colour map --near..--far m, black = no depth
  RGBD   colour blended with the depth colour map
Click a window to read the depth (mm) at that pixel. Keys: q/Esc quit,
s save a snapshot (RGB png + 16-bit depth png), l IR laser on/off,
f IR flood light on/off.

The camera can be open in one program only: stop the ROS driver (run_oak.sh) first.
--no-rgb: grey left mono image instead of colour, depth aligned to it (640x400).
Usage: oak_stream.py [--mode all|rgb|depth|rgbd] [--fps 15] [--near 0.3] [--far 1.5] [--no-rgb]
"""
import argparse
import os
import time

import cv2
import depthai as dai
import numpy as np

SAVE_DIR = os.path.expanduser("~/fr3_hungros2/nhat_ky/anh/oak_stream")


def build_pipeline(fps, no_rgb=False):
    p = dai.Pipeline()
    if not no_rgb:
        rgb = p.create(dai.node.ColorCamera)
        rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        rgb.setIspScale(2, 3)                  # 1920x1080 -> 1280x720
        rgb.setFps(fps)
    left, right = p.create(dai.node.MonoCamera), p.create(dai.node.MonoCamera)
    for mono, socket in ((left, dai.CameraBoardSocket.CAM_B), (right, dai.CameraBoardSocket.CAM_C)):
        mono.setBoardSocket(socket)
        mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono.setFps(fps)
    stereo = p.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)
    if no_rgb:                                 # depth aligned to the rectified left mono, 640x400
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)
        image = stereo.rectifiedLeft
    else:
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(1280, 720)
        image = rgb.isp
    left.out.link(stereo.left)
    right.out.link(stereo.right)
    for name, out in (("rgb", image), ("depth", stereo.depth)):
        x = p.create(dai.node.XLinkOut)
        x.setStreamName(name)
        out.link(x.input)
    return p


def colorize(depth_mm, near, far):
    d = depth_mm.astype(np.float32) / 1000.0
    norm = np.clip((d - near) / (far - near), 0, 1)
    col = cv2.applyColorMap((255 * (1 - norm)).astype(np.uint8), cv2.COLORMAP_JET)   # near = red
    col[depth_mm == 0] = 0
    return col


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["all", "rgb", "depth", "rgbd"], default="all")
    ap.add_argument("--fps", type=float, default=15)
    ap.add_argument("--near", type=float, default=0.3)
    ap.add_argument("--far", type=float, default=1.5)
    ap.add_argument("--laser", type=float, default=0.8, help="IR dot projector 0..1")
    ap.add_argument("--no-rgb", action="store_true",
                    help="grey image from the left mono camera instead of the colour camera (colour camera broken)")
    a = ap.parse_args()
    names = ["RGB", "Depth", "RGBD"] if a.mode == "all" else [{"rgb": "RGB", "depth": "Depth", "rgbd": "RGBD"}[a.mode]]

    clicks = {}
    for n in names:
        cv2.namedWindow(n, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(n, 960, 540)
        cv2.setMouseCallback(n, lambda ev, x, y, fl, n=n: clicks.__setitem__("pt", (x, y)) if ev == cv2.EVENT_LBUTTONDOWN else None)

    with dai.Device(build_pipeline(a.fps, a.no_rgb)) as dev:
        print(f"Camera: {dev.getDeviceName()} {dev.getMxId()}, USB {dev.getUsbSpeed().name}")
        laser, flood = a.laser, 0.0
        dev.setIrLaserDotProjectorIntensity(laser)
        dev.setIrFloodLightIntensity(flood)
        q_rgb = dev.getOutputQueue("rgb", maxSize=2, blocking=False)
        q_depth = dev.getOutputQueue("depth", maxSize=2, blocking=False)
        bgr = depth = None
        t_fps, n_fps, fps = time.time(), 0, 0.0
        while True:
            m = q_rgb.tryGet()
            if m is not None:
                bgr = m.getCvFrame()
                if bgr.ndim == 2:                  # --no-rgb: grey mono frame
                    bgr = cv2.cvtColor(bgr, cv2.COLOR_GRAY2BGR)
                n_fps += 1
            m = q_depth.tryGet()
            if m is not None:
                depth = m.getFrame()
            if time.time() - t_fps > 1.0:
                fps, n_fps, t_fps = n_fps / (time.time() - t_fps), 0, time.time()
            if bgr is None or depth is None:
                cv2.waitKey(5)
                continue
            dcol = colorize(depth, a.near, a.far)
            views = {"RGB": bgr.copy(), "Depth": dcol.copy(), "RGBD": cv2.addWeighted(bgr, 0.55, dcol, 0.45, 0)}
            valid = 100.0 * np.count_nonzero(depth) / depth.size
            for n in names:
                img = views[n]
                label = {"RGB": "Mono", "RGBD": "Mono+D"}.get(n, n) if a.no_rgb else n
                cv2.putText(img, f"{label}  {fps:.1f} fps  depth hop le {valid:.0f}%  laser {laser:.1f}  flood {flood:.1f}",
                            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                if "pt" in clicks:
                    x, y = clicks["pt"]
                    if 0 <= y < depth.shape[0] and 0 <= x < depth.shape[1]:
                        patch = depth[max(0, y - 3):y + 4, max(0, x - 3):x + 4]
                        z = np.median(patch[patch > 0]) if np.any(patch > 0) else 0
                        cv2.drawMarker(img, (x, y), (255, 255, 255), cv2.MARKER_CROSS, 24, 2)
                        cv2.putText(img, f"{z:.0f} mm", (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.imshow(n, img)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                os.makedirs(SAVE_DIR, exist_ok=True)
                stamp = time.strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(os.path.join(SAVE_DIR, f"{stamp}_rgb.png"), bgr)
                cv2.imwrite(os.path.join(SAVE_DIR, f"{stamp}_depth_mm.png"), depth)       # 16-bit, mm
                cv2.imwrite(os.path.join(SAVE_DIR, f"{stamp}_depth_color.png"), dcol)
                cv2.imwrite(os.path.join(SAVE_DIR, f"{stamp}_rgbd.png"), views["RGBD"])
                print(f"Da luu {SAVE_DIR}/{stamp}_*.png")
            if key == ord("l"):
                laser = 0.0 if laser > 0 else a.laser
                dev.setIrLaserDotProjectorIntensity(laser)
            if key == ord("f"):
                flood = 0.0 if flood > 0 else 0.5
                dev.setIrFloodLightIntensity(flood)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
