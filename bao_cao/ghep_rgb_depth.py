#!/usr/bin/env python3
"""Side-by-side figures: colour image | colour image with depth overlaid (TURBO, 0.4-1.0 m),
pixels without depth left dark, with a depth scale bar. Built from snap_rgbd.py stills."""
import os
import cv2
import numpy as np

D = os.path.expanduser("~/fr3_hungros2/nhat_ky/du_lieu")
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hinh")
SCENES = {
    "ghep_home.jpg": "2026-11-02_021605_hop_gia_trang/anh/021609_truoc",
    "ghep_khay_den.jpg": "2026-11-01_161721_graspnet_tay_cam/anh/215652_khay_kiem_tra",
    "ghep_pregrasp_khay.jpg": "2026-11-01_165713_pregrasp_khay_den/anh/165805_sau",
    "ghep_nhac_gia.jpg": "2026-11-02_001611_pregrasp_gap_gia_trang_8/anh/001845_sau",
}
for name, stem in SCENES.items():
    bgr = cv2.imread(os.path.join(D, stem + "_rgb.jpg"))
    d = cv2.imread(os.path.join(D, stem + "_depth_mm.png"), cv2.IMREAD_UNCHANGED).astype(np.float32)
    col = cv2.applyColorMap(np.clip((d - 400) / 600 * 255, 0, 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    over = (0.35 * bgr).astype(np.uint8)                  # no depth: darkened colour image
    has = d > 0
    over[has] = (0.4 * bgr[has] + 0.6 * col[has]).astype(np.uint8)
    h, w = bgr.shape[:2]
    bar = cv2.applyColorMap(np.linspace(0, 255, 200).astype(np.uint8)[None].repeat(12, 0), cv2.COLORMAP_TURBO)
    over[h - 40:h - 28, w - 230:w - 30] = bar
    for v, x in (("0,4 m", w - 240), ("1,0 m", w - 62)):
        cv2.putText(over, v, (x, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    sep = np.full((h, 8, 3), 255, np.uint8)
    cv2.imwrite(os.path.join(OUT, name), np.hstack([bgr, sep, over]), [cv2.IMWRITE_JPEG_QUALITY, 92])
    print(name, f"{100 * has.mean():.0f}% pixel co depth")
