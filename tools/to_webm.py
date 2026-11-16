#!/usr/bin/env python3
"""Re-encode videos recorded before 2026-11-01 17:00 (mp4v .mp4, which the GNOME player
cannot open here) to VP8 .webm next to them. Usage: to_webm.py FILE.mp4 [...]"""
import sys

import cv2

for src in sys.argv[1:]:
    c = cv2.VideoCapture(src)
    fps, w, h = c.get(cv2.CAP_PROP_FPS) or 15.0, int(c.get(3)), int(c.get(4))
    dst = src.rsplit(".", 1)[0] + ".webm"
    out = cv2.VideoWriter(dst, cv2.VideoWriter_fourcc(*"VP80"), fps, (w, h))
    n = 0
    while True:
        ok, f = c.read()
        if not ok:
            break
        out.write(f)
        n += 1
    out.release()
    print(f"{dst}: {n} khung")
