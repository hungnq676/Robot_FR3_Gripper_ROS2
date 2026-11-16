#!/usr/bin/env python3
"""Simulation figures for the report from the recorded Gazebo pick of the mouse (2026-11-02 04:5x):
virtual camera RGB | depth, the 5-step pick sequence, the jaw force sensors over the pick, and the
RFP602 model curve used by the emulator."""
import os
import shutil

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageDraw, ImageFont

B = os.path.dirname(os.path.abspath(__file__))
RUN = os.path.join(B, "du_lieu_mo_phong", "chuot")
MP = os.path.join(B, "hinh", "mo_phong")
OUT = os.path.join(B, "hinh")
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "savefig.dpi": 200})

# 1. virtual OAK-D: colour | colour with depth (0.8-2.0 m; the camera is 0.95 m above the table, tilted)
bgr = cv2.imread(os.path.join(RUN, "oak_rgb.png"))
d = cv2.imread(os.path.join(RUN, "oak_depth_mm.png"), cv2.IMREAD_UNCHANGED).astype(np.float32)
col = cv2.applyColorMap(np.clip((d - 800) / 1200 * 255, 0, 255).astype(np.uint8), cv2.COLORMAP_TURBO)
over = (0.35 * bgr).astype(np.uint8)
has = d > 0
over[has] = (0.4 * bgr[has] + 0.6 * col[has]).astype(np.uint8)
h, w = bgr.shape[:2]
over[h - 50:h - 34, w - 300:w - 40] = cv2.applyColorMap(
    np.linspace(0, 255, 260).astype(np.uint8)[None].repeat(16, 0), cv2.COLORMAP_TURBO)
for v, x in (("0,8 m", w - 315), ("2,0 m", w - 75)):
    cv2.putText(over, v, (x, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2, cv2.LINE_AA)
cv2.imwrite(os.path.join(OUT, "mp_oak_ghep.jpg"), np.hstack([bgr, np.full((h, 10, 3), 255, np.uint8), over]),
            [cv2.IMWRITE_JPEG_QUALITY, 92])
shutil.copy(os.path.join(RUN, "gemini_annotated.png"), os.path.join(OUT, "mp_gemini.png"))

# 2. pick sequence, 5 Gazebo frames
frames = [(15, "(a) tiếp cận"), (27, "(b) hạ thẳng xuống"), (33, "(c) kẹp, dừng theo lực"),
          (36, "(d) nhấc lên"), (45, "(e) mang tới vị trí đặt")]
try:
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 26)
except OSError:
    font = ImageFont.load_default()
tiles = []
for i, lab in frames:
    im = Image.open(os.path.join(MP, f"chuot_{i:02d}.png")).convert("RGB")
    im = im.crop((0, int(im.height * 0.06), im.width, int(im.height * 0.66)))
    dr = ImageDraw.Draw(im)
    dr.rectangle((0, im.height - 40, im.width, im.height), fill="white")
    dr.text((10, im.height - 36), lab, fill="black", font=font)
    tiles.append(im)
tw, th = tiles[0].size
sheet = Image.new("RGB", (tw * 3 + 20, th * 2 + 10), "white")
for k, im in enumerate(tiles):
    r, c = divmod(k, 3)
    x = c * (tw + 10) + (0 if r == 0 else (tw + 10) // 2)
    sheet.paste(im, (x, r * (th + 10)))
sheet.save(os.path.join(OUT, "mp_chuoi_gap.png"))

# 3. jaw forces over the pick; phases from the MTC log, aligned to recorder time (t0 = log - 259.2 s)
f = np.genfromtxt(os.path.join(RUN, "luc.csv"), delimiter=",", names=True)
phases = [(30.6, 34.3, "lập kế hoạch", "#dddddd"), (34.5, 42.3, "tiếp cận", "#cfe3f7"),
          (42.3, 62.1, "hạ xuống", "#e0d4f5"), (62.1, 66.1, "kẹp", "#ffd8b0"),
          (66.5, 90.7, "nhấc lên", "#d3f0d3"), (90.7, 101.7, "mang đi", "#f5f0c0")]
fig, axs = plt.subplots(1, 2, figsize=(12, 4.2), gridspec_kw={"width_ratios": [2.2, 1]})
for ax, (a, b) in zip(axs, ((25, 105), (61.5, 67.5))):
    m = (f["t"] >= a) & (f["t"] <= b)
    for j, (p0, p1, lab, c) in enumerate(phases):
        if p1 > a and p0 < b:
            ax.axvspan(max(p0, a), min(p1, b), color=c, lw=0)
            if ax is axs[0]:
                ax.text((max(p0, a) + min(p1, b)) / 2, 8.7 - 0.6 * (j % 2), lab, ha="center", fontsize=8.5)
    ax.plot(f["t"][m], f["force_1"][m], color="#d62728", lw=1.2, label="má 1 (clamp_1)")
    ax.plot(f["t"][m], f["force_2"][m], color="#1f77b4", lw=1.2, label="má 2 (clamp_2)")
    ax.axhline(2.0, color="k", ls="--", lw=1)
    ax.set_xlim(a, b); ax.set_ylim(0, 9.3); ax.set_xlabel("thời gian (s)"); ax.grid(alpha=0.3)
axs[0].set_ylabel("lực đo trên tấm cảm biến (N)")
axs[0].text(26, 2.2, "ngưỡng dừng kẹp 2 N", fontsize=8.5)
axs[0].legend(loc="center left", fontsize=8.5)
axs[0].set_title("Toàn bộ lần gắp")
axs[1].set_title("Phóng to giai đoạn kẹp")
k = np.where((f["force_1"] >= 2) & (f["force_2"] >= 2))[0]
if len(k):
    t = f["t"][k[0]]
    axs[1].axvline(t, color="k", lw=1)
    axs[1].annotate(f"cả hai má ≥ 2 N\nt = {t:.1f} s → dừng kẹp", (t, 2), (t - 3.6, 5.5), fontsize=8.5,
                    arrowprops=dict(arrowstyle="->"))
fig.tight_layout()
fig.savefig(os.path.join(OUT, "mp_luc_kep.png"), bbox_inches="tight", facecolor="white"); plt.close(fig)

# 4. RFP602 model: force -> resistance -> 12-bit ADC count (R = 30000/F, 10 kOhm divider)
F = np.linspace(0.2, 49, 400)
R = 30000 / F
adc = 4095 * 10000 / (10000 + R)
fig, ax = plt.subplots(figsize=(7.5, 3.8))
ax.plot(F, adc, color="#1f77b4", lw=2)
ax.set_xlabel("lực ép F (N)"); ax.set_ylabel("giá trị ADC 12 bit", color="#1f77b4")
ax2 = ax.twinx()
ax2.plot(F, R / 1000, color="#d62728", lw=1.5, ls="--")
ax2.set_ylabel("điện trở cảm biến R (kΩ)", color="#d62728"); ax2.set_yscale("log")
for v in (0.5, 2.0, 6.3):
    ax.plot(v, 4095 * 10000 / (10000 + 30000 / v), "ko", ms=4)
    ax.annotate(f"{v} N", (v, 4095 * 10000 / (10000 + 30000 / v)), (v + 2, 4095 * 10000 / (10000 + 30000 / v) - 250),
                fontsize=8.5)
ax.grid(alpha=0.3); ax.set_title("Mô hình cảm biến RFP602 trong mô phỏng: R = 30 000 / F, cầu chia 10 kΩ")
fig.tight_layout()
fig.savefig(os.path.join(OUT, "mp_fsr_dac_tuyen.png"), bbox_inches="tight", facecolor="white"); plt.close(fig)
print("xong", f"lực max {f['force_1'].max():.1f}/{f['force_2'].max():.1f} N")
