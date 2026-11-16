#!/usr/bin/env python3
"""Point-cloud figures for the report from a real run (2026-11-02 02:16): the scene cloud the
camera measured (robot base frame), the crop that goes into GraspNet (20 cm around the object,
table up to 25 cm), the object's SAM points, and the GraspNet grasps drawn as gripper outlines."""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

RUN = os.path.expanduser("~/fr3_hungros2/nhat_ky/du_lieu/2026-11-02_021605_hop_gia_trang/anh")
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hinh")
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "savefig.dpi": 200})

ply = os.path.join(RUN, "021609_truoc_cloud.ply")
with open(ply) as f:
    n = next(int(l.split()[-1]) for l in f if l.startswith("element vertex"))
    while not next(f).startswith("end_header"):
        pass
data = np.loadtxt(ply, skiprows=10, max_rows=n)
P, C = data[:, :3] * 1000, data[:, 3:] / 255.0           # mm, rgb 0..1
res = yaml.safe_load(open(os.path.join(RUN, "021715_graspnet_white_object.yaml")))
obj = np.load(os.path.join(RUN, "021715_graspnet_white_object_object_points_base.npy")) * 1000
grasps = res["graspnet"][:5]
c = np.median(obj[:, :2], 0)
table_z = 12.0

# GraspNet input: within 200 mm of the object in xy, from the table up to 250 mm
near = (np.linalg.norm(P[:, :2] - c, axis=1) < 200) & (P[:, 2] > table_z - 30) & (P[:, 2] < table_z + 250)
S, SC = P[near], C[near]
print(f"scene {len(P)} pts, GraspNet crop {len(S)}, object {len(obj)}")


def gripper(g):
    """GraspNet gripper outline (mm): x = approach, y = closing, tips at t + depth*x."""
    T = np.array(g["T_base"])
    t, x, y = T[:3, 3] * 1000, T[:3, 0], T[:3, 1]
    w, d = g["width"] * 1000, g["depth"] * 1000
    tip_l, tip_r = t + d * x + w / 2 * y, t + d * x - w / 2 * y
    base_l, base_r = t - 20 * x + w / 2 * y, t - 20 * x - w / 2 * y
    stem = t - 20 * x, t - 60 * x
    return [np.array([tip_l, base_l, base_r, tip_r]), np.array(stem)]


cols = ["#d62728", "#ff7f0e", "#e5ae00", "#2ca02c", "#1f77b4"]
rng = np.random.default_rng(0)

# Figure 1: whole scene cloud in the robot frame (subsampled), crop box marked
tbl = (P[:, 2] > -20) & (P[:, 2] < 300) & (P[:, 0] > -800) & (P[:, 0] < 150) & (P[:, 1] > -250) & (P[:, 1] < 750)
Pt, Ct = P[tbl], C[tbl]
k = rng.choice(len(Pt), 40000, replace=False)
fig = plt.figure(figsize=(10, 7.5))
ax = fig.add_subplot(111, projection="3d")
ax.scatter(Pt[k, 0], Pt[k, 1], Pt[k, 2], c=Ct[k], s=1.0, linewidths=0)
th = np.linspace(0, 2 * np.pi, 100)
for z in (table_z, table_z + 250):
    ax.plot(c[0] + 200 * np.cos(th), c[1] + 200 * np.sin(th), z, color="#d62728", lw=1.2)
ax.scatter([0], [0], [0], c="k", s=60, marker="s")
ax.text(0, 0, 40, "gốc robot", fontsize=9)
for v, col, lab in (((150, 0, 0), "r", "x"), ((0, 150, 0), "g", "y"), ((0, 0, 150), "b", "z")):
    ax.quiver(0, 0, 0, *v, color=col, lw=2, arrow_length_ratio=0.15)
    ax.text(*(np.array(v) * 1.15), lab, color=col, fontsize=11, fontweight="bold")
ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)"); ax.set_zlabel("z (mm)")
ax.set_xlim(-800, 150); ax.set_ylim(-250, 750); ax.set_zlim(-20, 300); ax.view_init(elev=35, azim=-130)
ax.text(c[0], c[1], table_z + 270, "vùng đưa vào GraspNet", color="#d62728", fontsize=9)
ax.set_title("Đám mây điểm của cả bàn trong hệ toạ độ robot (base_footprint)")
fig.savefig(os.path.join(OUT, "dam_may_ca_ban.png"), bbox_inches="tight", facecolor="white"); plt.close(fig)

# Figure 2: GraspNet input crop + object points + top-5 grasps (3D)
k = rng.choice(len(S), min(len(S), 20000), replace=False)
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")
ax.scatter(S[k, 0], S[k, 1], S[k, 2], c=SC[k], s=1.2, linewidths=0, alpha=0.6)
ax.scatter(obj[:, 0], obj[:, 1], obj[:, 2], c="#7b2cbf", s=1.5, linewidths=0, label="điểm của vật (mặt nạ SAM)")
for i, g in enumerate(grasps):
    fing, stem = gripper(g)
    ax.plot(*fing.T, color=cols[i], lw=2.2, label=f"#{i + 1}: điểm số {g['score']:.2f}, mở {g['width'] * 1000:.0f} mm")
    ax.plot(*stem.T, color=cols[i], lw=2.2)
ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)"); ax.set_zlabel("z (mm)")
ax.set_xlim(c[0] - 200, c[0] + 200); ax.set_ylim(c[1] - 200, c[1] + 200); ax.set_zlim(0, 200)
ax.view_init(elev=30, azim=-120)
ax.legend(loc="upper left", fontsize=8.5)
ax.set_title(f"Đầu vào GraspNet: {len(S)} điểm trong bán kính 20 cm quanh vật, và 5 tư thế gắp tốt nhất")
fig.savefig(os.path.join(OUT, "dam_may_graspnet_3d.png"), bbox_inches="tight", facecolor="white"); plt.close(fig)

# Figure 3: top (x-y) and side (x-z) projections of the crop with grasps
fig, axs = plt.subplots(1, 2, figsize=(12, 5.5))
for ax, (a, b, lab) in zip(axs, ((0, 1, ("x (mm)", "y (mm)")), (0, 2, ("x (mm)", "z (mm)")))):
    ax.scatter(S[:, a], S[:, b], c=SC, s=0.5, linewidths=0, alpha=0.5)
    ax.scatter(obj[:, a], obj[:, b], c="#7b2cbf", s=0.8, linewidths=0)
    for i, g in enumerate(grasps):
        fing, stem = gripper(g)
        ax.plot(fing[:, a], fing[:, b], color=cols[i], lw=2)
        ax.plot(stem[:, a], stem[:, b], color=cols[i], lw=2)
        ax.text(fing[0, a], fing[0, b], f"#{i + 1}", color=cols[i], fontsize=9, fontweight="bold")
    ax.set_xlabel(lab[0]); ax.set_ylabel(lab[1]); ax.set_aspect("equal"); ax.grid(alpha=0.3)
axs[0].set_title("Nhìn từ trên xuống (mặt phẳng x–y)")
axs[1].set_title("Nhìn ngang (mặt phẳng x–z), mặt bàn ở z ≈ 12 mm")
axs[1].axhline(table_z, color="#888888", ls="--", lw=1)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "dam_may_graspnet_2d.png"), bbox_inches="tight", facecolor="white"); plt.close(fig)
print("xong")
