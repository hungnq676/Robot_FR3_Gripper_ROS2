#!/usr/bin/env python3
"""Diagrams and charts for the research report (bao_cao/hinh/*.png), drawn from the
numbers recorded in nhat_ky/NHAT_KY.md and the run logs."""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch, Rectangle, Wedge
import numpy as np

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hinh")
os.makedirs(OUT, exist_ok=True)
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11, "savefig.dpi": 200})
INK, EDGE, ACC, ACC2, GOOD, BAD = "#222222", "#555555", "#1f6fb2", "#d9822b", "#2e8b57", "#c0392b"


def box(ax, x, y, w, h, title, sub=None, fc="white", ec=EDGE, lw=1.2, fs=11):
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h, boxstyle="round,pad=0.02,rounding_size=0.08",
                                fc=fc, ec=ec, lw=lw))
    if sub:
        ax.text(x, y + h * 0.14, title, ha="center", va="center", fontsize=fs, fontweight="bold", color=INK)
        ax.text(x, y - h * 0.2, sub, ha="center", va="center", fontsize=fs - 2, color="#444444")
    else:
        ax.text(x, y, title, ha="center", va="center", fontsize=fs, fontweight="bold", color=INK)


def arrow(ax, p, q, text=None, color=EDGE, style="-|>", ls="-", rad=0.0, fs=9, off=(0, 0.12)):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle=style, mutation_scale=14, color=color, lw=1.3, ls=ls,
                                 connectionstyle=f"arc3,rad={rad}"))
    if text:
        ax.text((p[0] + q[0]) / 2 + off[0], (p[1] + q[1]) / 2 + off[1], text, ha="center", va="center",
                fontsize=fs, color="#333333")


def save(fig, name):
    fig.savefig(os.path.join(OUT, name), bbox_inches="tight", facecolor="white")
    plt.close(fig)


# 1. System architecture
fig, ax = plt.subplots(figsize=(11, 6.2))
ax.set_xlim(0, 11); ax.set_ylim(0, 6.2); ax.axis("off")
ax.add_patch(Rectangle((0.2, 0.2), 6.9, 5.8, fc="#f4f7fb", ec="#9db4cc", lw=1))
ax.text(0.4, 5.75, "Máy tính ROG (Ubuntu 22.04, ROS 2 Humble, GPU GTX 1660 Ti)", fontsize=10, color="#2b4a6b", fontweight="bold")
box(ax, 1.6, 4.8, 2.3, 0.8, "Driver camera", "astra_camera, 640×480")
box(ax, 1.6, 3.4, 2.3, 0.8, "Nhận thức", "Gemini, SAM, GraspNet")
box(ax, 1.6, 2.0, 2.3, 0.8, "Vật cản", "scene_obstacles.py")
box(ax, 4.6, 4.8, 2.6, 0.8, "TF hand-eye", "cam_tf.py")
box(ax, 4.6, 3.4, 2.6, 0.8, "MoveIt 2 + MTC", "mtc_pregrasp.py", fc="#e7f0fa", ec=ACC, lw=2)
box(ax, 4.6, 2.0, 2.6, 0.8, "ros2_control", "fairino_hardware (ServoJ)")
box(ax, 4.6, 0.75, 2.6, 0.7, "gripper.py", "HTTP tới ESP32")
box(ax, 9.2, 4.8, 2.6, 0.8, "Camera Astra Pro Plus", "eye-to-hand, cao 0,7 m")
box(ax, 9.2, 2.0, 2.6, 0.8, "Robot Fairino FR3", "6 bậc tự do, 192.168.58.2")
box(ax, 9.2, 0.75, 2.6, 0.7, "ESP32-CAM + BusLinker", "servo Hiwonder ID 1")
box(ax, 9.2, 3.4, 2.6, 0.8, "Gemini API (Google)", "khung bao theo câu lệnh")
def poly(ax, pts, text=None, tpos=None):
    xs, ys = zip(*pts)
    ax.plot(xs[:-1], ys[:-1], color=EDGE, lw=1.3)
    arrow(ax, pts[-2], pts[-1])
    if text:
        ax.text(tpos[0], tpos[1], text, ha="center", fontsize=9, color="#333333")
poly(ax, [(9.2, 5.2), (9.2, 5.45), (1.6, 5.45), (1.6, 5.2)], "USB: ảnh màu + depth 30 Hz", (6.9, 5.52))
arrow(ax, (1.6, 4.4), (1.6, 3.8))
arrow(ax, (1.6, 4.4), (1.6, 2.4), rad=0.35)
arrow(ax, (2.75, 3.4), (3.3, 3.4))
arrow(ax, (2.75, 2.0), (3.3, 3.2))
arrow(ax, (4.6, 4.4), (4.6, 3.8))
arrow(ax, (4.6, 3.0), (4.6, 2.4))
arrow(ax, (5.9, 2.0), (7.9, 2.0), "Ethernet: lệnh ServoJ 8 ms", off=(0, 0.15))
arrow(ax, (5.9, 0.75), (7.9, 0.75), "Wi-Fi: HTTP", off=(0, 0.15))
poly(ax, [(5.9, 3.25), (6.4, 3.25), (6.4, 1.2), (5.0, 1.2), (5.0, 1.1)])
poly(ax, [(1.0, 3.0), (1.0, 2.7), (7.3, 2.7), (7.3, 3.4), (7.9, 3.4)], "ảnh + câu lệnh", (2.6, 2.78))
save(fig, "kien_truc.png")

# 2. Frames: eye-to-hand loop
fig, ax = plt.subplots(figsize=(10, 4.9))
ax.set_xlim(0, 10); ax.set_ylim(-0.3, 4.6); ax.axis("off")
box(ax, 1.5, 1.0, 2.2, 0.8, "base_footprint", "gốc robot")
box(ax, 5.0, 1.0, 2.2, 0.8, "wrist2_link", "khâu J5")
box(ax, 8.4, 1.0, 2.2, 0.8, "AprilTag id 2", "33,9 mm")
box(ax, 5.0, 3.7, 2.6, 0.8, "camera_color_optical", "Astra")
arrow(ax, (2.6, 1.0), (3.9, 1.0), "động học\nthuận (biết)", off=(0, -0.75))
arrow(ax, (6.1, 1.0), (7.3, 1.0), "Y: tag trên\nkhâu (cần tìm)", off=(0, -0.75), color=ACC2)
arrow(ax, (1.8, 1.4), (4.0, 3.4), "X: camera so với\ngốc (cần tìm)", color=ACC2, off=(-1.2, 0.2))
arrow(ax, (6.0, 3.4), (8.2, 1.4), "PnP từ ảnh (đo)", off=(0.9, 0.25))
ax.text(5.0, 2.25, "X · T_cam→tag = T_base→wrist2 · Y", ha="center", fontsize=12, color=INK,
        bbox=dict(fc="#fff8ec", ec=ACC2))
save(fig, "he_toa_do.png")

# 3. MTC task tree
fig, ax = plt.subplots(figsize=(10, 6))
ax.set_xlim(0, 10); ax.set_ylim(0, 6); ax.axis("off")
steps = [("CurrentState", "trạng thái hiện tại"), ("Connect (Pilz PTP)", "tới điểm chờ, chi phí PathLength"),
         ("MoveRelative (Cartesian)", "tiến vào 80 mm theo trục TCP"), ("ComputeIK ← GeneratePose", "tư thế gắp, 16 nghiệm IK"),
         ("MoveRelative +z", "nhấc lên 20 cm"), ("Connect (Pilz PTP)", "mang tới trên hộp"),
         ("MoveRelative + ComputeIK", "hạ vào hộp, 4 hướng thả"), ("MoveRelative +z", "nhấc khỏi hộp"),
         ("MoveTo (Pilz PTP)", "về HOME")]
for i, (t, s) in enumerate(steps):
    y = 5.5 - i * 0.62
    fc = "#e7f0fa" if i in (3, 6) else "white"
    box(ax, 3.4, y, 4.4, 0.5, t, None, fc=fc, fs=10)
    ax.text(5.8, y, s, va="center", fontsize=9.5, color="#333333")
    if i < len(steps) - 1:
        arrow(ax, (3.4, y - 0.25), (3.4, y - 0.37))
ax.text(0.3, 5.5 - 3 * 0.62, "gripper\nkẹp sau\nđoạn 2", fontsize=9, color=GOOD, va="center")
ax.text(0.3, 5.5 - 6 * 0.62, "gripper\nmở sau\nđoạn 5", fontsize=9, color=GOOD, va="center")
save(fig, "mtc_task.png")

# 4. Gripper geometry (side view, mm)
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_xlim(-130, 130); ax.set_ylim(-20, 200); ax.set_aspect("equal"); ax.axis("off")
ax.add_patch(Rectangle((-32, 182), 64, 18, fc="#dddddd", ec=EDGE))
ax.text(40, 191, "mặt bích FR3 (Ø63)", fontsize=9, va="center")
ax.add_patch(Rectangle((-37, 154), 74, 28, fc="#eeeeee", ec=EDGE))
ax.text(42, 168, "vành chuyển tiếp 28 mm", fontsize=9, va="center")
ax.add_patch(Rectangle((-82, 81), 164, 73, fc="#f4f4f4", ec=EDGE))
ax.text(0, 117, "thân gripper 164 × 85 × 73 mm\nservo Hiwonder + ESP32-CAM", ha="center", fontsize=9)
for s in (-1, 1):
    ax.add_patch(Rectangle((s * 44 if s > 0 else -82, 21), 38, 60, fc="#cfe2f3", ec=ACC))
ax.annotate("", xy=(-44, 10), xytext=(44, 10), arrowprops=dict(arrowstyle="<->", color=INK))
ax.text(0, 2, "mở tối đa 88 mm (khép tới ≈ 30 mm)", ha="center", fontsize=9)
ax.plot([-100, 100], [51, 51], ls="--", color=ACC2)
ax.text(-125, 51, "TCP\n(grasp_frame)", fontsize=9, color=ACC2, va="center")
ax.annotate("", xy=(105, 21), xytext=(105, 51), arrowprops=dict(arrowstyle="<->", color=ACC2))
ax.text(108, 36, "30 mm", fontsize=9, color=ACC2, va="center")
ax.annotate("", xy=(122, 21), xytext=(122, 182), arrowprops=dict(arrowstyle="<->", color=INK))
ax.text(124, 110, "165 mm", fontsize=9, rotation=90, va="center")
ax.annotate("", xy=(-95, 21), xytext=(-95, 81), arrowprops=dict(arrowstyle="<->", color=INK))
ax.text(-128, 72, "ngón\n60 mm", fontsize=9, va="center")
save(fig, "gripper_hinh_hoc.png")

# 5. Workspace top view (base_footprint, mm)
fig, ax = plt.subplots(figsize=(7.5, 7))
ax.set_aspect("equal")
for r, ls in ((300, ":"), (450, ":"), (622, "-")):
    ax.add_patch(Circle((0, 0), r, fill=False, ls=ls, ec="#888888"))
    ax.text(r * np.cos(np.radians(100)), r * np.sin(np.radians(100)), f"{r} mm", fontsize=8, color="#666666")
ax.add_patch(Wedge((0, 0), 622, 158, 175, fc=BAD, alpha=0.12, ec="none"))
for a in (175, -175):
    ax.plot([0, 650 * np.cos(np.radians(a))], [0, 650 * np.sin(np.radians(a))], color=BAD, lw=1.5)
ax.plot([0, 650 * np.cos(np.radians(158))], [0, 650 * np.sin(np.radians(158))], color=BAD, ls="--", lw=1)
ax.text(-640, 140, "vùng sát giới hạn j1 = ±175°\n(phải lật tay)", color=BAD, fontsize=9)
ax.add_patch(Rectangle((-266 - 150, 479 - 115), 300, 230, angle=0, fc="#f6e27a", ec="#b59b00", alpha=0.7))
ax.text(-266, 420, "hộp vàng (02:16)", ha="center", fontsize=9)
pts = {"giá trắng (thành công 00:16)": (-468, 224), "giá trắng (02:16)": (-325, 171), "khay đen (23:29)": (-269, 520),
       "khối đỏ": (-347, -4)}
for (k, (x, y)), c in zip(pts.items(), (GOOD, ACC, "#333333", BAD)):
    ax.plot(x, y, "o", color=c, ms=8)
    ax.text(x + 12, y + 12, k, fontsize=8.5, color=c)
ax.plot(0, 0, "ks", ms=10); ax.text(15, -35, "đế robot", fontsize=9)
ax.set_xlim(-700, 200); ax.set_ylim(-250, 700)
ax.set_xlabel("x (mm, hệ base_footprint)"); ax.set_ylabel("y (mm)")
ax.grid(alpha=0.25)
save(fig, "vung_lam_viec.png")

# 6. Hand-eye calibration rounds (LOO reprojection error)
rounds = ["OAK lần 2\n(30/10)", "OAK lần 3", "OAK lần 4", "OAK lần 5\n(tag 33,9)", "OAK lần 6", "Astra\n31/10",
          "Astra\n01/11", "Astra\n02/11\n00:05", "Astra\n02/11\n17:00"]
loo = [0.67, 0.53, 1.60, 0.73, 0.69, 0.94, 0.91, 0.81, 0.87]
n = [9, 10, 18, 16, 14, 21, 21, 19, 23]
fig, ax = plt.subplots(figsize=(11, 4.5))
bars = ax.bar(rounds, loo, color=[ACC] * 5 + [ACC2] * 4)
for b, v, k in zip(bars, loo, n):
    ax.text(b.get_x() + b.get_width() / 2, v + 0.03, f"{v:.2f} px\n{k} mẫu", ha="center", fontsize=8.5)
ax.set_ylabel("sai số chiếu kiểm tra chéo LOO (px)"); ax.set_ylim(0, 1.95)
ax.spines[["top", "right"]].set_visible(False)
save(fig, "handeye_cac_lan.png")

# 7. ServoJ queue before/after the driver fix
fig, axs = plt.subplots(1, 2, figsize=(10, 4))
axs[0].bar(["trước khi sửa\n(đầu phiên)", "trước khi sửa\n(sau một lúc)", "sau khi sửa"], [15, 124, 3],
           color=[BAD, BAD, GOOD])
axs[0].set_ylabel("số lệnh ServoJ trong hàng đợi"); axs[0].set_title("Hàng đợi lệnh của bộ điều khiển", fontsize=11)
for i, v in enumerate([15, 124, 3]):
    axs[0].text(i, v + 2, f"{v}" if i < 2 else "0–3", ha="center")
axs[1].bar(["OK\n(< 1,5°)", "cảnh báo\n(1,5–2,8°)", "bất thường"], [22, 8, 1], color=[GOOD, ACC2, BAD])
axs[1].set_title("31 lần chạy sau khi sửa (track_watch.log)", fontsize=11); axs[1].set_ylabel("số lần")
for i, v in enumerate([22, 8, 1]):
    axs[1].text(i, v + 0.4, str(v), ha="center")
for a in axs:
    a.spines[["top", "right"]].set_visible(False)
save(fig, "hang_doi_servoj.png")

# 8. Depth correction model over the image
fx, cx, cy = 544.19, 315.32, 233.92
u, v = np.meshgrid(np.arange(640), np.arange(480))
ratio = 1.0170 - 0.0734 * (u - cx) / fx - 0.0297 * (v - cy) / fx
fig, ax = plt.subplots(figsize=(7.5, 5))
im = ax.imshow(ratio, cmap="coolwarm", origin="upper")
cs = ax.contour(ratio, levels=np.arange(0.97, 1.08, 0.01), colors="k", linewidths=0.6)
ax.clabel(cs, fmt="%.2f", fontsize=8)
fig.colorbar(im, ax=ax, label="Z đo / z thật")
ax.set_xlabel("u (px)"); ax.set_ylabel("v (px)")
save(fig, "mo_hinh_depth.png")
print("xong:", sorted(os.listdir(OUT)))

# 9. Gripper electronics block diagram
fig, ax = plt.subplots(figsize=(11, 6.0))
ax.set_xlim(0, 11); ax.set_ylim(-0.1, 6.3); ax.axis("off")
box(ax, 1.7, 5.3, 2.8, 0.9, "Máy tính ROG", "ROS 2 + micro-ROS Agent")
box(ax, 5.5, 5.3, 2.8, 0.9, "Bộ phát Wi-Fi", "mạng phòng thí nghiệm")
box(ax, 5.5, 3.3, 3.0, 1.3, "ESP32 (mạch ESP32-CAM)", "micro-ROS client\n+ HTTP chẩn đoán, OTA", fc="#e7f0fa", ec=ACC, lw=2)
box(ax, 9.2, 3.3, 2.6, 1.1, "BusLinker V3", "chuyển UART ↔ bus\nbán song công")
box(ax, 9.2, 1.0, 2.6, 1.0, "Servo bus Hiwonder", "35 kg·cm, ID 1, 0–240°")
box(ax, 5.5, 1.0, 3.0, 1.0, "Nguồn servo", "đo được 13,4 V tại servo")
arrow(ax, (3.1, 5.3), (4.1, 5.3), "Wi-Fi", off=(0, 0.18), style="<|-|>")
arrow(ax, (5.5, 4.85), (5.5, 3.95), "Wi-Fi 2,4 GHz\nUDP (XRCE-DDS)", off=(-1.0, 0), style="<|-|>")
arrow(ax, (7.0, 3.55), (7.9, 3.55), None)
arrow(ax, (7.9, 3.05), (7.0, 3.05), None)
ax.text(7.45, 4.05, "U0T (GPIO1) → RX", fontsize=9, ha="center", color="#333333")
ax.text(7.45, 2.45, "U0R (GPIO3) ← TX", fontsize=9, ha="center", color="#333333")
arrow(ax, (9.2, 2.75), (9.2, 1.5), "bus 1 dây\n115200 baud", off=(0.75, 0), style="<|-|>")
arrow(ax, (6.6, 1.5), (8.4, 2.75), None, rad=0.0)
ax.text(7.95, 1.85, "cấp nguồn\ncho BusLinker", fontsize=8.5, ha="center", color="#333333")
ax.text(1.7, 3.3, "Camera OV2640 có sẵn trên\nmạch ESP32-CAM: không dùng,\ntắt bằng GPIO32 = 1", fontsize=9, ha="center", va="center", color="#777777")
ax.text(5.5, 0.0, "GND chung giữa ESP32, BusLinker và nguồn; jumper BusLinker đặt ở chế độ Servo-TTL", ha="center", fontsize=9, color="#333333")
save(fig, "gripper_dien_tu.png")

# 10. Gripper software layers (micro-ROS)
fig, ax = plt.subplots(figsize=(11, 7.6))
ax.set_xlim(0, 11); ax.set_ylim(-0.2, 7.6); ax.axis("off")
layers = [
    (6.95, "Kịch bản gắp (mtc_pregrasp.py) và công cụ thử", "mở trước khi tiến vào → kẹp theo bề dày đo từ camera → mở khi thả", "white"),
    (5.8, "Thư viện gripper.py (máy tính)", "open(), grip_measured(), free(), status(); kiểm tra nhiệt độ; thử lại 3 lần", "white"),
    (4.65, "Đồ thị ROS 2 trên máy tính (DDS)", "topic lệnh kẹp  →  ESP32      ·      ESP32  →  topic trạng thái servo", "#f4f7fb"),
    (3.5, "micro-ROS Agent (máy tính)", "cầu nối giữa DDS của ROS 2 và giao thức XRCE-DDS gọn nhẹ, qua UDP trên Wi-Fi", "#f4f7fb"),
    (2.35, "Firmware ESP32: micro-ROS client (rclc)", "nút gripper, subscriber lệnh, publisher trạng thái; lệnh cho phép, chặn ≥ 60 °C, tự dừng 1 s", "#e7f0fa"),
    (1.2, "Giao thức bus Hiwonder (UART 115200)", "0x55 0x55 ID LEN CMD tham số… CHECKSUM", "#f4f7fb"),
    (0.15, "Servo Hiwonder (vòng điều khiển vị trí bên trong)", "vị trí 0–1000 = 0–240°, đọc nhiệt độ, điện áp, vị trí", "white"),
]
for y, t, sub, fc in layers:
    box(ax, 5.5, y, 10.2, 0.85, t, sub, fc=fc, fs=11)
for k in range(len(layers) - 1):
    arrow(ax, (5.5, layers[k][0] - 0.43), (5.5, layers[k + 1][0] + 0.43), style="<|-|>")
save(fig, "gripper_phan_mem.png")

# 11. Grip flowchart (grip_measured)
fig, ax = plt.subplots(figsize=(10, 7.4))
ax.set_xlim(0, 10); ax.set_ylim(0, 7.4); ax.axis("off")
box(ax, 5, 6.8, 7.6, 0.7, "Bề dày phần vật giữa hai má e (mm, đo từ camera)")
box(ax, 5, 5.75, 7.6, 0.7, "Góc servo lúc chạm: pos_chạm = 1000 · (e − 30) / 58")
box(ax, 5, 4.7, 7.6, 0.7, "Đích = pos_chạm − 5° (≈ 21 đơn vị); kiểm tra nhiệt độ < 60 °C")
box(ax, 5, 3.6, 4.6, 0.8, "Servo bị chặn trước đích?", None, fc="#fff8ec", ec=ACC2)
box(ax, 1.9, 2.2, 3.2, 0.9, "Đang ép vào vật", "lực ≈ độ lệch tới đích", fc="#eaf6ee", ec=GOOD)
box(ax, 7.6, 2.2, 3.6, 0.9, "Chưa chạm: khép tiếp", "về hết hành trình tới khi bị chặn")
box(ax, 7.6, 0.75, 3.6, 0.9, "Bị chặn tại p: đích = p − 5°", "(không ép thêm nếu p sát cuối hành trình)", fc="#eaf6ee", ec=GOOD)
for a_, b_ in (((5, 6.45), (5, 6.1)), ((5, 5.4), (5, 5.05)), ((5, 4.35), (5, 4.0))):
    arrow(ax, a_, b_)
arrow(ax, (2.7, 3.6), (1.9, 2.65), "có", off=(-0.3, 0.2))
arrow(ax, (7.3, 3.6), (7.6, 2.65), "không", off=(0.45, 0.2))
arrow(ax, (7.6, 1.75), (7.6, 1.2))
save(fig, "gripper_luu_do_kep.png")
print("xong gripper")

# 12. Grasp filter funnel (real run 2026-11-02 17:38, white holder lying flat) and the earlier 21:14 run
fig, axs = plt.subplots(1, 2, figsize=(13, 5.2), gridspec_kw={"width_ratios": [1.35, 1]})
stages = [("GraspNet đề xuất", 133), ("Sau 4 bộ lọc trong grasp_once\n(trên vật, không va chạm,\nđộ mở 40–85 mm, ăn sâu ≤ 30 mm)", 11),
          ("Thử cả hai chiều\n(xoay 180° quanh hướng tiếp cận)", 22), ("Lập được kế hoạch MTC\n(có IK, không va chạm cảnh)", 16),
          ("Phần bị kẹp dày ≥ 33 mm,\nmá không đè lên vật", 12), ("Được chọn (độ khó thấp nhất)", 1)]
ax = axs[0]
mx = max(v for _, v in stages)
for i, (lab, v) in enumerate(stages):
    y = len(stages) - 1 - i
    w = 0.08 + 0.92 * v / mx
    ax.barh(y, w, left=(1 - w) / 2, height=0.72, color=GOOD if i == len(stages) - 1 else ACC, alpha=0.85)
    ax.text(0.5, y, str(v), ha="center", va="center", color="white", fontweight="bold", fontsize=12)
    ax.text(1.04, y, lab, ha="left", va="center", fontsize=9.5)
ax.set_xlim(0, 2.25); ax.set_ylim(-0.6, len(stages) - 0.4); ax.axis("off")
ax.set_title("Lần gắp giá trắng nằm ngửa (02/11, 17:38)", fontsize=11)
ax = axs[1]
lab2 = ["GraspNet", "Trên vật", "Không\nva chạm", "Độ mở\n40–85 mm", "Ăn sâu\n≤ 30 mm"]
v2 = [77, 39, 24, 24, 24]
ax.bar(range(5), v2, color=ACC, alpha=0.85)
for i, v in enumerate(v2):
    ax.text(i, v + 1.5, str(v), ha="center", fontsize=10)
ax.set_xticks(range(5)); ax.set_xticklabels(lab2, fontsize=9)
ax.set_ylabel("số tư thế còn lại"); ax.set_ylim(0, 90); ax.grid(axis="y", alpha=0.3)
ax.set_title("Giá trắng đứng (01/11, 21:14): từng bộ lọc", fontsize=11)
fig.tight_layout()
save(fig, "loc_grasp_pheu.png")

# 13. Jaw zones in the TCP frame with the real object points of the chosen grasp (17:38, grasp #2)
import yaml
RUN = os.path.expanduser("~/fr3_hungros2/nhat_ky/du_lieu/2026-11-01_161721_graspnet_tay_cam/anh/173811_graspnet_white_plastic_object")
res = yaml.safe_load(open(RUN + ".yaml"))
g = res["graspnet"][1]
Tg = np.array(g["T_base"])
xg, yg, zg = Tg[:3, 0], Tg[:3, 1], Tg[:3, 2]
p_tcp = Tg[:3, 3] + (g["depth"] + 0.015 - 0.030) * xg              # formula (3.16), --deeper 15 mm
R_tcp = np.column_stack([yg, zg, xg])                               # x: closing, y: finger width, z: approach
P = (np.load(RUN + "_object_points_base.npy") - p_tcp) @ R_tcp * 1000  # object points in the TCP frame, mm
fig, axs = plt.subplots(1, 2, figsize=(13, 5.6))
for ax, (a, b, xl, yl, title) in zip(axs, ((0, 2, "x: hướng khép má (mm)", "z: hướng tiếp cận (mm)", "Nhìn ngang (mặt phẳng x–z)"),
                                        (0, 1, "x: hướng khép má (mm)", "y: bề ngang ngón (mm)", "Nhìn dọc hướng tiếp cận (mặt phẳng x–y)"))):
    if b == 2:
        sel = np.abs(P[:, 1]) < 19
        zone = Rectangle((-44, -10), 88, 40, fc=GOOD, alpha=0.15, ec=GOOD, lw=1.5)
        jl, jr = Rectangle((-82, -10), 38, 40, fc=BAD, alpha=0.12, ec=BAD), Rectangle((44, -10), 38, 40, fc=BAD, alpha=0.12, ec=BAD)
        for s in (-1, 1):   # jaws, open (88 mm), tips at z = +30 mm
            ax.add_patch(Rectangle((s * 44 - (8 if s > 0 else -8) * 0 + (0 if s > 0 else -8), -50), 8, 80, fc="#333333", alpha=0.8))
        ax.axhline(30, color="#333333", ls=":", lw=1)
        ax.text(60, 33, "đầu ngón", fontsize=9)
        ax.plot([0], [0], "k+", ms=14, mew=2); ax.text(3, -6, "TCP", fontsize=10)
        ax.set_ylim(-60, 80)
    else:
        sel = (P[:, 2] > -10) & (P[:, 2] < 30)
        zone = Rectangle((-44, -19), 88, 38, fc=GOOD, alpha=0.15, ec=GOOD, lw=1.5)
        jl, jr = Rectangle((-82, -19), 38, 38, fc=BAD, alpha=0.12, ec=BAD), Rectangle((44, -19), 38, 38, fc=BAD, alpha=0.12, ec=BAD)
        ax.set_ylim(-80, 80)
    for r in (zone, jl, jr):
        ax.add_patch(r)
    ax.scatter(P[:, a], P[:, b], s=1, c="#bbbbbb", lw=0, label="điểm của vật (mặt nạ SAM)")
    ax.scatter(P[sel, a], P[sel, b], s=2, c=ACC2, lw=0, label="điểm trong lát cắt đang xét")
    ax.set_xlim(-110, 110); ax.set_aspect("equal"); ax.grid(alpha=0.25)
    ax.set_xlabel(xl); ax.set_ylabel(yl); ax.set_title(title, fontsize=11)
mid = (np.abs(P[:, 0]) < 44) & (np.abs(P[:, 1]) < 19) & (P[:, 2] > -10) & (P[:, 2] < 30)
under = (np.abs(P[:, 0]) >= 44) & (np.abs(P[:, 0]) < 82) & (np.abs(P[:, 1]) < 19) & (P[:, 2] > -10) & (P[:, 2] < 30)
e = np.ptp(P[mid, 0]) if mid.any() else 0
axs[0].legend(loc="lower left", fontsize=8.5, markerscale=6)
fig.suptitle(f"Tư thế được chọn: {mid.sum()} điểm nằm giữa hai má (vùng xanh), bề dày phần bị kẹp e = {e:.0f} mm; "
             f"{under.sum()} điểm ở chỗ má đi xuống (vùng đỏ)", fontsize=11)
fig.tight_layout()
save(fig, "loc_grasp_vung_ma.png")
print("loc:", mid.sum(), under.sum(), round(e, 1))
