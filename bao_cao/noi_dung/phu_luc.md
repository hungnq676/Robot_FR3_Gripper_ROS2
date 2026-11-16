# PHỤ LỤC

## Phụ lục A. Hướng dẫn vận hành

Các bước dưới đây chạy trên máy ROG, trong thư mục `~/fr3_hungros2`. Mỗi lệnh chạy trong một cửa sổ terminal riêng.

1. Bật bộ điều khiển robot, xoá lỗi trên teach pendant nếu có, kiểm tra kết nối tới 192.168.58.2.
2. Khởi động camera: `./run_astra.sh`.
3. Khởi động robot, MoveIt và RViz: `./run_real.sh`.
4. Kiểm tra camera có bị xê dịch không: `tools/run_tool.sh cam_check.py`. Nếu lệch trên 12 mm, hiệu chuẩn lại theo bước 5.
5. Hiệu chuẩn (khi cần): chạy `tools/run_tool.sh tag_check.py` ở một cửa sổ, `tools/run_tool.sh handeye_guided.py --margin 40 --angle 20 40 --min-dist 0.33 --poses 20` ở cửa sổ khác, gõ `y` cho từng tư thế; sau đó `tools/run_tool.sh handeye_solve.py` và `tools/run_tool.sh depth_plane_fit.py --table-z 0.012`.
6. Bật ESP32 của gripper, kiểm tra: `tools/gripper.py status` (nhiệt độ phải dưới 60 °C).
7. Đặt vật trong vùng đánh dấu (Hình 5.24), cách vật khác và thành hộp ít nhất 15 cm.
8. Chạy kịch bản: `tools/run_pick_to_box.sh "<tên vật>" "<tên hộp>" <nhãn> --deeper 0.015 --lift 0.20 --scale 0.3`.
9. Xem toàn bộ đường đi trong RViz (MotionPlanning → Planned Path), rồi gõ `y` để chạy hoặc `n` để huỷ.
10. Kết quả nằm trong `nhat_ky/du_lieu/<ngày_giờ>_hop_<nhãn>/`.

TABLE Bảng A.1. Xử lý một số lỗi thường gặp
| Hiện tượng | Nguyên nhân thường gặp | Cách xử lý |
| --- | --- | --- |
| Driver báo ServoJ lỗi 14 liên tục | Bộ điều khiển đang có lỗi (ví dụ khớp vượt giới hạn mềm) | Xoá lỗi trên teach pendant, đưa khớp về trong giới hạn |
| Driver báo ServoJ lỗi 99 | Robot dừng do dừng khẩn cấp hoặc phát hiện va chạm | Nhả dừng khẩn cấp, xoá lỗi, khởi động lại `run_real.sh` |
| Driver báo ServoJ lỗi −2 | Mất kết nối SDK sau dừng khẩn cấp hoặc cáp mạng chập chờn | Khởi động lại `run_real.sh` |
| Gripper không phản hồi | ESP32 mất Wi-Fi hoặc sụt nguồn | Rút cắm lại nguồn ESP32, kiểm tra bằng `ping` |
| Gripper báo nhiệt độ từ 60 °C | Giữ vật quá lâu | Nhả lực (`tools/gripper.py free`), chờ nguội |
| Không lập được kế hoạch | Vật sát giới hạn j1, quá xa, hoặc quá gần vật khác | Dời vật vào vùng đánh dấu |
| Gemini báo 429 | Hết lượt gọi miễn phí trong ngày | Hệ thống tự chuyển sang mô hình dự phòng |

## Phụ lục B. Trích mã nguồn chính

### B.1 Chuyển tư thế gắp GraspNet sang TCP và giới hạn độ ăn sâu (mtc_pregrasp.py)

```
def tcp_poses(g, deeper=0.0):
    T = np.array(g["T_base"])
    x, y, z = T[:3, 0], T[:3, 1], T[:3, 2]
    out = []
    for turn in (0.0, np.pi):
        R = np.column_stack([y, z, x]) @ Rotation.from_euler("z", turn).as_matrix()
        push = g.get("depth", 0.02) + deeper
        if x[2] < -1e-3:
            low = (R @ FINGER_CORNERS.T)[2].min()   # lowest finger corner below the TCP
            limit = (T[2, 3] - TCP_FROM_TIPS * x[2] + low - (TABLE_TOP_Z + CORNER_ABOVE_TABLE)) / -x[2]
            push = min(push, limit)
        P = np.eye(4)
        P[:3, :3] = R
        P[:3, 3] = T[:3, 3] + (push - TCP_FROM_TIPS) * x
        out.append(P)
    return out
```

### B.2 Đo bề dày phần vật nằm giữa hai má (mtc_pregrasp.py)

```
def width_between_jaws(res, P):
    path = res.get("object_points")
    if not path or not os.path.exists(path):
        return None
    q = (np.load(path).astype(float) - P[:3, 3]) @ P[:3, :3]     # x closing, y finger width, z approach
    slab = (np.abs(q[:, 1]) < JAW_HALF_Y) & (q[:, 2] > TCP_FROM_TIPS - 0.040) & (q[:, 2] < TCP_FROM_TIPS)
    inside = slab & (np.abs(q[:, 0]) < JAW_IN_X)
    hit = slab & (np.abs(q[:, 0]) >= JAW_IN_X) & (np.abs(q[:, 0]) < 0.082)
    if inside.sum() < 5:
        return 0.0, 0.0, int(inside.sum()), int(hit.sum())
    x = q[inside, 0]
    return 2000 * np.abs(x).max(), 1000 * (x.max() - x.min()), int(inside.sum()), int(hit.sum())
```

### B.3 Độ lệch khớp đế lớn nhất của một lời giải (mtc_pregrasp.py)

```
def joint_swing(solution, joint="j1"):
    vals = []
    for t in trajectories(solution):
        jt = t.joint_trajectory
        if joint in jt.joint_names:
            k = jt.joint_names.index(joint)
            vals += [p.positions[k] for p in jt.points]
    return float(np.degrees(np.max(np.abs(np.array(vals) - vals[0])))) if vals else 0.0
```

### B.4 Kẹp theo góc tính từ camera, cộng thêm 5° (gripper.py)

```
def grip_measured(self, object_mm, extra_deg=5.0, ms=1500):
    units = extra_deg * 1000 / 240
    sign = 1 if CLOSE_POS > OPEN_POS else -1
    touch = pos_for_width(object_mm)
    target = int(max(0, min(1000, touch + sign * units)))
    p = self.move(target, ms)
    if p is not None and abs(p - target) > 3:          # stalled before the target: pressing on the object
        return p, True, touch
    p = self.move(CLOSE_POS, ms)                       # not touching yet: close until it stalls
    if p is None or abs(p - CLOSE_POS) <= 3:
        return p, False, touch
    if abs(p - CLOSE_POS) > 15:
        self._packet(1, struct.pack("<HH", int(max(0, min(1000, p + sign * units))), 200))
    return p, True, touch
```

### B.5 Hộp bao 3D của vật (grasp_once.py)

```
def object_box(ob, table_z):
    r = np.linalg.norm(ob[:, :2] - np.median(ob[:, :2], 0), axis=1)
    above = ob[r <= np.percentile(r, 95)]
    (cx, cy), (w, h), ang = cv2.minAreaRect(above[:, :2].astype(np.float32))
    yaw = np.radians(ang)
    if w < h:   # x along the longer side
        w, h, yaw = h, w, yaw + np.pi / 2
    top = float(np.percentile(above[:, 2], 95))
    return dict(center=np.array([cx, cy, (top + table_z) / 2]), size=np.array([w, h, top - table_z]),
                yaw=float(yaw), top=top, table=float(table_z))
```
