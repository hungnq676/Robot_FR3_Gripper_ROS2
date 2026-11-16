#!/usr/bin/env python3
"""Pointing test for the camera <-> robot calibration on the real FR3.

Finds the cube of the chosen colour (red by default) in the camera colour image, takes its depth (median of 20 frames
inside the blob), undoes the depth error with the camera depth model
(camera.DEPTH_MODEL_FILE; none yet -> raw depth), and maps the
point into base_footprint through the hand-eye TF. Then it plans a Pilz PTP
move that puts the finger tips straight above the cube top, tool axis down,
shows it in RViz (Planned Path) and executes ONLY after the user types y.
Measure the remaining x/y offset and the tip-to-cube gap with a ruler.
2026-11-01: --above is the finger-tip gap (it was the flange gap; the real
gripper reaches ~165 mm past the flange, so --above 0.10 would hit the table).

With --gemini "<command>" Gemini Robotics-ER boxes the object instead (the
camera image goes to Google's servers).

Usage: point_test.py [--color red|green|yellow|blue | --gemini "<command>"] [--above 0.03] [--scale 0.3] [--detect-only]
Must run in an interactive terminal (it waits on input()).
"""
import argparse
import collections
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

sys.path.insert(0, os.path.dirname(__file__))
import camera  # noqa: E402
from handeye_guided import JOINTS, Guided, tf_matrix  # noqa: E402

FLANGE_IN_WRIST3 = np.array([0.0, 0.0, 0.1])   # flange face centre, same as the table-touch check
TIP_IN_WRIST3 = FLANGE_IN_WRIST3 + [0.0, 0.0, 0.165]   # open finger tips on the axis: table touch 2026-11-01 (URDF real_gripper)
COLORS = {   # OpenCV HSV ranges
    "red": [((0, 120, 70), (8, 255, 255)), ((170, 120, 70), (180, 255, 255))],
    "green": [((40, 80, 50), (85, 255, 255))],
    "yellow": [((20, 120, 100), (35, 255, 255))],
    "blue": [((100, 120, 50), (130, 255, 255))],
}
GEMINI_MODEL = "gemini-robotics-er-2-preview"
GEMINI_PROMPT = """The image is a robot's camera view of a table. The user's command
(it may be in Vietnamese): "{command}". Find the ONE object on the table the command
refers to. Answer ONLY with JSON: {{"label": "<short English name>", "box_2d": [ymin, xmin,
ymax, xmax], "reason": "<short reason>"}} with coordinates normalised to 0-1000.
If nothing matches, or several objects match equally well, answer
{{"label": null, "box_2d": null, "reason": "<why>"}}."""
TABLE_TOP_Z = 0.012    # base_footprint; 2026-10-31 from the Astra tag samples (old table touched 30/10: 0.0087)
# Images go to the running record_session.sh session (anh/), else to a folder per day.
_SESSION = os.path.expanduser("~/.ros/fr3_session_dir")
LOG_DIR = (os.path.join(open(_SESSION).read().strip(), "anh") if os.path.exists(_SESSION)
           else os.path.expanduser(time.strftime("~/fr3_hungros2/nhat_ky/anh/%Y-%m-%d")))


class PointTest(Guided):
    def __init__(self):
        super().__init__()
        self.rgb = None
        self.info = None
        self.depths = collections.deque(maxlen=20)
        q = rclpy.qos.qos_profile_sensor_data
        self.create_subscription(Image, camera.RGB_TOPIC, lambda m: setattr(self, "rgb", m), q)
        self.create_subscription(CameraInfo, camera.INFO_TOPIC, lambda m: setattr(self, "info", m), q)
        self.create_subscription(Image, camera.DEPTH_TOPIC, self.on_depth, q)
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.depth_model = camera.load_depth_model()
        self.use3d = True
        self.last_points = None

    def correct(self, z, u=None, v=None):
        """Depth model; a single depth needs its pixel u, v (the Astra model varies over the image)."""
        return camera.correct_depth(z, self.depth_model, u, v)

    def on_depth(self, m):
        self.depths.append(np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).copy())

    def locate_3d(self, mask, depth, K, dist, T):
        """3D object position from all its pixels instead of the 2D blob centre.

        Seen at a slant a cube shows its top and one or two sides, so the 2D
        centre lies off the top face. Here every mask pixel becomes a 3D point
        (corrected depth), the table height comes from a ring of pixels just
        around the mask, points on the table are dropped, and the target is the
        centre of the top band of the remaining points. depth is the corrected
        depth (m). Returns dict or None."""
        def points(sel):
            v, u = np.nonzero(sel & np.isfinite(depth) & (depth > 0))
            rays = cv2.undistortPoints(np.column_stack((u, v)).astype(np.float64).reshape(-1, 1, 2), K, dist).reshape(-1, 2)
            cam = np.column_stack((rays * depth[v, u][:, None], depth[v, u]))
            return cam @ T[:3, :3].T + T[:3, 3]
        m = mask.astype(np.uint8)
        # Stereo depth "bleeds" a few px past object edges, so keep the ring well away.
        ring = (cv2.dilate(m, np.ones((101, 101), np.uint8)) > 0) & ~(cv2.dilate(m, np.ones((51, 51), np.uint8)) > 0)
        table = points(ring)
        if len(table) < 50:
            return None
        table_z = float(np.percentile(table[:, 2], 30))   # other objects in the ring sit above it
        obj = points(cv2.erode(m, np.ones((3, 3), np.uint8)) > 0)
        obj = obj[obj[:, 2] > table_z + 0.008]
        if len(obj) < 30:
            return None
        top_z = float(np.percentile(obj[:, 2], 95))
        band = obj[obj[:, 2] > top_z - 0.008]
        (cx, cy), (w, h), _ = cv2.minAreaRect(band[:, :2].astype(np.float32))
        return dict(center=np.array([cx, cy, top_z]), table_z=table_z, height=top_z - table_z,
                    top_size=(max(w, h), min(w, h)), n_obj=len(obj), n_top=len(band), points=obj)

    def find_cube(self, color="red"):
        """Largest blob of the colour lying on the table -> (pixel centre, raw depth,
        corrected depth, point in base, area, image). Blobs above the table (the
        robot's green status ring, for one) are skipped."""
        while self.rgb is None or self.info is None or len(self.depths) < 20:
            time.sleep(0.1)
        D = np.stack(list(self.depths)).astype(np.float32) / 1000.0
        D[D <= 0] = np.nan
        depth = np.nanmedian(D, 0)
        m = self.rgb
        bgr = camera.to_bgr(m)
        K = np.array(self.info.k).reshape(3, 3)
        dist = np.array(self.info.d)
        T = tf_matrix(self.tf.lookup_transform("base_footprint", camera.OPTICAL_FRAME, Time()))
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in COLORS[color]:
            mask |= cv2.inRange(hsv, lo, hi)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
        best = None
        for k in range(1, n):
            area = int(stats[k, cv2.CC_STAT_AREA])
            if area < 150 or (best is not None and area <= best[4]):
                continue
            inner = cv2.erode((lab == k).astype(np.uint8), np.ones((7, 7), np.uint8)) > 0
            z_raw = float(np.nanmedian(depth[inner if inner.any() else lab == k]))
            if not np.isfinite(z_raw):
                continue
            uv = cent[k]
            z = self.correct(z_raw, uv[0], uv[1])
            ray = np.r_[cv2.undistortPoints(uv.reshape(1, 1, 2), K, dist).ravel(), 1.0]
            p = T[:3, :3] @ (ray * z) + T[:3, 3]
            if p[2] > 0.12 or np.hypot(p[0], p[1]) > 0.8:
                continue
            best = (uv, z_raw, z, p, area, stats[k, :4], lab == k)
        if best is None:
            return None
        uv, z_raw, z, p, area, (x, y, w, h), blob = best
        p = self.use_3d(p, blob, depth, K, dist, T, bgr)
        cv2.rectangle(bgr, (x, y), (x + w, y + h), (255, 0, 255), 2)
        cv2.drawMarker(bgr, tuple(uv.astype(int)), (255, 0, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(bgr, f"x={p[0]*1000:.0f} y={p[1]*1000:.0f} z={p[2]*1000:.0f} mm", (x, max(20, y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
        return uv, z_raw, z, p, area, bgr

    def use_3d(self, p2d, mask, depth_raw, K, dist, T, bgr):
        """Report the 2D-centre point and the 3D estimate; return the one to use."""
        print(f"  tam 2D (khung/blob + depth): x={p2d[0]*1000:.1f} y={p2d[1]*1000:.1f} z={p2d[2]*1000:.1f} mm")
        if not self.use3d:
            return p2d
        r = self.locate_3d(mask, self.correct(depth_raw), K, dist, T)
        if r is None:
            print("  uoc luong 3D that bai (it diem) -> dung tam 2D")
            return p2d
        c = r["center"]
        d = (c[:2] - p2d[:2]) * 1000
        print(f"  tam 3D mat tren   : x={c[0]*1000:.1f} y={c[1]*1000:.1f} z={c[2]*1000:.1f} mm "
              f"(ban z={r['table_z']*1000:.1f}, vat cao {r['height']*1000:.0f} mm, mat tren "
              f"{r['top_size'][0]*1000:.0f}x{r['top_size'][1]*1000:.0f} mm, {r['n_obj']} diem)")
        print(f"  3D - 2D: dx={d[0]:.1f} dy={d[1]:.1f} mm")
        cam = np.linalg.inv(T) @ np.r_[c, 1.0]
        uv = cv2.projectPoints(cam[:3].reshape(1, 3), np.zeros(3), np.zeros(3), K, dist)[0].ravel()
        cv2.drawMarker(bgr, tuple(uv.astype(int)), (0, 255, 0), cv2.MARKER_TILTED_CROSS, 22, 2)
        self.last_points = r["points"]
        return c

    def snapshot(self):
        """Median depth (bias corrected), BGR image, K, D, T_base_optical."""
        while self.rgb is None or self.info is None or len(self.depths) < 20:
            time.sleep(0.1)
        D = np.stack(list(self.depths)).astype(np.float32) / 1000.0
        D[D <= 0] = np.nan
        depth = np.nanmedian(D, 0)
        self.raw_depth = depth
        depth = self.correct(depth)
        m = self.rgb
        bgr = camera.to_bgr(m)
        T = tf_matrix(self.tf.lookup_transform("base_footprint", camera.OPTICAL_FRAME, Time()))
        return depth, bgr, np.array(self.info.k).reshape(3, 3), np.array(self.info.d), T

    def find_gemini(self, command):
        """Gemini boxes the commanded object; 3D from the corrected depth inside the box."""
        import io
        import json
        import re
        from google import genai
        from google.genai import types
        from PIL import Image as PILImage
        depth, bgr, K, dist, T = self.snapshot()
        key = os.environ.get("GEMINI_API_KEY", "").strip()
        path = os.path.expanduser("~/.config/fr3_demo/gemini_api_key")
        if not key and os.path.exists(path):
            key = open(path).read().strip()
        buf = io.BytesIO()
        PILImage.fromarray(bgr[:, :, ::-1]).save(buf, format="PNG")
        t0 = time.time()
        client = genai.Client(api_key=key)   # keep a reference: a temporary closes its HTTP client
        reply = client.models.generate_content(
            model=GEMINI_MODEL,
            contents=[types.Part.from_bytes(data=buf.getvalue(), mime_type="image/png"),
                      GEMINI_PROMPT.format(command=command)],
            config=types.GenerateContentConfig(http_options=types.HttpOptions(timeout=60000)))
        text = reply.text or ""
        print(f"Gemini ({time.time() - t0:.1f} s): {text.strip()}")
        match = re.search(r"\{.*\}", text, re.S)
        answer = json.loads(match.group(0)) if match else {}
        if not answer.get("box_2d"):
            return None
        h, w = bgr.shape[:2]
        y0, x0, y1, x1 = answer["box_2d"]
        x0, x1 = int(x0 * w / 1000), int(x1 * w / 1000)
        y0, y1 = int(y0 * h / 1000), int(y1 * h / 1000)
        # Depth from the middle half of the box (edges mix in the table behind).
        cx, cy, hw, hh = (x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 4, (y1 - y0) / 4
        z = float(np.nanmedian(depth[int(cy - hh):int(cy + hh) + 1, int(cx - hw):int(cx + hw) + 1]))
        uv = np.array([cx, cy])
        ray = np.r_[cv2.undistortPoints(uv.reshape(1, 1, 2), K, dist).ravel(), 1.0]
        p = T[:3, :3] @ (ray * z) + T[:3, 3]
        box = np.zeros(depth.shape, bool)
        box[y0:y1 + 1, x0:x1 + 1] = True   # table pixels inside the box drop out by height
        p = self.use_3d(p, box, self.raw_depth, K, dist, T, bgr)
        # Cross-check without depth: the same ray cut with the touched table plane.
        r = T[:3, :3] @ ray
        p_table = T[:3, 3] + r * (TABLE_TOP_Z - T[2, 3]) / r[2]
        if not np.all(np.isfinite(p)):
            # No depth on the object at all (2026-11-01: the glossy teal charger reads 0):
            # aim at the ray cut with the table; the height is unknown, --above must allow for it.
            print("  KHONG co depth tren vat -> dung tia cat mat ban, z = mat ban (chieu cao vat chua biet)")
            p = p_table
        print(f"  khung ({x0},{y0})-({x1},{y1}); depth -> x={p[0]*1000:.0f} y={p[1]*1000:.0f} z={p[2]*1000:.0f} mm; "
              f"tia cat mat ban -> x={p_table[0]*1000:.0f} y={p_table[1]*1000:.0f} mm")
        cv2.rectangle(bgr, (x0, y0), (x1, y1), (255, 0, 255), 2)
        cv2.putText(bgr, f"{answer.get('label')}: x={p[0]*1000:.0f} y={p[1]*1000:.0f} z={p[2]*1000:.0f} mm",
                    (x0, max(20, y0 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
        return uv, float(np.nanmedian(self.depths[-1][int(cy - hh):int(cy + hh) + 1,
                                                     int(cx - hw):int(cx + hw) + 1])) / 1000.0, z, p, (x1 - x0) * (y1 - y0), bgr

    def solve_ik(self, tip_target):
        """Tool axis straight down; try yaw every 15 deg, keep the solution closest to now."""
        best = None
        down = Rotation.from_euler("xyz", [np.pi, 0, 0])
        for yaw in np.radians(np.arange(-180, 180, 15)):
            R = (Rotation.from_euler("z", yaw) * down).as_matrix()
            origin = tip_target - R @ TIP_IN_WRIST3
            req = GetPositionIK.Request()
            req.ik_request.group_name = "arm"
            req.ik_request.ik_link_name = "wrist3_link"
            req.ik_request.avoid_collisions = True
            req.ik_request.robot_state.joint_state.name = JOINTS
            req.ik_request.robot_state.joint_state.position = [float(v) for v in self.q]
            ps = PoseStamped()
            ps.header.frame_id = "base_footprint"
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, origin)
            qx, qy, qz, qw = Rotation.from_matrix(R).as_quat()
            ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = qx, qy, qz, qw
            req.ik_request.pose_stamped = ps
            req.ik_request.timeout.nanosec = 200_000_000
            res = self.wait(self.ik.call_async(req), 3.0)
            if res is None or res.error_code.val != 1:
                continue
            sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
            qs = np.array([sol[j] for j in JOINTS])
            cost = np.abs(qs - self.q).max()
            if best is None or cost < best[0]:
                best = (cost, qs)
        return None if best is None else best[1]


def main():
    rclpy.init()
    try:
        run()
    finally:
        rclpy.try_shutdown()


def run():
    ap = argparse.ArgumentParser()
    ap.add_argument("--depth-k", type=float, default=None,
                    help="override the depth model: disparity bias k (1/m); 0 with --depth-scale 1 = raw")
    ap.add_argument("--depth-scale", type=float, default=None, help="override: z_true = z / scale")
    ap.add_argument("--above", type=float, default=0.03, help="finger-tip gap above the cube top (m)")
    ap.add_argument("--scale", type=float, default=0.3)
    ap.add_argument("--color", default="red", choices=sorted(COLORS))
    ap.add_argument("--gemini", metavar="COMMAND", help="let Gemini pick the object, e.g. 'cai coc giay'")
    ap.add_argument("--center", choices=["3d", "2d"], default="3d",
                    help="3d: centre of the top face from the object's 3D points (default); 2d: blob/box centre")
    ap.add_argument("--detect-only", action="store_true")
    a = ap.parse_args()

    node = PointTest()
    if a.depth_k is not None or a.depth_scale is not None:
        node.depth_model = {"k_per_m": a.depth_k or 0.0, "scale": a.depth_scale or 1.0}
    node.use3d = a.center == "3d"
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    t0 = time.time()
    while node.q is None and time.time() - t0 < 10:
        time.sleep(0.1)
    time.sleep(2.0)   # TF buffer

    found = node.find_gemini(a.gemini) if a.gemini else node.find_cube(a.color)
    if found is None:
        print("Gemini KHONG chon duoc vat nao." if a.gemini else f"KHONG thay vat mau {a.color} tren ban.")
        return
    uv, z_raw, z, p, area, img = found
    os.makedirs(LOG_DIR, exist_ok=True)
    shot = os.path.join(LOG_DIR, time.strftime("%H%M") + ("_chi_diem_gemini.jpg" if a.gemini else f"_chi_diem_{a.color}.jpg"))
    cv2.imwrite(shot, img)
    if node.last_points is not None:
        np.save(shot.replace(".jpg", "_points.npy"), node.last_points)
    print(f"Vat {'Gemini' if a.gemini else a.color}: {area} px tai anh ({uv[0]:.0f},{uv[1]:.0f}); depth do {z_raw*1000:.0f} mm -> hieu chinh {z*1000:.0f} mm")
    print(f"=> mat tren khoi trong toa do robot: x={p[0]*1000:.1f} y={p[1]*1000:.1f} z={p[2]*1000:.1f} mm")
    print(f"Anh: {shot}")
    if a.detect_only:
        return

    node.move.wait_for_server(timeout_sec=10)
    node.exe.wait_for_server(timeout_sec=10)
    node.ik.wait_for_service(timeout_sec=10)
    target = p + np.array([0.0, 0.0, a.above])
    q_goal = node.solve_ik(target)
    if q_goal is None:
        print("KHONG co nghiem IK (va cham voi ban / ngoai tam). Thu --above lon hon. Khong chay.")
        return
    print(f"Dich dau ngon: x={target[0]*1000:.1f} y={target[1]*1000:.1f} z={target[2]*1000:.1f} mm "
          f"(cach mat tren khoi {a.above*1000:.0f} mm)")
    print(f"Khop dich (deg): {np.degrees(q_goal).round(1).tolist()}")
    print(f"Thay doi (deg): {np.degrees(q_goal - node.q).round(1).tolist()}")
    res = node.plan(q_goal, a.scale)
    if res is None:
        print("Pilz PTP KHONG lap duoc ke hoach. Khong chay.")
        return
    pts = res.planned_trajectory.joint_trajectory.points
    dur = pts[-1].time_from_start.sec + pts[-1].time_from_start.nanosec * 1e-9
    for _ in range(3):
        node.show(res)
        time.sleep(0.3)
    print(f"Ke hoach OK: {dur:.1f} s, toc do {a.scale}. Xem duong di trong RViz (Planned Path).")
    if input("Go y + Enter de CHAY, phim khac de huy: ").strip().lower() != "y":
        print("Huy. Robot khong chay.")
        return
    ok = node.execute(res.planned_trajectory)
    print("Da toi. Do lech x/y va khe mat bich - mat khoi bang thuoc." if ok else "CHAY THAT BAI (xem log driver).")


if __name__ == "__main__":
    main()
