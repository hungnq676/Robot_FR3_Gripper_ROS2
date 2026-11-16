#!/usr/bin/env python3
"""One-shot grasp detection on the real camera (Astra Pro Plus). Moves nothing.

  RGB + median of N depth frames (corrected with camera.DEPTH_MODEL_FILE, if any)
  -> target object: Gemini box for a spoken command (--gemini) or the largest
     blob of a colour (--color), refined to a pixel mask by SAM (MobileSAM)
  -> GraspNet (RealSense checkpoint) on the scene points around the object
  -> drop grasps that collide with the scene, are wider than the gripper or do
     not sit on the object
  -> grasp poses in base_footprint (hand-eye TF), drawn on the image and saved.
     No hand-eye TF yet: poses relative to the table plane (RANSAC), no RViz.

Run in the graspnet conda env with ROS sourced: tools/run_grasp.sh [args]
Usage: grasp_once.py (--color red | --gemini "<command>") [--top 5] [--max-width 0.085] [--min-width 0.04] [--max-depth 0.03]
Real gripper (2026-11-01): jaws open 88 mm inside (closed 2.9 mm), fingers 60 mm
long; only the outer 30 mm should take the object, so GraspNet depths (finger
tips past the grasp centre) are capped at --max-depth.
The servo runs out of travel with the jaws still ~30 mm apart (not re-meshed),
so only objects wider than that can be held: grasps narrower than --min-width
are dropped.
"""
import argparse
import os
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
import tf2_ros
import torch
import yaml
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

import camera

GN_DIR = os.path.expanduser("~/graspnet_runtime/GraspNet_Pointnet2_PyTorch1.13.1")
sys.path.insert(0, GN_DIR)
from graspnetAPI import GraspGroup  # noqa: E402
from models.graspnet import GraspNet, pred_decode  # noqa: E402
from utils.collision_detector import ModelFreeCollisionDetector  # noqa: E402

CHECKPOINT = os.path.join(GN_DIR, "logs/log_rs/checkpoint-rs.tar")
SAM_WEIGHTS = os.path.expanduser("~/graspnet_runtime/mobile_sam.pt")
# Images go to the running record_session.sh session (anh/), else to a folder per day.
_SESSION = os.path.expanduser("~/.ros/fr3_session_dir")
LAST_RESULT = os.path.expanduser("~/.ros/fr3_last_grasp.yaml")
LOG_DIR = (os.path.join(open(_SESSION).read().strip(), "anh") if os.path.exists(_SESSION)
           else os.path.expanduser(time.strftime("~/fr3_hungros2/nhat_ky/anh/%Y-%m-%d")))
NUM_POINT = 20000
COLORS = {   # OpenCV HSV ranges
    "red": [((0, 120, 70), (8, 255, 255)), ((170, 120, 70), (180, 255, 255))],
    "green": [((40, 80, 50), (85, 255, 255))],
    "yellow": [((20, 120, 100), (35, 255, 255))],
    "blue": [((100, 120, 50), (130, 255, 255))],
}
GEMINI_MODELS = ["gemini-robotics-er-2-preview", "gemini-flash-latest", "gemini-3.5-flash", "gemini-3-flash-preview"]
GEMINI_PROMPT = """The image is a robot's camera view of a table. The user's command
(it may be in Vietnamese): "{command}". Find the ONE object on the table the command
refers to. Answer ONLY with JSON: {{"label": "<short English name>", "box_2d": [ymin, xmin,
ymax, xmax], "reason": "<short reason>"}} with coordinates normalised to 0-1000.
If nothing matches, answer {{"label": null, "box_2d": null, "reason": "<why>"}}."""


def tf_matrix(t):
    q, p = t.transform.rotation, t.transform.translation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def capture(frames, tf_wait=3.0):
    """Colour, median of `frames` depth frames, K, D and base -> optical
    (None when the hand-eye TF is not there within tf_wait s of the images)."""
    rclpy.init()
    node = rclpy.create_node("grasp_once")
    got = {"depth": []}
    q = rclpy.qos.qos_profile_sensor_data
    node.create_subscription(Image, camera.RGB_TOPIC, lambda m: got.__setitem__("rgb", m), q)
    node.create_subscription(CameraInfo, camera.INFO_TOPIC, lambda m: got.__setitem__("info", m), q)
    node.create_subscription(Image, camera.DEPTH_TOPIC, lambda m: got["depth"].append(
        camera.depth_meters(m)), q)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    t0, t_ready = time.time(), None
    while time.time() - t0 < 20:
        rclpy.spin_once(node, timeout_sec=0.05)
        if len(got["depth"]) >= frames and "rgb" in got and "info" in got:
            t_ready = t_ready or time.time()
            if buf.can_transform("base_footprint", camera.OPTICAL_FRAME, Time()) or time.time() - t_ready > tf_wait:
                break
    else:
        raise RuntimeError("thieu anh / depth / camera_info (camera da chay chua? run_astra.sh)")
    T = None
    if buf.can_transform("base_footprint", camera.OPTICAL_FRAME, Time()):
        T = tf_matrix(buf.lookup_transform("base_footprint", camera.OPTICAL_FRAME, Time()))
    bgr = camera.to_bgr(got["rgb"])
    D = np.stack(got["depth"][-frames:]).astype(np.float32)
    D[D <= 0] = np.nan
    depth = np.nan_to_num(np.nanmedian(D, 0))
    info = got["info"]
    node.destroy_node()
    rclpy.try_shutdown()
    return bgr, depth, np.array(info.k).reshape(3, 3), np.array(info.d), T


def table_frame(depth, K, iters=300, thresh=0.01):
    """No hand-eye TF yet: RANSAC the largest plane (the table) and return the
    camera -> table transform, z up out of the table, so the table filters and
    the geometric grasp work. Coordinates are then table-relative, not robot."""
    v, u = np.mgrid[0:depth.shape[0]:4, 0:depth.shape[1]:4]
    z = depth[v, u]
    ok = (z > 0.3) & (z < 1.5)
    P = np.stack([(u[ok] - K[0, 2]) * z[ok] / K[0, 0], (v[ok] - K[1, 2]) * z[ok] / K[1, 1], z[ok]], 1)
    rng = np.random.default_rng(0)
    best, best_n = None, 0
    for _ in range(iters):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        cnt = np.count_nonzero(np.abs((P - a) @ n) < thresh)
        if cnt > best_n:
            best, best_n = (n, a), cnt
    n, a = best
    if n @ a > 0:                                  # normal towards the camera = up
        n = -n
    origin = (a @ n) * n                           # foot of the camera on the plane
    x = np.array([1.0, 0, 0]) - n[0] * n
    x /= np.linalg.norm(x)
    R = np.stack([x, np.cross(n, x), n])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, -R @ origin
    print(f"Mat ban (RANSAC): {100 * best_n / len(P):.0f}% diem, camera cao {-(origin @ n) * 1000:.0f} mm, "
          f"nghieng {np.degrees(np.arccos(abs(n[2]))):.0f} deg")
    return T


def box_from_color(bgr, color):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in COLORS[color]:
        mask |= cv2.inRange(hsv, lo, hi)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask)
    if n < 2:
        return None, None
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    x, y, w, h = stats[k, :4]
    return [int(x), int(y), int(x + w), int(y + h)], color


def box_from_gemini(bgr, command):
    import io
    import json
    import re
    from google import genai
    from google.genai import types
    from PIL import Image as PILImage
    key = os.environ.get("GEMINI_API_KEY", "").strip()
    path = os.path.expanduser("~/.config/fr3_demo/gemini_api_key")
    if not key and os.path.exists(path):
        key = open(path).read().strip()
    buf = io.BytesIO()
    PILImage.fromarray(bgr[:, :, ::-1]).save(buf, format="PNG")
    client = genai.Client(api_key=key)
    # 2026-11-02: the free tier allows 20 requests/day for the robotics model; on 429 fall back
    # to general models with their own quota (same box_2d 0-1000 convention).
    models = [m for m in os.environ.get("GEMINI_MODELS", "").split(",") if m] or GEMINI_MODELS
    for model in models:
        t0 = time.time()
        try:
            reply = client.models.generate_content(
                model=model,
                contents=[types.Part.from_bytes(data=buf.getvalue(), mime_type="image/png"),
                          GEMINI_PROMPT.format(command=command)],
                config=types.GenerateContentConfig(http_options=types.HttpOptions(timeout=60000)))
            break
        except Exception as e:   # noqa: BLE001 - quota / unknown model: try the next one
            print(f"Gemini {model}: {str(e)[:90]} -> thu model khac")
    else:
        raise RuntimeError("het quota moi model Gemini")
    text = reply.text or ""
    print(f"Gemini {model} ({time.time() - t0:.1f} s): {text.strip()}")
    match = re.search(r"\{.*\}", text, re.S)
    answer = json.loads(match.group(0)) if match else {}
    if not answer.get("box_2d"):
        return None, None
    h, w = bgr.shape[:2]
    y0, x0, y1, x1 = answer["box_2d"]
    return [int(x0 * w / 1000), int(y0 * h / 1000), int(x1 * w / 1000), int(y1 * h / 1000)], answer.get("label")


def sam_mask(bgr, box):
    from ultralytics import SAM
    model = SAM(SAM_WEIGHTS if os.path.exists(SAM_WEIGHTS) else "mobile_sam.pt")
    t0 = time.time()
    res = model(bgr, bboxes=[box], verbose=False)
    print(f"SAM: {time.time() - t0:.1f} s")
    if not res or res[0].masks is None:
        return None
    m = res[0].masks.data[0].cpu().numpy() > 0.5
    if m.shape != bgr.shape[:2]:
        m = cv2.resize(m.astype(np.uint8), (bgr.shape[1], bgr.shape[0]), interpolation=cv2.INTER_NEAREST) > 0
    return m


def camera_points(depth, K, sel, pixels=False):
    v, u = np.nonzero(sel & (depth > 0.2) & (depth < 1.2))
    z = depth[v, u]
    pts = np.column_stack(((u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z)).astype(np.float32)
    return (pts, v, u) if pixels else pts


def object_box(ob, table_z):
    """Upright box around the object's points (base frame): footprint = min-area
    rectangle of all SAM-mask points minus the 5 % farthest from their centre
    (stray depth); height = top (95th pct) - table. Only keeping points well
    above the table shrank a 30 mm cube to 36 x 15 mm, because stereo depth
    flattens small objects; all points gave 33 x 30 mm (2026-10-30)."""
    r = np.linalg.norm(ob[:, :2] - np.median(ob[:, :2], 0), axis=1)
    above = ob[r <= np.percentile(r, 95)]
    (cx, cy), (w, h), ang = cv2.minAreaRect(above[:, :2].astype(np.float32))
    yaw = np.radians(ang)
    if w < h:   # x along the longer side
        w, h, yaw = h, w, yaw + np.pi / 2
    top = float(np.percentile(above[:, 2], 95))
    return dict(center=np.array([cx, cy, (top + table_z) / 2]), size=np.array([w, h, top - table_z]),
                yaw=float(yaw), top=top, table=float(table_z))


def geometric_grasp(box, max_width, min_width=0.0, margin=0.01, finger_depth=0.02):
    """Top-down two-finger grasp across the box's short side, in GraspNet's
    convention (x = approach, y = closing direction). Returns (T_base, width, ok, why)."""
    short = np.array([-np.sin(box["yaw"]), np.cos(box["yaw"]), 0.0])
    x = np.array([0.0, 0.0, -1.0])
    z = np.cross(x, short)
    Tg = np.eye(4)
    Tg[:3, :3] = np.column_stack((x, short, z))
    Tg[:3, 3] = [box["center"][0], box["center"][1], box["top"] - min(finger_depth, box["size"][2] / 2)]
    width = box["size"][1] + margin
    ok = min_width <= width <= max_width
    why = ("" if ok else f"canh ngan {box['size'][1] * 1000:.0f} mm + {margin * 1000:.0f} > do mo {max_width * 1000:.0f} mm"
           if width > max_width else f"canh ngan {box['size'][1] * 1000:.0f} mm: kep khong khep toi (con {min_width * 1000:.0f} mm)")
    return Tg, width, ok, why


def load_network(device):
    net = GraspNet(input_feature_dim=0, num_view=300, num_angle=12, num_depth=4, cylinder_radius=0.05,
                   hmin=-0.02, hmax_list=[0.01, 0.02, 0.03, 0.04], is_training=False)
    net.to(device)
    net.load_state_dict(torch.load(CHECKPOINT, map_location=device)["model_state_dict"])
    net.eval()
    return net


def draw_grasp(img, g, K, color, label=None):
    draw_pose(img, g.rotation_matrix, g.translation, g.width, K, color, label)


def draw_pose(img, R, c, w, K, color, label=None, thick=2):
    """Gripper as seen from the camera (camera frame pose): two fingers
    (+/- y * width/2) and the approach (x)."""
    pts = {
        "c": c, "l": c + R[:, 1] * w / 2, "r": c - R[:, 1] * w / 2,
        "lb": c + R[:, 1] * w / 2 - R[:, 0] * 0.02, "rb": c - R[:, 1] * w / 2 - R[:, 0] * 0.02,
        "a": c - R[:, 0] * 0.06,
    }
    uv = {k: tuple(np.round(K @ p / p[2])[:2].astype(int)) for k, p in pts.items()}
    cv2.line(img, uv["l"], uv["lb"], color, thick)
    cv2.line(img, uv["r"], uv["rb"], color, thick)
    cv2.line(img, uv["lb"], uv["rb"], color, thick)
    mid = tuple(int(v) for v in (np.array(uv["lb"]) + np.array(uv["rb"])) // 2)
    cv2.line(img, mid, uv["a"], color, thick)
    if label:
        cv2.putText(img, label, (uv["c"][0] + 6, uv["c"][1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


def gripper_lines(T, width, depth=0.04, stem=0.06):
    """Line segments (base frame) of a parallel gripper at grasp pose T (x approach, y closing)."""
    R, c = T[:3, :3], T[:3, 3]
    l, r = c + R[:, 1] * width / 2, c - R[:, 1] * width / 2
    lb, rb = l - R[:, 0] * depth, r - R[:, 0] * depth
    mid = (lb + rb) / 2
    return [l, lb, r, rb, lb, rb, mid, mid - R[:, 0] * stem]


def publish_view(obj_base, rgb, box, geo, gn, hold):
    """Object cloud, its box and the grasps on /fr3_demo/object_cloud and
    /fr3_demo/grasp_markers (frame base_footprint) for tools/grasp_view.rviz."""
    from geometry_msgs.msg import Point
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import PointField
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import ColorRGBA, Header
    from visualization_msgs.msg import Marker, MarkerArray
    rclpy.init()
    node = rclpy.create_node("grasp_view")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    cloud_pub = node.create_publisher(point_cloud2.PointCloud2, "/fr3_demo/object_cloud", qos)
    mark_pub = node.create_publisher(MarkerArray, "/fr3_demo/grasp_markers", qos)
    header = Header(frame_id="base_footprint", stamp=node.get_clock().now().to_msg())
    packed = (rgb[:, 0].astype(np.uint32) << 16 | rgb[:, 1].astype(np.uint32) << 8 | rgb[:, 2].astype(np.uint32))
    fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate("xyz")]
    fields.append(PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1))
    cloud = point_cloud2.create_cloud(header, fields, [(*p, c) for p, c in zip(
        obj_base.astype(np.float32).tolist(), packed.view(np.float32).tolist())])
    markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])

    def marker(mid, kind, color, scale):
        m = Marker(header=header, ns="grasp_view", id=mid, type=kind, action=Marker.ADD)
        m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        m.scale.x, m.scale.y, m.scale.z = scale
        m.pose.orientation.w = 1.0
        return m
    b = marker(1, Marker.CUBE, (0.0, 0.8, 1.0, 0.25), box["size"])
    b.pose.position.x, b.pose.position.y, b.pose.position.z = map(float, box["center"])
    qz = Rotation.from_euler("z", box["yaw"]).as_quat()
    b.pose.orientation.x, b.pose.orientation.y, b.pose.orientation.z, b.pose.orientation.w = map(float, qz)
    markers.markers.append(b)
    grippers = [(geo[0], geo[1], (0.0, 1.0, 0.0, 1.0) if geo[2] else (1.0, 0.0, 0.0, 1.0), "hinh hoc")] if geo else []
    grippers += [(np.array(g["T_base"]), g["width"], (1.0, 0.6, 0.0, 1.0) if i == 0 else (1.0, 0.85, 0.3, 0.6),
                  f"GraspNet #{i + 1}" if i == 0 else None) for i, g in enumerate(gn)]
    for i, (Tg, w, col, text) in enumerate(grippers):
        m = marker(10 + i, Marker.LINE_LIST, col, (0.004, 0.0, 0.0))
        m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in gripper_lines(Tg, w)]
        markers.markers.append(m)
        if text:
            t = marker(100 + i, Marker.TEXT_VIEW_FACING, col, (0.0, 0.0, 0.02))
            t.text = text
            t.pose.position.x, t.pose.position.y, t.pose.position.z = map(float, Tg[:3, 3] - Tg[:3, 0] * 0.09)
            markers.markers.append(t)
    cloud_pub.publish(cloud)
    mark_pub.publish(markers)
    print("RViz: /fr3_demo/object_cloud + /fr3_demo/grasp_markers (xem bang tools/run_grasp_view.sh)")
    try:
        t0 = time.time()
        while rclpy.ok() and (hold or time.time() - t0 < 2.0):
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


def main():
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--color", choices=sorted(COLORS))
    src.add_argument("--gemini", metavar="COMMAND")
    ap.add_argument("--top", type=int, default=5)
    ap.add_argument("--max-width", type=float, default=0.085, help="gripper opening, m (real: 88 mm open, 3 mm margin)")
    ap.add_argument("--min-width", type=float, default=0.040,
                    help="narrowest grasp kept, m (real: jaws stop ~30 mm apart at the servo's end of travel)")
    ap.add_argument("--max-depth", type=float, default=0.03,
                    help="GraspNet depth (finger tips past the grasp centre) kept, m: grip with the outer 30 mm only")
    ap.add_argument("--frames", type=int, default=15)
    ap.add_argument("--radius", type=float, default=0.2, help="scene kept within this base-xy distance of the object, m")
    ap.add_argument("--pad", type=int, default=250, help="px of scene kept around the object box")
    ap.add_argument("--on-object", type=float, default=0.02, help="grasp centre to object point, m")
    ap.add_argument("--no-show", action="store_true", help="do not open the image")
    ap.add_argument("--no-hold", action="store_true", help="exit after publishing to RViz (default: wait for Ctrl+C)")
    ap.add_argument("--save-as", metavar="YAML",
                    help="write the result here instead of ~/.ros/fr3_last_grasp.yaml (e.g. the box to put things in)")
    ap.add_argument("--assume-height", type=float, metavar="M",
                    help="object without depth (dark/matte, e.g. the black tray, 47 mm): footprint = SAM mask "
                         "projected onto the plane this high above the table; top-down geometric grasp only")
    ap.add_argument("--no-sam", action="store_true", help="use the box (colour: blob) instead of a SAM mask")
    a = ap.parse_args()

    bgr, raw, K, dist, T = capture(a.frames)
    model = camera.load_depth_model()
    depth = camera.correct_depth(raw, model)
    print(f"Depth hieu chinh: {model.get('note') or model}")
    robot_frame = T is not None
    if not robot_frame:
        T = table_frame(depth, K)
        print(f"Chua co TF base_footprint -> {camera.OPTICAL_FRAME} (hand-eye): toa do theo MAT BAN, khong theo robot.")
    box, label = box_from_gemini(bgr, a.gemini) if a.gemini else box_from_color(bgr, a.color)
    if box is None:
        print("Khong tim thay vat.")
        return
    print(f"Vat: {label}, khung {box}")
    mask = None if a.no_sam else sam_mask(bgr, box)
    if mask is None:
        mask = np.zeros(depth.shape, bool)
        if a.color and a.no_sam:
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            for lo, hi in COLORS[a.color]:
                mask |= cv2.inRange(hsv, lo, hi) > 0
            mask[:box[1]], mask[box[3] + 1:], mask[:, :box[0]], mask[:, box[2] + 1:] = False, False, False, False
        else:
            mask[box[1]:box[3] + 1, box[0]:box[2] + 1] = True
    # Scene around the object (GraspNet wants context; collisions need the table).
    pad = a.pad
    region = np.zeros(depth.shape, bool)
    region[max(0, box[1] - pad):box[3] + pad, max(0, box[0] - pad):box[2] + pad] = True
    scene = camera_points(depth, K, region)
    obj, ov, ou = camera_points(depth, K, cv2.erode(mask.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0, pixels=True)
    obj_rgb = bgr[ov, ou][:, ::-1]
    # Keep only the table around the object (no floor, little of the robot):
    # within a.radius of the object in base xy, up to 25 cm above the table.
    if len(obj) and len(scene):
        sb = scene @ T[:3, :3].T + T[:3, 3]
        ob = obj @ T[:3, :3].T + T[:3, 3]
        near = np.linalg.norm(sb[:, :2] - np.median(ob[:, :2], 0), axis=1) < a.radius
        table_z = np.percentile(sb[near, 2], 20) if near.sum() > 100 else np.min(ob[:, 2])
        scene = scene[near & (sb[:, 2] > table_z - 0.03) & (sb[:, 2] < table_z + 0.25)]
        print(f"Mat ban quanh vat z = {table_z * 1000:.0f} mm; vat cao {(np.percentile(ob[:, 2], 95) - table_z) * 1000:.0f} mm")
    print(f"Diem: canh {len(scene)}, vat {len(obj)}")
    box3d, geo = None, None
    if a.assume_height:
        # 2026-11-01: the black tray returns depth on ~15 % of its pixels, so no box and no GraspNet.
        # Its outline seen from above is its rim: cast the mask pixels' rays onto the rim plane.
        sb = scene @ T[:3, :3].T + T[:3, 3]
        tz = sb[np.abs(sb[:, 2] - 0.012) < 0.03, 2]
        table_z = float(np.median(tz)) if len(tz) > 100 else 0.012
        v, u = np.nonzero(mask)
        rays = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones(len(u))], 1) @ T[:3, :3].T
        t = (table_z + a.assume_height - T[2, 3]) / rays[:, 2]
        obj = np.empty((0, 3))          # the few depth points on it are noise: not saved as object_points
        box3d = object_box(T[:3, 3] + t[:, None] * rays, table_z)
        print(f"Khong dung depth cua vat: vien SAM chieu len mat phang cao {a.assume_height * 1000:.0f} mm tren ban "
              f"(ban z = {table_z * 1000:.0f} mm)")
    elif len(obj) >= 30:
        ob = obj @ T[:3, :3].T + T[:3, 3]
        box3d = object_box(ob, table_z)
    if box3d is not None:
        Tgeo, wgeo, ok, why = geometric_grasp(box3d, a.max_width, a.min_width)
        geo = (Tgeo, wgeo, ok)
        L, W, H = box3d["size"] * 1000
        print(f"Hop bao 3D: {L:.0f} x {W:.0f} x {H:.0f} mm, tam ({box3d['center'][0] * 1000:.0f}, "
              f"{box3d['center'][1] * 1000:.0f}), xoay {np.degrees(box3d['yaw']):.0f} deg")
        print(f"Grasp hinh hoc: tam ({Tgeo[0, 3] * 1000:.0f}, {Tgeo[1, 3] * 1000:.0f}, {Tgeo[2, 3] * 1000:.0f}) mm, "
              f"tu tren xuong, mo {wgeo * 1000:.0f} mm -> " + ("DAT" if ok else f"KHONG DAT ({why})"))
    vis = bgr.copy()
    overlay = vis.copy()
    overlay[mask] = (255, 0, 255)
    vis = cv2.addWeighted(overlay, 0.35, vis, 0.65, 0)
    cv2.rectangle(vis, tuple(box[:2]), tuple(box[2:]), (255, 0, 255), 1)
    name = "".join(ch if ch.isalnum() else "_" for ch in str(a.color or label or "gemini"))[:30]
    shot = os.path.join(LOG_DIR, f"{time.strftime('%H%M%S')}_graspnet_{name}.jpg")

    def finish(note, grasps=()):
        if geo is not None:
            Tc = np.linalg.inv(T) @ geo[0]
            draw_pose(vis, Tc[:3, :3], Tc[:3, 3], geo[1], K, (0, 255, 0) if geo[2] else (0, 0, 255), "HH", thick=3)
        cv2.putText(vis, note, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 4)
        cv2.putText(vis, note, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
        cv2.imwrite(shot, vis)
        geo_out = None if geo is None else dict(T_base=geo[0].tolist(), width=float(geo[1]), ok=bool(geo[2]),
                                                 box_size_m=box3d["size"].tolist(), box_yaw_deg=float(np.degrees(box3d["yaw"])))
        box3d_out = None if box3d is None else dict(center=np.asarray(box3d["center"]).tolist(),
                                                    size=np.asarray(box3d["size"]).tolist(), yaw=float(box3d["yaw"]))
        result = dict(target=label, box=box, note=note, robot_frame=bool(robot_frame), target_box3d=box3d_out,
                      geometric=geo_out, graspnet=list(grasps))
        if robot_frame and len(obj):   # for mtc_pregrasp.py: object width between the jaws of a grasp
            result["object_points"] = shot.replace(".jpg", "_object_points_base.npy")
            np.save(result["object_points"], (obj @ T[:3, :3].T + T[:3, 3]).astype(np.float32))
        yaml.safe_dump(result, open(shot.replace(".jpg", ".yaml"), "w"), sort_keys=False)
        if robot_frame:   # newest result for mtc_pregrasp.py / scene_obstacles.py
            yaml.safe_dump(dict(result, image=shot), open(os.path.expanduser(a.save_as or LAST_RESULT), "w"), sort_keys=False)
        np.save(shot.replace(".jpg", "_object_points_cam.npy"), obj)
        print(f"Anh: {shot}")
        if not a.no_show:
            subprocess.Popen(["xdg-open", shot], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if box3d is not None and robot_frame:   # RViz view is in base_footprint
            if not a.no_hold:
                print("Dang hien thi trong RViz; Ctrl+C de thoat.")
            publish_view(obj @ T[:3, :3].T + T[:3, 3], obj_rgb, box3d, geo, list(grasps), hold=not a.no_hold)

    if a.assume_height:
        return finish(f"{label}: chi grasp hinh hoc (cao gia dinh {a.assume_height * 1000:.0f} mm)")
    if len(obj) < 30 or len(scene) < 1000:
        print("Qua it diem depth tren vat / canh.")
        return finish("qua it diem depth")
    if not torch.cuda.is_available():
        # The pointnet2 sampling op is CUDA-only ("CPU not supported", 2026-11-02 after CUDA broke).
        print("Khong co CUDA: bo qua GraspNet, chi grasp hinh hoc (mtc_pregrasp.py --geometric).")
        return finish(f"{label}: chi grasp hinh hoc (khong co GPU)")
    idx = np.random.choice(len(scene), NUM_POINT, replace=len(scene) < NUM_POINT)
    device = torch.device("cuda:0")
    net = load_network(device)
    t0 = time.time()
    with torch.no_grad():
        end = net({"point_clouds": torch.from_numpy(scene[idx][None]).to(device)})
        # This fork's pred_decode multiplies the width head by 10 (clamped to
        # 75 mm), the official baseline by 1.2 -- the head is trained in metres
        # (loss.py is unchanged), so x10 saturates every width. Undo it here.
        end["grasp_width_pred"] = end["grasp_width_pred"] * (1.2 / 10.0)
        gg = GraspGroup(pred_decode(end)[0].detach().cpu().numpy())
    print(f"GraspNet ({device}): {time.time() - t0:.1f} s, {len(gg)} grasp")
    if len(gg) == 0:
        print("GraspNet khong ra grasp nao.")
        return finish("GraspNet: 0 grasp")
    # On the object: grasp centre within on_object m of an object point.
    sub = obj[::max(1, len(obj) // 800)]
    d = np.min(np.linalg.norm(gg.translations[:, None, :] - sub[None], axis=2), axis=1)
    print(f"  khoang cach tam grasp -> vat: min {d.min() * 1000:.0f} mm, trung vi {np.median(d) * 1000:.0f} mm; "
          f"do mo {gg.widths.min() * 1000:.0f}..{gg.widths.max() * 1000:.0f} mm")
    gg = gg[d < a.on_object]
    n_on = len(gg)
    print(f"  nam tren vat (< {a.on_object * 1000:.0f} mm): {n_on}")
    if len(gg):
        coll = ModelFreeCollisionDetector(scene, voxel_size=0.01).detect(gg, approach_dist=0.05, collision_thresh=0.01)
        for g in gg[coll]:
            draw_grasp(vis, g, K, (150, 150, 150))          # grey: on the object but colliding
        gg = gg[~coll]
    print(f"  khong va cham: {len(gg)}")
    if len(gg):
        wide = gg.widths > a.max_width
        for g in gg[wide]:
            draw_grasp(vis, g, K, (0, 0, 255))              # red: wider than the gripper
        gg = gg[~wide]
    print(f"  do mo <= {a.max_width * 1000:.0f} mm: {len(gg)}")
    if len(gg):
        narrow = gg.widths < a.min_width
        for g in gg[narrow]:
            draw_grasp(vis, g, K, (0, 0, 255))              # red: narrower than the jaws can close
        gg = gg[~narrow]
    print(f"  do mo >= {a.min_width * 1000:.0f} mm (kep chi khep toi ~30 mm): {len(gg)}")
    if len(gg):
        gg = gg[gg.depths <= a.max_depth + 1e-6]
    print(f"  ngon an vao <= {a.max_depth * 1000:.0f} mm: {len(gg)}")
    if len(gg) == 0:
        print("Khong con grasp nao.")
        return finish(f"{label}: GraspNet 0 dat ({n_on} tren vat; xam=va cham, do=qua rong); HH = hinh hoc")
    gg = gg.nms().sort_by_score()[:a.top]
    out = []
    for i, g in enumerate(gg):
        Tg = np.eye(4)
        Tg[:3, :3], Tg[:3, 3] = g.rotation_matrix, g.translation
        Tb = T @ Tg
        approach = Tb[:3, 0]
        tilt = np.degrees(np.arccos(np.clip(-approach[2], -1, 1)))
        print(f"#{i + 1} diem {g.score:.3f}, mo {g.width * 1000:.0f} mm, sau {g.depth * 1000:.0f} mm, tam ({Tb[0, 3] * 1000:.0f}, {Tb[1, 3] * 1000:.0f}, "
              f"{Tb[2, 3] * 1000:.0f}) mm, huong tiep can lech thang dung {tilt:.0f} deg")
        draw_grasp(vis, g, K, (0, 140, 255) if i == 0 else (0, 220, 255), f"#{i + 1}")
        out.append(dict(score=float(g.score), width=float(g.width), depth=float(g.depth), T_base=Tb.tolist(),
                        tilt_deg=float(tilt)))
    finish(f"{label}: HH = hinh hoc (xanh la), #1.. = GraspNet (cam)", out)

if __name__ == "__main__":
    main()
