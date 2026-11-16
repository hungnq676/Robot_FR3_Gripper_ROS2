#!/usr/bin/env python3
"""Put the objects seen on the table into the MoveIt planning scene as boxes,
so plans (RViz, mtc_pregrasp.py) go around them. Moves nothing.

  median of N depth frames (camera.DEPTH_MODEL_FILE applied) -> points in
  base_footprint through the hand-eye TF -> points 8 mm..30 cm above the
  table, minus the robot itself (capsules around its links from TF) ->
  connected regions in the image -> per region an upright box: rotated
  rectangle around its points in xy (+ --pad), from the table to its top.

The target of the last grasp_once.py run (~/.ros/fr3_last_grasp.yaml) is left
out by default: the fingers have to reach around it (--keep-target to add it).
Previous boxes ("vat_*") are replaced each run; --clear only removes them.
Objects without depth (glossy, dark, < ~2 cm) cannot be seen and are missing.

Usage: scene_obstacles.py [--frames 15] [--pad 0.01] [--keep-target] [--clear]
"""
import argparse
import os
import sys
import time
import warnings

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(__file__))
import camera  # noqa: E402

PREFIX = "vat_"
TABLE_TOP_Z = 0.012       # base_footprint, same as point_test / real_cell_scene
MIN_ABOVE = 0.008         # depth noise on the table is ~3 mm RMS after the depth model
MAX_ABOVE = 0.30
MIN_PIXELS = 120
ROBOT_LINKS = ["base_link", "shoulder_link", "upperarm_link", "forearm_link", "wrist1_link",
               "wrist2_link", "wrist3_link", "base_link_gripper", "grasp_frame"]
ROBOT_RADIUS = 0.09       # capsule radius around the chain of link origins (arm ~ 7 cm thick + margin)
LAST_RESULT = os.path.expanduser("~/.ros/fr3_last_grasp.yaml")
_SESSION = os.path.expanduser("~/.ros/fr3_session_dir")
LOG_DIR = (os.path.join(open(_SESSION).read().strip(), "anh") if os.path.exists(_SESSION)
           else os.path.expanduser(time.strftime("~/fr3_hungros2/nhat_ky/anh/%Y-%m-%d")))


def tf_matrix(t):
    T = np.eye(4)
    q = t.transform.rotation
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    return T


def capture(node, tf, frames):
    got = {"depth": []}
    q = rclpy.qos.qos_profile_sensor_data
    node.create_subscription(CameraInfo, camera.INFO_TOPIC, lambda m: got.__setitem__("info", m), q)
    node.create_subscription(Image, camera.RGB_TOPIC, lambda m: got.__setitem__("rgb", m), q)
    node.create_subscription(Image, camera.DEPTH_TOPIC, lambda m: got["depth"].append(
        np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).copy()), q)
    t0 = time.time()
    while len(got["depth"]) < frames or "info" not in got or "rgb" not in got:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0 > 15:
            raise RuntimeError("khong co anh/depth (run_astra.sh da chay chua?)")
    T = tf_matrix(tf.lookup_transform("base_footprint", camera.OPTICAL_FRAME, Time()))
    links = []
    for name in ROBOT_LINKS:
        try:
            links.append(tf_matrix(tf.lookup_transform("base_footprint", name, Time()))[:3, 3])
        except Exception:   # noqa: BLE001 - a missing link only shortens the robot mask
            pass
    D = np.stack(got["depth"][-frames:]).astype(np.float64) / 1000.0
    D[D <= 0] = np.nan
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        depth = np.nanmedian(D, 0)
    depth = camera.correct_depth(np.nan_to_num(depth), camera.load_depth_model())
    return camera.to_bgr(got["rgb"]), depth, np.array(got["info"].k).reshape(3, 3), T, np.array(links)


def near_robot(P, links):
    """Points within ROBOT_RADIUS of the polyline through the robot's link origins."""
    near = np.zeros(len(P), bool)
    for a, b in zip(links[:-1], links[1:]):
        ab = b - a
        t = np.clip(((P - a) @ ab) / max(ab @ ab, 1e-9), 0, 1)
        near |= np.linalg.norm(P - (a + t[:, None] * ab), axis=1) < ROBOT_RADIUS
    return near


def find_boxes(depth, K, T, links, pad, target):
    h, w = depth.shape
    v, u = np.mgrid[0:h, 0:w]
    z = depth
    P = np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z], -1).reshape(-1, 3)
    B = P @ T[:3, :3].T + T[:3, 3]
    ok = (z.ravel() > 0.3) & (B[:, 2] > TABLE_TOP_Z + MIN_ABOVE) & (B[:, 2] < TABLE_TOP_Z + MAX_ABOVE)
    if len(links) > 1:
        ok &= ~near_robot(B, links)
    mask = ok.reshape(h, w).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask)
    boxes = []
    for k in range(1, n):
        if stats[k, cv2.CC_STAT_AREA] < MIN_PIXELS:
            continue
        sel = (lab.ravel() == k) & ok
        pts = B[sel]
        if len(pts) < MIN_PIXELS // 2:
            continue
        (cx, cy), (sx, sy), ang = cv2.minAreaRect((pts[:, :2] * 1000).astype(np.float32))
        top = np.percentile(pts[:, 2], 95)
        box = dict(center=[cx / 1000, cy / 1000, (TABLE_TOP_Z + top) / 2],
                   size=[sx / 1000 + 2 * pad, sy / 1000 + 2 * pad, top - TABLE_TOP_Z],
                   yaw=float(np.radians(ang)), pixels=int(sel.sum()), label=k)
        if target is not None and np.linalg.norm(np.array(box["center"][:2]) - target[:2]) < max(0.04, target[2]):
            box["target"] = True
        boxes.append(box)
    return boxes, lab


def scene_call(node, cli, req):
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
    if fut.result() is None:
        raise RuntimeError(f"{cli.srv_name}: khong tra loi (move_group da chay chua?)")
    return fut.result()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=15)
    ap.add_argument("--pad", type=float, default=0.01, help="margin added on each side in xy, m")
    ap.add_argument("--keep-target", action="store_true", help="also add the last grasp_once target")
    ap.add_argument("--clear", action="store_true", help="only remove the vat_* boxes")
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("scene_obstacles")
    tf = Buffer()
    TransformListener(tf, node)
    get = node.create_client(GetPlanningScene, "/get_planning_scene")
    apply = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not apply.wait_for_service(timeout_sec=10):
        sys.exit("khong co /apply_planning_scene (run_real.sh da chay chua?)")
    t0 = time.time()
    while time.time() - t0 < 1.5:
        rclpy.spin_once(node, timeout_sec=0.1)

    req = GetPlanningScene.Request()
    req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
    old = [o.id for o in scene_call(node, get, req).scene.world.collision_objects if o.id.startswith(PREFIX)]
    diff = PlanningScene(is_diff=True)
    for oid in old:
        diff.world.collision_objects.append(CollisionObject(id=oid, operation=CollisionObject.REMOVE))
    if a.clear:
        scene_call(node, apply, ApplyPlanningScene.Request(scene=diff))
        print(f"Da xoa {len(old)} vat cu.")
        return

    target = None
    if not a.keep_target and os.path.exists(LAST_RESULT):
        last = yaml.safe_load(open(LAST_RESULT))
        if last.get("target_box3d"):
            c, s = last["target_box3d"]["center"], last["target_box3d"]["size"]
            target = np.array([c[0], c[1], 0.5 * float(np.hypot(s[0], s[1]))])
            print(f"Bo qua vat dich cua grasp_once: {last.get('target')} tai ({c[0] * 1000:.0f}, {c[1] * 1000:.0f}) mm")

    bgr, depth, K, T, links = capture(node, tf, a.frames)
    boxes, lab = find_boxes(depth, K, T, links, a.pad, target)
    vis = bgr.copy()
    n_add = 0
    for i, b in enumerate(boxes):
        colour = (0, 200, 255) if b.get("target") else (0, 255, 0)
        vis[lab == b["label"]] = (0.5 * vis[lab == b["label"]] + 0.5 * np.array(colour)).astype(np.uint8)
        ys, xs = np.nonzero(lab == b["label"])
        txt = "dich" if b.get("target") else f"{PREFIX}{n_add + 1}"
        cv2.putText(vis, txt, (int(xs.mean()) - 20, int(ys.min()) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)
        print(f"  {txt:7s}: tam ({b['center'][0] * 1000:.0f}, {b['center'][1] * 1000:.0f}) mm, "
              f"{b['size'][0] * 1000:.0f} x {b['size'][1] * 1000:.0f} x {b['size'][2] * 1000:.0f} mm, "
              f"xoay {np.degrees(b['yaw']):.0f} deg, {b['pixels']} diem")
        if b.get("target"):
            continue
        n_add += 1
        co = CollisionObject(id=f"{PREFIX}{n_add}", operation=CollisionObject.ADD)
        co.header.frame_id = "base_footprint"
        co.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(x) for x in b["size"]]))
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, b["center"])
        qx, qy, qz, qw = Rotation.from_euler("z", b["yaw"]).as_quat()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
        co.primitive_poses.append(pose)
        diff.world.collision_objects.append(co)
    scene_call(node, apply, ApplyPlanningScene.Request(scene=diff))
    os.makedirs(LOG_DIR, exist_ok=True)
    shot = os.path.join(LOG_DIR, time.strftime("%H%M%S") + "_vat_can.jpg")
    cv2.imwrite(shot, vis)
    print(f"Them {n_add} vat can vao planning scene (xoa {len(old)} cu). Anh: {shot}")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
