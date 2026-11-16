#!/usr/bin/env python3
"""Guided hand-eye data collection on the REAL FR3. Nothing moves without the user.

For the J6-cap tag (tag36h11 id 2 on wrist2_link) it proposes poses around the
current one where the camera should see the tag well: tag 15-50 deg off the
camera axis (a tag seen face-on flips its pose), inside the image, flange above
the table, and spread in position, HEIGHT and tilt (one height only left the
depth axis unobserved in the first calibration, 2026-10-30). For each pose:
  plan with Pilz PTP (plan only) -> show the path in RViz -> print what will
  move -> wait for the user: y = execute exactly that path, s = skip, q = quit.
tools/tag_check.py (running separately) records a sample whenever the arm
stands still. Needs the real stack (run_real.sh) and the camera TF.

Close camera (2026-10-30, camera ~0.7 m above the objects): the old
calibration no longer says where the camera is, and big random moves leave
the small field of view. --camera-from-tag estimates the camera from the tag
seen now (tag pose on wrist2 from ~/.ros/fr3_tag_in_wrist2.yaml), and
--spread/--angle/--margin keep the proposals small and inside the image.
Astra (2026-10-31): no depth closer than ~0.5 m, and samples far above the
table extrapolate badly (round 5) -> --min-dist 0.45 keeps the tag at working range.

Usage: handeye_guided.py [--poses 15] [--scale 0.3] [--dry-run]
       [--camera-from-tag] [--spread 0.5] [--angle 15 40] [--margin 120] [--min-dist 0.45]
       [--min-z 0.15] [--max-step 40]
"""
import os
import argparse
import math
import threading
import time

import numpy as np
import rclpy
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (Constraints, DisplayTrajectory, JointConstraint,
                             MotionPlanRequest, RobotState)
from moveit_msgs.srv import GetPositionFK
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, JointState
import cv2
import tf2_ros
import yaml
from sensor_msgs.msg import Image

import camera

JOINTS = ["j1", "j2", "j3", "j4", "j5", "j6"]
TAG_IN_WRIST2 = np.array([0.0027, -0.0502, 0.0995])   # from the first calibration
TAG_NORMAL_W2 = np.array([0.0, -1.0, 0.0])
SPREAD_DEG = np.array([25, 20, 25, 30, 30, 0])         # J6 does not move the tag
MIN_FLANGE_Z = 0.15       # m above base_footprint: table height is still uncertain
MAX_STEP_DEG = 70.0       # largest single-joint travel allowed per move


def tf_matrix(t):
    q, p = t.transform.rotation, t.transform.translation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def pose_matrix(ps):
    q, p = ps.pose.orientation, ps.pose.position
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [p.x, p.y, p.z]
    return T


class Guided(Node):
    def __init__(self):
        super().__init__("handeye_guided")
        self.q = None
        self.K = None
        self.size = None
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_subscription(CameraInfo, camera.INFO_TOPIC, self.on_info, 1)
        self.tf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tf, self)
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.move = ActionClient(self, MoveGroup, "/move_action")
        self.exe = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.display = self.create_publisher(DisplayTrajectory, "/display_planned_path", 1)
        # Pose-proposal limits (handeye_guided main() overrides them from the command line).
        self.D = None
        self.min_z = MIN_FLANGE_Z
        self.angle = (15.0, 50.0)
        self.margin = 80.0
        self.min_dist = 0.0
        self.spread = 1.0
        self.candidates = 600
        self.tag_xyz, self.tag_normal = TAG_IN_WRIST2, TAG_NORMAL_W2

    def on_js(self, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in JOINTS):
            self.q = np.array([d[j] for j in JOINTS])

    def on_info(self, m):
        self.K = np.array(m.k).reshape(3, 3)
        self.D = np.array(m.d)
        self.size = (m.width, m.height)

    @staticmethod
    def wait(future, timeout):
        t0 = time.time()
        while not future.done() and time.time() - t0 < timeout:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def fk_links(self, q, links):
        req = GetPositionFK.Request()
        req.header.frame_id = "base_footprint"
        req.fk_link_names = links
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = [float(v) for v in q]
        res = self.wait(self.fk.call_async(req), 2.0)
        if res is None or res.error_code.val != 1:
            return None
        return {n: pose_matrix(p) for n, p in zip(res.fk_link_names, res.pose_stamped)}

    def camera_from_tag(self, tag_in_w2, tag_size=0.0339, tag_id=2):
        """base_footprint -> optical frame from the tag in the current image and FK."""
        msgs = []
        sub = self.create_subscription(Image, camera.RGB_TOPIC, msgs.append, 1)
        t0 = time.time()
        while not msgs and time.time() - t0 < 5:
            time.sleep(0.1)
        self.destroy_subscription(sub)
        if not msgs or self.D is None:
            return None, "khong co anh camera"
        m = msgs[-1]
        gray = cv2.cvtColor(camera.to_bgr(m), cv2.COLOR_BGR2GRAY)
        det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
        corners, ids, _ = det.detectMarkers(gray)
        if ids is None or tag_id not in ids.ravel():
            return None, "camera khong thay tag"
        c = corners[list(ids.ravel()).index(tag_id)][0].astype(np.float64)
        h = tag_size / 2
        obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
        n, rvecs, tvecs, errs = cv2.solvePnPGeneric(obj, c, self.K, self.D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        best = int(np.argmin(np.ravel(errs)))
        T_opt_tag = np.eye(4)
        T_opt_tag[:3, :3] = cv2.Rodrigues(rvecs[best])[0]
        T_opt_tag[:3, 3] = tvecs[best].ravel()
        T_base_w2 = tf_matrix(self.tf.lookup_transform("base_footprint", "wrist2_link", Time()))
        errs = np.sort(np.ravel(errs))
        note = f"tag {np.linalg.norm(c[0] - c[1]):.0f} px, sai so PnP {errs[0]:.2f} px"
        if len(errs) > 1 and errs[1] < 2.0 * errs[0]:
            note += " (tag nhin gan nhu thang mat: huong camera chi gan dung, du de chon tu the)"
        return T_base_w2 @ tag_in_w2 @ np.linalg.inv(T_opt_tag), note

    def score(self, q, T_base_opt):
        """Tag centre/normal in base, its pixel, view angle; None when unusable."""
        fk = self.fk_links(q, ["wrist2_link", "wrist3_link", "grasp_frame"])
        if fk is None:
            return None
        if min(fk[l][2, 3] for l in fk) < self.min_z:
            return None
        p = (fk["wrist2_link"] @ np.r_[self.tag_xyz, 1])[:3]
        nrm = fk["wrist2_link"][:3, :3] @ self.tag_normal
        cam = T_base_opt[:3, 3]
        to_cam = cam - p
        dist = np.linalg.norm(to_cam)
        ang = math.degrees(math.acos(np.clip(nrm @ to_cam / dist, -1, 1)))
        if not self.angle[0] <= ang <= self.angle[1] or not self.min_dist <= dist <= 1.4:
            return None
        pc = np.linalg.inv(T_base_opt) @ np.r_[p, 1]
        if pc[2] <= 0.2:
            return None
        u = self.K[0, 0] * pc[0] / pc[2] + self.K[0, 2]
        v = self.K[1, 1] * pc[1] / pc[2] + self.K[1, 2]
        w, h = self.size
        m = self.margin
        if not (m < u < w - m and 0.75 * m < v < h - 0.75 * m):
            return None
        return dict(p=p, n=nrm, ang=ang, dist=dist, uv=(u, v))

    def propose(self, n_poses, rng, T_base_opt=None):
        if T_base_opt is None:
            T_base_opt = tf_matrix(self.tf.lookup_transform("base_footprint", camera.OPTICAL_FRAME, Time()))
        q0 = self.q.copy()
        cands = []
        for _ in range(self.candidates):
            q = q0 + np.radians(rng.uniform(-1, 1, 6) * SPREAD_DEG * self.spread)
            s = self.score(q, T_base_opt)
            if s is not None:
                cands.append((q, s))
        if not cands:
            return []
        # Farthest-point selection on tag position (height weighted x2) and normal.
        feats = np.array([np.r_[s["p"][:2], 2.0 * s["p"][2], 0.15 * s["n"]] for _, s in cands])
        chosen = [int(np.argmax(feats[:, 2]))]            # start from the highest pose
        while len(chosen) < min(n_poses, len(cands)):
            d = np.min(np.linalg.norm(feats[:, None] - feats[chosen][None], axis=2), axis=1)
            chosen.append(int(np.argmax(d)))
        picked = [cands[i] for i in chosen]
        # Visit order: nearest neighbour in joint space from the current pose.
        order, cur, left = [], q0, list(range(len(picked)))
        while left:
            k = min(left, key=lambda i: np.abs(picked[i][0] - cur).max())
            order.append(picked[k])
            cur = picked[k][0]
            left.remove(k)
        return order

    def plan(self, q_goal, scale):
        req = MotionPlanRequest()
        req.group_name = "arm"
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = "PTP"
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = scale
        req.max_acceleration_scaling_factor = scale
        req.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=j, position=float(v), tolerance_above=1e-3,
                            tolerance_below=1e-3, weight=1.0) for j, v in zip(JOINTS, q_goal)])]
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = True
        gh = self.wait(self.move.send_goal_async(goal), 10.0)
        if gh is None or not gh.accepted:
            return None
        res = self.wait(gh.get_result_async(), 20.0)
        if res is None or res.result.error_code.val != 1:
            return None
        return res.result

    def show(self, result):
        msg = DisplayTrajectory()
        msg.trajectory_start = result.trajectory_start
        msg.trajectory = [result.planned_trajectory]
        self.display.publish(msg)

    def execute(self, trajectory):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        gh = self.wait(self.exe.send_goal_async(goal), 10.0)
        if gh is None or not gh.accepted:
            return False
        res = self.wait(gh.get_result_async(), 120.0)
        return res is not None and res.result.error_code.val == 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--poses", type=int, default=15)
    ap.add_argument("--scale", type=float, default=0.3)
    ap.add_argument("--dry-run", action="store_true", help="plan and show only, never execute")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--camera-from-tag", action="store_true",
                    help="camera moved: estimate it from the tag seen now instead of the old TF")
    ap.add_argument("--spread", type=float, default=1.0, help="scale of the random joint offsets")
    ap.add_argument("--angle", type=float, nargs=2, default=[15.0, 50.0], help="tag tilt to the camera, deg")
    ap.add_argument("--margin", type=float, default=80.0, help="tag centre this many px inside the image")
    ap.add_argument("--min-dist", type=float, default=0.0, help="closest tag-to-camera distance, m")
    ap.add_argument("--min-z", type=float, default=MIN_FLANGE_Z, help="lowest wrist/TCP height, m")
    ap.add_argument("--max-step", type=float, default=MAX_STEP_DEG, help="largest joint travel per move, deg")
    a = ap.parse_args()

    rclpy.init()
    node = Guided()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    t0 = time.time()
    while (node.q is None or node.K is None) and time.time() - t0 < 10:
        time.sleep(0.1)
    if node.q is None or node.K is None:
        print("Chua co /joint_states hoac camera_info. Robot that va camera da chay chua?")
        return
    for c in (node.fk,):
        c.wait_for_service(timeout_sec=10)
    node.move.wait_for_server(timeout_sec=10)
    node.exe.wait_for_server(timeout_sec=10)

    node.min_z, node.angle, node.margin, node.spread = a.min_z, tuple(a.angle), a.margin, a.spread
    node.min_dist = a.min_dist
    node.candidates = 600 if a.spread >= 1.0 else 3000
    T_base_opt = None
    if a.camera_from_tag:
        path = os.path.expanduser("~/.ros/fr3_tag_in_wrist2.yaml")
        tag_in_w2 = np.array(yaml.safe_load(open(path))["matrix"])
        node.tag_xyz, node.tag_normal = tag_in_w2[:3, 3], tag_in_w2[:3, 2]
        T_base_opt, note = node.camera_from_tag(tag_in_w2)
        if T_base_opt is None:
            print(f"Khong uoc luong duoc camera: {note}. Dua tag vao giua anh roi chay lai.")
            return
        rpy = np.degrees(Rotation.from_matrix(T_base_opt[:3, :3]).as_euler("xyz"))
        print(f"Camera uoc luong tu tag ({note}): xyz {T_base_opt[:3, 3].round(3).tolist()} m, "
              f"truc nhin {T_base_opt[:3, 2].round(2).tolist()}, rpy {rpy.round(0).tolist()} deg")
    poses = node.propose(a.poses, np.random.default_rng(a.seed), T_base_opt)
    print(f"\nDe xuat {len(poses)} tu the (tag nghieng {a.angle[0]:.0f}-{a.angle[1]:.0f} do so voi camera, "
          f"le anh {a.margin:.0f} px, bien do x{a.spread}, khop/TCP cao >= {a.min_z} m).")
    if not poses:
        return
    zs = [s["p"][2] for _, s in poses]
    print(f"Do cao tag: {min(zs):.2f} .. {max(zs):.2f} m tren base_footprint")
    done = 0
    for i, (q, s) in enumerate(poses, 1):
        cur = node.q.copy()
        delta = np.degrees(q - cur)
        print(f"\n=== Tu the {i}/{len(poses)} ===")
        print(f"  khop dich (deg): {np.degrees(q).round(1).tolist()}")
        print(f"  thay doi  (deg): {delta.round(1).tolist()}")
        print(f"  tag du kien: cao {s['p'][2]:.2f} m, cach camera {s['dist']:.2f} m, nghieng {s['ang']:.0f} deg so voi truc camera")
        if np.abs(delta).max() > a.max_step:
            print(f"  BO QUA: co khop phai quay > {a.max_step} deg")
            continue
        res = node.plan(q, a.scale)
        if res is None:
            print("  BO QUA: Pilz PTP khong lap duoc ke hoach (va cham hoac ngoai gioi han)")
            continue
        pts = res.planned_trajectory.joint_trajectory.points
        dur = pts[-1].time_from_start.sec + pts[-1].time_from_start.nanosec * 1e-9
        node.show(res)
        print(f"  Ke hoach OK: {dur:.1f} s, toc do {a.scale:.1f}. Duong di dang hien trong RViz (Planned Path).")
        if a.dry_run:
            print("  (dry-run: khong chay)")
            continue
        ans = input("  Go y + Enter de CHAY, s de bo qua, q de dung: ").strip().lower()
        if ans == "q":
            print("Dung theo yeu cau.")
            break
        if ans != "y":
            print("  Bo qua.")
            continue
        ok = node.execute(res.planned_trajectory)
        print("  Da toi noi." if ok else "  CHAY THAT BAI (xem log driver). Dung lai.")
        if not ok:
            break
        done += 1
        print("  Dung yen 3 s de tag_check ghi mau...")
        time.sleep(3.0)
    print(f"\nXong: da chay {done} tu the.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
