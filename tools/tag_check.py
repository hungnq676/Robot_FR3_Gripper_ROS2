#!/usr/bin/env python3
"""Check the J6-cap tag (tag36h11 id 2, 33 mm black square) against the robot, before any
hand-eye result exists.

Each time the arm stands still for ~1 s with the tag in view, record:
  robot : base_footprint -> wrist2_link from TF, times the nominal tag offset
  camera: tag pose in the camera colour optical frame (solvePnP, IPPE_SQUARE)
Pairs of samples are compared without the camera extrinsics: the distance the
tag moved and the angle it turned must be the same seen from the robot and
from the camera. With 3+ samples a rough eye-to-hand solve (cv2) is printed.
Read-only: sends no motion. Output goes to stdout (and the log file passed).

Every accepted sample is also saved to camera.SAMPLES_FILE in the
format of fairino_fr3_manipulation/handeye_calibration.py, so the full solve
(closed form + joint reprojection refinement) can be run on them with
  ros2 run fairino_fr3_manipulation handeye_calibration.py --ros-args \
    --params-file .../handeye_calibration.yaml -p solve_only:=true -p use_sim_time:=false
"""
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, JointState
import tf2_ros

import camera

TAG_ID = 2
TAG_SIZE = 0.0339  # inner black square (outer frame 50 mm); 33.9 mm from the reprojection minimum, round 5 (33.0 left PnP 2.4% short)
TAG_IN_WRIST2 = np.array([0.0, -0.0502, 0.102])
SAMPLES_FILE = camera.SAMPLES_FILE
CAMERA_BASE_FRAME = camera.BASE_FRAME


def tf_to_matrix(t):
    q = t.transform.rotation
    p = t.transform.translation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [p.x, p.y, p.z]
    return T


class TagCheck(Node):
    def __init__(self, log_path):
        super().__init__("tag_check")
        self.log = open(log_path, "a", buffering=1) if log_path else None
        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.image = None
        self.joints = []
        self.samples = []
        self.corners = []
        self.T_base_cam = None
        self.last_sample_q = None
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(Image, camera.RGB_TOPIC, self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, camera.INFO_TOPIC, self.on_info, qos_profile_sensor_data)
        # Depth aligned to colour: z of the tag centre, an observation
        # along the viewing ray that the corner reprojection alone barely sees.
        self.depth = None
        self.tag_depth = []
        self.create_subscription(Image, camera.DEPTH_TOPIC, self.on_depth, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 50)
        self.create_timer(0.5, self.tick)
        # --append: keep the samples already in SAMPLES_FILE and add new ones
        # (needs the joints/tag_depth keys written since 2026-10-30).
        if "--append" in sys.argv and os.path.exists(SAMPLES_FILE):
            old = np.load(SAMPLES_FILE)
            if "joints" in old.files and "tag_depth" in old.files:
                self.samples = [(q, Tb, Tc) for q, Tb, Tc in zip(old["joints"], old["base_gripper"], old["cam_tag"])]
                self.corners = list(old["corners"])
                self.tag_depth = list(old["tag_depth"])
                self.say(f"tag_check: ghi tiep, da co {len(self.samples)} mau trong {SAMPLES_FILE}")
            else:
                self.say("tag_check: file mau cu thieu joints/tag_depth, bat dau file moi")
        self.say(f"tag_check: waiting for the arm to stop with tag id {TAG_ID} in view")

    def say(self, text):
        print(text, flush=True)
        if self.log:
            self.log.write(text + "\n")

    def on_info(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        self.D = np.array(msg.d) if len(msg.d) else np.zeros(5)

    def on_image(self, msg):
        self.image = msg

    def on_depth(self, msg):
        self.depth = msg

    def tag_z(self, corners):
        """Median depth (m, optical z) inside the tag; NaN when unavailable."""
        if self.depth is None:
            return float("nan")
        d = np.frombuffer(self.depth.data, np.uint16).reshape(self.depth.height, self.depth.width)
        mask = np.zeros(d.shape, np.uint8)
        cv2.fillConvexPoly(mask, corners.astype(np.int32), 1)
        vals = d[(mask > 0) & (d > 0)].astype(float) / 1000.0
        return float(np.median(vals)) if vals.size >= 20 else float("nan")

    def on_joints(self, msg):
        named = dict(zip(msg.name, msg.position))
        if all(f"j{i}" in named for i in range(1, 7)):
            self.joints.append((time.time(), np.array([named[f"j{i}"] for i in range(1, 7)])))
            self.joints = self.joints[-200:]

    def still(self):
        recent = [q for t, q in self.joints if time.time() - t < 1.0]
        return len(recent) > 50 and np.ptp(np.array(recent), axis=0).max() < 1e-4

    def tick(self):
        if self.K is None or self.image is None or not self.still():
            return
        q = self.joints[-1][1]
        if self.last_sample_q is not None and np.abs(q - self.last_sample_q).max() < np.radians(2):
            return
        gray = cv2.cvtColor(self.bridge.imgmsg_to_cv2(self.image, "bgr8"), cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None or TAG_ID not in ids.ravel():
            return
        c = corners[list(ids.ravel()).index(TAG_ID)][0].astype(np.float64)
        h = TAG_SIZE / 2
        obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
        # Small tag: two poses can fit the corners (flip by 30-45 deg). Skip
        # the frame when the second pose fits nearly as well as the best.
        n_sol, rvecs, tvecs, errs = cv2.solvePnPGeneric(obj, c, self.K, self.D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if n_sol == 0:
            return
        errs = np.ravel(errs)
        order = np.argsort(errs)
        if n_sol > 1 and errs[order[1]] < 2.0 * errs[order[0]]:
            self.say(f"  (bo khung hinh: tag mo ho huong, sai so 2 nghiem {errs[order[0]]:.2f}/{errs[order[1]]:.2f} px)")
            return
        rvec, tvec = rvecs[order[0]], tvecs[order[0]]
        proj, _ = cv2.projectPoints(obj, rvec, tvec, self.K, self.D)
        reproj = float(np.linalg.norm(proj.reshape(4, 2) - c, axis=1).mean())
        T_cam_tag = np.eye(4)
        T_cam_tag[:3, :3] = cv2.Rodrigues(rvec)[0]
        T_cam_tag[:3, 3] = tvec.ravel()
        try:
            T_base_w2 = tf_to_matrix(self.tf_buffer.lookup_transform("base_footprint", "wrist2_link", Time()))
        except Exception as e:
            self.say(f"TF error: {e}")
            return
        self.last_sample_q = q
        side = np.mean([np.linalg.norm(c[k] - c[(k + 1) % 4]) for k in range(4)])
        self.samples.append((q, T_base_w2, T_cam_tag))
        self.corners.append(c)
        zd = self.tag_z(c)
        self.tag_depth.append(zd)
        self.save(T_cam_tag)
        n = len(self.samples)
        self.say(f"\n[mau {n}] khop (deg) {np.degrees(q).round(1).tolist()}")
        self.say(f"  tag: {side:.0f} px, cach camera {np.linalg.norm(tvec):.3f} m, sai so chieu {reproj:.2f} px,"
                 f" z theo PnP {tvec.ravel()[2]:.3f} / theo depth {zd:.3f} m")
        self.compare()

    def save(self, T_cam_tag_last):
        optical = self.image.header.frame_id
        try:
            T_opt_cambase = tf_to_matrix(self.tf_buffer.lookup_transform(optical, CAMERA_BASE_FRAME, Time()))
        except Exception as e:
            self.say(f"  (chua luu mau: thieu TF {optical} -> {CAMERA_BASE_FRAME}: {e})")
            return
        # Rough base->optical guess (only used for a log comparison by the solver).
        guess = self.T_base_cam if self.T_base_cam is not None else np.eye(4)
        os.makedirs(os.path.dirname(SAMPLES_FILE), exist_ok=True)
        np.savez(SAMPLES_FILE, base_gripper=[s[1] for s in self.samples],
                 cam_tag=[s[2] for s in self.samples], corners=self.corners,
                 K=self.K, D=self.D, T_base_optical_guess=guess,
                 T_optical_camera_base=T_opt_cambase, optical=optical, tag_size=TAG_SIZE,
                 tag_depth=np.array(self.tag_depth), joints=np.array([s[0] for s in self.samples]))

    def compare(self):
        n = len(self.samples)
        if n < 2:
            self.say("  (can it nhat 2 lan dung de so sanh)")
            return
        tag_base = [(T @ np.r_[TAG_IN_WRIST2, 1])[:3] for _, T, _ in self.samples]
        tag_cam = [Tc[:3, 3] for _, _, Tc in self.samples]
        self.say("  so sanh cap mau (robot vs camera):")
        d_err, a_err = [], []
        for i in range(n - 1):
            j = n - 1
            d_r = np.linalg.norm(tag_base[i] - tag_base[j])
            d_c = np.linalg.norm(tag_cam[i] - tag_cam[j])
            R_r = self.samples[i][1][:3, :3].T @ self.samples[j][1][:3, :3]
            R_c = self.samples[i][2][:3, :3].T @ self.samples[j][2][:3, :3]
            a_r = np.degrees(np.linalg.norm(Rotation.from_matrix(R_r).as_rotvec()))
            a_c = np.degrees(np.linalg.norm(Rotation.from_matrix(R_c).as_rotvec()))
            d_err.append(d_c - d_r)
            a_err.append(a_c - a_r)
            self.say(f"    mau {i + 1}->{j + 1}: di chuyen robot {d_r * 1000:6.1f} mm / camera {d_c * 1000:6.1f} mm"
                     f" (lech {(d_c - d_r) * 1000:+5.1f}) | xoay robot {a_r:5.1f} / camera {a_c:5.1f} deg"
                     f" (lech {a_c - a_r:+4.1f})")
        if n >= 3:
            self.solve()

    def solve(self):
        # Eye-to-hand: feed base->gripper as "gripper2base" inverted (OpenCV convention).
        R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
        for _, T_bw, T_ct in self.samples:
            T_wb = np.linalg.inv(T_bw)
            R_g2b.append(T_wb[:3, :3])
            t_g2b.append(T_wb[:3, 3])
            R_t2c.append(T_ct[:3, :3])
            t_t2c.append(T_ct[:3, 3])
        try:
            R, t = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=cv2.CALIB_HAND_EYE_PARK)
        except cv2.error as e:
            self.say(f"  giai hand-eye loi: {e}")
            return
        T_base_cam = np.eye(4)
        T_base_cam[:3, :3] = R
        T_base_cam[:3, 3] = t.ravel()
        self.T_base_cam = T_base_cam
        # Tag centre seen by the camera, moved into the base frame, against the
        # robot's own prediction (FK x nominal cap position).
        err = []
        for _, T_bw, T_ct in self.samples:
            p_cam = (T_base_cam @ T_ct)[:3, 3]
            p_fk = (T_bw @ np.r_[TAG_IN_WRIST2, 1])[:3]
            err.append(np.linalg.norm(p_cam - p_fk))
        self.say(f"  hand-eye tam ({len(self.samples)} mau): camera trong he robot xyz {T_base_cam[:3, 3].round(3).tolist()} m,"
                 f" tag camera vs robot lech TB {np.mean(err) * 1000:.1f} / max {np.max(err) * 1000:.1f} mm"
                 f" (chi tin khi >= 8 mau, xoay nhieu huong)")


def main():
    rclpy.init()
    paths = [a for a in sys.argv[1:] if not a.startswith("--")]
    node = TagCheck(paths[0] if paths else None)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
