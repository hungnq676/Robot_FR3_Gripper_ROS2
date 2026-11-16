#!/usr/bin/env python3
"""Eye-to-hand calibration: where is the fixed camera relative to the robot base?

An AprilTag is fixed somewhere on the gripper (its exact mounting need not be
known). The robot moves to N poses in front of the camera; at each one we
record the gripper pose in the base frame (forward kinematics via TF) and the
tag pose in the camera (PnP). Solving  T_base_gripper_i * Y = X * T_cam_tag_i
for X = T_base_camera (and Y = T_gripper_tag) with OpenCV's calibrateHandEye
(eye-to-hand form: base->gripper transforms as input) gives the camera pose
without measuring anything by hand.

Result: ~/.ros/fr3_oak_handeye.yaml  (base_frame -> camera_base_frame), the
tag-on-gripper estimate and per-sample residuals. The closed-form estimate is
refined by least squares on the reprojection error of the tag corners in all
samples (X and Y together). Samples are saved (samples_file) and can be solved
again offline with solve_only:=true.

  ros2 launch fairino_fr3_manipulation handeye_calibration.launch.py

Real robot (no camera TF yet): first guide the arm by hand so the tag faces the
camera, then start with  real:=true . The start pose becomes the workspace
centre and the first tag detection gives a rough camera position to aim with.
Every motion is planned first and only executed when its joint travel stays
small; the arm returns to the start pose at the end.
"""
import datetime
import os
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from scipy.optimize import least_squares
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PositionIKRequest
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

ARM_JOINTS = ["j1", "j2", "j3", "j4", "j5", "j6"]
METHODS = {"TSAI": cv2.CALIB_HAND_EYE_TSAI, "PARK": cv2.CALIB_HAND_EYE_PARK,
           "HORAUD": cv2.CALIB_HAND_EYE_HORAUD, "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
           "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS}


def to_matrix(rotation, translation):
    m = np.eye(4)
    m[:3, :3] = rotation
    m[:3, 3] = translation
    return m


def tf_to_matrix(transform):
    q, t = transform.rotation, transform.translation
    return to_matrix(Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(), [t.x, t.y, t.z])


class HandEyeCalibration(Node):
    def __init__(self):
        super().__init__("handeye_calibration")
        p = self.declare_parameter
        self.base_frame = p("base_frame", "base_footprint").value
        self.gripper_frame = p("gripper_frame", "base_link_gripper").value
        self.tcp_link = p("tcp_link", "grasp_frame").value
        self.camera_base_frame = p("camera_base_frame", "oak-d-base-frame").value
        self.tag_id = p("tag_id", 1).value
        self.tag_size = p("tag_size", 0.08).value
        # Nominal tag mounting (only used to aim the tag at the camera).
        self.tag_offset = np.array(p("nominal_tag_in_gripper", [0.077, -0.0045, 0.040]).value)
        self.tag_normal = np.array(p("nominal_tag_normal_in_gripper", [0.0, -1.0, 0.0]).value)
        self.tag_normal /= np.linalg.norm(self.tag_normal)
        # Axis of gripper_frame that points along the tool (kept as close to
        # straight down as the tag aiming allows, so the gripper stays clear).
        self.tool_axis = np.array(p("tool_axis_in_gripper", [0.0, 0.0, 1.0]).value)
        self.center = np.array(p("workspace_center", [0.35, -0.20, 0.30]).value)
        # Real robot: centre the sampled poses on the tag position of the pose
        # the arm is in at start, and bootstrap the camera position from the
        # tag seen there when no camera TF exists yet.
        self.center_from_start = p("center_from_start_pose", False).value
        # Reject plans whose largest single-joint travel exceeds this (rad).
        self.max_joint_travel = p("max_joint_travel", 1.5).value
        self.bootstrap_step = p("bootstrap_step", 0.04).value
        self.force_bootstrap = p("force_bootstrap", False).value  # test the bootstrap in sim
        # Candidate tag poses tried per step; the one nearest in joint space wins.
        self.candidates = p("candidates_per_pose", 4).value
        self.spread = np.array(p("workspace_spread", [0.10, 0.10, 0.08]).value)
        self.max_tilt = np.radians(p("max_tilt_deg", 20.0).value)
        self.num_poses = p("num_poses", 15).value
        self.min_samples = p("min_samples", 8).value
        self.frames_per_pose = p("frames_per_pose", 5).value
        self.max_reprojection_px = p("max_reprojection_error_px", 1.0).value
        # Reject a frame when the second IPPE pose fits within this factor of the best.
        self.ambiguity_ratio = p("pose_ambiguity_ratio", 2.0).value
        self.velocity_scaling = p("velocity_scaling", 0.25).value
        self.seed = p("random_seed", 7).value
        self.output_file = os.path.expanduser(p("output_file", "~/.ros/fr3_oak_handeye.yaml").value)
        self.return_to_observe = p("return_to_observe", True).value
        self.samples_file = os.path.expanduser(
            p("samples_file", "~/.ros/fr3_handeye_samples.npz").value)
        self.solve_only = p("solve_only", False).value

        group = ReentrantCallbackGroup()
        self.rgb = self.info = self.joints = None
        self.create_subscription(Image, p("image_topic", "/oak/rgb/image_raw").value,
                                 lambda m: setattr(self, "rgb", m), 1, callback_group=group)
        self.create_subscription(CameraInfo, p("camera_info_topic", "/oak/rgb/camera_info").value,
                                 lambda m: setattr(self, "info", m), 1, callback_group=group)
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: setattr(self, "joints", m), 10, callback_group=group)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik", callback_group=group)
        self.move_client = ActionClient(self, MoveGroup, "/move_action", callback_group=group)
        self.execute_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory",
                                           callback_group=group)
        self.observe_client = self.create_client(Trigger, "/fr3_demo/move_to_observe",
                                                 callback_group=group)
        detector_params = cv2.aruco.DetectorParameters()
        detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), detector_params)
        s = self.tag_size / 2.0
        self.object_points = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], float)

    # ------------------------------------------------------------- utilities
    def wait(self, future, timeout):
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def lookup(self, target, source):
        return tf_to_matrix(self.tf_buffer.lookup_transform(
            target, source, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)).transform)

    def current_arm(self):
        js = self.joints
        return [js.position[js.name.index(j)] for j in ARM_JOINTS]

    def wait_still(self, seconds=1.0, timeout=8.0):
        start = time.time()
        still_since = None
        while time.time() - start < timeout:
            js = self.joints
            moving = js is not None and js.velocity and max(abs(v) for v in js.velocity) > 0.005
            still_since = None if moving else (still_since or time.time())
            if still_since and time.time() - still_since >= seconds:
                return
            time.sleep(0.05)

    # ------------------------------------------------------- pose generation
    def gripper_pose_for(self, tag_position, camera_position, rng):
        """Gripper orientation whose tag normal points at the camera (plus a
        random tilt and spin for diversity), and the gripper origin that puts
        the tag at tag_position."""
        to_camera = camera_position - tag_position
        to_camera /= np.linalg.norm(to_camera)
        axis = np.cross(rng.normal(size=3), to_camera)
        tilted = Rotation.from_rotvec(axis / np.linalg.norm(axis) * rng.uniform(0, self.max_tilt))
        target_normal = tilted.apply(to_camera)
        # Rotation taking the tag normal onto target_normal while keeping the
        # tool axis as close to straight down as possible: match the frames
        # (normal, tool axis projected off the normal, their cross product).
        def frame(first, second):
            second = second - (second @ first) * first
            if np.linalg.norm(second) < 1e-6:
                # Tag normal (anti)parallel to the tool axis (tag on the J6 end
                # cap): any perpendicular works, the random spin below covers it.
                helper = np.eye(3)[np.argmin(np.abs(first))]
                second = helper - (helper @ first) * first
            second /= np.linalg.norm(second)
            return np.column_stack((first, second, np.cross(first, second)))
        R = frame(target_normal, np.array([0.0, 0.0, -1.0])) @ frame(
            self.tag_normal, self.tool_axis).T
        R = Rotation.from_rotvec(target_normal * rng.uniform(-1.0, 1.0)).as_matrix() @ R
        return to_matrix(R, tag_position - R @ self.tag_offset)

    def solve_ik(self, T_base_tcp):
        request = GetPositionIK.Request()
        ik = PositionIKRequest()
        ik.group_name = "arm"
        ik.ik_link_name = self.tcp_link
        ik.avoid_collisions = True
        ik.robot_state.joint_state.name = ARM_JOINTS
        ik.robot_state.joint_state.position = [float(v) for v in self.current_arm()]
        ik.pose_stamped.header.frame_id = self.base_frame
        q = Rotation.from_matrix(T_base_tcp[:3, :3]).as_quat()
        pose = ik.pose_stamped.pose
        pose.position.x, pose.position.y, pose.position.z = [float(v) for v in T_base_tcp[:3, 3]]
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = [float(v) for v in q]
        ik.timeout.sec = 1
        request.ik_request = ik
        result = self.wait(self.ik_client.call_async(request), 5.0)
        if result is None or result.error_code.val != 1:
            return None
        named = dict(zip(result.solution.joint_state.name, result.solution.joint_state.position))
        return [named[j] for j in ARM_JOINTS]

    def move_to(self, joints):
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = "arm"
        req.num_planning_attempts = 3
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self.velocity_scaling
        req.max_acceleration_scaling_factor = self.velocity_scaling
        constraints = Constraints()
        for name, value in zip(ARM_JOINTS, joints):
            constraints.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(value), tolerance_above=1e-3,
                tolerance_below=1e-3, weight=1.0))
        req.goal_constraints = [constraints]
        goal.request = req
        # Plan first, check the path, then execute: no surprise detours.
        # Sampling planners sometimes detour: plan up to 3 times, keep the shortest.
        goal.planning_options.plan_only = True
        trajectory, travel = None, np.inf
        for _ in range(3):
            handle = self.wait(self.move_client.send_goal_async(goal), 10.0)
            if handle is None or not handle.accepted:
                continue
            result = self.wait(handle.get_result_async(), 30.0)
            if result is None or result.result.error_code.val != 1:
                continue
            points = np.array([pt.positions for pt in
                               result.result.planned_trajectory.joint_trajectory.points])
            length = np.abs(np.diff(points, axis=0)).sum(axis=0).max() if len(points) > 1 else 0.0
            if length < travel:
                trajectory, travel = result.result.planned_trajectory, length
            if travel <= self.max_joint_travel:
                break
        if trajectory is None:
            return False
        if travel > self.max_joint_travel:
            self.get_logger().warn(
                f"plan rejected: a joint would travel {travel:.2f} rad (> {self.max_joint_travel})")
            return False
        execute = ExecuteTrajectory.Goal(trajectory=trajectory)
        handle = self.wait(self.execute_client.send_goal_async(execute), 10.0)
        if handle is None or not handle.accepted:
            return False
        result = self.wait(handle.get_result_async(), 120.0)
        return result is not None and result.result.error_code.val == 1

    # -------------------------------------------------------------- measuring
    def measure_tag(self):
        """Median tag pose over frames_per_pose frames -> (T_cam_tag, error)."""
        poses, errors, corners_seen, last = [], [], [], None
        K = np.array(self.info.k).reshape(3, 3)
        D = np.array(self.info.d, float)
        deadline = time.time() + 5.0
        while len(poses) < self.frames_per_pose and time.time() < deadline:
            msg = self.rgb
            if msg is None or msg is last:
                time.sleep(0.03)
                continue
            last = msg
            img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, : msg.width * 3]
            gray = cv2.cvtColor(img.reshape(msg.height, msg.width, 3), cv2.COLOR_RGB2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)
            if ids is None or self.tag_id not in ids.ravel():
                continue
            pts = corners[list(ids.ravel()).index(self.tag_id)][0].astype(float)
            # A small square tag has two poses that fit its 4 corners almost
            # equally (tilted either way); picking the wrong one flips the
            # orientation by 30-45 deg (seen on the real arm 2026-10-30, tag
            # ~25-38 px). Keep a frame only when the best pose clearly beats
            # the other one.
            n, rvecs, tvecs, reproj = cv2.solvePnPGeneric(
                self.object_points, pts, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if n == 0:
                continue
            order = np.argsort(np.ravel(reproj))
            if n > 1 and float(np.ravel(reproj)[order[1]]) < self.ambiguity_ratio * float(np.ravel(reproj)[order[0]]):
                continue
            rvec, tvec = rvecs[order[0]], tvecs[order[0]]
            proj, _ = cv2.projectPoints(self.object_points, rvec, tvec, K, D)
            error = float(np.linalg.norm(proj.reshape(-1, 2) - pts, axis=1).mean())
            if error > self.max_reprojection_px:
                continue
            poses.append((rvec.ravel(), tvec.ravel()))
            errors.append(error)
            corners_seen.append(pts)
        if len(poses) < max(2, self.frames_per_pose // 2):
            return None, None, None
        rotation = Rotation.from_rotvec([r for r, _ in poses]).mean().as_matrix()
        translation = np.median([t for _, t in poses], axis=0)
        return (to_matrix(rotation, translation), float(np.mean(errors)),
                np.median(corners_seen, axis=0))

    def bootstrap_camera(self, T_base_gripper):
        """Rough base->optical without any camera TF, for aiming only.

        The tag is translated by bootstrap_step along base x, y and z with the
        orientation kept; its position seen by the camera moves by the same
        vectors expressed in the camera frame, so their best-fit rotation is
        R_base_camera (Kabsch). Only tag positions are used - PnP rotation of a
        small tag seen head-on is ambiguous, its position is not."""
        log = self.get_logger()
        T_gripper_tcp = self.lookup(self.gripper_frame, self.tcp_link)
        base_points, camera_points = [], []
        for step in np.vstack((np.zeros(3), np.eye(3) * self.bootstrap_step)):
            T_goal = T_base_gripper.copy()
            T_goal[:3, 3] += step
            if step.any():
                joints = self.solve_ik(T_goal @ T_gripper_tcp)
                if joints is None or not self.move_to(joints):
                    log.error(f"bootstrap: cannot move the tag by {step.tolist()} m")
                    return None
                self.wait_still()
            T_cam_tag, _, _ = self.measure_tag()
            if T_cam_tag is None:
                log.error("bootstrap: tag not visible")
                return None
            T_now = self.lookup(self.base_frame, self.gripper_frame)
            base_points.append((T_now @ np.append(self.tag_offset, 1.0))[:3])
            camera_points.append(T_cam_tag[:3, 3])
        A = np.array(base_points[1:]) - base_points[0]
        B = np.array(camera_points[1:]) - camera_points[0]
        U, _, Vt = np.linalg.svd(B.T @ A)
        R = (U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt).T
        fit_mm = 1000 * np.abs(A - B @ R.T).max()
        T = to_matrix(R, base_points[0] - R @ camera_points[0])
        log.info(f"no camera TF yet: rough camera position {np.round(T[:3, 3], 3).tolist()} "
                 f"(displacement fit {fit_mm:.1f} mm)")
        if not self.move_to(self.start_joints):
            log.warn("bootstrap: could not return to the start pose")
        return T

    # ------------------------------------------------------------------ main
    def run(self):
        log = self.get_logger()
        if self.solve_only:
            self.solve(np.load(self.samples_file))
            return
        for name, ready in [("/compute_ik", self.ik_client.wait_for_service(10.0)),
                            ("/move_action", self.move_client.wait_for_server(10.0)),
                            ("/execute_trajectory", self.execute_client.wait_for_server(10.0))]:
            if not ready:
                log.error(f"{name} not available (is move_group running?)")
                return
        while self.rgb is None or self.info is None or self.joints is None:
            time.sleep(0.1)
        optical = self.rgb.header.frame_id
        self.start_joints = start_joints = self.current_arm()
        T_base_gripper_start = self.lookup(self.base_frame, self.gripper_frame)
        if self.center_from_start:
            self.center = (T_base_gripper_start @ np.append(self.tag_offset, 1.0))[:3]
            log.info(f"workspace centre = tag at start pose {np.round(self.center, 3).tolist()}")
        try:
            if self.force_bootstrap:
                raise TransformException("force_bootstrap")
            T_base_optical_guess = self.lookup(self.base_frame, optical)
        except TransformException:
            T_base_optical_guess = self.bootstrap_camera(T_base_gripper_start)
            if T_base_optical_guess is None:
                log.error(f"no camera TF {self.base_frame} -> {optical} and tag {self.tag_id} "
                          "not visible at the start pose: guide the arm so the tag faces the camera")
                return
        camera_position = T_base_optical_guess[:3, 3]
        log.info(f"aiming at camera position {np.round(camera_position, 3).tolist()} "
                 f"({np.linalg.norm(camera_position - self.center):.2f} m from the centre)")
        T_gripper_tcp = self.lookup(self.gripper_frame, self.tcp_link)
        rng = np.random.default_rng(self.seed)
        samples = []
        for i in range(self.num_poses):
            joints, now = None, np.array(self.current_arm())
            for _ in range(self.candidates):
                tag_position = self.center + rng.uniform(-1, 1, 3) * self.spread
                T_base_gripper_goal = self.gripper_pose_for(tag_position, camera_position, rng)
                candidate = self.solve_ik(T_base_gripper_goal @ T_gripper_tcp)
                if candidate is not None and (joints is None or np.abs(
                        np.array(candidate) - now).max() < np.abs(np.array(joints) - now).max()):
                    joints = candidate
            if joints is None:
                log.warn(f"pose {i + 1}: no collision-free IK, skipped")
                continue
            if not self.move_to(joints):
                log.warn(f"pose {i + 1}: motion failed, skipped")
                continue
            self.wait_still()
            T_cam_tag, error, corners = self.measure_tag()
            if T_cam_tag is None:
                log.warn(f"pose {i + 1}: tag {self.tag_id} not seen well, skipped")
                continue
            T_base_gripper = self.lookup(self.base_frame, self.gripper_frame)
            samples.append((T_base_gripper, T_cam_tag, corners))
            log.info(f"pose {i + 1}: sample {len(samples)} (reprojection {error:.2f} px, "
                     f"tag at {np.linalg.norm(T_cam_tag[:3, 3]):.2f} m)")
        np.savez(self.samples_file, base_gripper=[s[0] for s in samples],
                 cam_tag=[s[1] for s in samples], corners=[s[2] for s in samples],
                 K=np.array(self.info.k).reshape(3, 3), D=np.array(self.info.d, float),
                 T_base_optical_guess=T_base_optical_guess,
                 T_optical_camera_base=self.lookup(optical, self.camera_base_frame),
                 optical=optical, tag_size=self.tag_size)
        log.info(f"{len(samples)} samples saved to {self.samples_file}")
        if self.center_from_start:
            if not self.move_to(start_joints):
                log.warn("could not plan back to the start pose")
        elif self.return_to_observe and self.observe_client.wait_for_service(2.0):
            self.wait(self.observe_client.call_async(Trigger.Request()), 90.0)
        if len(samples) < self.min_samples:
            log.error(f"only {len(samples)} usable samples (need {self.min_samples}); "
                      "widen workspace_spread or check that the tag faces the camera")
            return
        self.solve(np.load(self.samples_file))

    def solve(self, data):
        log = self.get_logger()
        samples = list(zip(data["base_gripper"], data["cam_tag"]))
        corners, K, D = data["corners"], data["K"], data["D"]
        T_base_optical_guess = data["T_base_optical_guess"]
        base_to_gripper = [np.linalg.inv(T) for T, _ in samples]
        R_b2g = [T[:3, :3] for T in base_to_gripper]
        t_b2g = [T[:3, 3] for T in base_to_gripper]
        R_t2c = [T[:3, :3] for _, T in samples]
        t_t2c = [T[:3, 3] for _, T in samples]
        best = None
        for name, method in METHODS.items():
            try:
                R, t = cv2.calibrateHandEye(R_b2g, t_b2g, R_t2c, t_t2c, method=method)
            except cv2.error:
                continue
            X = to_matrix(R, t.ravel())  # base -> camera optical
            # Y_i = T_gripper_tag implied by each sample; their spread is the residual.
            Ys = [np.linalg.inv(Tbg) @ X @ Tct for Tbg, Tct in samples]
            spread_mm = 1000.0 * np.linalg.norm(np.std([Y[:3, 3] for Y in Ys], axis=0))
            log.info(f"{name:10s}: camera at {np.round(X[:3, 3], 4).tolist()} m, "
                     f"tag-on-gripper spread {spread_mm:.2f} mm")
            if best is None or spread_mm < best[2]:
                best = (name, X, spread_mm, Ys)
        name, X, spread_mm, Ys = best
        Y = to_matrix(Rotation.from_matrix([y[:3, :3] for y in Ys]).mean().as_matrix(),
                      np.mean([y[:3, 3] for y in Ys], axis=0))
        X, Y, rms_before, rms_after = self.refine(X, Y, samples, corners, K, D)
        log.info(f"reprojection refinement: RMS {rms_before:.2f} px -> {rms_after:.2f} px")
        name += "+reprojection-LSQ"
        T_base_camera = X @ data["T_optical_camera_base"]
        rotation = Rotation.from_matrix(T_base_camera[:3, :3])
        result = {
            "parent_frame": self.base_frame,
            "child_frame": self.camera_base_frame,
            "xyz": [round(float(v), 5) for v in T_base_camera[:3, 3]],
            "rpy": [round(float(v), 5) for v in rotation.as_euler("xyz")],
            "quaternion_xyzw": [round(float(v), 6) for v in rotation.as_quat()],
            "method": name,
            "samples": len(Ys),
            "tag_on_gripper_spread_mm_closed_form": round(float(spread_mm), 2),
            "reprojection_rms_px": round(float(rms_after), 3),
            "tag_in_gripper": {"frame": self.gripper_frame,
                               "xyz": [round(float(v), 4) for v in Y[:3, 3]],
                               "rpy": [round(float(v), 4)
                                       for v in Rotation.from_matrix(Y[:3, :3]).as_euler("xyz")]},
            "calibrated_at": datetime.datetime.now().isoformat(timespec="seconds"),
        }
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
        with open(self.output_file, "w") as stream:
            yaml.safe_dump(result, stream, sort_keys=False)
        delta = np.linalg.inv(T_base_optical_guess) @ X
        log.info(f"HAND-EYE DONE ({name}) -> {self.output_file}")
        log.info(f"  {self.base_frame} -> {self.camera_base_frame}: xyz={result['xyz']} "
                 f"rpy={result['rpy']}")
        log.info(f"  tag on gripper: {result['tag_in_gripper']['xyz']}, "
                 f"reprojection RMS {rms_after:.2f} px over {len(samples)} samples")
        log.info(f"  vs current camera TF: {1000 * np.linalg.norm(delta[:3, 3]):.1f} mm, "
                 f"{np.degrees(Rotation.from_matrix(delta[:3, :3]).magnitude()):.2f} deg")


    def refine(self, X, Y, samples, corners, K, D):
        """Least squares on the tag-corner reprojection error over all samples,
        jointly for X = T_base_optical and Y = T_gripper_tag (6 + 6 parameters)."""
        s = float(self.tag_size) / 2.0
        obj = np.array([[-s, s, 0, 1], [s, s, 0, 1], [s, -s, 0, 1], [-s, -s, 0, 1]], float).T

        def unpack(v):
            return (to_matrix(Rotation.from_rotvec(v[0:3]).as_matrix(), v[3:6]),
                    to_matrix(Rotation.from_rotvec(v[6:9]).as_matrix(), v[9:12]))

        def residuals(v):
            Xv, Yv = unpack(v)
            X_inv = np.linalg.inv(Xv)
            out = []
            for (T_bg, _), seen in zip(samples, corners):
                cam = (X_inv @ T_bg @ Yv @ obj)[:3]
                pix, _ = cv2.projectPoints(cam.T.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, D)
                out.append((pix.reshape(-1, 2) - seen).ravel())
            return np.concatenate(out)

        v0 = np.concatenate([Rotation.from_matrix(X[:3, :3]).as_rotvec(), X[:3, 3],
                             Rotation.from_matrix(Y[:3, :3]).as_rotvec(), Y[:3, 3]])
        rms = lambda v: float(np.sqrt(np.mean(residuals(v) ** 2)))
        fit = least_squares(residuals, v0, loss="soft_l1", f_scale=1.0)
        Xr, Yr = unpack(fit.x)
        return Xr, Yr, rms(v0), rms(fit.x)


def main():
    rclpy.init()
    node = HandEyeCalibration()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
