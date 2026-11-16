#!/usr/bin/env python3
"""Estimate the OAK camera pose (extrinsics) from one AprilTag landmark.

The tag is fixed at a known pose in `landmark_frame` (e.g. measured relative to
the robot). The camera sees the tag, solvePnP gives T_optical_tag, and

    T_landmark_cameraBase = T_landmark_tag * inv(T_optical_tag) * inv(T_cameraBase_optical)

T_cameraBase_optical comes from TF (published by the camera driver / sim), so the
result is the transform to publish as landmark_frame -> camera_base_frame.

A single small tag at ~1 m gives PnP a tilt/depth ambiguity of a few mm. When an
aligned depth image is available, a plane is fitted to the depth points on the
tag: its normal and distance replace the PnP tilt and range, and PnP keeps only
the in-plane rotation.

Tag frame convention (OpenCV IPPE_SQUARE): origin at the tag centre, x to the
tag's right, y to the tag's top, z out of the printed face.
"""
import datetime
import os

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

DICTIONARIES = {
    "36h11": cv2.aruco.DICT_APRILTAG_36h11,
    "25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "16h5": cv2.aruco.DICT_APRILTAG_16h5,
}


def to_matrix(rotation, translation):
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


def transform_to_matrix(transform):
    q = transform.rotation
    t = transform.translation
    return to_matrix(Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(), [t.x, t.y, t.z])


class AprilTagCameraCalibration(Node):
    def __init__(self):
        super().__init__("camera_calibration_apriltag")
        p = self.declare_parameter
        self.image_topic = p("image_topic", "/oak/rgb/image_raw").value
        self.info_topic = p("camera_info_topic", "/oak/rgb/camera_info").value
        self.depth_topic = p("depth_topic", "/oak/stereo/image_raw").value
        self.use_depth = p("use_depth_refinement", True).value
        self.family = p("tag_family", "36h11").value
        self.tag_id = p("tag_id", 0).value
        self.tag_size = p("tag_size", 0.12).value
        self.landmark_frame = p("landmark_frame", "world").value
        landmark_xyz = p("landmark_xyz", [0.0, 0.0, 0.0]).value
        landmark_rpy = p("landmark_rpy", [0.0, 0.0, 0.0]).value
        self.camera_base_frame = p("camera_base_frame", "oak-d-base-frame").value
        self.samples = p("samples", 30).value
        self.max_reprojection_px = p("max_reprojection_error_px", 1.5).value
        self.output_file = os.path.expanduser(
            p("output_file", "~/.ros/fr3_oak_extrinsics.yaml").value)

        self.T_landmark_tag = to_matrix(
            Rotation.from_euler("xyz", landmark_rpy).as_matrix(), landmark_xyz)
        s = self.tag_size / 2.0
        self.object_points = np.array(
            [[-s, s, 0.0], [s, s, 0.0], [s, -s, 0.0], [-s, -s, 0.0]], dtype=np.float64)
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(DICTIONARIES[self.family]),
            self._detector_parameters())

        self.camera_info = None
        self.depth = None
        self.optical_frame = None
        self.poses = []  # T_optical_tag per accepted frame
        self.errors = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, self.info_topic, self.on_info, 1)
        self.create_subscription(Image, self.image_topic, self.on_image, 1)
        if self.use_depth:
            self.create_subscription(Image, self.depth_topic, self.on_depth, 1)
        self.get_logger().info(
            f"Looking for tag{self.family} id {self.tag_id} ({self.tag_size} m) on "
            f"{self.image_topic}; landmark in '{self.landmark_frame}' at "
            f"xyz={landmark_xyz} rpy={landmark_rpy}")

    @staticmethod
    def _detector_parameters():
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        return parameters

    def on_info(self, msg):
        self.camera_info = msg

    def on_depth(self, msg):
        # Depth aligned to RGB: Gazebo 32FC1 metres, depthai-ros 16UC1 millimetres.
        if msg.encoding == "32FC1":
            depth = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
        elif msg.encoding == "16UC1":
            depth = np.frombuffer(msg.data, np.uint16).reshape(
                msg.height, msg.width).astype(np.float32) / 1000.0
        else:
            return
        self.depth = depth

    def refine_with_depth(self, T_optical_tag, image_points, K):
        """Replace tilt and range of the PnP pose with a plane fitted to depth."""
        if self.depth is None or self.depth.shape != (
                self.camera_info.height, self.camera_info.width):
            return None
        mask = np.zeros(self.depth.shape, np.uint8)
        cv2.fillConvexPoly(mask, image_points.astype(np.int32), 1)
        mask = cv2.erode(mask, np.ones((5, 5), np.uint8))  # stay off the tag edge
        v, u = np.nonzero(mask)
        z = self.depth[v, u]
        valid = np.isfinite(z) & (z > 0.0)
        if valid.sum() < 200:
            return None
        u, v, z = u[valid], v[valid], z[valid]
        points = np.column_stack(
            ((u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z))
        centroid = points.mean(axis=0)
        normal = np.linalg.svd(points - centroid)[2][2]
        if normal @ centroid > 0.0:  # tag z axis points back toward the camera
            normal = -normal
        # Tilt: smallest rotation taking the PnP tag z axis onto the fitted normal.
        z_pnp = T_optical_tag[:3, 2]
        axis = np.cross(z_pnp, normal)
        angle = np.arctan2(np.linalg.norm(axis), z_pnp @ normal)
        tilt = Rotation.from_rotvec(axis / np.linalg.norm(axis) * angle) \
            if np.linalg.norm(axis) > 1e-9 else Rotation.identity()
        rotation = tilt.as_matrix() @ T_optical_tag[:3, :3]
        # Range: intersect the ray through the PnP tag centre with the fitted plane.
        ray = T_optical_tag[:3, 3] / np.linalg.norm(T_optical_tag[:3, 3])
        centre = ray * (normal @ centroid) / (normal @ ray)
        return to_matrix(rotation, centre), np.degrees(angle)

    def on_image(self, msg):
        if self.camera_info is None or len(self.poses) >= self.samples:
            return
        self.optical_frame = msg.header.frame_id
        gray = self._to_gray(msg)
        if gray is None:
            return
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None or self.tag_id not in ids.ravel():
            self.get_logger().warn(
                f"tag id {self.tag_id} not visible", throttle_duration_sec=2.0)
            return
        image_points = corners[list(ids.ravel()).index(self.tag_id)][0].astype(np.float64)
        K = np.array(self.camera_info.k).reshape(3, 3)
        D = np.array(self.camera_info.d, dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            self.object_points, image_points, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return
        projected, _ = cv2.projectPoints(self.object_points, rvec, tvec, K, D)
        error = float(np.linalg.norm(projected.reshape(-1, 2) - image_points, axis=1).mean())
        if error > self.max_reprojection_px:
            self.get_logger().warn(f"rejecting frame, reprojection error {error:.2f} px")
            return
        T_optical_tag = to_matrix(Rotation.from_rotvec(rvec.ravel()).as_matrix(), tvec.ravel())
        if self.use_depth:
            refined = self.refine_with_depth(T_optical_tag, image_points, K)
            if refined is None:
                self.get_logger().warn(
                    "no usable depth on the tag yet", throttle_duration_sec=2.0)
                return
            T_optical_tag, self.last_tilt_correction = refined
        self.poses.append(T_optical_tag)
        self.errors.append(error)
        if len(self.poses) == self.samples:
            self.finish()

    @staticmethod
    def _to_gray(msg):
        data = np.frombuffer(msg.data, np.uint8)
        if msg.encoding in ("rgb8", "bgr8"):
            image = data.reshape(msg.height, msg.step)[:, : msg.width * 3].reshape(
                msg.height, msg.width, 3)
            code = cv2.COLOR_RGB2GRAY if msg.encoding == "rgb8" else cv2.COLOR_BGR2GRAY
            return cv2.cvtColor(image, code)
        if msg.encoding == "mono8":
            return data.reshape(msg.height, msg.step)[:, : msg.width]
        return None

    def finish(self):
        try:
            T_base_optical = transform_to_matrix(self.tf_buffer.lookup_transform(
                self.camera_base_frame, self.optical_frame, rclpy.time.Time()).transform)
        except TransformException as error:
            self.get_logger().error(
                f"need TF {self.camera_base_frame} -> {self.optical_frame}: {error}")
            self.poses.clear()
            return

        translations = np.array([pose[:3, 3] for pose in self.poses])
        rotation = Rotation.from_matrix([pose[:3, :3] for pose in self.poses]).mean()
        T_optical_tag = to_matrix(rotation.as_matrix(), translations.mean(axis=0))
        T_landmark_base = (
            self.T_landmark_tag @ np.linalg.inv(T_optical_tag) @ np.linalg.inv(T_base_optical))

        xyz = T_landmark_base[:3, 3]
        base_rotation = Rotation.from_matrix(T_landmark_base[:3, :3])
        result = {
            "parent_frame": self.landmark_frame,
            "child_frame": self.camera_base_frame,
            "xyz": [round(float(v), 5) for v in xyz],
            "rpy": [round(float(v), 5) for v in base_rotation.as_euler("xyz")],
            "quaternion_xyzw": [round(float(v), 6) for v in base_rotation.as_quat()],
            "tag": {"family": self.family, "id": self.tag_id, "size": self.tag_size},
            "samples": len(self.poses),
            "mean_reprojection_error_px": round(float(np.mean(self.errors)), 3),
            "tag_distance_m": round(float(np.linalg.norm(T_optical_tag[:3, 3])), 3),
            "depth_refinement": bool(self.use_depth),
            "tag_translation_std_mm": [round(float(v) * 1000.0, 2)
                                       for v in translations.std(axis=0)],
            "calibrated_at": datetime.datetime.now().isoformat(timespec="seconds"),
        }
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
        with open(self.output_file, "w") as stream:
            yaml.safe_dump(result, stream, sort_keys=False)

        log = self.get_logger()
        log.info(f"CALIBRATION DONE -> {self.output_file}")
        log.info(f"  {self.landmark_frame} -> {self.camera_base_frame}: "
                 f"xyz={result['xyz']} rpy={result['rpy']}")
        if self.use_depth:
            log.info(f"  depth plane corrected PnP tilt by {self.last_tilt_correction:.2f} deg")
        log.info(f"  reprojection {result['mean_reprojection_error_px']} px, tag at "
                 f"{result['tag_distance_m']} m, std {result['tag_translation_std_mm']} mm")
        self.compare_with_current_tf(T_landmark_base)
        log.info("Restart the camera TF (simulation: run_all.sh) to use the new extrinsics.")
        raise SystemExit

    def compare_with_current_tf(self, T_estimated):
        """Report the difference to the TF currently published (in simulation this
        is the nominal pose, i.e. ground truth)."""
        try:
            T_current = transform_to_matrix(self.tf_buffer.lookup_transform(
                self.landmark_frame, self.camera_base_frame, rclpy.time.Time()).transform)
        except TransformException:
            return
        delta = np.linalg.inv(T_current) @ T_estimated
        angle = np.degrees(Rotation.from_matrix(delta[:3, :3]).magnitude())
        self.get_logger().info(
            f"  vs current TF: position {np.linalg.norm(delta[:3, 3]) * 1000.0:.1f} mm, "
            f"rotation {angle:.2f} deg")


def main():
    rclpy.init()
    node = AprilTagCameraCalibration()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
