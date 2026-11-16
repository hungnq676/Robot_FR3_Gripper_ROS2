#!/usr/bin/env python3
"""Layer-1 tabletop perception: find objects on the known table from RGB-D.

Pipeline per update:
  depth (aligned to RGB, median filtered) -> 3D points in the world frame
  -> fit the tabletop plane near its known height; heights are measured from the
     fitted plane, which absorbs depth noise bias and small calibration tilt
  -> keep the volume above the tabletop
  -> remove points on the robot (its collision meshes placed with live TF)
  -> voxelise and split into connected clusters (one cluster = one object)
  -> per cluster: footprint rectangle, height, box/cylinder, colour
  -> track ids over time (a new object must be seen min_confirmations times in
     a row), publish PerceivedObjectArray + markers
  Updates are skipped while the robot moves: depth and robot TF are then out of
  step, the self-filter misses arm edges and they would show up as objects.
  -> keep the MoveIt planning scene in sync (add / move / remove obstacles)

Only the table is configured; everything else comes from the camera.
"""
import os
import struct
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from fairino_fr3_msgs.msg import PerceivedObject, PerceivedObjectArray
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy import ndimage
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def transform_to_matrix(transform):
    q, t = transform.rotation, transform.translation
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


def read_stl(path):
    data = open(path, "rb").read()
    if data[:5] == b"solid" and b"facet" in data[:512]:
        vertices = [list(map(float, line.split()[1:4]))
                    for line in data.decode(errors="ignore").splitlines()
                    if line.strip().startswith("vertex")]
        return np.array(vertices, dtype=np.float64).reshape(-1, 3, 3)
    count = struct.unpack("<I", data[80:84])[0]
    records = np.frombuffer(data[84:84 + count * 50], dtype=np.dtype(
        [("normal", "<3f4"), ("vertices", "<9f4"), ("attribute", "<u2")]))
    return records["vertices"].reshape(-1, 3, 3).astype(np.float64)


def box_triangles(size):
    """12 triangles of an axis-aligned box centred at the origin."""
    hx, hy, hz = np.asarray(size) / 2.0
    c = np.array([[sx * hx, sy * hy, sz * hz]
                  for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    faces = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1),
             (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    return np.array([[c[a], c[b], c[d]] for a, b, d in faces])


def sample_triangles(triangles, spacing):
    """Roughly uniform surface samples, about one per spacing^2 of area."""
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    areas = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    counts = np.random.default_rng(0).poisson(areas / spacing ** 2)
    index = np.repeat(np.arange(len(triangles)), counts)
    r1 = np.sqrt(np.random.default_rng(1).random(len(index)))
    r2 = np.random.default_rng(2).random(len(index))
    samples = ((1 - r1)[:, None] * a[index] + (r1 * (1 - r2))[:, None] * b[index]
               + (r1 * r2)[:, None] * c[index])
    return np.vstack([samples, triangles.reshape(-1, 3)])


def origin_matrix(element):
    matrix = np.eye(4)
    origin = element.find("origin") if element is not None else None
    if origin is not None:
        xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
        matrix[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
        matrix[:3, 3] = xyz
    return matrix


def resolve_package_uri(uri):
    if uri.startswith("package://"):
        package, _, relative = uri[len("package://"):].partition("/")
        return os.path.join(get_package_share_directory(package), relative)
    return uri[len("file://"):] if uri.startswith("file://") else uri


COLOR_NAMES = [  # (upper hue limit in degrees, name)
    (15, "red"), (40, "orange"), (70, "yellow"), (160, "green"),
    (200, "cyan"), (260, "blue"), (300, "purple"), (345, "pink"), (360, "red")]


def color_name(rgb):
    hsv = cv2.cvtColor(np.uint8([[rgb]]), cv2.COLOR_RGB2HSV_FULL)[0, 0] / 255.0
    hue, saturation, value = hsv[0] * 360.0, hsv[1], hsv[2]
    if saturation < 0.25:
        return "white" if value > 0.75 else "black" if value < 0.25 else "gray"
    return next((name for limit, name in COLOR_NAMES if hue < limit), "red")


def fit_circle(xy):
    """Least-squares (Kasa) circle through 2D points -> (cx, cy, r)."""
    A = np.column_stack((2 * xy[:, 0], 2 * xy[:, 1], np.ones(len(xy))))
    b = (xy ** 2).sum(axis=1)
    cx, cy, c = np.linalg.lstsq(A, b, rcond=None)[0]
    return cx, cy, np.sqrt(max(c + cx ** 2 + cy ** 2, 0.0))


def smooth(history):
    """Robust average of recent measurements of one shape: median position and
    size, circular mean of yaw (period 180 deg, a box looks the same turned
    over). Single frames jitter by several mm and up to ~7 deg, which matters
    for long objects that only just fit between the jaws."""
    result = dict(history[-1])
    result["position"] = np.median([m["position"] for m in history], axis=0)
    result["dimensions"] = np.median([m["dimensions"] for m in history], axis=0).tolist()
    result["grasp_width"] = float(np.median([m["grasp_width"] for m in history]))
    # A nearly square footprint swaps its long/short sides between frames
    # (yaw jumps 90 deg): average with a 90 deg period there.
    dims = result["dimensions"]
    period_factor = 4.0 if result["shape"] == "box" and dims[1] > 0.85 * dims[0] else 2.0
    angles = np.array([period_factor * m["yaw"] for m in history])
    yaw = np.arctan2(np.sin(angles).mean(), np.cos(angles).mean()) / period_factor
    result["yaw"] = float((yaw + np.pi / 2) % np.pi - np.pi / 2)
    return result


class TrackedObject:
    def __init__(self, object_id, measurement, stamp, history_length=10):
        self.id = object_id
        self.last_seen = stamp
        self.published = None  # measurement last sent to MoveIt
        self.shape_votes = {}
        self.histories = {}
        self.history_length = history_length
        self.vote(measurement)

    def vote(self, measurement):
        """Single views of round objects can look box-like; the shape follows the
        majority over recent frames, and its measurements are averaged."""
        shape = measurement["shape"]
        self.shape_votes[shape] = self.shape_votes.get(shape, 0) + 1
        history = self.histories.setdefault(shape, [])
        history.append(measurement)
        del history[:-self.history_length]
        best = max(self.shape_votes, key=self.shape_votes.get)
        self.measurement = smooth(self.histories[best])


class TabletopPerception(Node):
    def __init__(self):
        super().__init__("tabletop_perception")
        p = self.declare_parameter
        self.depth_topic = p("depth_topic", "/oak/stereo/image_raw").value
        self.rgb_topic = p("rgb_topic", "/oak/rgb/image_raw").value
        self.info_topic = p("camera_info_topic", "/oak/rgb/camera_info").value
        self.world_frame = p("world_frame", "world").value
        self.table_top_z = p("table.top_z", 1.0).value
        self.table_x = p("table.x_range", [0.0, 1.5]).value
        self.table_y = p("table.y_range", [0.0, 1.0]).value
        self.edge_margin = p("table.edge_margin", 0.02).value
        # Optional table outline [x1, y1, x2, y2, ...] in the world frame for a
        # table that is not aligned with the world axes; replaces x/y_range.
        polygon = p("table.polygon", [0.0]).value
        self.table_polygon = np.reshape(polygon, (-1, 2)) if len(polygon) >= 6 else None
        # Stereo disparity bias (was fitted for the OAK): z_true = 1 / (1/z_measured + k), k in 1/m.
        self.depth_bias_k = p("depth_bias_k", 0.0).value
        self.min_height = p("min_height", 0.008).value
        self.max_height = p("max_height", 0.40).value
        self.depth_range = p("depth_range", [0.2, 3.0]).value
        self.pixel_stride = p("pixel_stride", 2).value
        self.median_kernel = p("depth_median_kernel", 5).value
        self.plane_search = p("table.plane_search", 0.03).value
        self.voxel = p("voxel_size", 0.005).value
        self.min_voxels = p("min_cluster_voxels", 25).value
        self.cylinder_fill_ratio = p("cylinder_fill_ratio", 0.9).value
        self.top_face_band = p("top_face_band", 0.01).value
        self.robot_filter_distance = p("robot_filter_distance", 0.02).value
        self.robot_sample_spacing = p("robot_sample_spacing", 0.01).value
        self.carried_object_radius = p("carried_object_radius", 0.09).value
        self.tcp_link = p("tcp_link", "grasp_frame").value
        self.max_opening = p("gripper_max_opening", 0.068).value
        self.grasp_margin = p("grasp_margin", 0.006).value
        self.match_distance = p("match_distance", 0.04).value
        self.forget_after = p("forget_after", 2.0).value
        self.min_confirmations = p("min_confirmations", 3).value
        self.max_joint_speed = p("max_joint_speed", 0.01).value
        self.still_time = p("still_time", 0.5).value
        self.position_threshold = p("scene_update_position", 0.005).value
        self.yaw_threshold = p("scene_update_yaw", 0.05).value
        self.size_threshold = p("scene_update_size", 0.006).value
        self.padding = p("collision_padding", 0.005).value
        update_rate = p("update_rate", 2.0).value

        self.info = None
        self.depth_msg = None
        self.rgb_msg = None
        self.robot_samples = {}  # link -> Nx4 homogeneous samples in the link frame
        self.tracked = {}
        self.next_id = 1
        self.attached_ids = set()
        self.candidates = []  # [measurement, consecutive sightings]
        self.last_motion = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, self.info_topic, self.on_info, 1)
        self.create_subscription(Image, self.depth_topic, self.on_depth, 1)
        self.create_subscription(Image, self.rgb_topic, self.on_rgb, 1)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/robot_description", self.on_robot_description, latched)
        self.objects_pub = self.create_publisher(
            PerceivedObjectArray, "/fr3_demo/perceived_objects", 1)
        self.marker_pub = self.create_publisher(MarkerArray, "/fr3_demo/perceived_markers", 1)
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.scene_client = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.create_timer(1.0 / update_rate, self.update)
        self.get_logger().info(
            f"Perceiving objects on table z={self.table_top_z} from {self.depth_topic}")

    # ------------------------------------------------------------------ inputs
    def on_info(self, msg):
        self.info = msg

    def on_depth(self, msg):
        self.depth_msg = msg

    def on_rgb(self, msg):
        self.rgb_msg = msg

    def on_joint_states(self, msg):
        if msg.velocity and max(abs(v) for v in msg.velocity) > self.max_joint_speed:
            self.last_motion = self.get_clock().now()

    def robot_is_still(self):
        if self.last_motion is None:
            return True
        return (self.get_clock().now() - self.last_motion).nanoseconds * 1e-9 > self.still_time

    def on_robot_description(self, msg):
        root = ET.fromstring(msg.data)
        total = 0
        for link in root.findall("link"):
            triangles = []
            for collision in link.findall("collision"):
                geometry = collision.find("geometry")
                mesh = geometry.find("mesh") if geometry is not None else None
                box = geometry.find("box") if geometry is not None else None
                if mesh is not None:
                    mesh_triangles = read_stl(resolve_package_uri(mesh.get("filename")))
                    scale = [float(v) for v in mesh.get("scale", "1 1 1").split()]
                    mesh_triangles = mesh_triangles * np.array(scale)
                elif box is not None:
                    mesh_triangles = box_triangles([float(v) for v in box.get("size").split()])
                else:
                    continue
                origin = origin_matrix(collision)
                flat = mesh_triangles.reshape(-1, 3) @ origin[:3, :3].T + origin[:3, 3]
                triangles.append(flat.reshape(-1, 3, 3))
            if triangles:
                samples = sample_triangles(np.vstack(triangles), self.robot_sample_spacing)
                # one sample per robot_sample_spacing voxel is plenty for the filter
                _, unique = np.unique(np.floor(samples / self.robot_sample_spacing),
                                      axis=0, return_index=True)
                samples = samples[unique]
                self.robot_samples[link.get("name")] = np.hstack(
                    [samples, np.ones((len(samples), 1))])
                total += len(samples)
        self.get_logger().info(
            f"Robot self-filter: {len(self.robot_samples)} links, {total} surface samples")

    # -------------------------------------------------------------- main loop
    def update(self):
        if self.info is None or self.depth_msg is None or self.rgb_msg is None:
            return
        if not self.robot_is_still():
            # Keep the current objects (and the MoveIt scene) as they are.
            self.publish(self.depth_msg.header.stamp)
            return
        depth_msg, rgb_msg = self.depth_msg, self.rgb_msg
        stamp = rclpy.time.Time.from_msg(depth_msg.header.stamp)
        try:
            T_world_camera = transform_to_matrix(self.tf_buffer.lookup_transform(
                self.world_frame, depth_msg.header.frame_id, stamp,
                timeout=Duration(seconds=0.2)).transform)
        except TransformException as error:
            self.get_logger().warn(f"camera TF unavailable: {error}", throttle_duration_sec=5.0)
            return
        self.request_attached_objects()

        depth = self.decode_depth(depth_msg)
        rgb = np.frombuffer(rgb_msg.data, np.uint8).reshape(
            rgb_msg.height, rgb_msg.step)[:, : rgb_msg.width * 3].reshape(
            rgb_msg.height, rgb_msg.width, 3)
        if rgb_msg.encoding == "bgr8":  # the OAK driver published BGR (the Astra sends rgb8)
            rgb = rgb[:, :, ::-1]
        if depth is None or depth.shape != rgb.shape[:2]:
            self.get_logger().warn("depth must be aligned to RGB", throttle_duration_sec=5.0)
            return

        points, pixels = self.depth_to_world(depth, T_world_camera)
        points, pixels = self.crop_to_table(points, pixels)
        if len(points) == 0:
            return
        heights = points[:, 2] - self.table_plane_z(points)
        keep = (heights > self.min_height) & (heights < self.max_height)
        points, pixels, heights = points[keep], pixels[keep], heights[keep]
        keep = self.not_robot(points, stamp)
        points, pixels, heights = points[keep], pixels[keep], heights[keep]

        measurements = [m for m in (self.measure(points[i], pixels[i], heights[i], rgb)
                                    for i in self.cluster(points)) if m is not None]
        self.track(measurements, depth, T_world_camera)
        self.publish(depth_msg.header.stamp)

    def decode_depth(self, msg):
        if msg.encoding == "32FC1":
            depth = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
        elif msg.encoding == "16UC1":
            depth = np.frombuffer(msg.data, np.uint16).reshape(
                msg.height, msg.width).astype(np.float32) / 1000.0
        else:
            self.get_logger().error(f"unsupported depth encoding {msg.encoding}", once=True)
            return None
        depth = np.where(np.isfinite(depth), depth, 0.0).astype(np.float32)
        if self.depth_bias_k:
            depth = depth / (1.0 + self.depth_bias_k * depth)
        return cv2.medianBlur(depth, self.median_kernel) if self.median_kernel > 1 else depth

    def table_plane_z(self, points):
        """Height of the fitted tabletop plane under each point.

        Least squares z = a*x + b*y + c on points near the nominal table height,
        refitted once without outliers (object bottoms, noise)."""
        near = np.abs(points[:, 2] - self.table_top_z) < self.plane_search
        if near.sum() < 500:
            return np.full(len(points), self.table_top_z)
        A = np.column_stack((points[near, 0], points[near, 1], np.ones(near.sum())))
        z = points[near, 2]
        coefficients = np.linalg.lstsq(A, z, rcond=None)[0]
        residual = z - A @ coefficients
        inliers = np.abs(residual) < 2.5 * max(residual.std(), 1e-4)
        coefficients = np.linalg.lstsq(A[inliers], z[inliers], rcond=None)[0]
        self.plane = coefficients
        return points[:, 0] * coefficients[0] + points[:, 1] * coefficients[1] + coefficients[2]

    def depth_to_world(self, depth, T_world_camera):
        s = self.pixel_stride
        v, u = np.mgrid[0:depth.shape[0]:s, 0:depth.shape[1]:s]
        z = depth[v, u]
        valid = np.isfinite(z) & (z > self.depth_range[0]) & (z < self.depth_range[1])
        u, v, z = u[valid], v[valid], z[valid]
        K = np.array(self.info.k).reshape(3, 3)
        camera_points = np.column_stack(
            ((u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z))
        world = camera_points @ T_world_camera[:3, :3].T + T_world_camera[:3, 3]
        return world, np.column_stack((u, v))

    def crop_to_table(self, points, pixels):
        m = self.edge_margin
        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        if self.table_polygon is None:
            inside = ((x > self.table_x[0] + m) & (x < self.table_x[1] - m)
                      & (y > self.table_y[0] + m) & (y < self.table_y[1] - m))
        else:
            inside = self.inside_polygon(points[:, :2], m)
        keep = (inside
                & (z > self.table_top_z - self.plane_search)
                & (z < self.table_top_z + self.max_height))
        return points[keep], pixels[keep]

    def inside_polygon(self, xy, margin):
        """Points at least margin inside the convex table outline (either winding)."""
        poly = self.table_polygon
        sign = np.sign(np.sum(poly[:, 0] * np.roll(poly[:, 1], -1) - np.roll(poly[:, 0], -1) * poly[:, 1]))
        keep = np.ones(len(xy), bool)
        for a, b in zip(poly, np.roll(poly, -1, axis=0)):
            edge = b - a
            cross = edge[0] * (xy[:, 1] - a[1]) - edge[1] * (xy[:, 0] - a[0])
            keep &= sign * cross > margin * np.linalg.norm(edge)
        return keep

    def not_robot(self, points, stamp):
        """Mask of points farther than robot_filter_distance from the robot surface
        and outside the carried-object sphere around the TCP."""
        keep = np.ones(len(points), bool)
        if len(points) == 0 or not self.robot_samples:
            return keep
        robot = []
        for link, samples in self.robot_samples.items():
            T = self.lookup(link, stamp)
            if T is None:
                self.get_logger().warn(f"no TF for robot link {link}", throttle_duration_sec=5.0)
                continue
            robot.append(samples @ T.T)
        if robot:
            distance, _ = cKDTree(np.vstack(robot)[:, :3]).query(
                points, distance_upper_bound=self.robot_filter_distance)
            keep &= ~np.isfinite(distance)
        tcp = self.lookup(self.tcp_link, stamp)
        if tcp is not None:
            keep &= np.linalg.norm(points - tcp[:3, 3], axis=1) > self.carried_object_radius
        return keep

    def lookup(self, frame, stamp):
        """world <- frame at the image time, else the latest available."""
        for time in (stamp, rclpy.time.Time()):
            try:
                return transform_to_matrix(self.tf_buffer.lookup_transform(
                    self.world_frame, frame, time, timeout=Duration(seconds=0.05)).transform)
            except TransformException:
                continue
        return None

    def cluster(self, points):
        """Connected components of the occupied voxel grid; yields point indices."""
        if len(points) == 0:
            return
        keys = np.floor(points / self.voxel).astype(np.int64)
        keys -= keys.min(axis=0)
        grid = np.zeros(keys.max(axis=0) + 1, dtype=bool)
        grid[tuple(keys.T)] = True
        labels, count = ndimage.label(grid, structure=np.ones((3, 3, 3)))
        point_labels = labels[tuple(keys.T)]
        voxel_counts = ndimage.sum(grid, labels, index=np.arange(1, count + 1))
        for label, voxels in enumerate(voxel_counts, start=1):
            if voxels >= self.min_voxels:
                yield np.nonzero(point_labels == label)[0]

    def measure(self, points, pixels, heights, rgb):
        height = np.percentile(heights, 98)
        if height < self.min_height:
            return None
        base_z = np.median(points[:, 2] - heights)  # table plane under the object
        xy = points[:, :2].astype(np.float32)
        (cx, cy), (w, h), angle = cv2.minAreaRect(xy)
        hull_area = cv2.contourArea(cv2.convexHull(xy))
        fill = hull_area / max(w * h, 1e-9)
        yaw = np.radians(angle)
        if w < h:  # x axis along the longer side
            w, h, yaw = h, w, yaw + np.pi / 2
        yaw = (yaw + np.pi / 2) % np.pi - np.pi / 2
        if fill < self.cylinder_fill_ratio and h / w > 0.6:
            shape = "cylinder"
            # Only the near half of a cylinder is visible; the hull's arc still
            # fixes the full circle, while a bounding rectangle would shrink it.
            hull = cv2.convexHull(xy).reshape(-1, 2).astype(np.float64)
            cx, cy, radius = fit_circle(hull)
            radius = max(radius, 0.5 * w)
            dimensions = [height, radius]
            grasp_width = 2 * radius
            yaw = 0.0
        else:
            shape = "box"
            # The top face is seen nearly head-on, so its depth noise hardly moves
            # points sideways; side-wall points smear the footprint by a few mm.
            # A domed top (mouse, rounded lid) has a top band smaller than the
            # footprint; then keep the all-points rectangle (never undersize).
            top = xy[heights > height - self.top_face_band]
            if len(top) >= 20:
                (tcx, tcy), (tw, th), tangle = cv2.minAreaRect(top)
                if tw * th >= 0.8 * w * h:
                    cx, cy, w, h, yaw = tcx, tcy, tw, th, np.radians(tangle)
                    if w < h:
                        w, h, yaw = h, w, yaw + np.pi / 2
                    yaw = (yaw + np.pi / 2) % np.pi - np.pi / 2
            dimensions = [w, h, height]
            grasp_width = h
        colour = np.median(rgb[pixels[:, 1], pixels[:, 0]], axis=0)
        return {
            "shape": shape, "dimensions": dimensions, "grasp_width": float(grasp_width),
            "position": np.array([cx, cy, base_z + height / 2.0]),
            "yaw": float(yaw), "rgb": colour, "color": color_name(colour.astype(np.uint8)),
            "num_points": len(points), "fill": float(fill),
        }

    # --------------------------------------------------------------- tracking
    def track(self, measurements, depth, T_world_camera):
        now = self.get_clock().now()
        unmatched = set(self.tracked)
        new_candidates = []
        for m in sorted(measurements, key=lambda m: -m["num_points"]):
            candidates = [(np.linalg.norm(self.tracked[i].measurement["position"][:2]
                                          - m["position"][:2]), i) for i in unmatched]
            distance, best = min(candidates, default=(np.inf, None))
            if distance < self.match_distance:
                self.tracked[best].vote(m)
                self.tracked[best].last_seen = now
                unmatched.discard(best)
            else:
                new_candidates.append(m)
        self.confirm_candidates(new_candidates, now)
        for object_id in unmatched:
            tracked = self.tracked[object_id]
            if object_id in self.attached_ids or self.is_occluded(tracked, depth, T_world_camera):
                tracked.last_seen = now  # held by the robot or hidden behind it: keep
            elif (now - tracked.last_seen).nanoseconds * 1e-9 > self.forget_after:
                self.get_logger().info(f"{object_id} disappeared")
                self.remove_from_scene(object_id)
                del self.tracked[object_id]

    def confirm_candidates(self, measurements, now):
        """An unmatched cluster becomes an object only after it has been seen in
        min_confirmations consecutive updates (filters one-off noise / arm edges)."""
        previous, self.candidates = self.candidates, []
        for m in measurements:
            count = 1
            for old, old_count in previous:
                if np.linalg.norm(old["position"][:2] - m["position"][:2]) < self.match_distance:
                    count = old_count + 1
                    break
            if count < self.min_confirmations:
                self.candidates.append((m, count))
                continue
            object_id = f"obj_{self.next_id}"
            self.next_id += 1
            self.tracked[object_id] = TrackedObject(object_id, m, now)
            self.get_logger().info(
                f"new {object_id}: {m['color']} {m['shape']} "
                f"{np.round(m['dimensions'], 3).tolist()} at "
                f"{np.round(m['position'], 3).tolist()} (fill {m['fill']:.2f})")

    def is_occluded(self, tracked, depth, T_world_camera):
        """True if something is in front of where the object's top should be."""
        m = tracked.measurement
        top = m["position"] + [0.0, 0.0, (m["dimensions"][-1] if m["shape"] == "box"
                                          else m["dimensions"][0]) / 2.0]
        camera = np.linalg.inv(T_world_camera) @ np.append(top, 1.0)
        if camera[2] <= 0:
            return False
        K = np.array(self.info.k).reshape(3, 3)
        u, v = int(K[0, 0] * camera[0] / camera[2] + K[0, 2]), int(
            K[1, 1] * camera[1] / camera[2] + K[1, 2])
        if not (0 <= u < depth.shape[1] and 0 <= v < depth.shape[0]):
            return True  # out of view: cannot tell, keep it
        measured = depth[v, u]
        return bool(np.isfinite(measured) and measured < camera[2] - 0.03)

    def request_attached_objects(self):
        if not self.scene_client.service_is_ready():
            return
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        self.scene_client.call_async(request).add_done_callback(self.on_attached_objects)

    def on_attached_objects(self, future):
        try:
            attached = future.result().scene.robot_state.attached_collision_objects
        except Exception:
            return
        self.attached_ids = {a.object.id for a in attached}

    # ---------------------------------------------------------------- outputs
    def object_pose(self, m):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = m["position"].tolist()
        q = Rotation.from_euler("z", m["yaw"]).as_quat()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
        return pose

    def changed(self, tracked):
        old, new = tracked.published, tracked.measurement
        if old is None:
            return True
        yaw_delta = abs((new["yaw"] - old["yaw"] + np.pi / 2) % np.pi - np.pi / 2)
        return (np.linalg.norm(new["position"] - old["position"]) > self.position_threshold
                or yaw_delta > self.yaw_threshold
                or np.max(np.abs(np.subtract(new["dimensions"], old["dimensions"])))
                > self.size_threshold)

    def publish(self, stamp):
        scene = PlanningScene(is_diff=True)
        objects = PerceivedObjectArray()
        objects.header.stamp = stamp
        objects.header.frame_id = self.world_frame
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for tracked in sorted(self.tracked.values(), key=lambda t: int(t.id.split("_")[1])):
            m = tracked.measurement
            message = self.to_message(tracked, objects.header)
            objects.objects.append(message)
            markers.markers.extend(self.to_markers(message, len(markers.markers)))
            if tracked.id not in self.attached_ids and self.changed(tracked):
                scene.world.collision_objects.append(self.to_collision_object(message))
                tracked.published = m
        self.objects_pub.publish(objects)
        self.marker_pub.publish(markers)
        if scene.world.collision_objects:
            self.scene_pub.publish(scene)

    def to_message(self, tracked, header):
        m = tracked.measurement
        message = PerceivedObject()
        message.id = tracked.id
        message.shape = m["shape"]
        message.shape_type = (PerceivedObject.SHAPE_BOX if m["shape"] == "box"
                              else PerceivedObject.SHAPE_CYLINDER)
        message.color = m["color"]
        message.rgb = ColorRGBA(r=float(m["rgb"][0]) / 255.0, g=float(m["rgb"][1]) / 255.0,
                                b=float(m["rgb"][2]) / 255.0, a=1.0)
        message.pose.header = header
        message.pose.pose = self.object_pose(m)
        message.dimensions = [float(d) for d in m["dimensions"]]
        message.grasp_width = m["grasp_width"]
        message.graspable = m["grasp_width"] <= self.max_opening - self.grasp_margin
        message.num_points = m["num_points"]
        return message

    def to_collision_object(self, message):
        primitive = SolidPrimitive()
        if message.shape == "box":
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [d + 2 * self.padding for d in message.dimensions]
        else:
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [message.dimensions[0] + 2 * self.padding,
                                    message.dimensions[1] + self.padding]
        collision = CollisionObject()
        collision.header = message.pose.header
        collision.id = message.id
        collision.primitives = [primitive]
        collision.primitive_poses = [message.pose.pose]
        collision.operation = CollisionObject.ADD
        return collision

    def remove_from_scene(self, object_id):
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [
            CollisionObject(id=object_id, operation=CollisionObject.REMOVE)]
        scene.world.collision_objects[0].header.frame_id = self.world_frame
        self.scene_pub.publish(scene)

    def to_markers(self, message, first_id):
        shape = Marker()
        shape.header = message.pose.header
        shape.ns, shape.id = "objects", first_id
        shape.pose = message.pose.pose
        shape.color = ColorRGBA(r=message.rgb.r, g=message.rgb.g, b=message.rgb.b, a=0.6)
        if message.shape == "box":
            shape.type = Marker.CUBE
            shape.scale.x, shape.scale.y, shape.scale.z = message.dimensions
        else:
            shape.type = Marker.CYLINDER
            shape.scale.x = shape.scale.y = 2 * message.dimensions[1]
            shape.scale.z = message.dimensions[0]
        label = Marker()
        label.header = message.pose.header
        label.ns, label.id, label.type = "labels", first_id + 1, Marker.TEXT_VIEW_FACING
        label.pose = Pose()
        label.pose.position.x = message.pose.pose.position.x
        label.pose.position.y = message.pose.pose.position.y
        label.pose.position.z = message.pose.pose.position.z + 0.12
        label.scale.z = 0.03
        label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        label.text = (f"{message.id} {message.color} {message.shape}"
                      f"{'' if message.graspable else ' (too wide)'}")
        return [shape, label]


def main():
    rclpy.init()
    node = TabletopPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
