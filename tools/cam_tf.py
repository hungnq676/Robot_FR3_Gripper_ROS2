#!/usr/bin/env python3
"""Publish the hand-eye result as a static TF base_footprint -> camera_link (the
yaml's parent/child frames), linking the camera driver's TF tree to the robot's, so RViz shows the camera (and
its optical axes) where calibration says it is.
Reads camera.HANDEYE_FILE (written by handeye_solve.py).
Usage: cam_tf.py [yaml]"""
import os
import sys

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import camera


def main():
    path = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else camera.HANDEYE_FILE)
    with open(path) as stream:
        cal = yaml.safe_load(stream)
    rclpy.init()
    node = Node("fr3_cam_tf")
    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = cal["parent_frame"]
    t.child_frame_id = cal["child_frame"]
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = cal["xyz"]
    q = cal["quaternion_xyzw"]
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
    StaticTransformBroadcaster(node).sendTransform(t)
    node.get_logger().info(
        f"{t.header.frame_id} -> {t.child_frame_id}: xyz={cal['xyz']} rpy={cal['rpy']} "
        f"({cal.get('samples')} samples, RMS {cal.get('reprojection_rms_px')} px, {cal.get('calibrated_at')})")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
