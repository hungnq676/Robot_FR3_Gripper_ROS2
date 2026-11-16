#!/usr/bin/env python3
"""Simulation only: turn the generic /fr3_demo/gazebo_attach request into a weld
of the Gazebo model that is actually between the jaws.

The pick pipeline only knows perceived objects (obj_N), not Gazebo model names.
On an attach request this node takes the graspable model whose centre is
nearest the TCP, triggers that model's DetachableJoint, and relays its state
on /fr3_demo/gazebo_attachment_state ("attached" / "detached").
"""
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformException, TransformListener


class GazeboGraspAttacher(Node):
    def __init__(self):
        super().__init__("gazebo_grasp_attacher")
        self.models = self.declare_parameter("graspable_models", ["demo_box"]).value
        self.tcp_link = self.declare_parameter("tcp_link", "grasp_frame").value
        self.world_frame = self.declare_parameter("world_frame", "world").value
        self.max_distance = self.declare_parameter("max_distance", 0.08).value
        self.positions = {}
        self.active = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.attach_publishers = {}
        for model in self.models:
            self.create_subscription(
                Pose, f"/fr3_demo/gz/pose/{model}",
                lambda msg, m=model: self.positions.__setitem__(
                    m, np.array([msg.position.x, msg.position.y, msg.position.z])), 10)
            self.create_subscription(
                String, f"/fr3_demo/gz/state/{model}",
                lambda msg, m=model: self.on_state(m, msg), 10)
            self.attach_publishers[model] = self.create_publisher(
                Empty, f"/fr3_demo/gz/attach/{model}", 1)
        self.state_pub = self.create_publisher(String, "/fr3_demo/gazebo_attachment_state", 10)
        self.create_subscription(Empty, "/fr3_demo/gazebo_attach", self.on_attach, 1)
        self.get_logger().info(f"Weldable models: {self.models}")

    def on_attach(self, _):
        try:
            t = self.tf_buffer.lookup_transform(
                self.world_frame, self.tcp_link, rclpy.time.Time()).transform.translation
        except TransformException as error:
            self.get_logger().error(f"no TCP pose: {error}")
            return
        tcp = np.array([t.x, t.y, t.z])
        distances = {m: np.linalg.norm(p - tcp) for m, p in self.positions.items()}
        if not distances:
            self.get_logger().error("no model poses received from Gazebo")
            return
        model = min(distances, key=distances.get)
        if distances[model] > self.max_distance:
            self.get_logger().error(
                f"nothing between the jaws: nearest is {model} at {distances[model]:.3f} m")
            return
        self.active = model
        self.get_logger().info(f"welding {model} ({distances[model] * 1000:.0f} mm from TCP)")
        self.attach_publishers[model].publish(Empty())

    def on_state(self, model, msg):
        if model == self.active:
            self.state_pub.publish(msg)
            if msg.data == "detached":
                self.active = None


def main():
    rclpy.init()
    node = GazeboGraspAttacher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
