#!/usr/bin/env python3
"""Simulation only: emulate the two RFP602 jaw force sensors from Gazebo contacts.

Each jaw has an RFP602 (10 mm FSR, ~0.2-49 N) under a rigid plate covering the
jaw's contact face (pad_width x pad_length), so a touch anywhere on the plate -
including an object's edge when it sits a few degrees off - loads the sensor.
A contact counts when its patch overlaps the plate; its force along the jaw
axis is what the sensor feels.

Per jaw: normal force F -> FSR resistance R = fsr_r0 / F -> voltage divider
with divider_r to the ADC -> 12-bit count; then the count is converted back to
force with the nominal curve, which is what the real sensor board publishes.
Noise, the trigger threshold and saturation follow a typical FSR402-class part.
"""
import numpy as np
import rclpy
from fairino_fr3_msgs.msg import FingerForces
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformException, TransformListener

JAWS = ("clamp_1", "clamp_2")


class FsrEmulator(Node):
    def __init__(self):
        super().__init__("fsr_emulator")
        p = self.declare_parameter
        self.gripper_frame = p("gripper_frame", "base_link_gripper").value
        self.world_frame = p("world_frame", "world").value
        # Pad centre in the gripper frame (jaw axis x, finger width y, length z
        # toward the tips; fingertips at z = 0.09 m, finger width y 0..0.046 m).
        self.pad_y = p("pad_center_y", 0.023).value
        self.pad_z = p("pad_center_z", 0.060).value
        # +3 mm so contacts reported exactly on the jaw's edge still count.
        self.pad_half = np.array([p("pad_width", 0.046).value,
                                  p("pad_length", 0.060).value]) / 2.0 + 0.003
        self.fsr_r0 = p("fsr_r0", 30000.0).value          # ohm * N
        self.divider_r = p("divider_r", 10000.0).value    # ohm
        self.max_force = p("max_force", 49.0).value        # 5 kg
        self.trigger_force = p("trigger_force", 0.2).value
        self.contact_force = p("contact_force", 0.5).value
        self.noise = p("noise_fraction", 0.03).value
        self.stale_after = p("stale_after", 0.05).value
        rate = p("publish_rate", 100.0).value

        self.latest = {jaw: None for jaw in JAWS}
        self.rng = np.random.default_rng()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        for jaw in JAWS:
            self.create_subscription(
                Contacts, f"/fr3_demo/gz/contacts/{jaw}",
                lambda msg, j=jaw: self.latest.__setitem__(j, (self.get_clock().now(), msg)), 10)
        self.pub = self.create_publisher(FingerForces, "/fr3_demo/gripper/finger_forces", 10)
        self.create_timer(1.0 / rate, self.publish)
        self.get_logger().info(
            f"RFP602 emulator: plate {np.round(self.pad_half * 2000).tolist()} mm "
            f"at y={self.pad_y} z={self.pad_z}")

    def pad_force(self, jaw, T_gripper_world):
        entry = self.latest[jaw]
        if entry is None or (self.get_clock().now() - entry[0]).nanoseconds * 1e-9 > self.stale_after:
            return 0.0
        R, t = T_gripper_world[:3, :3], T_gripper_world[:3, 3]
        jaw_axis = np.array([1.0, 0.0, 0.0])
        total = 0.0
        for contact in entry[1].contacts:
            jaw_is_body_1 = jaw in contact.collision1.name
            points = np.array([R @ [p.x, p.y, p.z] + t for p in contact.positions])
            if len(points) == 0:
                continue
            # Flat-on-flat contact comes as a few points at the corners of the
            # contact patch. The raised pad takes the load if the patch (their
            # bounding box on the jaw face) overlaps it.
            low, high = points[:, 1:].min(axis=0), points[:, 1:].max(axis=0)
            pad = np.array([self.pad_y, self.pad_z])
            if np.any(high < pad - self.pad_half) or np.any(low > pad + self.pad_half):
                continue
            for wrench in contact.wrenches:
                force = wrench.body_1_wrench.force if jaw_is_body_1 else wrench.body_2_wrench.force
                total += abs(float(R @ [force.x, force.y, force.z] @ jaw_axis))
        return total

    def to_sensor(self, force):
        """True force -> (raw ADC count, force the sensor board would report)."""
        force *= 1.0 + self.noise * self.rng.standard_normal()
        if force < self.trigger_force:
            return 0, 0.0
        force = min(force, self.max_force)
        resistance = self.fsr_r0 / force
        raw = int(round(4095 * self.divider_r / (self.divider_r + resistance)))
        if raw <= 0:
            return 0, 0.0
        measured = self.fsr_r0 / (self.divider_r * (4095.0 / raw - 1.0)) if raw < 4095 else self.max_force
        return raw, float(min(measured, self.max_force))

    def publish(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.gripper_frame, self.world_frame, rclpy.time.Time()).transform
        except TransformException:
            return
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat(
            [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w]).as_matrix()
        T[:3, 3] = [tf.translation.x, tf.translation.y, tf.translation.z]
        msg = FingerForces()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.gripper_frame
        msg.raw_clamp_1, msg.force_clamp_1 = self.to_sensor(self.pad_force("clamp_1", T))
        msg.raw_clamp_2, msg.force_clamp_2 = self.to_sensor(self.pad_force("clamp_2", T))
        msg.contact_clamp_1 = msg.force_clamp_1 >= self.contact_force
        msg.contact_clamp_2 = msg.force_clamp_2 >= self.contact_force
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FsrEmulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
