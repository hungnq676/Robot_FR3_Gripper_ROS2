#!/usr/bin/env python3
"""Real robot, no camera node yet: put the cell into MoveIt exactly as in the
table-only simulation and announce the screwdriver handle as a perceived object
so the MTC planner can target it.

Poses are the simulation world poses (robot base at world (1.20, 0.72), yaw
-90 deg) converted to base_footprint:  x_b = 0.72 - y_w,  y_b = x_w - 1.20,
yaw_b = yaw_w + 90 deg. The handle pose comes from the OAK image measured on
2026-10-28 (Gemini detection + table-corner frame).
"""
import math

import rclpy
from fairino_fr3_msgs.msg import PerceivedObject, PerceivedObjectArray
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

FRAME = "base_footprint"
# id: (size xyz in base frame, centre xyz in base frame, yaw in base frame)
BOXES = {
    "work_table": ((0.50, 1.00, 0.05), (0.08, -0.14, -0.028), 0.0),   # top 3 mm below the base
    "fence_long_camera_side": ((0.02, 1.32, 1.0), (-0.38, -0.07, 0.50), 0.0),
    "fence_short_back": ((0.82, 0.02, 1.0), (0.02, 0.57, 0.50), 0.0),
}
HANDLE = dict(id="obj_2", size=(0.106, 0.027, 0.030), centre=(0.211, -0.258, 0.016),
              yaw=-0.656 + math.pi / 2)


def pose(xyz, yaw):
    p = Pose()
    p.position.x, p.position.y, p.position.z = map(float, xyz)
    p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def box(object_id, size, xyz, yaw):
    o = CollisionObject(id=object_id, operation=CollisionObject.ADD)
    o.header.frame_id = FRAME
    o.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(map(float, size)))]
    o.primitive_poses = [pose(xyz, yaw)]
    return o


class RealTaskScene(Node):
    def __init__(self):
        super().__init__("real_task_scene")
        self.client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.pub = self.create_publisher(PerceivedObjectArray, "/fr3_demo/perceived_objects", 1)
        self.create_timer(0.5, self.announce)

    def add_scene(self):
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("waiting for move_group ...")
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [box(k, *v) for k, v in BOXES.items()] + [
            box(HANDLE["id"], HANDLE["size"], HANDLE["centre"], HANDLE["yaw"])]
        future = self.client.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(self, future)
        ok = future.result() is not None and future.result().success
        self.get_logger().info(f"table, 2 fences and screwdriver handle {HANDLE['id']}: "
                               + ("added" if ok else "FAILED"))

    def announce(self):
        o = PerceivedObject(id=HANDLE["id"], shape="box", shape_type=PerceivedObject.SHAPE_BOX,
                            color="red", graspable=True, num_points=0)
        o.pose = PoseStamped(pose=pose(HANDLE["centre"], HANDLE["yaw"]))
        o.pose.header.frame_id = FRAME
        o.dimensions = list(map(float, HANDLE["size"]))
        o.grasp_width = float(min(HANDLE["size"][:2]))
        msg = PerceivedObjectArray(objects=[o])
        msg.header.frame_id = FRAME
        msg.header.stamp = o.pose.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = RealTaskScene()
    node.add_scene()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
