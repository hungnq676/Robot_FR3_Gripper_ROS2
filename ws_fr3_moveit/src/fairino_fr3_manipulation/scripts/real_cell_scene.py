#!/usr/bin/env python3
"""Real cell: add the fixed obstacles MoveIt must know about before the camera
sees anything. For now just the robot's own table top, as a large slab, so no
plan can drive the arm or the gripper into the table.

Table height was measured on 2026-10-30 by touching. The flange was jogged
18 mm above the table, and FK put the tabletop at z = +8.7 mm in
base_footprint. The slab top therefore sits at +12 mm (3 mm margin). The
robot base (base_link) stands inside the slab, so the table/base_link pair is
allowed in the ACM.
"""
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (AllowedCollisionEntry, CollisionObject, PlanningScene,
                             PlanningSceneComponents)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

TABLE_ID = "robot_table"


class RealCellScene(Node):
    def __init__(self):
        super().__init__("real_cell_scene")
        self.frame = self.declare_parameter("frame", "base_footprint").value
        self.size = self.declare_parameter("table_size", [1.6, 1.6, 0.05]).value
        self.center = self.declare_parameter("table_center_xy", [0.0, 0.0]).value
        self.top_z = self.declare_parameter("table_top_z", 0.012).value
        self.base_links = self.declare_parameter("links_on_table", ["base_link"]).value
        self.client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.get_scene = self.create_client(GetPlanningScene, "/get_planning_scene")

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def allowed_collisions(self):
        """Current ACM plus table <-> robot base. A diff that carries an ACM
        replaces the whole matrix in move_group, so start from the live one."""
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        acm = self.call(self.get_scene, req).scene.allowed_collision_matrix
        names = list(acm.entry_names)
        rows = [list(e.enabled) for e in acm.entry_values]
        if TABLE_ID not in names:
            names.append(TABLE_ID)
            for row in rows:
                row.append(False)
            rows.append([False] * len(names))
        t = names.index(TABLE_ID)
        for link in self.base_links:
            if link in names:
                i = names.index(link)
                rows[i][t] = rows[t][i] = True
        acm.entry_names = names
        acm.entry_values = [AllowedCollisionEntry(enabled=r) for r in rows]
        return acm

    def run(self):
        for client, name in ((self.client, "/apply_planning_scene"), (self.get_scene, "/get_planning_scene")):
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f"waiting for move_group {name} ...")
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(self.size))
        pose = Pose()
        pose.position.x, pose.position.y = float(self.center[0]), float(self.center[1])
        pose.position.z = self.top_z - self.size[2] / 2.0
        pose.orientation.w = 1.0
        table = CollisionObject(id=TABLE_ID, operation=CollisionObject.ADD)
        table.header.frame_id = self.frame
        table.primitives, table.primitive_poses = [box], [pose]
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [table]
        scene.allowed_collision_matrix = self.allowed_collisions()
        res = self.call(self.client, ApplyPlanningScene.Request(scene=scene))
        ok = res is not None and res.success
        self.get_logger().info(
            f"{TABLE_ID} {self.size} m, top at z={self.top_z} in {self.frame}, "
            f"allowed vs {self.base_links}: " + ("added" if ok else "FAILED"))


def main():
    rclpy.init()
    node = RealCellScene()
    node.run()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
