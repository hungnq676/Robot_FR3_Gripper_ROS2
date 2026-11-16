#!/usr/bin/env python3
"""MoveIt Task Constructor on the REAL FR3: from where the arm is, to a waiting
pose above/behind a GraspNet grasp, then straight in along the approach axis
until the jaws are around the object -- ready to close. With --pick (gripper
over Wi-Fi, gripper.py): open the jaws before moving, close them on the object
after arriving, then lift straight up --lift m. Without --pick nothing is closed.

  grasps : ~/.ros/fr3_last_grasp.yaml from grasp_once.py (best first)
  scene  : run scene_obstacles.py first so the other objects are obstacles
  task   : CurrentState -> Connect (Pilz PTP) -> [MoveRelative in (Cartesian),
           ComputeIK(GeneratePose(grasp))]
  run    : the planned path is shown in RViz (Planned Path) with the TCP, finger and
           object points; each step (to the waiting point, in, close, lift) runs ONLY
           after the user types y, and the TCP is read back after each move.
  close  : --pick closes to the object's measured width (target_box3d along the
           closing axis) minus --squeeze, not all the way.

GraspNet grasp (x = approach, y = closing, tips at t + depth * x) -> our TCP
grasp_frame (z = approach, x = closing, 30 mm in from the finger tips, URDF):
  p_tcp = t + (depth - TCP_FROM_TIPS) * x,  R_tcp = [y, z, x] (and the same
  turned 180 deg about the approach axis: the jaws are symmetric).

Usage: mtc_pregrasp.py [--result FILE] [--index N] [--approach 0.08] [--scale 0.2] [--deeper 0.015]
                       [--pick [--lift 0.10]] [--dry-run]
Must run in an interactive terminal (it waits on input()).
"""
import argparse
import os
import sys
import tempfile
import threading
import time

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

LAST_RESULT = os.path.expanduser("~/.ros/fr3_last_grasp.yaml")
TCP_FROM_TIPS = 0.030      # grasp_frame -> finger tips along z (URDF real gripper, 2026-11-01)
TCP_LINK = "grasp_frame"
TABLE_TOP_Z = 0.012        # base_footprint (point_test, real_cell_scene)
CORNER_ABOVE_TABLE = 0.003   # the scene table slab already sits 3 mm above the touched table
# Finger collision corners in grasp_frame (URDF real gripper): jaws 44..82 mm each side in x;
# the triangle fingers are y -51..+23 mm at the TCP level and only y -20..+10 mm at the
# tip box, which ends 3 mm past the tips.
FINGER_CORNERS = np.array([[sx * 0.082, yy, zz] for sx in (-1, 1)
                           for yy, zz in ((-0.051, 0.0), (0.023, 0.0), (-0.020, TCP_FROM_TIPS + 0.003),
                                          (0.010, TCP_FROM_TIPS + 0.003))])
ARM = "arm"


def moveit_params():
    """Robot model + planners of the real robot (same as move_group with real:=true), as a params file."""
    from moveit_configs_utils import MoveItConfigsBuilder
    cfg = (MoveItConfigsBuilder("fairino_fr3", package_name="fairino_fr3_moveit_config")
           .robot_description(mappings={"hardware_type": "fairino", "use_gazebo": "false", "mount_to_world": "false"})
           .robot_description_semantic(file_path="config/fairino_fr3.srdf")
           .joint_limits(file_path="config/joint_limits_real.yaml")
           .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits_real.yaml")
           .planning_pipelines(default_planning_pipeline="pilz_industrial_motion_planner",
                               pipelines=["ompl", "pilz_industrial_motion_planner"])
           .to_moveit_configs()).to_dict()
    f = tempfile.NamedTemporaryFile("w", suffix="_mtc_params.yaml", delete=False)
    yaml.safe_dump({"/**": {"ros__parameters": cfg}}, f)
    f.close()
    return f.name


def tcp_poses(g, deeper=0.0):
    """GraspNet grasp (dict from grasp_once) -> two TCP poses (4x4) in base_footprint.
    deeper: extra push along the approach (2026-11-01: on the gamepad the tips stopped
    ~15 mm short). The push is cut so that the lowest corner of the finger boxes stays
    CORNER_ABOVE_TABLE over the table: a tilted grasp on a low object (the tray) otherwise
    puts a finger corner into the table and IK fails."""
    T = np.array(g["T_base"])
    x, y, z = T[:3, 0], T[:3, 1], T[:3, 2]
    out = []
    for turn in (0.0, np.pi):
        R = np.column_stack([y, z, x]) @ Rotation.from_euler("z", turn).as_matrix()
        push = g.get("depth", 0.02) + deeper
        if x[2] < -1e-3:
            low = (R @ FINGER_CORNERS.T)[2].min()   # lowest finger corner below the TCP
            limit = (T[2, 3] - TCP_FROM_TIPS * x[2] + low - (TABLE_TOP_Z + CORNER_ABOVE_TABLE)) / -x[2]
            push = min(push, limit)
        P = np.eye(4)
        P[:3, :3] = R
        P[:3, 3] = T[:3, 3] + (push - TCP_FROM_TIPS) * x
        out.append(P)
    return out


def pose_stamped(P):
    from geometry_msgs.msg import PoseStamped
    ps = PoseStamped()
    ps.header.frame_id = "base_footprint"
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, P[:3, 3])
    q = Rotation.from_matrix(P[:3, :3]).as_quat()
    ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = map(float, q)
    return ps


def build_task(node, P, approach, scale, chain=None):
    """chain = dict(lift, D, up, drop_in): also plan, in the same task (so RViz shows the whole
    path before anything moves, 2026-11-02): straight up `lift` -> PTP to above the box ->
    straight in `drop_in` to D -> straight up `up` -> PTP to HOME_DEG. The solution then has 7
    segments: to wait, in, lift, to box, down, up, home (the gripper acts between them)."""
    from geometry_msgs.msg import PoseStamped, Vector3Stamped
    from moveit.task_constructor import core, stages
    from std_msgs.msg import Header
    task = core.Task()
    task.name = "fr3_tien_toi_gap"
    task.loadRobotModel(node)
    task.properties["group"] = ARM
    task.properties["eef"] = "custom_gripper"
    task.properties["ik_frame"] = TCP_LINK

    current = stages.CurrentState("hien tai")
    task.add(current)

    ptp = core.PipelinePlanner(node, "pilz_industrial_motion_planner")
    ptp.planner = "PTP"
    ptp.max_velocity_scaling_factor = scale
    ptp.max_acceleration_scaling_factor = scale
    connect = stages.Connect("di toi diem cho", [(ARM, ptp)])
    connect.timeout = 10.0
    connect.setCostTerm(core.PathLength())   # prefer the IK solution reached with the least joint motion
    task.add(connect)

    cart = core.CartesianPath()
    cart.max_velocity_scaling_factor = scale / 2
    cart.max_acceleration_scaling_factor = scale / 2
    cart.step_size = 0.005
    grasp = core.SerialContainer("tien vao vat")
    move_in = stages.MoveRelative("tien vao", cart)
    move_in.group = ARM
    move_in.ik_frame = PoseStamped(header=Header(frame_id=TCP_LINK))
    move_in.min_distance = approach * 0.8
    move_in.max_distance = approach
    direction = Vector3Stamped()
    direction.header.frame_id = TCP_LINK
    direction.vector.z = 1.0
    move_in.setDirection(direction)
    grasp.insert(move_in)

    target = stages.GeneratePose("tu the san sang kep")
    target.pose = pose_stamped(P)
    target.setMonitoredStage(task["hien tai"])   # `current` was handed over by task.add()
    ik = stages.ComputeIK("IK", target)
    ik.group = ARM
    ik.ik_frame = PoseStamped(header=Header(frame_id=TCP_LINK))
    ik.max_ik_solutions = 16
    ik.min_solution_distance = 0.3   # spread the arm configurations (rad), the cheapest wins
    ik.properties.configureInitFrom(core.Stage.PropertyInitializerSource.INTERFACE, ["target_pose"])
    grasp.insert(ik)
    task.add(grasp)
    if chain is None:
        return task

    def straight(name, dist, frame, z, min_frac):
        st = stages.MoveRelative(name, cart)
        st.group = ARM
        st.ik_frame = PoseStamped(header=Header(frame_id=TCP_LINK))
        st.min_distance = dist * min_frac
        st.max_distance = dist
        v = Vector3Stamped()
        v.header.frame_id = frame
        v.vector.z = z
        st.setDirection(v)
        return st

    task.add(straight("nhac len", chain["lift"], "base_footprint", 1.0, 0.5))   # >= half the lift; the carry PTP goes on from there
    carry = stages.Connect("mang toi hop", [(ARM, ptp)])
    carry.timeout = 10.0
    carry.setCostTerm(core.PathLength())
    task.add(carry)
    place = core.SerialContainer("tha vao hop")
    place.insert(straight("ha xuong", chain["drop_in"], TCP_LINK, 1.0, 0.8))
    over = stages.GeneratePose("tren hop")
    over.pose = pose_stamped(chain["D"])
    over.setMonitoredStage(task["nhac len"])
    ik2 = stages.ComputeIK("IK hop", over)
    ik2.group = ARM
    ik2.ik_frame = PoseStamped(header=Header(frame_id=TCP_LINK))
    ik2.max_ik_solutions = 16
    ik2.min_solution_distance = 0.3
    ik2.properties.configureInitFrom(core.Stage.PropertyInitializerSource.INTERFACE, ["target_pose"])
    place.insert(ik2)
    task.add(place)
    task.add(straight("nhac khoi hop", chain["up"], "base_footprint", 1.0, 0.8))   # out of the box before going home
    home = stages.MoveTo("ve HOME", ptp)
    home.group = ARM
    home.setGoal({k: float(np.radians(v)) for k, v in HOME_DEG.items()})
    task.add(home)
    return task


def build_lift(node, lift, scale, down=False):
    """CurrentState -> straight up (base_footprint +z; down=True: -z) by lift, Cartesian at half speed."""
    from geometry_msgs.msg import PoseStamped, Vector3Stamped
    from moveit.task_constructor import core, stages
    from std_msgs.msg import Header
    task = core.Task()
    task.name = "fr3_nhac_len"
    task.loadRobotModel(node)
    task.properties["group"] = ARM
    task.properties["eef"] = "custom_gripper"
    task.properties["ik_frame"] = TCP_LINK
    task.add(stages.CurrentState("hien tai"))
    cart = core.CartesianPath()
    cart.max_velocity_scaling_factor = scale / 2
    cart.max_acceleration_scaling_factor = scale / 2
    cart.step_size = 0.005
    up = stages.MoveRelative("nhac len", cart)
    up.group = ARM
    up.ik_frame = PoseStamped(header=Header(frame_id=TCP_LINK))
    up.min_distance = lift * (0.95 if down else 0.5)   # going down: the whole way or not at all
    up.max_distance = lift
    direction = Vector3Stamped()
    direction.header.frame_id = "base_footprint"
    direction.vector.z = -1.0 if down else 1.0
    up.setDirection(direction)
    task.add(up)
    return task


def object_width_along(res, axis):
    """Width of the object's 3D box (grasp_once target_box3d) along a horizontal direction, m."""
    box = res.get("target_box3d")
    if not box:
        return None
    u = np.array([np.cos(box["yaw"]), np.sin(box["yaw"])])
    a = np.asarray(axis[:2], float)
    a = a / max(np.linalg.norm(a), 1e-9)
    return abs(a @ u) * box["size"][0] + abs(a @ np.array([-u[1], u[0]])) * box["size"][1]


JAW_IN_X = 0.044      # grasp_frame: jaw inner faces at +-44 mm when open (88 mm)
JAW_HALF_Y = 0.019    # fingers are 38 mm wide


def width_between_jaws(res, P):
    """Object points (grasp_once object_points, base frame) that sit between the open jaws of
    TCP pose P: within the finger width and from the tips back 40 mm along the approach.
    Returns (gap_mm, extent_mm, n_in, n_hit): the jaws close symmetrically, so the first contact
    is at 2 * max|x|; n_hit counts points where a jaw itself would be (beyond +-44 mm)."""
    path = res.get("object_points")
    if not path or not os.path.exists(path):
        return None
    q = (np.load(path).astype(float) - P[:3, 3]) @ P[:3, :3]     # x closing, y finger width, z approach
    slab = (np.abs(q[:, 1]) < JAW_HALF_Y) & (q[:, 2] > TCP_FROM_TIPS - 0.040) & (q[:, 2] < TCP_FROM_TIPS)
    inside = slab & (np.abs(q[:, 0]) < JAW_IN_X)
    hit = slab & (np.abs(q[:, 0]) >= JAW_IN_X) & (np.abs(q[:, 0]) < 0.082)
    if inside.sum() < 5:
        return 0.0, 0.0, int(inside.sum()), int(hit.sum())
    x = q[inside, 0]
    return 2000 * np.abs(x).max(), 1000 * (x.max() - x.min()), int(inside.sum()), int(hit.sum())


def check_points(P, res):
    """Print where the TCP, finger tips and finger corners will be against the table and the object."""
    tips = P[:3, 3] + TCP_FROM_TIPS * P[:3, 2]
    low = (P[:3, :3] @ FINGER_CORNERS.T)[2].min() + P[2, 3]
    print(f"  TCP ({P[0, 3] * 1000:.0f}, {P[1, 3] * 1000:.0f}, {P[2, 3] * 1000:.0f}) mm; "
          f"dau ngon ({tips[0] * 1000:.0f}, {tips[1] * 1000:.0f}, {tips[2] * 1000:.0f}) mm, cao hon ban {(tips[2] - TABLE_TOP_Z) * 1000:.0f} mm; "
          f"goc ngon thap nhat cao hon ban {(low - TABLE_TOP_Z) * 1000:.0f} mm")
    box = res.get("target_box3d")
    if box:
        c = np.array(box["center"])
        top = c[2] + box["size"][2] / 2
        off = np.linalg.norm(P[:2, 3] - c[:2])
        print(f"  vat: tam ({c[0] * 1000:.0f}, {c[1] * 1000:.0f}) mm, dinh z {top * 1000:.0f} mm; TCP lech tam vat {off * 1000:.0f} mm, "
              f"dau ngon thap hon dinh vat {(top - tips[2]) * 1000:.0f} mm")


HOME_FILE = os.path.expanduser("~/.ros/fr3_home.yaml")   # tools/save_home.py
HOME_DEG = {"j1": 132.9, "j2": -124.4, "j3": -56.9, "j4": -96.4, "j5": 66.3, "j6": -167.7}  # camera view clear
if os.path.exists(HOME_FILE):
    HOME_DEG = yaml.safe_load(open(HOME_FILE))["home_deg"]


def build_home(node, scale):
    """CurrentState -> Pilz PTP to HOME_DEG (the pose the camera calibration/observation used)."""
    from moveit.task_constructor import core, stages
    task = core.Task()
    task.name = "fr3_ve_cho"
    task.loadRobotModel(node)
    task.add(stages.CurrentState("hien tai"))
    ptp = core.PipelinePlanner(node, "pilz_industrial_motion_planner")
    ptp.planner = "PTP"
    ptp.max_velocity_scaling_factor = scale
    ptp.max_acceleration_scaling_factor = scale
    home = stages.MoveTo("ve cho", ptp)
    home.group = ARM
    home.setGoal({k: float(np.radians(v)) for k, v in HOME_DEG.items()})
    task.add(home)
    return task


def plan_run(task, what, tries=5):
    """Plan, execute the cheapest solution; True when it ran OK."""
    if not (task.plan(tries) and len(task.solutions) > 0):
        print(f"{what}: khong lap duoc ke hoach.")
        return False
    err = task.execute(min(task.solutions, key=lambda s: s.cost))
    print(f"{what}: {'OK' if err.val == 1 else f'loi {err.val}'}")
    return err.val == 1


def joint_swing(solution, joint="j1"):
    """Largest distance (deg) of one joint from where it starts, over the whole solution."""
    vals = []
    for t in trajectories(solution):
        jt = t.joint_trajectory
        if joint in jt.joint_names:
            k = jt.joint_names.index(joint)
            vals += [p.positions[k] for p in jt.points]
    return float(np.degrees(np.max(np.abs(np.array(vals) - vals[0])))) if vals else 0.0


def trajectories(solution):
    msg = solution.toMsg()
    return [s.trajectory for s in msg.sub_trajectory if s.trajectory.joint_trajectory.points]


def box_as_walls(node, box, wall=0.012):
    """The container as 4 walls instead of the solid vat_* box scene_obstacles made of it, so the
    arm may reach inside to set the object down (2026-11-02: it was let go 8 cm above the rim)."""
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
    from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
    from shape_msgs.msg import SolidPrimitive
    get = node.create_client(GetPlanningScene, "/get_planning_scene")
    apply = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not (get.wait_for_service(timeout_sec=5) and apply.wait_for_service(timeout_sec=5)):
        print("  (khong goi duoc planning scene: hop van la khoi dac)")
        return

    def call(cli, req):
        fut = cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 5:
            time.sleep(0.05)
        return fut.result()

    req = GetPlanningScene.Request()
    req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    scene = call(get, req)
    c = np.array(box["center"])
    L, W, H = box["size"]
    diff = PlanningScene(is_diff=True)
    for o in (scene.scene.world.collision_objects if scene else []):
        if o.id.startswith("vat_") or o.id.startswith("hop_"):
            q = o.primitive_poses[0].position if o.primitive_poses else o.pose.position
            if o.id.startswith("hop_") or np.hypot(o.pose.position.x + q.x - c[0], o.pose.position.y + q.y - c[1]) < 0.10:
                diff.world.collision_objects.append(CollisionObject(id=o.id, operation=CollisionObject.REMOVE))
    u = np.array([np.cos(box["yaw"]), np.sin(box["yaw"])])
    v = np.array([-u[1], u[0]])
    qz = Rotation.from_euler("z", box["yaw"]).as_quat()
    for k, (off, size) in enumerate([(u * (L - wall) / 2, (wall, W, H)), (-u * (L - wall) / 2, (wall, W, H)),
                                     (v * (W - wall) / 2, (L, wall, H)), (-v * (W - wall) / 2, (L, wall, H))]):
        co = CollisionObject(id=f"hop_thanh_{k + 1}", operation=CollisionObject.ADD)
        co.header.frame_id = "base_footprint"
        co.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(x) for x in size]))
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = float(c[0] + off[0]), float(c[1] + off[1]), float(c[2])
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, qz)
        co.primitive_poses.append(pose)
        diff.world.collision_objects.append(co)
    call(apply, ApplyPlanningScene.Request(scene=diff))
    print(f"  hop dung -> 4 thanh day {wall * 1000:.0f} mm (bo khoi dac), robot duoc dua vat vao trong")


def drop_target_obstacle(node, res):
    """Remove the vat_* boxes (scene_obstacles.py) that sit on the target: they were
    made before this grasp_once run (2026-11-01: the tray was still vat_3, no plan)."""
    import rclpy
    from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
    from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
    box = res.get("target_box3d")
    if not box:
        return
    c = np.array(box["center"][:2])
    reach = max(0.04, 0.5 * float(np.hypot(*box["size"][:2])))
    get = node.create_client(GetPlanningScene, "/get_planning_scene")
    apply = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not (get.wait_for_service(timeout_sec=5) and apply.wait_for_service(timeout_sec=5)):
        print("  (khong goi duoc planning scene, bo qua buoc loc vat dich)")
        return
    req = GetPlanningScene.Request()
    req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    fut = get.call_async(req)
    t0 = time.time()
    while not fut.done() and time.time() - t0 < 5:
        time.sleep(0.05)
    if not fut.done():
        return
    diff = PlanningScene(is_diff=True)
    for o in fut.result().scene.world.collision_objects:
        if not o.id.startswith("vat_"):
            continue
        p = o.pose.position
        q = o.primitive_poses[0].position if o.primitive_poses else p
        xy = np.array([p.x + q.x, p.y + q.y])
        if np.linalg.norm(xy - c) < reach:
            diff.world.collision_objects.append(CollisionObject(id=o.id, operation=CollisionObject.REMOVE))
            print(f"  bo vat can {o.id} ({xy[0] * 1000:.0f}, {xy[1] * 1000:.0f}) mm: chinh la vat dich")
    if diff.world.collision_objects:
        fut = apply.call_async(ApplyPlanningScene.Request(scene=diff))
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 5:
            time.sleep(0.05)


def place(a):
    """Put the held object down: straight down --lift - 5 mm, open the jaws, straight up --lift."""
    import rclcpp
    from gripper import MAX_TEMP_C, Gripper, width_mm
    rclcpp.init(["mtc_pregrasp", "--ros-args", "--params-file", moveit_params()])
    node = rclcpp.Node("mtc_pregrasp")
    drop = a.lift - 0.005          # 5 mm short of where it was picked up: release just above the table
    down = build_lift(node, drop, a.scale, down=True)
    if not (down.plan(5) and len(down.solutions) > 0):
        print("Khong lap duoc ke hoach ha xuong.")
        return
    print(f"Dat xuong: ha thang {drop * 1000:.0f} mm -> mo gripper -> nhac len {a.lift * 100:.0f} cm.")
    print(">>> Chay? [y/N] (go y + Enter)", flush=True)
    if a.dry_run or input().strip().lower() != "y":
        print("Khong chay.")
        return
    err = down.execute(min(down.solutions, key=lambda s: s.cost))
    if err.val != 1:
        print(f"Ha xuong: loi {err.val}. Dung, gripper van kep.")
        return
    grip = Gripper()
    try:
        p = grip.open()
        print(f"Da tha vat: gripper pos {p} (~{width_mm(p):.0f} mm)")
    except RuntimeError as e:   # 2026-11-01: 76 C after holding ~5 min -> open refused; free the jaws instead
        grip.free()
        print(f"{e}\nNha torque gripper (ma long ra, servo nguoi). Cho servo nguoi roi mo het ra...")
        # 2026-11-02 17:15: freeing alone left the jaws closed on the part; the user wants them opened.
        t0, p = time.time(), None
        while time.time() - t0 < 300:
            time.sleep(10)
            try:
                c = grip.temp()
            except Exception:
                continue
            if c is None:
                continue
            print(f"  servo {c:.0f} C")
            if c < MAX_TEMP_C - 2:
                p = grip.open()
                break
        if p is None:
            print("Servo chua nguoi sau 5 phut: ma van nha torque. KHONG nhac len: kiem tra vat da roi khoi ma chua.")
            return
        print(f"Da mo gripper: pos {p} (~{width_mm(p):.0f} mm)")
    up = build_lift(node, a.lift, a.scale)
    if up.plan(5) and len(up.solutions) > 0:
        err = up.execute(min(up.solutions, key=lambda s: s.cost))
        print(f"Nhac len: {'OK' if err.val == 1 else f'loi {err.val}'}")


def home_only(a):
    import rclcpp
    rclcpp.init(["mtc_pregrasp", "--ros-args", "--params-file", moveit_params()])
    node = rclcpp.Node("mtc_pregrasp")
    print(f"Ve HOME (deg): {HOME_DEG}")
    import rclpy
    from sensor_msgs.msg import JointState
    rclpy.init()
    n = rclpy.create_node("mtc_home_check")
    js = {}
    n.create_subscription(JointState, "/joint_states", lambda m: js.update(zip(m.name, m.position)), 10)
    t0 = time.time()
    while not all(j in js for j in HOME_DEG) and time.time() - t0 < 5:
        rclpy.spin_once(n, timeout_sec=0.1)
    if all(j in js for j in HOME_DEG):
        off = max(abs(np.degrees(js[j]) - v) for j, v in HOME_DEG.items())
        if off < 2.0:
            print(f"Da o HOME (lech toi da {off:.1f} deg).")
            return
    if a.dry_run:
        return
    if not a.yes:
        print(">>> Chay? [y/N] (go y + Enter)", flush=True)
        if input().strip().lower() != "y":
            print("Khong chay.")
            return
    plan_run(build_home(node, a.scale), "Ve HOME")


def lift_only(a):
    import rclcpp
    rclcpp.init(["mtc_pregrasp", "--ros-args", "--params-file", moveit_params()])
    node = rclcpp.Node("mtc_pregrasp")
    lift = build_lift(node, a.lift, a.scale)
    if not (lift.plan(5) and len(lift.solutions) > 0):
        print("Khong lap duoc ke hoach nhac len.")
        return
    up = min(lift.solutions, key=lambda s: s.cost)
    print(f"Nhac thang len {a.lift * 100:.0f} cm (chi phi {up.cost:.2f}).")
    print(">>> Chay that? [y/N] (go y + Enter)", flush=True)
    if a.dry_run or input().strip().lower() != "y":
        print("Khong chay.")
        return
    err = lift.execute(up)
    print(f"Nhac len: {'OK' if err.val == 1 else f'loi {err.val}'}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--result", default=LAST_RESULT, help="grasp_once.py result (yaml)")
    ap.add_argument("--index", type=int, default=0, help="start from this GraspNet grasp (0 = best)")
    ap.add_argument("--approach", type=float, default=0.08, help="straight-in distance along the approach axis, m")
    ap.add_argument("--scale", type=float, default=0.2, help="velocity/acceleration scaling (approach: half)")
    ap.add_argument("--deeper", type=float, default=0.015,
                    help="push the tips this much further than GraspNet's depth, m (finger corners stay 3 mm above the table slab)")
    ap.add_argument("--geometric", action="store_true",
                    help="use grasp_once's top-down grasp across the box's short side instead of GraspNet")
    ap.add_argument("--pick", action="store_true", help="open before, close on the object, then lift (real gripper)")
    ap.add_argument("--lift", type=float, default=0.10, help="lift straight up after closing, m")
    ap.add_argument("--max-base-turn", type=float, default=90.0,
                    help="drop plans that turn the base joint j1 further than this from where it starts, deg")
    ap.add_argument("--grip-deg", type=float, default=5.0,
                    help="close this many servo degrees past the angle where the measured object is touched")
    ap.add_argument("--squeeze", type=float, default=0.008,
                    help="after the jaws stall on the object, target this much further closed, m (= grip force)")
    ap.add_argument("--into-box", metavar="YAML",
                    help="after lifting: carry the object over this box (grasp_once --save-as), let go, lift, go home")
    ap.add_argument("--drop-above", type=float, default=0.06, help="(old, unused: see --place-above-floor)")
    ap.add_argument("--place-above-floor", type=float, default=0.02,
                    help="lower the object until its bottom is this far above the box floor, then let go, m")
    ap.add_argument("--only-home", action="store_true", help="just go to HOME (~/.ros/fr3_home.yaml)")
    ap.add_argument("--yes", action="store_true", help="do not ask (the caller already asked, run_pick_to_box.sh)")
    ap.add_argument("--only-lift", action="store_true", help="just lift straight up --lift m from where the arm is")
    ap.add_argument("--place", action="store_true", help="put the held object down (--lift back down), open, lift")
    ap.add_argument("--step", action="store_true", help="ask before each step (waiting point, in, close, lift); default: ask once")
    ap.add_argument("--dry-run", action="store_true", help="plan and show only")
    a = ap.parse_args()
    if a.only_lift:
        return lift_only(a)
    if a.only_home:
        return home_only(a)
    if a.place:
        return place(a)

    res = yaml.safe_load(open(a.result))
    grasps = res.get("graspnet") or []
    if a.geometric:
        geo = res.get("geometric") or {}
        if not geo.get("ok"):
            sys.exit(f"{a.result}: khong co grasp hinh hoc dat ({geo})")
        # 2026-11-01: GraspNet's 75 mm grasps on the white bracket put a finger on the bracket
        # (user hit the e-stop); the top-down grasp across the short side straddles it.
        grasps = [dict(score=1.0, width=geo["width"], depth=0.02, T_base=geo["T_base"])]
    if not grasps:
        sys.exit(f"{a.result}: khong co grasp GraspNet nao ({res.get('note')})")
    print(f"Vat: {res.get('target')} ({len(grasps)} grasp, anh {res.get('image')})")

    import rclcpp
    import rclpy
    from moveit_msgs.msg import DisplayTrajectory, RobotState
    from sensor_msgs.msg import JointState

    params = moveit_params()
    rclcpp.init(["mtc_pregrasp", "--ros-args", "--params-file", params])
    node = rclcpp.Node("mtc_pregrasp")
    rclpy.init()
    pub_node = rclpy.create_node("mtc_pregrasp_view")
    display = pub_node.create_publisher(DisplayTrajectory, "/display_planned_path", 1)
    joints = {}
    pub_node.create_subscription(JointState, "/joint_states", lambda m: joints.update(zip(m.name, m.position)), 10)
    threading.Thread(target=rclpy.spin, args=(pub_node,), daemon=True).start()
    time.sleep(1.0)
    drop_target_obstacle(pub_node, res)
    if a.into_box:
        box_as_walls(pub_node, yaml.safe_load(open(os.path.expanduser(a.into_box)))["target_box3d"])

    # Every grasp, both jaw turns. Drop the ones whose jaws do not straddle the object
    # (object points between / under the jaws), then keep the EASIEST, not just the
    # cheapest path (2026-11-01: the cheapest was often tilted and 50 mm off the object):
    #   ease = tilt (deg) + TCP-to-object-centre distance (mm) + path cost (rad)
    centre = None
    if res.get("object_points") and os.path.exists(res["object_points"]):
        centre = np.median(np.load(res["object_points"]).astype(float)[:, :2], 0)
    elif res.get("target_box3d"):
        centre = np.array(res["target_box3d"]["center"][:2])
    box = None
    if a.into_box:
        box = yaml.safe_load(open(os.path.expanduser(a.into_box))).get("target_box3d")
        if not box:
            sys.exit(f"{a.into_box}: khong co hop 3D cua hop dung")
        btop = box["center"][2] + box["size"][2] / 2

    def chain_for(P, yaw=0.0):
        """Over the box: same TCP orientation as the grasp (the lift is a pure translation); the
        object hangs as far below the TCP as the TCP was above the table when gripped."""
        if box is None:
            return None
        # 2026-11-02: the box centre lies at ~166 deg from the base, next to the j1 limit (+-175):
        # drop at the point inside the opening (70 mm from the walls) nearest to the object
        c = np.array(box["center"][:2])
        u = np.array([np.cos(box["yaw"]), np.sin(box["yaw"])])
        v = np.array([-u[1], u[0]])
        half = np.maximum(np.array(box["size"][:2]) / 2 - 0.11, 0.0)   # wall 12 mm + fingers reach 82 mm sideways + margin
        d = P[:2, 3] - c
        xy = c + np.clip(d @ u, -half[0], half[0]) * u + np.clip(d @ v, -half[1], half[1]) * v
        D = P.copy()
        # 2026-11-02: keeping the grasp's yaw over the box made the arm flip (j1 150 -> -30);
        # the object may land turned, so the drop pose may turn about the vertical (yaw)
        D[:3, :3] = Rotation.from_euler("z", yaw).as_matrix() @ P[:3, :3]
        # set the object down near the floor: its bottom hangs (P_z - table) below the TCP
        floor = btop - box["size"][2] + 0.005
        # ...but the gripper body must stay above the rim (inside, it hit the walls in every plan):
        # TCP 5 mm above the rim puts the 60 mm fingers and the object inside the box
        D[:3, 3] = [xy[0], xy[1], max(floor + a.place_above_floor + (P[2, 3] - TABLE_TOP_Z), btop + 0.005)]
        drop_in = max(0.05, btop + 0.03 + (P[2, 3] - TABLE_TOP_Z) - D[2, 3])   # from object just above the rim
        return dict(lift=a.lift, D=D, up=drop_in + 0.03, drop_in=drop_in)

    found = None
    for i, g in enumerate(grasps[a.index:], start=a.index):
        for k, P in enumerate(tcp_poses(g, a.deeper)):
            t0 = time.time()
            for yaw in (np.radians([0, 90, -90, 180]) if box is not None else (0.0,)):
                task = build_task(node, P, a.approach, a.scale, chain_for(P, yaw))
                ok = bool(task.plan(10)) and len(task.solutions) > 0
                if ok and min(joint_swing(sv) for sv in task.solutions) <= a.max_base_turn:
                    break   # this drop yaw needs no flip
            tilt = np.degrees(np.arccos(np.clip(-P[2, 2], -1, 1)))
            line = (f"grasp #{i + 1}{' (xoay 180)' if k else ''}: diem {g['score']:.3f}, mo {g['width'] * 1000:.0f} mm, "
                    f"sau {g.get('depth', 0) * 1000:.0f}+{a.deeper * 1000:.0f} mm, TCP ({P[0, 3] * 1000:.0f}, {P[1, 3] * 1000:.0f}, "
                    f"{P[2, 3] * 1000:.0f}) mm, nghieng {tilt:.0f} deg -> ")
            if not ok:
                print(line + f"khong ({time.time() - t0:.1f} s)")
                continue
            # 2026-11-01: the cheapest plan swung the base ~300 deg (j1 132.9 -> -173.3), user said no
            sols = [(sv, joint_swing(sv)) for sv in task.solutions]
            ok_sols = [x for x in sols if x[1] <= a.max_base_turn]
            if not ok_sols:
                best = min(sols, key=lambda x: x[1])[0]
                ends = [np.degrees(t.joint_trajectory.points[-1].positions[t.joint_trajectory.joint_names.index("j1")])
                        for t in trajectories(best) if "j1" in t.joint_trajectory.joint_names]
                print(line + f"{len(sols)} ke hoach -> loai (xoay de it nhat {min(x[1] for x in sols):.0f} deg > {a.max_base_turn:.0f}; "
                      f"j1 cuoi tung doan: {', '.join(f'{v:.0f}' for v in ends)})")
                continue
            sol, swing = min(ok_sols, key=lambda x: x[0].cost)
            line += f"{len(task.solutions)} ke hoach, chi phi {sol.cost:.2f}, xoay de {swing:.0f} deg"
            local = width_between_jaws(res, P)
            if local is not None:
                gap, extent, n_in, n_hit = local
                # the jaws push the part to the middle and stop at its thickness (extent): it must be
                # thicker than the ~30 mm the jaws can close to, and the first touch must fit the opening
                if n_in < 5 or n_hit > 20 or extent < 33 or gap > 84:
                    print(line + f" -> loai (diem giua ma {n_in}, cho ma {n_hit}, phan kep day {extent:.0f} mm)")
                    continue
                line += f", phan kep day {extent:.0f} mm"
            off = 1000 * np.linalg.norm(P[:2, 3] - centre) if centre is not None else 0.0
            ease = tilt + off + sol.cost
            print(line + f", lech tam {off:.0f} mm -> do kho {ease:.0f}")
            if found is None or ease < found[4]:
                found = (i, task, sol, P, ease, yaw)
    if not found:
        print("Khong lap duoc ke hoach cho grasp nao (va cham voi ban/vat can, ngoai tam). Khong chay.")
        return
    i, task, sol, P, _, drop_yaw = found
    trajs = trajectories(sol)
    dur = sum(t.joint_trajectory.points[-1].time_from_start.sec + t.joint_trajectory.points[-1].time_from_start.nanosec * 1e-9
              for t in trajs)
    msg = DisplayTrajectory()
    msg.trajectory = trajs
    msg.trajectory_start = RobotState()
    msg.trajectory_start.joint_state.name = list(joints)
    msg.trajectory_start.joint_state.position = list(joints.values())
    for _ in range(3):
        display.publish(msg)
        time.sleep(0.3)
    q_end = trajs[-1].joint_trajectory.points[-1].positions
    names = trajs[-1].joint_trajectory.joint_names
    print(f"Ke hoach grasp #{i + 1}: {len(trajs)} doan (di toi diem cho {a.approach * 1000:.0f} mm truoc vat, roi tien thang vao"
          + (f", nhac {a.lift * 100:.0f} cm, mang toi hop, ha, nhac, ve HOME" if box is not None else "") + "), "
          f"{dur:.1f} s, chi phi {sol.cost:.2f}")
    print("Khop cuoi (deg): " + ", ".join(f"{n} {np.degrees(v):.1f}" for n, v in zip(names, q_end)))
    print("Kiem tra diem (truoc khi chay):")
    check_points(P, res)
    obj_mm = None
    if a.pick:
        local = width_between_jaws(res, P)
        if local is not None:
            gap, extent, n_in, n_hit = local
            print(f"  diem vat giua 2 ma: {n_in}, vat day {extent:.0f} mm theo huong kep, ma cham vat khi con {gap:.0f} mm; "
                  f"diem vat ngay cho ma: {n_hit}")
            if n_in < 5:
                print("  ! khong co diem vat giua 2 ma (grasp truot khoi vat). Khong chay.")
                return
            if n_hit > 20:
                print("  ! ma se dam vao vat khi tien vao. Khong chay.")
                return
            w = extent / 1000     # the jaws centre the part and stop at its thickness
        else:   # older grasp_once result: whole box
            w = object_width_along(res, P[:3, 0])      # grasp_frame x = closing direction
        if w is None:
            print("  khong do duoc be rong vat, se dong cham vat (grip).")
        else:
            from gripper import CLOSED_MM, OPEN_MM, pos_for_width
            obj_mm = w * 1000
            target = max(CLOSED_MM, obj_mm - a.squeeze * 1000)
            pt = pos_for_width(obj_mm)
            print(f"  phan vat bi kep day: {obj_mm:.0f} mm -> cham vat o pos {pt} ({pt * 0.24:.1f} deg), "
                  f"kep toi {pt * 0.24 - a.grip_deg:.1f} deg (them {a.grip_deg:.0f} deg; chua cham thi dong tiep toi khi ket)")
            if obj_mm > OPEN_MM - 4:
                print(f"  ! vat rong hon do mo {OPEN_MM:.0f} mm: ngon se cham vat khi tien vao. Khong chay.")
                return
            if obj_mm < CLOSED_MM + 2:
                print(f"  ! vat hep hon {CLOSED_MM:.0f} mm: kep khong khep toi. Khong chay.")
                return
    if box is not None:
        drop_z = chain_for(P, drop_yaw)["D"][2, 3]
        if len(trajs) != 7:
            print(f"  ! ke hoach co {len(trajs)} doan, can 7 (cho, vao, nhac, mang, ha, nhac, HOME). Khong chay.")
            return
        dx, dy = chain_for(P, drop_yaw)["D"][:2, 3] * 1000
        print(f"  hop dung: tam ({box['center'][0] * 1000:.0f}, {box['center'][1] * 1000:.0f}) mm, tha tai ({dx:.0f}, {dy:.0f}) mm, "
              f"{box['size'][0] * 1000:.0f} x {box['size'][1] * 1000:.0f} x {box['size'][2] * 1000:.0f} mm, mieng z {btop * 1000:.0f} mm "
              f"-> ha vao trong, day vat cach day hop {a.place_above_floor * 1000:.0f} mm roi tha (TCP z {drop_z * 1000:.0f} mm), roi nhac len va ve cho")
    print("Xem duong di trong RViz (MotionPlanning > Planned Path). "
          + (f"Sau do: kep vat roi nhac len {a.lift * 100:.0f} cm." if a.pick else "Gripper KHONG kep."))
    if a.dry_run:
        print("(dry-run: khong chay)")
        return

    from moveit_msgs.action import ExecuteTrajectory
    from rclpy.action import ActionClient
    from tf2_ros import Buffer, TransformListener
    tf = Buffer()
    TransformListener(tf, pub_node)
    exe = ActionClient(pub_node, ExecuteTrajectory, "/execute_trajectory")

    def run(traj):
        """Execute one planned segment through move_group; True when it finished OK."""
        if not exe.wait_for_server(timeout_sec=10):
            print("khong co /execute_trajectory")
            return False
        fut = exe.send_goal_async(ExecuteTrajectory.Goal(trajectory=traj))
        while not fut.done():
            time.sleep(0.05)
        if not fut.result().accepted:
            return False
        res_fut = fut.result().get_result_async()
        while not res_fut.done():
            time.sleep(0.05)
        code = res_fut.result().result.error_code.val
        if code != 1:
            print(f"  loi thuc thi {code}")
        return code == 1

    def tcp_now():
        from rclpy.time import Time
        t = tf.lookup_transform("base_footprint", TCP_LINK, Time()).transform.translation
        return np.array([t.x, t.y, t.z])

    asked = []

    def ask(q):
        if a.yes:
            print(f"--- {q} -> co", flush=True)
            return True
        # 2026-11-01: the user checked the first real pick step by step; from then on one yes runs it all
        if asked and not a.step:
            print(f"--- {q} -> co", flush=True)
            return True
        asked.append(q)
        if not a.step:
            q = ("Chay ca chuoi (diem cho -> tien vao -> kep -> nhac len"
                 + (" -> mang vao hop -> tha -> ve HOME" if box is not None else "") + ")?")
        # whole line first: run_pregrasp_demo.sh pipes the output through grep, which holds a line until its newline
        print(f">>> {q} [y/N] (go y + Enter)", flush=True)
        return input().strip().lower() == "y"

    if not ask(f"1/4 Di toi diem cho ({a.approach * 1000:.0f} mm truoc vat)?"):
        print("Khong chay.")
        return
    if a.pick:
        from gripper import Gripper, width_mm
        grip = Gripper()
        p = grip.open()     # the jaws must be open before going around the object
        print(f"Gripper mo: pos {p} (~{width_mm(p):.0f} mm)")
    if not run(trajs[0]):
        print("Dung.")
        return
    t = tcp_now()
    d = P[:3, 3] - t
    along = d @ P[:3, 2]
    print(f"Diem cho: TCP ({t[0] * 1000:.0f}, {t[1] * 1000:.0f}, {t[2] * 1000:.0f}) mm; con {along * 1000:.0f} mm theo huong tien vao, "
          f"lech ngang {np.linalg.norm(d - along * P[:3, 2]) * 1000:.0f} mm so voi duong tien vao")
    if np.linalg.norm(d - along * P[:3, 2]) > 0.010:     # no one checks by eye when asked only once
        print("Lech duong tien vao > 10 mm: dung o diem cho, khong tien vao.")
        return
    if not ask("2/4 Nhin robot: 2 ma dung hai ben vat? Tien vao?"):
        print("Dung o diem cho.")
        return
    if len(trajs) > 1 and not run(trajs[1]):
        print("Dung.")
        return
    t = tcp_now()
    print(f"Da toi: TCP ({t[0] * 1000:.0f}, {t[1] * 1000:.0f}, {t[2] * 1000:.0f}) mm, lech dich {np.linalg.norm(t - P[:3, 3]) * 1000:.0f} mm")
    if np.linalg.norm(t - P[:3, 3]) > 0.010:
        print("Lech dich > 10 mm: khong kep.")
        return
    if not a.pick:
        print("Gripper dang mo quanh vat, chua kep.")
        return
    if not ask("3/4 Kep vat?"):
        print("Khong kep, gripper van mo.")
        return
    if obj_mm is not None:      # camera width -> touching angle, then --grip-deg further closed
        contact, held, touch = grip.grip_measured(obj_mm, a.grip_deg)
        print(f"Kep: tinh cham vat o pos {touch} ({touch * 0.24:.1f} deg), dich +{a.grip_deg:.0f} deg; "
              f"servo dung o pos {contact} ({contact * 0.24:.1f} deg)")
    else:
        contact, held = grip.grip(squeeze_mm=a.squeeze * 1000)   # close to contact, then squeeze (force)
    if held:
        print(f"Kep vat tai pos {contact} (~{width_mm(contact):.0f} mm giua 2 ma"
              + (f", vat do duoc {obj_mm:.0f} mm)" if obj_mm else ")"))
    else:
        # 2026-11-02: the stand was held both times the servo reached ~pos 3 (the printed fingers
        # bend): the servo position does not tell whether the object is held -> carry on anyway
        print(f"Servo khep toi pos {contact} (~{width_mm(contact):.0f} mm) khong bi ket; ngon nhua co the uon -> van mang di.")
    if not ask(f"4/4 Nhac len {a.lift * 100:.0f} cm?"):
        print("Khong nhac. Robot dung yen, gripper van kep.")
        return
    if box is not None:
        # the rest of the planned chain, shown in RViz before anything moved
        for name, traj in zip(["Nhac len", "Mang toi tren hop", "Ha xuong"], trajs[2:5]):
            if not run(traj):
                print(f"{name}: dung. Gripper van kep: tha bang tools/gripper.py open (do vat).")
                return
            print(f"{name}: OK")
        try:
            p = grip.open()
            print(f"Tha vat vao hop: gripper pos {p}")
        except RuntimeError as e:     # too hot to move: free the jaws, the object drops in
            grip.free()
            print(f"{e} -> nha torque, vat roi vao hop")
        for name, traj in zip(["Nhac khoi hop", "Ve HOME"], trajs[5:7]):
            if not run(traj):
                print(f"{name}: dung.")
                return
            print(f"{name}: OK")
        return
    lift = build_lift(node, a.lift, a.scale)
    if not (lift.plan(5) and len(lift.solutions) > 0):
        print("Khong lap duoc ke hoach nhac len (va cham?). Robot dung yen, gripper van kep.")
        return
    err = lift.execute(min(lift.solutions, key=lambda s: s.cost))
    print(f"Nhac len {a.lift * 100:.0f} cm: {'OK' if err.val == 1 else f'loi {err.val}'}"
          + (" -- dang giu vat." if held else ""))
    pub_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
    # Skip interpreter teardown: destroying the MTC task after the rclcpp context
    # aborts ("terminate called without an active exception") and dumps core.
    sys.stdout.flush()
    os._exit(0)
