#!/usr/bin/env python3
"""Plan a Pilz PTP move to given joint angles on the real FR3, show it in RViz
(Planned Path), and execute it ONLY after the user types y. Reuses the planning
and execution code of handeye_guided.py.

Usage: goto_joints.py j1 j2 j3 j4 j5 j6   (degrees)  [--scale 0.3]
"""
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

sys.path.insert(0, __import__("os").path.dirname(__file__))
from handeye_guided import Guided  # noqa: E402


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    scale = float(sys.argv[sys.argv.index("--scale") + 1]) if "--scale" in sys.argv else 0.3
    if "--scale" in sys.argv:
        args.remove(sys.argv[sys.argv.index("--scale") + 1])
    q_goal = np.radians([float(a) for a in args[:6]])

    rclpy.init()
    node = Guided()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    t0 = time.time()
    while node.q is None and time.time() - t0 < 10:
        time.sleep(0.1)
    node.move.wait_for_server(timeout_sec=10)
    node.exe.wait_for_server(timeout_sec=10)

    delta = np.degrees(q_goal - node.q)
    print(f"Dich (deg): {np.degrees(q_goal).round(1).tolist()}")
    print(f"Thay doi (deg): {delta.round(1).tolist()}")
    res = node.plan(q_goal, scale)
    if res is None:
        print("Pilz PTP KHONG lap duoc ke hoach (va cham / ngoai gioi han). Khong chay.")
        return
    pts = res.planned_trajectory.joint_trajectory.points
    dur = pts[-1].time_from_start.sec + pts[-1].time_from_start.nanosec * 1e-9
    for _ in range(3):
        node.show(res)
        time.sleep(0.3)
    print(f"Ke hoach OK: {dur:.1f} s, toc do {scale}. Xem duong di trong RViz (MotionPlanning > Planned Path).")
    ans = input("Go y + Enter de CHAY, phim khac de huy: ").strip().lower()
    if ans != "y":
        print("Huy. Robot khong chay.")
        return
    ok = node.execute(res.planned_trajectory)
    print("Da toi noi." if ok else "CHAY THAT BAI (xem log driver).")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
