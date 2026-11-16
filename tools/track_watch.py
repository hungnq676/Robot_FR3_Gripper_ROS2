#!/usr/bin/env python3
"""Log every real-arm move: duration, max tracking error per joint, peak
reference speed, how far each joint really moved, and the controller result.
Read-only. Usage: track_watch.py <seconds> <logfile>"""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 1200
log = open(sys.argv[2], "a", buffering=1) if len(sys.argv) > 2 else None
def say(s):
    print(s, flush=True)
    if log: log.write(s + "\n")

rclpy.init(); n = Node("track_watch"); rows = []; move = []; count = [0]
def cb(m):
    if len(m.error.positions) != 6: return
    v = np.array(m.reference.velocities) if len(m.reference.velocities) == 6 else np.zeros(6)
    r = (time.time(), np.array(m.error.positions), v, np.array(m.feedback.positions))
    if np.abs(v).max() > 1e-4: move.append(r)
n.create_subscription(JointTrajectoryControllerState, "/arm_controller/controller_state", cb, 50)
say(f"track_watch: ghi trong {dur/60:.0f} phut")
t0 = time.time()
while time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.1)
    if move and time.time() - move[-1][0] > 1.5:
        count[0] += 1
        E = np.array([r[1] for r in move]); V = np.array([r[2] for r in move]); P = np.array([r[3] for r in move])
        worst = np.degrees(np.abs(E).max())
        verdict = "OK" if worst < 1.5 else ("CANH BAO" if worst < 2.8 else "SAP BI NGAT / BI NGAT")
        say(f"[lan {count[0]}] {time.strftime('%H:%M:%S')} {move[-1][0]-move[0][0]:.1f}s | lech max (deg) {np.degrees(np.abs(E).max(0)).round(2).tolist()} | "
            f"toc do max {np.degrees(np.abs(V).max()):.1f} deg/s | da quay {np.degrees(P.max(0)-P.min(0)).round(1).tolist()} -> {verdict}")
        move.clear()
