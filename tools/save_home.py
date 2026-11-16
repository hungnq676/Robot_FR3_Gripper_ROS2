#!/usr/bin/env python3
"""Save where the arm stands now as HOME (~/.ros/fr3_home.yaml, degrees): the pose the
pick-into-box scenario starts from, looks at the table from (camera view clear), and
returns to. Move the arm there first (pendant / RViz). Moves nothing.

Usage: save_home.py
"""
import os
import time

import numpy as np
import rclpy
import yaml
from sensor_msgs.msg import JointState

HOME_FILE = os.path.expanduser("~/.ros/fr3_home.yaml")
JOINTS = ["j1", "j2", "j3", "j4", "j5", "j6"]


def main():
    rclpy.init()
    node = rclpy.create_node("save_home")
    got = {}
    node.create_subscription(JointState, "/joint_states", lambda m: got.update(zip(m.name, m.position)), 10)
    t0 = time.time()
    while not all(j in got for j in JOINTS) and time.time() - t0 < 5:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not all(j in got for j in JOINTS):
        raise SystemExit("khong doc duoc /joint_states (run_real.sh da chay chua?)")
    home = {j: round(float(np.degrees(got[j])), 2) for j in JOINTS}
    if os.path.exists(HOME_FILE):
        os.replace(HOME_FILE, HOME_FILE.replace(".yaml", time.strftime("_cu_%Y%m%d_%H%M%S.yaml")))
    yaml.safe_dump(dict(home_deg=home, saved_at=time.strftime("%Y-%m-%dT%H:%M:%S")), open(HOME_FILE, "w"), sort_keys=False)
    print(f"HOME (deg): {home}\n-> {HOME_FILE}")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
