#!/usr/bin/env python3
"""Fit the camera depth error against the robot: for every hand-eye sample the
tag centre's true optical z is known from robot FK + hand-eye (X, tag offset),
the IR depth gave Z. Fit Z = a * z_true + b, report residuals and write
the camera depth model (tools/camera.py DEPTH_MODEL_FILE) so perception can undo
it: z_true = (Z - b) / a.

2026-10-30 (OAK): depth read ~7-9% farther than PnP; a pointing test showed heights
7 cm off after a plain /1.077 correction, so a scale-only model was not enough.
Usage: depth_model.py [samples.npz] [handeye.yaml]
"""
import os
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

import camera

args = sys.argv[1:]
d = np.load(os.path.expanduser(args[0] if args else camera.SAMPLES_FILE))
cal = yaml.safe_load(open(os.path.expanduser(args[1] if len(args) > 1 else camera.HANDEYE_FILE)))

T_base_cambase = np.eye(4)
T_base_cambase[:3, :3] = Rotation.from_quat(cal["quaternion_xyzw"]).as_matrix()
T_base_cambase[:3, 3] = cal["xyz"]
X = T_base_cambase @ np.linalg.inv(d["T_optical_camera_base"])     # base -> optical
tag_w2 = np.r_[cal["tag_in_gripper"]["xyz"], 1.0]

z_true, z_meas = [], []
for Tb, Zm in zip(d["base_gripper"], d["tag_depth"]):
    if not np.isfinite(Zm):
        continue
    p_opt = np.linalg.inv(X) @ Tb @ tag_w2
    z_true.append(p_opt[2])
    z_meas.append(Zm)
z_true, z_meas = np.array(z_true), np.array(z_meas)
A = np.c_[z_true, np.ones_like(z_true)]
(a, b), *_ = np.linalg.lstsq(A, z_meas, rcond=None)
res = z_meas - (a * z_true + b)
ratio = z_meas / z_true
print(f"{len(z_true)} mau, z that {z_true.min():.3f}..{z_true.max():.3f} m")
print(f"ti le depth/that: TB {ratio.mean():.4f} (min {ratio.min():.4f}, max {ratio.max():.4f})")
print(f"fit Z = {a:.4f} * z + {b * 1000:+.1f} mm, du sau fit: RMS {np.sqrt(np.mean(res ** 2)) * 1000:.1f} mm, max {np.abs(res).max() * 1000:.1f} mm")
scale_only = z_meas / ratio.mean() - z_true
print(f"(chi he so {ratio.mean():.4f}: du RMS {np.sqrt(np.mean(scale_only ** 2)) * 1000:.1f} mm)")
out = camera.DEPTH_MODEL_FILE
yaml.safe_dump({"model": "affine", "formula": "Z_measured = a * z_true + b (optical z, metres)",
                "a": round(float(a), 5), "b": round(float(b), 5),
                "residual_rms_mm": round(float(np.sqrt(np.mean(res ** 2)) * 1000), 1),
                "samples": int(len(z_true)), "range_m": [round(float(z_true.min()), 3), round(float(z_true.max()), 3)],
                "handeye_calibrated_at": cal.get("calibrated_at")}, open(out, "w"), sort_keys=False)
print(f"-> {out}")
