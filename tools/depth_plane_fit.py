#!/usr/bin/env python3
"""Fit the Astra depth error on the (level) table, using the hand-eye as truth.

2026-10-31: with the colour-only hand-eye the Astra depth put the level table
~5 deg tilted, and on the tag samples depth/PnP fell from ~1.07 (left of the
image) to ~1.02 (right): the depth is tilted against the colour camera, so a
single scale cannot fix it. Here every table pixel's true optical z is the
pixel ray (colour K, hand-eye) cut with the plane z = --table-z in
base_footprint; the ratio measured/true is fitted as a + b x + c y over the
normalised pixel (x, y), trimming objects and the robot. The tag samples
(robot FK + hand-eye) are an independent check. Writes camera.DEPTH_MODEL_FILE
(model pixel_ratio, used by camera.correct_depth).

Needs the camera running (run_astra.sh); the table mostly clear.
Usage: depth_plane_fit.py --table-z 0.012 [--frames 30] [--dry-run]
"""
import argparse
import datetime
import os
import warnings

import numpy as np
import rclpy
import yaml
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

import camera


def base_to_optical():
    cal = yaml.safe_load(open(camera.HANDEYE_FILE))
    T_bc = np.eye(4)
    T_bc[:3, :3] = Rotation.from_quat(cal["quaternion_xyzw"]).as_matrix()
    T_bc[:3, 3] = cal["xyz"]
    T_opt_cb = np.load(camera.SAMPLES_FILE)["T_optical_camera_base"]
    return T_bc @ np.linalg.inv(T_opt_cb), cal


def capture(frames):
    rclpy.init()
    node = rclpy.create_node("depth_plane_fit")
    got = {"depth": []}
    q = rclpy.qos.qos_profile_sensor_data
    node.create_subscription(CameraInfo, camera.INFO_TOPIC, lambda m: got.__setitem__("info", m), q)
    node.create_subscription(Image, camera.DEPTH_TOPIC, lambda m: got["depth"].append(
        np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).copy()), q)
    while len(got["depth"]) < frames or "info" not in got:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.try_shutdown()
    D = np.stack(got["depth"][-frames:]).astype(np.float64) / 1000.0
    D[D <= 0] = np.nan
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        return np.nanmedian(D, 0), np.array(got["info"].k).reshape(3, 3)


def table_report(name, z, K, X, u, v):
    """Height and tilt in base of the table pixels with depth z."""
    P = np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z], 1)
    B = P @ X[:3, :3].T + X[:3, 3]
    A = np.c_[B[:, 0], B[:, 1], np.ones(len(B))]
    c, *_ = np.linalg.lstsq(A, B[:, 2], rcond=None)
    print(f"  {name}: mat ban z trung vi {np.median(B[:, 2]) * 1000:+.1f} mm, nghieng "
          f"{np.degrees(np.arctan(np.hypot(c[0], c[1]))):.2f} deg, z p5..p95 "
          f"{np.percentile(B[:, 2], 5) * 1000:+.1f}..{np.percentile(B[:, 2], 95) * 1000:+.1f} mm")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--table-z", type=float, required=True, help="touched table top in base_footprint, m")
    ap.add_argument("--frames", type=int, default=30)
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    X, cal = base_to_optical()
    depth, K = capture(a.frames)
    h, w = depth.shape
    v, u = np.mgrid[0:h:2, 0:w:2]
    v, u, zm = v.ravel().astype(float), u.ravel().astype(float), depth[::2, ::2].ravel()
    x, y = (u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1]
    # Pixel ray in base, cut with z = table_z: true optical z of the table there.
    d_base = np.stack([x, y, np.ones_like(x)], 1) @ X[:3, :3].T
    zt = (a.table_z - X[2, 3]) / d_base[:, 2]
    ok = np.isfinite(zm) & (zm > 0.3) & (zt > 0.3) & (zt < 2.0)
    r = zm / zt
    sel = ok & (np.abs(r - np.nanmedian(r[ok])) < 0.08)
    for _ in range(5):                                   # trim objects, robot, table edge
        A = np.c_[np.ones(sel.sum()), x[sel], y[sel]]
        coef, *_ = np.linalg.lstsq(A, r[sel], rcond=None)
        res = r - (coef[0] + coef[1] * x + coef[2] * y)
        sig = 1.4826 * np.median(np.abs(res[sel]))
        sel = ok & (np.abs(res) < max(3 * sig, 0.004))
    fa, fb, fc = coef
    err_mm = res[sel] * zt[sel] * 1000
    print(f"{sel.sum()} diem mat ban (anh {w}x{h}), z that {zt[sel].min():.3f}..{zt[sel].max():.3f} m")
    print(f"ti so depth/that = {fa:.4f} + {fb:+.4f} x + {fc:+.4f} y   (x, y: pixel chuan hoa)")
    print(f"  tren khung hinh: trai {fa - fb * K[0, 2] / K[0, 0]:.4f}, phai {fa + fb * (w - K[0, 2]) / K[0, 0]:.4f}, "
          f"tren {fa - fc * K[1, 2] / K[1, 1]:.4f}, duoi {fa + fc * (h - K[1, 2]) / K[1, 1]:.4f}")
    print(f"  du sau khi sua tren mat ban: RMS {np.sqrt(np.mean(err_mm ** 2)):.1f} mm, p95 {np.percentile(np.abs(err_mm), 95):.1f} mm")
    model = {"model": "pixel_ratio", "a": round(float(fa), 6), "b": round(float(fb), 6), "c": round(float(fc), 6),
             "fx": float(K[0, 0]), "fy": float(K[1, 1]), "cx": float(K[0, 2]), "cy": float(K[1, 2])}
    zc = camera.correct_depth(zm[sel], model, u[sel], v[sel])
    table_report("truoc", zm[sel], K, X, u[sel], v[sel])
    table_report("sau  ", zc, K, X, u[sel], v[sel])

    # Independent check: tag centres, true optical z from robot FK + hand-eye.
    d = np.load(camera.SAMPLES_FILE)
    tag = np.r_[cal["tag_in_gripper"]["xyz"], 1.0]
    before, after = [], []
    for Tb, C, zd in zip(d["base_gripper"], d["corners"], d["tag_depth"]):
        if not np.isfinite(zd):
            continue
        z_true = (np.linalg.inv(X) @ Tb @ tag)[2]
        cu, cv = C.mean(0)
        before.append(zd - z_true)
        after.append(camera.correct_depth(zd, model, cu, cv) - z_true)
    if before:
        b, f = np.array(before) * 1000, np.array(after) * 1000
        print(f"Kiem tra tren {len(b)} mau tag (khong dung de fit): depth - that truoc TB {b.mean():+.1f} "
              f"(RMS {np.sqrt(np.mean(b ** 2)):.1f}) mm, sau TB {f.mean():+.1f} (RMS {np.sqrt(np.mean(f ** 2)):.1f}) mm")
    if a.dry_run:
        print("(dry-run: khong ghi file)")
        return
    model.update({"formula": "Z_measured / z_true = a + b*(u-cx)/fx + c*(v-cy)/fy",
                  "table_z_m": a.table_z, "points": int(sel.sum()),
                  "residual_rms_mm": round(float(np.sqrt(np.mean(err_mm ** 2))), 1),
                  "range_m": [round(float(zt[sel].min()), 3), round(float(zt[sel].max()), 3)],
                  "handeye_calibrated_at": cal.get("calibrated_at"),
                  "fitted_at": datetime.datetime.now().isoformat(timespec="seconds")})
    if os.path.exists(camera.DEPTH_MODEL_FILE):
        os.replace(camera.DEPTH_MODEL_FILE, camera.DEPTH_MODEL_FILE.replace(
            ".yaml", f"_backup_{datetime.datetime.now():%Y%m%d_%H%M%S}.yaml"))
    yaml.safe_dump(model, open(camera.DEPTH_MODEL_FILE, "w"), sort_keys=False)
    print(f"-> {camera.DEPTH_MODEL_FILE}")


if __name__ == "__main__":
    main()
