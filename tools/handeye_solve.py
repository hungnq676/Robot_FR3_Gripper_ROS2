#!/usr/bin/env python3
"""Solve the eye-to-hand calibration from tools/tag_check.py samples.

1. Drop flipped tag poses: for each pair the angle the tag turned seen by the
   camera must equal the angle the J5 housing turned (no extrinsics needed);
   samples disagreeing by > LIMIT_DEG with most others are removed.
2. Closed form (cv2.calibrateHandEye, best of 5 methods by tag-on-wrist spread).
3. Joint least squares on X = T_base_optical and Y = T_wrist2_tag over
     - tag-corner reprojection error (px, sigma 0.3 px), and
     - tag-centre depth (optical z) against the depth reading (sigma 5 mm).
   The first calibration (6 samples, one height) had the depth axis almost
   unobserved: reprojection 0.48 px yet the table came out 10-13 cm low.
4. Leave-one-out: re-solve without each sample and predict it, so the reported
   error is on data the solve did not see.

Writes camera.HANDEYE_FILE (backs up the previous one).
Usage: handeye_solve.py [samples.npz] [--no-depth] [--dry-run]
"""
import datetime
import os
import shutil
import sys

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

import camera

LIMIT_DEG = 6.0
SIGMA_PX = 0.3
SIGMA_DEPTH = 0.005
OUT = camera.HANDEYE_FILE

args = [a for a in sys.argv[1:] if not a.startswith("--")]
use_depth = "--no-depth" not in sys.argv
dry = "--dry-run" in sys.argv
src = os.path.expanduser(args[0] if args else camera.SAMPLES_FILE)
d = np.load(src)
Tb_all, Tc_all, C_all = d["base_gripper"], d["cam_tag"], d["corners"]
Z_all = d["tag_depth"] if "tag_depth" in d.files else np.full(len(Tb_all), np.nan)
K, D, size = d["K"], d["D"], float(d["tag_size"])
n = len(Tb_all)


def rot_angle(R):
    return np.degrees(np.linalg.norm(Rotation.from_matrix(R).as_rotvec()))


# 1. flipped samples
bad = np.zeros((n, n), bool)
for i in range(n):
    for j in range(i + 1, n):
        a_r = rot_angle(Tb_all[i][:3, :3].T @ Tb_all[j][:3, :3])
        a_c = rot_angle(Tc_all[i][:3, :3].T @ Tc_all[j][:3, :3])
        bad[i, j] = bad[j, i] = abs(a_c - a_r) > LIMIT_DEG
keep = [i for i in range(n) if bad[i].sum() <= (n - 1) / 2]
print(f"{n} mau, giu {len(keep)}, bo {n - len(keep)} mau lat: {[i + 1 for i in range(n) if i not in keep]}")
Tb, Tc, C, Z = Tb_all[keep], Tc_all[keep], C_all[keep], Z_all[keep]
print(f"depth tag co cho {np.isfinite(Z).sum()}/{len(Z)} mau; dung depth: {use_depth}")

h = size / 2
OBJ = np.array([[-h, h, 0, 1], [h, h, 0, 1], [h, -h, 0, 1], [-h, -h, 0, 1]], float).T


def to_T(v):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(v[:3]).as_matrix()
    T[:3, 3] = v[3:]
    return T


def to_v(T):
    return np.r_[Rotation.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3]]


def closed_form(idx):
    R_b2g = [np.linalg.inv(Tb[i])[:3, :3] for i in idx]
    t_b2g = [np.linalg.inv(Tb[i])[:3, 3] for i in idx]
    R_t2c = [Tc[i][:3, :3] for i in idx]
    t_t2c = [Tc[i][:3, 3] for i in idx]
    best = None
    for m in (cv2.CALIB_HAND_EYE_TSAI, cv2.CALIB_HAND_EYE_PARK, cv2.CALIB_HAND_EYE_HORAUD,
              cv2.CALIB_HAND_EYE_ANDREFF, cv2.CALIB_HAND_EYE_DANIILIDIS):
        try:
            R, t = cv2.calibrateHandEye(R_b2g, t_b2g, R_t2c, t_t2c, method=m)
        except cv2.error:
            continue
        X = np.eye(4); X[:3, :3] = R; X[:3, 3] = t.ravel()
        Ys = [np.linalg.inv(Tb[i]) @ X @ Tc[i] for i in idx]
        spread = np.linalg.norm(np.std([y[:3, 3] for y in Ys], axis=0))
        if best is None or spread < best[0]:
            Y = np.eye(4)
            Y[:3, :3] = Rotation.from_matrix([y[:3, :3] for y in Ys]).mean().as_matrix()
            Y[:3, 3] = np.mean([y[:3, 3] for y in Ys], axis=0)
            best = (spread, X, Y)
    return best[1], best[2]


# --fit-focal: also estimate a common scale f on fx, fy (cx, cy kept). (OAK,
# 2026-10-30:) the RGB stream is 1080P ISP-scaled 2/3 to 1280x720; IR depth read ~7% farther than
# the PnP distance on every sample (2026-10-30), i.e. camera_info fx may be off.
fit_focal = "--fit-focal" in sys.argv


def K_scaled(f):
    Kf = K.copy()
    Kf[0, 0] *= f
    Kf[1, 1] *= f
    return Kf


def predict(X, Y, i, f=1.0):
    T_opt_tag = np.linalg.inv(X) @ Tb[i] @ Y
    pc = (T_opt_tag @ OBJ)[:3]
    uv, _ = cv2.projectPoints(pc.T, np.zeros(3), np.zeros(3), K_scaled(f), D)
    return uv.reshape(4, 2), T_opt_tag[2, 3]


def residuals(v, idx, depth):
    X, Y = to_T(v[:6]), to_T(v[6:12])
    f = v[12] if len(v) > 12 else 1.0
    r = []
    for i in idx:
        uv, z = predict(X, Y, i, f)
        r.extend(((uv - C[i]) / SIGMA_PX).ravel())
        if depth and np.isfinite(Z[i]):
            r.append((z - Z[i]) / SIGMA_DEPTH)
    return np.array(r)


def solve(idx, depth):
    X0, Y0 = closed_form(idx)
    v0 = np.r_[to_v(X0), to_v(Y0)] if not fit_focal else np.r_[to_v(X0), to_v(Y0), 1.0]
    sol = least_squares(residuals, v0, args=(idx, depth), loss="huber", f_scale=3.0)
    return to_T(sol.x[:6]), to_T(sol.x[6:12]), (sol.x[12] if fit_focal else 1.0)


def report(X, Y, idx, f=1.0):
    px = [np.linalg.norm(predict(X, Y, i, f)[0] - C[i], axis=1).mean() for i in idx]
    dz = [predict(X, Y, i, f)[1] - Z[i] for i in idx if np.isfinite(Z[i])]
    return float(np.sqrt(np.mean(np.square(px)))), (float(np.sqrt(np.mean(np.square(dz)))) if dz else float("nan"))


# Independent hint: IR depth / PnP z of the same tag (PnP uses camera_info fx).
ratio = [Z[i] / Tc[i][2, 3] for i in range(len(Tb)) if np.isfinite(Z[i])]
if ratio:
    print(f"depth / PnP z: trung vi {np.median(ratio):.3f} (min {np.min(ratio):.3f}, max {np.max(ratio):.3f})")

all_idx = list(range(len(Tb)))
X, Y, FOC = solve(all_idx, use_depth)
rms_px, rms_dz = report(X, Y, all_idx, FOC)
print(f"\nKet qua ({len(all_idx)} mau): chieu RMS {rms_px:.2f} px, depth RMS {rms_dz * 1000:.1f} mm")
if fit_focal:
    print(f"  he so tieu cu uoc luong: {FOC:.4f} -> fx = {K[0, 0] * FOC:.1f} (camera_info: {K[0, 0]:.1f})")
print(f"  camera (optical) trong base: {X[:3, 3].round(4).tolist()} m")
print(f"  tag tren wrist2: {(Y[:3, 3] * 1000).round(1).tolist()} mm (CAD: [0, -50.2, 102])")

# Leave-one-out
loo_px, loo_dz, loo_pos = [], [], []
for k in all_idx:
    rest = [i for i in all_idx if i != k]
    if len(rest) < 4:
        break
    Xk, Yk, Fk = solve(rest, use_depth)
    uv, z = predict(Xk, Yk, k, Fk)
    loo_px.append(np.linalg.norm(uv - C[k], axis=1).mean())
    if np.isfinite(Z[k]):
        loo_dz.append(z - Z[k])
    loo_pos.append(np.linalg.norm(Xk[:3, 3] - X[:3, 3]))
if loo_px:
    dz_txt = (f"depth TB {np.mean(np.abs(loo_dz)) * 1000:.1f} mm (max {np.max(np.abs(loo_dz)) * 1000:.1f}), "
              if loo_dz else "depth: khong co, ")
    print(f"Leave-one-out: chieu TB {np.mean(loo_px):.2f} px (max {np.max(loo_px):.2f}), {dz_txt}"
          f"vi tri camera dao dong toi {np.max(loo_pos) * 1000:.1f} mm")

T_base_cambase = X @ d["T_optical_camera_base"]
rot = Rotation.from_matrix(T_base_cambase[:3, :3])
result = {
    "parent_frame": "base_footprint",
    "child_frame": camera.BASE_FRAME,
    "xyz": [round(float(v), 5) for v in T_base_cambase[:3, 3]],
    "rpy": [round(float(v), 5) for v in rot.as_euler("xyz")],
    "quaternion_xyzw": [round(float(v), 6) for v in rot.as_quat()],
    "method": "closed-form + LSQ reprojection" + (" + IR depth" if use_depth else ""),
    "samples": len(all_idx),
    "reprojection_rms_px": round(rms_px, 3),
    "depth_rms_mm": round(rms_dz * 1000, 1) if np.isfinite(rms_dz) else None,
    "leave_one_out_px_mean": round(float(np.mean(loo_px)), 3) if loo_px else None,
    "leave_one_out_depth_mm_mean": round(float(np.mean(np.abs(loo_dz))) * 1000, 1) if loo_dz else None,
    "tag_in_gripper": {"frame": "wrist2_link", "xyz": [round(float(v), 4) for v in Y[:3, 3]]},
    "focal_scale": round(float(FOC), 5),
    "fx_fy_used": [round(float(K[0, 0] * FOC), 2), round(float(K[1, 1] * FOC), 2)],
    "calibrated_at": datetime.datetime.now().isoformat(timespec="seconds"),
}
print(f"base_footprint -> {camera.BASE_FRAME}: xyz={result['xyz']} rpy={result['rpy']}")
if dry:
    print("(dry-run: khong ghi file)")
else:
    if os.path.exists(OUT):
        shutil.copy(OUT, OUT.replace(".yaml", f"_backup_{datetime.datetime.now():%Y%m%d_%H%M%S}.yaml"))
    with open(OUT, "w") as f:
        yaml.safe_dump(result, f, sort_keys=False)
    print(f"-> {OUT}")
