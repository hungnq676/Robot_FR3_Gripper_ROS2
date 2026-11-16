"""The real camera, in one place: topics, TF frames and calibration files.

Since 2026-10-31 the cell uses the Orbbec Astra Pro Plus (ros2_astra_camera,
run_astra.sh) instead of the OAK-D Pro W, whose colour camera died (NHAT_KY 18).
The OAK names are kept in OAK for the old files and tools/oak_stream.py.
"""
import os

import numpy as np
import yaml

RGB_TOPIC = "/camera/color/image_raw"
INFO_TOPIC = "/camera/color/camera_info"
DEPTH_TOPIC = "/camera/depth/image_raw"            # 16UC1 mm, registered to colour (depth_registration:=true)
OPTICAL_FRAME = "camera_color_optical_frame"
BASE_FRAME = "camera_link"                         # root of the driver's TF tree; hand-eye parent -> this

HANDEYE_FILE = os.path.expanduser("~/.ros/fr3_astra_handeye.yaml")
SAMPLES_FILE = os.path.expanduser("~/.ros/fr3_astra_handeye_samples.npz")
DEPTH_MODEL_FILE = os.path.expanduser("~/.ros/fr3_astra_depth_model.yaml")

OAK = dict(rgb="/oak/rgb/image_raw", info="/oak/rgb/camera_info", depth="/oak/stereo/image_raw",
           optical="oak_rgb_camera_optical_frame", base="oak-d-base-frame",
           handeye="~/.ros/fr3_oak_handeye.yaml", depth_model="~/.ros/fr3_oak_depth_model.yaml")


if os.environ.get("FR3_CAMERA_MODE") == "sim":
    RGB_TOPIC = OAK["rgb"]
    INFO_TOPIC = OAK["info"]
    DEPTH_TOPIC = OAK["depth"]
    OPTICAL_FRAME = OAK["optical"]
    DEPTH_MODEL_FILE = "/tmp/fr3_sim_no_depth_correction.yaml"


def depth_meters(msg):
    """Decode registered ROS depth in metres, respecting row padding."""
    dtype, scale = (np.float32, 1.0) if msg.encoding == "32FC1" else (np.uint16, 0.001)
    dtype = np.dtype(dtype).newbyteorder(">" if msg.is_bigendian else "<")
    return np.ndarray((msg.height, msg.width), dtype=dtype, buffer=bytes(msg.data),
                      strides=(msg.step, dtype.itemsize)).astype(np.float32) * scale


def to_bgr(msg):
    """sensor_msgs/Image (rgb8 from the Astra, bgr8 from the OAK) -> BGR array."""
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
    return img[:, :, ::-1].copy() if msg.encoding == "rgb8" else img.copy()


def load_depth_model():
    """Depth correction fitted by depth_model.py; none yet -> identity."""
    if os.path.exists(DEPTH_MODEL_FILE):
        return yaml.safe_load(open(DEPTH_MODEL_FILE))
    return {"model": "scale", "scale": 1.0, "note": f"{DEPTH_MODEL_FILE} chua co -> khong hieu chinh"}


def correct_depth(z, model, u=None, v=None):
    """Measured depth (m, array or float) -> true depth with a depth model:
    scale (z / scale), affine (Z = a z + b, depth_model.py), pixel_ratio
    (Z / z = a + b x + c y with x, y the normalised pixel, depth_plane_fit.py;
    a whole image needs no u, v, anything else gives its pixel u, v) or the
    old OAK inverse_disparity_bias (1 / (1/z + k), then / scale if given).
    Zeros and NaN (no depth) are left as they are."""
    z = np.asarray(z, dtype=np.float64)
    out = z.copy()
    ok = z > 0
    if model.get("model") == "affine":
        out[ok] = (z[ok] - float(model["b"])) / float(model["a"])
    elif model.get("model") == "pixel_ratio":
        if u is None:
            if z.ndim != 2:
                raise ValueError("pixel_ratio depth model: pass u, v for anything but a whole image")
            v, u = np.mgrid[0:z.shape[0], 0:z.shape[1]]
        x = (np.asarray(u, dtype=np.float64) - model["cx"]) / model["fx"]
        y = (np.asarray(v, dtype=np.float64) - model["cy"]) / model["fy"]
        ratio = np.broadcast_to(model["a"] + model["b"] * x + model["c"] * y, z.shape)
        out[ok] = z[ok] / ratio[ok]
    else:
        if model.get("k_per_m"):
            out[ok] = 1.0 / (1.0 / out[ok] + float(model["k_per_m"]))
        out[ok] = out[ok] / float(model.get("scale", 1.0))
    return out if out.ndim else float(out)
