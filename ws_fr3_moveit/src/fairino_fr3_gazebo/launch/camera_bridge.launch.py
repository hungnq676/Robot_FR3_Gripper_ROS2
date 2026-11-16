import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Nominal pose of the OAK-D Pro W in world (x y z roll pitch yaw); keep in sync
# with the oak_d_pro_w include in worlds/fr3_workspace.sdf. Used only until the
# AprilTag calibration (calibrate_camera.launch.py) has written EXTRINSICS_FILE.
OAK_POSE = ("0.52", "0.64", "1.95", "1.57079632679", "1.10", "0")
EXTRINSICS_FILE = os.path.expanduser("~/.ros/fr3_oak_extrinsics.yaml")


def static_tf(name, parent, child, xyz=("0", "0", "0"), rpy=("0", "0", "0")):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            "--x", xyz[0], "--y", xyz[1], "--z", xyz[2],
            "--roll", rpy[0], "--pitch", rpy[1], "--yaw", rpy[2],
            "--frame-id", parent, "--child-frame-id", child,
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )


def camera_base_tf(context):
    use_calibration = LaunchConfiguration("use_calibration").perform(context) == "true"
    if use_calibration and os.path.exists(EXTRINSICS_FILE):
        with open(EXTRINSICS_FILE) as stream:
            calibration = yaml.safe_load(stream)
        xyz = [str(v) for v in calibration["xyz"]]
        rpy = [str(v) for v in calibration["rpy"]]
        message = (f"OAK extrinsics from AprilTag calibration ({calibration['calibrated_at']}): "
                   f"xyz={calibration['xyz']} rpy={calibration['rpy']}")
    else:
        xyz, rpy = OAK_POSE[:3], OAK_POSE[3:]
        message = ("OAK extrinsics: NOMINAL pose (no calibration used); run "
                   "'ros2 launch fairino_fr3_manipulation calibrate_camera.launch.py'")
    return [
        LogInfo(msg=message),
        static_tf("oak_base_tf", "world", "oak-d-base-frame", xyz, rpy),
    ]


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("fairino_fr3_gazebo"), "config", "camera_bridges.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_calibration", default_value="false",
                description="Use ~/.ros/fr3_oak_extrinsics.yaml for world -> oak-d-base-frame",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="camera_bridge",
                parameters=[{"config_file": config, "use_sim_time": True}],
                output="screen",
            ),
            # Same frame tree as luxonis/depthai-ros depthai_descriptions, so the
            # perception code runs unchanged on the real camera.
            OpaqueFunction(function=camera_base_tf),
            static_tf("oak_rgb_tf", "oak-d-base-frame", "oak_rgb_camera_frame"),
            static_tf(
                "oak_rgb_optical_tf", "oak_rgb_camera_frame", "oak_rgb_camera_optical_frame",
                rpy=("-1.5707963", "0", "-1.5707963"),
            ),
        ]
    )
