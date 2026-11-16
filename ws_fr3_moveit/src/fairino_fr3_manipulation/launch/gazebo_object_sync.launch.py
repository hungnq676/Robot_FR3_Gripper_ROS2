import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    share = get_package_share_directory("fairino_fr3_manipulation")
    config = PathJoinSubstitution(
        [FindPackageShare("fairino_fr3_manipulation"), "config", "gazebo_object_sync.yaml"]
    )
    attacher_config = os.path.join(share, "config", "gazebo_grasp_attacher.yaml")
    fsr_config = os.path.join(share, "config", "fsr_emulator.yaml")
    with open(attacher_config) as stream:
        models = yaml.safe_load(stream)["gazebo_grasp_attacher"]["ros__parameters"][
            "graspable_models"]

    # Per-object weld topics (see fairino_fr3.gazebo.xacro) and model poses.
    bridge_args = ["/fr3_demo/gazebo_detach@std_msgs/msg/Empty]gz.msgs.Empty"]
    # Jaw contact sensors -> RFP602 emulator.
    bridge_args += [
        f"/fr3_demo/gz/contacts/{jaw}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts"
        for jaw in ("clamp_1", "clamp_2")
    ]
    remappings = []
    for model in models:
        bridge_args += [
            f"/fr3_demo/gz/attach/{model}@std_msgs/msg/Empty]gz.msgs.Empty",
            f"/fr3_demo/gz/state/{model}@std_msgs/msg/String[gz.msgs.StringMsg",
            f"/model/{model}/pose@geometry_msgs/msg/Pose[gz.msgs.Pose",
        ]
        remappings.append((f"/model/{model}/pose", f"/fr3_demo/gz/pose/{model}"))

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="grasp_weld_bridge",
                arguments=bridge_args,
                remappings=remappings,
                output="screen",
            ),
            Node(
                package="fairino_fr3_manipulation",
                executable="gazebo_grasp_attacher.py",
                name="gazebo_grasp_attacher",
                parameters=[attacher_config],
                output="screen",
            ),
            Node(
                package="fairino_fr3_manipulation",
                executable="fsr_emulator.py",
                name="fsr_emulator",
                parameters=[fsr_config],
                output="screen",
            ),
            Node(
                package="fairino_fr3_manipulation",
                executable="gazebo_object_sync",
                name="gazebo_object_sync",
                parameters=[config, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
        ]
    )
