from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("fairino_fr3_manipulation"), "config", "camera_calibration.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="fairino_fr3_manipulation",
                executable="calibrate_camera_apriltag.py",
                name="camera_calibration_apriltag",
                parameters=[config],
                output="screen",
            )
        ]
    )
