from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # config:=tabletop_perception_real.yaml for the real robot
    config = PathJoinSubstitution(
        [FindPackageShare("fairino_fr3_manipulation"), "config", LaunchConfiguration("config")]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value="tabletop_perception.yaml"),
            Node(
                package="fairino_fr3_manipulation",
                executable="tabletop_perception.py",
                name="tabletop_perception",
                parameters=[config],
                output="screen",
            ),
        ]
    )
