from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    object_config = PathJoinSubstitution(
        [FindPackageShare("fairino_fr3_manipulation"), "config", "object.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="fairino_fr3_manipulation",
                executable="object_description_publisher",
                name="object_description_publisher",
                output="screen",
                parameters=[object_config, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
