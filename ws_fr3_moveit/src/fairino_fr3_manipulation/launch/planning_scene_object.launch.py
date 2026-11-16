from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="fairino_fr3_manipulation",
                executable="planning_scene_object_manager",
                name="planning_scene_object_manager",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
