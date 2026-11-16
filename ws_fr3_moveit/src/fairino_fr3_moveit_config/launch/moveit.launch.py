from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(launch_file):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fairino_fr3_moveit_config"),
                    "launch",
                    launch_file,
                ]
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )


def generate_launch_description():
    return LaunchDescription(
        [
            include("move_group.launch.py"),
            include("moveit_rviz.launch.py"),
        ]
    )
