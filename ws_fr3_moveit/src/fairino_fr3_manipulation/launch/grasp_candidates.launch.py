from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    grasp_config = PathJoinSubstitution(
        [FindPackageShare("fairino_fr3_manipulation"), "config", "grasp.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="fairino_fr3_manipulation",
                executable="grasp_pose_generator",
                name="grasp_pose_generator",
                output="screen",
                parameters=[grasp_config, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
