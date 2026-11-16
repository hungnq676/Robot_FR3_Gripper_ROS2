from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    config = PathJoinSubstitution(
        [FindPackageShare("fairino_fr3_manipulation"), "config", "pick_and_carry.yaml"]
    )
    moveit_config = (
        MoveItConfigsBuilder(
            "fairino_fr3", package_name="fairino_fr3_moveit_config"
        )
        .robot_description(
            mappings={
                "hardware_type": "mock",
                "use_gazebo": "false",
                "mount_to_world": "false",
            }
        )
        .robot_description_semantic(file_path="config/fairino_fr3.srdf")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="fairino_fr3_manipulation",
                executable="pick_and_carry",
                name="pick_and_carry",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    config,
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )
