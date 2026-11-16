from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    demo_config = PathJoinSubstitution(
        [FindPackageShare("fairino_fr3_manipulation"), "config", "attached_object_demo.yaml"]
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
            Node(
                package="fairino_fr3_manipulation",
                executable="attached_object_demo",
                name="attached_object_demo",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {"use_sim_time": True},
                    demo_config,
                ],
            )
        ]
    )
