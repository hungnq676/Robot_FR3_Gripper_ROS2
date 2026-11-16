from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("fairino_fr3_manipulation"), "config", LaunchConfiguration("config")]
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
        # Pilz PTP is tried first for free-space moves, OMPL is the fallback.
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )
    real = LaunchConfiguration("real")
    is_real = PythonExpression(["'", real, "' == 'true'"])
    return LaunchDescription(
        [
            # real:=true -> wall clock and every motion at velocity_scaling (5 %).
            DeclareLaunchArgument("real", default_value="false"),
            DeclareLaunchArgument("config", default_value="mtc_pick_task.yaml"),
            DeclareLaunchArgument("velocity_scaling", default_value="0.05"),
            Node(
                package="fairino_fr3_manipulation",
                executable="mtc_pick_task_planner",
                name="mtc_pick_task_planner",
                output="screen",
                parameters=[
                    moveit_config.to_dict(), config,
                    {
                        "use_sim_time": PythonExpression(["not ", is_real]),
                        "planner.velocity_scaling": PythonExpression([
                            LaunchConfiguration("velocity_scaling"), " if ", is_real, " else 0.0"]),
                    },
                ],
            )
        ]
    )
