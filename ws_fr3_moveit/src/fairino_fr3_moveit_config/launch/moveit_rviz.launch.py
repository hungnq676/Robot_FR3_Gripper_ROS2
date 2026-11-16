from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def moveit_rviz(context):
    use_sim_time = LaunchConfiguration("use_sim_time")
    # Real arm: the Velocity/Accel Scaling boxes start from the defaults in
    # config/joint_limits_real.yaml, same file as move_group (real:=true).
    suffix = "_real" if LaunchConfiguration("real").perform(context) == "true" else ""
    moveit_config = (
        MoveItConfigsBuilder("fairino_fr3", package_name="fairino_fr3_moveit_config")
        .robot_description(
            mappings={
                # fairino: the real gripper's collision/TCP (fairino_fr3_model.xacro),
                # same model as robot_state_publisher in real_robot.launch.py.
                "hardware_type": "fairino" if suffix else "mock",
                "use_gazebo": "false",
                # Keep the RViz grid at the robot base instead of at the
                # Gazebo world's floor level.
                "mount_to_world": "false",
            }
        )
        .robot_description_semantic(file_path="config/fairino_fr3.srdf")
        .joint_limits(file_path=f"config/joint_limits{suffix}.yaml")
        # RViz sends its own pipeline choice (Context tab), which starts from
        # this default: Pilz PTP on the real arm, like move_group.
        .planning_pipelines(
            default_planning_pipeline="pilz_industrial_motion_planner" if suffix else "ompl",
            pipelines=["ompl", "pilz_industrial_motion_planner"],
        )
        .to_moveit_configs()
    )
    rviz_config = PathJoinSubstitution(
        # moveit_real.rviz = moveit.rviz with Velocity/Accel Scaling 0.9 (~10% speed)
        [FindPackageShare("fairino_fr3_moveit_config"), "config", f"moveit{suffix}.rviz"]
    )

    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="moveit_rviz",
            arguments=["-d", rviz_config],
            parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("real", default_value="false"),
            OpaqueFunction(function=moveit_rviz),
        ]
    )
