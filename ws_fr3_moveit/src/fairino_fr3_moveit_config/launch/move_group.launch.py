from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def move_group(context):
    use_sim_time = LaunchConfiguration("use_sim_time")
    # Real arm: slower joint/Cartesian limits (config/*_real.yaml), so even
    # velocity scaling 1.0 in RViz stays at a speed a person can react to.
    real = LaunchConfiguration("real").perform(context) == "true"
    suffix = "_real" if real else ""
    # Both pipelines are offered in RViz (Context tab). The real arm defaults to
    # Pilz PTP: a straight line in joint space, all joints start and stop
    # together, no OMPL detours/corners that made the arm stall mid-move.
    default_pipeline = "pilz_industrial_motion_planner" if real else "ompl"
    moveit_config = (
        MoveItConfigsBuilder("fairino_fr3", package_name="fairino_fr3_moveit_config")
        .robot_description(
            mappings={
                # fairino: the real gripper's collision/TCP (fairino_fr3_model.xacro),
                # same model as robot_state_publisher in real_robot.launch.py.
                "hardware_type": "fairino" if real else "mock",
                "use_gazebo": "false",
                # Plan in the robot mounting frame. Gazebo may place this frame
                # on a table, but MoveIt does not need that world offset.
                "mount_to_world": "false",
            }
        )
        .robot_description_semantic(file_path="config/fairino_fr3.srdf")
        .joint_limits(file_path=f"config/joint_limits{suffix}.yaml")
        .pilz_cartesian_limits(file_path=f"config/pilz_cartesian_limits{suffix}.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            default_planning_pipeline=default_pipeline,
            pipelines=["ompl", "pilz_industrial_motion_planner"],
        )
        .to_moveit_configs()
    )

    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {
                    "use_sim_time": use_sim_time,
                    # Required by moveit::task_constructor::Task::execute().
                    "capabilities": "move_group/ExecuteTaskSolutionCapability",
                },
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("real", default_value="false"),
            OpaqueFunction(function=move_group),
        ]
    )
