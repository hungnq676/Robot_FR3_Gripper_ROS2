from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(package_name, launch_file):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(package_name), "launch", launch_file]
            )
        )
    )


def generate_launch_description():
    # Ordering is handled by service availability rather than fixed timers:
    # - ros_gz_sim create waits for Gazebo's create service;
    # - gz_ros2_control waits for robot_state_publisher;
    # - controller spawners wait for /controller_manager.
    return LaunchDescription(
        [
            DeclareLaunchArgument("table_only", default_value="false",
                                  description="Robot, table and camera only"),
            include("fairino_fr3_gazebo", "gazebo.launch.py"),
            include("fairino_fr3_gazebo", "clock_bridge.launch.py"),
            include("fairino_fr3_gazebo", "robot_state_publisher.launch.py"),
            include("fairino_fr3_gazebo", "spawn_robot.launch.py"),
            include("fairino_fr3_gazebo", "controllers.launch.py"),
            include("fairino_fr3_gazebo", "camera_bridge.launch.py"),
        ]
    )
