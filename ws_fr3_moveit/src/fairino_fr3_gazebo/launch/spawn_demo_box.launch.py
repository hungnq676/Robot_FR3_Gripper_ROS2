from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = PathJoinSubstitution(
        [FindPackageShare("fairino_fr3_gazebo"), "models", "demo_box", "model.sdf"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="fr3_workspace"),
            # These world coordinates correspond to [0.40, -0.15, 0.05]
            # in base_footprint for world_to_fr3_mount yaw=-1.57.
            DeclareLaunchArgument("x", default_value="1.05"),
            DeclareLaunchArgument("y", default_value="0.32"),
            DeclareLaunchArgument("z", default_value="1.05"),
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_demo_box",
                output="screen",
                arguments=[
                    "-world", LaunchConfiguration("world"),
                    "-name", "demo_box",
                    "-file", model,
                    "-x", LaunchConfiguration("x"),
                    "-y", LaunchConfiguration("y"),
                    "-z", LaunchConfiguration("z"),
                ],
            ),
        ]
    )
