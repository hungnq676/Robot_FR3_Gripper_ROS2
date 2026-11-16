from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bridge_config = PathJoinSubstitution(
        [FindPackageShare("fairino_fr3_gazebo"), "config", "bridges.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gz_bridge",
                parameters=[{"config_file": bridge_config, "use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="external_camera_tf",
                arguments=[
                    "-0.30", "0.50", "2.40",
                    "0", "0.9273", "0",
                    "world", "external_camera_link",
                ],
                output="screen",
            ),
        ]
    )
