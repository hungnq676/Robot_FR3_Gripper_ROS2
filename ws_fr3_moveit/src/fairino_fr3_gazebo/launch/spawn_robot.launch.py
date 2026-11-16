from launch import LaunchDescription
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("fairino_fr3_description"), "urdf", "fairino_fr3.urdf.xacro"]
    )
    controller_config = PathJoinSubstitution(
        [FindPackageShare("fairino_fr3_gazebo"), "config", "gz_ros2_controllers.yaml"]
    )
    robot_xml = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
            " hardware_type:=gz use_gazebo:=true mount_to_world:=true controller_config:=",
            controller_config,
            # No graspable objects / hand-eye tag in the table-only world.
            " sim_objects:=",
            PythonExpression(["'false' if '", LaunchConfiguration("table_only", default="false"),
                              "' == 'true' else 'true'"]),
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-world", "fr3_workspace",
                    "-name", "fairino_fr3",
                    "-string", robot_xml,
                    "-x", "0.0",
                    "-y", "0.0",
                    "-z", "0.0",
                    "-Y", "0.0",
                ],
                output="screen",
            )
        ]
    )
