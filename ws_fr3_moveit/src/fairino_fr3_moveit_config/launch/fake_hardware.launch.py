from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("fairino_fr3_description"), "urdf", "fairino_fr3.urdf.xacro"]
    )
    controllers = PathJoinSubstitution(
        [FindPackageShare("fairino_fr3_moveit_config"), "config", "ros2_controllers.yaml"]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro_file, " hardware_type:=mock"]),
            value_type=str,
        )
    }

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, controllers],
            output="screen",
        ),
    ]
    for controller in ["joint_state_broadcaster", "arm_controller", "gripper_controller"]:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--controller-manager-timeout", "30"],
                output="screen",
            )
        )
    return LaunchDescription(nodes)
