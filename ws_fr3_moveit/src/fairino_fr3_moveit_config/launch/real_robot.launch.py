"""Real FR3 arm + MoveIt + RViz (gripper simulated until it is built).

Needs the frcobot_ros2 workspace sourced first (fairino_hardware plugin):
    source ~/ros2_ws/install/setup.bash && source ~/ws_fr3_moveit/install/setup.bash
The PC must reach the controller at 192.168.58.2 and the robot must be in
automatic mode and enabled on the teach pendant.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    moveit_config_share = FindPackageShare("fairino_fr3_moveit_config")
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("fairino_fr3_description"), "urdf", "fairino_fr3.urdf.xacro"])
    controllers = PathJoinSubstitution([moveit_config_share, "config", "ros2_controllers_real.yaml"])
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro_file, " hardware_type:=fairino"]),
            value_type=str)
    }

    def include(launch_file):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([moveit_config_share, "launch", launch_file])),
            # real:=true -> move_group loads the slow config/*_real.yaml limits
            launch_arguments={"use_sim_time": "false", "real": "true"}.items())

    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[robot_description], output="screen"),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[robot_description, controllers], output="screen"),
    ]
    for controller in ["joint_state_broadcaster", "arm_controller", "gripper_controller"]:
        nodes.append(Node(package="controller_manager", executable="spawner",
                          arguments=[controller, "--controller-manager-timeout", "30"], output="screen"))
    # MoveIt only after the driver has published real joint states.
    nodes.append(TimerAction(period=4.0, actions=[
        include("move_group.launch.py"),
        include("moveit_rviz.launch.py"),
    ]))
    # Table after RViz is up: added earlier, move_group has it but RViz misses the
    # update and shows no table (2026-10-29), which hides the obstacle from the user.
    nodes.append(TimerAction(period=12.0, actions=[
        Node(package="fairino_fr3_manipulation", executable="real_cell_scene.py", output="screen"),
    ]))
    return LaunchDescription(nodes)
