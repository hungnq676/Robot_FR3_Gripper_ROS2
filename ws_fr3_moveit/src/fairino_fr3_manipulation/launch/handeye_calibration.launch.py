from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("fairino_fr3_manipulation"), "config", "handeye_calibration.yaml"]
    )
    real = LaunchConfiguration("real")
    is_real = PythonExpression(["'", real, "' == 'true'"])
    # real:=true also loads the real camera's topics/frames/files on top.
    real_config = PathJoinSubstitution([
        FindPackageShare("fairino_fr3_manipulation"), "config",
        PythonExpression(["'handeye_calibration_real.yaml' if ", is_real, " else 'handeye_calibration.yaml'"])])
    return LaunchDescription(
        [
            # real:=true  -> wall clock, start from the hand-guided pose, slow moves.
            DeclareLaunchArgument("real", default_value="false"),
            DeclareLaunchArgument("velocity_scaling", default_value="0.1"),
            Node(
                package="fairino_fr3_manipulation",
                executable="handeye_calibration.py",
                name="handeye_calibration",
                parameters=[
                    config,
                    real_config,
                    {
                        "use_sim_time": PythonExpression(["not ", is_real]),
                        "center_from_start_pose": is_real,
                        "velocity_scaling": PythonExpression([
                            LaunchConfiguration("velocity_scaling"), " if ", is_real, " else 0.25"]),
                    },
                ],
                output="screen",
            )
        ]
    )
