import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_fr3_gazebo = get_package_share_directory('fr3_gazebo')
    pkg_fairino = get_package_share_directory('fairino_description')
    pkg_gripper = get_package_share_directory('fr3_with_custom_gripper_description')

    world = os.path.join(pkg_fr3_gazebo, 'worlds', 'table.world')
    models = os.path.join(pkg_fr3_gazebo, 'models')

    resource_paths = ':'.join([
        models,
        pkg_fr3_gazebo,
        pkg_fairino,
        pkg_gripper,
        os.path.dirname(pkg_fairino),
        os.path.dirname(pkg_gripper),
        '/home/hung/FR3/install',
        '/home/hung/FR3/src',
    ])

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=resource_paths
        ),

        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '4', '-r', world],
            output='screen'
        ),
    ])
