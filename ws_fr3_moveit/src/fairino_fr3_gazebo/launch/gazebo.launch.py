import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, SetEnvironmentVariable


def launch_gazebo(context):
    package_share = get_package_share_directory("fairino_fr3_gazebo")
    description_share = get_package_share_directory("fairino_fr3_description")
    # table_only:=true -> robot, table and camera only (no objects, no tags).
    table_only = context.launch_configurations.get("table_only", "false") == "true"
    world = os.path.join(package_share, "worlds",
                         "fr3_table_only.sdf" if table_only else "fr3_workspace.sdf")
    models = os.path.join(package_share, "models")

    # Gazebo converts package://fairino_fr3_description/... mesh URIs to
    # model://fairino_fr3_description/....  It therefore needs the directory
    # containing the package (the parent `share` directory), not only the
    # package's own mesh directory.  The Gazebo model directory is also needed
    # for model://table, model://depth_camera and model://apriltag_0.
    description_share_parent = os.path.dirname(description_share)
    resource_entries = [models, description_share_parent]
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH")
    if existing_resource_path:
        resource_entries.append(existing_resource_path)
    resource_path = os.pathsep.join(resource_entries)

    return [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        ExecuteProcess(
            cmd=[
                "gz", "sim", "-r", "-v", "4",
                "--physics-engine", "gz-physics-bullet-featherstone-plugin",
                world,
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_gazebo)])
