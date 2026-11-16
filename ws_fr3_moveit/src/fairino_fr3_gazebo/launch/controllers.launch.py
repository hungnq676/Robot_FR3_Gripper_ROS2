from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    controller_names = [
        "joint_state_broadcaster",
        "arm_controller",
        "gripper_controller",
    ]
    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            # Spawners can be started with the rest of the simulation. They
            # wait until the robot is spawned and gz_ros2_control creates
            # /controller_manager, avoiding brittle fixed delays.
            arguments=[name, "--controller-manager", "/controller_manager", "--controller-manager-timeout", "120"],
            output="screen",
        )
        for name in controller_names
    ]
    # gz-sim DetachableJoint creates its fixed joint as soon as it loads, which
    # would glue demo_box to the gripper from the start. Detach it once the
    # robot (and therefore the plugin) is up so the box rests freely on the table.
    # A single gz publish can be lost before transport discovery completes, so
    # repeat it; extra detaches are ignored by the plugin.
    detach_demo_box = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "for i in 1 2 3 4 5; do "
            "gz topic -t /fr3_demo/gazebo_detach -m gz.msgs.Empty -p ' '; sleep 1; done",
        ],
        output="screen",
    )
    return LaunchDescription(
        spawners
        + [
            RegisterEventHandler(
                OnProcessExit(target_action=spawners[0], on_exit=[detach_demo_box])
            )
        ]
    )
