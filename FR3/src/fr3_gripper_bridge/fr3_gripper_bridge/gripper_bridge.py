#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory


class GripperActionBridge(Node):
    def __init__(self):
        super().__init__("fr3_gripper_action_bridge")

        self.joint_name = "custom_gripper_actuation_joint"

        self.open_joint = 0.9
        self.closed_joint = -2.6
        self.mid_joint = (self.open_joint + self.closed_joint) / 2.0

        self.open_steps = 0
        self.closed_steps = 6500
        self.tolerance_steps = 80
        self.timeout_sec = 8.0

        self.current_steps = 0
        self.current_joint = self.open_joint

        self.goal_pub = self.create_publisher(
            Int32,
            "/fr3_gripper/goal_steps",
            10
        )

        self.joint_pub = self.create_publisher(
            JointState,
            "/joint_states",
            10
        )

        self.steps_sub = self.create_subscription(
            Int32,
            "/fr3_gripper/estimated_steps",
            self.steps_callback,
            10
        )

        self.timer = self.create_timer(0.05, self.publish_joint_state)

        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info("FR3 gripper Action bridge READY")

    def steps_callback(self, msg):
        self.current_steps = max(self.open_steps, min(self.closed_steps, msg.data))
        self.current_joint = self.steps_to_joint(self.current_steps)

    def steps_to_joint(self, steps):
        ratio = steps / float(self.closed_steps)
        return self.open_joint + ratio * (self.closed_joint - self.open_joint)

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.joint_name]
        msg.position = [self.current_joint]
        self.joint_pub.publish(msg)

    def goal_callback(self, goal_request):
        traj = goal_request.trajectory

        if self.joint_name not in traj.joint_names:
            self.get_logger().error("Rejected: joint name not found")
            return GoalResponse.REJECT

        if len(traj.points) == 0:
            self.get_logger().error("Rejected: empty trajectory")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("Cancel requested")

        stop_msg = Int32()
        stop_msg.data = self.current_steps
        self.goal_pub.publish(stop_msg)

        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        traj = goal_handle.request.trajectory
        joint_index = traj.joint_names.index(self.joint_name)

        target_joint = traj.points[-1].positions[joint_index]

        if target_joint > self.mid_joint:
            target_steps = self.open_steps
            target_name = "OPEN"
        else:
            target_steps = self.closed_steps
            target_name = "CLOSE"

        self.get_logger().info(
            f"MoveIt command: {target_name}, joint={target_joint:.3f}, steps={target_steps}"
        )

        cmd = Int32()
        cmd.data = target_steps
        self.goal_pub.publish(cmd)

        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = [self.joint_name]

        start_time = time.time()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result

            error_steps = abs(self.current_steps - target_steps)

            feedback.actual.positions = [self.current_joint]
            feedback.desired.positions = [self.steps_to_joint(target_steps)]
            feedback.error.positions = [
                self.steps_to_joint(target_steps) - self.current_joint
            ]
            goal_handle.publish_feedback(feedback)

            if error_steps <= self.tolerance_steps:
                self.get_logger().info(f"Gripper reached {target_name}")
                goal_handle.succeed()

                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result

            if time.time() - start_time > self.timeout_sec:
                self.get_logger().error(f"Gripper timeout while moving {target_name}")
                goal_handle.abort()

                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = "Gripper timeout"
                return result

            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = GripperActionBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()