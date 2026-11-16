#!/usr/bin/env python3
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32


def main():
    if len(sys.argv) < 2:
        print("Usage: gripper_goal.py <steps>")
        print("Example: gripper_goal.py 5200")
        print("Example: gripper_goal.py -1   # set zero")
        return

    goal = int(sys.argv[1])

    rclpy.init()

    node = rclpy.create_node("fr3_gripper_goal_sender")

    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )

    pub = node.create_publisher(Int32, "/fr3_gripper/goal_steps", qos)

    print("Waiting for subscriber...")
    t0 = time.time()
    while pub.get_subscription_count() == 0 and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f"Subscriber count: {pub.get_subscription_count()}")

    msg = Int32()
    msg.data = goal

    # Gửi vài lần để chống mất UDP/discovery jitter.
    for i in range(5):
        pub.publish(msg)
        print(f"Published {goal}, #{i + 1}")
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.15)

    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
