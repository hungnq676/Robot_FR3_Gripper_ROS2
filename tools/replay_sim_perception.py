#!/usr/bin/env python3
"""Freeze the last camera perception during a recorded simulation execution."""
import os,sys,yaml,rclpy
from fairino_fr3_msgs.msg import PerceivedObjectArray
from rosidl_runtime_py.set_message import set_message_fields
if os.environ.get('ROS_DOMAIN_ID')!='34': raise RuntimeError('Simulation only')
rclpy.init(); n=rclpy.create_node('frozen_camera_perception')
p=n.create_publisher(PerceivedObjectArray,'/fr3_demo/perceived_objects',10)
m=PerceivedObjectArray(); set_message_fields(m,yaml.safe_load(open(sys.argv[1]).read().split('---')[0]))
n.create_timer(.5,lambda:p.publish(m))
rclpy.spin(n)
