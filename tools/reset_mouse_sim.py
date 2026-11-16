#!/usr/bin/env python3
import os,yaml,rclpy
from pathlib import Path
from moveit_msgs.msg import CollisionObject,PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
if os.environ.get('ROS_DOMAIN_ID')!='34': raise RuntimeError('Simulation only')
rclpy.init();n=rclpy.create_node('reset_mouse_scene')
r=yaml.safe_load((Path(__file__).resolve().parents[1]/'nhat_ky/du_lieu/2026-11-03_mouse_open_box/perceived_objects.yaml').read_text().split('---')[0]);o=next(x for x in r['objects'] if x['id']=='obj_4')
co=CollisionObject(id='obj_4',operation=CollisionObject.ADD);co.header.frame_id='world';co.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=o['dimensions'])]
p=Pose()
for k,v in o['pose']['pose']['position'].items():setattr(p.position,k,v)
for k,v in o['pose']['pose']['orientation'].items():setattr(p.orientation,k,v)
co.primitive_poses=[p];s=PlanningScene(is_diff=True);s.world.collision_objects=[co]
c=n.create_client(ApplyPlanningScene,'/apply_planning_scene');c.wait_for_service();f=c.call_async(ApplyPlanningScene.Request(scene=s));rclpy.spin_until_future_complete(n,f);print(f.result());rclpy.shutdown()
