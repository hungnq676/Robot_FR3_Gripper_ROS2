#!/usr/bin/env python3
"""Prepare a simulation-only full-chain MTC config and physical container scene."""
import os, time
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
import rclpy
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
if os.environ.get('ROS_DOMAIN_ID') != '34': raise RuntimeError('Simulation domain 34 required')
root=Path(__file__).resolve().parents[1]
out=root/'nhat_ky/du_lieu/2026-11-03_mouse_open_box'
r=yaml.safe_load((out/'mouse_grasp.yaml').read_text())
# GraspNet candidates are recorded. These side grasps are unsuitable for the
# low tabletop object; explicitly use the SAM depth geometric top-down fallback.
T=np.array(r['geometric']['T_base']); R=T[:3,[1,2,0]]
q=Rotation.from_matrix(R).as_quat(); xyz=T[:3,3].copy(); xyz[2]=.046
cfg=yaml.safe_load((root/'ws_fr3_moveit/src/fairino_fr3_manipulation/config/mtc_pick_task.yaml').read_text())
p=cfg['mtc_pick_task_planner']['ros__parameters']
p.update({'target':'obj_4','observe.pose':'','place.position':[.22,.23,.20], 'place.release_into_box':True,
          'grasp.external_tcp':list(map(float,[*xyz,*q])), 'gripper.guarded_close':False,
          'max_solutions':3, 'approach.min_distance':.10, 'approach.max_distance':.14})
config=root/'ws_fr3_moveit/src/fairino_fr3_manipulation/config/mouse_open_box_sim.yaml'
config.write_text(yaml.safe_dump(cfg))
(out/'mtc_config.yaml').write_text(yaml.safe_dump(cfg))
rclpy.init(); n=rclpy.create_node('prepare_mouse_box_sim')
def call(kind,name,req):
 c=n.create_client(kind,name)
 if not c.wait_for_service(timeout_sec=10): raise RuntimeError(name)
 f=c.call_async(req); rclpy.spin_until_future_complete(n,f,timeout_sec=15)
 if f.result() is None: raise RuntimeError(name+' timed out')
 return f.result()
req=GetPlanningScene.Request(); req.components.components=8
old=call(GetPlanningScene,'/get_planning_scene',req)
s=PlanningScene(is_diff=True)
# Remove camera's solid representations of either container (the walls below
# replace them). Other objects remain collision obstacles.
for co in old.scene.world.collision_objects:
 if co.id=='cardboard_box' or co.id.startswith('open_box_'):
  s.world.collision_objects.append(CollisionObject(id=co.id,operation=CollisionObject.REMOVE))
 elif co.id.startswith('obj_') and co.id!='obj_4':
  pos=co.pose.position
  if co.primitive_poses:
   pos=co.primitive_poses[0].position
  # Camera perception objects are world-frame, with positions in co.pose.
  if co.pose.position.x or co.pose.position.y: pos=co.pose.position
  if co.id not in {'obj_2','obj_3','obj_5'}:
   s.world.collision_objects.append(CollisionObject(id=co.id,operation=CollisionObject.REMOVE))
parts=[('bottom',(0,0,-.046),(.24,.18,.008)),('left',(-.116,0,0),(.008,.18,.10)),('right',(.116,0,0),(.008,.18,.10)),('front',(0,-.086,0),(.224,.008,.10)),('back',(0,.086,0),(.224,.008,.10))]
for name,offset,size in parts:
 co=CollisionObject(id='open_box_'+name,operation=CollisionObject.ADD); co.header.frame_id='world'
 co.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=list(size))]
 pose=Pose(); pose.orientation.w=1.; pose.position.x=1.43+offset[0]; pose.position.y=.50+offset[1]; pose.position.z=1.05+offset[2]
 co.primitive_poses=[pose]; s.world.collision_objects.append(co)
assert call(ApplyPlanningScene,'/apply_planning_scene',ApplyPlanningScene.Request(scene=s)).success
print('Scene: 5 physical container parts; grasp source: SAM depth top-down fallback, GraspNet side candidates rejected.')
n.destroy_node(); rclpy.shutdown()
