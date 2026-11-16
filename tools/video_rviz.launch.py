from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from pathlib import Path

def generate_launch_description():
 c=(MoveItConfigsBuilder('fairino_fr3',package_name='fairino_fr3_moveit_config')
 .robot_description(mappings={'hardware_type':'mock','use_gazebo':'false','mount_to_world':'false'})
 .robot_description_semantic(file_path='config/fairino_fr3.srdf')
 .planning_pipelines(pipelines=['ompl','pilz_industrial_motion_planner']).to_moveit_configs())
 return LaunchDescription([Node(package='rviz2',executable='rviz2',arguments=['-d',str(Path(__file__).parent/'mouse_box_sim.rviz')],parameters=[c.to_dict(),{'use_sim_time':True}],output='screen')])
