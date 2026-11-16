cmake_minimum_required(VERSION 3.8)
project(fairino_fr3_manipulation)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(fairino_fr3_msgs REQUIRED)

add_executable(object_description_publisher src/object_description_publisher.cpp)
ament_target_dependencies(object_description_publisher
  rclcpp
  moveit_msgs
  shape_msgs
  visualization_msgs
)
target_compile_features(object_description_publisher PUBLIC cxx_std_17)

add_executable(planning_scene_object_manager src/planning_scene_object_manager.cpp)
ament_target_dependencies(planning_scene_object_manager
  rclcpp
  moveit_msgs
  moveit_ros_planning_interface
  std_msgs
  std_srvs
)
target_compile_features(planning_scene_object_manager PUBLIC cxx_std_17)

add_executable(grasp_pose_generator src/grasp_pose_generator.cpp)
ament_target_dependencies(grasp_pose_generator
  rclcpp
  geometry_msgs
  moveit_msgs
  std_msgs
  visualization_msgs
)
target_compile_features(grasp_pose_generator PUBLIC cxx_std_17)

add_executable(pick_and_carry src/pick_and_carry.cpp)
ament_target_dependencies(pick_and_carry
  rclcpp
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  std_srvs
)
target_compile_features(pick_and_carry PUBLIC cxx_std_17)

add_executable(gazebo_object_sync src/gazebo_object_sync.cpp)
ament_target_dependencies(gazebo_object_sync
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  shape_msgs
  tf2_geometry_msgs
  tf2_ros
)
target_compile_features(gazebo_object_sync PUBLIC cxx_std_17)

add_executable(mtc_pick_task_planner src/mtc_pick_task_planner.cpp)
ament_target_dependencies(mtc_pick_task_planner
  rclcpp
  fairino_fr3_msgs
  geometry_msgs
  moveit_core
  moveit_task_constructor_core
  moveit_ros_planning_interface
  std_msgs
  std_srvs
)
target_compile_features(mtc_pick_task_planner PUBLIC cxx_std_17)

add_executable(attached_object_demo src/attached_object_demo.cpp)
ament_target_dependencies(attached_object_demo
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  shape_msgs
)
target_compile_features(attached_object_demo PUBLIC cxx_std_17)

install(TARGETS
  attached_object_demo
  object_description_publisher
  planning_scene_object_manager
  grasp_pose_generator
  pick_and_carry
  gazebo_object_sync
  mtc_pick_task_planner
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/calibrate_camera_apriltag.py
  scripts/tabletop_perception.py
  scripts/gazebo_grasp_attacher.py
  scripts/fsr_emulator.py
  scripts/gemini_target_selector.py
  scripts/handeye_calibration.py
  scripts/real_cell_scene.py
  scripts/real_task_scene.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
