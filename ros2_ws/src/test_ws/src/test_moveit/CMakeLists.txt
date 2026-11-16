cmake_minimum_required(VERSION 3.8)
project(test_moveit)

# Tìm thư viện cần thiết
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Tạo executable
add_executable(test_move src/test_move.cpp)

# Link thư viện
ament_target_dependencies(test_move
  rclcpp
  moveit_ros_planning_interface
)

# Cài đặt
install(TARGETS test_move
  DESTINATION lib/${PROJECT_NAME}
)

# Kết thúc
ament_package()
