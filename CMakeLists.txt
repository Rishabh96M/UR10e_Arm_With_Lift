cmake_minimum_required(VERSION 3.5)
project(ur10e_arm_with_lift)

# Default to C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

install(
  DIRECTORY config launch rviz srdf urdf
  DESTINATION share/${PROJECT_NAME}
)

# Add your executable
add_executable(draw_circle_cartesian src/draw_circle_cartesian.cpp)
add_executable(go_to_point src/go_to_point.cpp)

# Link dependencies
ament_target_dependencies(draw_circle_cartesian  
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
)

ament_target_dependencies(go_to_point
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
)
# Install
install(TARGETS
  draw_circle_cartesian
  go_to_point
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
