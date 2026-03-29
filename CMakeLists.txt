cmake_minimum_required(VERSION 3.5)
project(apf_pathplanner_tutorial)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(potbot_lib REQUIRED)
find_package(potbot_ros REQUIRED)

set(DEPENDENCIES
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  interactive_markers
  potbot_lib
  potbot_ros
)

add_executable(apf_pathplanner_tutorial src/apf_pathplanner_tutorial.cpp)
ament_target_dependencies(apf_pathplanner_tutorial ${DEPENDENCIES})

add_executable(interactive_marker_node src/interactive_marker_node.cpp)
ament_target_dependencies(interactive_marker_node ${DEPENDENCIES})

install(TARGETS
  apf_pathplanner_tutorial
  interactive_marker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
