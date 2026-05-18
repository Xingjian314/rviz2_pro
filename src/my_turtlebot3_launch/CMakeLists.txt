cmake_minimum_required(VERSION 3.8)
project(my_turtlebot3_launch)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)

add_executable(laser_to_pointcloud src/laser_to_pointcloud_node.cpp)
ament_target_dependencies(laser_to_pointcloud
  rclcpp
  sensor_msgs
  laser_geometry
)

install(TARGETS laser_to_pointcloud DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
