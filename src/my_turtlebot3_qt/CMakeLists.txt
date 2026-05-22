cmake_minimum_required(VERSION 3.8)
project(my_turtlebot3_qt)

cmake_policy(SET CMP0072 NEW)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGL OpenGLWidgets)

add_executable(pointcloud_viewer
  src/main.cpp
  src/pointcloud_widget.cpp
  src/pointcloud_widget.hpp
)

ament_target_dependencies(pointcloud_viewer
  rclcpp
  sensor_msgs
)

target_link_libraries(pointcloud_viewer
  Qt6::Widgets
  Qt6::OpenGL
  Qt6::OpenGLWidgets
  OpenGL::GL
  OpenGL::GLU
)

install(TARGETS pointcloud_viewer DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
