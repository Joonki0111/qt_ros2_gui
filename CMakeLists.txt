cmake_minimum_required(VERSION 3.8)
project(qt_ros2_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(roscco_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

include_directories(include)

add_library(
  can_interface
  src/can_interface.cpp
)

add_library(
  ros2_main
  src/ros2_main.cpp
)

ament_target_dependencies(
  ros2_main
  rclcpp
  std_msgs
  roscco_msgs
  autoware_vehicle_msgs
)

add_library(
  qt_main
  src/qt_main.cpp
)

ament_target_dependencies(
  qt_main
  Qt5Core
  Qt5Widgets
)

target_link_libraries(
  qt_main
  ros2_main
  can_interface
)

add_executable(
  exec 
  src/exec.cpp
)

target_link_libraries(
  exec
  qt_main
)

ament_target_dependencies(
  exec
  Qt5Core
)

install(TARGETS 
  exec
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS 
  qt_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
