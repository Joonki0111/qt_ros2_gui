cmake_minimum_required(VERSION 3.8)
project(qt_ros2_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(adma_ros_driver_msgs REQUIRED)
find_package(roscco_msgs REQUIRED)

include_directories(include)

add_library(
  ros2_main
  src/ros2_main.cpp
)
add_library(
  qt_main
  src/qt_main.cpp
)
add_executable(
  exec 
  src/exec.cpp
)

ament_target_dependencies(
  ros2_main
  rclcpp
  std_msgs
  autoware_auto_vehicle_msgs
  autoware_adapi_v1_msgs
  rosgraph_msgs
  adma_ros_driver_msgs
  roscco_msgs
)
ament_target_dependencies(
  qt_main
  Qt5Core
  Qt5Widgets
)
ament_target_dependencies(
  exec
  Qt5Core
)

target_link_libraries(
  qt_main
  ros2_main
)
target_link_libraries(
  exec
  qt_main
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
