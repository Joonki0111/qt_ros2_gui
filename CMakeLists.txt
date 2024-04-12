
cmake_minimum_required(VERSION 3.8)
project(qt_ros2_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

include_directories(include)

add_executable(
    main 
    src/main.cpp)

ament_target_dependencies(
    main
    rclcpp
    std_msgs 
    Qt5Core
    Qt5Widgets
)

install(TARGETS main
  DESTINATION lib/${PROJECT_NAME})

ament_package()
