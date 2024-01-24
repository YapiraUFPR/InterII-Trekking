cmake_minimum_required(VERSION 3.8)
project(sensor_drivers_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(camera_publisher src/camera_publisher.cpp)
ament_target_dependencies(camera_publisher rclcpp sensor_msgs OpenCV)

target_link_libraries( camera_publisher ${OpenCV_LIBS} )

install(TARGETS
  camera_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
