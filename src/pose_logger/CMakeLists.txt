cmake_minimum_required(VERSION 3.8)
project(pose_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(geometry_msgs REQUIRED)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

# pose_logger
add_executable(pose_logger src/pose_logger.cpp)
ament_target_dependencies(pose_logger rclcpp geometry_msgs OpenCV)
target_link_libraries( pose_logger ${OpenCV_LIBS} )
install(TARGETS
  pose_logger
  DESTINATION lib/${PROJECT_NAME})


ament_package()
