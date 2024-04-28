cmake_minimum_required(VERSION 3.8)
project(controller_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(steering_node src/steering_node.cpp) 
ament_target_dependencies(steering_node rclcpp std_msgs nav_msgs geometry_msgs OpenCV Eigen3)
target_link_libraries( steering_node ${OpenCV_LIBS})
install(TARGETS
  steering_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
