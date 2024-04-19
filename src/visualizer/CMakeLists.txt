cmake_minimum_required(VERSION 3.8)
project(visualizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rclpy REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# OpenCV dependencies
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# visualization_node
add_executable(visualization_node src/visualization_node.cpp)
ament_target_dependencies(visualization_node rclcpp geometry_msgs visualization_msgs OpenCV)
target_link_libraries( visualization_node ${OpenCV_LIBS} )
install(TARGETS
  visualization_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# Install Python executables
# install(PROGRAMS
#   scripts/
#   DESTINATION lib/${PROJECT_NAME}
# )

# # Install launch files
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
