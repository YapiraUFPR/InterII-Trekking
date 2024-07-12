cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(custom_msgs REQUIRED)

include_directories(include)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(steering_node src/steering_node.cpp) 
ament_target_dependencies(steering_node rclcpp std_msgs vision_msgs nav_msgs geometry_msgs OpenCV Eigen3)
target_link_libraries( steering_node ${OpenCV_LIBS})
install(TARGETS
  steering_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(slam_node src/slam_node.cpp src/pid.cpp) 
ament_target_dependencies(slam_node rclcpp std_msgs sensor_msgs geometry_msgs OpenCV Eigen3)
target_link_libraries( slam_node ${OpenCV_LIBS})
install(TARGETS
  slam_node
  DESTINATION lib/${PROJECT_NAME})
  
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/imu_tracking_node.py
  scripts/teleop_node.py
  scripts/cone_detector.py
  scripts/mark_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
