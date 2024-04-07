cmake_minimum_required(VERSION 3.8)
project(drivers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# OpenCV dependencies
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# camera_publisher
add_executable(camera_publisher src/camera_publisher.cpp)
ament_target_dependencies(camera_publisher rclcpp sensor_msgs OpenCV)
target_link_libraries( camera_publisher ${OpenCV_LIBS} )
install(TARGETS
  camera_publisher
  DESTINATION lib/${PROJECT_NAME})

# led_listener
add_executable(led_listener src/led_listener.cpp)
ament_target_dependencies(led_listener rclcpp std_msgs OpenCV)
target_link_libraries( led_listener ${OpenCV_LIBS} )
install(TARGETS
  led_listener
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/bno_publisher.py
  scripts/ina_publisher.py
  scripts/vl5_publisher.py
  scripts/tcs32_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()