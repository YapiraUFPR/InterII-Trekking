cmake_minimum_required(VERSION 3.8)
project(mapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(nav_msgs REQUIRED)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

# mapper
add_executable(mapper src/mapper.cpp)
ament_target_dependencies(mapper rclcpp nav_msgs OpenCV)
target_link_libraries( mapper ${OpenCV_LIBS} )
install(TARGETS
  mapper
  DESTINATION lib/${PROJECT_NAME})


ament_package()
