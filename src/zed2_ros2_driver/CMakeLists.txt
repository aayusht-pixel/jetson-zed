cmake_minimum_required(VERSION 3.5)
project(zed2_ros2_driver)

# Default to C99 for C and C++14 for C++
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
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
find_package(stereo_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(ZED_INCLUDE_DIR /usr/local/zed/include)
include_directories(${ZED_INCLUDE_DIR})

add_executable(zed_node src/zed_node.cpp)

# Link to the specific ZED library
set(ZED_LIBRARY /usr/local/zed/lib/libsl_zed.so)
target_link_libraries(zed_node ${ZED_LIBRARY})

ament_target_dependencies(zed_node rclcpp sensor_msgs stereo_msgs nav_msgs)

install(TARGETS zed_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
