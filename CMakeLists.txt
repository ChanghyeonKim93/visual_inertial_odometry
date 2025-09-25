cmake_minimum_required(VERSION 3.10)
project(visual_inertial_odometry)

set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2 -pthread")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -msse -mavx -mavx2")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -ffast-math -ftree-vectorize -march=armv8-a+simd")
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)

add_subdirectory(core)
add_subdirectory(ros2)

ament_package()
