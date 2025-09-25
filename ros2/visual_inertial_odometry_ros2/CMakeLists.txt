message("   -- [PROJECT]: " ${CMAKE_PROJECT_NAME}_ros2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2 -pthread -pg")

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(visual_inertial_odometry_node visual_inertial_odometry_node.cc)
target_include_directories(visual_inertial_odometry_node
                           PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(
  visual_inertial_odometry_node visual_inertial_odometry_core
  ${EIGEN3_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(visual_inertial_odometry_node rclcpp sensor_msgs
                          geometry_msgs nav_msgs message_filters)
install(TARGETS visual_inertial_odometry_node DESTINATION lib/${PROJECT_NAME}
)# install
# executable
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}) # install launch
                                                            # files
