set(MODULE_CORE_NAME ${MODULE_NAME}_core)
project(${MODULE_CORE_NAME})

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

set(SRC ${SRC} visual_inertial_odometry.cc feature_extractor/point_extractor.cc
        feature_extractor/point_extractor.cc feature_tracker/point_tracker.cc)

# set(CMAKE_INSTALL_PREFIX )
add_library(${MODULE_CORE_NAME} STATIC ${SRC})
target_link_libraries(${MODULE_CORE_NAME} ${EIGEN3_LIBRARIES} ${OpenCV_LIBS})
install(
  TARGETS ${MODULE_CORE_NAME}
  DESTINATION lib/${MODULE_NAME}
  CONFIGURATIONS Release)
