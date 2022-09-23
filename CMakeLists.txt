cmake_minimum_required(VERSION 3.0.2)
project(qr_detector_zbar)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

find_package(catkin REQUIRED COMPONENTS
  rgbd
  message_generation
)

set(_opencv_version 4)
find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
  message(STATUS "Did not find OpenCV 4, trying OpenCV 3")
  set(_opencv_version 3)
endif()

find_package(OpenCV ${_opencv_version} REQUIRED
  COMPONENTS
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
  CONFIG
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# - - - - - - - - - - - - - - - - - TOOLS - - - - - - - - - - - - - - - - -
add_library(${PROJECT_NAME} src/qr_detector_zbar.cpp)
target_link_libraries(${PROJECT_NAME} /usr/lib/libzbar.so)

add_executable(qr_detector_zbar_node src/qr_detector.cpp)
target_link_libraries(qr_detector_zbar_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(qr_detector_zbar_test test/test_qr.cpp)
target_link_libraries(qr_detector_zbar_test ${PROJECT_NAME} ${catkin_LIBRARIES})

