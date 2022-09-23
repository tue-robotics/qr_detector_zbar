cmake_minimum_required(VERSION 3.0.2)
project(qr_detector_zbar)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

find_package(catkin REQUIRED COMPONENTS
  rgbd
  roscpp
  std_msgs
)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(ZBar REQUIRED)

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
  CONFIG
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rgbd
  DEPENDS OpenCV
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZBAR_INVLUDE_DIRS}
)

# - - - - - - - - - - - - - - - - - TOOLS - - - - - - - - - - - - - - - - -
add_library(${PROJECT_NAME} src/qr_detector_zbar.cpp)
target_link_libraries(${PROJECT_NAME} ${ZBAR_LIBRARIES} ${catkin_LIBRARIES})

add_executable(qr_detector_zbar_node src/qr_detector.cpp)
target_link_libraries(qr_detector_zbar_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(qr_detector_zbar_test test/test_qr.cpp)
target_link_libraries(qr_detector_zbar_test ${PROJECT_NAME} ${catkin_LIBRARIES})

