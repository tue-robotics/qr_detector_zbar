cmake_minimum_required(VERSION 3.0.2)
project(qr_detector_zbar)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

find_package(catkin REQUIRED COMPONENTS
  rgbd
  message_generation
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

