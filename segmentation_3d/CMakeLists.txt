cmake_minimum_required(VERSION 3.0.2)
project(segmentation_3d)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  gb_visual_detection_3d_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  segmentation_msgs
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  message_filters
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS gb_visual_detection_3d_msgs pcl_conversions pcl_ros roscpp rospy segmentation_msgs sensor_msgs std_msgs tf2_geometry_msgs tf2_ros
)

## Specify additional locations of header files
include_directories(
  include/${PROJECT_NAME}/
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(segmentation_processor_node src/segmentation_processor.cpp src/point_processor.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(segmentation_processor_node
  ${catkin_LIBRARIES}
)

## Install targets
install(TARGETS segmentation_processor_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

