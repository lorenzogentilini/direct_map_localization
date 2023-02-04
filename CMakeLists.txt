cmake_minimum_required(VERSION 3.0.2)
project(direct_map_localization)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  tf2_ros
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(octomap REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tesseract_ros
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS OpenCV
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(direct_map_localization
  src/grid_map.cpp
  src/direct_localization.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(direct_map_localization
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

## Declare a C++ executable
add_executable(localization_node src/localization_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(localization_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${CERES_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  direct_map_localization
)  