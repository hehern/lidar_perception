cmake_minimum_required(VERSION 2.8.3)
project(lidar_perception)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(PCL REQUIRED)
include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(OpenCV 4.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(${EIGEN3_LIBRARY_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES people_tracker
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  src

)

add_executable(lidar_perception_node 
              src/lidar_perception_node.cpp
              src/lidar_perception.cpp
              src/pcl_clip/pcl_clip.cpp
              src/remove_ground/remove_ground.cpp
              src/cluster/region_growing_segmentation.cpp
              src/min_box/min_box.cc
              src/cluster/EuclideanCluster.cpp
              src/cluster/dbscan.cpp
              include/lidar_perception/object.cc
              src/hm_tracker/hm_tracker.cc
              src/hm_tracker/feature_descriptor.cc
              src/hm_tracker/hungarian_matcher.cc
              src/hm_tracker/kalman_filter.cc
              src/hm_tracker/object_track.cc
              src/hm_tracker/track_object_distance.cc
              src/hm_tracker/tracked_object.cc
              include/lidar_perception/hungarian/hungarian_bigraph_matcher.cc
              include/lidar_perception/common/graph_util.cc
              include/lidar_perception/common/geometry_util.cc
              )
add_dependencies(lidar_perception_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(lidar_perception_node  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${EIGEN3_LIBRARIES})
