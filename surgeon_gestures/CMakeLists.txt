cmake_minimum_required(VERSION 3.0.2)
project(surgeon_gestures)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  tf
)

find_package( PCL REQUIRED )

find_package (Boost COMPONENTS program_options filesystem REQUIRED)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

 catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  roscpp std_msgs message_runtime
)

FIND_PACKAGE (Eigen3 3.0.0 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories (${GLFW_INCLUDE_DIRS}
                     ${PROJECT_NAME}/include
)


set (ECM_ENDO_SCOPE_POINT_CLOUD_HDR_FILES
#    include/${PROJECT_NAME}/EndoScopePointCloud.h
    )

set(ECM_ENDO_SCOPE_POINT_CLOUD_SOURCE_FILES
#    src/EndoScopePointCloud.cpp
    src/rotate_point_cloud.cpp
    )

add_executable (ecm_endoscope_point_cloud ${ECM_ENDO_SCOPE_POINT_CLOUD_HDR_FILES} ${ECM_ENDO_SCOPE_POINT_CLOUD_SOURCE_FILES})
target_link_libraries (ecm_endoscope_point_cloud ${PCL_LIBRARIES} ${catkin_LIBRARIES})
