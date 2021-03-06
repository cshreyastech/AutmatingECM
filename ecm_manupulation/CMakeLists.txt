cmake_minimum_required(VERSION 3.0.2)
project(ecm_manupulation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set (ambf_msgs_DIR "/mnt/OneTB/localcodebase/ambf_repos/AutmatingECM/cmake")
find_package(catkin REQUIRED COMPONENTS
   ambf_msgs
   geometry_msgs
   message_generation
   tf
)

set (AMBF_CLIENT_LIBRARY "/mnt/OneTB/localcodebase/ambf_repos/aimlab/ambf/build/devel/lib/libambf_client_cpp.so")

FIND_PACKAGE (Eigen3 3.0.0 REQUIRED)
#find_package(GTest REQUIRED)

FIND_PACKAGE (ambf_client REQUIRED)
include_directories(${AMBF_CLIENT_INCLUDE_DIR})
MESSAGE(STATUS "Found AMBF: ${AMBF_CLIENT_INCLUDE_DIR}")
find_package (Boost COMPONENTS program_options filesystem REQUIRED)

catkin_package(INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS ambf_msgs
#  DEPENDS system_lib
)

include_directories( include ${AMBF_CLIENT_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})


#find_package (ambf_client REQUIRED)
find_package (GLFW REQUIRED)
find_package (Boost COMPONENTS program_options filesystem REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



include_directories (${GLFW_INCLUDE_DIRS}
                     ${PROJECT_NAME}/include
)


set (ECM_IK_HDR_FILES
    include/${PROJECT_NAME}/AMBFClient.h
    include/${PROJECT_NAME}/Dynamics.h
    include/${PROJECT_NAME}/Kinematics.h
    include/${PROJECT_NAME}/ECM.h
    include/${PROJECT_NAME}/DH.h
    include/${PROJECT_NAME}/Utilities.h)

set(ECM_IK_SOURCE_FILES
    src/main.cpp
    src/AMBFClient.cpp
    src/Dynamics.cpp
    src/Kinematics.cpp
    src/ECM.cpp
    src/DH.cpp
    src/Utilities.cpp
    )

add_executable (ecm  ${ECM_IK_HDR_FILES} ${ECM_IK_SOURCE_FILES})
target_link_libraries (ecm ${AMBF_CLIENT_LIBRARY} ${GLFW_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES} pthread)

