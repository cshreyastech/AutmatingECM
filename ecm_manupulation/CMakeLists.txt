cmake_minimum_required(VERSION 3.0.2)
project(ecm_manupulation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ecm_kinematics)

find_package(catkin REQUIRED COMPONENTS
  # ambf_msgs
  # geometry_msgs
  # message_generation
  # tf
)


find_package(catkin REQUIRED COMPONENTS
  #message_runtime
)

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
    include/${PROJECT_NAME}/Kinematics.h
    include/${PROJECT_NAME}/ECM.h
    include/${PROJECT_NAME}/DH.h
    include/${PROJECT_NAME}/Utilities.h)

set(ECM_IK_SOURCE_FILES
    src/Kinematics.cpp
    src/ECM.cpp
    src/DH.cpp
    src/Utilities.cpp
    )

add_executable (ecm  ${ECM_IK_HDR_FILES} ${ECM_IK_SOURCE_FILES})
target_link_libraries (ecm ${AMBF_LIBRARIES} ${AMBF_CLIEMT_LIBRARIES} ${GLFW_LIBRARIES} ${Boost_LIBRARIES})

