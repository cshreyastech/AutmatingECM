cmake_minimum_required(VERSION 3.0.2)
project(ecm_manupulation_modules)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ecm_kinematics)
