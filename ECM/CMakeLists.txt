cmake_minimum_required(VERSION 3.0.0)
project(ECM VERSION 2.0)

set(ECM_HRD_DIR
  include
  # ${Envision_SOURCE_DIR}
  # ${spdlog_INCLUDE_DIRS_PS}
  # ${PCH_INCLUDE_DIRS}
  # ${Eigen3_INCLUDE_DIRS_PS}

  # ${ROS_INCLUDE_DIRS_PS}
  # ${AMBF_CLIENT_INCLUDE_DIRS_PS}
  # ${AMBF_MSGS_INCLUDE_DIRS_PS}
  # ${boost_headers_DIR}
  )

#####Beginning of ECMControlApp#############

SET ( ECM_CONTROL_SOURCE_FILES
src/Kinematics.cpp
src/DH.cpp
# src/Dynamics.cpp
# src/PathPlanning.cpp
# src/Control.cpp
src/AMBFWrapper.cpp
src/Utilities.cpp
src/AutomatingECM.cpp
)

SET (ECM_LINK_LIBRARIES
  # PRIVATE ${Envision_BINARY_DIR}/libEnvision.a
  PRIVATE ${AMBF_CLIENT_LIBRARIES}
  )

SET (ECMControlApp ECMControl_app)
ADD_EXECUTABLE ( ${ECMControlApp} 
  ${ECM_CONTROL_SOURCE_FILES} 
  )

target_precompile_headers(${ECMControlApp} PRIVATE ${CMAKE_SOURCE_DIR}/PCH/include/PCH/pch.h)

target_include_directories(${ECMControlApp}
  PRIVATE ${ECM_HRD_DIR} 
)

target_link_libraries(${ECMControlApp}
  ${ECM_LINK_LIBRARIES}
)

set_target_properties(${ECMControlApp} PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED YES
)
#####End of ECMControlApp#############
