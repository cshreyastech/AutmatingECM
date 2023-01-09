cmake_minimum_required(VERSION 3.0.0)
project(ECM VERSION 2.0)

set(COMMON_HRD_DIR
  include
  ${Envision_SOURCE_DIR}
  ${spdlog_INCLUDE_DIR}
  ${PCH_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIR}

  ${ROS_INCLUDE_DIR}
  ${AMBF_CLIENT_INCLUDE_DIR}
  ${AMBF_MSGS_INCLUDE_DIR}
  ${boost_headers_DIR}
  )


set(COMMON_LINK_LIBRARIES
  PRIVATE ${Envision_BINARY_DIR}/libEnvision.a
  PRIVATE ${spdlog_BINARY_DIR}/libspdlog.a
  # PRIVATE ${BOOST_LIBRARY_LIBRARY_PS}
  PRIVATE ${AMBF_CLIENT_LIBRARY}
  )

#####Beginning of ECMControlApp#############
set(ECMControlApp ECMControl_app)

SET ( ECM_CONTROL_SOURCE_FILES
  src/Kinematics.cpp
  src/Dynamics.cpp
  src/PathPlanning.cpp
  src/Control.cpp
  src/AMBFWrapper.cpp
  src/main.cpp
  # src/Utilities.cpp
  )

ADD_EXECUTABLE ( ${ECMControlApp} 
  ${ECM_CONTROL_SOURCE_FILES} 
  
  )

target_include_directories(${ECMControlApp}
  PRIVATE ${COMMON_HRD_DIR}
  PRIVATE ${ECM_HRD_DIR} 
)

target_link_libraries(${ECMControlApp} 
  ${COMMON_LINK_LIBRARIES}
)

set_target_properties(${ECMControlApp} PROPERTIES
CXX_STANDARD 20
CXX_STANDARD_REQUIRED YES
)
#####End of ECMControlApp#############
