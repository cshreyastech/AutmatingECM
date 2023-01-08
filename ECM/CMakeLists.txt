cmake_minimum_required(VERSION 3.0.0)
project(ECM VERSION 2.0)

set(ECM_HRD_DIR
  include
  ${Envision_SOURCE_DIR}
  ${spdlog_INCLUDE_DIRS_PS}
  ${PCH_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS_PS}

  ${ROS_INCLUDE_DIRS_PS}
  ${AMBF_CLIENT_INCLUDE_DIRS_PS}
  ${AMBF_MSGS_INCLUDE_DIRS_PS}
  )

SET ( ECM_SOURCE_FILES
  src/main.cpp
  # src/Utilities.cpp
  )


set(TESTS_LINK_LIBRARIES
  PRIVATE ${Envision_BINARY_DIR}/libSway.a
  # PRIVATE ${spdlog_BINARY_DIR}/libspdlog.a
  # PRIVATE ${BOOST_LIBRARY_LIBRARY_PS}
  # PRIVATE ${AMBF_CLIENT_LIBRARIES_PS}
  )

#####Beginning of TrialApp#############
# SET ( Trial_SOURCE_FILES
#   src/KUKAKinematicsTests.cc
# )

set(TrialApp Trial_app)
ADD_EXECUTABLE ( ${TrialApp} 
  ${ECM_SOURCE_FILES} 
  # ${KUKATests_SOURCE_FILES}
  )

target_include_directories(${TrialApp}
PRIVATE ${TESTS_HRD_DIR} 
PRIVATE KUKATests/include
)

# target_link_libraries(${KUKATestsApp} 
#   ${TESTS_LINK_LIBRARIES}
# )

set_target_properties(${TrialApp} PROPERTIES
CXX_STANDARD 20
CXX_STANDARD_REQUIRED YES
)

#####End of KUKA Test App#############
