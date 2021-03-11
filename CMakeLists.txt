#  Software License Agreement (BSD License)
#  Copyright (c) 2020, AMBF.
#
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  * Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above
#  copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided
#  with the distribution.
#
#  * Neither the name of authors nor the names of its contributors may
#  be used to endorse or promote products derived from this software
#  without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  $Author: seb $
#  $Author: Shreyas Chandra Sekhar $
#  $Date:  February, 2021 $
#  $Rev: $


#
# project configuration
#
# COURTESY:


cmake_minimum_required (VERSION 3.8.0)
project (AutomatingECMProject)
set(CMAKE_CXX_STANDARD 17)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)


include_directories(${catkin_INCLUDE_DIRS})

get_filename_component(PARENT_CMAKE ../cmake ABSOLUTE)
set (CMAKE_MODULE_PATH ${PARENT_CMAKE} )

find_package (Boost COMPONENTS program_options filesystem REQUIRED)

include_directories (${PROJECT_SOURCE_DIR}/src/AutmatingECM/external/Eigen)

# static library external dependencies source files
file (GLOB_RECURSE source_eigen     RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/external/Eigen/Eigen/*)
file (GLOB_RECURSE source_ambf     ABSOLUTE /mnt/OneTB/localcodebase/ambf_repos/aimlab/ambf/*)
#MESSAGE(STATUS "Found AMBF: ${source_ambf}")

# group source files (MSVC likes this)
foreach (FILE ${source} ${source_eigen} ${source_ambf})
  get_filename_component (PARENT_DIR "${FILE}" PATH)
  string (REGEX REPLACE "(\\./)?(src|include)/?" "" GROUP "${PARENT_DIR}")
  string (REPLACE "/" "\\" GROUP "${GROUP}")
  source_group ("${GROUP}" FILES "${FILE}")
endforeach ()

# output location
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)


#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ecm_manupulation_modules)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ecm_manupulation)

