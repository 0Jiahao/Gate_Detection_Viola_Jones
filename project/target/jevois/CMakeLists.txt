######################################################################################################################
#
# JeVois Smart Embedded Machine Vision Toolkit - Copyright (C) 2016 by Laurent Itti, the University of Southern
# California (USC), and iLab at USC. See http://iLab.usc.edu and http://jevois.org for information about this project.
#
# This file is part of the JeVois Smart Embedded Machine Vision Toolkit.  This program is free software; you can
# redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software
# Foundation, version 2.  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
# without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
# License for more details.  You should have received a copy of the GNU General Public License along with this program;
# if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# Contact information: Laurent Itti - 3641 Watt Way, HNB-07A - Los Angeles, BA 90089-2520 - USA.
# Tel: +1 213 740 3527 - itti@pollux.usc.edu - http://iLab.usc.edu - http://jevois.org
######################################################################################################################

## CMake build rules for JeVois Base library and executables

## You may provide the installed JeVois config root as:
#   cmake -DJEVOIS_CONFIG=/jevois/config ..

cmake_minimum_required(VERSION 3.1)
SET(CMAKE_CXX_FLAGS "-std=c++0x")
set(PROJECT_ROOT ../..)
get_filename_component(PROJECT_ROOT ${PROJECT_ROOT} ABSOLUTE)
message("Project Dir:")
message(${PROJECT_ROOT})

## Set vendor names, our modules will be placed in a directory by that name under /jevois/modules:
set(JEVOIS_VENDOR "MavLab")
set(JEVOIS_CONFIG "/jevois/config" CACHE STRING "Path to JeVois config to use")

## Include helper functions, config, etc from the JeVois install:
set(CMAKE_MODULE_PATH ${JEVOIS_CONFIG})
include(jevois_config)
include(JeVois)

project(mavpilotbase)
jevois_project_set_flags()

include(${PROJECT_ROOT}/outerloop/CMakeLists.txt)

# Setup our library using the source files in src/Components and call it mavpilotbase with the current version number:
jevois_setup_library(src/Components mavpilotbase 0.1.0)

# Setup our modules that are in src/Modules, make them depend on mavpilotbase:
jevois_setup_modules(src/Modules mavpilotbase)
############ Include External Code #############################
# Include Eigen
set(Eigen3_DIR ${PROJECT_ROOT}/lib/eigen/cmake)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# Include Mavlink
include_directories(${PROJECT_ROOT}/protocol/
	)

# Include our stuff from outerloop
include_directories(
                    ${OUTERLOOP_INCLUDES}
        ${PROJECT_ROOT}/target/jevois/include/Components
        )


## If your module will use components provided by jevoisbase, uncomment the lines below:
if (JEVOIS_PLATFORM)
  link_directories("/var/lib/jevois-microsd/lib/JeVois") # find libjevoisbase.so for platform
endif (JEVOIS_PLATFORM)
include_directories("/var/lib/jevois-build/usr/include") # find jevoisbase includes

## If you also want to use some of the jevoisbase contribs, just add the appropriate include paths here.  You will also
## need to set JEVOIS_SRC_ROOT environment variable in your shell to the parent of the jevoisbase directory, e.g., type:
#   export JEVOIS_SRC_ROOT=/home/myname
## in your shell.
#include_directories("$ENV{JEVOIS_SRC_ROOT}/jevoisbase/Contrib/darknet-nnpack/include")
#include_directories("$ENV{JEVOIS_SRC_ROOT}/jevoisbase/Contrib/NNPACK/include")
#include_directories("$ENV{JEVOIS_SRC_ROOT}/jevoisbase/Contrib/pthreadpool/include")


## Setup our modules that are in src/Modules. First arg: source directory for modules; 2nd arg: target build
## dependencies (i.e., cmake targets which must be built here before we build the modules), usually empty for a single
## module. See the CMakeLists.txt in jevoisbase for an example where we first build a shared library for all shared
## components, and then each module gets this library as a target dependency:

target_sources(mavpilotbase 	PUBLIC
                        	${HAL_JEVOIS_SOURCES}
                        	${HAL_JEVOIS_HEADERS}
                        	${OUTERLOOP_SOURCES}
                        	${OUTERLOOP_HEADERS}
			        )
add_definitions(-DTARGET_JEVOIS=1
                #-DDEBUG_SCHEDULING=1
                -DPROFILING=1
                -Wno-reorder
                -Wno-unused-parameter
                -Wno-unused-function
                -Wno-unused-variable
                )

## Add any link libraries for each module. Add 'jevoisbase' here if you want to link against it:
target_link_libraries(mavpilotbase ${JEVOIS_OPENCV_LIBS} opencv_imgproc opencv_core)
target_link_libraries (mavpilotbase Eigen3::Eigen)
#target_link_libraries(mavpilotbase jevoisbase)

## Install any shared resources (cascade classifiers, neural network weights, etc) in the share/ sub-directory:
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/share"
  DESTINATION "${JEVOIS_INSTALL_ROOT}" COMPONENT bin)

## Wrapping up:
jevois_project_finalize()  # call this last in your CMakeLists.txt, will un-mount the live microSD if applicable
