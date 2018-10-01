cmake_minimum_required(VERSION 3.5.0)
set (CMAKE_CXX_STANDARD 11)
project(opencv_demo)
set(AIRSIM_ROOT ../../../AirSim)
set(CMAKE_BINARY_DIR ../bin)

set(PROJECT_ROOT ../../../../../)

LIST(APPEND CMAKE_MODULE_PATH "${AIRSIM_ROOT}/cmake/cmake-modules") 
include(${PROJECT_ROOT}/target/simulator/simulations/client/CMakeLists.txt)

INCLUDE("${AIRSIM_ROOT}/cmake/cmake-modules/CommonSetup.cmake")
add_subdirectory(${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1 ${CMAKE_CURRENT_BINARY_DIR}/rpclib)

set(OpenCV_DIR ${PROJECT_ROOT}/lib/opencv/build)
find_package(OpenCV REQUIRED)

CommonSetup()

#override some CommonSetup settings:
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
SET(LIBRARY_OUTPUT_PATH ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

IncludeEigen()

SetupConsoleBuild()

include_directories(
  ../../common
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${INCLUDES}
  ${OpenCV_INCLUDE_DIRS}
  ${ASC_INCLUDES}

)

link_directories(${AIRSIM_ROOT}/AirLib/lib/x64/Debug/)

set(SOURCES
			src/main.cpp
			${ASC_SOURCES}
			)
set(HEADERS
		   ${ASC_HEADERS}
		    )

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
			
CommonTargetLink()
target_link_libraries(${PROJECT_NAME} AirLib)
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
