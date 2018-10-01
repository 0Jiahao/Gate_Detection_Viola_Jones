cmake_minimum_required (VERSION 3.5.1)
set (CMAKE_CXX_STANDARD 11)

project(jevois_in_the_loop)
set(PROJECT_ROOT ../../../..)
set(AIRSIM_ROOT ${PROJECT_ROOT}/target/simulator/AirSim)
set(EXECUTABLE_OUTPUT_PATH ../bin/)
set(CMAKE_BINARY_DIR ../bin/)

set(CMAKE_BUILD_TYPE Debug)
include_directories(${PROJECT_ROOT}/protocol/)

include(${PROJECT_ROOT}/outerloop/CMakeLists.txt)
include(${PROJECT_ROOT}/target/simulator/simulations/client/CMakeLists.txt)


LIST(APPEND CMAKE_MODULE_PATH "${AIRSIM_ROOT}/cmake/cmake-modules")

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
        ${PROJECT_ROOT}/protocol/
        ${AIRSIM_ROOT}/AirLib/include
        ${RPC_LIB_INCLUDES}
        ${OpenCV_INCLUDE_DIRS}
        ${ASC_INCLUDES}
        ${OUTERLOOP_INCLUDES}
)

link_directories(${AIRSIM_ROOT}/AirLib/lib/x64/Debug/)

set(SOURCES
        src/Main.cpp
        src/Main.h
        src/MavlinkInterface.cpp
        src/SerialPort.cpp
        src/SerialPort.h
        src/MavlinkInterface.h
        src/SerialPortAdapter.h
        ${ASC_SOURCES}
        ${OUTERLOOP_SOURCES}
        ${OUTERLOOP_HEADERS})
if(UNIX)
    list(APPEND SOURCES src/SerialPortAdapterUnix.cpp
            src/SerialPortAdapterUnix.h)
elseif(WIN32)
    list(APPEND SOURCES src/SerialPortAdapterWin.cpp
            src/SerialPortAdapterWin.h)
endif()

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})


if(UNIX)
    target_link_libraries(${PROJECT_NAME} pthread)
endif()

target_compile_definitions(${PROJECT_NAME} PRIVATE TARGET_SIMULATOR=1)

CommonTargetLink()
target_link_libraries(${PROJECT_NAME} AirLib)
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})