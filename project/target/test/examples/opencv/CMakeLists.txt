cmake_minimum_required (VERSION 3.5.1)
set (CMAKE_CXX_STANDARD 11)

project(opencv_demo)
set(PROJECT_ROOT ../../../../)
set(EXECUTABLE_OUTPUT_PATH ../bin)

set(OpenCV_DIR ${PROJECT_ROOT}/lib/opencv/build)


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${INCLUDES})

add_executable(opencv_demo src/main.cpp)
message(${OpenCV_LIBS})
target_link_libraries(opencv_demo ${OpenCV_LIBS})
