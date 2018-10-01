cmake_minimum_required (VERSION 3.5.1)
set (CMAKE_CXX_STANDARD 11)

project(vision_test)
set(PROJECT_ROOT ../../../)
set(EXECUTABLE_OUTPUT_PATH ../bin)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)



include(${PROJECT_ROOT}/outerloop/CMakeLists.txt)


set(OpenCV_DIR ${PROJECT_ROOT}/lib/opencv/build)
find_package(OpenCV REQUIRED)

include_directories(	${OUTERLOOP_INCLUDES}
						${OpenCV_INCLUDE_DIRS})

set(SOURCES
			src/VisionMain.cpp
        ${OUTERLOOP_SOURCES}
			)
set(HEADERS
			src/VisionMain.h
        ${OUTERLOOP_INCLUDES}

        )

set(Eigen3_DIR ${PROJECT_ROOT}/lib/eigen/cmake)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)


add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries (${PROJECT_NAME} Eigen3::Eigen)

