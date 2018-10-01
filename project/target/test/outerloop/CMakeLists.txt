cmake_minimum_required (VERSION 3.5.1)
set (CMAKE_CXX_STANDARD 11)

project(outerloop_test)
set(PROJECT_ROOT ../../../)
set(EXECUTABLE_OUTPUT_PATH ../bin)
include(${PROJECT_ROOT}/outerloop/CMakeLists.txt)


set(OpenCV_DIR ${PROJECT_ROOT}/lib/opencv/build)
find_package(OpenCV REQUIRED)
set(Eigen3_DIR ${PROJECT_ROOT}/lib/eigen/cmake)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)


include_directories(	${OUTERLOOP_INCLUDES}
						${OpenCV_INCLUDE_DIRS}
						)

set(SOURCES
			src/main.cpp
			${OUTERLOOP_SOURCES}
			)
set(HEADERS
		    ${OUTERLOOP_HEADERS}
		    )

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_compile_definitions(${PROJECT_NAME} PRIVATE TARGET_SIMULATOR=1)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries (${PROJECT_NAME} Eigen3::Eigen)
target_link_libraries (${PROJECT_NAME} pthread)






