cmake_minimum_required (VERSION 3.5.1)
set (CMAKE_CXX_STANDARD 11)

project(drone_model_test)
set(PROJECT_ROOT ../../../)
set(EXECUTABLE_OUTPUT_PATH ../bin)



set(Eigen3_DIR ${PROJECT_ROOT}/lib/eigen/cmake)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
		${PROJECT_ROOT}/target/simulator/simulations/client/src
)

set(SOURCES
			src/main.cpp
			${PROJECT_ROOT}/target/simulator/simulations/client/src/DroneModel.cpp
			)
set(HEADERS
		${PROJECT_ROOT}/target/simulator/simulations/client/src/DroneModel.h
		    )

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_link_libraries (${PROJECT_NAME} Eigen3::Eigen)






