cmake_minimum_required (VERSION 3.5.1)
set (CMAKE_CXX_STANDARD 11)

project(eigen_demo)
set(PROJECT_ROOT ../../../../)
set(EXECUTABLE_OUTPUT_PATH ../bin)

set(Eigen3_DIR ${PROJECT_ROOT}/lib/eigen/cmake)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(eigen_demo src/main.cpp)
target_link_libraries (eigen_demo Eigen3::Eigen)
