cmake_minimum_required(VERSION 3.5.1)
set (CMAKE_CXX_STANDARD 11)

set(OpenCV_DIR ${PROJECT_ROOT}/lib/opencv/build)
find_package(OpenCV REQUIRED)

include_directories(
		${OpenCV_INCLUDE_DIRS})

set(SRC_DIR ${PROJECT_ROOT}/target/simulator/simulations/client/src)
set(ASC_INCLUDES ${SRC_DIR})
set(ASC_SOURCES 
						${SRC_DIR}/airsim/AirSimInterface.cpp
						${SRC_DIR}/airsim/Session.cpp
						${SRC_DIR}/Image.cpp
						${SRC_DIR}/gui/CmdListener.cpp
						${SRC_DIR}/SimRunner.cpp
						${SRC_DIR}/gui/CamStream.cpp
                        ${SRC_DIR}/sim/DroneModel.cpp
                        ${SRC_DIR}/sim/DynamicModel.cpp
                        ${SRC_DIR}/sim/Simulation.cpp
                        ${SRC_DIR}/sim/SampleController.cpp
                        ${SRC_DIR}/sim/SensorModel.cpp
                        ${SRC_DIR}/tools/DataLogger.cpp
        ${SRC_DIR}/gui/DroneCamStream.cpp

		)
set(ASC_HEADERS 
						${SRC_DIR}/airsim/AirSimInterface.h
						${SRC_DIR}/airsim/Session.h
						${SRC_DIR}/Image.h
						${SRC_DIR}/gui/CmdListener.h
						${SRC_DIR}/SimRunner.h
						${SRC_DIR}/gui/CamStream.h
						${SRC_DIR}/sim/Simulation.h
                        ${SRC_DIR}/sim/DroneModel.h
                        ${SRC_DIR}/sim/DynamicModel.h
                        ${SRC_DIR}/sim/SampleController.h
                        ${SRC_DIR}/sim/SensorModel.h
                        ${SRC_DIR}/tools/DataLogger.h
        ${SRC_DIR}/gui/DroneCamStream.h


        )




