# CMakeLists for the common sources. Makes sure that the compiler finds all files. Projects that use the 
# code can include it in their CMakeLists.txt
cmake_minimum_required(VERSION 3.5.1)

set(SRC_DIR ${PROJECT_ROOT}/outerloop/src)

set(OUTERLOOP_SOURCES 				${SRC_DIR}/vision/Pose.cpp
						${SRC_DIR}/vision/PoseEstimator.cpp
						${SRC_DIR}/vision/Frame.cpp
						${SRC_DIR}/vision/SnakeGateDetector.cpp
						${SRC_DIR}/vision/VJGateDetector.cpp
						${SRC_DIR}/vision/Vision.cpp
						${SRC_DIR}/os/Scheduler.cpp
						${SRC_DIR}/filtering/Filter.cpp
						${SRC_DIR}/filtering/State.cpp
						${SRC_DIR}/control/Control.cpp
						${SRC_DIR}/control/Attitude.cpp
						${SRC_DIR}/comm/MotionRead.cpp
						${SRC_DIR}/comm/Comm.cpp
						${SRC_DIR}/vision/Polygon.cpp
						${SRC_DIR}/control/Controller.cpp
						${SRC_DIR}/os/Profiler.cpp
						${SRC_DIR}/os/Log.cpp
						${SRC_DIR}/dronerace/control.cpp
						${SRC_DIR}/dronerace/fifo.cpp
						${SRC_DIR}/dronerace/filter.cpp
						${SRC_DIR}/dronerace/flightplan.cpp
                                                ${SRC_DIR}/dronerace/ransac.cpp
                                                ${SRC_DIR}/dronerace/ransacLinearModel.cpp
		)

set(OUTERLOOP_HEADERS 				${SRC_DIR}/vision/Pose.h
						${SRC_DIR}/vision/PoseEstimator.h
						${SRC_DIR}/vision/SnakeGateDetector.h
						${SRC_DIR}/vision/VJGateDetector.h
						${SRC_DIR}/os/HardwareAbstractionLayer.h
                        			${SRC_DIR}/os/Profiler.h
						${SRC_DIR}/os/Scheduler.h
						${SRC_DIR}/filtering/Filter.h
						${SRC_DIR}/filtering/State.h
						${SRC_DIR}/control/Control.h
						${SRC_DIR}/control/Attitude.h
						${SRC_DIR}/comm/MotionRead.h
						${SRC_DIR}/comm/Comm.h
						${SRC_DIR}/vision/Polygon.h
			     		        ${SRC_DIR}/control/Controller.h
                                                ${SRC_DIR}/Config.h
        					${SRC_DIR}/os/Log.h
						${SRC_DIR}/ConfigJevois.h
						${SRC_DIR}/ConfigSimulator.h
						${SRC_DIR}/dronerace/control.h
						${SRC_DIR}/dronerace/fifo.h
						${SRC_DIR}/dronerace/filter.h
						${SRC_DIR}/dronerace/flightplan.h
                                                ${SRC_DIR}/dronerace/ransac.h
						${SRC_DIR}/dronerace/ransacLinearModel.h

		)

set(OUTERLOOP_INCLUDES 				${SRC_DIR}/filtering
					    	${SRC_DIR}/vision
						${SRC_DIR}/os
						${SRC_DIR}/control
						${SRC_DIR}/comm
						${SRC_DIR}/dronerace
                        			${SRC_DIR}/
						 )



