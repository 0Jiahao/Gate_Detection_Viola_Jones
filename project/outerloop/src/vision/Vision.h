//Vision
//Reads camera image and produces gate 2d/3d estimate
#pragma once

#include "HardwareAbstractionLayer.h"
#include "Pose.h"
#include "SnakeGateDetector.h"
#include "PoseEstimator.h"
#include "Profiler.h"

namespace mav{
class Vision
{

public:
	~Vision() = default;
	Vision();
	void updateFrame(Frame& frame);
	void updateAHRS(MotionRead ahrsReadings);
	void gateDetection();
	void poseEstimation();
	bool hasNewGate();
	void clearNewGate();
	Pose getPose();
	Polygon getBestGate();
    SnakeGateDetector* getDetector(){return detector;}
	PoseEstimator* getPoseEstimator(){return poseEstimator;}
protected:
	Frame frame;
	SnakeGateDetector *detector;
	PoseEstimator *poseEstimator;
#ifdef PROFILING
	int gateDetectionTimer,poseEstimationTimer;
#endif
};
}