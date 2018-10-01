#include "Vision.h"
#include "Frame.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
namespace mav{


void Vision::gateDetection(){
#ifdef PROFILING
    Profiler::startTimer(gateDetectionTimer);
#endif

	detector->snake_gate_detection(this->frame);
#ifdef PROFILING
    Profiler::stopTimer(gateDetectionTimer);
#endif
}

void Vision::poseEstimation() {
#ifdef PROFILING
    Profiler::startTimer(poseEstimationTimer);
#endif

	poseEstimator->updateGate(detector->getBestGate());
    poseEstimator->estimatePose();

#ifdef PROFILING
    Profiler::stopTimer(poseEstimationTimer);
#endif
}

Pose Vision::getPose(){
	return  poseEstimator->getCurrentPoseEstimate();
}

bool Vision::hasNewGate() {
	return detector->hasNewGate();
}

void Vision::clearNewGate()
{
	return detector->clearNewGate();
}


void Vision::updateAHRS(MotionRead ahrsReadings) {
    poseEstimator->updateAHRS(ahrsReadings);
}

Vision::Vision() {
    detector = new SnakeGateDetector();
    poseEstimator = new PoseEstimator();
#ifdef PROFILING
    gateDetectionTimer = Profiler::createTimer("SnakeGate");
    poseEstimationTimer = Profiler::createTimer("PoseEstimate");

#endif
}

Polygon
Vision::
getBestGate()
{
    return detector->getBestGate();
}

void
Vision::
updateFrame(Frame &frame)
{
	this->frame = frame;
}

}
