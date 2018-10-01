#include "AirSimInterface.h"
#include "HalSimu.h"
#include "OsSimu.h"
#include "Scheduler.h"
#include <iostream>
#include <SimClient.h>
#include <opencv2/highgui.hpp>

using namespace mav;
int main()
{
	const float movement_execution_time = 3;
	auto airsim = AirSimInterface(movement_execution_time);
	//TODO shared ptr hasn't been fully tested with simulator and might need fixing
	std::shared_ptr<HalSimu> hal = (std::shared_ptr<HalSimu> ) new HalSimu(airsim);
	auto os = OsSimu();
	Scheduler::setHal(hal);
	Scheduler::setOs(&os);
	auto simClient = SimClient(airsim);

	Vision vision;
	simClient.startSim([&vision]() {
		vision.readFrame();
		vision.gateDetection();
		vision.draw_gate_candidates();
		vision.writeMessage();
        Frame currentFrame = vision.getCurrentFrame();
        cv::imshow( "Detections", currentFrame.getMat() );
        cv::waitKey(1);

	});


	return 0;
}
