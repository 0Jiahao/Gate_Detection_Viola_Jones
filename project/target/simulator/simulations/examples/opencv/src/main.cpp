#include "AirSimInterface.h"
#include <iostream>
#include <thread>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"

AirSimInterface airsim;
std::atomic<bool> stop = false;
void demoControlLoop() {
	vector<float> yaw =		{0, -0.5, 0.5, 0,   0,  0,  0, 0 };
	vector<float> pitch =	{0,  0,   0,   0.5, 0.5,0,  0, 0 };
	vector<float> roll =	{0,	 0,	  0,   0,	0,	0.5,0.5,0};
	vector<float> lift =	{-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5, -0.5};

	for (int i=0; i<8; i++){
		airsim.sendMotionCommand(yaw[i], pitch[i], roll[i], lift[i]);
		auto image = airsim.getImage();
		cv::Mat mat = cv::Mat(image.getHeight(),image.getWidth(),CV_8UC4);
		if (stop) {
			break;
		}
	}

}

void camStream() {
	for (;;){
		auto image = airsim.getImage();
		cv::Mat mat = cv::Mat(image.getHeight(),image.getWidth(),CV_8UC4);
		memcpy(mat.data,image.getData().data(),image.getData().size()*sizeof(uint8_t));
		cv::imshow("Front Camera",mat);
        cv::waitKey(1);
		if (stop) {
			break;
		}
	}

}

void cmdListener() {
	while (airsim.isConnected() && !stop)
	{
		std::string command;
		std::getline(std::cin, command);
		std::cout << "Received command: " << command << std::endl;

		if (command == "q") {
			stop = true;
			std::cout << "Terminating loop..." << command << std::endl;
		}
		else {
			std::cout << "Unknown Command" << std::endl;
		}
	}
}
int main()
{
	float movement_execution_time = 3;
	airsim = AirSimInterface(movement_execution_time);
	airsim.connect();
	airsim.takeControl();

	std::thread threadLoop(demoControlLoop);
	std::thread camLoop(camStream);

	cmdListener();

	threadLoop.join();
	camLoop.join();
	airsim.land();
	airsim.leaveControl();
	airsim.disconnect();


	return 0;
}
