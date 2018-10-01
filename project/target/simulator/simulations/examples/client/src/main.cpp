#include "AirSimInterface.h"
#include <iostream>
#include <thread>
#include <SimClient.h>
#include "CmdListener.h"
AirSimInterface airsim;
void demoControlLoop() {
    static int i=0;
    if(i < 8){
        static vector<float> yaw =		{0, -0.5, 0.5, 0,   0,  0,  0, 0 };
        static vector<float> pitch =	{0,  0,   0,   0.5, 0.5,0,  0, 0 };
        static vector<float> roll =		{0,	 0,	  0,   0,	0,	0.5,0.5,0};
        static vector<float> lift =		{-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5, -0.5};
        //airsim.sendMotionCommand(yaw[i], pitch[i], roll[i], lift[i]);
        i++;
    }
}

int main()
{
	float movement_execution_time = 3;
	airsim = AirSimInterface(movement_execution_time);
	auto client = SimClient(airsim);
	client.startSim([](){demoControlLoop();});

	return 0;
}
