/**
 * Program to test the control loop without using the simulator or real sensors.
 */
#include "Scheduler.h"
#include <thread>
#include <chrono>
#include <iostream>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <opencv/cv.hpp>

using namespace mav;
/**
 * We define a dummy class for the hardware abstraction layer that just writes the output to console
 */
class HALTest:public HardwareAbstractionLayer{
    void print(const std::string &msg) override {
            std::cout<<msg<<std::endl;
    }

    void sendLowLevelMotion(Attitude &setpoint) override {
        std::cout<<"Setpoint:" << setpoint.toString()<<std::endl;
    }
    void sendState(State &state) override{}
};

int main()
{
    /**
     * We configure the system such that our test class is used in all the modules
     */
    HardwareAbstractionLayer *hal = new HALTest();
    Comm::setHal(hal);
    Vision vision;
    Filter filter;
    Control control;
    Comm comm;
    Log log("test");
    
    Scheduler scheduler(vision,control,filter,comm,log);
    hal->setEventHandler(&scheduler);

    /**
     * We create some test data
     */
    auto mat = cv::imread("../../images/00002.bmp");
    Frame testFrame(mat);
    MotionRead ahrs = MotionRead(0,0,0,0,0,0,0,0,0,-4.5);

    /**
     * We run a loop that simulates a real system for 5 minutes
     */
    const double sleepTimeMS = 2;//500Hz

    scheduler.startPeriodicTask();

	for (long long timer = 0; timer < 5*60*60*1000; timer+=sleepTimeMS) {

        // New sensor data with 500hz
	    hal->newSensorData(ahrs);

	    // New images at 10 Hz
		if(timer % 100 == 0){
		    hal->newFrame(testFrame);
		}

		if(timer % 1000 == 0){
		    scheduler.logValues = true;
		    scheduler.logFrames = true;
		}

		std::this_thread::sleep_for(std::chrono::duration<double,std::milli>(sleepTimeMS));

	}
	scheduler.stopPeriodicTask();
    return 0;
}