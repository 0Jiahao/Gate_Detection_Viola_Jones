/**
 * Outerloop Simulation
 *
 * This project simulates all modules which will be running on the JeVois.
 *
 * In total we have 3 threads:
 * 	- AirsimInterface Thread -> Gets the images from airsim and updates the camera position according to the dynamic model of the drone.
 * 	- Low-level/Environment Thread -> Runs the dynamic model of sensor and drone as well as the low level controller
 * 	- High-Level Thread -> Runs the high level control loop which will run in similar fashion on the JeVois
 */
#include "airsim/AirSimInterface.h"
#include "Scheduler.h"
#include <iostream>
#include <gui/DroneCamStream.h>
#include "SimRunner.h"
#include "sim/DynamicModel.h"
#include "opencv2/highgui/highgui.hpp"
#include "tools/DataLogger.h"
#include "sim/DroneModel.h"
#include "sim/SampleController.h"
using namespace mav;

#define AIRSIM_UPDATE_RATE_HZ 100.0
#define HIGHLEVEL_UPDATE_RATE_HZ 500.0
int main()
{
	// Create headers for log files
	mav::DataLogger::log("./", "log_controller.csv", "t,x_r,y_r,z_r,v_x_r,v_y_r,v_z_r,psi_r");
	mav::DataLogger::log("./", "log_model.csv", "t,x,y,z,v_x,v_y,v_z,phi,theta,psi,p,q,r,phi_r,theta_r,psi_r,force_0,force_1,force_2");
	mav::DataLogger::log("./", "log_drone.csv", "Timestamp,Ax,Ay,Az,P,Q,R,Phi,Theta,Psi,PosX,PosY,PosZ,StateX,StateY,StateZ,StateBvZ,StatebX,StatebY,StatebZ");
	bool saveImages = false;
	mav::DataLogger dataLogger("./", "log_drone.csv",saveImages);

	// Configure interface to airsim
	AirSimInterface airsim;

	// Create objects for environment simulation
    DroneModel droneModel;
    SensorModel sensorModel;
    auto dynamicModel = new DynamicModel(droneModel,sensorModel);
    // Create simulation object
    SampleController autopilot;
    Simulation sim(dynamicModel, &airsim,&autopilot);

	// Connect simulation with high level control loop
    Comm::setHal((HardwareAbstractionLayer*)&sim);
	Scheduler scheduler;
	scheduler.loggingActive = true;
    scheduler.getLogger()->logFrame = true;

    //Here we subscribe the scheduler to the simulation
    // so that it is informed whenever there is a new image or new sensor data
    sim.setEventHandler(&scheduler);


	// Gui Elements to display internal and external values
	CamStream *camStream = new DroneCamStream(&sim,scheduler.getLogger());

	//SimRunner handles the threads for simulation
	auto simClient = SimRunner(&sim,camStream);
	simClient.control_frq_hz = HIGHLEVEL_UPDATE_RATE_HZ;
    simClient.airsim_frq_hz = AIRSIM_UPDATE_RATE_HZ;
    autopilot.setReference(0,0,0,0);

    // Now we start the scheduler which is like starting the jevois
	scheduler.initControl(0);
	scheduler.startPeriodicTask();


	// Now we start the simulation so the drone model and the communication with airsim
	simClient.startSim([&scheduler,&dataLogger]() {


		if (scheduler.getLogger()->getLogSize() > (int)0.5*LOG_BUFFER_SIZE){
			auto log = scheduler.getLogger()->emptyLog();
			dataLogger.append(log);
		}
    });

    cv::destroyAllWindows();

	return 0;
}
