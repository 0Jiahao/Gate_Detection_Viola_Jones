/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink_control.cpp
 *
 * @brief An example offboard control process via mavlink
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include <Attitude.h>
#include <HardwareAbstractionLayer.h>
#include <opencv2/highgui.hpp>
#include "airsim/AirSimInterface.h"
#include <iostream>
#include "SimRunner.h"
#include "sim/DynamicModel.h"
#include "tools/DataLogger.h"
#include "sim/DroneModel.h"
#include "sim/SampleController.h"
#include "MavlinkInterface.h"
#include "Main.h"

using namespace mav;

#define AIRSIM_UPDATE_RATE_HZ 500.0
#define SIM_UPDATE_RATE_HZ 500.0

/***
 * Event Handler for Jevois-In-The-Loop simulation.
 * - new sensor values are streamed via mavlink
 */
class HILEventHandler: public EventHandler {
private:
	MavlinkInterface *imav;
public:
	HILEventHandler(MavlinkInterface *imav){
		this->imav = imav;
	}

	void onNewSensorData(MotionRead &motionRead) override {
		mavlink_highres_imu_t imu;
		imu.xgyro = static_cast<float>(motionRead.getP());
		imu.ygyro = static_cast<float>(motionRead.getQ());
		imu.zgyro = static_cast<float>(motionRead.getR());
		imu.xacc = static_cast<float>(motionRead.getA_x());
		imu.yacc = static_cast<float>(motionRead.getA_y());
		imu.zacc = static_cast<float>(motionRead.getA_z());
		imu.abs_pressure = static_cast<float>(motionRead.getZ());
		imu.time_usec = get_time_usec();
		mavlink_message_t message;
		mavlink_msg_highres_imu_encode(static_cast<uint8_t>(imav->system_id),
									   static_cast<uint8_t>(imav->companion_id), &message, &imu);
        //std::cout  << "Streaming ahrs:" << motionRead.toString() << std::endl;

        imav->write_message(message);
	}


};




// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int
top (int argc, char **argv)
{

	// --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------

	// Default input arguments
#ifdef __APPLE__
	char *uart_name = (char*)"/dev/tty.usbmodem1";
#else
	char *uart_name = (char*)"COM4";
#endif
	int baudrate = 115200;

	// do the parse, will throw an int if it fails
	parse_commandline(argc, argv, uart_name, baudrate);

	printf("Parsed %s, %d",uart_name,baudrate);

    // --------------------------------------------------------------------------
    //   Start listening to mavlink messages on serial port
    // --------------------------------------------------------------------------

    Serial_Port serial_port(uart_name, baudrate);
	MavlinkInterface mavlinkInterface(&serial_port);
	serial_port.start();
	mavlinkInterface.start();

    // --------------------------------------------------------------------------
    //   Configure Logging
    // --------------------------------------------------------------------------
	DataLogger::log("./", "log_controller.csv", "t,x_r,y_r,z_r,v_x_r,v_y_r,v_z_r,psi_r");
	DataLogger::log("./", "log_model.csv", "t,x,z,z,v_x,v_y,v_z,phi,theta,psi,p,q,r,phi_r,theta_r,psi_r,force_0,force_1,force_2");
	bool saveImages = false;

    // --------------------------------------------------------------------------
    //   Configure Simulator
    // --------------------------------------------------------------------------

    // Create objects for environment simulation
	DroneModel droneModel;
	SensorModel sensorModel;
	DynamicModel dynamicModel(droneModel,sensorModel);
	SampleController autopilot;

	// Create simulation object
	AirSimInterface airsim;
	Simulation sim(&dynamicModel,&airsim,&autopilot);

    CamStream camStream(&sim);
    SimRunner simClient(&sim,&camStream);
    simClient.control_frq_hz = SIM_UPDATE_RATE_HZ;
    simClient.airsim_frq_hz = AIRSIM_UPDATE_RATE_HZ;

    // subscribe event handler
    sim.setEventHandler(new HILEventHandler(&mavlinkInterface));

    // --------------------------------------------------------------------------
    //   Start simulation. Stream setpoints received from mavlink to airsim
    // --------------------------------------------------------------------------
    sim.sendLowLevelMotion(Attitude(0,0,0,0));

	simClient.startSim([&mavlinkInterface,&sim]() {
		auto msgs = mavlinkInterface.getMessages();
            float z = msgs.altitude.altitude_terrain;
            float roll = msgs.attitude.roll;
            float pitch = msgs.attitude.pitch;
            float yaw = msgs.attitude.yaw;
            Attitude setPoint(roll,pitch,yaw,z);
            //std::cout  << "Setpoint:" << setPoint.toString() << std::endl;
            sim.sendLowLevelMotion(setPoint);


	});

	cv::destroyAllWindows();


	// --------------------------------------------------------------------------
	//   RUN COMMANDS
	// --------------------------------------------------------------------------

	/*
	 * Now we can implement the algorithm we want on top of the autopilot interface
	 */
	//commands(autopilot_interface);

	// --------------------------------------------------------------------------
	//   THREAD and PORT SHUTDOWN
	// --------------------------------------------------------------------------

	/*
	 * Now that we are done we can stop the threads and close_port the port
	 */
	mavlinkInterface.stop();
	serial_port.stop();


	// --------------------------------------------------------------------------
	//   DONE
	// --------------------------------------------------------------------------

	// woot!
	return 0;

}


// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void
commands(MavlinkInterface &api)
{
	//---------------------------------------------------------------------------
	//Send some dummy data for 2 minutes
	//---------------------------------------------------------------------------
	for (int i=0; i<120*2; i++){
		mavlink_highres_imu_t imu;
		imu.pressure_alt = (float)i;
		imu.xacc = (float)i/1;
		imu.yacc = (float)i/2;
		imu.zacc = (float)i/4;
		imu.xgyro = (float)i*3;
		imu.ygyro = 0.0;
		imu.zgyro = static_cast<float>(0.1 + (float)i);
		imu.time_usec = get_time_usec();
		mavlink_message_t message;
		mavlink_msg_highres_imu_encode(static_cast<uint8_t>(api.system_id), static_cast<uint8_t>(api.companion_id), &message, &imu);
		int len = api.write_message(message);
		// check the write
		if ( len <= 0 )
			fprintf(stderr,"WARNING: could not send imu \n");

		std::this_thread::sleep_for(std::chrono::duration<double,std::milli>(500));
	}

}


// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

	// string for command line usage
	const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

	}
	// end: for each input argument

	// Done!
	return;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit(sig);
	}
	catch (int error){}

	// serial port
	try {
		serial_port_quit->handle_quit(sig);
	}
	catch (int error){}

	// end program here
	exit(0);

}


// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
{
	// This program uses throw, wrap one big try/catch here
	try
	{
		int result = top(argc,argv);
		return result;
	}

	catch ( int error )
	{
		fprintf(stderr,"mavlink_control threw exception %i \n" , error);
		return error;
	}

}


