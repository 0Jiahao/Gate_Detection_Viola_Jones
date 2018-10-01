/**
 * MavSim
 * Handles simulation of environment and hardware.
 * AHRS and Drone are simulated from us (DynamicModel)
 * Camera, Obstacles come from airsim.
 */
//
// Created by phil on 06/06/2018.
//

#ifndef MAVSIM_H
#define MAVSIM_H


#include <airsim/AirSimInterface.h>
#include <HardwareAbstractionLayer.h>
#include "DynamicModel.h"
#include "SampleController.h"
using namespace mav;
class Simulation: public HardwareAbstractionLayer {
protected:
    DynamicModel *model;
    AirSimInterface *airsim;
    SampleController *autopilot;
    std::mutex controllerGuard;
	static long long Simulation::getCurrentTimeMillis();

public:
    Simulation(mav::DynamicModel *model, AirSimInterface *airsim, SampleController *autopilot);
    void setRefPosition(float x, float y, float z, float heading);
    void sendLowLevelMotion(Attitude command) override ;
    void sendState(State &state) override;
    Image getImage(){
        return airsim->getImage();
    }
    void print(const std::string &msg) override ;
    Simulation(Simulation &old);
	void modelUpdate();
	void airsimUpdate();

};


#endif //OUTERLOOP_SIM_MAVSIM_H
