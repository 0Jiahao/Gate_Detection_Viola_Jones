/**
 * SimRunner.
 * Connects Simulation with HighLevel Control Loop and User Interface elements.
 */
//
// Created by mail- on 01/04/2018.
//

#ifndef AIRSIMCLIENT_H
#define AIRSIMCLIENT_H


#include "airsim/AirSimInterface.h"
#include "gui/CamStream.h"
#include "sim/DynamicModel.h"
#include "HardwareAbstractionLayer.h"
class SimRunner {

protected:
    Simulation *sim;
    CamStream *camStream;
    std::thread simThread,airsimThread,camThread;
    std::atomic<bool> running=false;
    std::atomic<bool> paused=false;
    void threadSimulation(std::function<void()> controlLoop);
    void threadAirsim();
public:
    ~SimRunner();
    SimRunner() = default;
    SimRunner(const SimRunner &);
    SimRunner(Simulation *sim, CamStream *camStream);

    std::atomic<float> control_frq_hz = 100;
    std::atomic<float> airsim_frq_hz=500;

    /**
     * Start the simulation.
     * @param controlLoop function that contains high level control loop
     */
	virtual void startSim(std::function<void()> controlLoop);
	virtual void stopSim();
	virtual void togglePause();
    virtual void toggleCamStream();
};


#endif //CLIENT_DEMO_AIRSIMCLIENT_H
