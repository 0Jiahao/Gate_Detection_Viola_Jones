//
// Created by phil on 01/04/2018.
//

#include <sim/Simulation.h>
#include "SimRunner.h"
#include "gui/CmdListener.h"
#include "MotionRead.h"
void SimRunner::startSim(std::function<void()> controlLoop) {

    CmdListener cmdListener = CmdListener();
    cmdListener.addCommand("q","Quit Application",[this](std::string arg0, std::string arg1){ stopSim();});
    cmdListener.addCommand("p","Pause/Continue simulation.",[this](std::string arg0, std::string arg1){togglePause();});
    cmdListener.addCommand("c","Pause/Continue cam stream.",[this](std::string arg0, std::string arg1){toggleCamStream();});
    cmdListener.printHelp();
    this->running = true;
    camThread = std::thread(&CamStream::start,camStream);
	airsimThread = std::thread(&SimRunner::threadAirsim,this);
	simThread = std::thread(&SimRunner::threadSimulation,this,controlLoop);
    
	while(this->running){
        cmdListener.listen();
    }
    if (camStream->isRunning()){
        camStream->stop();
        camThread.join();
    }
    airsimThread.join();
    simThread.join();
}

void SimRunner::stopSim() {
    this->running = false;
    std::cout<<"Stopping simulation.."<<std::endl;
}

void SimRunner::togglePause() {
    this->paused = !this->paused;

    if(this->paused){
        std::cout<<"Control loop paused." << std::endl;
    }else{
        std::cout<<"Control loop running.." <<std::endl;
    }

}

SimRunner::~SimRunner() {
    if (this->running){
        this->stopSim();
    }
}

SimRunner::SimRunner(const SimRunner &old) {
    this->sim = old.sim;
    this->camStream = old.camStream;
}

void SimRunner::threadSimulation(std::function<void()> controlLoop) {
	std::cout << "Starting simulation with control frequency: " << control_frq_hz << ' Hz' << std::endl;
	while(this->running){
        if(!this->paused){
            sim->modelUpdate();
			controlLoop();
			std::this_thread::sleep_for(std::chrono::milliseconds((long long)(1000/control_frq_hz)));

        }
    }
}

void SimRunner::threadAirsim() {
    std::cout << "Starting airsim comm with frequency: " << airsim_frq_hz << ' Hz' << std::endl;
    while(this->running){
        if(!this->paused){
            sim->airsimUpdate();
            std::this_thread::sleep_for(std::chrono::milliseconds((long long)(1000/airsim_frq_hz)));

        }
    }
}



void SimRunner::toggleCamStream() {
    if(camStream->isRunning()){
        std::cout<<"Stopping cam stream.." << std::endl;

        camStream->stop();
        camThread.join();

    }else{
        std::cout<<"Starting cam stream.." << std::endl;
        camThread = std::thread(&CamStream::start,camStream);

    }
}

SimRunner::SimRunner(Simulation *sim, CamStream *camStream) {
    this->sim = sim;
    this->camStream = camStream;
}



