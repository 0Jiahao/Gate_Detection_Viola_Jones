#include <Scheduler.h>
#include <control.h>
#include "Control.h"

//TODO@simon insert fancy control algorithm here
namespace mav{

Control::Control()
{
    tempModeChange = false;
#ifdef PROFILING
    loopTimer = Profiler::createTimer("ControlLoop");
#endif
}

void Control::updateState(State &state)
{
	controller->updateState(state);
}

void Control::loop(Filter * filter){

#ifdef PROFILING
    Profiler::startTimer(loopTimer);
#endif
    float dt = (getCurrentTimeMillis()-lastTimeStamp)/1000.0;
    float desired_phi,desired_theta,desired_psi,desired_alt;
    control_run(dt,&desired_phi,&desired_theta,&desired_psi,&desired_alt);
    lastTimeStamp = getCurrentTimeMillis();
    controller->setPoint = Attitude(desired_phi,desired_theta,desired_psi,desired_alt);
    //controller->setPoint = Attitude(0.0,0.0,0.0,-2.0);

#ifdef PROFILING
    Profiler::stopTimer(loopTimer);
#endif
}


void Control::setTimeStamp(){
    lastTimeStamp = mav::getCurrentTimeMillis();
};



void Control::initializeControl()
{
}





Attitude Control::getSetPoints(){
	return controller->getSetpoint();
}




void Control::firstPartLogic(Filter * filter)
{

}


void Control::secondPartLogic(Filter * filter)
{
}

    void Control::update(MotionRead &ahrs) {
    }
}