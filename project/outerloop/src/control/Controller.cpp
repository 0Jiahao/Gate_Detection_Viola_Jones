//
// Created by Video on 24/07/2018.
//


#include <Scheduler.h>
#include "Controller.h"

namespace mav{
void Controller::init() {
    lastStepTime = getCurrentTimeMillis();
}
bool Controller::goThroughGate(float desiredTheta,float desiredPsi,float desiredAltitude,float endPosition) {
        return true;

}

bool Controller::land()
{
        return true;
}

bool Controller::arc(float radius,float desiredTheta,float deltaPsi,float desiredAltitude,bool right,int gateNumber) {
        return true;
}

void Controller::setTimeStamp(){this->controllerTimeStamp = getCurrentTimeMillis()
 ;}
long long Controller::getTime(){return getCurrentTimeMillis()-this->controllerTimeStamp/1000.0;
 };

    void SecondOrderButterWorthFilter::processFilter(float newDate)
    {
    }


    void SecondOrderButterWorthFilter::initFilter()
    {
    }


Controller::Controller()
{
}
}