//
// Created by Video on 24/07/2018.
//

#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include <State.h>
#include <mutex>
#include <atomic>
#include "Attitude.h"
#include "Control.h"

namespace mav {

enum CONTROLLER_LIST {CONTROLLER_GO_THROUGH_GATE,CONTROLLER_LAND,CONTROLLER_ARC};

    class SecondOrderButterWorthFilter{
    public:
        SecondOrderButterWorthFilter() = default;
        ~SecondOrderButterWorthFilter() = default;
        void initFilter();
        float getFilteredResult(){};
        void processFilter(float newDate);
    private:
    };

class Controller {
public:
    Controller();
    ~Controller() = default;
    void init();
    void updateState(State &state){
        guard.lock();
        this->state = state;
        guard.unlock();
    };
    void update(MotionRead &ahrs){
        guard.lock();
        this->ahrs = ahrs;
        guard.unlock();
    };
    Attitude getSetpoint(){ return setPoint; };
    void idle();
    bool goThroughGate(float desiredTheta,float desiredPsi,float desiredAltitude,float endPosition);
    bool arc(float radius,float desiredTheta,float deltaPsi,float desiredAltitude,bool right,int gateNumber);
    bool land();
    void setTimeStamp();
    long long getTime();
    float getArcTargetHeading(){return arcTargetHeading;};
    void setKp(float k_p){k_p_x = k_p;};
    void setKd(float k_d){k_d_x = k_d;};
    Attitude setPoint;


protected:
    Vector3f arcStates;
    Vector3f arcInputs;
    float arcTargetHeading;
    enum CONTROLLER_LIST controllerInUse;
    long long controllerTimeStamp;
    State state;
    MotionRead ahrs;
    float previousError;
    long long lastStepTime;
    std::mutex guard;
    SecondOrderButterWorthFilter filteredD;
    std::atomic<float> k_p_x;
    std::atomic<float> k_d_x;

};



}

#endif //OUTERLOOP_SIM_CONTROLLERS_H
