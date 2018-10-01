//Filtering
//Takes motion sensors + vision and estimates state
#pragma once

#include <mutex>
#include "MotionRead.h"
#include "State.h"
#include "Pose.h"
#include "Eigen/Dense"


#ifndef K_X
#define K_X -0.5
#endif

#ifndef K_Y
#define K_Y -0.5
#endif

#ifndef QX
#define QX 0.2
#endif

#ifndef QY
#define QY 0.2
#endif


#ifndef QZ
#define QZ 0.2
#endif


#ifndef QV
#define QV 0.1
#endif

#ifndef QBX
#define QBX 0.0001
#endif

#ifndef QBY
#define QBY 0.0001
#endif


#ifndef QBZ
#define QBZ 0.0001
#endif

#ifndef RX
#define RX 0.1
#endif

#ifndef RY
#define RY 0.1
#endif

#ifndef RZ
#define RZ 0.1
#endif

#ifndef KALMAN_STEP
#define KALMAN_STEP 1.0/512.0
#endif

#include "Profiler.h"

using namespace Eigen;

namespace mav{

class Filter
{

public:
	Filter();
	~Filter() = default;
	Filter(const Filter &old);
	void predictStates();
	void updateSensorReadings(MotionRead sensorReadings);
	//todo::
	void clearFlagRunEKF(){flagRunEKF = false;};
	void setFlagRunEKF(mav::State initialEstimation);
	bool getFlagRunEKF();
	void updateState(Pose & pose);
	State getState(){
	    guard.lock();
	    State state(this->state);
	    guard.unlock();
        return state;
	};
	MotionRead getSensorReadings(){
        guard.lock();
        MotionRead readings(this->sensorReadings);
        guard.unlock();
        return readings;
	}
    float getX(){return state.getX();};
	float getY(){return state.getY();};
	float getVX(){return state.getZ();};
	float getVY(){return state.getV_z_B();};
	float getPsi(){return state.getBx();};
private:
    std::mutex guard;
    State state;
    MotionRead sensorReadings;
    mav::Pose pose;
    bool flagRunEKF;
    Matrix<double,7,7> P;
    Matrix<double,7,7> Q;
    Matrix<double,3,3> R;
    long long lastUpdateTime;
    float phi;
    float theta;
    float psi;
    float a_x;
    float a_y;
    float a_z;
    float p;
    float q;
#ifdef PROFILING
    int predictTimer,updateTimer;
#endif

};

}