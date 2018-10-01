//#include <Scheduler.h>
#include <Scheduler.h>
#include <ransac.h>
#include <flightplan.h>
#include "Filter.h"
#include "Comm.h"
#include "filter.h"
namespace mav{
//TODO put EKF here

Filter::Filter()
{

	filter_reset();
	std::cout << "[Filter()] initialized P is " << P << std::endl;
    clearFlagRunEKF();

#ifdef PROFILING
    updateTimer = Profiler::createTimer("FilterUpdate");
    predictTimer = Profiler::createTimer("FilterPredict");
#endif
}

void Filter::updateSensorReadings(MotionRead sensorReadings){
    guard.lock();
	this->sensorReadings = sensorReadings;
	guard.unlock();
}

void Filter::setFlagRunEKF(mav::State initialEstimation)
{
	flagRunEKF = true;
	//std::cout<< "[setFlagRunEKF] initial states is" <<  initialEstimation.getX() << std::endl<< initialEstimation.getY() << std::endl<<initialEstimation.getZ()<<std::endl;
	state = initialEstimation;
    guard.lock();
    phi = sensorReadings.getPhi();
	theta= sensorReadings.getTheta();
	psi = sensorReadings.getPsi();
	a_x = sensorReadings.getA_x();
	a_y = sensorReadings.getA_y();
	a_z = sensorReadings.getA_z();
	p = sensorReadings.getP();
	q = sensorReadings.getQ();
    guard.unlock();

    lastUpdateTime = getCurrentTimeMillis();
	//std::cout<< "[setFlagRunEKF] initial states is" <<  this-> state.getX() << std::endl<< this-> state.getY() << std::endl<< this-> state.getZ()<<std::endl;

}


void Filter::predictStates()
{
#ifdef PROFILING
    Profiler::startTimer(predictTimer);
#endif
//	std::string msg = "Prediction is running\n";
//	Comm::print(msg);
	// one step prediction \dot{x} = f(x)

	std::cout << "[updateState] EKF is running " << std::endl;

	long long currentTime = getCurrentTimeMillis();
    float dt = (currentTime - lastUpdateTime)/1000.0;
	filter_predict(phi,theta,psi,dt);

	Matrix<double, 7, 1>  currentStates;
	//currentStates << dr_state.x,dr_state.y,dr_state.vx,dr_state.vy,dr_state.psi,dr_state.x-dr_ransac.corr_x,dr_state.y-dr_ransac.corr_y;
    currentStates << dr_state.x,dr_state.y,dr_state.vx,dr_state.vy,dr_state.psi,mx,my;

	state.setStateVector(currentStates);

    guard.lock();

    phi = sensorReadings.getPhi();
    theta= sensorReadings.getTheta();
    psi = sensorReadings.getPsi();
    a_x = sensorReadings.getA_x();
    a_y = sensorReadings.getA_y();
    a_z = sensorReadings.getA_z();
    p = sensorReadings.getP();
    q = sensorReadings.getQ();
    guard.unlock();

    lastUpdateTime = currentTime;
#ifdef PROFILING
    Profiler::stopTimer(predictTimer);
#endif
}

bool Filter::getFlagRunEKF() {
	return flagRunEKF;
}

//
void Filter::updateState(Pose &pose)
{
#ifdef PROFILING
    Profiler::startTimer(updateTimer);
#endif

        // todo: is dx = x? what is dx?
        //if(fabs(dr_fp.gate_psi - sensorReadings.getPsi())<RadOfDeg(10.0))
        if(1)
        {
            float scale_x = 1.11;
            float scale_y = 0.88;
            dr_vision.dx = scale_x * pose.getX();
            dr_vision.dy = scale_y * pose.getY();
            dr_vision.dz = pose.getZ();
            filter_correct();
        }
#ifdef PROFILING
        Profiler::stopTimer(updateTimer);
#endif
}

Filter::
Filter(const Filter &old) :
    guard(),
    state(old.state),
    sensorReadings(old.sensorReadings),
    pose(old.pose),
    flagRunEKF(old.flagRunEKF),
    P(old.P),
    Q(old.Q),
    R(old.R),
    lastUpdateTime(old.lastUpdateTime),
    phi(old.phi),
    theta(old.theta),
    psi(old.psi),
    a_x(old.a_x),
    a_y(old.a_y),
    a_z(old.a_z),
    p(old.p),
    q(old.q)
    #ifdef PROFILING
    ,predictTimer(old.predictTimer),
    updateTimer(old.updateTimer)
    #endif
    {}
}