#include "Scheduler.h"
#include "MotionRead.h"
#include "Attitude.h"
#include "State.h"
#include "Pose.h"
#include "Profiler.h"
#include <chrono>
#include <future>
#include <opencv2/imgproc.hpp>
#include "filter.h"
#include "control.h"

namespace mav{
std::atomic<long long> Scheduler::t_ms(0);


Scheduler::
Scheduler(const Scheduler &old):
vision(old.vision),
filter(old.filter),
control(old.control),
comm(old.comm),
log(old.log),
t_periodic(1000/F_PERIODIC_HZ),
t_send_cmd_send_ms(1000/F_SEND_CMD_HZ),
t_send_state_ms(1000/F_SEND_STATE_HZ),
t_log_ms(1000/F_LOG_HZ),
t_log_frame_ms(1000/F_LOG_FRAME_HZ),
logFrames(LOG_FRAMES),
logValues(LOG_VALUES)
{
}


long long
Scheduler::
getStartupTimeMillis()
{
	return t_ms;
}

Scheduler::
Scheduler(Vision &vision,Control &control, Filter &filter,Comm &comm, Log &log):
        logFrames(LOG_FRAMES),
        logValues(LOG_VALUES),
        vision(vision),
        comm(comm),
        filter(filter),
        control(control),
        log(log),
        t_periodic(1000/F_PERIODIC_HZ),
        t_send_cmd_send_ms(1000/F_SEND_CMD_HZ),
        t_send_state_ms(1000/F_SEND_STATE_HZ),
        t_log_ms(1000/F_LOG_HZ),
        t_log_frame_ms(static_cast<const long long int>(1000.0 / F_LOG_FRAME_HZ))
{

    State initialGuess( -16.0,0, -4.5,0 ,0, 0, 0);

    std::ostringstream msg;
    msg << "Initializing filter with\n" << initialGuess.toString() << std::endl ;
    Comm::print(msg.str());

    filter.setFlagRunEKF(initialGuess);

	running = false;

}

void
Scheduler::
onNewFrame(Frame &frame) {
    static long long last_log = 0;
    vision.updateFrame(frame);
    vision.gateDetection();
    if(logFrames &&
       t_ms-last_log > t_log_frame_ms){
        last_log = t_ms;
        log.append(frame);
    }

}

void
Scheduler::
periodicTasks() {
    t_ms += t_periodic;

	if (vision.hasNewGate()) {
        vision.poseEstimation();
        auto pose = vision.getPose();
        filter.updateState(pose);
		vision.clearNewGate();
	}

    filter.predictStates();
	auto state = filter.getState();
    control.updateState(state);
    control.loop(&filter);

    /* periodic cmd sending */
	if(t_ms % t_send_cmd_send_ms == 0){
	    auto cmd = control.getSetPoints();
        comm.sendLowLevel(cmd);
	}
    /* periodic position for ground station */
    if(t_ms % t_send_state_ms == 0){
        comm.sendState(state);
    }

    /* Periodic logging we run this in a new thread that stops after completion
     * to avoid that the control/filter loop gets stuck when writing
     * to the sd card */
    if(logValues && t_ms % t_log_ms == 0){
        auto ahrs = filter.getSensorReadings();
        auto cmd = control.getSetPoints();
        auto pose = vision.getPose();
        auto bestGate = vision.getBestGate();
        std::thread([this,&ahrs,&cmd,&pose,&bestGate,&state](){
            this->log.append(ahrs, cmd, state, pose, bestGate,control);
        }).detach();
    }

#ifdef PROFILING
    if(t_ms % 1000 == 0) {
        Profiler::print();
    }
#endif


}

void
Scheduler::
onNewSensorData(MotionRead &motionRead) {

    vision.updateAHRS(motionRead);
    filter.updateSensorReadings(motionRead);
    control.update(motionRead);


}

void
Scheduler::
runPeriodic() {
    while(running){
        periodicTasks();
        std::this_thread::sleep_for(std::chrono::duration<double,std::milli>(t_periodic));
#ifdef DEBUG_SCHEDULING
        std::string msg = "Periodic tasks active";
        Comm::print_periodic(msg,1.0);
#endif
    }
}

void
Scheduler::
startPeriodicTask() {
    if(!running){
        running = true;
        periodicThread = std::thread(&Scheduler::runPeriodic,this);
#ifdef DEBUG_SCHEDULING
        std::ostringstream msg;
        msg << "Starting periodic task thread" << std::endl ;
        Comm::print(msg.str());
#endif
    }
}

void
Scheduler::
stopPeriodicTask() {
    if(running){
        running = false;
        periodicThread.join();
#ifdef DEBUG_SCHEDULING
        std::ostringstream msg;
        msg << "Periodic Task thread stopped" << std::endl ;
        Comm::print(msg.str());
#endif
    }

}

void
Scheduler::
onModeChange()
{
    State initialGuess( -5.0,0, -1.5,0 ,0, 0, 0);

    /*
    std::ostringstream msg;
    msg << "Initializing control and filter with\n" << initialGuess.toString() << std::endl ;
    Comm::print(msg.str());
     */

    filter.setFlagRunEKF(initialGuess);
    control.initializeControl();
    control.setTimeStamp();
    filter_reset();
    control_reset();
}

}