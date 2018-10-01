//Scheduler
//Control which tasks run when
#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <chrono>
#include <thread>

#include "Comm.h"
#include "Control.h"
#include "Filter.h"
#include "Vision.h"
#include "HardwareAbstractionLayer.h"
#include "Log.h"


namespace mav{
	static long long getCurrentTimeMillis(){
		using namespace std::chrono;
		milliseconds ms = duration_cast< milliseconds >(
				system_clock::now().time_since_epoch()
		);
		return ms.count();
	}

class Scheduler:public EventHandler
{

public:
	std::atomic<bool> logValues,logFrames;

	/*
	Returns milliseconds since system startup.
	*/
	static long long getStartupTimeMillis();
	~Scheduler() = default;
	Scheduler(Vision &vision,Control &control, Filter &filter,Comm &comm, Log &log);
    Scheduler(const Scheduler &old);

    /**
     * start the periodic tasks in a new thread
     */
    void startPeriodicTask();

    /**
     * stop the periodic tasks thread
     */
    void stopPeriodicTask();

    /**
     * Tasks
     */

    /**
    * Tasks that are executed periodically
    */
    void periodicTasks();

    /**
     * Executed when a new frame is available
     * @param frame the new image
     */
	void onNewFrame(Frame &frame) override;

	/**
	 * Executed when new sensor data is available
	 * @param motionRead the new sensor data
	 */
	void onNewSensorData(MotionRead &motionRead) override ;

    /**
     * Executed when control model has changed between autopilot/manual control
     */
    void onModeChange() override ;


private:
    Vision vision;
    Control control;
    Comm comm;
    Filter filter;
    Log log;
    static std::atomic<long long> t_ms;
    std::thread periodicThread;
    std::atomic<bool> running;
    const double t_periodic;
    const long long t_send_cmd_send_ms,t_send_state_ms,t_log_ms,t_log_frame_ms;

    /**
     * Loop for periodic task
     */
    void runPeriodic();
};
}
#endif