//Takes state estimate and produced motion command

#ifndef CONTROL_H
#define CONTROL_H

//#include <Scheduler.h>
#include "State.h"
#include "Attitude.h"
#include "Controller.h"

#include "Profiler.h"

#include "Filter.h"


namespace mav{

    enum ControlState{
        Idle = 0,
        GoThroughGate,
        Arc,
    };

enum HIGH_LEVEL_GUIDANCE_STATE {FIRST_PART,SECOND_PART,THIRD_PART};
enum LOW_LEVEL_GUIDANCE_STATE {TAKE_OFF,GO_THROUGH_GATE,LAND};

class Control
{
private:
    Controller *controller = new Controller();
    ControlState controlState = GoThroughGate;
    enum HIGH_LEVEL_GUIDANCE_STATE high_level_guidance_state;
    enum LOW_LEVEL_GUIDANCE_STATE low_level_guidance_state;
    bool tempModeChange; // todo delete
    long long timeStampAutonomous;
	long long lastTimeStamp;




#ifdef PROFILING
    int loopTimer;
#endif


    void firstPartLogic(Filter * filter);
    void secondPartLogic(Filter * filter);
   int gateNumber;

public:
	Control();
	~Control() = default;

	void setHighLevelGuidanceState(enum HIGH_LEVEL_GUIDANCE_STATE highLevelState){this->high_level_guidance_state = highLevelState;};
    enum HIGH_LEVEL_GUIDANCE_STATE getHighLevelGuidanceState(){return high_level_guidance_state;};
    void setLowLevelGuidanceState(enum LOW_LEVEL_GUIDANCE_STATE lowLevelState){this->low_level_guidance_state = lowLevelState;};
    enum LOW_LEVEL_GUIDANCE_STATE getLowLevelGuidanceState(){return low_level_guidance_state;};


	void updateState(State &state);
	void update(MotionRead &ahrs);
	void loop(Filter* filter);
	void initializeControl();
	Attitude getSetPoints();
	void clearGateNumber(){gateNumber = 0;};
        int getGateNumber(){return gateNumber;};
	float getArcTargetHeading(){return this->controller->getArcTargetHeading();};
		void setControllerKp(float k_p){controller->setKp(k_p);};
		void setControllerKd(float k_d){controller->setKd(k_d);};

		void setTimeStamp();
};

}
#endif
