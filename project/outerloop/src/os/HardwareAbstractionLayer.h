#ifndef HARDWARE_ABSTRACTION_LAYER_H
#define HARDWARE_ABSTRACTION_LAYER_H
#include <utility>
#include <functional>
#include "State.h"
#include "Frame.h"
#include "MotionRead.h"
#include "Attitude.h"
#include <chrono>
namespace mav{
class EventHandler{
public:
    virtual void onNewFrame(Frame &frame){}
    virtual void onNewSensorData(MotionRead &motionRead){};
    virtual void onModeChange(){};
};



class HardwareAbstractionLayer
{
public:
	HardwareAbstractionLayer()=default;
	virtual ~HardwareAbstractionLayer()=default;

	virtual void newFrame(Frame &frame){

	    if(eventHandler){
#ifdef DEBUG_SCHEDULING
    std::string msg = "New Sensor Data Event";
    print_periodic(msg,1.0);
#endif
            eventHandler->onNewFrame(frame);

        }
	}

    virtual void newSensorData(MotionRead &motionRead){
	    if(eventHandler){
#ifdef DEBUG_SCHEDULING
    std::string msg = "New Sensor Data Event";
    print_periodic(msg,1.0);
#endif
            eventHandler->onNewSensorData(motionRead);
	    }
	}

    virtual void modeChange(){
        if(eventHandler){
    #ifdef DEBUG_SCHEDULING
                std::string msg = "Mode Change Event";
        print_periodic(msg,1.0);
    #endif
            eventHandler->onModeChange();
        }
    }

	virtual void setEventHandler(EventHandler *handler){
		this->eventHandler = handler;
	}


    virtual void sendLowLevelMotion(Attitude &command) = 0 ;
    virtual void sendState(State &state) = 0 ;


    virtual void print(const std::string &msg) = 0;

protected:
    EventHandler *eventHandler{};
    void
    print_periodic(std::string &msg,double period_s){
        auto time = std::chrono::system_clock::now();
        static std::chrono::time_point<std::chrono::system_clock> lastPrint = time;
        std::chrono::duration<double> elapsed = time - lastPrint;
        if(elapsed.count() > period_s){
            std::ostringstream msg_app;
            msg_app << time.time_since_epoch().count() << msg << std::endl ;
            print(msg_app.str());
            lastPrint = time;
        }
    }
};
}
#endif