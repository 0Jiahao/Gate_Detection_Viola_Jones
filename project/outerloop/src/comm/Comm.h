//Communication Module
//Takes care of communicating with betaflight/ground station
#ifndef COMM_H
#define COMM_H
#include "Attitude.h"
#include "HardwareAbstractionLayer.h"
#include <iostream>
#include <memory>
#include "State.h"
#include <chrono>
#include <string>
namespace mav{

class Comm
{
private:
    static HardwareAbstractionLayer *hal;

public:
    
	static void print(const std::string &msg);
	static void setHal(HardwareAbstractionLayer* hal);
	~Comm() = default;
	void sendLowLevel(Attitude &cmd);
	void sendState(State &state);
    static void print_periodic(const std::string &msg,double period_s);
};

}

#endif