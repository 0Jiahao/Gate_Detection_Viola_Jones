#include "Comm.h"
namespace mav{
HardwareAbstractionLayer* Comm::hal = nullptr;



void Comm::sendLowLevel(Attitude &cmd){
    if(hal){
        hal->sendLowLevelMotion(cmd);
    }

}

void Comm::print(const std::string &msg) {
    if(hal){
        hal->print(msg);
    }
}

void Comm::setHal(HardwareAbstractionLayer* hal) {
    Comm::hal = hal;
}

void Comm::sendState(State &state) {
    if(hal){
        hal->sendState(state);
    }
}

void Comm::print_periodic(const std::string &msg, double period_s) {
    {
        auto time = std::chrono::system_clock::now();
        static std::chrono::time_point<std::chrono::system_clock> lastPrint = time;
        std::chrono::duration<double> elapsed = time - lastPrint;
        if(elapsed.count() > period_s){
            std::ostringstream msg_app;
            msg_app << time.time_since_epoch().count() << ": " << msg << std::endl ;
            Comm::print(msg_app.str());
            lastPrint = time;
        }
    };
}


}