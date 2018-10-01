/*
MAV::Hardware Abstraction Layer implementation

@author philipp duernay
@email phild@protonmail.com

*/


#define EOL_CSV "\n"

#include <list>
#include <jevois/Debug/Log.H>
#include <jevois/Core/StdioInterface.H>
#include <jevois/Image/RawImageOps.H>
#include <sys/sysinfo.h>
#include <Log.h>
#include <Scheduler.h>

#include "MotionRead.h"
#include "Attitude.h"
#include "Frame.h"
#include "HALComponent/HALComponent.H"
#include "MAVLink/MAVLink.H"
namespace mav {

//! Utility function to get system uptime
uint64_t
get_boot_time_us(void) {
    std::chrono::milliseconds uptime(0u);
    struct sysinfo x;
    if (sysinfo(&x) == 0) {
        uptime = std::chrono::milliseconds(static_cast<unsigned long long>(x.uptime) * 1000ULL);
        return (uint64_t) uptime.count();
    }
    return 0;
}

HALComponent::
HALComponent(std::string const &instance) : Component(instance) {
    LDEBUG("HAL Constructor");
    mavLink = addSubComponent<MAVLink>("MAVLink", MAVLink::Type_t::Hard);
    mavLink->set_instance(mavLink);
    MAVLink::get_instance(MAVLink::Type_t::Hard)->setMsgHandler(this);
}

void
HALComponent::
postInit() {
}

void
HALComponent::
postUninit() {

}

HALComponent::
~HALComponent() {}

void
HALComponent::
sendLowLevelMotion(Attitude &command) {

    portGuard.lock();
    auto m = MAVLink::get_instance(MAVLink::Type_t::Hard);
    if(m){
        mavlink_msg_manual_setpoint_send( m->getChannel(), (uint32_t) get_boot_time_us(),
                                  static_cast<float> (command.roll),
                                  static_cast<float> (command.pitch),
                                  static_cast<float> (command.yaw),
                                  static_cast<float>(command.altitude),
                                  0,
                                  0);

        /*
        mavlink_msg_attitude_send(m->getChannel(), (uint32_t) get_boot_time_us(),
                                  static_cast<float> (command.roll),
                                  static_cast<float> (command.pitch),
                                   static_cast<float> (command.yaw),
                                  0,
                                  0,
                                  0);

        mavlink_msg_altitude_send(m->getChannel(), (uint32_t)get_boot_time_us(),
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  static_cast<float>(command.altitude),
                                  0.0);
*/
    }

    portGuard.unlock();


}

void
HALComponent::
sendState(mav::State &state){
    portGuard.lock();
    auto m = MAVLink::get_instance(MAVLink::Type_t::Hard);
    if(m){
        mavlink_msg_local_position_ned_send(m->getChannel(),(uint32_t) get_boot_time_us(),
                                            static_cast<float> (state.getX()),
                                            static_cast<float> (state.getY()),
                                            static_cast<float> (state.getZ()),
                                            static_cast<float> (state.getBx()),
                                            static_cast<float> (state.getBy()),
                                            static_cast<float> (state.getBz())
                                            );
        /*
        mavlink_msg_highres_imu_send(m->getChannel(),get_boot_time_us(),
                                     imu.xacc,
                                     imu.yacc,
                                     imu.zacc,
                                     imu.xgyro,
                                     imu.ygyro,
                                     imu.zgyro,
                                     imu.xmag,
                                     imu.ymag,
                                     imu.zmag,
                                     imu.abs_pressure,
                                     imu.diff_pressure,
                                     imu.pressure_alt,
                                     imu.temperature,
                                     imu.fields_updated);*/
    }
    portGuard.unlock();
}


void
HALComponent::
print(const std::string &msg) {
    LINFO(msg);
}


void
HALComponent::
newImage(const jevois::RawImage &rawImage) {
    cv::Mat rawImageCV = jevois::rawimage::cvImage(rawImage);//V4L2_PIX_FMT_YUYV

    //cv::Mat rawImageCV = jevois::rawimage::convertToCvBGR(rawImage);
    auto frame = Frame(rawImageCV);
    this->newFrame(frame);
}

void
HALComponent::
mavlink_periodic() {
    portGuard.lock();

    auto m = MAVLink::get_instance(MAVLink::Type_t::Hard);
    if (m) {
        m->receive();
        heartbeat(std::chrono::system_clock::now());
    }else{
        LERROR("No Valid MAVLink Instance");
    }

    portGuard.unlock();


}

void
HALComponent::
heartbeat(std::chrono::time_point<std::chrono::system_clock> time) {
    // Send Heartbeat Every 1 second
    static std::chrono::time_point<std::chrono::system_clock> lastBeat = time;
    std::chrono::duration<double> elapsed = time - lastBeat;
    if (elapsed.count() >= 1.0) {
        auto m = MAVLink::get_instance(MAVLink::Type_t::Hard);
        if(m){
            m->heartbeat();
        }
        LDEBUG("Heartbeat at: " << elapsed.count());
        lastBeat = time;
    }
}

void
HALComponent::
handle_msg(mavlink_message_t *msg) {
    LDEBUG("Handling Message ID: " << msg->msgid);
    auto m= MAVLink::get_instance(MAVLink::Type_t::Hard);
    if(!m){
        return;
    }
    switch (msg->msgid) {

        case MAVLINK_MSG_ID_HIGHRES_IMU: {
            LDEBUG("MSG_ID_HIGHRES_IMU");
           
            mavlink_msg_highres_imu_decode(msg, &(imu));

            imu.xacc = transformAcc(imu.xacc);
            imu.yacc = transformAcc(imu.yacc);
            imu.zacc = transformAcc(imu.zacc);



            MotionRead motionRead(imu.xmag,imu.ymag,imu.zmag,
                                  imu.xacc,imu.yacc,imu.zacc,
                                  imu.xgyro,imu.ygyro,imu.zgyro,
                                  imu.abs_pressure);

            if(motionRead.hasNaN()){
                LERROR("Nan received!");
            }else{
                this->newSensorData(motionRead);
                guard.lock();
                ahrs = motionRead;
                guard.unlock();
                if(this->optiTrackLog){
                    std::ostringstream logLine;
                    /*
                    logLine << Scheduler::getStartupTimeMillis() << ","
                            << imu.abs_pressure << ","
                            << imu.diff_pressure << ","
                            << imu.pressure_alt << ","
                            << imu.temperature;
                            */
                    logLine << "Hi Shuo" << EOL_CSV;
                    std::thread([this,&logLine](){

                    this->optiTrackLog->append(logLine.str());

                    }).detach();
                }
            }


        }
            break;

        case MAVLINK_MSG_ID_SET_MODE:
        {
            LDEBUG("MODE MSG");
            static uint8_t currentMode = 0;
            mavlink_set_mode_t mode;
            mavlink_msg_set_mode_decode(msg,&(mode));
            if(mode.base_mode != currentMode){
                print("Mode changed");
                this->modeChange();
                currentMode = mode.base_mode;
            }

        }
            break;

        case MAVLINK_MSG_ID_PING:{
            print("Ping received");
        }
            break;


        case MAVLINK_MSG_ID_COMMAND_LONG:{
            mavlink_command_long_t my_params;
            mavlink_msg_command_long_decode(msg,&(my_params));
            float k_p = my_params.param1;
            float k_d = my_params.param2;
            pControl->setControllerKp(k_p);
            pControl->setControllerKd(k_d);
            float my_param_3 = my_params.param3;
            float my_param_4 = my_params.param4;
            float my_param_5 = my_params.param5;
            float my_param_6 = my_params.param6;
            float my_param_7 = my_params.param7;
        }
            break;

        default: {
            LINFO("No Handler for this message:" << msg->msgid);
        }
            break;
    }

}

void
HALComponent::
startMavlinkTask(){
    if(!running){
        running = true;
        mavlinkThread = std::thread(&HALComponent::mavlinkTask,this);
#ifdef DEBUG_SCHEDULING
        std::ostringstream msg;
        msg << std::chrono::system_clock::now().time_since_epoch().count() <<
            "Starting mavlink task.." << std::endl ;
        print(msg.str());
#endif
    }
}

void
HALComponent::
stopMavlinkTask(){
    if(running){
        running =false;
        mavlinkThread.join();
#ifdef DEBUG_SCHEDULING
        std::ostringstream msg;
        msg << std::chrono::system_clock::now().time_since_epoch().count() <<
            "Stopping mavlink task.." << std::endl ;
        print(msg.str());
#endif
    }
}

void
HALComponent::
mavlinkTask(){
    while(running){
        mavlink_periodic();
        std::this_thread::sleep_for(std::chrono::duration<double,std::milli>(T_MAVLINK_READ_MS));
#ifdef DEBUG_SCHEDULING
        std::string msg = "Mavlink Task";
        print_periodic(msg,1.0);
#endif
    }
}

float
HALComponent::
transformAcc(float acc){
    return acc/ACC_CONV_F*GRAVITY;
}




}