//
// Created by phil on 06/06/2018.
//

#include <opencv2/imgproc.hpp>
#include <Image.h>
#include "Simulation.h"


void Simulation::airsimUpdate(){
    mav::Kinematics kin = model->getTrueKinematics();
    airsim->setPose(kin.x,kin.y,kin.z,kin.phi,kin.theta,kin.psi);

    Image image = airsim->getImage();
    cv::Mat mat;
    if(!image.empty()){
        mat = cv::Mat(image.getHeight(), image.getWidth(), CV_8UC4);
        memcpy(mat.data, image.getData().data(), image.getData().size() * sizeof(uint8_t));
    }else{
        mat = cv::Mat(10, 10, CV_8UC4);

    }
    cv::Mat mat_bgr;
    cv::cvtColor(mat, mat_bgr, cv::COLOR_RGB2BGR);
    auto frame = Frame(mat_bgr);
    this->newFrame(frame);
}

long long Simulation::getCurrentTimeMillis()
{
    using namespace std::chrono;
    milliseconds ms = duration_cast< milliseconds >(
            system_clock::now().time_since_epoch()
    );
    return ms.count();
}


void Simulation::modelUpdate(){
//    std::cout << "Model Update" << std::endl;
    controllerGuard.lock();
    mav::Attitude cmd = autopilot->controllerRun(model->getTrueKinematics());
    controllerGuard.unlock();

    auto sensorReadings = model->getSensorReadings();
    this->newSensorData(sensorReadings);
    model->update(static_cast<float>(cmd.roll), static_cast<float>(cmd.pitch), static_cast<float>(cmd.yaw), static_cast<float>(cmd.altitude));
}

Simulation::Simulation(Simulation &old) {
    this->model = model;
    this->airsim = old.airsim;
}

Simulation::Simulation(mav::DynamicModel *model, AirSimInterface *airsim, SampleController *autopilot) {
    this->model = model;
    this->airsim = airsim;
    this->autopilot = autopilot;
}

void Simulation::sendLowLevelMotion(Attitude command) {
    controllerGuard.lock();
    autopilot->setAttitude(static_cast<float>(command.roll), static_cast<float>(command.pitch),
                            static_cast<float>(command.yaw), static_cast<float>(command.altitude));
    controllerGuard.unlock();
}

void Simulation::print(const std::string &msg) {
    std::cout << msg << std::endl;
}

void Simulation::setRefPosition(float x, float y, float z, float heading) {
    controllerGuard.lock();
    autopilot->setReference(x,y,z,heading);
    controllerGuard.unlock();

}

void Simulation::sendState(State &state) {
    //don't need to do anything here
}
