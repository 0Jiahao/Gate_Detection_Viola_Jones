//
// Created by phil on 04/06/2018.
//

#ifndef DRONECAMSTREAM_H
#define DRONECAMSTREAM_H


#include <gui/CamStream.h>
#include <Logger.h>
#include <sim/Simulation.h>

class DroneCamStream : public CamStream{
public:
    DroneCamStream(Simulation *sim, mav::Logger *logger);
    void draw(std::string winname) override;
protected:
    void annotate(cv::Mat &mat) override;
    mav::Logger *logger;
};


#endif //OUTERLOOP_SIM_DRONECAMSTREAM_H
