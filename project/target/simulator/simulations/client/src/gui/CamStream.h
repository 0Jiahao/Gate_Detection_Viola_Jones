//
// Created by phil on 03/06/2018.
//

#ifndef CAMSTREAM_H
#define CAMSTREAM_H


#include <sim/Simulation.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

class CamStream {
public:
    CamStream(Simulation *sim);
    CamStream();
    CamStream(CamStream const &that);

    void start();
    virtual void draw(std::string winname);

    void stop();
    bool isRunning();
protected:
    cv::Mat getImage();
    virtual void annotate(cv::Mat &mat);
    double round3(float value);
    std::string valueStr(std::string title,std::vector<float> values);
    std::string valueStr(std::vector<float> values);
    void annotateValue(cv::Mat &mat,cv::Point location, std::string text);
    std::atomic<bool> running;
    Simulation *sim;
};


#endif //CAMSTREAM_H
