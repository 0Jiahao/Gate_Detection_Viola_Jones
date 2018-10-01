#include "Pose.h"
#include "HardwareAbstractionLayer.h"
#include "Vision.h"
#include <iostream>
#include <Comm.h>
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "Frame.h"
using namespace mav;

#define RESOURCE_PATH "/home/phil/code/dronerace2018/target/test/vision/resource/4631.jpg"

int main()
{
    cv::Mat mat = cv::imread(RESOURCE_PATH);
    Frame frame(mat);
    Vision *vision = new Vision();
    //for (int i=0; i< 1; i++){
        vision->updateFrame(frame);
        vision->gateDetection();
        frame.drawPolygon(vision->getBestGate(),cv::Scalar(0,255,0));
        //vision->writeMessage();
        cv::imshow( "Line",frame.getMat());
        cv::waitKey(0);
    //}
    return 0;
}
