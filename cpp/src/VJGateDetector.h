#ifndef VJDETECTOR_H
#define VJDETECTOR_H
#include "opencv2/core.hpp"
#include <opencv2/imgproc.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <time.h>
#include <atomic>
#include <Comm.h>
#include <mutex>
#include "Config.h"
#include "Detector.h"
#include "MavTypes.h"


namespace mav{

class VJGateDetector : public Detector
{
private:
    cv::CascadeClassifier tl_light;
    cv::CascadeClassifier tr_light;
    cv::CascadeClassifier br_light;
    cv::CascadeClassifier bl_light;
    std::vector<Polygon> gates;
    Polygon best_gate;
    bool newGate;
    SensorData AHRS;
public:
    VJGateDetector();
    void detect(Frame &frame) override;
    std::vector<Polygon> extract_gates(std::vector<cv::Rect> tl_corners, std::vector<cv::Rect> tr_corners, std::vector<cv::Rect> br_corners, std::vector<cv::Rect> bl_corners);
	std::vector<Polygon> extract_gates_3(std::vector<cv::Rect> tl_corners, std::vector<cv::Rect> tr_corners, std::vector<cv::Rect> br_corners, std::vector<cv::Rect> bl_corners, float ratio, float angle_c, float dist_c);
	Polygon getBestGate();
    bool hasNewGate() override {return newGate;};
	void setNewGate(){newGate = true;};
	void clearNewGate() override {newGate = false;};
    void updateAhrs(SensorData &ahrs);
    int y_max=Y_MAX,y_min=Y_MIN,cr_max=Cr_MAX,cr_min=Cr_MIN,cb_max=Cb_MAX,cb_min=Cb_MIN;
    std::vector<Polygon> getDetections() override;
};

}
#endif