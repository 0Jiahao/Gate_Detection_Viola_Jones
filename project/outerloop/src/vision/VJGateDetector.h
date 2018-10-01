#include "opencv2/core.hpp"
#include "Frame.h"
#include "Polygon.h"
#include <vector>
#include <time.h>
#include <atomic>
#include <Comm.h>
#include <mutex>
#include "Config.h"

using namespace cv;
using namespace std;

namespace mav{

class VJGateDetector
{
private:
    CascadeClassifier tl_light;
    CascadeClassifier tr_light;
    CascadeClassifier br_light;
    CascadeClassifier bl_light;
    vector<Polygon> gates;
    Polygon best_gate;
    std::atomic<bool> newGate;
    int y_max=Y_MAX,y_min=Y_MIN,cr_max=Cr_MAX,cr_min=Cr_MIN,cb_max=Cb_MAX,cb_min=Cb_MIN;
    int principal_cx=CAMERA_C_X, principal_cy=CAMERA_C_Y;
    float undistortion_k=CAMERA__DISTORTION_K, focal_length = CAMERA_F;
    Polygon undistortion(Polygon &gate);
    std::mutex guard;
public:
    VJGateDetector();
    void VJGateDetection(Mat& img);
    vector<Polygon> extract_gates(vector<Rect> tl_corners, vector<Rect> tr_corners, vector<Rect> br_corners, vector<Rect> bl_corners);
	
	Polygon getBestGate();
    bool hasNewGate(){return newGate;};
	void setNewGate(){newGate = true;};
	void clearNewGate(){newGate = false;};

    void setYMax(int y_max);
    void setYMin(int y_min);
    void setCrMax(int cr_max);
    void setCrMin(int cr_min);
    void setCbMax(int cb_max);
    void setCbMin(int cb_min);
    void setUndistortionK(float k);
    void setFocalLength(float f);
    void setCx(int cx);
    void setCy(int cy);
};

class VJGate
{
public:
    Point tl;
    Point tr;
    Point br;
    Point bl;
    VJGate(Point _tl, Point _tr, Point _br, Point _bl);
};

void VJDetector(Mat& img, CascadeClassifier tl_light, CascadeClassifier tr_light, CascadeClassifier bl_light, CascadeClassifier br_light);

}