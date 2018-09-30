#include "opencv2/core.hpp"
#include <vector>

using namespace cv;

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