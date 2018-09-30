#include "opencv2/core.hpp"
#include <opencv2/imgproc.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include <vector>
#include "VJGateDetector.h"

using namespace cv;
using namespace std;

VJGate::VJGate(Point _tl, Point _tr, Point _br, Point _bl)
{
    this->tl = _tl;
    this->tr = _tr;
    this->br = _br;
    this->bl = _bl; 
};

Point Rect_center(Rect rect)
{
    return (0.5 * rect.tl() + 0.5 * rect.br());
}

void draw_corners(Mat& img, vector<Rect> corners, Scalar color)
{
    for(int i = 0; i < corners.size(); i++)
        {
            circle(img, Rect_center(corners[i]), 1, color, 5);
        }
}

void draw_gates(Mat& img, vector<VJGate> gates)
{
    for(int i = 0; i < gates.size(); i++)
        {
            line(img,gates[i].tl,gates[i].tr,Scalar(203,192,255),5);
            line(img,gates[i].tr,gates[i].br,Scalar(203,192,255),5);
            line(img,gates[i].br,gates[i].bl,Scalar(203,192,255),5);
            line(img,gates[i].bl,gates[i].tl,Scalar(203,192,255),5);
        }
}

vector<VJGate> extract_gates(vector<Rect> tl_corners, vector<Rect> tr_corners, vector<Rect> br_corners, vector<Rect> bl_corners)
{
    vector<VJGate> gates;
        for(int tl = 0; tl < tl_corners.size(); tl++)
        {
            // connect tl -> tr
            for(int tr = 0; tr < tr_corners.size(); tr++)
            {
                // check connection tl -> tr
                if((tl_corners[tl].br().x < tr_corners[tr].tl().x) && \
                   (tl_corners[tl].br().y > tr_corners[tr].tl().y) && \
                   (tl_corners[tl].tl().y < tr_corners[tl].br().y))
                {
                    // connect tr -> br
                    for(int br = 0; br < br_corners.size(); br++)
                    {
                        // check connection tr -> br
                        if((tr_corners[tr].br().y < br_corners[br].tl().y) && \
                           (tr_corners[tr].tl().x < br_corners[br].br().x) && \
                           (tr_corners[tr].br().x > br_corners[tl].tl().x))
                        {
                            // connect br -> bl
                            for(int bl = 0; bl < bl_corners.size(); bl++)
                            {
                                // check connection br -> bl -> tl
                                if((br_corners[br].tl().x > bl_corners[bl].br().x) && \
                                   (br_corners[br].tl().y < bl_corners[bl].br().y) && \
                                   (br_corners[br].br().y > bl_corners[bl].tl().y) && \
                                   (tl_corners[tl].br().y < bl_corners[bl].tl().y) && \
                                   (tl_corners[tl].tl().x < bl_corners[bl].br().x) && \
                                   (tl_corners[tl].br().x > bl_corners[bl].tl().x))
                                {
                                    VJGate detected_gate(Rect_center(tl_corners[tl]), \
                                                         Rect_center(tr_corners[tr]), \
                                                         Rect_center(br_corners[br]), \
                                                         Rect_center(bl_corners[bl]));
                                    gates.push_back(detected_gate);
                                }
                            }
                        }
                    }
                }
            }
        }
    return gates;
}

void VJDetector(Mat& img, CascadeClassifier tl_light, CascadeClassifier tr_light, CascadeClassifier bl_light, CascadeClassifier br_light)
{
    Mat gray;
    cv::cvtColor(img, gray, CV_BGR2GRAY);
    // extract corners
    vector<Rect> tl_corners;
    vector<Rect> tr_corners;
    vector<Rect> bl_corners;
    vector<Rect> br_corners;
    tl_light.detectMultiScale( gray, tl_corners, 2, 3, 0, Size(32, 32),Size(32,32));
    tr_light.detectMultiScale( gray, tr_corners, 2, 3, 0, Size(32, 32),Size(32,32));
    bl_light.detectMultiScale( gray, bl_corners, 2, 3, 0, Size(32, 32),Size(32,32));
    br_light.detectMultiScale( gray, br_corners, 2, 3, 0, Size(32, 32),Size(32,32));
    // extract rectangles based on corners
    vector<VJGate> gates;
    gates = extract_gates(tl_corners, tr_corners, br_corners, bl_corners);
    // draw corners tl->red tr->yellow br->blue bl->green
    draw_corners(img, tl_corners, Scalar(  0,  0,255));
    draw_corners(img, tr_corners, Scalar(  0,255,255));
    draw_corners(img, bl_corners, Scalar(255,  0,  0));
    draw_corners(img, br_corners, Scalar(  0,255,  0));
    // draw detections
    draw_gates(img, gates);
 }