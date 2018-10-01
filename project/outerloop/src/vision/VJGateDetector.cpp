#include "opencv2/core.hpp"
#include <opencv2/imgproc.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "stdio.h"
#include "Frame.h"
#include <iostream>
#include <Comm.h>
#include <time.h>
#include <Scheduler.h>
#include "Config.h"
#include <cmath>
#include "VJGateDetector.h"

using namespace cv;
using namespace std;

namespace mav{

VJGateDetector::VJGateDetector()
{
    this->tl_light.load("/jevois/modules/MavLab/CascadeDetector/model/0.8_30_t_l_corner_mad.xml");
    this->tr_light.load("/jevois/modules/MavLab/CascadeDetector/model/0.8_30_t_r_corner_mad.xml");
    this->br_light.load("/jevois/modules/MavLab/CascadeDetector/model/0.8_30_b_r_corner_mad.xml");
    this->bl_light.load("/jevois/modules/MavLab/CascadeDetector/model/0.8_30_b_l_corner_mad.xml");
};


// VJGate::VJGate(Point _tl, Point _tr, Point _br, Point _bl)
// {
//     this->tl = _tl;
//     this->tr = _tr;
//     this->br = _br;
//     this->bl = _bl; 
// };

Point_t Rect_center(Rect rect)
{
    Point cvcenter = 0.5 * rect.tl() + 0.5 * rect.br();
    Point_t center = {cvcenter.y, cvcenter.x};
    return center;
}

void draw_corners(Mat& img, vector<Rect> corners, Scalar color)
{
    for(int i = 0; i < corners.size(); i++)
        {
            circle(img, 0.5 * corners[i].tl() + 0.5 * corners[i].br(), 1, color, 5);
        }
}

vector<Polygon> VJGateDetector::extract_gates(vector<Rect> tl_corners, vector<Rect> tr_corners, vector<Rect> br_corners, vector<Rect> bl_corners)
{
    vector<Polygon> gates;
        // tl -> tr -> br
        for(int tl = 0; tl < tl_corners.size(); tl++)
        {
            // connect tl -> tr
            for(int tr = 0; tr < tr_corners.size(); tr++)
            {
                // check connection tl -> tr
                if((tl_corners[tl].br().x < tr_corners[tr].tl().x) && \
                   (tl_corners[tl].br().y > tr_corners[tr].tl().y) && \
                   (tl_corners[tl].tl().y < tr_corners[tr].br().y))
                {
                    // connect tr -> br
                    for(int br = 0; br < br_corners.size(); br++)
                    {
                        // check connection tr -> br
                        if((tr_corners[tr].br().y < br_corners[br].tl().y) && \
                           (tr_corners[tr].tl().x < br_corners[br].br().x) && \
                           (tr_corners[tr].br().x > br_corners[br].tl().x))
                        {
                            Point_t p_1 = Rect_center(tl_corners[tl]);
                            Point_t p_2 = Rect_center(tr_corners[tr]);
                            Point_t p_3 = Rect_center(br_corners[br]);
                            float Len_1 = sqrtf(powf(p_1.col-p_2.col,2) + powf(p_1.row-p_2.row,2));
                            float Len_2 = sqrtf(powf(p_2.col-p_3.col,2) + powf(p_2.row-p_3.row,2));
                            if((Len_1 / Len_2 < 1.5) && (Len_2 / Len_1 < 1.5))
                            {
                                Rect Virtual_corner(tl_corners[tl].tl() + \
                                                    br_corners[br].tl() - \
                                                    tr_corners[tr].tl() ,
                                                    tr_corners[tr].size());
                                Polygon detected_gate = {Rect_center(tl_corners[tl]), \
                                                            Rect_center(tr_corners[tr]), \
                                                            Rect_center(Virtual_corner), \
                                                            Rect_center(br_corners[br])};
                                detected_gate = this->undistortion(detected_gate);
                                gates.push_back(detected_gate);
                            }
                        }
                    }
                }
            }
        }
        // tr -> br -> bl
        for(int tr = 0; tr < tr_corners.size(); tr++)
        {
            // connect tr -> br
            for(int br = 0; br < br_corners.size(); br++)
            {
                // check connection tr -> br
               if((tr_corners[tr].br().y < br_corners[br].tl().y) && \
                  (tr_corners[tr].tl().x < br_corners[br].br().x) && \
                  (tr_corners[tr].br().x > br_corners[br].tl().x))
                {
                    // connect br -> bl
                    for(int bl = 0; bl < bl_corners.size(); bl++)
                    {
                        // check connection br -> bl
                        if((br_corners[br].tl().x > bl_corners[bl].br().x) && \
                           (br_corners[br].tl().y < bl_corners[bl].br().y) && \
                           (br_corners[br].br().y > bl_corners[bl].tl().y))
                        {
                            Point_t p_1 = Rect_center(tr_corners[tr]);
                            Point_t p_2 = Rect_center(br_corners[br]);
                            Point_t p_3 = Rect_center(bl_corners[bl]);
                            float Len_1 = sqrtf(powf(p_1.col-p_2.col,2) + powf(p_1.row-p_2.row,2));
                            float Len_2 = sqrtf(powf(p_2.col-p_3.col,2) + powf(p_2.row-p_3.row,2));
                            if((Len_1 / Len_2 < 1.5) && (Len_2 / Len_1 < 1.5))
                            {
                                Rect Virtual_corner(tr_corners[tr].tl() + \
                                                    bl_corners[bl].tl() - \
                                                    br_corners[br].tl() ,
                                                    br_corners[br].size());
                                Polygon detected_gate = {Rect_center(Virtual_corner), \
                                                            Rect_center(tr_corners[tr]), \
                                                            Rect_center(bl_corners[bl]), \
                                                            Rect_center(br_corners[br])};
                                detected_gate = this->undistortion(detected_gate);
                                gates.push_back(detected_gate);
                            }
                        }
                    }
                }
            }
        }
        // br -> bl -> tl
        for(int br = 0; br < br_corners.size(); br++)
        {
            // connect br -> bl
            for(int bl = 0; bl < bl_corners.size(); bl++)
            {
                // check connection tr -> br
               if((br_corners[br].tl().x > bl_corners[bl].br().x) && \
                  (br_corners[br].tl().y < bl_corners[bl].br().y) && \
                  (br_corners[br].br().y > bl_corners[bl].tl().y))
                {
                    // connect bl -> tl
                    for(int tl = 0; tl < tl_corners.size(); tl++)
                    {
                        // check connection bl -> tl
                        if((tl_corners[tl].br().y < bl_corners[bl].tl().y) && \
                           (tl_corners[tl].tl().x < bl_corners[bl].br().x) && \
                           (tl_corners[tl].br().x > bl_corners[bl].tl().x))
                        {
                            Point_t p_1 = Rect_center(br_corners[br]);
                            Point_t p_2 = Rect_center(bl_corners[bl]);
                            Point_t p_3 = Rect_center(tl_corners[tl]);
                            float Len_1 = sqrtf(powf(p_1.col-p_2.col,2) + powf(p_1.row-p_2.row,2));
                            float Len_2 = sqrtf(powf(p_2.col-p_3.col,2) + powf(p_2.row-p_3.row,2));
                            if((Len_1 / Len_2 < 1.5) && (Len_2 / Len_1 < 1.5))
                            {
                                Rect Virtual_corner(br_corners[br].tl() + \
                                                    tl_corners[tl].tl() - \
                                                    bl_corners[bl].tl() ,
                                                    bl_corners[bl].size());
                                Polygon detected_gate = {Rect_center(tl_corners[tl]), \
                                                            Rect_center(Virtual_corner), \
                                                            Rect_center(bl_corners[bl]), \
                                                            Rect_center(br_corners[br])};
                                detected_gate = this->undistortion(detected_gate);
                                gates.push_back(detected_gate);
                            }
                        }
                    }
                }
            }
        }
        // bl -> tl -> tr
        for(int bl = 0; bl < bl_corners.size(); bl++)
        {
            // connect bl -> tl
            for(int tl = 0; tl < tl_corners.size(); tl++)
            {
                // check connection bl -> tl
               if((tl_corners[tl].br().y < bl_corners[bl].tl().y) && \
                  (tl_corners[tl].tl().x < bl_corners[bl].br().x) && \
                  (tl_corners[tl].br().x > bl_corners[bl].tl().x))
                {
                    // connect tl -> tr
                    for(int tr = 0; tr < tr_corners.size(); tr++)
                    {
                        // check connection bl -> tl
                        if((tl_corners[tl].br().x < tr_corners[tr].tl().x) && \
                           (tl_corners[tl].br().y > tr_corners[tr].tl().y) && \
                           (tl_corners[tl].tl().y < tr_corners[tr].br().y))
                        {
                            Point_t p_1 = Rect_center(bl_corners[bl]);
                            Point_t p_2 = Rect_center(tl_corners[tl]);
                            Point_t p_3 = Rect_center(tr_corners[tr]);
                            float Len_1 = sqrtf(powf(p_1.col-p_2.col,2) + powf(p_1.row-p_2.row,2));
                            float Len_2 = sqrtf(powf(p_2.col-p_3.col,2) + powf(p_2.row-p_3.row,2));
                            if((Len_1 / Len_2 < 1.5) && (Len_2 / Len_1 < 1.5))
                            {
                                Rect Virtual_corner(bl_corners[bl].tl() + \
                                                    tr_corners[tr].tl() - \
                                                    tl_corners[tl].tl() ,
                                                    tl_corners[tl].size());
                                Polygon detected_gate = {Rect_center(tl_corners[tl]), \
                                                        Rect_center(tr_corners[tr]), \
                                                        Rect_center(bl_corners[bl]), \
                                                        Rect_center(Virtual_corner)};
                                detected_gate = this->undistortion(detected_gate);
                                gates.push_back(detected_gate);
                            }
                        }
                    }
                }
            }
        }
    return gates;
}

void VJGateDetector::VJGateDetection(Mat& img)
{
    Mat gray;
    cv::cvtColor(img, gray, CV_BGR2GRAY);
    // extract corners
    vector<Rect> tl_corners;
    vector<Rect> tr_corners;
    vector<Rect> bl_corners;
    vector<Rect> br_corners;
    this->tl_light.detectMultiScale( gray, tl_corners, 1.1, 1, 0, Size(32, 32),Size(32,32));
    this->tr_light.detectMultiScale( gray, tr_corners, 1.1, 1, 0, Size(32, 32),Size(32,32));
    this->bl_light.detectMultiScale( gray, bl_corners, 1.1, 1, 0, Size(32, 32),Size(32,32));
    this->br_light.detectMultiScale( gray, br_corners, 1.1, 1, 0, Size(32, 32),Size(32,32));
    // extract rectangles based on corners
    this->gates = extract_gates(tl_corners, tr_corners, br_corners, bl_corners);
    if(this->gates.size() > 0)
    {
        setNewGate();
    }
    Polygon max_gate;
    float max_area = 0;
    float digonal_1 = 0;
    float digonal_2 = 0;
    for(int i = 0; i < gates.size(); i++)
    {
        Point_t tl_pt = gates[i].get_vertex(1);
        Point_t tr_pt = gates[i].get_vertex(2);
        Point_t bl_pt = gates[i].get_vertex(3);
        Point_t br_pt = gates[i].get_vertex(4);
        digonal_1 = sqrtf(powf((tl_pt.row - br_pt.row),2) + powf((tl_pt.col - br_pt.col),2));
        digonal_2 = sqrtf(powf((tr_pt.row - bl_pt.row),2) + powf((tr_pt.col - bl_pt.col),2));
        if(digonal_1 * digonal_2 > max_area)
        {
            max_area = digonal_1 * digonal_2;
            max_gate = gates[i];
        }
    }
    guard.lock();
    this->best_gate = max_gate;
    guard.unlock();
    cv::cvtColor(gray,img,CV_GRAY2BGR);
    // draw corners tl->red tr->yellow br->blue bl->green
    draw_corners(img, tl_corners, Scalar(  0,  0,255));
    draw_corners(img, tr_corners, Scalar(  0,255,255));
    draw_corners(img, bl_corners, Scalar(255,  0,  0));
    draw_corners(img, br_corners, Scalar(  0,255,  0));
    // draw detections
    for(int i = 0; i < gates.size(); i++)
    {
        this->best_gate.draw_polygon(img,Scalar(255,255,0));
    }
}

Polygon VJGateDetector::undistortion(Polygon &gate)
{
    // point 1
    Point_t pt1 = gate.get_vertex(1);
    float x = pt1.col - principal_cx;
    float y = pt1.row - principal_cy;
    float r = sqrtf(powf(x,2) +	 powf(y,2));
    float R = focal_length * tanf(asinf(sinf(atanf(r / focal_length))* undistortion_k));
    float ratio = R / r;
    x = ratio * x;
    y = ratio * y;
    pt1.col = x + principal_cx;
    pt1.row = y + principal_cy;

    // point 2
    Point_t pt2 = gate.get_vertex(2);
    x = pt2.col - principal_cx;
    y = pt2.row - principal_cy;
    r = sqrtf(powf(x,2) +	 powf(y,2));
    R = focal_length * tanf(asinf(sinf(atanf(r / focal_length))* undistortion_k));
    ratio = R / r;
    x = ratio * x;
    y = ratio * y;
    pt2.col = x + principal_cx;
    pt2.row = y + principal_cy;

    // point 3
    Point_t pt3 = gate.get_vertex(3);
    x = pt3.col - principal_cx;
    y = pt3.row - principal_cy;
    r = sqrtf(powf(x,2) +	 powf(y,2));
    R = focal_length * tanf(asinf(sinf(atanf(r / focal_length))* undistortion_k));
    ratio = R / r;
    x = ratio * x;
    y = ratio * y;
    pt3.col = x + principal_cx;
    pt3.row = y + principal_cy;

    // point 4
    Point_t pt4 = gate.get_vertex(4);
    x = pt4.col - principal_cx;
    y = pt4.row - principal_cy;
    r = sqrtf(powf(x,2) +	 powf(y,2));
    R = focal_length * tanf(asinf(sinf(atanf(r / focal_length))* undistortion_k));
    ratio = R / r;
    x = ratio * x;
    y = ratio * y;
    pt4.col = x + principal_cx;
    pt4.row = y + principal_cy;

    return Polygon(pt1,pt2,pt3,pt4);
}

void
VJGateDetector::setYMax(int y_max) {
    this->y_max = y_max;
    std::ostringstream msg;
    msg << "Setting y_max to: " << y_max << std::endl ;
    Comm::print(msg.str());
}

void
VJGateDetector::setYMin(int y_min) {
    this->y_min = y_min;
    std::ostringstream msg;
    msg << "Setting y_min to: " << y_min << std::endl ;
    Comm::print(msg.str());
}

void
VJGateDetector::setCrMin(int cr_min) {
    this->cr_min = cr_min;
    std::ostringstream msg;
    msg << "Setting cr_min to: " << cr_min << std::endl ;
    Comm::print(msg.str());
}

void
VJGateDetector::setCrMax(int cr_max) {
    this->cr_max = cr_max;
    std::ostringstream msg;
    msg << "Setting cr_max to: " << cr_max << std::endl ;
    Comm::print(msg.str());
}

void
VJGateDetector::setCbMin(int cb_min) {
    this->cb_min = cb_min;
    std::ostringstream msg;
    msg << "Setting cb_min to: " << cb_min << std::endl ;
    Comm::print(msg.str());
}

void
VJGateDetector::setCbMax(int cb_max) {
    this->cb_max = cb_max;
    std::ostringstream msg;
    msg << "Setting cb_max to: " << cb_max << std::endl ;
    Comm::print(msg.str());
}

Polygon VJGateDetector::getBestGate() {
	guard.lock();
	Polygon bestGateCopy(best_gate);
	guard.unlock();
	return bestGateCopy;
}

}

