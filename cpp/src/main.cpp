#include "opencv2/opencv.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

class Gate
{
public:
    Point tl;
    Point tr;
    Point br;
    Point bl;
    Gate(Point _tl, Point _tr, Point _br, Point _bl)
    {
        this->tl = _tl;
        this->tr = _tr;
        this->br = _br;
        this->bl = _bl; 
    }
};

Point Rect_center(Rect rect)
{
    return (0.5 * rect.tl() + 0.5 * rect.br());
}

void draw_corners(Mat& img, vector<Rect> corners, Scalar color)
{
    for(int i = 0; i < corners.size(); i++)
    {
        rectangle(img, corners[i], color, 2, 8, 0 );
    }
}

void draw_gates(Mat& img, vector<Gate> gates)
{
    for(int i = 0; i < gates.size(); i++)
        {
            line(img,gates[i].tl,gates[i].tr,Scalar(203,192,255),5);
            line(img,gates[i].tr,gates[i].br,Scalar(203,192,255),5);
            line(img,gates[i].br,gates[i].bl,Scalar(203,192,255),5);
            line(img,gates[i].bl,gates[i].tl,Scalar(203,192,255),5);
        }
}

void draw_best_gates(Mat& img, Gate gates)
{
    line(img,gates.tl,gates.tr,Scalar(203,192,255),3);
    line(img,gates.tr,gates.br,Scalar(203,192,255),3);
    line(img,gates.br,gates.bl,Scalar(203,192,255),3);
    line(img,gates.bl,gates.tl,Scalar(203,192,255),3);
}

vector<Gate> extract_gates_3(vector<Rect> tl_corners, vector<Rect> tr_corners, vector<Rect> br_corners, vector<Rect> bl_corners, float ratio, float angle_c, float dist_c)
{
    vector<Gate> gates;
        //tl -> tr -> br
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
                            Point p_1 = Rect_center(tl_corners[tl]);
                            Point p_2 = Rect_center(tr_corners[tr]);
                            Point p_3 = Rect_center(br_corners[br]);
                            float Len_1 = sqrtf(powf(p_1.x-p_2.x,2) + powf(p_1.y-p_2.y,2));
                            float Len_2 = sqrtf(powf(p_2.x-p_3.x,2) + powf(p_2.y-p_3.y,2));
                            float Len_3 = sqrtf(powf(p_1.x-p_3.x,2) + powf(p_1.y-p_3.y,2));
                            float angle = acosf((powf(Len_1,2) + powf(Len_2,2) - powf(Len_3,2)) / (2 * Len_1 * Len_2)) / 3.14 * 180;
                            if((Len_1 / Len_2 < ratio) && (Len_2 / Len_1 < ratio) && (angle > 90 - angle_c) && (angle < 90 + angle_c))
                            {
                                    Rect Virtual_corner(tl_corners[tl].tl() + \
                                                        br_corners[br].tl() - \
                                                        tr_corners[tr].tl() ,
                                                        tr_corners[tr].size());
                                    Point Virtual_center = Rect_center(Virtual_corner);
                                    Point best_center = Virtual_center;
                                    float min_dist = 800;
                                    for(int bl = 0; bl < bl_corners.size(); bl++)
                                    {
                                        float dist = sqrtf(powf(Virtual_center.x - Rect_center(bl_corners[bl]).x,2) + powf(Virtual_center.y - Rect_center(bl_corners[bl]).y,2));
                                        if((dist < dist_c) && (dist < min_dist))
                                        {
                                            best_center = Rect_center(bl_corners[bl]);
                                            min_dist = dist;
                                        }
                                    }
                                    Gate detected_gate(Rect_center(tl_corners[tl]), \
                                                       Rect_center(tr_corners[tr]), \
                                                       Rect_center(br_corners[br]), \
                                                       best_center);
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
                            Point p_1 = Rect_center(tr_corners[tr]);
                            Point p_2 = Rect_center(br_corners[br]);
                            Point p_3 = Rect_center(bl_corners[bl]);
                            float Len_1 = sqrtf(powf(p_1.x-p_2.x,2) + powf(p_1.y-p_2.y,2));
                            float Len_2 = sqrtf(powf(p_2.x-p_3.x,2) + powf(p_2.y-p_3.y,2));
                            float Len_3 = sqrtf(powf(p_1.x-p_3.x,2) + powf(p_1.y-p_3.y,2));
                            float angle = acosf((powf(Len_1,2) + powf(Len_2,2) - powf(Len_3,2)) / (2 * Len_1 * Len_2)) / 3.14 * 180;
                            if((Len_1 / Len_2 < ratio) && (Len_2 / Len_1 < ratio) && (angle > 90 - angle_c) && (angle < 90 + angle_c))
                            {
                                    Rect Virtual_corner(tr_corners[tr].tl() + \
                                                        bl_corners[bl].tl() - \
                                                        br_corners[br].tl() ,
                                                        br_corners[br].size());
                                    Point Virtual_center = Rect_center(Virtual_corner);
                                    Point best_center = Virtual_center;
                                    float min_dist = 800;
                                    for(int tl = 0; tl < tl_corners.size(); tl++)
                                    {
                                        float dist = sqrtf(powf(Virtual_center.x - Rect_center(tl_corners[tl]).x,2) + powf(Virtual_center.y - Rect_center(tl_corners[tl]).y,2));
                                        if((dist < dist_c) && (dist < min_dist))
                                        {
                                            best_center = Rect_center(tl_corners[tl]);
                                            min_dist = dist;
                                        }
                                    }
                                    Gate detected_gate(best_center, \
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
                            Point p_1 = Rect_center(br_corners[br]);
                            Point p_2 = Rect_center(bl_corners[bl]);
                            Point p_3 = Rect_center(tl_corners[tl]);
                            float Len_1 = sqrtf(powf(p_1.x-p_2.x,2) + powf(p_1.y-p_2.y,2));
                            float Len_2 = sqrtf(powf(p_2.x-p_3.x,2) + powf(p_2.y-p_3.y,2));
                            float Len_3 = sqrtf(powf(p_1.x-p_3.x,2) + powf(p_1.y-p_3.y,2));
                            float angle = acosf((powf(Len_1,2) + powf(Len_2,2) - powf(Len_3,2)) / (2 * Len_1 * Len_2)) / 3.14 * 180;
                            if((Len_1 / Len_2 < ratio) && (Len_2 / Len_1 < ratio) && (angle > 90 - angle_c) && (angle < 90 + angle_c))
                            {
                                    Rect Virtual_corner(br_corners[br].tl() + \
                                                        tl_corners[tl].tl() - \
                                                        bl_corners[bl].tl() ,
                                                        bl_corners[bl].size());
                                    Point Virtual_center = Rect_center(Virtual_corner);
                                    Point best_center = Virtual_center;
                                    float min_dist = 800;
                                    for(int tr = 0; tr < tr_corners.size(); tr++)
                                    {
                                        float dist = sqrtf(powf(Virtual_center.x - Rect_center(tr_corners[tr]).x,2) + powf(Virtual_center.y - Rect_center(tr_corners[tr]).y,2));
                                        if((dist < dist_c) && (dist < min_dist))
                                        {
                                            best_center = Rect_center(tr_corners[tr]);
                                            min_dist = dist;
                                        }
                                    }
                                    Gate detected_gate(Rect_center(tl_corners[tl]), \
                                                       best_center, \
                                                       Rect_center(br_corners[br]), \
                                                       Rect_center(bl_corners[bl]));
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
                            Point p_1 = Rect_center(bl_corners[bl]);
                            Point p_2 = Rect_center(tl_corners[tl]);
                            Point p_3 = Rect_center(tr_corners[tr]);
                            float Len_1 = sqrtf(powf(p_1.x-p_2.x,2) + powf(p_1.y-p_2.y,2));
                            float Len_2 = sqrtf(powf(p_2.x-p_3.x,2) + powf(p_2.y-p_3.y,2));
                            float Len_3 = sqrtf(powf(p_1.x-p_3.x,2) + powf(p_1.y-p_3.y,2));
                            float angle = acosf((powf(Len_1,2) + powf(Len_2,2) - powf(Len_3,2)) / (2 * Len_1 * Len_2)) / 3.14 * 180;
                            if((Len_1 / Len_2 < ratio) && (Len_2 / Len_1 < ratio) && (angle > 90 - angle_c) && (angle < 90 + angle_c))
                            {
                                    Rect Virtual_corner(bl_corners[bl].tl() + \
                                                        tr_corners[tr].tl() - \
                                                        tl_corners[tl].tl() ,
                                                        tl_corners[tl].size());
                                    Point Virtual_center = Rect_center(Virtual_corner);
                                    Point best_center = Virtual_center;
                                    float min_dist = 800;
                                    for(int br = 0; br < br_corners.size(); br++)
                                    {
                                        float dist = sqrtf(powf(Virtual_center.x - Rect_center(br_corners[br]).x,2) + powf(Virtual_center.y - Rect_center(br_corners[br]).y,2));
                                        if((dist < dist_c) && (dist < min_dist))
                                        {
                                            best_center = Rect_center(br_corners[br]);
                                            min_dist = dist;
                                        }
                                    }
                                    Gate detected_gate(Rect_center(tl_corners[tl]), \
                                                       Rect_center(tr_corners[tr]), \
                                                       best_center, \
                                                       Rect_center(bl_corners[bl]));
                                    gates.push_back(detected_gate);
                            }
                        }
                    }
                }
            }
        }
    return gates;
}

int main()
{
    // load the trained model
    CascadeClassifier tl_light;
    CascadeClassifier tr_light;
    CascadeClassifier bl_light;
    CascadeClassifier br_light;
    tl_light.load("../../model/0.8_30_t_l_corner_mad.xml");
    tr_light.load("../../model/0.8_30_t_r_corner_mad.xml");
    bl_light.load("../../model/0.8_30_b_l_corner_mad.xml");
    br_light.load("../../model/0.8_30_b_r_corner_mad.xml");
    // file name in ../dataset/tst_reall
    //234 222 250 36 285
    for(int i = 36; i < 285;i++)
    {
        cout << i << endl;
        ostringstream file_idx;
        // file_idx << setw(5) << setfill('0') << i;
        file_idx << i;
        string filename = "../../dataset/mad_img/my_photo-" + file_idx.str() + ".jpg";
        // read gray image
        Mat raw, img;
        raw = cv::imread(filename);
        cv::cvtColor(raw, img, CV_BGR2GRAY);
        // auto m = getRotationMatrix2D(Point2f(img.cols/2,img.rows/2),30,1);
        // warpAffine(img,img,m,img.size());
        resize(img, img, Size(640,480));
        // extract corners
        vector<Rect> tl_corners;
        vector<Rect> tr_corners;
        vector<Rect> bl_corners;
        vector<Rect> br_corners;
        tl_light.detectMultiScale( img, tl_corners, 1.1, 2,0, Size(20, 20),Size(35,35));
        tr_light.detectMultiScale( img, tr_corners, 1.1, 2,0, Size(20, 20),Size(35,35));
        bl_light.detectMultiScale( img, bl_corners, 1.1, 2,0, Size(20, 20),Size(35,35));
        br_light.detectMultiScale( img, br_corners, 1.1, 2,0, Size(20, 20),Size(35,35));
        cv::cvtColor(img, img, CV_GRAY2BGR);
        // extract rectangles based on corners
        vector<Gate> gates;
        gates = extract_gates_3(tl_corners, tr_corners, br_corners, bl_corners, 1.15, 3, 15);
        Gate best_gate(Point(0,0),Point(0,0),Point(0,0),Point(0,0));
        if(gates.size() > 0)
        {
            Gate max_gate(Point(0,0),Point(0,0),Point(0,0),Point(0,0));
            float max_area = 0;
            float digonal_1 = 0;
            float digonal_2 = 0;
            for(int i = 0; i < gates.size(); i++)
            {
                digonal_1 = sqrtf(powf((gates[i].tl.x - gates[i].br.x),2) + powf((gates[i].tl.y - gates[i].br.y),2));
                digonal_2 = sqrtf(powf((gates[i].tr.x - gates[i].bl.x),2) + powf((gates[i].tr.y - gates[i].bl.y),2));
                if(digonal_1 * digonal_2 > max_area)
                {
                    max_area = digonal_1 * digonal_2;
                    max_gate = gates[i];
                }
            }
            best_gate = max_gate;
        }
        // draw detections
        draw_best_gates(raw, best_gate);
        // draw corners tl->red tr->yellow br->blue bl->green
        draw_corners(raw, tl_corners, Scalar(  0,  0,255));
        draw_corners(raw, tr_corners, Scalar(  0,255,255));
        draw_corners(raw, br_corners, Scalar(255,  0,  0));
        draw_corners(raw, bl_corners, Scalar(  0,255,  0));
        // filename = "../../" + file_idx.str() + ".jpg";
        // imwrite(filename, raw);
        imshow("img",raw);
        cv::waitKey(150);
    }
 }
