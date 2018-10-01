//
// Created by phil on 04/06/2018.
//

#ifndef POLYGON_H
#define POLYGON_H

#include <opencv2/core/mat.hpp>

namespace mav{

    struct Point_t
    {
        int row;
        int col;
    };

    struct Vector_t
    {
        int x;
        int y;
    };

    struct Square_t
    {
        Point_t center;
        int edge_length;
    };

class Polygon
{
private:

    Point_t m_p1;
    Point_t m_p2;
    Point_t m_p3;
    Point_t m_p4;
public:

    Polygon() = default;
    Polygon(Point_t p1,Point_t p2,Point_t p3,Point_t p4);
    ~Polygon() = default;
    void draw_polygon(cv::Mat &mat,cv::Scalar color);
    void draw_polygon_vetex(cv::Mat &mat);
    Point_t get_vertex(int id_vertex);
    void set_vertex(int id_vertex,Point_t point);
};
}

#endif //OUTERLOOP_SIM_POLYGON_H
