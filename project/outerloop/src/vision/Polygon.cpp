//
// Created by phil on 04/06/2018.
//

#include <opencv2/imgproc.hpp>
#include "Polygon.h"
namespace mav{
Polygon::Polygon(Point_t p1,Point_t p2,Point_t p3,Point_t p4){
    this->m_p1.col = p1.col;
    this->m_p1.row = p1.row;
    this->m_p2.col = p2.col;
    this->m_p2.row = p2.row;
    this->m_p3.col = p3.col;
    this->m_p3.row = p3.row;
    this->m_p4.col = p4.col;
    this->m_p4.row = p4.row;
}

void Polygon::draw_polygon(cv::Mat &mat,cv::Scalar color)
{
    cv::line(mat, cv::Point(m_p1.col,m_p1.row),cv:: Point(m_p2.col,m_p2.row),color,2);
    cv::line(mat, cv::Point(m_p1.col,m_p1.row),cv:: Point(m_p3.col,m_p3.row),color,2);
    cv::line(mat, cv::Point(m_p4.col,m_p4.row),cv:: Point(m_p3.col,m_p3.row),color,2);
    cv::line(mat, cv::Point(m_p4.col,m_p4.row),cv:: Point(m_p2.col,m_p2.row),color,2);

}
void Polygon::draw_polygon_vetex(cv::Mat &mat){
    cv::circle(mat, cv::Point(m_p1.col,m_p1.row), 2, cv::Scalar(0,0,255));
    cv::circle(mat, cv::Point(m_p2.col,m_p2.row), 2, cv::Scalar(0,0,255));
    cv::circle(mat, cv::Point(m_p3.col,m_p3.row), 2, cv::Scalar(0,0,255));
    cv::circle(mat, cv::Point(m_p4.col,m_p4.row), 2, cv::Scalar(0,0,255));
}

Point_t Polygon::get_vertex(int id_vertex){
    switch(id_vertex){
        case 1:
            return this->m_p1;
        case 2:
            return this->m_p2;
        case 3:
            return this->m_p3;
        case 4:
            return this->m_p4;
        default:
            return this->m_p1;
    }
}


void Polygon::set_vertex(int id_vertex,Point_t point)
{
    switch(id_vertex){
        case 1:
            this->m_p1 = point;
            break;
        case 2:
            this->m_p2 = point;
            break;
        case 3:
            this->m_p3 = point;
            break;
        case 4:
            this->m_p4 = point;
    }
}
}