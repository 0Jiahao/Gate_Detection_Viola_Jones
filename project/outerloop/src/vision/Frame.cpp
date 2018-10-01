#include "Frame.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "stdio.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

namespace mav{

Frame::Frame(const Frame &that){
	mat = that.mat;
}

Frame::Frame(cv::Mat &mat) {
	this->mat = mat;
}


cv::Mat Frame::getMat() {
	return this->mat;
}



void Frame::writeText(std::string text,cv::Point position)
{
	cv::putText(this->mat, //target image
				text, //text
				position,
				cv::FONT_HERSHEY_PLAIN,
				0.6,
				CV_RGB(255, 255, 255), //font color
				1);
}

void Frame::drawPolygon(Polygon polygon,cv::Scalar color) {
	polygon.draw_polygon(mat,color);
}
}
