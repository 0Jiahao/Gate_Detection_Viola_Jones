#pragma once
#include "opencv2/core.hpp"
#include "Polygon.h"

namespace mav{
class Frame
{
private:
	cv::Mat mat;
public:
	Frame() = default;
	~Frame() = default;
	Frame(cv::Mat &mat);
	Frame(const Frame &that);
	cv::Mat getMat();
	void writeText(std::string text,cv::Point position);
	void drawPolygon(Polygon polygon,cv::Scalar color);
};
}

