#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
int main()
{
	cv::Mat mat = cv::imread("/Users/LiShuo/dronerace2018/target/test/examples/opencv/resource/00001.jpg");
	cv::imshow("Hello OpenCV", mat);
	cv::waitKey(0);
    return 0;
}
