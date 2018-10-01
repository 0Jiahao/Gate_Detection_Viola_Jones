#pragma once
#include "opencv2/core.hpp"
#include "Frame.h"
#include "Polygon.h"
#include <vector>
#include <time.h>
#include <atomic>
#include <Comm.h>
#include <mutex>
#include "Config.h"

struct image_t {
	int w;             ///< Image width
	int h;             ///< Image height
	int step0;
	int step1;
	int buf_idx;        ///< Buffer index for V4L2 freeing
	int buf_size;      ///< The buffer size
	uchar * buf;              ///< Image buffer (depending on the image_type)
};



namespace mav{

struct DEBUG_COLOR_FITNESS
{
	double cf;
	int totalPixcel;
	int colorPixcel;
};



enum Direction {UP,DOWN,LEFT,RIGHT};

class SnakeGateDetector
{
private:
    SnakeGateDetector(const SnakeGateDetector &that);
    cv::Mat mat;
    cv::Mat YCrCb;
    cv::Mat mask;
	std::mutex guard;
    struct DEBUG_COLOR_FITNESS debugColorFitness;
	int n_gates;
	int n_samples;
    double cf_max = 0;
    Polygon best_gate;
    std::atomic<bool> newGate;
	bool check_color(image_t* im,int cols, int rows);
    bool check_color_ycbcr(image_t* im,int cols, int rows);
	bool snake_up_and_down(image_t * im, Point_t & p0, Point_t & p_t1, Point_t & p_t2 );
	bool snake_left_and_right(image_t *im, Point_t & p0,Point_t & p_t1,Point_t & p_t2);
    Point_t search_horizontal_pixels(image_t * im,Point_t &p0,int pixels,enum Direction direction,bool &flag);
	Point_t search_vertical_pixels(image_t * im,Point_t &p0,int pixels,enum Direction direction,bool &flag);
	bool check_coordinate(image_t * im,int x,int y);
	void get_square_from_gate_corner(Polygon &gate_raw,Polygon &square);
	double color_fitness(image_t * im,Polygon & gate);
	double distance_2D(Point_t& p1,Point_t& p2){return sqrt(pow(p1.col-p2.col,2)+pow(p1.row-p2.row,2));};
	std::vector<Point_t> linePoints(Point_t &p0,Point_t &p1);
	void colorBetweenTwoPoints(image_t * im,Point_t &p0,Point_t &p1,int * numColorPixels, int * numTotalPixels);
    void refineRawGate(image_t * im,Polygon & rawGate);
	Polygon getHis(Point_t &p0,int edge,double theta,image_t *im);
	Point_t coordinate_transform(Point_t & p_local,Point_t & p_origin, double theta);
    bool isColorBlock(image_t * im,Polygon & poly);

    int y_max=Y_MAX,y_min=Y_MIN,cr_max=Cr_MAX,cr_min=Cr_MIN,cb_max=Cb_MAX,cb_min=Cb_MIN,min_length=MIN_LENGTH;
    float min_color_fitness=MIN_COLOR_FITNESS,max_color_fitness_corner23=MAX_COLOR_FITNESS_CORNER23,threshold_color_block=THRESHOLD_COLOR_BLOCK;
    int principal_cx=CAMERA_C_X, principal_cy=CAMERA_C_Y;
    float undistortion_k=CAMERA__DISTORTION_K, focal_length = CAMERA_F;

    Polygon undistortion(Polygon &gate);
    public:
	SnakeGateDetector() = default;
	~SnakeGateDetector() = default;
	void  snake_gate_detection(Frame &frame);

	std :: vector<Polygon> gate_candidates_raw;

	Polygon getBestGate();
	bool hasNewGate(){return newGate;};
	void setNewGate(){newGate = true;};
	void clearNewGate(){newGate = false;};
	struct DEBUG_COLOR_FITNESS getDebugCf(){return debugColorFitness;};



	/* Setters for parameters */

    void setYMax(int y_max);
    void setYMin(int y_min);
    void setCrMax(int cr_max);
    void setCrMin(int cr_min);
    void setCbMax(int cb_max);
    void setCbMin(int cb_min);
    void setMinLength(int min_length);
    void setMinColorFitness(float min_color_fitness);
    void setMaxColorFitnessCorner23(float max_color_fitness_corner23);
    void setThresholdColorBlock(float threshold_color_block);
    void setUndistortionK(float k);
    void setFocalLength(float f);
    void setCx(int cx);
    void setCy(int cy);
};


}
