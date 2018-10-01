#include "SnakeGateDetector.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "stdio.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "Frame.h"
#include <iostream>
#include <Comm.h>
#include <time.h>
#include <Scheduler.h>
#include "Config.h"
#include <cmath>

namespace mav{

SnakeGateDetector::SnakeGateDetector(const SnakeGateDetector &that){
    this->mat = that.mat;
	this->gate_candidates_raw = that.gate_candidates_raw;
	//this->square_candidates = that.square_candidates;
}





	void SnakeGateDetector::snake_gate_detection(Frame &frame) {

	    long long time  = Scheduler::getStartupTimeMillis();
     	YCrCb = frame.getMat();

		if(DEBUG_SNAKE_LEFT_RIGHT | DEBUG_SNAKE_UP_DOWN | DEBUG_CHECK_COLOR_BLOCK | WAIT_KEY_TIME)
		{
            cv::inRange(this->YCrCb,cv::Scalar(y_min,cr_min,cb_min), cv::Scalar(y_max,cr_max,cb_max),this->mask);
			cv::imshow("mask",this->mask);
			cv::waitKey(WAIT_KEY_TIME);
		}
		image_t image;
		image.w = this->YCrCb.cols;
		image.h = this->YCrCb.rows;
		image.buf = this->YCrCb.data;
		image.step0 = this->YCrCb.step[0];
		image.step1 = this->YCrCb.step[1];
		image_t *img = &image;

		this->n_gates = 0;
		this->gate_candidates_raw.clear();
        cf_max = 0;
		double cf = 0;
		for (int i = 0; i < N_SAMPLES; i++) {
			// get a random coordinate:
			// srand(time(NULL));
            if(n_gates > 5)
			{
				break;
			}
			int cols = rand() % img->w;
			int rows = rand() % img->h;
			struct Point_t p0, p_t1, p_t2,p1,p2,p3,p4;
			p0.col = cols;
			p0.row = rows;

			// check if it has the right color
			if (check_color(img, p0.col,p0.row)) {

				// snake up and down:
				if (snake_up_and_down(img,p0,p_t1,p_t2))
				{
				    if ( snake_left_and_right(img,p_t1,p1,p2) &
					snake_left_and_right(img,p_t2,p3,p4))
					{
					    Polygon detected_gate_raw(p1,p2,p3,p4);
                        if(isColorBlock(img,detected_gate_raw))
						{
                            //std::cout << "[snake gate detector] it is a color block" << std::endl;
							continue;
						}
						refineRawGate(img,detected_gate_raw);
                        cf=color_fitness(img,detected_gate_raw);
                        //std::cout << "[snake gate detector] cf is " << cf <<std::endl;
                        if(cf>min_color_fitness)
						{
							this->gate_candidates_raw.push_back(detected_gate_raw);
							this->n_gates++;
							//std::cout << "[snake gate detector n_gate]" << n_gates << std::endl;
							if(cf > cf_max)
							{
								cf_max = cf;
								guard.lock();
								best_gate = undistortion(detected_gate_raw);
								guard.unlock();
							}
						}
					}
				}
			}
		}
		// snake gate is done. Check if there is a gate detected
		if(cf_max!=0)
		{
			//std::cout << "Gate is detected" << std::endl;
            std::ostringstream msg;
            msg << "[snake gate detector] best gate p1 = (" << best_gate.get_vertex(1).col << "," << best_gate.get_vertex(1).row<<
                ")" << std::endl;
            Comm::print(msg.str());
            setNewGate();
		}
		//time = Scheduler::getStartupTimeMillis()-time;
		//std::cout << "snake gate detection time is " << time/1000.0 << std::endl;
		//static long long lastTime = Scheduler::getStartupTimeMillis();
		//std::cout << "snake gate detection call after " << Scheduler::getStartupTimeMillis()-lastTime << std::endl;
		//lastTime = Scheduler::getStartupTimeMillis();
	}


	bool SnakeGateDetector :: check_color(image_t* im,int cols, int rows){
		if (cols < 0 || cols >= im->w-2 || rows < 0 || rows >= im->h) {
			return 0;
		}

        int  Y = *(im->buf + im->step0 * rows+ cols*im->step1);

        int  Cr,Cb;
		if (cols % 2 == 0){
            Cb = *(im->buf+im->step0*rows+cols*im->step1+1);
            Cr = *(im->buf+im->step0*rows+(cols+1)*im->step1+1);
		}else{
            Cr = *(im->buf+im->step0*rows+cols*im->step1+1);
            Cb = *(im->buf+im->step0*rows+(cols+1)*im->step1+1);
		}

		if (
				(Y >= y_min)
				&& (Y <= y_max)
				&& (Cr>=cr_min)
				&& (Cr<=cr_max)
				&& (Cb>=cb_min)
				&& (Cb<=cb_max)
				) {
			return 1;
		} else {
			return 0;
		}
	}

    bool SnakeGateDetector :: check_color_ycbcr(image_t* im,int cols, int rows){
        if (cols < 0 || cols >= im->w || rows < 0 || rows >= im->h) {
            return 0;
        }

        int  Y = *(im->buf + im->step0 * rows+ cols*im->step1);
        int  Cr = *(im->buf+im->step0*rows+cols*im->step1+1);
        int  Cb = *(im->buf+im->step0*rows+cols*im->step1+2);

        if (
                (Y >= y_min)
                && (Y <= y_max)
                && (Cr>=cr_min)
                && (Cr<=cr_max)
                && (Cb>=cb_min)
                && (Cb<=cb_max)
                ) {
            return 1;
        } else {
            return 0;
        }
    }


	bool SnakeGateDetector::snake_left_and_right(image_t *im, Point_t & p0,Point_t & p_t1,Point_t & p_t2){

		bool done = false;
		p_t1.row = p0.row;
		p_t1 . col = p0 .col;
		p_t2.row = p0.row;
		p_t2 . col = p0 .col;

		// snake towards negative x (left) p_t1 is left one and p_t2 is right one
		while (!done) {
            if(DEBUG_SNAKE_LEFT_RIGHT)
			{
				cv::circle(this->mat,cv::Point(p_t1.col,p_t1.row),1,cv::Scalar(0,255,0));
				cv::imshow("temp",this->mat);
				cv::waitKey(WAIT_KEY_TIME);
			}
			bool next_pixel_found = false;
			int i;
			for(i = 0;i<=MAX_HORIZONTAL_PIXELS;i++)
			{
				Point_t p_temp = search_vertical_pixels(im,p_t1,i,LEFT,next_pixel_found);
				if(next_pixel_found)
				{
					p_t1 = p_temp;
					break;
				}
			}
			if(i == MAX_HORIZONTAL_PIXELS+1 && !next_pixel_found)
			{
				done = true;
			}
		}

		done = false;
		// snake towards positive x (right)
		while (p_t2.col < im->w - 1 && !done) {

            if(DEBUG_SNAKE_LEFT_RIGHT)
			{
				cv::circle(this->mat,cv::Point(p_t2.col,p_t2.row),1,cv::Scalar(0,255,0));
				cv::imshow("temp",this->mat);
				cv::waitKey(WAIT_KEY_TIME);

			}
			bool next_pixel_found = false;
			int i;
			for(i = 0;i<=MAX_HORIZONTAL_PIXELS;i++)
			{
				Point_t p_temp = search_vertical_pixels(im,p_t2,i,RIGHT,next_pixel_found);
				if(next_pixel_found)
				{
					p_t2 = p_temp;
					break;
				}
			}
			if(i == MAX_HORIZONTAL_PIXELS+1 && !next_pixel_found)
			{
				done = true;
			}
		}
		if(distance_2D(p_t1,p_t2)>min_length)
			return 1;
		else
			return 0;
	}

	Point_t SnakeGateDetector::search_vertical_pixels(image_t * im,Point_t &p0,int pixels,enum Direction direction,bool &flag)
	{
		Point_t p = Point_t();
		int x=0,y=0;
		// up
		if(direction == LEFT)
		{
			x = p0.col-1 ;
			y = p0.row-pixels;
		}
		else if(direction == RIGHT)
		{
			x = p0.col+1 ;
			y = p0.row - pixels;
		}
        if(check_coordinate(im,x,y) & check_color(im,x,y))
		{
			p.col = x;
			p.row = y;
			flag = true;
			return p;
		}


		// down
		if(direction == LEFT)
		{
			x = p0.col - 1;
			y = p0.row+pixels;
		}
		else if(direction == RIGHT)
		{
			x = p0.col + 1;
			y = p0.row + pixels;
		}
		if(check_coordinate(im,x,y) & check_color(im,x,y)) {
			p.col = x;
			p.row = y;
			flag = true;
			return p;
		}
		flag = false;
		return p;
	}

	bool SnakeGateDetector::snake_up_and_down(image_t * im, Point_t & p0, Point_t & p_t1, Point_t & p_t2 ){
		bool done = false;

        p_t1 = p0;
		// snake towards negative y (down?)
		while (!done) {
            if(DEBUG_SNAKE_UP_DOWN)
			{
				cv::circle(this->mat,cv::Point(p_t1.col,p_t1.row),1,cv::Scalar(255,0,0));
				cv::imshow("temp",this->mat);
				cv::waitKey(WAIT_KEY_TIME);
			}
			bool next_pixel_found;
            int i;
			for(i = 0;i<=MAX_HORIZONTAL_PIXELS;i++)
			{
                Point_t p_temp;
				p_temp = search_horizontal_pixels(im,p_t1,i,UP,next_pixel_found);
                if(next_pixel_found == true)
				{
					p_t1 = p_temp;
					break;
				}
			}
            if(i == MAX_HORIZONTAL_PIXELS+1 && next_pixel_found == false)
			{
				done = true;
			}
		}

        p_t2 = p0;
		done = false;
		// snake towards positive y (up?)
		// while ((*y_high) < im->h - 1 && !done) {
		while (!done) {

            if(DEBUG_SNAKE_UP_DOWN)
			{
				cv::circle(this->mat,cv::Point(p_t2.col,p_t2.row),1,cv::Scalar(255,0,0));
				cv::imshow("temp",this->mat);
				cv::waitKey(WAIT_KEY_TIME);
			}
			bool next_pixel_found;
            int i;
			for(i = 0;i<=MAX_HORIZONTAL_PIXELS;i++)
			{
				Point_t p_temp;
				p_temp = search_horizontal_pixels(im,p_t2,i,DOWN,next_pixel_found);
				if(next_pixel_found == true)
				{
					p_t2 = p_temp;
					break;
				}
			}
            if(i == MAX_HORIZONTAL_PIXELS+1 && next_pixel_found == false)
			{
				done = true;
			}
		}
		if(distance_2D(p_t1,p_t2)>min_length)
			return true;
		else
			return false;
	}


	Point_t SnakeGateDetector:: search_horizontal_pixels(image_t * im,Point_t &p0,int pixels,enum Direction direction,bool &flag)
	{
		// this function is used to search the pixels above/below the base point p0 to avoid stucks.
		// flag == 1  a search is done successfully

		Point_t p = Point_t();
		int x=0,y=0;
		// To right
		if(direction == UP)
		{
			x = p0.col + pixels;
			y = p0.row-1;
		}
		else if(direction == DOWN)
		{
			x = p0.col + pixels;
			y = p0.row+1;
		}
        if(check_coordinate(im,x,y) & check_color(im,x,y))
		{
			p.col = x;
			p.row = y;
			flag = true;
			return p;
		}


		// to left
		if(direction == UP)
		{
			x = p0.col - pixels;
			y = p0.row-1;
		}
		else if(direction == DOWN)
		{
			x = p0.col - pixels;
			y = p0.row+1;
		}
		if(check_coordinate(im,x,y) & check_color(im,x,y))
		{
			p.col = x;
			p.row = y;
			flag = true;
			return p;
		}

		flag = false;
		return p;

	}


	void SnakeGateDetector::get_square_from_gate_corner(Polygon &gate_raw,Polygon &square)
	{


	    Point_t p1 = gate_raw.get_vertex(1);
		Point_t p2 = gate_raw.get_vertex(2);
		Point_t p3 = gate_raw.get_vertex(3);
		Point_t p4 = gate_raw.get_vertex(4);

		int col_temp[4] = {p1.col,p2.col,p3.col,p4.col};
		int row_temp[4] = {p1.row,p2.row,p3.row,p4.row};
		int col_min = 99999;
		int row_min = 99999;
		int col_max= -99999;
		int row_max = -99999;

	    for (int i = 0; i< 4;i++)
		{
			if (col_temp[i] > col_max) col_max = col_temp[i];
			if (col_temp[i] < col_min) col_min= col_temp[i];
			if (row_temp[i] > row_max) row_max= row_temp[i];
			if (row_temp[i] < row_min) row_min= row_temp[i];
		}

		int retangle_col_length = col_max-col_min;
		int retangle_row_length = row_max-row_min;
		int center_col= col_min + retangle_col_length/2;
		int center_row= row_min + retangle_row_length/2;

		//  check if this square exceeds the edge of the image

        int square_edge = 0;
		retangle_col_length>retangle_row_length?square_edge = retangle_col_length:square_edge= retangle_row_length;

		// in some situations, this square exceed the edge of image, we should make the square smaller
		int dis_center_4_edges[4] = {center_row, this->YCrCb.cols - center_col,
		this->YCrCb.rows - center_row, center_col};
		for (int i = 0;i<4;i++)
		{
			if (square_edge > dis_center_4_edges[i])
			{
				square_edge = dis_center_4_edges[i];
			}
		}

		Point_t sq_p1,sq_p2,sq_p3,sq_p4;
		sq_p1.row	= center_row-square_edge/2;
		sq_p1.col	= center_col-square_edge/2;
		sq_p2.row	= center_row-square_edge/2;
		sq_p2.col	= center_col+square_edge/2;
		sq_p3.row	= center_row+square_edge/2;
		sq_p3.col	= center_col-square_edge/2;
		sq_p4.row	= center_row+square_edge/2;
		sq_p4.col	= center_col+square_edge/2;

		square.set_vertex(1,sq_p1);
		square.set_vertex(2,sq_p2);
		square.set_vertex(3,sq_p3);
		square.set_vertex(4,sq_p4);
	}




	bool SnakeGateDetector::check_coordinate(image_t * im,int x,int y)
	{
		if(x > 0 && x < im->w-1 && y > 0 && y < im->h-1)
		{
			return true;
		} else
		{
			return false;
		}
	}

	std::vector<Point_t> SnakeGateDetector::linePoints(Point_t &p0,Point_t &p1)
	{
		std::vector<Point_t> pointsOfLine;
		int x0 = p0.col;
		int y0 = p0.row;
		int x1 = p1.col;
		int y1 = p1.row;

		int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
		int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1;
		int err = (dx>dy ? dx : -dy)/2, e2;

		for(;;)
		{

            Point_t p0;
			p0.col = x0;
			p0.row = y0;
			pointsOfLine.push_back(p0);
			if (x0==x1 && y0==y1) break;
			e2 = err;
			if (e2 >-dx)
			{
				err -= dy;
				x0 += sx;
			}
			if (e2 < dy)
			{
				err += dx;
				y0 += sy;
			}
		}
		return pointsOfLine;
	}


	void SnakeGateDetector::colorBetweenTwoPoints(image_t * im,Point_t &p0,Point_t &p1,int * numColorPixels, int * numTotalPixels)
	{
        int numColorPixelsTemp = 0;
		int numTotalPixelsTemp = 0;

		int x0 = p0.col;
		int y0 = p0.row;
		int x1 = p1.col;
		int y1 = p1.row;

		int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
		int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1;
		int err = (dx>dy ? dx : -dy)/2, e2;

		for(;;)
		{
			if (x0==x1 && y0==y1) break;
			e2 = err;
			if (e2 >-dx)
			{
				err -= dy;
				x0 += sx;
			}
			if (e2 < dy)
			{
				err += dx;
				y0 += sy;
			}
			numTotalPixelsTemp++;
            if (check_color(im,x0,y0))
			{
				numColorPixelsTemp++;
			}
		}

		*numColorPixels = *numColorPixels + numColorPixelsTemp;
		*numTotalPixels = *numTotalPixels + numTotalPixelsTemp;
	}



	double SnakeGateDetector::color_fitness(image_t * im,Polygon & gate)
	{
        int color_pixels = 0;
		Point_t p1 = gate.get_vertex(1);
		Point_t p2 = gate.get_vertex(2);
		Point_t p3 = gate.get_vertex(3);
		Point_t p4 = gate.get_vertex(4);

        int numColorPixel = 0;
		int numTotalPixel = 0;

		colorBetweenTwoPoints(im,p1,p2,&numColorPixel,&numTotalPixel);
		colorBetweenTwoPoints(im,p2,p4,&numColorPixel,&numTotalPixel);
		colorBetweenTwoPoints(im,p3,p4,&numColorPixel,&numTotalPixel);
		colorBetweenTwoPoints(im,p3,p1,&numColorPixel,&numTotalPixel);

////		draw_points(gate_points);
////		cv::imshow("gate_points",this->mat);
////		cv::waitKey(0);
//
		double cf = (double)numColorPixel/(double)numTotalPixel;

//        debugColorFitness.cf = cf;
//        debugColorFitness.colorPixcel = numColorPixel;
//		debugColorFitness.totalPixcel = numTotalPixel;
		// to check if the detected gate is a line.
	    // When there is a line with around 45 degree, snake gate detection may think it is a gate.
        // It can avoid all the check. It is a special case
        float cf_2_3;
        int num_pix23 = 0;
		int num_colorPix23 = 0;
		colorBetweenTwoPoints(im,p2,p3,&num_colorPix23,&num_pix23);
		cf_2_3 = (double)num_colorPix23 / num_pix23;

        if (cf_2_3 > max_color_fitness_corner23)
			cf = 0;
		return cf;

	}
	void SnakeGateDetector::refineRawGate(image_t * im,Polygon & rawGate)
	{
		Point_t p1 = rawGate.get_vertex(1);
		Point_t p2 = rawGate.get_vertex(2);
		Point_t p3 = rawGate.get_vertex(3);
		Point_t p4 = rawGate.get_vertex(4);
		double theta = atan2(p2.row-p1.row,p2.col-p1.col);
		int edge = (distance_2D(p1,p2)+distance_2D(p3,p4)+distance_2D(p1,p3)+distance_2D(p2,p4))/4.0/3.0;
		Polygon roughSquare_1 = getHis(p1,edge,theta,im);
		Polygon roughSquare_2 = getHis(p2,edge,theta,im);
		Polygon roughSquare_3 = getHis(p3,edge,theta,im);
		Polygon roughSquare_4 = getHis(p4,edge,theta,im);
        rawGate.set_vertex(1,p1);
		rawGate.set_vertex(2,p2);
		rawGate.set_vertex(3,p3);
		rawGate.set_vertex(4,p4);
//		roughSquare_1.draw_polygon(this->mat,cv::Scalar(100,100,100));
//		roughSquare_2.draw_polygon(this->mat,cv::Scalar(100,100,100));
//		roughSquare_3.draw_polygon(this->mat,cv::Scalar(100,100,100));
//		roughSquare_4.draw_polygon(this->mat,cv::Scalar(100,100,100));


	}


	Polygon SnakeGateDetector::getHis(Point_t &p0,int edge,double theta,image_t *im)
	{
		// define four corners of local coordinates
		Point_t p1_local={-round(edge/2.0),-round(edge/2.0)};
		Point_t p2_local={-round(edge/2.0),round(edge/2.0)};
		Point_t p3_local={round(edge/2.0),-round(edge/2.0)};
		Point_t p4_local={round(edge/2.0),round(edge/2.0)};

		// rotate and translate local to global
        Point_t p1 = coordinate_transform(p1_local,p0,theta);
		Point_t p2 = coordinate_transform(p2_local,p0,theta);
		Point_t p3 = coordinate_transform(p3_local,p0,theta);
		Point_t p4 = coordinate_transform(p4_local,p0,theta);

        // search histogram in local frame
		std::vector<int> his_x(im->w, 0);
		std::vector<int> his_y(im->h, 0);
		for(int i = 0;i < edge; i=i+1)
			for(int j = 0; j<edge;j=j+1)
		{
			int x = -round(edge/2.0)+i;
			int y = -round(edge/2.0)+j;
            Point_t p_local_temp;
			p_local_temp.row = y;
			p_local_temp.col = x;

			Point_t p_global_temp = coordinate_transform(p_local_temp,p0,theta);
            if(check_coordinate(im,p_global_temp.col,p_global_temp.row))
			{
				if(check_color(im,p_global_temp.col,p_global_temp.row))
				{
                    his_x[i]++;
					his_y[j]++;
				}
			}
		}

		int max_id_x=0,max_x=0,max_id_y=0,max_y=0;

		for(int i = 0;i<his_x.size();i++)
		{
			if(his_x[i]>max_x)
			{
				max_x = his_x[i];
				max_id_x = i;
			}
		}

		for(int i = 0;i<his_y.size();i++)
		{
			if(his_y[i]>max_y)
			{
				max_y = his_y[i];
				max_id_y = i;
			}
		}

		Point_t best_gate_corner_local = {max_id_y-round(edge/2.0),max_id_x-round(edge/2.0)};
		Point_t best_gate_corner_global = coordinate_transform(best_gate_corner_local,p0,theta);
		if(check_coordinate(im,best_gate_corner_global.col,best_gate_corner_global.row))
			p0 = best_gate_corner_global;
		Polygon small_square(p1,p2,p3,p4);
//		small_square.draw_polygon(this->mat,cv::Scalar(100,100,100));
//		cv::circle(this->mat,cv::Point(p0.col,p0.row),1,cv::Scalar(0,255,0));
//		cv::imshow("debug hist",this->mat);
//		cv::waitKey(0);
        return small_square;
	}


	Point_t SnakeGateDetector::coordinate_transform(Point_t & p_local,Point_t & p_origin, double theta)
	{
		Point_t p_global;
		p_global.col = cos(theta)*p_local.col-sin(theta)*p_local.row;
		p_global.row = sin(theta)*p_local.col + cos(theta)*p_local.row;
		p_global.col = p_global.col + p_origin.col;
		p_global.row = p_global.row+ p_origin.row;
        return p_global;
	}


	bool SnakeGateDetector:: isColorBlock(image_t * im,Polygon & poly)
	{
        if(DEBUG_CHECK_COLOR_BLOCK)
		{
			poly.draw_polygon(mat,cv::Scalar(155,155,155));
		}

		Point_t p1 = poly.get_vertex(1);
		Point_t p2 = poly.get_vertex(2);
		Point_t p3 = poly.get_vertex(3);
		Point_t p4 = poly.get_vertex(4);

		struct Vector_t p1p2 = {p2.col-p1.col,p2.row-p1.row};
		struct Vector_t p1p3 = {p3.col-p1.col,p3.row-p1.row};
		struct Vector_t p2p3 = {p3.col-p2.col,p3.row-p2.row};
		struct Vector_t p2p4 = {p4.col-p2.col,p4.row-p2.row};

		struct Point_t P_random;
		int num_color_pix_in_polygon = 0;

		for (int i = 0;i<NUM_RANDOM_POINTS_IN_BLOCK;i++)
		{
			double u = (double) rand()/RAND_MAX;
			double v = (double) rand()/RAND_MAX;
			if(u+v>1)
			{
				u = 1-u;
				v = 1-v;
			}
			if((double) rand() / (RAND_MAX)<0.5)
			{
				// random points in triangle p1p2p3
				P_random.col = p1.col+round(u*p1p2.x)+ round(v*p1p3.x);
				P_random.row= p1.row+round(u*p1p2.y)+ round(v*p1p3.y);
                if(check_color(im,P_random.col,P_random.row)) num_color_pix_in_polygon++;
			}

			else
			{
				// random points in triangle p2p3p4
				P_random.col = p2.col + round(u * p2p3.x) + round(v * p2p4.x);
				P_random.row = p2.row + round(u * p2p3.y) + round(v * p2p4.y);
				if (check_color(im, P_random.col, P_random.row)) num_color_pix_in_polygon++;
			}


			if(DEBUG_CHECK_COLOR_BLOCK)
			{
				cv::circle(this->mat,cv::Point(P_random.col,P_random.row),1,cv::Scalar(255,0,0));
				cv::imshow("color block",this->mat);
				cv::waitKey(WAIT_KEY_TIME);
			}

		}

		if((double)num_color_pix_in_polygon/NUM_RANDOM_POINTS_IN_BLOCK>threshold_color_block)
		{
			//std::ostringstream debugMsg;
			//debugMsg << "color pixcel / num points "<< (double)num_color_pix_in_polygon/NUM_RANDOM_POINTS_IN_BLOCK<<std::endl;
            //debugMsg<< "color pixels" << num_color_pix_in_polygon<<std::endl;
			//Comm::print(debugMsg.str());
			return true;

		}
		else
			return false;
	}

void
SnakeGateDetector::
setYMax(int y_max) {
    this->y_max = y_max;
    std::ostringstream msg;
    msg << "Setting y_max to: " << y_max << std::endl ;
    Comm::print(msg.str());
}

void
SnakeGateDetector::
setYMin(int y_min) {
    this->y_min = y_min;
    std::ostringstream msg;
    msg << "Setting y_min to: " << y_min << std::endl ;
    Comm::print(msg.str());
}

void
SnakeGateDetector::
setCrMin(int cr_min) {
    this->cr_min = cr_min;
    std::ostringstream msg;
    msg << "Setting cr_min to: " << cr_min << std::endl ;
    Comm::print(msg.str());
}

void
SnakeGateDetector::
setCrMax(int cr_max) {
    this->cr_max = cr_max;
    std::ostringstream msg;
    msg << "Setting cr_max to: " << cr_max << std::endl ;
    Comm::print(msg.str());
}

void
SnakeGateDetector::
setCbMin(int cb_min) {
    this->cb_min = cb_min;
    std::ostringstream msg;
    msg << "Setting cb_min to: " << cb_min << std::endl ;
    Comm::print(msg.str());
}

void
SnakeGateDetector::
setCbMax(int cb_max) {
    this->cb_max = cb_max;
    std::ostringstream msg;
    msg << "Setting cb_max to: " << cb_max << std::endl ;
    Comm::print(msg.str());
}

void
SnakeGateDetector::
setMinLength(int min_length) {
    this->min_length = min_length;
    std::ostringstream msg;
    msg << "Setting min_length to: " << min_length << std::endl ;
    Comm::print(msg.str());
}

void
SnakeGateDetector::
setMinColorFitness(float min_color_fitness) {
    this->min_color_fitness = min_color_fitness;
    std::ostringstream msg;
    msg << "Setting min_color_fitness to: " << min_color_fitness << std::endl ;
    Comm::print(msg.str());
}

void
SnakeGateDetector::
setMaxColorFitnessCorner23(float max_color_fitness_corner23) {
    this->max_color_fitness_corner23 = max_color_fitness_corner23;
    std::ostringstream msg;
    msg << "Setting max_color_fitness_corner23 to: " << max_color_fitness_corner23 << std::endl ;
    Comm::print(msg.str());
}

void
SnakeGateDetector::
setThresholdColorBlock(float threshold_color_block) {
    this->threshold_color_block = threshold_color_block;
    std::ostringstream msg;
    msg << "Setting threshold color block to: " << threshold_color_block << std::endl ;
    Comm::print(msg.str());
}

void
SnakeGateDetector::
setUndistortionK(float k) {
    this->undistortion_k = k;
    std::ostringstream msg;
    msg << "Setting undistortion k to: " << this->undistortion_k << std::endl ;
    Comm::print(msg.str());
}

void
SnakeGateDetector::
setCx(int cx) {
    this->principal_cx = cx;
    std::ostringstream msg;
    msg << "Setting principal cx to: " << principal_cx << std::endl ;
    Comm::print(msg.str());
}

void
SnakeGateDetector::
setCy(int cy) {
    this->principal_cy = cy;
    std::ostringstream msg;
    msg << "Setting principal cy to: " << principal_cy << std::endl ;
    Comm::print(msg.str());
}

void
SnakeGateDetector::
setFocalLength(float f) {
    this->focal_length = f;
    std::ostringstream msg;
    msg << "Setting focal length to: " << focal_length << std::endl ;
    Comm::print(msg.str());
}

Polygon 
SnakeGateDetector::
getBestGate() {
	guard.lock();
	Polygon bestGateCopy(best_gate);
	guard.unlock();
	return bestGateCopy;
}

Polygon
SnakeGateDetector::
undistortion(Polygon &gate)
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


}
