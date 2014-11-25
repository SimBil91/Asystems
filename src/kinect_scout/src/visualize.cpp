/*
 * visualize.cpp
 *
 *  Created on: Nov 24, 2014
 *      Author: simon
 */

#include "kinect_scout/visualize.hpp"
using namespace cv;
void drawArrow(Mat& img, Point p, const Scalar& color, double angle, int arrowMagnitude, int thickness, int lineType, int shift)
{
    //Draw the principle line
	//Calculate second point out of angle and length information
    const double PI = 3.141592653;
	Point2f q;
	angle=angle*PI/180.0;
	q.y=-sin(angle)*arrowMagnitude*4+p.y;
	q.x=cos(angle)*arrowMagnitude*4+p.x;
	line(img, p, q, color, thickness, lineType, shift);
    //compute the coordinates of the first segment
    p.x = (int) ( q.x -  arrowMagnitude * cos(angle + (PI)/4));
    p.y = (int) ( q.y +  arrowMagnitude * sin(angle + (PI)/4));
    //Draw the first segment
    line(img, p, q, color, thickness, lineType, shift);
    //compute the coordinates of the second segment
    p.x = (int) ( q.x -  arrowMagnitude * cos((double)angle - (PI)/4));
    p.y = (int) ( q.y +  arrowMagnitude * sin((double)angle - (PI)/4));
    //Draw the second segment
    line(img, p, q, color, thickness, lineType, shift);
}

void add_status(Mat& img,int width, vector<string> information) {
	// append image to the right
	Size size;
	cv::hconcat(img, Mat().zeros(img.size().height,width, img.type()), img);
}
