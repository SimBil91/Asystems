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

void push_message(vector<Status_message>& Status, string message, int active) {
	Status_message st_message;
	st_message.message=message;
	st_message.active=active;
	Status.push_back(st_message);
}

void add_status(Mat& img,int width, vector<Status_message> information) {
	// create black container and add text to img
	Mat bar = Mat().zeros(img.size().height,width, img.type());
	Status_message current_status;
	int pad_height=10;
	int pad_width=10;
	double font_size=4;
	Size textSize;
	while (!information.empty()) {
		current_status=information.back();
		textSize = getTextSize(current_status.message, FONT_HERSHEY_COMPLEX_SMALL,
				font_size,1 , 0);

		if ((textSize.width>(width-20))) {
			font_size=font_size*0.9;
			if (font_size<=0.1) information.back().message="2long";
		}
		else if (pad_height>bar.size().height-20) {
			break;
		}
		else {
		information.pop_back();
		pad_height+=textSize.height;
		Scalar color;
		if (current_status.active==0) color=CV_RGB(255,0,0);
		else if (current_status.active==1) color=CV_RGB(0,255,0);
		else if (current_status.active==2) color=CV_RGB(255,255,0);
		putText(bar, current_status.message, cv::Point(pad_width,pad_height),
												FONT_HERSHEY_COMPLEX_SMALL, font_size, color, 1, CV_AA);
		pad_height+=10;
		font_size=4;
		}
	}
	// append image to the right
	cv::hconcat(img,bar,img);

}
