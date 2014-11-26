/*
 * visualize.hpp
 *
 *  Created on: Nov 24, 2014
 *      Author: simon
 */

#ifndef VISUALIZE_HPP_
#define VISUALIZE_HPP_

#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>

using namespace cv;
void drawArrow(Mat& img, Point p, const Scalar& color, double angle=0, int arrowMagnitude=9, int thickness=1, int lineType=8, int shift=0);

void add_status(Mat& img,int width, vector<string> information);

#endif /* VISUALIZE_HPP_ */
