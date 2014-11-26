/*
 * move_location.cpp
 *
 *  Created on: Nov 3, 2014
 *      Author: simon
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "kinect_scout/gestures.hpp"

using namespace cv;

int main(int argc, char** argv) {
	ros::init(argc, argv, "move_location");
	ros::NodeHandle n;
	tf::TransformListener listener;
	ros::Rate rate(10.0);

	// Define Kinect Matrices:
	Mat C = (Mat_<double>(3, 3) << 525.0, 0, 319.5, 0, 525.0, 239.5, 0, 0, 1);
	Mat t = (Mat_<double>(3, 1) << 0,0,0);
	Mat r = (Mat_<double>(3, 1) << 0,0,0);
	Mat d = (Mat_<double>(5, 1) << 0,0,0,0,0);
	Mat P = (Mat_<double>(3, 4) << 525.0, 0, 319.5, 0, 0, 525.0, 239.5, 0, 0, 0, 1, 0);
	// READ AND DISPLAY MAP
	Mat map;
	map = imread("/home/simon/ROS_WS/maps/final.pgm",
			CV_LOAD_IMAGE_GRAYSCALE);
	if (!map.data) {  // Check for invalid input
		std::cout << "Could not open or find the map" << std::endl;
		return -1;
	}
	tf::StampedTransform body_left_hand, body_right_hand, openni_right_hand;
	int button=0;
	cvNamedWindow("Map", CV_WINDOW_NORMAL);
	cvSetWindowProperty("Map", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	Gesture gesture, gesture_prev=NONE;
	int fix_location=0;
	Point2f fixed_location;
	while (n.ok()&(button!='q')) {
		try {
			listener.lookupTransform("/torso_1", "/left_hand_1", ros::Time(0),
					body_left_hand);
			listener.lookupTransform("/torso_1", "/right_hand_1", ros::Time(0),
					body_right_hand);
			listener.lookupTransform("/openni_depth_frame", "/left_hand_1",
					ros::Time(0), openni_right_hand);
		} catch (tf::TransformException &ex) {
			//ROS_ERROR("%s",ex.what());
			ROS_INFO("No User detected!");
			ros::Duration(1.0).sleep();
		}
		// project 3D points to map area
		std::vector<cv::Point3f> worldPoints;
		std::vector<cv::Point2f> projectedPoints;
		//Mat objectPoint = (Mat_<double>(4, 1) << openni_right_hand.getOrigin().y(),openni_right_hand.getOrigin().z(),openni_right_hand.getOrigin().x(),1);
		worldPoints.push_back(
				Point3f(openni_right_hand.getOrigin().y(),
						openni_right_hand.getOrigin().z(),
						openni_right_hand.getOrigin().x()));
		//Mat result=P*objectPoint;
		//result=result/result.at<double>(2,0);
		//std::cout << result << std::endl;
		// Project Points back to image plane
		projectPoints(worldPoints,r,t,C,d,projectedPoints);
		projectedPoints[0].x=projectedPoints[0].x*(float(map.size().width+200)/(640))-float(map.size().width)/4;
		projectedPoints[0].y=map.size().height-projectedPoints[0].y*(float(map.size().height+200)/(480))+float(map.size().height)/4;
		if (projectedPoints[0].x>map.size().width) projectedPoints[0].x=map.size().width;
		if (projectedPoints[0].y>map.size().height) projectedPoints[0].y=map.size().height;
		if (projectedPoints[0].y<0) projectedPoints[0].y=0;
		std::cout << projectedPoints << "\n";

		// Recognize gesture
		gesture=recognize_gesture(body_left_hand, body_right_hand);
		if (gesture_prev!=gesture) {
			switch (gesture) {
				case UP_BOTH:
					fix_location=0;
					break;
				default:

					break;
			}
			gesture_prev=gesture;
		}
		double threshold=0.5;
		if (fix_location==0&&(body_right_hand.getOrigin().x()<-threshold)) {
			fix_location=1;
			fixed_location=projectedPoints[0];
		}
		// draw hand position onto map plane
		Mat map_draw;
		cvtColor(map, map_draw, CV_GRAY2RGB);
		if (fix_location)
			circle(map_draw, fixed_location, 8, Scalar(0,255,0),2);
		else
			circle(map_draw, projectedPoints[0], 8, Scalar(255,0,0),2);
		imshow("Map", map_draw);  // Show the image
		button=waitKey(30);
		}
		return 0;
		}

