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

using namespace cv;

int main(int argc, char** argv) {
	ros::init(argc, argv, "move_location");
	ros::NodeHandle n;
	tf::TransformListener listener;
	ros::Rate rate(10.0);
	// READ AND DISPLAY MAP
	Mat map;
	map = imread("/home/simon/ROS_WS/maps/map2_ed_cr.pgm",
			CV_LOAD_IMAGE_GRAYSCALE);
	if (!map.data) {  // Check for invalid input
		std::cout << "Could not open or find the map" << std::endl;
		return -1;
	}
	tf::StampedTransform body_left_hand, body_right_hand, openni_right_hand;

	while (n.ok()) {
		try {
			listener.lookupTransform("/torso_1", "/left_hand_1", ros::Time(0),
					body_left_hand);
			listener.lookupTransform("/torso_1", "/right_hand_1", ros::Time(0),
					body_right_hand);
		} catch (tf::TransformException &ex) {
			//ROS_ERROR("%s",ex.what());
			ROS_INFO("No User detected!");
			ros::Duration(1.0).sleep();
		}

		Mat map_draw;
		cvtColor(map, map_draw, CV_GRAY2RGB);
		circle(map_draw, Point(2,5), 8, Scalar(255,0,0));
		imshow("Map: 5th floor", map_draw);  // Show the image
		waitKey(30);
		}
		return 0;
		}

