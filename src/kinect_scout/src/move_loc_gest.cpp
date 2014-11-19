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
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "kinect_scout/gestures.hpp"

using namespace cv;

int main(int argc, char** argv) {
	ros::init(argc, argv, "move_loc_gest");
	ros::NodeHandle n;
	tf::TransformListener listener;
	ros::Rate rate(10.0);

	// Move Base
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);
	//wait for the action server to come up
	/*while(!ac.waitForServer(ros::Duration(5.0))){
	  ROS_INFO("Waiting for the move_base action server to come up");
	}*/
	move_base_msgs::MoveBaseGoal goal;

	// MAP
	Mat map;
	map = imread("/home/simon/ROS_WS/maps/final.pgm",
			CV_LOAD_IMAGE_GRAYSCALE);
	if (!map.data) {  // Check for invalid input
		std::cout << "Could not open or find the map" << std::endl;
		return -1;
	}
	tf::StampedTransform body_left_hand, body_right_hand, openni_right_hand;

	Gesture gesture, gesture_prev=NONE;
	int goto_location=0;
	cvNamedWindow("Map", CV_WINDOW_NORMAL);
	//SetWindowProperty("Map", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	int button=0;
	int prev_location=0;
	while (n.ok()&button!='q') {
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
		// Recognize gesture
		gesture=recognize_gesture(body_left_hand, body_right_hand);
		if (gesture_prev!=gesture) {
			switch (gesture) {
				case LEFT:
					goto_location=2;
					break;
				case RIGHT:
					goto_location=1;
					break;
				case UP_LEFT:
					goto_location=4;
					break;
				case UP_RIGHT:
					goto_location=3;
					break;
				case UP_BOTH:
					goto_location=0;
					break;
				case FORWARD_LEFT: case FORWARD_RIGHT:
					break;
				case FORWARD_BOTH:
					break;
				default: // NULL, no gesture recognized
					break;
			}
			gesture_prev=gesture;
		}
		// draw hand position onto map plane
		Mat map_draw;
		// Define 4 Robot Positions
		Point2f Goals[4]={Point2f(41,167),Point2f(320,192),Point2f(178,319),Point2f(178,31)};
		cvtColor(map, map_draw, CV_GRAY2RGB);
		// Draw goals
		for (int i=0; i<4;i++) {
			if (i==goto_location-1) {
				circle(map_draw, Goals[i], 8, Scalar(0,255,0),2);
			}
			else {
				circle(map_draw, Goals[i], 8, Scalar(255,0,0),2);
			}
		}
	    imshow("Map", map_draw);  // Show the image
		button=waitKey(30);
		if (goto_location!=0&(prev_location!=goto_location)) {
			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();
			goal.target_pose.pose.position.x = Goals[goto_location-1].x*0.05;
			goal.target_pose.pose.position.y = (map.size().height-Goals[goto_location-1].y)*0.05;
			goal.target_pose.pose.orientation.w = 1;
			ac.sendGoal(goal);
			ROS_INFO("Sending goal (%f,%f)!", goal.target_pose.pose.position.x,goal.target_pose.pose.position.y);
		}
		prev_location=goto_location;
		/*ac.waitForResult();
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		    ROS_INFO("Hooray, the base moved 1 meter forward");
		else
		    ROS_INFO("The base failed to move forward 1 meter for some reason");*/
		}

		return 0;
		}

