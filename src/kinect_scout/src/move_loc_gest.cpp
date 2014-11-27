/*
 * move_location.cpp
 *
 *  Created on: Nov 3, 2014
 *      Author: simon
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <math.h>
#include <iostream>
#include <time.h>
#include <string>
#include <sstream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "kinect_scout/gestures.hpp"
#include "kinect_scout/visualize.hpp"

using namespace cv;
int scouty_pose[4]={0,0,0,0}; // x,y,angle,update
ros::Time loc_start_time;
ros::Time odom_start_time;
ros::Time laser_start_time;

void check_odom(const nav_msgs::Odometry::ConstPtr& msg) {
	odom_start_time=ros::Time::now();
}

void check_laser(const sensor_msgs::LaserScan::ConstPtr& msg) {
	laser_start_time=ros::Time::now();
}

void amcl_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  loc_start_time=ros::Time::now();
  // Convert to pixel space
  double width=msg->pose.pose.position.x/0.05;
  double height=msg->pose.pose.position.y/0.05;
  tf::Quaternion quat;
  quat.setX(msg->pose.pose.orientation.x);
  quat.setY(msg->pose.pose.orientation.y);
  quat.setZ(msg->pose.pose.orientation.z);
  quat.setW(msg->pose.pose.orientation.w);
  double angle=tf::getYaw(quat)*180/3.141592653;
  scouty_pose[0]=(int)width; scouty_pose[1]=(int)height; scouty_pose[2]=(int)angle;
  scouty_pose[3]=1;
  ROS_INFO("%f,%f,%f",width,height,angle);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "move_loc_gest");
	ros::NodeHandle n;
	tf::TransformListener listener;
	ros::Rate rate(30.0);
	// Move Base
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);
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

	Gesture gesture;
	Gesture gesture_prev=NONE;
	int goto_location=0;
	cvNamedWindow("Map", CV_WINDOW_NORMAL);
	//tftcvSetWindowProperty("Map", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	int button=0;
	int prev_location=0;
	int send_goal=0;
	int goal_active=0;
	ros::Subscriber pos;
	ros::Subscriber laser;
	ros::Subscriber odometry;
	time_t start_time;
	while (n.ok()&button!='q') {
		// Get Scoutys position
		pos = n.subscribe("amcl_pose", 1000, amcl_pose);
		laser = n.subscribe("odom", 1000, check_odom);
		odometry = n.subscribe("/laser_range_finder/scan", 1000, check_laser);
		Mat map_draw;
		cvtColor(map, map_draw, CV_GRAY2RGB);
		// Vector of Status messages:
		vector<Status_message> Status;
		try {
			listener.lookupTransform("/torso_1", "/left_hand_1", ros::Time(0),
					body_left_hand);
			listener.lookupTransform("/torso_1", "/right_hand_1", ros::Time(0),
					body_right_hand);
			push_message(Status,"Tracker",1);
		}

		catch (tf::TransformException &ex) {
			//ROS_ERROR("%s",ex.what());
			ROS_INFO("No User detected!");
			// send status message
			push_message(Status,"Tracker",0);
			//ros::Duration(1.0).sleep();
		}
		// Check if action Server is connected
		if (ac.isServerConnected()) {
			push_message(Status,"MB_Server",1);
		}
		else {
			push_message(Status,"MB_Server",0);
		}
		if (goal_active) {
			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				ROS_INFO("The goal was reached!");
				push_message(Status,"MB_GOAL",1);
			}
			else if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE){
				push_message(Status,"MB_GOAL",2);
			}
			else {
				push_message(Status,"MB_GOAL",0);
			}
		}
		// Check for messages
		// Check for amcl pose
		if ((ros::Time::now().toSec()-loc_start_time.toSec())<3.0) {
			push_message(Status,"Localization",1);
			// Draw Scouty to map
			if (scouty_pose[3]) {
				scouty_pose[1]=map.size().height-scouty_pose[1];
				scouty_pose[3]=0;
			}
			circle(map_draw, Point(scouty_pose[0],scouty_pose[1]), 4, Scalar(0,0,0),2);
			drawArrow(map_draw, Point(scouty_pose[0],scouty_pose[1]), Scalar(0,0,0), scouty_pose[2], 7, 2);
		}
		else {
			push_message(Status,"Localization",0);
		}
		// Check for odometry messages
		if ((ros::Time::now().toSec()-odom_start_time.toSec())<1.0) {
			push_message(Status,"Odometry",1);
		}
		else {
			push_message(Status,"Odometry",0);
		}
		// Check for laser messages
		if ((ros::Time::now().toSec()-laser_start_time.toSec())<1.0) {
			push_message(Status,"Laser",1);
		}
		else {
			push_message(Status,"Laser",0);
		}
		// Recognize gesture
		gesture=recognize_gesture(body_left_hand, body_right_hand);
		//gesture=GOD;
		if (button=='s') gesture=GOD;
		if (button=='u') gesture=UP_BOTH;

		if (gesture_prev!=gesture) {
			switch (gesture) {
				case LEFT:
					if (!goal_active) goto_location=2;
					break;
				case RIGHT:
					if (!goal_active) goto_location=1;
					break;
				case UP_LEFT:
					if (!goal_active) goto_location=4;
					break;
				case UP_RIGHT:
					if (!goal_active) goto_location=3;
					break;
				case UP_BOTH:
					goto_location=0;
					send_goal=0;
					if (goal_active) {
						// Cancel current goals
						ac.cancelAllGoals();
						goal_active=0;
					}
					break;
				case GOD:
					if (!send_goal) start_time=clock();
					send_goal=1;
					break;
				default:
					send_goal=0;// NULL, no gesture recognized
			}
			gesture_prev=gesture;
		}
		// Define 4 Robot Positions
		Point2f Goals[4]={Point2f(41,167),Point2f(320,192),Point2f(178,319),Point2f(178,31)};
		double angles[4]={0,140,270,20};
		// Draw goals
		for (int i=0; i<4;i++) {
			if (i==goto_location-1) {
				circle(map_draw, Goals[i], 8, Scalar(0,255,0),2);
				drawArrow(map_draw, Goals[i], Scalar(0,255,0), angles[i], 7, 2);
			}
			else {
				if (!goal_active) {
					circle(map_draw, Goals[i], 8, Scalar(255,0,0),2);
					drawArrow(map_draw, Goals[i], Scalar(255,0,0), angles[i], 7, 2);
				}
			}
		}
	  	if (goto_location!=0&&send_goal&&!goal_active) {
			if ((clock()-start_time)/(CLOCKS_PER_SEC/2)>3) {
				//send_goal=0;
				// Compute quaterion
				tf::Quaternion quat=tf::createQuaternionFromYaw(angles[goto_location-1]*3.141592653/180);
				goal.target_pose.header.frame_id = "map";
				goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose.position.x = Goals[goto_location-1].x*0.05;
				goal.target_pose.pose.position.y = (map.size().height-Goals[goto_location-1].y)*0.05;
				goal.target_pose.pose.orientation.w = quat.getW();
				goal.target_pose.pose.orientation.x = quat.getX();
				goal.target_pose.pose.orientation.y = quat.getY();
				goal.target_pose.pose.orientation.z = quat.getZ();
				ac.sendGoal(goal);
				goal_active=1;
				ROS_INFO("Sending goal (%f,%f,%f)!", goal.target_pose.pose.position.x,goal.target_pose.pose.position.y,angles[goto_location-1]);
				//ac.waitForResult();
				string text="Goal sent!";
				Size textSize = getTextSize(text, FONT_HERSHEY_COMPLEX_SMALL,
				                            1, 2, 0);
				putText(map_draw, "Goal sent!", cv::Point((map.size().width-textSize.width)/2,(map.size().height+textSize.height)/2),
						  				FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0,0,255), 2, CV_AA);
			}
			else {
		  		std::stringstream ss;
		  		ss<<(int)(3-((clock()-start_time)/(CLOCKS_PER_SEC/2)));
		  		Size textSize = getTextSize(ss.str(), FONT_HERSHEY_COMPLEX_SMALL,4, 2, 0);
		  		putText(map_draw, ss.str(), cv::Point((map.size().width-textSize.width)/2,(map.size().height+textSize.height)/2),
		  				FONT_HERSHEY_COMPLEX_SMALL, 4, cvScalar(0,0,255), 2, CV_AA);
			}
		}
		prev_location=goto_location;
		// Add status to Image
		add_status(map_draw, 150, Status);
		imshow("Map", map_draw);  // Show the image
		button=waitKey(30);
		ros::spinOnce();
		//rate.sleep();
	}
	return 0;
}

;
