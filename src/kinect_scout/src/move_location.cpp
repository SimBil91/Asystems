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
#include <ros/package.h>
#include <math.h>
#include <iostream>
#include <string>
#include <time.h>
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
	ros::init(argc, argv, "move_location");
	ros::NodeHandle n;
	tf::TransformListener listener;
	ros::Rate rate(30.0);
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);
	move_base_msgs::MoveBaseGoal goal;
	// Define Kinect Matrices:
	Mat C = (Mat_<double>(3, 3) << 525.0, 0, 319.5, 0, 525.0, 239.5, 0, 0, 1);
	Mat t = (Mat_<double>(3, 1) << 0,0,0);
	Mat r = (Mat_<double>(3, 1) << 0,0,0);
	Mat d = (Mat_<double>(5, 1) << 0,0,0,0,0);
	Mat P = (Mat_<double>(3, 4) << 525.0, 0, 319.5, 0, 0, 525.0, 239.5, 0, 0, 0, 1, 0);
	// READ AND DISPLAY MAP
	Mat map;
	map = imread(ros::package::getPath("move_scout")+"/maps/final.pgm",
			CV_LOAD_IMAGE_GRAYSCALE);
	if (!map.data) {  // Check for invalid input
		std::cout << "Could not open or find the map" << std::endl;
		return -1;
	}
	tf::StampedTransform body_left_hand, body_right_hand, openni_right_hand;
	cvNamedWindow("Map", CV_WINDOW_NORMAL);
	cvSetWindowProperty("Map", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	Gesture gesture;
	Gesture gesture_prev=NONE;
	int fix_location=0;
	int button=0;
	int send_goal=0;
	int goal_active=0;
	int fix_angle=0;
	int fixed_angle=0;
	int do_fix_angle=0;
	int do_fix_location=0;
	Point2f fixed_location;
	ros::Subscriber pos;
	ros::Subscriber laser;
	ros::Subscriber odometry;
	time_t start_time;
	while (n.ok()&(button!='q')) {
		// Get Scoutys position
		pos = n.subscribe("amcl_pose", 1000, amcl_pose);
		laser = n.subscribe("odom", 1000, check_odom);
		odometry = n.subscribe("/laser_range_finder/scan", 1000, check_laser);
		// Copy map
		Mat map_draw;
		cvtColor(map, map_draw, CV_GRAY2RGB);
		// Generate Status vector
		vector<Status_message> Status;
		try {
			listener.lookupTransform("/torso_1", "/left_hand_1", ros::Time(0),
					body_left_hand);
			listener.lookupTransform("/torso_1", "/right_hand_1", ros::Time(0),
					body_right_hand);
			listener.lookupTransform("/openni_depth_frame", "/left_hand_1",
					ros::Time(0), openni_right_hand);
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
			std::cout<< (ros::Time::now().toSec()-loc_start_time.toSec());
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
		if (button=='s') gesture=GOD;
		if (button=='u') gesture=UP_BOTH;
		if (gesture_prev!=gesture) {
			switch (gesture) {
				case UP_BOTH:
					fix_location=0;
					do_fix_location=0;
					send_goal=0;
					do_fix_angle=0;
					fix_angle=0;
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
					send_goal=0;
					break;
			}
			gesture_prev=gesture;
		}
		double threshold=0.5;
		// Fix location with left arm
		if (fix_location==0&&(body_right_hand.getOrigin().x()<-threshold&&body_right_hand.getOrigin().y()>-threshold&&body_right_hand.getOrigin().z()>-threshold)) {
			do_fix_location=1;
			fixed_location=projectedPoints[0];
		}
		else if (do_fix_location&&!(body_right_hand.getOrigin().x()<-threshold&&body_right_hand.getOrigin().y()>-threshold&&body_right_hand.getOrigin().z()>-threshold)) {
			// Check if pixel is free space, otherwise do not allow fixing
			Scalar intensity = map.at<uchar>((uchar)fixed_location.y, (uchar)fixed_location.x);
			if (intensity.val[0]>=250)	fix_location=1;
		}
		else do_fix_location=0;
		if (button=='f') {
			Scalar intensity = map.at<uchar>((uchar)projectedPoints[0].y, (uchar)projectedPoints[0].x);
			if (intensity.val[0]>=250)	fix_location=1;
			fixed_location=projectedPoints[0];
		}
		// draw hand position onto map plane
		if (fix_location) {
			circle(map_draw, fixed_location, 8, Scalar(0,255,0),2);
		}
		else circle(map_draw, projectedPoints[0], 8, Scalar(255,0,0),2);
		// Get angle and draw it
		if (fix_location) {
			// fix angle
			if (do_fix_location==0&&fix_angle==0&&(body_right_hand.getOrigin().x()<-threshold&&body_right_hand.getOrigin().y()>-threshold&&body_right_hand.getOrigin().z()>-threshold)) {
				do_fix_angle=1;
				fixed_angle=atan2((projectedPoints[0].y-fixed_location.y),(projectedPoints[0].x-fixed_location.x))/3.141592653*180;
				fix_angle=1;
			}
			else do_fix_angle=0;
			if (button=='d') {fix_angle=1;fixed_angle=atan2((projectedPoints[0].y-fixed_location.y),(projectedPoints[0].x+fixed_location.x))*180/3.141592653;}
    		// get angle between projected Hand and fixed point and draw
			if (!fix_angle) {
				int angle=atan2((projectedPoints[0].y-fixed_location.y),(projectedPoints[0].x-fixed_location.x))/3.141592653*180;
				drawArrow(map_draw, fixed_location, Scalar(255,0,0), angle, 7, 2);
			}
			else {
				drawArrow(map_draw, fixed_location, Scalar(0,255,0), fixed_angle, 7, 2);
				fix_angle=1;
			}
		}

		if (fix_location!=0&&fix_angle!=0&&send_goal&&!goal_active) {
			if ((clock()-start_time)/(CLOCKS_PER_SEC/2)>3) {
				//send_goal=0;
				// Compute quaterion
				tf::Quaternion quat=tf::createQuaternionFromYaw(fixed_angle*3.141592653/180);
				goal.target_pose.header.frame_id = "map";
				goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose.position.x = fixed_location.x*0.05;
				goal.target_pose.pose.position.y = (map.size().height-fixed_location.y)*0.05;
				goal.target_pose.pose.orientation.w = quat.getW();
				goal.target_pose.pose.orientation.x = quat.getX();
				goal.target_pose.pose.orientation.y = quat.getY();
				goal.target_pose.pose.orientation.z = quat.getZ();
				ac.sendGoal(goal);
				goal_active=1;
				ROS_INFO("Sending goal (%f,%f,%i)!", goal.target_pose.pose.position.x,goal.target_pose.pose.position.y,fixed_angle);
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
		// Add status to Image
		add_status(map_draw, 150, Status);
		imshow("Map", map_draw);  // Show the image
		button=waitKey(30);
		ros::spinOnce();
		//rate.sleep();
	}
	return 0;
}

