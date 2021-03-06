/*
 * gestures.cpp
 *
 *  Created on: Oct 28, 2014
 *      Author: simon
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"
#include <math.h>
#include <sound_play/sound_play.h>
#include <iostream>
#include "kinect_scout/gestures.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <ros/package.h>
#include <opencv2/imgproc/imgproc.hpp>

int SPEECH = 1;
using namespace cv;
int main(int argc, char** argv){
  ros::init(argc, argv, "move_gestures");

  ros::NodeHandle n;
  tf::TransformListener listener;
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Rate rate(10.0);
  geometry_msgs::Twist vel; // Set velocity values
  double linear,angular; // velocity parameters
  double scale_l=0.2,scale_a=0.4; // scale velocity
  cvNamedWindow("Map", CV_WINDOW_NORMAL);
  cvSetWindowProperty("Map", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
  Gesture gesture=NONE;
  Gesture gesture_prev=NONE;
  sound_play::SoundClient sc; // Object for playing sounds
  usleep(2000000);
  if (SPEECH) {
	  sc.startWave("/home/simon/ROS_WS/sounds/R2D2.wav");
	  usleep(1500000);
	  sc.say("Hi, I'm Scoutyi! Use gestures to move me!");
	  usleep(2000000);
  }
  tf::StampedTransform body_left_hand, body_right_hand;
  Mat img=imread(ros::package::getPath("kinect_scout")+"/img_scout/no.png",
			CV_LOAD_IMAGE_COLOR);
  int button=0;
  while (n.ok()&&button!='q'){

    try{
      listener.lookupTransform("/torso_1", "/left_hand_1",
                                       ros::Time(0), body_left_hand);
      listener.lookupTransform("/torso_1", "/right_hand_1",
                                       ros::Time(0), body_right_hand);
    }
    catch (tf::TransformException &ex) {
      //ROS_ERROR("%s",ex.what());
      ROS_INFO("No User detected!");
      ros::Duration(1.0).sleep();
    }
    gesture=recognize_gesture(body_left_hand, body_right_hand);
    // Only one command allowed at the same time!
    // Only check for new action if new gesture is recognized!
    if (gesture_prev!=gesture) {
    	linear=angular=0;
		switch (gesture) {
			case LEFT:
				ROS_INFO("GO_LEFT!");
				angular=-1.0;
				if (SPEECH) sc.say("Left!");
				img = imread(ros::package::getPath("kinect_scout")+"/img_scout/left.png",
						CV_LOAD_IMAGE_COLOR);
				break;
			case RIGHT:
				ROS_INFO("GO_RIGHT!");
				angular=1.0;
				if (SPEECH) sc.say("Right!");
				img = imread(ros::package::getPath("kinect_scout")+"/img_scout/right.png",
										CV_LOAD_IMAGE_COLOR);
				break;
			case UP_LEFT: case UP_RIGHT:
				ROS_INFO("GO_Forward!");
				linear=1.0;
				if (SPEECH) sc.say("Forward!");
				img = imread(ros::package::getPath("kinect_scout")+"/img_scout/forward.png",
										CV_LOAD_IMAGE_COLOR);
				break;
			case UP_BOTH:
				ROS_INFO("GO_Forward_FAST!");
				linear=-1.5;
				if (SPEECH) sc.say("Fast Forward!");
				img = imread(ros::package::getPath("kinect_scout")+"/img_scout/fast_forward.png",
										CV_LOAD_IMAGE_COLOR);
				break;
			case DOWN_LEFT: case DOWN_RIGHT:
				ROS_INFO("GO_BACK!");
				linear=1.0;
				if (SPEECH) sc.say("Back!");
				img = imread(ros::package::getPath("kinect_scout")+"/img_scout/back.png",
										CV_LOAD_IMAGE_COLOR);
				break;
			case DOWN_BOTH:
				ROS_INFO("GO_Back_FAST!");
				linear=1.5;
				if (SPEECH) sc.say("Fast Back!");
				img = imread(ros::package::getPath("kinect_scout")+"/img_scout/fast_back.png",
										CV_LOAD_IMAGE_COLOR);
				break;
			default: // NULL, no gesture recognized
				ROS_INFO("STOP!");
				if (SPEECH) sc.say("Stop!");
				img = imread(ros::package::getPath("kinect_scout")+"/img_scout/no.png",
										CV_LOAD_IMAGE_COLOR);
		}
		gesture_prev=gesture;
		if (!img.data) {  // Check for invalid input
			std::cout << "Could not open or find the image" << std::endl;
			return -1;
		}
    }
    std::cout << "Left_Arm:" << compute_norm(body_left_hand) << "\n";
    std::cout << "Right_Arm:" << compute_norm(body_right_hand) << "\n";
    // Set velocity suiting to Gesture command
    vel.linear.x=linear*scale_l;
    vel.angular.z=angular*scale_a;
    //ROS_INFO("vel:%f | ang:%f", vel.linear.x, vel.angular.z);
    chatter_pub.publish(vel);
    imshow("Map", img);  // Show the image
    button=waitKey(30);
    ros::spinOnce();
  }
  return 0;
}

