/*
 * move_gestures.cpp
 *
 *  Created on: Oct 28, 2014
 *      Author: simon
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"
#include <math.h>
#include <sound_play/sound_play.h>

#define SPEECH 1

enum Gesture {NONE,LEFT,RIGHT,UP_LEFT,UP_RIGHT,FORWARD_LEFT,FORWARD_RIGHT, FORWARD_BOTH, UP_BOTH};

double compute_distance(tf::StampedTransform &frame1, tf::StampedTransform &frame2) {
	return sqrt(pow(frame1.getOrigin().x()-frame2.getOrigin().x(),2)+pow(frame1.getOrigin().y()-frame2.getOrigin().y(),2)+pow(frame1.getOrigin().z()-frame2.getOrigin().z(),2));
}

double compute_norm(tf::StampedTransform &frame1) {
	return sqrt(pow(frame1.getOrigin().x(),2)+pow(frame1.getOrigin().y(),2)+pow(frame1.getOrigin().z(),2));
}

tf::StampedTransform compute_diff(tf::StampedTransform &frame1, tf::StampedTransform &frame2) {
	tf::StampedTransform result;
	// apply rotation matrix to frame 1
	frame1.setRotation(frame1.getRotation());
	result.getOrigin()=frame2.getOrigin()-frame1.getOrigin();
	return result;
}

enum Gesture recognize_gesture(tf::StampedTransform &left_hand,tf::StampedTransform &right_hand) {
	Gesture gesture;
	int gesture_count=0;
	// Recognize gestures:
	// Set Threshold, Half of maximum length (Later replaced by calibration)
	double threshold=0.5;
	// Left Hand LEFT
	if (left_hand.getOrigin().x()>threshold&&left_hand.getOrigin().y()<threshold&&left_hand.getOrigin().z()<threshold&&compute_norm(right_hand)<threshold) {
		gesture=LEFT;
		gesture_count++;
	}
	// left Hand UP
	if (left_hand.getOrigin().y()>threshold&&left_hand.getOrigin().x()<threshold&&left_hand.getOrigin().z()<threshold&&compute_norm(right_hand)<threshold) {
		gesture=UP_LEFT;
		gesture_count++;
	}
	// left Hand in front
	if (left_hand.getOrigin().z()<-threshold&&left_hand.getOrigin().x()>-threshold&&left_hand.getOrigin().y()>-threshold&&compute_norm(right_hand)<threshold) {
		gesture=FORWARD_LEFT;
		gesture_count++;
	}
	// right Hand RIGHT
	if (right_hand.getOrigin().x()<-threshold&&right_hand.getOrigin().y()>-threshold&&right_hand.getOrigin().z()>-threshold&&compute_norm(left_hand)<threshold) {
		gesture=RIGHT;
		gesture_count++;
	}
	// right Hand UP
	if (right_hand.getOrigin().y()>threshold&&right_hand.getOrigin().x()<threshold&&right_hand.getOrigin().z()<threshold&&compute_norm(left_hand)<threshold) {
		gesture=UP_RIGHT;
		gesture_count++;
	}
	// right Hand in front
	if (right_hand.getOrigin().z()<-threshold&&right_hand.getOrigin().x()>-threshold&&right_hand.getOrigin().y()>-threshold&&compute_norm(left_hand)<threshold) {
		gesture=FORWARD_RIGHT;
		gesture_count++;
	}
	// Both Hands UP
	if (left_hand.getOrigin().y()>threshold&&left_hand.getOrigin().x()<threshold&&left_hand.getOrigin().z()<threshold&&right_hand.getOrigin().y()>threshold&&right_hand.getOrigin().x()<threshold&&right_hand.getOrigin().z()<threshold) {
		gesture=UP_BOTH;
		gesture_count++;
	}
	// Both Hands in front
	if (left_hand.getOrigin().z()<-threshold&&left_hand.getOrigin().x()>-threshold&&left_hand.getOrigin().y()>-threshold&&right_hand.getOrigin().z()<-threshold&&right_hand.getOrigin().x()>-threshold&&right_hand.getOrigin().y()>-threshold) {
		gesture=FORWARD_BOTH;
		gesture_count++;
	}
	// if multiple gestures found return no gesture
	if (gesture_count==1)
		return gesture;
	else
		return NONE;
}



int main(int argc, char** argv){
  ros::init(argc, argv, "move_gestures");

  ros::NodeHandle n;
  tf::TransformListener listener;
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Rate rate(10.0);
  geometry_msgs::Twist vel; // Set velocity values
  double linear,angular; // velocity parameters
  double scale_l=0.2,scale_a=0.4; // scale velocity
  Gesture gesture, gesture_prev=NONE;
  sound_play::SoundClient sc; // Object for playing sounds
  usleep(2000000);

  if (SPEECH) {
	  sc.startWave("/home/simon/ROS_WS/sounds/R2D2.wav");
	  usleep(1500000);
	  sc.say("Hi, I'm Scoutyi! Use gestures to move me!");
	  usleep(2000000);
  }

  while (n.ok()){
    tf::StampedTransform body_left_hand, body_right_hand;
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
				angular=1.0;
				if (SPEECH) sc.say("Left!");
				break;
			case RIGHT:
				ROS_INFO("GO_RIGHT!");
				angular=-1.0;
				if (SPEECH) sc.say("Right!");
				break;
			case UP_LEFT: case UP_RIGHT:
				ROS_INFO("GO_Back!");
				linear=-1.0;
				if (SPEECH) sc.say("Back!");
				break;
			case UP_BOTH:
				ROS_INFO("GO_Back_FAST!");
				linear=-1.5;
				if (SPEECH) sc.say("Fast Back!");
				break;
			case FORWARD_LEFT: case FORWARD_RIGHT:
				ROS_INFO("GO_FORWARD!");
				linear=1.0;
				if (SPEECH) sc.say("Forward!");
				break;
			case FORWARD_BOTH:
				ROS_INFO("GO_FORWARD_FAST!");
				linear=1.5;
				if (SPEECH) sc.say("Fast Forward!");
				break;
			default: // NULL, no gesture recognized
				ROS_INFO("STOP!");
				if (SPEECH) sc.say("Stop!");
		}
		gesture_prev=gesture;
    }
    // Set velocity suiting to Gesture command
    vel.linear.x=linear*scale_l;
    vel.angular.z=angular*scale_a;
    //ROS_INFO("vel:%f | ang:%f", vel.linear.x, vel.angular.z);
    chatter_pub.publish(vel);
    rate.sleep();
  }
  return 0;
};


