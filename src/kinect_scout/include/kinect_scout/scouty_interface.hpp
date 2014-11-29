/*
 * scouty_interface.hpp
 *
 *  Created on: Nov 29, 2014
 *      Author: simon
 */

#ifndef SCOUTY_INTERFACE_HPP_
#define SCOUTY_INTERFACE_HPP_

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
#include "kinect_scout/operation_modes.hpp"

using namespace cv;

enum FSM_Operation_Mode {INIT, MOVE_GESTURES, MOVE_PRE_LOCATIONS, MOVE_LOCATIONS};

void check_odom(const nav_msgs::Odometry::ConstPtr& msg);

void check_laser(const sensor_msgs::LaserScan::ConstPtr& msg);

void amcl_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

void check_status(ros::NodeHandle& n, ros::Subscriber& pos, ros::Subscriber& laser, ros::Subscriber& odometry, vector<Status_message>& Status);

void get_transforms(tf::TransformListener& listener, tf::StampedTransform& body_left_hand, tf::StampedTransform& body_right_hand, tf::StampedTransform& openni_right_hand, vector<Status_message>& Status);


#endif /* SCOUTY_INTERFACE_HPP_ */
