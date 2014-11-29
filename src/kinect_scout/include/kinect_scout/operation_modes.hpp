/*
 * operation_modes.hpp
 *
 *  Created on: Nov 29, 2014
 *      Author: simon
 */

#ifndef OPERATION_MODES_HPP_
#define OPERATION_MODES_HPP_

#include "kinect_scout/scouty_interface.hpp"

Mat move_location(ros::NodeHandle& n, Mat& map, Gesture& gesture, vector<Status_message>& Status, tf::StampedTransform& openni_right_hand, int gesture_left, actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>& ac,  move_base_msgs::MoveBaseGoal& goal);
int do_send_goal(move_base_msgs::MoveBaseGoal &goal, int fixed_angle, Point fixed_location, Mat map, Mat& map_draw, actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>& ac);
Mat move_pre_location(ros::NodeHandle& n, Mat& map, Gesture& gesture, vector<Status_message>& Status, actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>& ac, move_base_msgs::MoveBaseGoal& goal);
Mat move_gestures(ros::NodeHandle& n, Gesture& gesture, vector<Status_message>& Status, ros::Publisher& chatter_pub, sound_play::SoundClient& sc);
#endif /* OPERATION_MODES_HPP_ */
