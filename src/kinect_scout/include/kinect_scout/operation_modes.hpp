/*
 * operation_modes.hpp
 *
 *  Created on: Nov 29, 2014
 *      Author: simon
 */

#ifndef OPERATION_MODES_HPP_
#define OPERATION_MODES_HPP_

#include "kinect_scout/scouty_interface.hpp"

enum FSM_Pre_Locations {WAIT_FOR_GESTURE, FIX_LOCATION, WAIT_FOR_CONFIRMATION, SEND_GOAL, WAIT_FOR_RESULT, CANCEL_GOAL, RESAMPLE_PARTICLES, GOAL_REACHED, GOAL_ABORTED};
enum FSM_Locations {lWAIT_FOR_GESTURE, lFIX_LOCATION,lWAIT_FOR_GESTURE2,fWAIT_FOR_RELEASE, lFIX_ANGLE, lWAIT_FOR_CONFIRMATION, lSEND_GOAL, lWAIT_FOR_RESULT, lCANCEL_GOAL, lRESAMPLE_PARTICLES, lGOAL_REACHED, lGOAL_ABORTED};
enum FSM_Move {mWAIT_FOR_GESTURE, mSEND_VELOCITY};

Mat move_location(ros::NodeHandle& n, Mat& map, Gesture& gesture, vector<Status_message>& Status, tf::StampedTransform& openni_right_hand, int gesture_left, actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>& ac,  move_base_msgs::MoveBaseGoal& goal,sound_play::SoundClient& sc);
int do_send_goal(move_base_msgs::MoveBaseGoal &goal, int fixed_angle, Point fixed_location, Mat map, Mat& map_draw, actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>& ac);
Mat move_pre_location(ros::NodeHandle& n, Mat& map, Gesture& gesture, vector<Status_message>& Status, actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>& ac, move_base_msgs::MoveBaseGoal& goal,sound_play::SoundClient& sc);
Mat move_gestures(ros::NodeHandle& n, Gesture& gesture, vector<Status_message>& Status, ros::Publisher& chatter_pub, sound_play::SoundClient& sc,Gesture& gesture_prev);
#endif /* OPERATION_MODES_HPP_ */
