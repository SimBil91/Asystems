/*
 * operation_modes.cpp
 *
 *  Created on: Nov 29, 2014
 *      Author: simon
 */
#include "kinect_scout/operation_modes.hpp"

extern ros::Time loc_start_time;
extern int scouty_pose[4];
extern int SPEECH;

int do_send_goal(move_base_msgs::MoveBaseGoal &goal, int fixed_angle, Point fixed_location, Mat map, Mat& map_draw, actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>& ac ) {
	// This function sends a goal

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
		ROS_INFO("Sending goal (%f,%f,%i)!", goal.target_pose.pose.position.x,goal.target_pose.pose.position.y,fixed_angle);
		// Ouput Goal sent message on screen
		string text="Goal sent!";
		Size textSize = getTextSize(text, FONT_HERSHEY_COMPLEX_SMALL,
									1, 2, 0);
		putText(map_draw, "Goal sent!", cv::Point((map.size().width-textSize.width)/2,(map.size().height+textSize.height)/2),
								FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0,0,255), 2, CV_AA);
		return 1; // Goal sent
}

Mat move_gestures(ros::NodeHandle& n, Gesture& gesture, vector<Status_message>& Status, ros::Publisher& chatter_pub, sound_play::SoundClient& sc) {
	geometry_msgs::Twist vel; // Set velocity values
	double linear,angular; // velocity parameters
	double scale_l=0.2,scale_a=0.4; // scale velocity
	Gesture gesture_prev=NONE;
	// Read first state image
	Mat img=imread(ros::package::getPath("kinect_scout")+"/img_scout/no.png",CV_LOAD_IMAGE_COLOR);
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
			exit(0);
		}
		// Set velocity suiting to Gesture command
		vel.linear.x=linear*scale_l;
		vel.angular.z=angular*scale_a;
		//Publish velocity data:
		chatter_pub.publish(vel);
	}
	return img;
}

Mat move_location(ros::NodeHandle& n, Mat& map, Gesture& gesture, vector<Status_message>& Status, tf::StampedTransform& openni_right_hand, int gesture_left, actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>& ac, move_base_msgs::MoveBaseGoal& goal) {

	// Define Kinect Matrices for Reprojection on image plane:
	Mat C = (Mat_<double>(3, 3) << 525.0, 0, 319.5, 0, 525.0, 239.5, 0, 0, 1);
	Mat t = (Mat_<double>(3, 1) << 0,0,0);
	Mat r = (Mat_<double>(3, 1) << 0,0,0);
	Mat d = (Mat_<double>(5, 1) << 0,0,0,0,0);
	Mat P = (Mat_<double>(3, 4) << 525.0, 0, 319.5, 0, 0, 525.0, 239.5, 0, 0, 0, 1, 0);

	// Copy map
	Mat map_draw;
	cvtColor(map, map_draw, CV_GRAY2RGB);
	Gesture gesture_prev=NONE;
	int fix_location=0;
	int send_goal=0;
	int goal_active=0;
	int fix_angle=0;
	int fixed_angle=0;
	int do_fix_angle=0;
	int do_fix_location=0;
	Point2f fixed_location;
	time_t start_time;
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
	// project 3D points to map area
	std::vector<cv::Point3f> worldPoints;
	std::vector<cv::Point2f> projectedPoints;
	worldPoints.push_back(
	Point3f(openni_right_hand.getOrigin().y(),
			openni_right_hand.getOrigin().z(),
			openni_right_hand.getOrigin().x()));
	// Project Points back to image plane
	projectPoints(worldPoints,r,t,C,d,projectedPoints);
	// Map projected point in order to smoothly navigate in the map, values obtained emperically
	projectedPoints[0].x=projectedPoints[0].x*(float(map.size().width+200)/(640))-float(map.size().width)/4;
	projectedPoints[0].y=map.size().height-projectedPoints[0].y*(float(map.size().height+200)/(480))+float(map.size().height)/4;
	// Fix maximum projection to edges of map
	if (projectedPoints[0].x>map.size().width) projectedPoints[0].x=map.size().width;
	if (projectedPoints[0].y>map.size().height) projectedPoints[0].y=map.size().height;
	if (projectedPoints[0].y<0) projectedPoints[0].y=0;

	// Transitions between states according to Gesture:
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

	// FSM
	if (fix_location==0&&gesture_left) {
		do_fix_location=1;
		fixed_location=projectedPoints[0];
	}
	else if (do_fix_location&&!gesture_left) {
		// Check if pixel is free space, otherwise do not allow fixing
		Scalar intensity = map.at<uchar>((uchar)fixed_location.y, (uchar)fixed_location.x);
		if (intensity.val[0]>=250)	fix_location=1;
	}
	else do_fix_location=0;

	// Draw hand position onto map plane
	if (fix_location) {
		circle(map_draw, fixed_location, 8, Scalar(0,255,0),2);
	}
	else circle(map_draw, projectedPoints[0], 8, Scalar(255,0,0),2);

	// Get angle and draw it
	if (fix_location) {
		// fix angle
		if (do_fix_location==0&&fix_angle==0&&gesture_left) {
			do_fix_angle=1;
			fixed_angle=-atan2((projectedPoints[0].y-fixed_location.y),(projectedPoints[0].x-fixed_location.x))/3.141592653*180;
			fix_angle=1;
		}
		else do_fix_angle=0;
		// get angle between projected Hand and fixed point and draw
		if (!fix_angle) {
			int angle=-atan2((projectedPoints[0].y-fixed_location.y),(projectedPoints[0].x-fixed_location.x))/3.141592653*180;
			drawArrow(map_draw, fixed_location, Scalar(255,0,0), angle, 7, 2);
		}
		else {
			drawArrow(map_draw, fixed_location, Scalar(0,255,0), fixed_angle, 7, 2);
			fix_angle=1;
		}
	}
	if (fix_location!=0&&fix_angle!=0&&send_goal&&!goal_active) {
		// Try to send a goal
		if ((clock()-start_time)/(CLOCKS_PER_SEC/2)>3) {
			goal_active=do_send_goal(goal, fixed_angle, fixed_location, map, map_draw, ac);
		}
		else {
			// Wait for countdown:
			std::stringstream ss;
			ss<<(int)(3-((clock()-start_time)/(CLOCKS_PER_SEC/2)));
			Size textSize = getTextSize(ss.str(), FONT_HERSHEY_COMPLEX_SMALL,4, 2, 0);
			// Print Countdown to screen
			putText(map_draw, ss.str(), cv::Point((map.size().width-textSize.width)/2,(map.size().height+textSize.height)/2),
					FONT_HERSHEY_COMPLEX_SMALL, 4, cvScalar(0,0,255), 2, CV_AA);
			goal_active=0;
		}
	}
	return map_draw;
}

Mat move_pre_location(ros::NodeHandle& n, Mat& map, Gesture& gesture, vector<Status_message>& Status, actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>& ac, move_base_msgs::MoveBaseGoal& goal) {
	// Copy map for processing
	Mat map_draw;
	cvtColor(map, map_draw, CV_GRAY2RGB);
	// Some definitions
	Gesture gesture_prev=NONE;
	int goto_location=0;
	int prev_location=0;
	int send_goal=0;
	int goal_active=0;
	time_t start_time;

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
	// Switch states according to gestures
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
	// Try to send goal
	if (goto_location!=0&&send_goal&&!goal_active) {
		if ((clock()-start_time)/(CLOCKS_PER_SEC/2)>3) {
			do_send_goal(goal,angles[goto_location-1],Goals[goto_location-1],map,map_draw,ac);
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

	return map_draw;

}

