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
extern FSM_Pre_Locations fsm_pre_locations;
extern FSM_Locations fsm_locations;
extern int goto_location;
extern time_t start_time1;
extern cv::Point2f fixed_location;
extern int fixed_angle;
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

Mat move_gestures(ros::NodeHandle& n, Gesture& gesture, vector<Status_message>& Status, ros::Publisher& chatter_pub, sound_play::SoundClient& sc,Gesture& gesture_prev) {
	geometry_msgs::Twist vel; // Set velocity values
	double linear,angular; // velocity parameters
	double scale_l=0.2,scale_a=0.4; // scale velocity
	// Read first state image
	Mat img=imread(ros::package::getPath("kinect_scout")+"/img_scout/no.png",CV_LOAD_IMAGE_COLOR);
	// Only check for new action if new gesture is recognized!
	linear=angular=0;
		switch (gesture) {
			case LEFT:
				ROS_INFO("GO_LEFT!");
				angular=-1.0;
				if (SPEECH&&(gesture_prev!=gesture)) sc.say("Left!");
				img = imread(ros::package::getPath("kinect_scout")+"/img_scout/right.png",
						CV_LOAD_IMAGE_COLOR);
				break;
			case RIGHT:
				ROS_INFO("GO_RIGHT!");
				angular=1.0;
				if (SPEECH&&(gesture_prev!=gesture)) sc.say("Right!");
				img = imread(ros::package::getPath("kinect_scout")+"/img_scout/left.png",
										CV_LOAD_IMAGE_COLOR);
				break;
			case UP_LEFT: case UP_RIGHT:
				ROS_INFO("GO_Forward!");
				linear=1.0;
				if (SPEECH&&(gesture_prev!=gesture)) sc.say("Forward!");
				img = imread(ros::package::getPath("kinect_scout")+"/img_scout/forward.png",
										CV_LOAD_IMAGE_COLOR);
				break;
			case UP_BOTH:
				ROS_INFO("GO_Forward_FAST!");
				linear=1.5;
				if (SPEECH&&(gesture_prev!=gesture)) sc.say("Fast Forward!");
				img = imread(ros::package::getPath("kinect_scout")+"/img_scout/fast_forward.png",
										CV_LOAD_IMAGE_COLOR);
				break;
			case DOWN_LEFT: case DOWN_RIGHT:
				ROS_INFO("GO_BACK!");
				linear=-1.0;
				if (SPEECH&&(gesture_prev!=gesture)) sc.say("Back!");
				img = imread(ros::package::getPath("kinect_scout")+"/img_scout/back.png",
										CV_LOAD_IMAGE_COLOR);
				break;
			case DOWN_BOTH:
				ROS_INFO("GO_Back_FAST!");
				linear=-1.5;
				if (SPEECH&&(gesture_prev!=gesture)) sc.say("Fast Back!");
				img = imread(ros::package::getPath("kinect_scout")+"/img_scout/fast_back.png",
										CV_LOAD_IMAGE_COLOR);
				break;
			default: // NULL, no gesture recognized
				ROS_INFO("STOP!");
				linear=0;
				angular=0;
				if (SPEECH&&(gesture_prev!=gesture)) sc.say("Stop!");
				img = imread(ros::package::getPath("kinect_scout")+"/img_scout/no.png",
										CV_LOAD_IMAGE_COLOR);
		}
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
	time_t start_time;
	// Check if action Server is connected
	if (ac.isServerConnected()) {
		push_message(Status,"MB_Server",1);
	}
	else {
		push_message(Status,"MB_Server",0);
	}
	if (fsm_locations==lWAIT_FOR_RESULT) {
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

	// ------ FSM Location ----
	if (fsm_locations==lWAIT_FOR_GESTURE) {
		fixed_location=Point2f(0,0);
		fixed_angle=450; // larger than 360;
		// If gesture equals
		if (gesture_left==1) {
			Scalar intensity = map.at<uchar>((uchar)projectedPoints[0].y, (uchar)projectedPoints[0].x);
			if (intensity.val[0]>=250) {
				fsm_locations=lFIX_LOCATION;
			}
		}
	}
	else if (fsm_locations==lFIX_LOCATION) {
		// Check if pixel is free space, otherwise do not allow fixing
		fixed_location=projectedPoints[0];
		if (!gesture_left) fsm_locations=lWAIT_FOR_GESTURE2;
		else fixed_location=Point2f(0,0);
	}
	else if (fsm_locations==lWAIT_FOR_GESTURE2) {
		if (gesture_left) {
			fsm_locations=lFIX_ANGLE;
		}
		if (gesture==UP_BOTH) fsm_locations=lWAIT_FOR_GESTURE;
	}
	else if (fsm_locations==lFIX_ANGLE) {
		// fix angle
		fixed_angle=-atan2((projectedPoints[0].y-fixed_location.y),(projectedPoints[0].x-fixed_location.x))/3.141592653*180;
		fsm_locations=lWAIT_FOR_CONFIRMATION;
	}
	else if (fsm_locations==lWAIT_FOR_CONFIRMATION) {
		if (gesture==UP_BOTH) {
			fsm_locations=lWAIT_FOR_GESTURE;
		}
		if (gesture==GOD) {
			if ((clock()-start_time1)/(CLOCKS_PER_SEC/2)>3) {
				fsm_locations=lSEND_GOAL;
			}
			else {
				std::stringstream ss;
				ss<<(int)(3-((clock()-start_time1)/(CLOCKS_PER_SEC/2)));
				Size textSize = getTextSize(ss.str(), FONT_HERSHEY_COMPLEX_SMALL,4, 2, 0);
				putText(map_draw, ss.str(), cv::Point((map.size().width-textSize.width)/2,(map.size().height+textSize.height)/2),
						FONT_HERSHEY_COMPLEX_SMALL, 4, cvScalar(0,0,255), 2, CV_AA);
			}
		}
		else start_time1=clock();
	}
	else if (fsm_locations==lSEND_GOAL) {
		do_send_goal(goal,fixed_angle,fixed_location,map,map_draw,ac);
		fsm_locations=lWAIT_FOR_RESULT;
	}
	else if (fsm_locations==lWAIT_FOR_RESULT) {
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO("The goal was reached!");
			push_message(Status,"MB_GOAL",1);
			fsm_locations=lGOAL_REACHED;
		}
		else if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE){
			push_message(Status,"MB_GOAL",2);
		}
		else {
			push_message(Status,"MB_GOAL",0);
			fsm_locations=lRESAMPLE_PARTICLES;
		}
		if (gesture==UP_BOTH) {
			fsm_locations=lCANCEL_GOAL;
		}
	}
	else if (fsm_locations==lRESAMPLE_PARTICLES) {
		std_srvs::Empty::Request req;
		std_srvs::Empty::Response res;
		ros::service::call("global_localization",req,res);
		ROS_INFO("Scouty lost. Resample particles...");
		fsm_locations=lGOAL_ABORTED;

	}
	else if (fsm_locations==lCANCEL_GOAL) {
		// Cancel current goals
		ac.cancelAllGoals();
		fsm_locations=lWAIT_FOR_GESTURE;
	}
	else if (fsm_locations==lGOAL_REACHED){
		if(gesture==UP_BOTH) fsm_locations=lWAIT_FOR_GESTURE;
	}
	else if (fsm_locations==lGOAL_ABORTED){
		if(gesture==UP_BOTH) fsm_locations=lWAIT_FOR_GESTURE;
	}

	// ------ END FSM -----
	// Transitions between states according to Gesture:

	// Draw hand position onto map plane
	if (fixed_location!=Point2f(0,0)) {
		circle(map_draw, fixed_location, 8, Scalar(0,255,0),2);
	}
	else circle(map_draw, projectedPoints[0], 8, Scalar(255,0,0),2);

	// Get angle and draw it
	if (fixed_location!=Point2f(0,0)) {
		// get angle between projected Hand and fixed point and draw
		if (fixed_angle==450) {
			int angle=-atan2((projectedPoints[0].y-fixed_location.y),(projectedPoints[0].x-fixed_location.x))/3.141592653*180;
			drawArrow(map_draw, fixed_location, Scalar(255,0,0), angle, 7, 2);
		}
		else {
			drawArrow(map_draw, fixed_location, Scalar(0,255,0), fixed_angle, 7, 2);
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

	// Check if action Server is connected
	if (ac.isServerConnected()) {
		push_message(Status,"MB_Server",1);
	}
	else {
		push_message(Status,"MB_Server",0);
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


	// Define 4 Robot Positions
	Point2f Goals[4]={Point2f(41,167),Point2f(320,192),Point2f(178,319),Point2f(178,31)};
	double angles[4]={0,140,270,20};


	// -----FSM PRE_LOCATIONS-----
	if (fsm_pre_locations==WAIT_FOR_GESTURE) {
		goto_location=0;
		// If gesture equals
		if (gesture==LEFT||gesture==RIGHT||gesture==UP_RIGHT||gesture==DOWN_LEFT) {
			fsm_pre_locations=FIX_LOCATION;
		}
	}
	if (fsm_pre_locations==FIX_LOCATION) {
		switch (gesture) {
			case LEFT:
				 goto_location=2;
				 fsm_pre_locations=WAIT_FOR_CONFIRMATION;
				break;
			case RIGHT:
				 goto_location=1;
				 fsm_pre_locations=WAIT_FOR_CONFIRMATION;
				break;
			case DOWN_LEFT:
				 goto_location=3;
				 fsm_pre_locations=WAIT_FOR_CONFIRMATION;
				break;
			case UP_RIGHT:
				 goto_location=4;
				 fsm_pre_locations=WAIT_FOR_CONFIRMATION;
				break;
			default:
				break;
		}
	}
	else if (fsm_pre_locations==WAIT_FOR_CONFIRMATION) {
		if (gesture==UP_BOTH) {
			fsm_pre_locations=WAIT_FOR_GESTURE;
		}
		if (gesture==GOD) {
			if ((clock()-start_time1)/(CLOCKS_PER_SEC/2)>3) {
				fsm_pre_locations=SEND_GOAL;
			}
			else {
				std::stringstream ss;
				ss<<(int)(3-((clock()-start_time1)/(CLOCKS_PER_SEC/2)));
				Size textSize = getTextSize(ss.str(), FONT_HERSHEY_COMPLEX_SMALL,4, 2, 0);
				putText(map_draw, ss.str(), cv::Point((map.size().width-textSize.width)/2,(map.size().height+textSize.height)/2),
						FONT_HERSHEY_COMPLEX_SMALL, 4, cvScalar(0,0,255), 2, CV_AA);
			}
		}
		else start_time1=clock();
	}
	else if (fsm_pre_locations==SEND_GOAL) {
		do_send_goal(goal,angles[goto_location-1],Goals[goto_location-1],map,map_draw,ac);
		fsm_pre_locations=WAIT_FOR_RESULT;
	}
	else if (fsm_pre_locations==WAIT_FOR_RESULT) {
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO("The goal was reached!");
			push_message(Status,"MB_GOAL",1);
			fsm_pre_locations=GOAL_REACHED;
		}
		else if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE){
			push_message(Status,"MB_GOAL",2);
		}
		else {
			push_message(Status,"MB_GOAL",0);
			fsm_pre_locations=RESAMPLE_PARTICLES;
		}
		if (gesture==UP_BOTH) {
			fsm_pre_locations=CANCEL_GOAL;
		}
	}
	else if (fsm_pre_locations==RESAMPLE_PARTICLES) {
		std_srvs::Empty::Request req;
		std_srvs::Empty::Response res;
		ros::service::call("global_localization",req,res);
		ROS_INFO("Scouty lost. Resample particles...");
		fsm_pre_locations=GOAL_ABORTED;

	}
	else if (fsm_pre_locations==CANCEL_GOAL) {
		// Cancel current goals
		ac.cancelAllGoals();
		fsm_pre_locations=WAIT_FOR_GESTURE;
	}
	else if (fsm_pre_locations==GOAL_REACHED){
		if(gesture==UP_BOTH) fsm_pre_locations=WAIT_FOR_GESTURE;
	}
	else if (fsm_pre_locations==GOAL_ABORTED){
		if(gesture==UP_BOTH) fsm_pre_locations=WAIT_FOR_GESTURE;
	}
	// ----- END FSM
	// Draw goals
	for (int i=0; i<4;i++) {
		if (i==goto_location-1) {
			circle(map_draw, Goals[i], 8, Scalar(0,255,0),2);
			drawArrow(map_draw, Goals[i], Scalar(0,255,0), angles[i], 7, 2);
		}
		else {
			if (fsm_pre_locations!=WAIT_FOR_RESULT) {
				circle(map_draw, Goals[i], 8, Scalar(255,0,0),2);
				drawArrow(map_draw, Goals[i], Scalar(255,0,0), angles[i], 7, 2);
			}
		}
	}
	std::cout<<"State:"<<fsm_pre_locations<<std::endl;
	return map_draw;

}

