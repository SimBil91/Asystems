/*
 * scouty_interface.cpp
 *
 *  Created on: Nov 29, 2014
 *      Author: simon
 */

#include "kinect_scout/scouty_interface.hpp"

// ---------FSMs----------
// enclosing FSM switching between the modes
FSM_Operation_Mode fsm_operation_mode=INIT;
// 3 FSMs representing the state of each mode
// ---------FSMs----------

// ----------Global variables-----------
//for Scouty Pose and checking if valid data is received.
int scouty_pose[4]={0,0,0,0}; // x,y,angle,update
ros::Time loc_start_time;
ros::Time odom_start_time;
ros::Time laser_start_time;
int SPEECH = 1;
// ----------Global variables-----------

// Functions valid for all different modes, they provide feedback about the current state of the sytem
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

void check_status(ros::NodeHandle& n, ros::Subscriber& pos, ros::Subscriber& laser, ros::Subscriber& odometry, vector<Status_message>& Status) {
	// Get Scoutys position
	pos = n.subscribe("amcl_pose", 1000, amcl_pose);
	// Get Laser/Odometry information
	laser = n.subscribe("odom", 1000, check_odom);
	odometry = n.subscribe("/laser_range_finder/scan", 1000, check_laser);
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
}

void get_transforms(tf::TransformListener& listener, tf::StampedTransform& body_left_hand, tf::StampedTransform& body_right_hand, tf::StampedTransform& openni_right_hand, vector<Status_message>& Status) {
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
}

int main(int argc, char** argv) {

	// Initialize ROS
	ros::init(argc, argv, "scouty_interface");
	ros::NodeHandle n;
	ros::Rate rate(30.0);
	// Create Transform Listener for Gesture Recognition
	tf::TransformListener listener;
	// Create transforms for left and right hand relativ to the torso and right hand relative to optical center
	tf::StampedTransform body_left_hand, body_right_hand, openni_right_hand;
	// Read map of 5th floor
	Mat map,interface;
	map = imread(ros::package::getPath("move_scout")+"/maps/final.pgm",
				CV_LOAD_IMAGE_GRAYSCALE);
	if (!map.data) {  // Check for invalid input
		std::cout << "Could not open or find the map" << std::endl;
		exit(0);
	}
	// Initialize interface
	interface=map;
	// Define Movebase actionlib Client
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);
	move_base_msgs::MoveBaseGoal goal;
	// Create a new window to show the interface
	cvNamedWindow("Scouty_Interface", CV_WINDOW_NORMAL);
	//cvSetWindowProperty("Scouty_Interface", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	// Create a Gesture enum, and one for the Left Hand especially
	Gesture gesture=NONE;
	bool left_hand;
	// Create Subscribers to check the system state
	ros::Subscriber pos;
	ros::Subscriber laser;
	ros::Subscriber odometry;
	// Create Publisher to publish velocity commands
	ros::Publisher veloc_pub=n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	// Create sound_play object
	sound_play::SoundClient sc;
	// Create button, which is filled by OpenCV loop
	int button=0;
	// Time when switch gesture is made
	ros::Time switch_time;
	bool time_set=0;
	while(ros::ok()&(button!='q')) {
		// ---These Variables and functions are shared by all Operation modes---
		// Generate Status vector
		vector<Status_message> Status;
		// Get the transforms if possible and add Status to vector
		get_transforms(listener,body_left_hand,body_right_hand,openni_right_hand,Status);
		// Recognize gesture
		gesture=recognize_gesture(body_left_hand, body_right_hand);
		int left_hand=gesture_is_left(body_right_hand);
		// Check system status
		check_status(n,pos,laser,odometry,Status);
		// Define Keyboard values to emulate gestures
		if (button=='s') gesture=GOD;
		if (button=='u') gesture=UP_BOTH;
		if (button=='n') gesture=CORNER_RIGHT;
		if (button=='f') left_hand=1; else left_hand=0;
		// If the gesture is the menu switch CORNER_RIGHT --> start clock
		if (gesture==CORNER_RIGHT) {if (!time_set) switch_time=ros::Time::now();time_set=1;}
		else time_set=0;

		if (fsm_operation_mode==INIT) {
			// Initialisation, only run at the beginning
			if (SPEECH) {  sc.say("Hi, I'm Scoutyi! Ready to serve your needs!");}
			fsm_operation_mode=MOVE_GESTURES;
		}
		else if (fsm_operation_mode==MOVE_GESTURES) {
			// Transition if gesture is hold for 2 seconds
			if (gesture==CORNER_RIGHT&&(ros::Time::now().toSec()-switch_time.toSec())>=2) {fsm_operation_mode=MOVE_PRE_LOCATIONS;ROS_INFO("MODE:PRE_LOCATIONS"); time_set=0;}
			// Move Scouty with Kinect gestures
			interface=move_gestures(n,gesture,Status,veloc_pub,sc);
		}
		else if (fsm_operation_mode==MOVE_PRE_LOCATIONS) {
			if (gesture==CORNER_RIGHT&&(ros::Time::now().toSec()-switch_time.toSec())>=2) {fsm_operation_mode=MOVE_LOCATIONS;ROS_INFO("MODE:LOCATIONS"); time_set=0;}
			// Send Scouty to predefined Locations
			interface=move_pre_location(n,map,gesture,Status,ac,goal);
		}
		else if (fsm_operation_mode==MOVE_LOCATIONS) {
			// Transition:
			if (gesture==CORNER_RIGHT&&(ros::Time::now().toSec()-switch_time.toSec())>=2) {fsm_operation_mode=MOVE_GESTURES;ROS_INFO("MODE:GESTURES"); time_set=0;}
			// Send Scouty to arbitrary Locations indicated
			interface=move_location(n,map,gesture,Status,openni_right_hand,left_hand,ac,goal);
		}

		// Add status to Image
		add_status(interface, 150, Status);
		// Show interface
		imshow("Scouty_Interface", interface);
		button=waitKey(30);
		ros::spinOnce();
	}
	return 0;
}

