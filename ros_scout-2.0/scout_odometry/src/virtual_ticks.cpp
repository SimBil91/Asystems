#include <ros/ros.h>
#include <time.h>
#include <scout_msgs/ScoutMotorsMsg.h>

ros::Publisher pub;


int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "virtual_ticks");
	ros::NodeHandle node_handler;
	ros::NodeHandle nh("~");
		
	// Create a ROS publisher for the output of the odometry
	pub = node_handler.advertise< scout_msgs::ScoutMotorsMsg >("/scout/motors", 1);
	
	scout_msgs::ScoutMotorsMsg msg;
	
	float rate=38.2;
	ros::Rate loop_rate(rate);

	msg.count_left=0;
	msg.count_right=0;

	while (ros::ok()){
		
		ros::spinOnce();
		
		msg.header.stamp=ros::Time::now();
		msg.count_left=msg.count_left-547;
		msg.count_right=msg.count_right+547;
		
		pub.publish(msg);
		
		loop_rate.sleep();
	}
	
	return 0;
}
 
