#include <ros/ros.h>
#include <math.h>
#include <time.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <scout_msgs/ScoutMotorsMsg.h>

#include <fix_angle.hpp>

#define BLUE "\033[22;34m" 
#define LIGHTBLUE "\033[01;34m"
#define ENDCOLOR "\033[01;37m"

// Scout calibration data
double LAMBDA_L;
double  LAMBDA_R;
double L;

bool USE_0_ORDER_INTEGR=true;

ros::Publisher pub_odometry;

double x, y, t;
ros::Time last_time;
int32_t last_ct[2];
bool first_time_msg=true;

std::string odometry_frame_id;
std::string base_frame_id;

bool debug;

void 
odometry(const scout_msgs::ScoutMotorsMsgConstPtr &msg){
	
	ROS_INFO_COND(debug, "%sScout Motors Msg received with a time shift of %s%f s%s",BLUE,LIGHTBLUE,(ros::Time::now()-msg->header.stamp).toSec(),ENDCOLOR);
	
	if(first_time_msg){
		last_ct[0]=msg->count_left;
		last_ct[1]=msg->count_right;
		last_time=msg->header.stamp;
		first_time_msg=false;
		return;
	}
	
	float dl = LAMBDA_L*(msg->count_left-last_ct[0]);
	float dr = LAMBDA_R*(msg->count_right-last_ct[1]);

	float ds, dt;
	
	//Calculate the pose off the robot
    if(USE_0_ORDER_INTEGR){
		//"""0th order integration"""
		ds = (dr+dl)/2;
		dt = (dr-dl)/L;
		x += ds*cos(t);
		y += ds*sin(t);
		t  = fix_angle(t+dt);
    }else{
		//"""1st order integration"""
		ds = (dr+dl)/2;     
		dt = (dr-dl)/L;
		if(dt!=0){
			x += ds*(sin(t+dt) - sin(t))/dt;
			y += ds*(cos(t) - cos(t+dt))/dt;
			t  = fix_angle(t+dt);
		}else{
			x += ds*cos(t);
			y += ds*sin(t);
		}
    }
       
    
    //first, we'll publish the transform over tf
    static tf::TransformBroadcaster broadcaster;
	tf::Quaternion q;
    q.setRPY(0.0, 0.0, t);
	tf::StampedTransform transform(tf::Transform(q, tf::Vector3(x,y,0.0)), msg->header.stamp, odometry_frame_id, base_frame_id);
    broadcaster.sendTransform(transform);
    
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = odometry_frame_id;
    odom.child_frame_id = base_frame_id;
    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    geometry_msgs::Quaternion quaternion_msg;
    tf::quaternionTFToMsg ( q, quaternion_msg);
    odom.pose.pose.orientation = quaternion_msg;   
    //set the velocity   
    odom.twist.twist.linear.x = ds/(msg->header.stamp.toSec()-last_time.toSec());
    odom.twist.twist.angular.z = dt/(msg->header.stamp.toSec()-last_time.toSec());
	    
    //publish the odom
	pub_odometry.publish(odom);
	
	//Save data for the next iter
	last_ct[0]=msg->count_left;
    last_ct[1]=msg->count_right;
	last_time=msg->header.stamp;

}

//~ void change_pos(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg){
	//~ 
	//~ // Every time we receve a change/reset on the estimation of the robot
	//~ // We reset the odometry tf to the origin 
	//~ 
	//~ //first, we'll publish the transform over tf
    //~ static tf::TransformBroadcaster broadcaster;
	//~ tf::Quaternion q;
    //~ tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
	//~ tf::StampedTransform transform(tf::Transform(q, tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y,0.0)), msg->header.stamp, odometry_frame_id, base_frame_id);
    //~ broadcaster.sendTransform(transform);
    //~ 
    //~ //next, we'll publish the odometry message over ROS
    //~ nav_msgs::Odometry odom;
    //~ odom.header.stamp = msg->header.stamp;
    //~ odom.header.frame_id = odometry_frame_id;
    //~ odom.child_frame_id = base_frame_id;
    //~ //set the position
    //~ odom.pose.pose.position.x = msg->pose.pose.position.x;
    //~ odom.pose.pose.position.y = msg->pose.pose.position.y;
    //~ odom.pose.pose.position.z = 0.0;
    //~ odom.pose.pose.orientation = msg->pose.pose.orientation;
    //~ //publish the odom
	//~ pub_odometry.publish(odom);
	//~ 
	//~ //Save data for the next iter
	//~ x=msg->pose.pose.position.x;
	//~ y=msg->pose.pose.position.y;
	//~ t=tf::getYaw(q);
//~ }

int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "odometry_node");
	ros::NodeHandle node_handler;
	ros::NodeHandle nh("~");
	
	nh.param("debug",debug,false);
	
	if (!(nh.getParam("LAMBDA_L", LAMBDA_L) && nh.getParam("LAMBDA_R",LAMBDA_R) && nh.getParam("L",L)) ){
		ROS_ERROR("Odometry calibration parameters not found! Check if they are defined in the launch file by the param tag.");
		ros::shutdown();
	}
	LAMBDA_L = 1/LAMBDA_L;
	LAMBDA_R = 1/LAMBDA_R;
	
	nh.param("base_frame_id",base_frame_id,std::string("base_frame"));
	nh.param("odometry_frame_id",odometry_frame_id,std::string("odometry_frame"));
	
	// Create a ROS subscribe for the input of the ticks
	ros::Subscriber sub1 = node_handler.subscribe< scout_msgs::ScoutMotorsMsg >("motors", 1, odometry);

	// Create a ROS publisher for the output of the odometry
	pub_odometry = node_handler.advertise< nav_msgs::Odometry >("odometry", 1);
	
	//~ ROS_INFO("Waiting 5s to allow all subscribers to connect...");
	//~ usleep(5*1000*1000);
	
	// Publish the tf for the fist time
	static tf::TransformBroadcaster broadcaster;
	tf::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
	tf::StampedTransform transform(tf::Transform(q, tf::Vector3(0.0,0.0,0.0)), ros::Time::now(), odometry_frame_id, base_frame_id);
    broadcaster.sendTransform(transform);
    
    // Publish the odometry for the fist time
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = odometry_frame_id;
    odom.child_frame_id = base_frame_id;   
    //set the position
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    geometry_msgs::Quaternion quaternion_msg;
    tf::quaternionTFToMsg ( q, quaternion_msg);
    odom.pose.pose.orientation = quaternion_msg;     
    //publish the odom
	pub_odometry.publish(odom);

	ROS_INFO("Odometry node running...");
  
	// Spin
	ros::spin ();
}
 
