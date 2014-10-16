
#include <stdio.h>
#include <unistd.h>

#include "ros/ros.h"
#include "scout_msgs/ScoutMotionSrv.h"
#include "scout_msgs/ScoutMotionMsg.h"
#include "sensor_msgs/Joy.h"

// message publishing is way faster than service call
#define USE_SRV 0
#define USE_MSG 1

int  vl = 0;
int  vr = 0;
bool en = false;  // NOTE: joystick control disabled by default
float joy_gain=20;

ros::ServiceClient motion;
scout_msgs::ScoutMotionSrv motion_req;
scout_msgs::ScoutMotionMsg motion_msg;
ros::Publisher pub;

void send_drive(void) {
#if USE_SRV
  // call service
  motion_req.request.enable = true;
  motion_req.request.velocity_left  = vl;
  motion_req.request.velocity_right = vr;
  motion.call(motion_req);
#endif

#if USE_MSG
  // publish message
  motion_msg.enable = true;
  motion_msg.velocity_left  = vl;
  motion_msg.velocity_right = vr;
  pub.publish(motion_msg);
#endif
}

void joy_cb(const sensor_msgs::Joy &msg) {
  // get values
  float vc = 15*msg.axes[1];
  float vd =  5*msg.axes[2];
  // compute wheel velocities
  vl = -joy_gain*(vc-vd);
  vr =  joy_gain*(vc+vd);
  joy_gain=joy_gain+5*msg.axes[5];
  if (msg.axes[4]>0) en=true;
  if (msg.axes[4]<0) en=false;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "keydrive");
  ros::NodeHandle n;

  // find motion service and topic
  motion = n.serviceClient<scout_msgs::ScoutMotionSrv>("/scout/motion");
  pub = n.advertise<scout_msgs::ScoutMotionMsg>("/scout/motion", 1);

  // start joystick callback
  ros::Subscriber sub_joy = n.subscribe("/joy", 1, joy_cb);

  // main loop
  ROS_INFO("Joystick driving node launched");
  ros::Rate loop(10);
  while (ros::ok()) {
    ros::spinOnce();
    if (en) send_drive();
    loop.sleep();
  }

  return 0;
}

// EOF
