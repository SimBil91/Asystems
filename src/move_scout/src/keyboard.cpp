#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <signal.h>
#include <termios.h>
#include <time.h>
#include <iostream>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

int kfd = 0;
struct termios cooked, raw;
float scale_l=0.2,scale_a=0.4;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Rate loop_rate(50);
  signal(SIGINT,quit);
  char c;
  bool dirty=false;
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the Scout 1.");
  float linear,angular;
  double last_send;
  while (ros::ok())
  {
	  // get the next event from the keyboard
	      if(read(kfd, &c, 1) < 0)
	      {
	        perror("read():");
	        exit(-1);
	      }
	      linear=angular=0;
	      ROS_DEBUG("value: 0x%02X\n", c);

	      switch(c)
	      {
	        case KEYCODE_L:
	          ROS_DEBUG("LEFT");
	          angular=1.0;
	          dirty = true;
	          break;
	        case KEYCODE_R:
	          ROS_DEBUG("RIGHT");
	          angular = -1.0;
	          dirty = true;
	          break;
	        case KEYCODE_U:
	          ROS_DEBUG("UP");
	          linear = 1.0;
	          dirty = true;
	          break;
	        case KEYCODE_D:
	          ROS_DEBUG("DOWN");
	          linear = -1.0;
	          dirty = true;
	          break;
	      }

    geometry_msgs::Twist vel;
    vel.linear.x=linear*scale_l;
    vel.angular.z=angular*scale_a;
    if(dirty ==true){
         ROS_INFO("vel:%f | ang:%f", vel.linear.x, vel.angular.z);
         chatter_pub.publish(vel);
         last_send=clock();
         dirty=false;
    }
    if ((clock()-last_send)*(1000/CLOCKS_PER_SEC)>=400) {
    	vel.linear.x=0;
    	vel.angular.z=0;
    	chatter_pub.publish(vel);
    }
    std::cout << (clock()-last_send)*(1000/CLOCKS_PER_SEC);
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}



