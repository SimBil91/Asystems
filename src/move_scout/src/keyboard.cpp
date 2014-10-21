#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <signal.h>
#include <termios.h>
#include <time.h>
#include <iostream>
#include <sound_play/sound_play.h>

#define SPEECH 1

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
  ros::init(argc, argv, "keyboard");
  ros::NodeHandle n;
  sound_play::SoundClient sc; // Object for playing sounds
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
  puts("Use arrow keys to move the Scout 1. \nPress 'u' to increase speed and 'd' to decrease it.");
  float linear,angular;
  double last_send;
  usleep(1000000);

  if (SPEECH) {
	  sc.startWave("/home/simon/ROS_WS/src/move_scout/src/R2D2.wav");
	  usleep(1500000);
	  sc.say("Hi, I'm Scoutyi! Use your arrow keys to move me!");
  }

  while (ros::ok()&&c!='q')
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
	          if (SPEECH) sc.say("Left!");
	          break;
	        case KEYCODE_R:
	          ROS_DEBUG("RIGHT");
	          angular = -1.0;
	          dirty = true;
	          if (SPEECH) sc.say("Right!");
	          break;
	        case KEYCODE_U:
	          ROS_DEBUG("UP");
	          linear = 1.0;
	          dirty = true;
	          if (SPEECH) sc.say("Forward!");
	          break;
	        case KEYCODE_D:
	          ROS_DEBUG("DOWN");
	          linear = -1.0;
	          dirty = true;
	          if (SPEECH) sc.say("Backg!");
	          break;
	        case 'u':
			  ROS_INFO("SPEED++");
			  scale_l+=0.1;
			  scale_a+=0.1;
			  if (SPEECH) sc.say("going faster");
			  break;
	        case 'd':
			  ROS_INFO("SPEED--");
			  scale_l+=0.1;
			  scale_a+=0.1;
			  if (SPEECH) sc.say("going slower");
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
    ros::spinOnce();
    loop_rate.sleep();

  }
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);

  return 0;
}



