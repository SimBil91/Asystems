#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <scout_msgs/ScoutMotionMsg.h>

#include <math.h>
#include <algorithm> //for std::min

#define VMAX 0.5 //  max wheel speed
#define AMAX 0.6 // max angular speed

using namespace ros;


class ScoutMoveBaseInterface
{    
  public:
    ScoutMoveBaseInterface () :
      nh_(),
      cmd_vel_sub_ (nh_.subscribe ("cmd_vel", 1, &ScoutMoveBaseInterface::cmdVelCallback, this)),
      scout_motion_pub_ (nh_.advertise<scout_msgs::ScoutMotionMsg> ("scout/motion",1))
    {
      if (!(nh_.getParam("LAMBDA_L", lambda_l_) && nh_.getParam("LAMBDA_R", lambda_r_) && nh_.getParam("L",l_)) )
      {
        ROS_FATAL("Odometry calibration parameters not found!");
	ros::shutdown();
      }
    }

  private:
    void cmdVelCallback (const geometry_msgs::TwistConstPtr& msg)
    {
      double v = msg->linear.x;
      double w = msg->angular.z;

      double v_left = v - 0.5*l_*w;
      double v_right = v + 0.5*l_*w; ///Angular speed in move_base is defined in the opposite direction than it is in scout_navigation.
      
      ///limit angular and wheel speeds
      double factor = 1.0;
      if(abs(w) >= AMAX)
      {
        factor = AMAX/abs(w);
      }
      
      if(abs(v_left) > VMAX)
      {
        factor = std::min(factor, VMAX/abs(v_left));
      }
      if(abs(v_right) > VMAX)
      {
        factor = std::min(factor, VMAX/abs(v_right));
      }

      v_left = 60*factor*v_left*lambda_l_/2064.0;
      v_right = 60*factor*v_right*lambda_r_/2064.0;

      scout_msgs::ScoutMotionMsg output;
      output.enable = true;
      output.velocity_left = round(v_left);
      output.velocity_right = round(v_right);

      scout_motion_pub_.publish(output);
    }

    NodeHandle nh_;
    
    Subscriber cmd_vel_sub_;
    Publisher scout_motion_pub_;
    double l_, lambda_l_, lambda_r_;
};


/// Main loop
int main (int argc, char** argv)
{
  init (argc, argv, "scout_move_base_interface");
  
  ScoutMoveBaseInterface smbi;
  
  spin();
  
  return 0;
}
