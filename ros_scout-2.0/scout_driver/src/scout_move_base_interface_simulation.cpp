#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <scout_msgs/ScoutMotionMsg.h>

#include <math.h>
#include <algorithm> //for std::min

#define VMAX 0.5 //  max wheel speed
#define AMAX 0.6 // max angular speed

using namespace ros;


class ScoutMoveBaseInterfaceSimulation
{    
  public:
    ScoutMoveBaseInterfaceSimulation () :
      nh_(),
      cmd_vel_pub_ ( nh_.advertise<geometry_msgs::Twist> ("cmd_vel", 1 ) ),
      scout_motion_sub_ ( nh_.subscribe ( "scout/motion", 1, &ScoutMoveBaseInterfaceSimulation::ScoutMotionCallback, this ) )
    {
      if (!(nh_.getParam("LAMBDA_L", lambda_l_) && nh_.getParam("LAMBDA_R", lambda_r_) && nh_.getParam("L",l_)) )
      {
        ROS_FATAL("Odometry calibration parameters not found!");
        ros::shutdown();
      }
    }

  private:
    void ScoutMotionCallback (const scout_msgs::ScoutMotionMsg& msg)
    {
      int32_t velocity_left = msg.velocity_left;
      int32_t velocity_right = msg.velocity_right;
      
//       double v = ( velocity_left - velocity_right ) / 2;
//       double w = ( velocity_right - velocity_left ) / l_;
//       
// //       double factor = 1.0;
// //       if ( abs ( w ) >= AMAX )
// //       {
// //         factor = AMAX / abs ( w );
// //       }
// //       
// //       if ( abs ( velocity_left ) > VMAX )
// //       {
// //         factor = std::min ( factor, VMAX / abs ( velocity_left ) );
// //       }
// //       if ( abs ( velocity_right ) > VMAX )
// //       {
// //         factor = std::min ( factor, VMAX / abs ( velocity_right ) );
// //       }
// //       
// //       double v_left = 60 * factor * velocity_left * lambda_l_ / 2064.0;
// //       double v_right = 60 * factor * velocity_right * lambda_r_ / 2064.0;
// //       
// //       v = ( v_left - v_right ) / 2;
// //       w = ( v_right - v_left ) / l_;
// //       
//       geometry_msgs::Twist output;
//       
//       output.linear.x = v / 7000;
//       output.linear.y = 0;
//       output.linear.z = 0;
//       
//       output.angular.x = 0;
//       output.angular.y = 0;
//       output.angular.z = w;
//       
//       cmd_vel_pub_.publish ( output );
      
      double v_l = ( velocity_left * 2064.0 ) / ( 60 * lambda_l_ );
      double v_r = ( velocity_right * 2064.0 ) / ( 60 * lambda_r_ );
      
      double v = v_l + 1 / 2 * ( v_r - v_l );
      double w = ( v_r - v_l ) / l_;
      
      geometry_msgs::Twist output;
      
      output.linear.x = v;
      output.linear.y = 0;
      output.linear.z = 0;
      
      output.angular.x = 0;
      output.angular.y = 0;
      output.angular.z = w;
      
      cmd_vel_pub_.publish ( output );
    }

    NodeHandle nh_;
    
    Publisher cmd_vel_pub_;
    Subscriber scout_motion_sub_;
    double l_, lambda_l_, lambda_r_;
};


/// Main loop
int main (int argc, char** argv)
{
  init (argc, argv, "scout_move_base_interface_simulation");
  
  ScoutMoveBaseInterfaceSimulation smbis;
  
  spin();
  
  return 0;
}
