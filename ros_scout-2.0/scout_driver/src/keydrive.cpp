
#include <stdio.h>
#include <unistd.h>
#include <curses.h>

#include "ros/ros.h"
#include "scout_msgs/ScoutMotorsMsg.h"
#include "scout_msgs/ScoutSonarsMsg.h"
#include "scout_msgs/ScoutBatteryMsg.h"
#include "scout_msgs/ScoutMotionSrv.h"

bool en = true;
int vc = 0;
int vd = 0;

void motors_cb(const scout_msgs::ScoutMotorsMsg &msg) {
  mvprintw(11, 12, "%6d %6d       ", msg.count_left, msg.count_right);
}

void battery_cb(const scout_msgs::ScoutBatteryMsg &msg) {
  mvprintw(10, 12, "%6.1f %6.1f       ", msg.battery1, msg.battery2);
}

void sonars_cb(const scout_msgs::ScoutSonarsMsg &msg) {
  const std::vector<float> &z = msg.sonars;
  for (int i=0 ; i<z.size() ; i++) {
    mvprintw(13+i, 0, "%6.1f ", z[i]);
    for (int j=0 ; j<z[i] ; j++)
      addch('#');
    clrtoeol();
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "keydrive");
  ros::NodeHandle n;
  ros::ServiceClient motion = n.serviceClient<scout_msgs::ScoutMotionSrv>("/scout/motion");

  // ncurses init
  initscr(); cbreak(); noecho();
  nonl(); intrflush(stdscr, FALSE); keypad(stdscr, TRUE);
  halfdelay(2);

  // display help message (5 lines)
  mvprintw(0, 0, "Keys:\n  o / p =   left / right\n  q / a =     up / down\n  e / d = enable / disable\n      x = quit");
  mvprintw(10, 2, "battery:");
  mvprintw(11, 1, "encoders:");
  mvprintw(12, 3, "sonars:");

  // start sensor readings callback
  ros::Subscriber sub_bat = n.subscribe("/scout/battery", 1, battery_cb);
  ros::Subscriber sub_son = n.subscribe("/scout/sonars", 1, sonars_cb);
  ros::Subscriber sub_mot = n.subscribe("/scout/motors", 1, motors_cb);

  // main loop
  bool quit = false;
  while (!quit) {
    int ch = getch();
    flushinp();
    switch(ch) {
    case KEY_UP:
    case 'q':
      vc++;
      break;
    case KEY_DOWN:
    case 'a':
      vc--;
      break;
    case KEY_LEFT:
    case 'o':
      vd++;
      break;
    case KEY_RIGHT:
    case 'p':
      vd--;
      break;
    case 'e':
      en = true;
      break;
    case 'd':
      en = false;
      break;
    case 'x':
      quit = true;
      // fallback
    case ' ':
      vc = vd = 0;
      break;
    }
    mvprintw(6, 0, " vc = %4d\n vd = %4d\n en = %s", vc, vd, en?"Y":"N");
    refresh();

    // compute wheel velocities
    int vl = -100*(vc-vd);
    int vr =  100*(vc+vd);

    // call service
    scout_msgs::ScoutMotionSrv srv;
    srv.request.enable = en;
    srv.request.velocity_left  = vl;
    srv.request.velocity_right = vr;
    motion.call(srv);

    ros::spinOnce();
  }
  endwin();
  return 0;
}

// EOF
