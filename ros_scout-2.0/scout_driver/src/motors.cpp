
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <signal.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "ros/ros.h"
#include "scout_msgs/ScoutMotorsMsg.h"
#include "scout_msgs/ScoutMotionSrv.h"
#include "scout_msgs/ScoutMotionMsg.h"

#define MOTION_TIMEOUT 2
#define TIMEOUT_USECS 100000
#define BUF_SIZE 1024

/* Motor low level controller acceleration */
#define DEFAULT_ACCEL 15

/* Serial default ports */
#define DEFAULT_PORT0 "/dev/ttyS0"
#define DEFAULT_PORT1 "/dev/ttyS1"

struct port {
  std::string name;
  int fd;
  char buf[BUF_SIZE];
  int idx;
};


void dumper(int, char *);
void get_msg(int, char *);

#define NP 2
struct port p[NP];

int count[NP];
bool valid_counts[NP];
int accel = DEFAULT_ACCEL;

struct timeval last_motion;
bool last_state;

scout_msgs::ScoutMotorsMsg motors_msg;

void port_open(int id) {
  int ret = open(p[id].name.c_str(), O_RDWR|O_NONBLOCK|O_NOCTTY);
  if (ret==-1) {
    perror(p[id].name.c_str());
    exit(1);
  }

  struct termios term;
  tcgetattr(ret, &term);
  cfmakeraw(&term);		
  cfsetspeed(&term, B9600);
  tcsetattr(ret, TCSANOW, &term);
  tcflush(ret, TCIOFLUSH);

  p[id].fd  = ret;
  p[id].idx = 0;
}

void port_send(int id, const char *buf) {
  int ret = write(p[id].fd, buf, strlen(buf));
  //printf("wrote %d bytes to port port %d\n", ret, id);
  if (ret==-1) {
    perror(p[id].name.c_str());
  }
}

void port_process(int id) {
  char *ibuf = p[id].buf;
        
  int r = read(p[id].fd, &(ibuf[p[id].idx]), BUF_SIZE-p[id].idx);
  //printf("read %d bytes from port %d\n", r, id);
  if (r>0) {
    p[id].idx += r;
    int ki, ks;
    for (ki=1,ks=0 ; ki<p[id].idx ; ki++) {
      /* printf("ki=%d ks=%d ibuf[ki]='%c'\n", ki, ks, ibuf[ki]); */
      if (0==strncmp(&ibuf[ki-1], "\r\n", 2)) {
        char obuf[BUF_SIZE];
        strncpy(obuf, &ibuf[ks], ki-ks-1); /* removes trailing \r\n */
        obuf[ki-ks-1] = 0;
        get_msg(id, obuf);
        ks = ki+1;
      }
    }
    if (ks>0) {
      memmove(ibuf, &ibuf[ks], p[id].idx-ks);
      p[id].idx -= ks;
    }
  }
}

void dumper(int id, char *out) {
  int r = strlen(out);

  printf("Read %d bytes from port #%d:  ", r, id);
#if 0
  for (int j=0 ; j<r ; j++) {
    printf(" 0x%02x", out[j]);
  }
  printf("   ");
#endif
  for (int j=0 ; j<r ; j++) {
    putchar(isprint(out[j])?out[j]:'.');
  }
  printf("\n");
}

void port_close(int id) {
  fsync(p[id].fd);
  close(p[id].fd);
}

void send_vel(int left, int right) {
  /* printf("send_vel: %d %d\n", left, right); */
  char buf[BUF_SIZE];

  sprintf(buf, "V%d\r", left);
  port_send(0, buf);
  sprintf(buf, "V%d\r", right);
  port_send(1, buf);

  /* ROS_INFO("(left)  V%d\r", left);
     ROS_INFO("(right) V%d\r", right); */
}  

void send_state(bool enable) {
  /* printf("send_state: %d\n", enable); */
  for (int i=0 ; i<NP ; i++)
    port_send(i, enable?"EN\r":"DI\r");
  last_state = enable;
}

void shutdown(int sig) {
  ROS_INFO("Disabling motors");
  send_vel(0, 0);
  port_close(0);
  port_close(1);
  exit(0);
}

void motors_init(void) {
  // Original (IdMind)
  // char init1[] = "di\rencres 2064\ransw 0\rapl 0\rsor 0\rsp 30000\rlcc 5000\rlpc 8000\rac 30000\rpor16\rI30\rcontmod\rv0\r";
  //char init2[] = "diprog\rprogseq\rseta1\ra1\rdelay80\rjpe2\rseta1\rdi\rjmp1\ra2\rdajnz3\ren\rseta2\rjmp1\ra3\rseta2\rjmp1\rend\renprog\r";
  // Modified (yoda)
  char init1[1024];
  char init2[] = "diprog\rprogseq\rseta1\ra1\rdelay25\rjpe2\rseta1\rdi\rjmp1\ra2\rdajnz3\ren\rseta2\rjmp1\ra3\rseta2\rjmp1\rend\renprog\r";

  sprintf(init1, "di\rencres 2064\ransw 0\rapl 0\rsor 0\rsp 30000\rlcc 5000\rlpc 8000\rac %d\rpor16\rI30\rcontmod\rv0\reepsav\r", accel);

  // p[0].name = "/dev/ttyS0";
  // p[1].name = "/dev/ttyS1";

  for (int i=0 ; i<NP ; i++) {
    port_open(i);
    port_send(i, init1);
    port_send(i, init2);
    // BUG: port_send(i, "AC15\r");
  }
  send_state(true);
}


void get_msg(int id, char *data) {
  /* printf("Port #%d got '%s'\n", id, data); */
  /*count[id] = atoi(data);*/
  int tmp;
  if ( 1==sscanf(data, "%d", &tmp) ) {
    count[id] = tmp;
    valid_counts[id] = true;
  }
}

void ask_encoders(void) {
  /* ask for encoders report */
  for (int i=0 ; i<NP ; i++) {
    /* printf("Sending POS to port %d\n", i); */
    port_send(i, "POS\r");
  }
}


void motion(bool enable, int velocity_left, int velocity_right) {
  //ROS_INFO("ENTER motion()");

  if (enable!=last_state)
    send_state(enable);
  send_vel(velocity_left, velocity_right); 

  if (-1==gettimeofday(&last_motion, NULL)) {
    perror("motion:gettimeofday()");
    exit(1);
  }

  //ROS_INFO("RETURN motion()");
}

bool motion_srv(scout_msgs::ScoutMotionSrv::Request &req, scout_msgs::ScoutMotionSrv::Response &res) {
  motion(req.enable, req.velocity_left, req.velocity_right);
  return true;
}

void motion_msg(const scout_msgs::ScoutMotionMsg &msg) {
  motion(msg.enable, msg.velocity_left, msg.velocity_right);
}

void reset_valid_counts(void) {
  for (int i=0 ; i<NP ; i++)
    valid_counts[i] = false;
}

bool all_valid_counts(void) {
  for (int i=0 ; i<NP ; i++)
    if (!valid_counts[i]) return false;
  return true;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "motors");
  ros::NodeHandle n;

  timerclear(&last_motion);
  reset_valid_counts();

  /* Node setup */
  ros::ServiceServer srv = n.advertiseService("/scout/motion", motion_srv);
  ros::Subscriber sub = n.subscribe("/scout/motion", 1, motion_msg);
  ros::Publisher pub = n.advertise<scout_msgs::ScoutMotorsMsg>("/scout/motors", 1000);

  /* Read parameters */
  ros::NodeHandle tmp("~");
  if ( !tmp.getParam("accel", accel) )
    accel = DEFAULT_ACCEL;
  if ( !tmp.getParam("port0", p[0].name) )
    p[0].name = DEFAULT_PORT0;
  if ( !tmp.getParam("port1", p[1].name) )
    p[1].name = DEFAULT_PORT1;

  motors_init();
  signal(SIGINT, shutdown);

  ROS_INFO("Motors node launched (accel=%d)", accel);
  while (ros::ok()) {
    /* prepare select() */
    struct timeval timeout = {0, TIMEOUT_USECS};
    fd_set rfds;
    FD_ZERO(&rfds);
    int maxfd = -1;
    for (int i=0 ; i<NP ; i++) {
      int fd = p[i].fd;
      if (fd>maxfd) maxfd=fd;
      FD_SET(fd, &rfds);
    }

    /* perform select() */
    int ret = select(maxfd+1, &rfds, NULL, NULL, &timeout);
    if (ret>0) {
      /* processing serial port data */
      for (int i=0 ; i<NP ; i++) {
        int fd = p[i].fd;
        if (FD_ISSET(fd, &rfds))
          port_process(i);
      }
    } else {
      /* timeout */
      ROS_WARN("No response from any motor");
      reset_valid_counts();
      ask_encoders();
    }
    
    /* check of all encoder position reports have arrived */
    if (all_valid_counts()) {
      /* send ROS message */
      motors_msg.header.stamp = ros::Time::now();
      motors_msg.count_left  = count[0];
      motors_msg.count_right = count[1];
      /* printf("Sending data %d %d\n", motors_msg.count_left, motors_msg.count_right); */
      pub.publish(motors_msg);
      reset_valid_counts();
      ask_encoders();
    }

    /* check motor command timeout */
    if (timerisset(&last_motion)) {
      struct timeval now, diff;
      if (-1==gettimeofday(&now, NULL)) {
        perror("main:gettimeofday()");
        exit(1);
      } else {
        timersub(&now, &last_motion, &diff);
        /* printf("diff: %d %d\n", diff.tv_sec, diff.tv_usec); */
        if (diff.tv_sec >= MOTION_TIMEOUT) {
          ROS_WARN("No velocity commands for %d seconds -- STOPPING", MOTION_TIMEOUT);
          send_vel(0, 0);
          timerclear(&last_motion);
        }
      }
    }

    /* handle ROS callbacks */
    ros::spinOnce();
  }

  return 0;
}


/* EOF */
