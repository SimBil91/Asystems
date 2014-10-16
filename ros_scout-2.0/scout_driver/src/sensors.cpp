
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <signal.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "ros/ros.h"
#include "scout_msgs/ScoutSonarsMsg.h"
#include "scout_msgs/ScoutBatteryMsg.h"

#define BUF_SIZE 1024
#define TIMEOUT_USEC 250000
#define USB_MAX 10

struct port {
  const char *name;
  int fd;
  unsigned char buf[BUF_SIZE];
  int idx;
};

#define NP 2
enum { BATTERY, SONARS };
struct port p[NP];

ros::Publisher pub[NP];

int usb_open(const char *filename) {
  int r = open(filename, O_RDWR|O_NONBLOCK|O_NOCTTY);
  if (r!=-1) {
    struct termios term;
    tcgetattr(r, &term);
    cfmakeraw(&term);
    tcsetattr(r, TCSANOW, &term);
    tcflush(r, TCIOFLUSH);
  }
  return r;
}

void usb_setspeed(int fd, speed_t baud) {
    struct termios term;
    tcgetattr(fd, &term);
    cfsetspeed(&term, baud);
    tcsetattr(fd, TCSANOW, &term);
    tcflush(fd, TCIOFLUSH);
}

void usb_close(int fd) {
  fsync(fd);
  close(fd);
}

void port_init(int id, int fd, const char *name) {
  char *n = new char[BUF_SIZE];
  sprintf(n, "%s (auto)", name);
  p[id].name = n;
  p[id].fd   = fd;
  p[id].idx  = 0;
}

void ask_battery(int fd) {
  write(fd, "T", 1);
}

void ask_sonars(int fd) {
  unsigned char buf[4] = {255, 1, 32, 1};
  write(fd, buf, 4);
}

int probe(void) {
  int total = 0;

  for (int i=0 ; i<NP ; i++)
    port_init(i, -1, NULL);

  for (int i=0 ; i<USB_MAX ; i++) {
    char name[BUF_SIZE];
    sprintf(name, "/dev/ttyUSB%d", i);
    printf("Probing %s...\n", name);
    int fd = usb_open(name);

    if (fd!=-1) {
      usb_setspeed(fd, B9600);
      ask_battery(fd);
      usleep(TIMEOUT_USEC);
      unsigned char buf[BUF_SIZE];
      int r = read(fd, buf, BUF_SIZE);
      if (r>0) {
        /* Battery port detected */
        buf[r] = 0;
        printf("battery: read %d bytes: 0x%02x 0x%02x\n", r, buf[0], buf[1]);
        port_init(BATTERY, fd, name);
        total++;
      } else {
        usb_setspeed(fd, B115200);
        ask_sonars(fd);
        usleep(TIMEOUT_USEC);
        r = read(fd, buf, BUF_SIZE);
        if (r>0) {
          /* Sonars port detected */
          printf("sonars: read %d bytes: 0x%02x 0x%02x 0x%02x\n\t", r, buf[0], buf[1], buf[2]);
          for (int i=0 ; i<16 ; i++)
            printf(" %d", ((int)buf[3+2*i]<<8)|(int)buf[3+2*i+1]);
          printf("\n");
          port_init(SONARS, fd, name);
          total++;
        } else {
          printf("nothing here\n");
          usb_close(fd);
        }
      }
    }
  }

  return total;
}

void port_process(int id) {
  unsigned char *ibuf = p[id].buf;
        
  int r = read(p[id].fd, &(ibuf[p[id].idx]), BUF_SIZE-p[id].idx);
  /* printf("read %d port %d\n", r, id); */
  if (r>0) {
    p[id].idx += r;
    int ki, ks;
    for (ki=0,ks=0 ; ki<p[id].idx ; ki++) {
      /* printf("ki=%d ks=%d ibuf[ki]='%c'\n", ki, ks, ibuf[ki]); */
      switch (id) {

      case SONARS:
        if (ibuf[ki]==0xFF && p[id].idx-ki>=35) {
          if (ibuf[ki+1]==0x20 && ibuf[ki+2]==0x30) {
            std::vector<float> z(16);
            for (int i=0 ; i<16 ; i++) {
              int raw = (((int)ibuf[ki+3+2*i])<<8) | (int)ibuf[ki+3+2*i+1];
              z[i] = 4*raw*340.9*100/(2*1843200);
              /* printf("sonar #%02d = %f\n", i, z[i]); */
            }
            /* publish sonars */
            scout_msgs::ScoutSonarsMsg msg;
            msg.sonars = z;
            pub[SONARS].publish(msg);
          }
          ks = ki+35;
        }
        break;

      case BATTERY:
        if (p[id].idx-ki>=2) {
          int idx = p[id].idx;
          float b[2];
          for (int i=0 ; i<2 ; i++) {
            /* HACK: assuming the two last values in buffer are the two battery values */
            int raw = ibuf[idx-2+i];
            b[i] = 0.116*raw;
            /* printf("battery #%d = %f\n", i, b[i]); */
          }
          /* publish battery */
          scout_msgs::ScoutBatteryMsg msg;
          msg.battery1 = b[0];
          msg.battery2 = b[1];
          pub[BATTERY].publish(msg);
          ks = idx;
        }
        break;

      }
    }
    if (ks>0) {
      memmove(ibuf, &ibuf[ks], p[id].idx-ks);
      p[id].idx -= ks;
    }
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "sensors");
  ros::NodeHandle n;

  /* Node setup */
  pub[BATTERY] = n.advertise<scout_msgs::ScoutBatteryMsg>("/scout/battery", 1000);
  pub[SONARS]  = n.advertise<scout_msgs::ScoutSonarsMsg>("/scout/sonars", 1000);

  if (probe()>0)  {
    ROS_INFO("Scout sensors node launched");
    while (ros::ok()) {
      /* prepare select() */
      struct timeval timeout = {0, 200000};
      fd_set rfds;
      FD_ZERO(&rfds);
      int maxfd = -1;
      for (int i=0 ; i<NP ; i++) {
        int fd = p[i].fd;
        if (fd!=-1) {
          if (fd>maxfd) maxfd=fd;
          FD_SET(fd, &rfds);
        }
      }
      /* perform select() */
      int ret = select(maxfd+1, &rfds, NULL, NULL, &timeout);
      if (ret>0) {
        for (int i=0 ; i<NP ; i++) {
          int fd = p[i].fd;
          if (fd!=-1 && FD_ISSET(fd, &rfds))
            port_process(i);
        }
      } else {
        /* request sensor readings */
        if (-1!=p[BATTERY].fd) ask_battery(p[BATTERY].fd);
        if (-1!=p[SONARS].fd)  ask_sonars(p[SONARS].fd);

        /* handle ROS callbacks */
        ros::spinOnce();
      }
    }
    return 0;
  } else {
    printf("*** NO PORTS FOUND ***\n");
    return 1;
  }
}

/* EOF */
