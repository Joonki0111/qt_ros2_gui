#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

class CAN_Interface
{
  private:
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    const char *ifname;
    bool roscco_status[3];


  public:
    explicit CAN_Interface();
    bool* read_roscco_status();
};
