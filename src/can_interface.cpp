#include "qt_ros2_test/can_interface.hpp"

CAN_Interface::CAN_Interface()
{
    const char *ifname = "vcan0";

    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) 
    {
        perror("socket create failure");
        return;
    }

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) 
    {
        perror("socket binding failure");
        return;
    }
}

bool* CAN_Interface::read_roscco_status()
{
    // CAN ID 필터 설정 (0x73, 0x83, 0x93에 대해서만 수신)
    struct can_filter rfilter[3];
    rfilter[0].can_id = 0x73;
    rfilter[0].can_mask = CAN_SFF_MASK;
    rfilter[1].can_id = 0x83;
    rfilter[1].can_mask = CAN_SFF_MASK;
    rfilter[2].can_id = 0x93;
    rfilter[2].can_mask = CAN_SFF_MASK;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    int nbytes = read(s, &frame, sizeof(struct can_frame));

    if (nbytes < 0) 
    {
        perror("can message read failure");
    }

    if (frame.can_id == 0x73 || frame.can_id == 0x83 || frame.can_id == 0x93) 
    {
        bool roscco_status_byte = static_cast<bool>(frame.data[2]);

        if (frame.can_id == 0x73)
        {
            roscco_status[0] = roscco_status_byte; //brake
        }
        else if (frame.can_id == 0x83)
        {
            roscco_status[1] = roscco_status_byte; //steering
        }
        else if (frame.can_id == 0x93)
        {
            roscco_status[2] = roscco_status_byte; //throttle
        }
    }

    return roscco_status;
}