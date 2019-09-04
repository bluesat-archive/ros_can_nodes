#include "can.hpp"
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <iostream>

static int soc = 0;

int open_can_port(const std::string& port) {
    ifreq ifr;
    sockaddr_can addr;

    // open socket
    soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (soc < 0) {
        return -1;
    }

    addr.can_family = AF_CAN;
    strcpy(ifr.ifr_name, port.c_str());

    if (ioctl(soc, SIOCGIFINDEX, &ifr) < 0) {
        return -1;
    }

    addr.can_ifindex = ifr.ifr_ifindex;

    fcntl(soc, F_SETFL, O_NONBLOCK);

    return bind(soc, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
}

int send_can_frame(const can_frame& frame) {
    return write(soc, &frame, sizeof(can_frame)) == sizeof(can_frame) ? 0 : -1;
}
