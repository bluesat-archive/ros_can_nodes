/*
 * Date Started: 3/12/2017
 * Original Author: Simon Ireland
 * Editors:
 * ROS Node Name:
 * ROS Package: ros_can_nodes
 * Purpose:
 * This code is released under the BSD License. Copyright BLUEsat UNSW, 2017
 */


#include <cstdio>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <ros/ros.h>
#include "CANHelpers.hpp"

static int soc = -1;

int CANHelpers::open_port(const std::string& port) {
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

    if (bind(soc, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        return -1;
    }

    return 0;
}

int CANHelpers::send_frame(const can_frame& frame) {
    const int retval = write(soc, &frame, sizeof(can_frame));
    if (retval != sizeof(can_frame)) {
        ROS_INFO("Failed to send !! %s", strerror(errno));
        return -1;
    }
    ROS_INFO("Sent !!");
    ROS_INFO("Header 0x%x", frame.can_id);
    return 0;
}

int CANHelpers::read_frame(can_frame& frame) {
    int recvbytes = -1;

    // 1 second timeout on read, will adjust based on testing
    timeval timeout = {1, 0};
    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(soc, &readSet);

    // Check if the socket is ready to read from
    // Possibly check errno for bad FD
    if (select((soc + 1), &readSet, NULL, NULL, &timeout) >= 0) {
        if (FD_ISSET(soc, &readSet)) {
            recvbytes = read(soc, &frame, sizeof(can_frame));
        }
    }

    return recvbytes;
}

void CANHelpers::close_port() {
    close(soc);
}
