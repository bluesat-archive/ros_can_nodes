/*
 * Date Started: 3/12/2017
 * Original Author: Simon Ireland
 * Editors:
 * ROS Node Name:
 * ROS Package: ros_can_nodes
 * Purpose:
 * This code is released under the MIT  License. Copyright BLUEsat UNSW, 2017
 */


#include <cstring>
#include <ctime>
#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "CANHelpers.hpp"

int soc;
tm last_print_time = {0};

int CANHelpers::open_can_port(const char *const port) {
    ifreq ifr;
    sockaddr_can addr;

    /* open socket */
    soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (soc < 0) {
        return (-1);
    }

    addr.can_family = AF_CAN;
    strcpy(ifr.ifr_name, port);

    if (ioctl(soc, SIOCGIFINDEX, &ifr) < 0) {
        return (-1);
    }

    addr.can_ifindex = ifr.ifr_ifindex;

    fcntl(soc, F_SETFL, O_NONBLOCK);

    if (bind(soc, (sockaddr *)&addr, sizeof(addr)) < 0) {
        return (-1);
    }

    return 0;
}

int CANHelpers::send_can_port(const can_frame& frame) {
    const auto err = write(soc, &frame, sizeof(can_frame)) != sizeof(can_frame) ? -1 : 0;

    const auto time_now = time(NULL);
    const auto time_since_last_print = difftime(time_now, mktime(&last_print_time));

    if (time_since_last_print >= 1.0) {
        if (err) {
            std::cout << "Failed to send !! " << strerror(errno) << "\n";
        } else {
            std::cout << "Sent !!\n";
        }
        last_print_time = *localtime(&time_now);
    }
    return err;
}

int CANHelpers::read_can_port(can_frame& frame) {
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

void CANHelpers::close_can_port() {
    close(soc);
}
