/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros_node_lib/network.h"
#include <ifaddrs.h>
#include <ros/io.h>

namespace roscan {

namespace network {

std::string determineHost();

static std::string g_host = determineHost();
static uint16_t g_tcpros_server_port = 0;

const std::string& getHost() {
    return g_host;
}

bool splitURI(const std::string& uri, std::string& host, uint32_t& port) {
    // skip over the protocol if it's there
    if (uri.substr(0, 7) == std::string{"http://"}) {
        host = uri.substr(7);
    } else if (uri.substr(0, 9) == std::string{"rosrpc://"}) {
        host = uri.substr(9);
    }
    // split out the port
    auto colon_pos = host.find_first_of(":");
    if (colon_pos == std::string::npos) {
        return false;
    }
    auto port_str = host.substr(colon_pos + 1);
    auto slash_pos = port_str.find_first_of("/");
    if (slash_pos != std::string::npos) {
        port_str = port_str.erase(slash_pos);
    }
    port = atoi(port_str.c_str());
    host = host.erase(colon_pos);
    return true;
}

uint16_t getTCPROSPort() {
    return g_tcpros_server_port;
}

static bool isPrivateIP(const char *ip) {
    return !strncmp("192.168", ip, 7) || !strncmp("10.", ip, 3) || !strncmp("169.254", ip, 7);
}

std::string determineHost() {
    std::string ip_env;
    // First, did the user set ROS_HOSTNAME?
    if (ros::get_environment_variable(ip_env, "ROS_HOSTNAME")) {
        if (ip_env.size() == 0) {
            ROS_WARN("invalid ROS_HOSTNAME (an empty string)");
        }
        return ip_env;
    }

    // Second, did the user set ROS_IP?
    if (ros::get_environment_variable(ip_env, "ROS_IP")) {
        if (ip_env.size() == 0) {
            ROS_WARN("invalid ROS_IP (an empty string)");
        }
        return ip_env;
    }

    // Third, try the hostname
    char host[1024] = {0};
    if (gethostname(host, sizeof(host) - 1) != 0) {
        ROS_ERROR("determineIP: gethostname failed");
    }
    // We don't want localhost to be our ip
    else if (strlen(host) && strcmp("localhost", host)) {
        return std::string{host};
    }

    // Fourth, fall back on interface search, which will yield an IP address
    ifaddrs *ifa = nullptr, *ifp = nullptr;
    int rc;
    if ((rc = getifaddrs(&ifp)) < 0) {
        ROS_FATAL("error in getifaddrs: [%s]", strerror(rc));
    }
    char preferred_ip[200] = {0};
    for (ifa = ifp; ifa; ifa = ifa->ifa_next) {
        char ip_[200];
        socklen_t salen;
        if (!ifa->ifa_addr) {
            continue; // evidently this interface has no ip address
        }
        if (ifa->ifa_addr->sa_family == AF_INET) {
            salen = sizeof(sockaddr_in);
        } else if (ifa->ifa_addr->sa_family == AF_INET6) {
            salen = sizeof(sockaddr_in6);
        } else {
            continue;
        }
        if (getnameinfo(ifa->ifa_addr, salen, ip_, sizeof(ip_), nullptr, 0, NI_NUMERICHOST) < 0) {
            continue;
        }
        // prefer non-private IPs over private IPs
        if (!strcmp("127.0.0.1", ip_) || strchr(ip_, ':')) {
            continue; // ignore loopback unless we have no other choice
        }
        if (ifa->ifa_addr->sa_family == AF_INET6 && !preferred_ip[0]) {
            strcpy(preferred_ip, ip_);
        } else if (isPrivateIP(ip_) && !preferred_ip[0]) {
            strcpy(preferred_ip, ip_);
        } else if (!isPrivateIP(ip_) && (isPrivateIP(preferred_ip) || !preferred_ip[0])) {
            strcpy(preferred_ip, ip_);
        }
    }
    freeifaddrs(ifp);
    if (!preferred_ip[0]) {
        ROS_ERROR("Couldn't find a preferred IP via the getifaddrs() call; I'm assuming that your IP "
                  "address is 127.0.0.1.  This should work for local processes, "
                  "but will almost certainly not work if you have remote processes."
                  "Report to the ROS development team to seek a fix.");
        return std::string{"127.0.0.1"};
    }
    return std::string{preferred_ip};
}

void init() {
    g_host = determineHost();
}

} // namespace network

} // namespace roscan
