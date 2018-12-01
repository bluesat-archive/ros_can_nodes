/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros_node_lib/RosNode.hpp"
#include "ros_node_lib/publisher_link.h"
#include "ros_node_lib/connection_manager.h"
#include "ros_node_lib/subscription.h"
#include <ros/connection.h>
#include <ros/header.h>
#include <ros/transport/transport.h>

namespace roscan {

bool PublisherLink::setHeader(const ros::Header& header) {
    header.getValue("callerid", caller_id_);

    std::string md5sum, type, latched_str;
    if (!header.getValue("md5sum", md5sum)) {
        ROS_ERROR("Publisher header did not have required element: md5sum");
        return false;
    }

    md5sum_ = md5sum;

    if (!header.getValue("type", type)) {
        ROS_ERROR("Publisher header did not have required element: type");
        return false;
    }

    latched_ = false;
    if (header.getValue("latching", latched_str)) {
        if (latched_str == "1") {
            latched_ = true;
        }
    }

    connection_id_ = node_->connection_manager()->getNewConnectionID();
    header_ = header;

    if (const auto parent = parent_.lock()) {
        parent->headerReceived(shared_from_this(), header);
    }
    return true;
}

} // namespace roscan
