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

#include "ros_node_lib/rosout_appender.h"
#include "ros_node_lib/advertise_options.h"
#include "ros_node_lib/subscriber_callbacks.h"
#include <rosgraph_msgs/Log.h>
#include <boost/make_shared.hpp>

namespace roscan {

ROSOutAppender::ROSOutAppender(const RosNodePtr& node)
    : node_{node}, shutting_down_{false}, publish_thread_{&ROSOutAppender::logThread, this} {
    AdvertiseOptions ops;
    ops.init<rosgraph_msgs::Log>("/rosout", 0);
    ops.latch = true;
    SubscriberCallbacksPtr cbs{boost::make_shared<SubscriberCallbacks>()};
    node_->topic_manager()->advertise(ops, cbs);
}

ROSOutAppender::~ROSOutAppender() {
    shutting_down_ = true;

    {
        std::lock_guard<std::mutex> lock{queue_mutex_};
        queue_condition_.notify_all();
    }

    if (publish_thread_.joinable()) {
        publish_thread_.join();
    }
}

void ROSOutAppender::log(ros::console::Level level, const char *const str, const char *const file, const char *const function, const int line) {
    rosgraph_msgs::LogPtr msg{boost::make_shared<rosgraph_msgs::Log>()};

    msg->header.stamp = ros::Time::now();
    if (level == ros::console::levels::Debug) {
        msg->level = rosgraph_msgs::Log::DEBUG;
    } else if (level == ros::console::levels::Info) {
        msg->level = rosgraph_msgs::Log::INFO;
    } else if (level == ros::console::levels::Warn) {
        msg->level = rosgraph_msgs::Log::WARN;
    } else if (level == ros::console::levels::Error) {
        msg->level = rosgraph_msgs::Log::ERROR;
    } else if (level == ros::console::levels::Fatal) {
        msg->level = rosgraph_msgs::Log::FATAL;
    }
    msg->name = node_->getName();
    msg->msg = str;
    msg->file = file;
    msg->function = function;
    msg->line = line;
    node_->getAdvertisedTopics(msg->topics);

    if (level == ros::console::levels::Fatal || level == ros::console::levels::Error) {
        last_error_ = str;
    }

    std::lock_guard<std::mutex> lock{queue_mutex_};
    log_queue_.push_back(msg);
    queue_condition_.notify_all();
}

void ROSOutAppender::logThread() {
    while (!shutting_down_) {
        V_Log local_queue;

        {
            std::unique_lock<std::mutex> lock{queue_mutex_};

            if (shutting_down_) {
                return;
            }

            queue_condition_.wait(lock);

            if (shutting_down_) {
                return;
            }

            local_queue.swap(log_queue_);
        }

        for (const auto& l: local_queue) {
            node_->topic_manager()->publish("/rosout", *l);
        }
    }
}

} // namespace roscan
