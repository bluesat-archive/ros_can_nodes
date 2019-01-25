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

#ifndef ROSCAN_ROSOUT_APPENDER_H
#define ROSCAN_ROSOUT_APPENDER_H

#include "ros_node_lib/RosNode.hpp"
#include <ros/message_forward.h>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <condition_variable>

namespace rosgraph_msgs {
ROS_DECLARE_MESSAGE(Log);
}

namespace roscan {

class ROSOutAppender : public ros::console::LogAppender {
    public:
        ROSOutAppender(const RosNodePtr& node);
        ~ROSOutAppender();

        const std::string& getLastError() const { return last_error_; }

        void log(ros::console::Level level, const char *const str, const char *const file, const char *const function, const int line) override;

    protected:
        void logThread();

        RosNodePtr node_;

        std::string last_error_;

        typedef std::vector<rosgraph_msgs::LogPtr> V_Log;
        V_Log log_queue_;
        std::mutex queue_mutex_;
        std::condition_variable queue_condition_;
        bool shutting_down_;

        std::thread publish_thread_;
};

} // namespace roscan

#endif // ROSCAN_ROSOUT_APPENDER_H
