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
 *   * Neither the names of Willow Garage, Inc. nor the names of its
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

#ifndef ROSCAN_SUBSCRIPTION_QUEUE_H
#define ROSCAN_SUBSCRIPTION_QUEUE_H

#include "ros_node_lib/rosdefs.h"
#include "ros_node_lib/callback_queue_interface.h"
#include <boost/enable_shared_from_this.hpp>
#include <deque>
#include <mutex>
#include <string>
#include <cstdint>
#include <ros/time.h>

namespace ros {

class MessageDeserializer;
typedef boost::shared_ptr<MessageDeserializer> MessageDeserializerPtr;

class SubscriptionCallbackHelper;
typedef boost::shared_ptr<SubscriptionCallbackHelper> SubscriptionCallbackHelperPtr;

} // namespace ros

namespace roscan {

class SubscriptionQueue : public CallbackInterface, public boost::enable_shared_from_this<SubscriptionQueue> {
    private:
        struct Item {
            ros::SubscriptionCallbackHelperPtr helper;
            ros::MessageDeserializerPtr deserializer;

            bool has_tracked_object;
            ros::VoidConstWPtr tracked_object;

            bool nonconst_need_copy;
            ros::Time receipt_time;
        };
        typedef std::deque<Item> D_Item;

    public:
        SubscriptionQueue(const std::string& topic, const int32_t queue_size, const bool allow_concurrent_callbacks)
            : topic_{topic}, size_{queue_size}, full_{false}, queue_size_{0}, allow_concurrent_callbacks_{allow_concurrent_callbacks} {}
        ~SubscriptionQueue() {}

        void push(const ros::SubscriptionCallbackHelperPtr& helper, const ros::MessageDeserializerPtr& deserializer,
                  const bool has_tracked_object, const ros::VoidConstWPtr& tracked_object, const bool nonconst_need_copy,
                  const ros::Time receipt_time = ros::Time(), bool *const was_full = nullptr);
        void clear();

        CallbackInterface::CallResult call() override;
        bool ready() const override { return true; }
        bool full();

    private:
        bool fullNoLock() const { return size_ > 0 && queue_size_ >= (uint32_t)size_; }
        std::string topic_;
        int32_t size_;
        bool full_;

        std::mutex queue_mutex_;
        D_Item queue_;
        uint32_t queue_size_;
        bool allow_concurrent_callbacks_;

        std::recursive_mutex callback_mutex_;
};

} // namespace roscan

#endif // ROSCAN_SUBSCRIPTION_QUEUE_H
