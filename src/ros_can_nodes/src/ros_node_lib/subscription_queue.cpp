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

#include "subscription_queue.h"
#include <ros/message_deserializer.h>
#include <ros/subscription_callback_helper.h>

namespace roscan {

void SubscriptionQueue::push(const ros::SubscriptionCallbackHelperPtr& helper, const ros::MessageDeserializerPtr& deserializer,
                             const bool has_tracked_object, const ros::VoidConstWPtr& tracked_object, const bool nonconst_need_copy,
                             const ros::Time receipt_time, bool *const was_full) {
    std::lock_guard<std::mutex> lock{queue_mutex_};

    if (was_full) {
        *was_full = false;
    }

    if (fullNoLock()) {
        queue_.pop_front();
        --queue_size_;

        full_ = true;

        if (was_full) {
            *was_full = true;
        }
    } else {
        full_ = false;
    }

    Item i;
    i.helper = helper;
    i.deserializer = deserializer;
    i.has_tracked_object = has_tracked_object;
    i.tracked_object = tracked_object;
    i.nonconst_need_copy = nonconst_need_copy;
    i.receipt_time = receipt_time;
    queue_.push_back(i);
    ++queue_size_;
}

void SubscriptionQueue::clear() {
    std::lock_guard<std::recursive_mutex> cb_lock{callback_mutex_};
    std::lock_guard<std::mutex> queue_lock{queue_mutex_};

    queue_.clear();
    queue_size_ = 0;
}

CallbackInterface::CallResult SubscriptionQueue::call() {
    // The callback may result in our own destruction.  Therefore, we may need to keep a reference to ourselves
    // that outlasts the scoped_try_lock
    boost::shared_ptr<SubscriptionQueue> self;
    //boost::recursive_mutex::scoped_try_lock lock(callback_mutex_, boost::defer_lock);
    std::unique_lock<std::recursive_mutex> lock{callback_mutex_, std::defer_lock};

    if (!allow_concurrent_callbacks_) {
        lock.try_lock();
        if (!lock.owns_lock()) {
            return CallbackInterface::TryAgain;
        }
    }

    ros::VoidConstPtr tracker;
    Item i;

    {
        std::lock_guard<std::mutex> lock{queue_mutex_};

        if (queue_.empty()) {
            return CallbackInterface::Invalid;
        }

        i = queue_.front();

        if (queue_.empty()) {
            return CallbackInterface::Invalid;
        }

        if (i.has_tracked_object) {
            tracker = i.tracked_object.lock();

            if (!tracker) {
                return CallbackInterface::Invalid;
            }
        }

        queue_.pop_front();
        --queue_size_;
    }

    // msg can be null here if deserialization failed
    if (const auto msg = i.deserializer->deserialize()) {
        try {
            self = shared_from_this();
        } catch (boost::bad_weak_ptr&) {}// For the tests, where we don't create a shared_ptr

        ros::SubscriptionCallbackHelperCallParams params;
        params.event = ros::MessageEvent<void const>{msg, i.deserializer->getConnectionHeader(), i.receipt_time, i.nonconst_need_copy, ros::MessageEvent<void const>::CreateFunction()};
        i.helper->call(params);
    }
    return CallbackInterface::Success;
}

bool SubscriptionQueue::full() {
    std::lock_guard<std::mutex> lock{queue_mutex_};
    return fullNoLock();
}

} // namespace roscan
