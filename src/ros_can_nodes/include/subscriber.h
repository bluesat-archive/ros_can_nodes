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

#ifndef ROSCAN_SUBSCRIBER_HANDLE_H
#define ROSCAN_SUBSCRIBER_HANDLE_H

#include "RosCanNode.h"
#include "common.h"
#include <ros/subscription_callback_helper.h>

namespace roscan {

class RosCanNode;
typedef boost::shared_ptr<RosCanNode> RosCanNodePtr;

class Subscriber;
typedef std::vector<Subscriber> V_Subscriber;

// Manages an subscription callback on a specific topic.
// A Subscriber should always be created through a call to NodeHandle::subscribe(), or copied from one
// that was. Once all copies of a specific
// Subscriber go out of scope, the subscription callback associated with that handle will stop
// being called.  Once all Subscriber for a given topic go out of scope the topic will be unsubscribed.
class Subscriber {
    public:
        Subscriber() {}
        Subscriber(const std::string& topic, const RosCanNodePtr& node, const ros::SubscriptionCallbackHelperPtr& helper)
            : topic_(topic), node_(node), helper_(helper), unsubscribed_(false) {}
        ~Subscriber() { unsubscribed_ = true; }

        // Unsubscribe the callback associated with this Subscriber
        // This method usually does not need to be explicitly called, as automatic shutdown happens when
        // all copies of this Subscriber go out of scope
        // This method overrides the automatic reference counted unsubscribe, and immediately
        // unsubscribes the callback associated with this Subscriber
        void shutdown();

        std::string getTopic() const;

        // Returns the number of publishers this subscriber is connected to
        uint32_t getNumPublishers() const;

    private:
        std::string topic_;
        RosCanNodePtr node_;
        ros::SubscriptionCallbackHelperPtr helper_;
        bool unsubscribed_;
};

} // namespace roscan

#endif // ROSCAN_SUBSCRIBER_HANDLE_H
