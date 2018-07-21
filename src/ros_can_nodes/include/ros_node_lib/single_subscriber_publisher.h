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

#ifndef ROSCAN_SINGLE_SUBSCRIBER_PUBLISHER_H
#define ROSCAN_SINGLE_SUBSCRIBER_PUBLISHER_H

#include "subscriber_link.h"
#include <ros/serialization.h>
#include <boost/utility.hpp>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

namespace roscan {

// Allows publication of a message to a single subscriber. Only available inside subscriber connection callbacks
class SingleSubscriberPublisher : public boost::noncopyable {
    public:
        SingleSubscriberPublisher(const SubscriberLinkPtr& link) : link_{link} {}
        ~SingleSubscriberPublisher() {}

        // Publish a message on the topic associated with this Publisher.
        // This version of publish will allow fast intra-process message-passing in the future,
        // so you may not mutate the message after it has been passed in here (since it will be
        // passed directly into a callback function)
        template <class M>
        void publish(const boost::shared_ptr<M const>& message) const { publish(*message); }

        // Publish a message on the topic associated with this Publisher.
        // This version of publish will allow fast intra-process message-passing in the future,
        // so you may not mutate the message after it has been passed in here (since it will be
        // passed directly into a callback function)
        template <class M>
        void publish(const boost::shared_ptr<M>& message) const { publish(*message); }

        // Publish a message on the topic associated with this Publisher.
        template <class M>
        void publish(const M& message) const { publish(ros::serialization::serializeMessage(message)); }

        // Returns the topic this publisher publishes on
        const std::string& getTopic() const { return link_->getTopic(); }

        // Returns the name of the subscriber node
        const std::string& getSubscriberName() const { return link_->getDestinationCallerID(); }

    private:
        void publish(const ros::SerializedMessage& m) const { link_->enqueueMessage(m, true, true); }

        SubscriberLinkPtr link_;
};
typedef boost::function<void(const SingleSubscriberPublisher&)> SubscriberStatusCallback;

} // namespace roscan

#endif // ROSCAN_SINGLE_SUBSCRIBER_PUBLISHER_H
