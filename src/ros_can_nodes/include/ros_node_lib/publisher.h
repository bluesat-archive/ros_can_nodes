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

#ifndef ROSCAN_PUBLISHER_HANDLE_H
#define ROSCAN_PUBLISHER_HANDLE_H

#include "common.h"
#include "subscriber_callbacks.h"
#include <ros/serialization.h>
#include <string>
#include <boost/function.hpp>

namespace roscan {

// Manages an advertisement on a specific topic.
// A Publisher should always be created through a call to NodeHandle::advertise(), or copied from one
// that was. Once all copies of a specific
// Publisher go out of scope, any subscriber status callbacks associated with that handle will stop
// being called.  Once all Publishers for a given topic go out of scope the topic will be unadvertised.
class Publisher {
    public:
        Publisher() : unadvertised_{false} {}
        Publisher(const std::string& topic, const RosCanNodePtr& node, const std::string& md5sum, const std::string& datatype, const SubscriberCallbacksPtr& callbacks)
            : topic_{topic}, node_{node}, md5sum_{md5sum}, datatype_{datatype}, callbacks_{callbacks}, unadvertised_{false} {}
        ~Publisher() { unadvertise(); }

        // Publish a message on the topic associated with this Publisher.
        // This version of publish will allow fast intra-process message-passing in the future,
        // so you may not mutate the message after it has been passed in here (since it will be
        // passed directly into a callback function)
        template <typename M>
        void publish(const boost::shared_ptr<M>& message) const {
            using namespace ros::serialization;
            namespace mt = ros::message_traits;

            if (unadvertised_) {
                return;
            }
            ros::SerializedMessage m;
            m.type_info = &typeid(M);
            m.message = message;

            publish(boost::bind(serializeMessage<M>, boost::ref(*message)), m);
        }

        // Publish a message on the topic associated with this Publisher.
        template <typename M>
        void publish(const M& message) const {
            using namespace ros::serialization;
            namespace mt = ros::message_traits;

            if (unadvertised_) {
                return;
            }
            ros::SerializedMessage m;
            publish(boost::bind(serializeMessage<M>, boost::ref(message)), m);
        }

        // Shutdown the advertisement associated with this Publisher
        // This method usually does not need to be explicitly called, as automatic shutdown happens when
        // all copies of this Publisher go out of scope
        // This method overrides the automatic reference counted unadvertise, and does so immediately.
        // Note that if multiple advertisements were made through NodeHandle::advertise(), this will
        // only remove the one associated with this Publisher
        void shutdown();

        // Returns the topic that this Publisher will publish on.
        std::string getTopic() const;

        // Returns the number of subscribers that are currently connected to this Publisher
        uint32_t getNumSubscribers() const;

        // Returns whether or not this topic is latched
        bool isLatched() const;

    private:
        void publish(const boost::function<ros::SerializedMessage(void)>& serfunc, ros::SerializedMessage& m) const;
        void incrementSequence() const;

        void unadvertise();

        std::string topic_;
        RosCanNodePtr node_;
        std::string md5sum_;
        std::string datatype_;
        SubscriberCallbacksPtr callbacks_;
        bool unadvertised_;
};

} // namespace roscan

#endif // ROSCAN_PUBLISHER_HANDLE_H
