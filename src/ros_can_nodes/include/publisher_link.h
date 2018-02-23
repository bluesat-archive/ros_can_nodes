/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
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

#ifndef ROSCAN_PUBLISHER_LINK_H
#define ROSCAN_PUBLISHER_LINK_H

#include "RosCanNode.h"
#include "subscription.h"
#include "common.h"
#include <ros/connection.h>
#include <ros/header.h>
#include <ros/transport_hints.h>
#include <boost/shared_array.hpp>
#include <boost/thread/mutex.hpp>

namespace ros {

class Connection;
typedef boost::shared_ptr<Connection> ConnectionPtr;

} // namespace ros

namespace roscan {

class RosCanNode;
typedef boost::shared_ptr<RosCanNode> RosCanNodePtr;

class Subscription;
typedef boost::shared_ptr<Subscription> SubscriptionPtr;
typedef boost::weak_ptr<Subscription> SubscriptionWPtr;

// Handles a connection to a single publisher on a given topic. Receives messages from a publisher
// and hands them off to its parent Subscription
class PublisherLink : public boost::enable_shared_from_this<PublisherLink> {
    public:
        class Stats {
            public:
                uint64_t bytes_received_, messages_received_, drops_;
                Stats() : bytes_received_(0), messages_received_(0), drops_(0) {}
        };

        PublisherLink(const RosCanNodePtr& node, const SubscriptionPtr& parent, const std::string& xmlrpc_uri, const ros::TransportHints& transport_hints)
            : node_(node), parent_(parent), connection_id_(0), publisher_xmlrpc_uri_(xmlrpc_uri), transport_hints_(transport_hints), latched_(false) {}
        
        virtual ~PublisherLink() {}

        const Stats& getStats() { return stats_; }
        const std::string& getPublisherXMLRPCURI() { return publisher_xmlrpc_uri_; }
        int getConnectionID() const { return connection_id_; }
        const std::string& getCallerID() { return caller_id_; }
        bool isLatched() { return latched_; }

        bool setHeader(const ros::Header& header);

        // Handles handing off a received message to the subscription, where it will be deserialized and called back
        virtual void handleMessage(const ros::SerializedMessage& m, bool ser, bool nocopy) = 0;
        virtual std::string getTransportType() = 0;
        virtual std::string getTransportInfo() = 0;
        virtual void drop() = 0;

        const std::string& getMD5Sum();

    protected:
        RosCanNodePtr node_;

        SubscriptionWPtr parent_;
        unsigned int connection_id_;
        std::string publisher_xmlrpc_uri_;

        Stats stats_;

        ros::TransportHints transport_hints_;

        bool latched_;
        std::string caller_id_;
        ros::Header header_;
        std::string md5sum_;
};

} // namespace roscan

#endif // ROSCAN_PUBLISHER_LINK_H
