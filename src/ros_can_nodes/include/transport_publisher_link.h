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

#ifndef ROSCAN_TRANSPORT_PUBLISHER_LINK_H
#define ROSCAN_TRANSPORT_PUBLISHER_LINK_H

#include "RosCanNode.h"
#include "publisher_link.h"
#include <ros/common.h>
#include <ros/connection.h>

namespace ros {

class Connection;
typedef boost::shared_ptr<ros::Connection> ConnectionPtr;

} // namespace ros

namespace roscan {

class RosCanNode;
typedef boost::shared_ptr<RosCanNode> RosCanNodePtr;

class Subscription;
typedef boost::shared_ptr<Subscription> SubscriptionPtr;
typedef boost::weak_ptr<Subscription> SubscriptionWPtr;

struct SteadyTimerEvent;

class PublisherLink;
typedef boost::shared_ptr<PublisherLink> PublisherLinkPtr;

class TransportPublisherLink;
typedef boost::shared_ptr<TransportPublisherLink> TransportPublisherLinkPtr;

// Handles a connection to a single publisher on a given topic.  Receives messages from a publisher
// and hands them off to its parent Subscription
class TransportPublisherLink : public PublisherLink {
    public:
        TransportPublisherLink(const RosCanNodePtr& node, const SubscriptionPtr& parent, const std::string& xmlrpc_uri, const ros::TransportHints& transport_hints);
        virtual ~TransportPublisherLink();

        bool initialize(const ros::ConnectionPtr& connection);

        const ros::ConnectionPtr& getConnection() { return connection_; }

        virtual std::string getTransportType();
        virtual std::string getTransportInfo();
        virtual void drop();

    private:
        void onConnectionDropped(const ros::ConnectionPtr& conn, ros::Connection::DropReason reason);
        bool onHeaderReceived(const ros::ConnectionPtr& conn, const ros::Header& header);

        // Handles handing off a received message to the subscription, where it will be deserialized and called back
        virtual void handleMessage(const ros::SerializedMessage& m, bool ser, bool nocopy);

        void onHeaderWritten(const ros::ConnectionPtr& conn);
        void onMessageLength(const ros::ConnectionPtr& conn, const boost::shared_array<uint8_t>& buffer, uint32_t size, bool success);
        void onMessage(const ros::ConnectionPtr& conn, const boost::shared_array<uint8_t>& buffer, uint32_t size, bool success);

        void onRetryTimer(const ros::SteadyTimerEvent&);

        ros::ConnectionPtr connection_;

        int32_t retry_timer_handle_;
        bool needs_retry_;
        ros::WallDuration retry_period_;
        ros::SteadyTime next_retry_;
        bool dropping_;
};

} // namespace roscan

#endif // ROSCAN_TRANSPORT_PUBLISHER_LINK_H
