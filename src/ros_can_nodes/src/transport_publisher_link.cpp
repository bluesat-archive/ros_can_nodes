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

#include "common.h"
#include "RosCanNode.h"
#include "publisher_link.h"
#include "transport_publisher_link.h"
#include "connection_manager.h"
#include "poll_manager.h"
#include "subscription.h"
#include <ros/callback_queue.h>
#include <ros/connection.h>
#include <ros/file_log.h>
#include <ros/header.h>
#include <ros/internal_timer_manager.h>
#include <ros/timer_manager.h>
#include <ros/transport/transport.h>
#include <ros/transport/transport_tcp.h>

namespace roscan {

TransportPublisherLink::~TransportPublisherLink() {
    dropping_ = true;

    if (retry_timer_handle_ != -1) {
        ros::getInternalTimerManager()->remove(retry_timer_handle_);
    }

    connection_->drop(ros::Connection::Destructing);
}

bool TransportPublisherLink::initialize(const ros::ConnectionPtr& connection) {
    connection_ = connection;
    // slot_type is used to automatically track the TransporPublisherLink class' existence
    // and disconnect when this class' reference count is decremented to 0. It increments
    // then decrements the shared_from_this reference count around calls to the
    // onConnectionDropped function, preventing a coredump in the middle of execution.
    connection_->addDropListener(ros::Connection::DropSignal::slot_type(&TransportPublisherLink::onConnectionDropped, this, _1, _2).track(shared_from_this()));

    if (connection_->getTransport()->requiresHeader()) {
        connection_->setHeaderReceivedCallback(boost::bind(&TransportPublisherLink::onHeaderReceived, this, _1, _2));

        SubscriptionPtr parent = parent_.lock();

        ros::M_string header;
        header["topic"] = parent->getName();
        header["md5sum"] = parent->md5sum();
        header["callerid"] = node_->getName();
        header["type"] = parent->datatype();
        header["tcp_nodelay"] = transport_hints_.getTCPNoDelay() ? "1" : "0";
        connection_->writeHeader(header, boost::bind(&TransportPublisherLink::onHeaderWritten, this, _1));
    } else {
        connection_->read(4, boost::bind(&TransportPublisherLink::onMessageLength, this, _1, _2, _3, _4));
    }

    return true;
}

void TransportPublisherLink::drop() {
    dropping_ = true;
    connection_->drop(ros::Connection::Destructing);

    if (SubscriptionPtr parent = parent_.lock()) {
        parent->removePublisherLink(shared_from_this());
    }
}

void TransportPublisherLink::onHeaderWritten(const ros::ConnectionPtr& conn) {
    (void)conn;
    // Do nothing
}

bool TransportPublisherLink::onHeaderReceived(const ros::ConnectionPtr& conn, const ros::Header& header) {
    (void)conn;
    ROS_ASSERT(conn == connection_);

    if (!setHeader(header)) {
        drop();
        return false;
    }

    if (retry_timer_handle_ != -1) {
        ros::getInternalTimerManager()->remove(retry_timer_handle_);
        retry_timer_handle_ = -1;
    }

    connection_->read(4, boost::bind(&TransportPublisherLink::onMessageLength, this, _1, _2, _3, _4));

    return true;
}

void TransportPublisherLink::onMessageLength(const ros::ConnectionPtr& conn, const boost::shared_array<uint8_t>& buffer, uint32_t size, bool success) {
    (void)conn;
    (void)size;
    if (retry_timer_handle_ != -1) {
        ros::getInternalTimerManager()->remove(retry_timer_handle_);
        retry_timer_handle_ = -1;
    }

    if (!success) {
        if (connection_)
            connection_->read(4, boost::bind(&TransportPublisherLink::onMessageLength, this, _1, _2, _3, _4));
        return;
    }

    ROS_ASSERT(conn == connection_);
    ROS_ASSERT(size == 4);

    uint32_t len = *((uint32_t*)buffer.get());

    if (len > 1000000000) {
        ROS_ERROR("a message of over a gigabyte was "
                  "predicted in tcpros. that seems highly "
                  "unlikely, so I'll assume protocol "
                  "synchronization is lost.");
        drop();
        return;
    }

    connection_->read(len, boost::bind(&TransportPublisherLink::onMessage, this, _1, _2, _3, _4));
}

void TransportPublisherLink::onMessage(const ros::ConnectionPtr& conn, const boost::shared_array<uint8_t>& buffer, uint32_t size, bool success) {
    if (!success && !conn)
        return;

    ROS_ASSERT(conn == connection_);

    if (success) {
        handleMessage(ros::SerializedMessage(buffer, size), true, false);
    }

    if (success || !connection_->getTransport()->requiresHeader()) {
        connection_->read(4, boost::bind(&TransportPublisherLink::onMessageLength, this, _1, _2, _3, _4));
    }
}

void TransportPublisherLink::onRetryTimer(const ros::SteadyTimerEvent&) {
    if (dropping_) {
        return;
    }

    if (needs_retry_ && ros::SteadyTime::now() > next_retry_) {
        retry_period_ = std::min(retry_period_ * 2, ros::WallDuration(20));
        needs_retry_ = false;
        SubscriptionPtr parent = parent_.lock();
        // TODO: support retry on more than just TCP
        // For now, since UDP does not have a heartbeat, we do not attempt to retry
        // UDP connections since an error there likely means some invalid operation has
        // happened.
        if (connection_->getTransport()->getType() == std::string("TCPROS")) {
            std::string topic = parent ? parent->getName() : "unknown";

            ros::TransportTCPPtr old_transport = boost::dynamic_pointer_cast<ros::TransportTCP>(connection_->getTransport());
            ROS_ASSERT(old_transport);
            const std::string& host = old_transport->getConnectedHost();
            int port = old_transport->getConnectedPort();

            ROSCPP_CONN_LOG_DEBUG("Retrying connection to [%s:%d] for topic [%s]", host.c_str(), port, topic.c_str());

            ros::TransportTCPPtr transport(boost::make_shared<ros::TransportTCP>(&node_->poll_manager()->getPollSet()));
            if (transport->connect(host, port)) {
                ros::ConnectionPtr connection(boost::make_shared<ros::Connection>());
                connection->initialize(transport, false, ros::HeaderReceivedFunc());
                initialize(connection);

                node_->connection_manager()->addConnection(connection);
            } else {
                ROSCPP_CONN_LOG_DEBUG("connect() failed when retrying connection to [%s:%d] for topic [%s]", host.c_str(), port, topic.c_str());
            }
        } else if (parent) {
            parent->removePublisherLink(shared_from_this());
        }
    }
}

void TransportPublisherLink::onConnectionDropped(const ros::ConnectionPtr& conn, ros::Connection::DropReason reason) {
    (void)conn;
    if (dropping_) {
        return;
    }

    ROS_ASSERT(conn == connection_);

    SubscriptionPtr parent = parent_.lock();

    if (reason == ros::Connection::TransportDisconnect) {
        std::string topic = parent ? parent->getName() : "unknown";

        ROSCPP_CONN_LOG_DEBUG("Connection to publisher [%s] to topic [%s] dropped", connection_->getTransport()->getTransportInfo().c_str(), topic.c_str());

        ROS_ASSERT(!needs_retry_);
        needs_retry_ = true;
        next_retry_ = ros::SteadyTime::now() + retry_period_;

        if (retry_timer_handle_ == -1) {
            retry_period_ = ros::WallDuration(0.1);
            next_retry_ = ros::SteadyTime::now() + retry_period_;
            // shared_from_this() shared_ptr is used to ensure TransportPublisherLink is not
            // destroyed in the middle of onRetryTimer execution
            retry_timer_handle_ = ros::getInternalTimerManager()->add(ros::WallDuration(retry_period_),
                                                                 boost::bind(&TransportPublisherLink::onRetryTimer, this, _1), node_->getInternalCallbackQueue().get(),
                                                                 shared_from_this(), false);
        } else {
            ros::getInternalTimerManager()->setPeriod(retry_timer_handle_, retry_period_);
        }
    } else {
        drop();
    }
}

void TransportPublisherLink::handleMessage(const ros::SerializedMessage& m, bool ser, bool nocopy) {
    stats_.bytes_received_ += m.num_bytes;
    stats_.messages_received_++;

    SubscriptionPtr parent = parent_.lock();

    if (parent) {
        stats_.drops_ += parent->handleMessage(m, ser, nocopy, getConnection()->getHeader().getValues(), shared_from_this());
    }
}

std::string TransportPublisherLink::getTransportType() {
    return connection_->getTransport()->getType();
}

std::string TransportPublisherLink::getTransportInfo() {
    return connection_->getTransport()->getTransportInfo();
}

} // namespace roscan
