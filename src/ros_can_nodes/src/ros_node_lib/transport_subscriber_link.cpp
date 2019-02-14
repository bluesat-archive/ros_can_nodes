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

#include "ros_node_lib/RosNode.hpp"
#include "ros_node_lib/publication.h"
#include "ros_node_lib/transport_subscriber_link.h"
#include "ros_node_lib/topic_manager.h"
#include "ros_node_lib/connection_manager.h"
#include <ros/connection.h>
#include <ros/header.h>
#include <ros/transport/transport.h>

namespace roscan {

bool TransportSubscriberLink::initialize(const ros::ConnectionPtr& connection) {
    connection_ = connection;
    dropped_conn_ = connection_->addDropListener(boost::bind(&TransportSubscriberLink::onConnectionDropped, this, _1));
    return true;
}

bool TransportSubscriberLink::handleHeader(const ros::Header& header) {
    std::string topic;
    if (!header.getValue("topic", topic)) {
        std::string msg{"Header from subscriber did not have the required element: topic"};
        ROS_ERROR("%s", msg.c_str());
        connection_->sendHeaderError(msg);
        return false;
    }

    // This will get validated by validateHeader below
    std::string client_callerid;
    header.getValue("callerid", client_callerid);
    const auto pt = node_->topic_manager()->lookupPublication(topic);
    if (!pt) {
        std::string msg{"received a connection for a nonexistent topic [" + topic + "] from [" + connection_->getTransport()->getTransportInfo() + "] [" + client_callerid + "]."};
        connection_->sendHeaderError(msg);
        return false;
    }

    std::string error_msg;
    if (!pt->validateHeader(header, error_msg)) {
        connection_->sendHeaderError(error_msg);
        return false;
    }

    destination_caller_id_ = client_callerid;
    connection_id_ = node_->connection_manager()->getNewConnectionID();
    topic_ = pt->getName();
    parent_ = PublicationWPtr{pt};

    // Send back a success, with info
    std::map<std::string, std::string> m;
    m["type"] = pt->getDataType();
    m["md5sum"] = pt->getMD5Sum();
    m["message_definition"] = pt->getMessageDefinition();
    m["callerid"] = node_->getName();
    m["latching"] = pt->isLatching() ? "1" : "0";
    m["topic"] = topic_;
    connection_->writeHeader(m, boost::bind(&TransportSubscriberLink::onHeaderWritten, this, _1));

    pt->addSubscriberLink(shared_from_this());
    return true;
}

void TransportSubscriberLink::onConnectionDropped(const ros::ConnectionPtr& conn) {
    (void)conn;
    if (const auto parent = parent_.lock()) {
        parent->removeSubscriberLink(shared_from_this());
    }
}

void TransportSubscriberLink::onHeaderWritten(const ros::ConnectionPtr& conn) {
    (void)conn;
    header_written_ = true;
    startMessageWrite(true);
}

void TransportSubscriberLink::onMessageWritten(const ros::ConnectionPtr& conn) {
    (void)conn;
    writing_message_ = false;
    startMessageWrite(true);
}

void TransportSubscriberLink::startMessageWrite(const bool immediate_write) {
    ros::SerializedMessage m;

    {
        std::lock_guard<std::mutex> lock{outbox_mutex_};
        if (writing_message_ || !header_written_) {
            return;
        }

        if (!outbox_.empty()) {
            writing_message_ = true;
            m = outbox_.front();
            outbox_.pop();
        }
    }

    if (m.num_bytes > 0) {
        connection_->write(m.buf, m.num_bytes, boost::bind(&TransportSubscriberLink::onMessageWritten, this, _1), immediate_write);
    }
}

void TransportSubscriberLink::enqueueMessage(const ros::SerializedMessage& m, const bool ser, const bool nocopy) {
    (void)nocopy;
    if (!ser) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock{outbox_mutex_};

        int max_queue = 0;
        if (const auto parent = parent_.lock()) {
            max_queue = parent->getMaxQueue();
        }

        if (max_queue > 0 && (int)outbox_.size() >= max_queue) {
            outbox_.pop(); // toss out the oldest thing in the queue to make room for us
            queue_full_ = true;
        } else {
            queue_full_ = false;
        }
        outbox_.push(m);
    }

    startMessageWrite(false);

    ++stats_.messages_sent_;
    stats_.bytes_sent_ += m.num_bytes;
    stats_.message_data_sent_ += m.num_bytes;
}

void TransportSubscriberLink::drop() {
    // Only drop the connection if it's not already sending a header error
    // If it is, it will automatically drop itself
    if (connection_->isSendingHeaderError()) {
        connection_->removeDropListener(dropped_conn_);
    } else {
        connection_->drop(ros::Connection::Destructing);
    }
}

} // namespace roscan
