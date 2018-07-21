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

#include "connection_manager.h"
#include "RosCanNode.hpp"
#include "network.h"
#include "poll_manager.h"
#include "transport_subscriber_link.h"
#include <ros/connection.h>
#include <boost/make_shared.hpp>

namespace roscan {

void ConnectionManager::start() {
    poll_conn_ = node_->poll_manager()->addPollThreadListener(boost::bind(&ConnectionManager::removeDroppedConnections, this));

    // Bring up the TCP listener socket
    tcpserver_transport_ = boost::make_shared<ros::TransportTCP>(&node_->poll_manager()->getPollSet());
    if (!tcpserver_transport_->listen(network::getTCPROSPort(), MAX_TCPROS_CONN_QUEUE, boost::bind(&ConnectionManager::tcprosAcceptConnection, this, _1))) {
        ROS_FATAL("Listen on port [%d] failed", network::getTCPROSPort());
    }

    // Bring up the UDP listener socket
    udpserver_transport_ = boost::make_shared<ros::TransportUDP>(&node_->poll_manager()->getPollSet());
    if (!udpserver_transport_->createIncoming(0, true)) {
        ROS_FATAL("Listen failed");
    }
}

void ConnectionManager::shutdown() {
    if (udpserver_transport_) {
        udpserver_transport_->close();
        udpserver_transport_.reset();
    }

    if (tcpserver_transport_) {
        tcpserver_transport_->close();
        tcpserver_transport_.reset();
    }

    node_->poll_manager()->removePollThreadListener(poll_conn_);
    clear(ros::Connection::Destructing);
}

void ConnectionManager::clear(ros::Connection::DropReason reason) {
    std::set<ros::ConnectionPtr> local_connections;
    {
        std::lock_guard<std::mutex> conn_lock{connections_mutex_};
        local_connections.swap(connections_);
    }

    for (const auto& conn: local_connections) {
        conn->drop(reason);
    }

    std::lock_guard<std::mutex> dropped_lock{dropped_connections_mutex_};
    dropped_connections_.clear();
}

uint32_t ConnectionManager::getNewConnectionID() {
    std::lock_guard<std::mutex> lock{connection_id_counter_mutex_};
    const auto ret = connection_id_counter_++;
    return ret;
}

void ConnectionManager::addConnection(const ros::ConnectionPtr& conn) {
    std::lock_guard<std::mutex> lock{connections_mutex_};

    connections_.insert(conn);
    conn->addDropListener(boost::bind(&ConnectionManager::onConnectionDropped, this, _1));
}

void ConnectionManager::onConnectionDropped(const ros::ConnectionPtr& conn) {
    std::lock_guard<std::mutex> lock{dropped_connections_mutex_};
    dropped_connections_.push_back(conn);
}

void ConnectionManager::removeDroppedConnections() {
    std::vector<ros::ConnectionPtr> local_dropped;
    {
        std::lock_guard<std::mutex> dropped_lock{dropped_connections_mutex_};
        dropped_connections_.swap(local_dropped);
    }

    std::lock_guard<std::mutex> conn_lock{connections_mutex_};

    for (const auto& conn: local_dropped) {
        connections_.erase(conn);
    }
}

void ConnectionManager::udprosIncomingConnection(const ros::TransportUDPPtr& transport, ros::Header& header) {
    const auto client_uri = std::string{""}; // TODO: transport->getClientURI();

    ros::ConnectionPtr conn{boost::make_shared<ros::Connection>()};
    addConnection(conn);

    conn->initialize(transport, true, NULL);
    onConnectionHeaderReceived(conn, header);
}

void ConnectionManager::tcprosAcceptConnection(const ros::TransportTCPPtr& transport) {
    const auto client_uri = transport->getClientURI();

    ros::ConnectionPtr conn{boost::make_shared<ros::Connection>()};
    addConnection(conn);

    conn->initialize(transport, true, boost::bind(&ConnectionManager::onConnectionHeaderReceived, this, _1, _2));
}

bool ConnectionManager::onConnectionHeaderReceived(const ros::ConnectionPtr& conn, const ros::Header& header) {
    auto ret = false;
    std::string val;
    if (header.getValue("topic", val)) {
        TransportSubscriberLinkPtr sub_link{boost::make_shared<TransportSubscriberLink>(node_)};
        sub_link->initialize(conn);
        ret = sub_link->handleHeader(header);
    // } else if (header.getValue("service", val)) {
    //     ros::ServiceClientLinkPtr link{boost::make_shared<ros::ServiceClientLink>()};
    //     link->initialize(conn);
    //     ret = link->handleHeader(header);
    } else {
        return false;
    }
    return ret;
}

} // namespace roscan
