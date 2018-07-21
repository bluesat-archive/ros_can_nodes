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

#ifndef ROSCAN_CONNECTION_MANAGER_H
#define ROSCAN_CONNECTION_MANAGER_H

#include "common.h"
#include "rosdefs.h"
#include <ros/connection.h>
#include <ros/header.h>
#include <ros/transport/transport_tcp.h>
#include <ros/transport/transport_udp.h>
#include <cstdint>
#include <mutex>
#include <boost/signals2/connection.hpp>

namespace roscan {

class ConnectionManager {
    public:
        ConnectionManager(const RosCanNodePtr& node) : node_{node}, connection_id_counter_{0} {}
        ~ConnectionManager() { shutdown(); }

        // Get a new connection ID
        uint32_t getNewConnectionID();

        // Add a connection to be tracked by the node.  Will automatically remove them if they've been dropped, but from inside the ros thread
        void addConnection(const ros::ConnectionPtr& connection);

        void clear(ros::Connection::DropReason reason);

        uint32_t getTCPPort() const { return tcpserver_transport_->getServerPort(); }
        uint32_t getUDPPort() const { return udpserver_transport_->getServerPort(); }

        const ros::TransportTCPPtr& getTCPServerTransport() const { return tcpserver_transport_; }
        const ros::TransportUDPPtr& getUDPServerTransport() const { return udpserver_transport_; }

        void udprosIncomingConnection(const ros::TransportUDPPtr& transport, ros::Header& header);

        void start();
        void shutdown();

    private:
        void onConnectionDropped(const ros::ConnectionPtr& conn);

        // Remove any dropped connections from our list, causing them to be destroyed
        // They can't just be removed immediately when they're dropped because the ros
        // thread may still be using them (or more likely their transport)
        void removeDroppedConnections();

        bool onConnectionHeaderReceived(const ros::ConnectionPtr& conn, const ros::Header& header);
        void tcprosAcceptConnection(const ros::TransportTCPPtr& transport);

        RosCanNodePtr node_;

        std::set<ros::ConnectionPtr> connections_;
        std::vector<ros::ConnectionPtr> dropped_connections_;
        std::mutex connections_mutex_;
        std::mutex dropped_connections_mutex_;

        // The connection ID counter, used to assign unique ID to each inbound or
        // outbound connection. Access via getNewConnectionID()
        uint32_t connection_id_counter_;
        std::mutex connection_id_counter_mutex_;

        boost::signals2::connection poll_conn_;

        ros::TransportTCPPtr tcpserver_transport_;
        ros::TransportUDPPtr udpserver_transport_;

        const static int MAX_TCPROS_CONN_QUEUE = 100; // magic
};

} // namespace roscan

#endif // ROSCAN_CONNECTION_MANAGER_H
