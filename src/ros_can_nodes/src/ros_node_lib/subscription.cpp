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

#include "subscription.h"
#include "RosCanNode.hpp"
#include "connection_manager.h"
#include "network.h"
#include "poll_manager.h"
#include "publication.h"
#include "publisher_link.h"
#include "transport_publisher_link.h"
#include "intraprocess_publisher_link.h"
#include "intraprocess_subscriber_link.h"
#include "callback_queue_interface.h"
#include "subscription_queue.h"
#include <ros/connection.h>
#include <ros/message_deserializer.h>
#include <ros/subscription_callback_helper.h>
#include <ros/transport/transport_tcp.h>
#include <ros/transport/transport_udp.h>
#include <ros/transport_hints.h>

using XmlRpc::XmlRpcValue;

namespace roscan {

Subscription::~Subscription() {
    pending_connections_.clear();
    callbacks_.clear();
}

void Subscription::shutdown() {
    {
        std::lock_guard<std::mutex> lock{shutdown_mutex_};
        shutting_down_ = true;
    }
    drop();
}

XmlRpcValue Subscription::getStats() {
    XmlRpcValue stats;
    stats[0] = name_;
    XmlRpcValue conn_data;
    conn_data.setSize(0);

    std::lock_guard<std::mutex> lock{publisher_links_mutex_};

    uint32_t cidx = 0;
    for (const auto& c: publisher_links_) {
        const auto& s = c->getStats();
        conn_data[cidx][0] = c->getConnectionID();
        conn_data[cidx][1] = (int)s.bytes_received_;
        conn_data[cidx][2] = (int)s.messages_received_;
        conn_data[cidx][3] = (int)s.drops_;
        conn_data[cidx][4] = 0; // figure out something for this. not sure.
    }
    stats[1] = conn_data;
    return stats;
}

// [(connection_id, publisher_xmlrpc_uri, direction, transport, topic_name, connected, connection_info_string)*]
// e.g. [(1, 'http://host:54893/', 'i', 'TCPROS', '/chatter', 1, 'TCPROS connection on port 59746 to [host:34318 on socket 11]')]
void Subscription::getInfo(XmlRpc::XmlRpcValue& info) {
    std::lock_guard<std::mutex> lock{publisher_links_mutex_};

    for (const auto& c: publisher_links_) {
        XmlRpcValue curr_info;
        curr_info[0] = (int)c->getConnectionID();
        curr_info[1] = c->getPublisherXMLRPCURI();
        curr_info[2] = "i";
        curr_info[3] = c->getTransportType();
        curr_info[4] = name_;
        curr_info[5] = true; // For length compatibility with rospy
        curr_info[6] = c->getTransportInfo();
        info[info.size()] = curr_info;
    }
}

uint32_t Subscription::getNumPublishers() {
    std::lock_guard<std::mutex> lock{publisher_links_mutex_};
    return (uint32_t)publisher_links_.size();
}

void Subscription::drop() {
    if (!dropped_) {
        dropped_ = true;
        dropAllConnections();
    }
}

void Subscription::dropAllConnections() {
    // Swap our subscribers list with a local one so we can only lock for a short period of time, because a
    // side effect of our calling drop() on connections can be re-locking the subscribers mutex
    V_PublisherLink localsubscribers;

    {
        std::lock_guard<std::mutex> lock{publisher_links_mutex_};
        localsubscribers.swap(publisher_links_);
    }

    for (const auto& s: localsubscribers) {
        s->drop();
    }
}

void Subscription::addLocalConnection(const PublicationPtr& pub) {
    std::lock_guard<std::mutex> lock{publisher_links_mutex_};
    if (dropped_) {
        return;
    }

    IntraProcessPublisherLinkPtr pub_link{boost::make_shared<IntraProcessPublisherLink>(node_, shared_from_this(), node_->xmlrpc_manager()->getServerURI(), transport_hints_)};
    IntraProcessSubscriberLinkPtr sub_link{boost::make_shared<IntraProcessSubscriberLink>(node_, pub)};
    pub_link->setPublisher(sub_link);
    sub_link->setSubscriber(pub_link);

    addPublisherLink(pub_link);
    pub->addSubscriberLink(sub_link);
}

bool urisEqual(const std::string& uri1, const std::string& uri2) {
    std::string host1, host2;
    uint32_t port1 = 0, port2 = 0;
    network::splitURI(uri1, host1, port1);
    network::splitURI(uri2, host2, port2);
    return port1 == port2 && host1 == host2;
}

bool Subscription::pubUpdate(const std::vector<std::string>& new_pubs) {
    std::lock_guard<std::mutex> lock{shutdown_mutex_};

    if (shutting_down_ || dropped_) {
        return false;
    }

    auto retval = true;

    {
        std::stringstream ss;

        for (const auto& up: new_pubs) {
            ss << up << ", ";
        }

        ss << " already have these connections: ";
        {
            std::lock_guard<std::mutex> lock{publisher_links_mutex_};
            for (const auto& spc: publisher_links_) {
                ss << spc->getPublisherXMLRPCURI() << ", ";
            }
        }

        std::lock_guard<std::mutex> lock{pending_connections_mutex_};
        for (const auto& c: pending_connections_) {
            ss << c->getRemoteURI() << ", ";
        }
    }

    std::vector<std::string> additions;
    V_PublisherLink subtractions;
    V_PublisherLink to_add;
    // could use the STL set operations... but these sets are so small
    // it doesn't really matter.
    {
        std::lock_guard<std::mutex> lock{publisher_links_mutex_};

        for (const auto& spc: publisher_links_) {
            auto found = false;
            for (const auto& up: new_pubs) {
                if (urisEqual(spc->getPublisherXMLRPCURI(), up)) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                subtractions.push_back(spc);
            }
        }

        for (const auto& up: new_pubs) {
            auto found = false;
            for (const auto& spc: publisher_links_) {
                if (urisEqual(up, spc->getPublisherXMLRPCURI())) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                std::lock_guard<std::mutex> lock{pending_connections_mutex_};
                for (const auto& c: pending_connections_) {
                    if (urisEqual(up, c->getRemoteURI())) {
                        found = true;
                        break;
                    }
                }
            }
            if (!found) {
                additions.push_back(up);
            }
        }
    }

    for (const auto& link: subtractions) {
        if (link->getPublisherXMLRPCURI() != node_->xmlrpc_manager()->getServerURI()) {
            link->drop();
        }
    }

    for (const auto& s: additions) {
        // this function should never negotiate a self-subscription
        if (node_->xmlrpc_manager()->getServerURI() != s) {
            retval &= negotiateConnection(s);
        }
    }
    return retval;
}

bool Subscription::negotiateConnection(const std::string& xmlrpc_uri) {
    XmlRpcValue tcpros_array, protos_array, params;
    XmlRpcValue udpros_array;
    ros::TransportUDPPtr udp_transport;
    auto protos = 0;
    auto transports = transport_hints_.getTransports();
    if (transports.empty()) {
        transport_hints_.reliable();
        transports = transport_hints_.getTransports();
    }
    for (const auto& t: transports) {
        if (t == "UDP") {
            auto max_datagram_size = transport_hints_.getMaxDatagramSize();
            udp_transport = boost::make_shared<ros::TransportUDP>(&node_->poll_manager()->getPollSet());
            if (!max_datagram_size) {
                max_datagram_size = udp_transport->getMaxDatagramSize();
            }
            udp_transport->createIncoming(0, false);
            udpros_array[0] = "UDPROS";
            std::map<std::string, std::string> m;
            m["topic"] = getName();
            m["md5sum"] = md5sum();
            m["callerid"] = node_->getName();
            m["type"] = datatype();
            boost::shared_array<uint8_t> buffer;
            uint32_t len;
            ros::Header::write(m, buffer, len);
            XmlRpcValue v(buffer.get(), len);
            udpros_array[1] = v;
            udpros_array[2] = network::getHost();
            udpros_array[3] = udp_transport->getServerPort();
            udpros_array[4] = max_datagram_size;

            protos_array[protos++] = udpros_array;
        } else if (t == "TCP") {
            tcpros_array[0] = std::string{"TCPROS"};
            protos_array[protos++] = tcpros_array;
        } else {
            ROS_WARN("Unsupported transport type hinted: %s, skipping", t.c_str());
        }
    }
    params[0] = node_->getName();
    params[1] = name_;
    params[2] = protos_array;
    std::string peer_host;
    uint32_t peer_port;
    if (!network::splitURI(xmlrpc_uri, peer_host, peer_port)) {
        ROS_ERROR("Bad xml-rpc URI: [%s]", xmlrpc_uri.c_str());
        return false;
    }

    XmlRpc::XmlRpcClient *c = new XmlRpc::XmlRpcClient{peer_host.c_str(), (int)peer_port, "/"};

    // Initiate the negotiation.  We'll come back and check on it later.
    if (!c->executeNonBlock("requestTopic", params)) {
        delete c;
        if (udp_transport) {
            udp_transport->close();
        }
        return false;
    }

    // The PendingConnectionPtr takes ownership of c, and will delete it on
    // destruction.
    PendingConnectionPtr conn{boost::make_shared<PendingConnection>(c, udp_transport, shared_from_this(), xmlrpc_uri)};

    node_->xmlrpc_manager()->addASyncConnection(conn);
    // Put this connection on the list that we'll look at later.
    {
        std::lock_guard<std::mutex> pending_connections_lock{pending_connections_mutex_};
        pending_connections_.insert(conn);
    }
    return true;
}

void closeTransport(const ros::TransportUDPPtr& trans) {
    if (trans) {
        trans->close();
    }
}

void Subscription::pendingConnectionDone(const PendingConnectionPtr& conn, XmlRpcValue& result) {
    std::lock_guard<std::mutex> lock{shutdown_mutex_};
    if (shutting_down_ || dropped_) {
        return;
    }

    {
        std::lock_guard<std::mutex> pending_connections_lock{pending_connections_mutex_};
        pending_connections_.erase(conn);
    }

    ros::TransportUDPPtr udp_transport;

    const auto peer_host = conn->getClient()->getHost();
    const auto peer_port = conn->getClient()->getPort();
    std::stringstream ss;
    ss << "http://" << peer_host << ":" << peer_port << "/";
    const auto xmlrpc_uri = ss.str();
    udp_transport = conn->getUDPTransport();

    XmlRpc::XmlRpcValue proto;
    if (!node_->xmlrpc_manager()->validateXmlrpcResponse("requestTopic", result, proto)) {
        closeTransport(udp_transport);
        return;
    }

    if (proto.size() == 0) {
        closeTransport(udp_transport);
        return;
    }

    if (proto.getType() != XmlRpcValue::TypeArray) {
        closeTransport(udp_transport);
        return;
    }
    if (proto[0].getType() != XmlRpcValue::TypeString) {
        closeTransport(udp_transport);
        return;
    }

    std::string proto_name = proto[0];
    if (proto_name == "TCPROS") {
        if (proto.size() != 3 || proto[1].getType() != XmlRpcValue::TypeString || proto[2].getType() != XmlRpcValue::TypeInt) {
            return;
        }
        std::string pub_host = proto[1];
        int pub_port = proto[2];

        ros::TransportTCPPtr transport{boost::make_shared<ros::TransportTCP>(&node_->poll_manager()->getPollSet())};
        if (transport->connect(pub_host, pub_port)) {
            ros::ConnectionPtr connection{boost::make_shared<ros::Connection>()};
            TransportPublisherLinkPtr pub_link{boost::make_shared<TransportPublisherLink>(node_, shared_from_this(), xmlrpc_uri, transport_hints_)};

            connection->initialize(transport, false, ros::HeaderReceivedFunc());
            pub_link->initialize(connection);

            node_->connection_manager()->addConnection(connection);

            std::lock_guard<std::mutex> lock{publisher_links_mutex_};
            addPublisherLink(pub_link);
        }
    } else if (proto_name == "UDPROS") {
        if (proto.size() != 6 ||
            proto[1].getType() != XmlRpcValue::TypeString ||
            proto[2].getType() != XmlRpcValue::TypeInt ||
            proto[3].getType() != XmlRpcValue::TypeInt ||
            proto[4].getType() != XmlRpcValue::TypeInt ||
            proto[5].getType() != XmlRpcValue::TypeBase64) {
            closeTransport(udp_transport);
            return;
        }
        std::string pub_host = proto[1];
        int pub_port = proto[2];
        int conn_id = proto[3];
        int max_datagram_size = proto[4];
        std::vector<char> header_bytes = proto[5];
        boost::shared_array<uint8_t> buffer{new uint8_t[header_bytes.size()]};
        memcpy(buffer.get(), &header_bytes[0], header_bytes.size());
        ros::Header h;
        std::string err;
        if (!h.parse(buffer, header_bytes.size(), err)) {
            closeTransport(udp_transport);
            return;
        }

        std::string error_msg;
        if (h.getValue("error", error_msg)) {
            closeTransport(udp_transport);
            return;
        }

        TransportPublisherLinkPtr pub_link{boost::make_shared<TransportPublisherLink>(node_, shared_from_this(), xmlrpc_uri, transport_hints_)};
        if (pub_link->setHeader(h)) {
            ros::ConnectionPtr connection{boost::make_shared<ros::Connection>()};
            connection->initialize(udp_transport, false, NULL);
            connection->setHeader(h);
            pub_link->initialize(connection);

            node_->connection_manager()->addConnection(connection);

            std::lock_guard<std::mutex> lock{publisher_links_mutex_};
            addPublisherLink(pub_link);
        } else {
            closeTransport(udp_transport);
            return;
        }
    }
}

uint32_t Subscription::handleMessage(const ros::SerializedMessage& m, const bool ser, const bool nocopy, const boost::shared_ptr<std::map<std::string, std::string>>& connection_header, const PublisherLinkPtr& link) {
    std::lock_guard<std::mutex> lock{callbacks_mutex_};

    uint32_t drops = 0;

    // Cache the deserializers by type info.  If all the subscriptions are the same type this has the same performance as before.  If
    // there are subscriptions with different C++ type (but same ROS message type), this now works correctly rather than passing
    // garbage to the messages with different C++ types than the first one.
    cached_deserializers_.clear();

    const auto receipt_time = ros::Time::now();

    for (const auto& info: callbacks_) {
        const std::type_info *ti = &info->helper_->getTypeInfo();

        if ((nocopy && m.type_info && *ti == *m.type_info) || (ser && (!m.type_info || *ti != *m.type_info))) {
            ros::MessageDeserializerPtr deserializer;

            for (const auto& des: cached_deserializers_) {
                if (*des.first == *ti) {
                    deserializer = des.second;
                    break;
                }
            }

            if (!deserializer) {
                deserializer = boost::make_shared<ros::MessageDeserializer>(info->helper_, m, connection_header);
                cached_deserializers_.push_back(std::make_pair(ti, deserializer));
            }

            auto was_full = false;
            auto nonconst_need_copy = false;
            if (callbacks_.size() > 1) {
                nonconst_need_copy = true;
            }

            info->subscription_queue_->push(info->helper_, deserializer, info->has_tracked_object_, info->tracked_object_, nonconst_need_copy, receipt_time, &was_full);

            if (was_full) {
                ++drops;
            } else {
                info->callback_queue_->addCallback(info->subscription_queue_, (uint64_t)info.get());
            }
        }
    }

    // measure statistics
    //statistics_.callback(connection_header, name_, link->getCallerID(), m, link->getStats().bytes_received_, receipt_time, drops > 0);

    // If this link is latched, store off the message so we can immediately pass it to new subscribers later
    if (link->isLatched()) {
        LatchInfo li;
        li.connection_header = connection_header;
        li.link = link;
        li.message = m;
        li.receipt_time = receipt_time;
        latched_messages_[link] = li;
    }
    cached_deserializers_.clear();
    return drops;
}

bool Subscription::addCallback(const ros::SubscriptionCallbackHelperPtr& helper, const std::string& md5sum, CallbackQueueInterface *const queue, const int32_t queue_size, const ros::VoidConstPtr& tracked_object, const bool allow_concurrent_callbacks) {
    //statistics_.init(helper);

    // Decay to a real type as soon as we have a subscriber with a real type
    {
        std::lock_guard<std::mutex> lock{md5sum_mutex_};
        if (md5sum_ == "*" && md5sum != "*") {
            md5sum_ = md5sum;
        }
    }

    if (md5sum != "*" && md5sum != this->md5sum()) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock{callbacks_mutex_};

        CallbackInfoPtr info{boost::make_shared<CallbackInfo>()};
        info->helper_ = helper;
        info->callback_queue_ = queue;
        info->subscription_queue_ = boost::make_shared<SubscriptionQueue>(name_, queue_size, allow_concurrent_callbacks);
        info->tracked_object_ = tracked_object;
        info->has_tracked_object_ = false;
        if (tracked_object) {
            info->has_tracked_object_ = true;
        }

        if (!helper->isConst()) {
            ++nonconst_callbacks_;
        }

        callbacks_.push_back(info);
        cached_deserializers_.reserve(callbacks_.size());

        // if we have any latched links, we need to immediately schedule callbacks
        if (!latched_messages_.empty()) {
            std::lock_guard<std::mutex> lock{publisher_links_mutex_};

            for (const auto& link: publisher_links_) {
                if (link->isLatched()) {
                    const auto des_it = latched_messages_.find(link);
                    if (des_it != latched_messages_.end()) {
                        const LatchInfo& latch_info = des_it->second;
                        ros::MessageDeserializerPtr des{boost::make_shared<ros::MessageDeserializer>(helper, latch_info.message, latch_info.connection_header)};
                        auto was_full = false;
                        info->subscription_queue_->push(info->helper_, des, info->has_tracked_object_, info->tracked_object_, true, latch_info.receipt_time, &was_full);
                        if (!was_full) {
                            info->callback_queue_->addCallback(info->subscription_queue_, (uint64_t)info.get());
                        }
                    }
                }
            }
        }
    }
    return true;
}

void Subscription::removeCallback(const ros::SubscriptionCallbackHelperPtr& helper) {
    CallbackInfoPtr info;
    {
        std::lock_guard<std::mutex> cbs_lock{callbacks_mutex_};
        const auto it = std::find_if(callbacks_.cbegin(), callbacks_.cend(), [&helper](const auto& c){ return helper == c->helper_; });
        if (it != callbacks_.cend()) {
                info = *it;
                callbacks_.erase(it);
                if (!helper->isConst()) {
                    --nonconst_callbacks_;
                }
        }
    }

    if (info) {
        info->subscription_queue_->clear();
        info->callback_queue_->removeByID((uint64_t)info.get());
    }
}

void Subscription::headerReceived(const PublisherLinkPtr& link, const ros::Header& h) {
    (void)h;
    std::lock_guard<std::mutex> lock{md5sum_mutex_};
    if (md5sum_ == "*") {
        md5sum_ = link->getMD5Sum();
    }
}

void Subscription::removePublisherLink(const PublisherLinkPtr& pub_link) {
    std::lock_guard<std::mutex> lock{publisher_links_mutex_};

    const auto it = std::find(publisher_links_.cbegin(), publisher_links_.cend(), pub_link);
    if (it != publisher_links_.cend()) {
        publisher_links_.erase(it);
    }

    if (pub_link->isLatched()) {
        latched_messages_.erase(pub_link);
    }
}

void Subscription::getPublishTypes(bool& ser, bool& nocopy, const std::type_info& ti) {
    std::lock_guard<std::mutex> lock{callbacks_mutex_};
    for (const auto& info: callbacks_) {
        if (info->helper_->getTypeInfo() == ti) {
            nocopy = true;
        } else {
            ser = true;
        }

        if (nocopy && ser) {
            return;
        }
    }
}

const std::string Subscription::md5sum() {
    std::lock_guard<std::mutex> lock{md5sum_mutex_};
    return md5sum_;
}

} // namespace roscan
